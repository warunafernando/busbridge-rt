"""Bring up I3C1 (port 1) once two wires exist, and say what is wrong if not.

Port 2 - I3C2 on AD_18/AD_19, Arduino A5/A4 - is exercised at every rate by the
acceptance suite. Port 1 is I3C1 on AON_16/AON_15, J3.12/J3.10 on the motor
connector, and has never been driven. It is not a second pin pair of one
controller: it is a different I3C instance, in a different IOMUXC block, with a
different pad-control layout. That is three independent things that can be
wrong, which is why this reports where it got to rather than pass or fail.

WHAT IS ALREADY TRUE, measured rather than assumed:

  - The firmware selects the port correctly. `I3C:PORT 1` muxes AON_16/AON_15
    and `I3C:PAD` reads back 0x0F at 0x443C00B4/B0, which in the AON block is
    SRE + DSE + PUE + PUS: fast slew, high driver, internal pull-up enabled.
    On port 2 the same constant lands in the main block, where PULL[3:2] = 11
    means No Pull - so port 1 gets an internal pull-up and port 2 does not. See
    the measurement beside pad_encode() in bb_board.c.
  - The pads reach the target board already, but to the WRONG PINS. RT bus 2
    (LPI2C2) shares AON_16/AON_15 and finds the EEPROM at 0x50 over them, so
    the J3 net is wired and working. It lands on the target's I2C2, PB10 and
    PB12, which have no I3C function at all.

That last point is the trap this script exists to stop someone falling into.
The target picks its EEPROM bus with default_bus(), which deliberately takes
the first bus that does NOT share pins with I3C - so the EEPROM is on I2C2 and
the I3C target is on PB6/PB7, a different net reached from RT's A5/A4. Reusing
the J3 wires therefore cannot work, however reasonable it looks.

WHAT IS NEEDED - two wires, which this script cannot fit.

  The bench as it stands, read off the boards:

    RT J3.12 / J3.10 (AON_16/15) -- NUCLEO PB10 (D6) / PB12 (CN10-16), the
                                    target's I2C2, EEPROM at 0x50
    RT A5 / A4       (AD_18/19)  -- NUCLEO PB6 (D15) / PB7 (D14), its I3C1
    RT D14 / D15     (bus 3)     -- not connected

  Move the NUCLEO end of the existing J3 pair; leave the RT end alone:

    SCL   J3.12 (AON_16)   from NUCLEO PB10 / D6        to PB6 / D15
    SDA   J3.10 (AON_15)   from NUCLEO PB12 / CN10-16   to PB7 / D14

  That joins the net A5/A4 already drive, pull-ups included. Both RT
  controllers then sit on one bus, which is fine - only one is claimed at a
  time and `I3C:FREE` releases the other's pads.

  Do NOT instead add a second wire to J3.12/J3.10. That pad already has a wire
  on it going to the target's I2C2, and a second one to PB6 would short the
  target's I2C2 net to its I3C net so that driving either drove both. One pad
  pair, one wire each: I3C1 and LPI2C2 are exclusive in the wiring exactly as
  they are in the mux.

  The cost while they are moved is RT's I2C bus 2, which loses the EEPROM - so
  the I2C parts of acceptance.py fail until the wires go back. `EE:I2C 3` moves
  the target's EEPROM to its I2C3 (PA8/D7, PC9/D10) if you would rather keep
  I2C alive on RT's unused bus 3.

  J45 selects GPIO_AON_15 between ETH0_RST_B and the header. It must be in the
  header position (2-3) - it already is, or RT bus 2 could not reach the target
  at all, which this script checks before blaming anything else.

  Pull-ups: the existing I3C net has whatever port 2 has been running on, and
  port 2 works with no internal pull-up, so external ones are present. Port 1
  adds its own internal pull-up on top. That is not a conflict, but if the bus
  misbehaves only on port 1, `I3C:PULL 0` takes the internal one out.

  Sniffing port 1 needs two more wires, from the same net to two free taps -
  d4 and d5 are unallocated and on FLEXIO1, the same peripheral as d8/d9.

Usage:  python i3c1_bringup.py [bus-hz] [root-hz]
"""
import sys

import bb
from acceptance import i3c_setup, PAT, clean

I3CR = "S A13+ D40+ D11+ D22+ D33+ D44+ D55+ P"
I3CW = "S A12+ D00+ D40- P"


def step(name, ok, detail=""):
    print("  %-46s %s  %s" % (name, "PASS" if ok else "FAIL", detail[:40]))
    sys.stdout.flush()
    return ok


def main():
    hz = int(sys.argv[1]) if len(sys.argv) > 1 else 1000000
    root = int(sys.argv[2]) if len(sys.argv) > 2 else 24000000

    r, n = bb.rt(), bb.nu()
    r.timeout(20000)
    n.timeout(20000)

    print("== I3C1 (port 1) bring-up: AON_16/AON_15, J3.12/J3.10 ==")

    # The net has to be wired before anything about I3C can be concluded. RT
    # bus 2 shares these exact pads, so it answers that question without
    # involving I3C at all - and if it fails, J45 is the first suspect.
    n.cmd("EE:I3C OFF")
    r.cmd("I3C:FREE")
    r.cmd("I2C:BUS 2")
    r.cmd("I2C:FREQ 400000")
    scan = r.cmd("I2C:SCAN")
    if not step("the J3 pads reach the target at all (I2C bus 2)",
                scan == "OK 50", scan):
        print("     J3.12/J3.10 are not connected, or J45 is in the")
        print("     ETH0_RST_B position. Nothing about I3C can be read from a")
        print("     bus that is not wired - fix this first.")
        return 1

    # Port selection and the pad register, which are firmware-side and were
    # already measured good. Re-checked because a bring-up that fails later is
    # much easier to read when the early steps are known to have held.
    n.cmd("EE:I3C ON")
    pp, daa = i3c_setup(hz, root, port=1)
    port = r.cmd("I3C:PORT?")
    step("port 1 is selected", "p1=AON_16/AON_15*" in port, port)
    pad = r.cmd("I3C:PAD")
    now = pad.split("scl_now=")[1].split()[0] if "scl_now=" in pad else "?"
    # 0x443C_xxxx is the AON IOMUXC block; the main one is 0x42A1_xxxx. Compared
    # without the "0x", because upper() would have capitalised that too and the
    # check could then never match - which is how this first reported aon=False
    # against a register that plainly began 0x443C.
    inaon = "443C" in pad.upper()
    step("the pads are the AON ones, configured",
         inaon and now.endswith("0F"), "scl_now=%s aon=%s" % (now, inaon))

    # The bus itself. A no-ack here, with everything above passing, means the
    # two wires are missing - the target's I3C is on PB6/PB7 and the J3 net
    # goes to its I2C2 pins.
    ok_daa = "targets=1" in daa
    if not step("ENTDAA finds the target", ok_daa, daa[:40]):
        print("     Everything firmware-side is right and nothing answered, so")
        print("     the wires are the thing that is missing:")
        print("       move the NUCLEO end of the J3 pair, RT end untouched:")
        print("         SCL  J3.12  PB10/D6       -> PB6/D15")
        print("         SDA  J3.10  PB12/CN10-16  -> PB7/D14")
        print("     Move, do not add: a second wire on J3.12 would short the")
        print("     target's I2C2 net to its I3C net. RT I2C bus 2 loses the")
        print("     EEPROM until they go back - see the notes at the top.")
        return 1

    n.cmd("EE:WR 0x40 " + PAT)
    w = r.cmd("I3C:W 0x09 00 40")
    step("private write", w.startswith("OK"), w)
    rd = r.cmd("I3C:R 0x09 6")
    step("private read returns the pattern", rd == "OK " + PAT, rd)
    ccc = r.cmd("I3C:CCC:R 0x09 0x8E 1")
    step("directed CCC (GETBCR)", ccc == "OK 2E", ccc)

    # And the capture engine against it, which needs its own two wires to taps
    # d4/d5. Reported rather than asserted: no tap wiring is not a port-1
    # fault, and saying so beats a failure that points at the wrong thing.
    for c in ("SNIFF:DMA 1", "SNIFF:DECODE 1", "SNIFF:MODE pinclock",
              "SNIFF:CH i3c d4 d5", "SNIFF:RATE %s" % pp,
              "SNIFF:CLEAR", "SNIFF:START"):
        r.cmd(c)
    r.cmd("I3C:W 0x09 00 40")
    r.cmd("I3C:R 0x09 6")
    r.cmd("I3C:W 0x09 00 40")
    r.cmd("SNIFF:STOP")
    ev = clean(r.cmd("SNIFF:READ? 120"))
    if ev.strip():
        step("the sniffer sees port 1 on taps d4/d5",
             ev.startswith(I3CW) and I3CR in ev, ev[:40])
    else:
        print("  %-46s ----  %s"
              % ("the sniffer sees nothing on taps d4/d5",
                 "tap wires not fitted"))
        print("     Optional: wire the same net to D4 and D5 to sniff port 1.")

    print("")
    print("  I3C1 is alive. Fold port 1 into acceptance.py's rate sweep next -")
    print("  i3c_setup() already takes a port argument.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
