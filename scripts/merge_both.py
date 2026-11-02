"""Merge a `SNIFF:SRC both` capture into one ordered stream, and say what it is worth.

Two engines run over one window. The capture core samples the taps and knows
only its own sample ticks; the controller records its own transfers and stamps
them with bb_cycles() on the CM33. Nothing on the board relates the two, so the
merge is done here, from a line measured inside the same window by SYS:SYNC
markers - see clock_offset.py for why a marker and not a message.

The output is deliberately not a single undifferentiated list of events. Every
line says which engine produced it, because that is the entire reason for
running both: they agree about every frame that works, and the moments they
disagree are the ones worth having. A merged view that hid which side a line
came from would destroy exactly the information it was built to show.

WHAT THIS MERGE CAN CLAIM, measured rather than assumed

  clock alignment     about 100 ns over a 30 s window (clock_offset.py); the
                      rate ratio is nominal to 0.3 ppb, so the two clocks come
                      off one reference and there is nothing to track
  controller stamping about 1 us against the wire, mean slightly NEGATIVE -
                      the controller writes MCTRL a little before the bus
                      moves, which is the right sign (trace_lag.py)

So frames can be ordered against each other with large margin: frames here are
hundreds of microseconds apart and the error is about one. Events INSIDE a
frame cannot be, at 12.5 MHz where a bit is 80 ns, and are marginal at 1 MHz.
This prints frame-level ordering and refuses to imply more.

Usage:  python merge_both.py [rounds] [bus-hz]
"""
import sys

import bb
from acceptance import i3c_setup
from clock_offset import drain_edges, fit, parse_sync, unwrap
from trace_lag import frame_starts, sync_line

SDA_BIT = 0x02


def main():
    rounds = int(sys.argv[1]) if len(sys.argv) > 1 else 3
    hz     = int(sys.argv[2]) if len(sys.argv) > 2 else 1000000

    r, n = bb.rt(), bb.nu()
    r.timeout(30000)
    n.timeout(30000)

    print("== SNIFF:SRC both, merged ==")
    i3c_setup(hz, 24000000)
    r.cmd("I3C:FREE")
    for c in ("SNIFF:SRC wire", "SNIFF:DECODE 0", "SNIFF:MODE oversample",
              "SNIFF:CH i3c d8 d9", "SNIFF:RATE %d" % hz, "SNIFF:CLEAR"):
        r.cmd(c)

    # `both` before arming, because SNIFF:RAW:START is what arms both engines
    # and it has to be the last thing that touches the capture core. Starting
    # the trace afterwards with SNIFF:START would restart the core and discard
    # every marker laid down since - which produced a fit that was 30 ms out
    # and looked entirely ordinary.
    r.cmd("SNIFF:SRC both")
    armed   = r.cmd("SNIFF:RAW:START %d" % (3 * 12 * 2 + rounds * 220 + 64))
    tick_hz = int(armed.split("tick_hz=")[1].split()[0])

    # Markers while the bus is idle and the pad is free, then the traffic.
    marks = sync_line(r)
    for _ in range(rounds):
        r.cmd("I3C:W 0x09 00 40")
        r.cmd("I3C:R 0x09 6")
    r.cmd("SNIFF:STOP")

    tr = r.cmd("SNIFF:TRACE? 250")
    t0 = int(tr.split("t0=")[1].split()[0])
    cycle_hz = int(tr.split("cycle_hz=")[1].split()[0])
    r.cmd("SNIFF:RAW:STOP")

    st = r.cmd("SNIFF:RAW:STAT?")
    if "full=0" not in st:
        print("  the edge ring filled; the window is not the whole run")
        return 1
    edges = drain_edges(r)

    # The line, from this window's own markers.
    sda, prev = [], None
    for t, l in edges:
        cur = bool(l & SDA_BIT)
        if prev is not None and cur != prev:
            sda.append((t, "rise" if cur else "fall"))
        prev = cur
    if len(sda) < len(marks):
        print("  only %d of %d markers landed" % (len(sda), len(marks)))
        return 1
    cm7_m  = unwrap([t for t, _ in sda[:len(marks)]])
    cm33_m = unwrap([int((a + b) / 2) for _, (a, b) in marks])
    a, b, res = fit(cm7_m, cm33_m)
    ns_per_cycle = 1e9 / cycle_hz
    err_ns = max(abs(x) for x in res) * ns_per_cycle

    # Wire frames and controller frames, both as capture ticks.
    idle = int(100.0 * tick_hz / 1e6)
    wire_starts = frame_starts(
        [(t, l) for (t, l) in edges if t > cm7_m[-1] + idle], idle)

    toks = [m for m in tr.split() if "=" in m and m.split("=", 1)[0].isdigit()]
    rows = []
    for m in toks:
        d, tok = m.split("=", 1)
        rows.append((((t0 + int(d)) - a) / b, tok))

    ctrl_starts = [t for t, tok in rows if tok == "S"]

    print("  alignment good to %.0f ns   (%d markers)" % (err_ns, len(marks)))
    print("  frames: %d on the wire, %d in the controller"
          % (len(wire_starts), len(ctrl_starts)))
    if len(wire_starts) != len(ctrl_starts):
        print("")
        print("  THE TWO DISAGREE ABOUT HOW MANY FRAMES HAPPENED.")
        print("  That is the finding, not an error in the merge - it is the")
        print("  case this source exists to catch. Not pairing them up.")
        return 1

    us = 1e6 / tick_hz
    base = wire_starts[0]
    print("")
    print("  %-12s %-12s %-10s %s" % ("wire (us)", "ctrl (us)", "delta", "frame"))
    for i, (w, c) in enumerate(zip(wire_starts, ctrl_starts)):
        print("  %-12.1f %-12.1f %-+10.2f %s"
              % ((w - base) * us, (c - base) * us, (c - w) * us,
                 "write" if i % 2 == 0 else "read"))

    worst = max(abs(c - w) * us * 1000.0 for w, c in zip(wire_starts,
                                                         ctrl_starts))
    bit_ns = 1e9 / hz
    print("")
    print("  worst frame disagreement : %.0f ns" % worst)
    print("  one bit at %.2f MHz      : %.0f ns" % (hz / 1e6, bit_ns))
    print("  frames are ~%.0f us apart, so ordering them is safe by a wide"
          % ((wire_starts[1] - wire_starts[0]) * us))
    print("  margin. Ordering events INSIDE a frame is not: %s"
          % ("marginal here" if worst < bit_ns else "the error exceeds a bit"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
