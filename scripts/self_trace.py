"""Exercise SNIFF:SRC self, and hold it against the wire.

A controller-side trace that nobody checked against a measurement is a story
the controller tells about itself. The whole value of the source is that it
can DISAGREE with the bus, so the first thing to establish is that it agrees
when it should - on a bench where the wire capture is already known exact at
every rate to 12.5 MHz.

Three things are checked, and they are deliberately different questions:

  1. The trace alone produces the frames the commands asked for. This needs no
     taps and no capture core, which is the point of the source.
  2. The same traffic captured on the wire produces the same tokens. Equality
     here is the claim worth making; the comparison is token for token rather
     than frame counts, because a count can agree while the bytes do not.
  3. The things the trace CANNOT see show up as holes rather than as silence.
     A CCC goes through the SDK's blocking transfer, so it is marked `!` and
     the comparison has to account for it. A trace that quietly omitted them
     would pass a naive comparison and mislead anyone merging the two streams.

Usage:  python self_trace.py [bus-hz] [root-hz]
"""
import sys

import bb
from acceptance import i3c_setup, PAT, clean

WRITE = "S A12+ D00+ D40- P"
READ  = "S A13+ D40+ D11+ D22+ D33+ D44+ D55+ P"

fails = []


def step(name, ok, detail=""):
    print("  %-52s %s  %s" % (name, "PASS" if ok else "FAIL", detail[:46]))
    sys.stdout.flush()
    if not ok:
        fails.append(name)
    return ok


def traffic(r):
    """One write and one read - the pair every other test here uses."""
    r.cmd("I3C:W 0x09 00 40")
    r.cmd("I3C:R 0x09 6")


def self_capture(r):
    r.cmd("SNIFF:SRC self")
    r.cmd("SNIFF:START i3c")
    traffic(r)
    r.cmd("SNIFF:STOP")
    return r.cmd("SNIFF:READ? 120")


def wire_oversampled(r):
    r.cmd("SNIFF:SRC wire")
    for c in ("SNIFF:DECODE 1", "SNIFF:MODE oversample",
              "SNIFF:CH i3c d8 d9", "SNIFF:RATE 1000000",
              "SNIFF:CLEAR", "SNIFF:START"):
        r.cmd(c)
    traffic(r)
    r.cmd("SNIFF:STOP")
    return r.cmd("SNIFF:READ? 120")


def wire_pinclocked(r, pp):
    r.cmd("SNIFF:SRC wire")
    for c in ("SNIFF:DMA 1", "SNIFF:DECODE 1", "SNIFF:MODE pinclock",
              "SNIFF:CH i3c d8 d9", "SNIFF:RATE %s" % pp,
              "SNIFF:CLEAR", "SNIFF:START"):
        r.cmd(c)
    traffic(r)
    r.cmd("SNIFF:STOP")
    return r.cmd("SNIFF:READ? 120")


def main():
    hz   = int(sys.argv[1]) if len(sys.argv) > 1 else 1000000
    root = int(sys.argv[2]) if len(sys.argv) > 2 else 24000000

    r, n = bb.rt(), bb.nu()
    r.timeout(20000)
    n.timeout(20000)

    print("== SNIFF:SRC self - controller-side trace ==")
    pp, daa = i3c_setup(hz, root)
    if not step("the target is on the bus (ENTDAA)", "targets=1" in daa, daa):
        return 1

    # ---- 1. the trace on its own ------------------------------------------
    raw = self_capture(r)
    step("the reply says where its records came from",
         "observed=controller" in raw, raw[:46])
    step("and that its ninth bits are computed, not seen",
         "tbits=computed" in raw and "addrack=observed" in raw, raw[:46])

    ev = clean(raw)
    step("a private write traces as the frame it drove",
         ev.startswith(WRITE), ev[:46])
    step("a private read traces as the frame it drove",
         READ in ev, ev[len(WRITE):][:46])
    step("nothing was lost from the ring", " lost=0 " in raw + " ",
         raw.split("lost=")[1][:8] if "lost=" in raw else "?")

    # Draining is destructive, and a second read must say so rather than
    # replay what the first one already handed over.
    again = clean(r.cmd("SNIFF:READ? 120"))
    step("a drained ring reads empty, not stale", again == "", again[:46])

    # ---- 2. against the wire ----------------------------------------------
    #
    # Oversampled, not pin-clocked, and the choice is the whole test. A
    # pin-clocked capture truncates the last frame of a window - its shifter
    # emits nothing until its word fills, which is open item 7 in STATUS.md -
    # so it ends this traffic at `D44+` and the comparison would fail for a
    # defect that has nothing to do with the trace. At 1 MHz oversampling has
    # four samples a bit and loses nothing, so equality here is a real claim.
    selfev = clean(self_capture(r))
    wire   = clean(wire_oversampled(r))
    step("the wire saw the same write", wire.startswith(WRITE), wire[:46])
    step("trace and wire agree token for token (oversampled)",
         selfev == wire, "self=%d wire=%d tokens" % (len(selfev.split()),
                                                     len(wire.split())))

    # And against the pin-clocked capture, where they are expected to differ -
    # by exactly the known tail. Checking the shape of a known disagreement is
    # worth as much as checking an agreement: if this ever stops being a clean
    # prefix, something new is wrong.
    pin = clean(wire_pinclocked(r, pp))
    step("pin-clocked wire is a prefix of the trace, not a conflict",
         selfev.startswith(pin) and pin != selfev,
         "trace has %d more" % (len(selfev.split()) - len(pin.split())))

    # ---- 3. what it cannot see is a hole, not a silence --------------------
    r.cmd("SNIFF:SRC self")
    r.cmd("SNIFF:START i3c")
    r.cmd("I3C:CCC:R 0x09 0x8E 1")      # GETBCR - goes through the SDK
    r.cmd("SNIFF:STOP")
    ccc = clean(r.cmd("SNIFF:READ? 120"))
    # Two, not one: a directed CCC read is two transfers - the command frame
    # and the read that follows it - and each is a frame this side cannot
    # describe. One hole per untraceable frame is the contract; collapsing
    # them would understate how much is missing.
    step("a CCC the trace cannot describe is marked, not dropped",
         ccc.split() == ["!", "!"], repr(ccc)[:46])

    # ---- 4. the case that matters most: an address nobody answers ---------
    #
    # This is where a controller-side trace can be confidently wrong, and was.
    # MCTRLDONE comes up for the address phase and MERRWARN.NACK is set after
    # it, so opening the frame at MCTRLDONE recorded `S AAA+` for a transfer
    # that reported no-ack. The frame is now held until a NACK appears or the
    # module takes a byte, whichever comes first.
    r.cmd("SNIFF:SRC self")
    r.cmd("SNIFF:START i3c")
    r.cmd("I3C:W 0x55 AA")
    r.cmd("I3C:R 0x55 2")
    r.cmd("SNIFF:STOP")
    nack = clean(r.cmd("SNIFF:READ? 60"))
    step("an address nobody answers traces as NACKed",
         nack.split() == ["S", "AAA-", "S", "AAB-"], repr(nack)[:46])

    # ---- timestamps --------------------------------------------------------
    r.cmd("SNIFF:START i3c")
    traffic(r)
    r.cmd("SNIFF:STOP")
    tr = r.cmd("SNIFF:TRACE? 40")
    step("SNIFF:TRACE? carries its counter frequency",
         "cycle_hz=" in tr, tr[:46])
    # Peeking must not consume: the tokens have to still be there afterwards.
    after = clean(r.cmd("SNIFF:READ? 120"))
    step("SNIFF:TRACE? peeks rather than drains",
         after.startswith(WRITE), after[:46])

    # Each mark is "<counter delta>=<token>", with no space in between - a
    # space there would split one field into two and the time would parse as
    # an event. Insisting the token half is non-empty is the point: the first
    # version of this check read empty strings and passed.
    marks = [t for t in tr.split()
             if "=" in t and t.split("=", 1)[0].isdigit()]
    toks  = [m.split("=", 1)[1] for m in marks]
    if not step("SNIFF:TRACE? pairs a time with a token", len(marks) == 14,
                "%d mark(s)" % len(marks)):
        print("     " + tr[:200])
    step("no mark has an empty token half", toks and all(toks),
         " ".join(marks[:4]))

    if marks:
        first = int(marks[0].split("=")[0])
        last  = int(marks[-1].split("=")[0])
        hzc   = int(tr.split("cycle_hz=")[1].split()[0])
        step("event times advance across a transfer", last > first,
             "%.1f us span" % ((last - first) * 1e6 / hzc))
        # Every D is computed on this side; no A is. Checked both ways round,
        # because "all data bytes are marked" is also satisfied by marking
        # everything, which would make the mark meaningless.
        step("every data byte is marked derived",
             all(t.endswith("~") for t in toks if t.startswith("D")),
             " ".join(t for t in toks if t.startswith("D"))[:40])
        step("no address byte is",
             not any(t.endswith("~") for t in toks if t.startswith("A")),
             " ".join(t for t in toks if t.startswith("A"))[:40])

    # `both` used to refuse for want of a measured offset between the two
    # clocks. clock_offset.py measured it; this checks the engines both run.
    step("SNIFF:SRC both is accepted", r.cmd("SNIFF:SRC both").startswith("OK"))
    for c in ("SNIFF:DECODE 1", "SNIFF:MODE oversample", "SNIFF:CH i3c d8 d9",
              "SNIFF:RATE 1000000", "SNIFF:CLEAR", "SNIFF:START i3c"):
        r.cmd(c)
    traffic(r)
    r.cmd("SNIFF:STOP")
    w = clean(r.cmd("SNIFF:READ? 120"))
    t = r.cmd("SNIFF:TRACE? 60")
    c = " ".join(m.split("=", 1)[1].rstrip("~") for m in t.split()
                 if "=" in m and m.split("=", 1)[0].isdigit())
    step("both: wire and controller describe the same frames", w == c,
         "%d vs %d" % (len(w.split()), len(c.split())))

    r.cmd("SNIFF:SRC wire")
    print("")
    print("  %d check(s) failed" % len(fails))
    for f in fails:
        print("    - " + f)
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
