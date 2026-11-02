"""Measure the CM33 clock against the capture core's, so `both` can be honest.

`SNIFF:SRC self` stamps events with bb_cycles(), SysTick-derived on the CM33 at
240 MHz. The capture core counts its own sample clock from the moment it
started. Interleaving the two streams needs the offset between those origins
and a bound on their relative drift, and until now neither had been measured -
so `SNIFF:SRC both` refused rather than guess, which was the right call and not
a permanent one.

HOW, AND WHY NOT THE OBVIOUS WAY

The obvious way is to ask the capture core for its tick over the link and note
bb_cycles() either side. That cannot work here. The round trip is tens of
microseconds, so its own jitter swamps the figure being measured; and the
core's tick is advanced by software draining shifters, so it lags real time by
however deep the queue happens to be. Both errors are far larger than a bit
time on a 12.5 MHz bus.

So the marker is physical. SYS:SYNC drives an edge on a pad wired to the taps
with bb_cycles() read either side of the store, and the capture engine
timestamps that same edge in its own ticks. One edge, two clocks. The
uncertainty is the width of the bracket and nothing else, and the bracket is
reported rather than averaged away.

WHAT COMES OUT

A straight-line fit of CM33 cycles against CM7 ticks over a span of seconds:

  slope      CM33 cycles per capture tick - the clocks' rate ratio, which says
             whether the two are driven from the same root or merely from
             roots that agree today
  intercept  where the capture's tick 0 sits in CM33 time: the offset
  residual   how far the worst point lies off that line. This is the honest
             error bar on any merge built on it, and the number that decides
             whether `both` can claim bit ordering, frame ordering, or neither

Usage:  python clock_offset.py [bursts] [per-burst] [gap-us]
"""
import struct
import sys
import time

import bb
from acceptance import i3c_setup

SDA_BIT = 0x02          # SNIFF:CH i3c d8 d9 - tap 0 is SCL, tap 1 is SDA
WRAP    = 1 << 32


def unwrap(seq):
    """A 32-bit counter read in order, made monotonic."""
    out, add, prev = [], 0, None
    for v in seq:
        if prev is not None and v < prev:
            add += WRAP
        out.append(v + add)
        prev = v
    return out


def drain_edges(r):
    """Every edge the capture holds, as (tick, lines).

    Asks until it is handed nothing, rather than until a declared count is
    reached: the firmware returns what it has to hand and the caller comes
    back, which is the contract SNIFF:RAW:READ? documents.
    """
    out = []
    while True:
        rep = r.cmd("SNIFF:RAW:READ? 79")
        if not rep.startswith("OK"):
            raise RuntimeError(rep)
        body = rep[2:].split()
        if not body:
            break
        raw = bytes(int(x, 16) for x in body)
        if len(raw) % 5:
            raise AssertionError("not a whole number of records: %d" % len(raw))
        for i in range(0, len(raw), 5):
            tick, lines = struct.unpack_from("<IB", raw, i)
            out.append((tick, lines))
    return out


def parse_sync(rep):
    """SYS:SYNC's brackets, in the order the edges were driven."""
    def pairs(field):
        s = rep.split(field)[1].split()[0]
        return [tuple(int(v) for v in p.split(",")) for p in s.split(";")]

    fall, rise = pairs("fall="), pairs("rise=")
    # Driven low then high, low then high: that is the order they appear in
    # the capture, so it is the order they are matched in.
    out = []
    for f, h in zip(fall, rise):
        out.append(("fall", f))
        out.append(("rise", h))
    return out


def fit(xs, ys):
    """Least squares y = a + b x, returned with the residuals."""
    count = len(xs)
    mx = sum(xs) / count
    my = sum(ys) / count
    sxx = sum((x - mx) ** 2 for x in xs)
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    b = sxy / sxx
    a = my - b * mx
    res = [y - (a + b * x) for x, y in zip(xs, ys)]
    return a, b, res


def main():
    bursts = int(sys.argv[1]) if len(sys.argv) > 1 else 5
    per    = int(sys.argv[2]) if len(sys.argv) > 2 else 12
    gap    = int(sys.argv[3]) if len(sys.argv) > 3 else 2000

    r, n = bb.rt(), bb.nu()
    r.timeout(30000)
    n.timeout(30000)

    print("== CM33 clock against the capture core's ==")
    i3c_setup(1000000, 24000000)
    # The pad has to be free for SYS:SYNC to drive it, and the bus has to be
    # idle: every edge in the window is taken to be one this script made.
    r.cmd("I3C:FREE")
    for c in ("SNIFF:SRC wire", "SNIFF:DECODE 0", "SNIFF:MODE oversample",
              "SNIFF:CH i3c d8 d9", "SNIFF:RATE 1000000", "SNIFF:CLEAR"):
        r.cmd(c)

    armed = r.cmd("SNIFF:RAW:START %d" % (bursts * per * 2 + 8))
    tick_hz = int(armed.split("tick_hz=")[1].split()[0])

    marks = []
    cycle_hz = 0
    t_wall0 = time.time()
    for k in range(bursts):
        rep = r.cmd("SYS:SYNC i3c2.sda %d %d" % (per, gap))
        if not rep.startswith("OK"):
            print("  SYS:SYNC refused: " + rep)
            return 1
        cycle_hz = int(rep.split("cycle_hz=")[1].split()[0])
        marks += parse_sync(rep)
        if k + 1 < bursts:
            time.sleep(1.0)     # a baseline long enough for drift to show
    span_wall = time.time() - t_wall0

    r.cmd("SNIFF:RAW:STOP")
    edges = drain_edges(r)

    # The first record is the initial line state, emitted one tick ahead of the
    # first real transition so an edge list can be decoded at all. It is a
    # state and not an event, so transitions are derived by comparing each
    # record against the one before rather than by trusting record count.
    trans = []
    prev = None
    for t, l in edges:
        cur = bool(l & SDA_BIT)
        if prev is not None and cur != prev:
            trans.append((t, "rise" if cur else "fall"))
        prev = cur

    print("  edges captured            : %d  (%d SDA transitions)"
          % (len(edges), len(trans)))
    print("  markers driven            : %d" % len(marks))
    print("  tick_hz=%d  cycle_hz=%d  wall span %.1f s"
          % (tick_hz, cycle_hz, span_wall))

    if len(trans) != len(marks):
        print("")
        print("  MISMATCH - not matching these up. A transition count that is")
        print("  not the number driven means something else moved the bus, or")
        print("  the capture lost edges. Either way a fit would be fiction.")
        print("  first 6 captured: %s" % (trans[:6],))
        print("  first 6 driven  : %s" % ([m[0] for m in marks[:6]],))
        return 1

    if any(t[1] != m[0] for t, m in zip(trans, marks)):
        print("  MISMATCH - directions disagree; the pairing is wrong.")
        return 1

    cm7  = unwrap([t for t, _ in trans])
    mids = [(a + b) / 2.0 for _, (a, b) in marks]
    cm33 = unwrap([int(m) for m in mids])
    brackets = [b - a for _, (a, b) in marks]

    a, b, res = fit(cm7, cm33)
    ns = 1e9 / cycle_hz                      # CM33 cycles -> ns
    worst = max(abs(x) for x in res)
    rms   = (sum(x * x for x in res) / len(res)) ** 0.5

    nominal = float(cycle_hz) / tick_hz
    span_s  = (cm7[-1] - cm7[0]) / float(tick_hz)
    # A straight line fitted over a span bounds drift directly: if the rate had
    # been changing, no single slope could hold the residual this small across
    # the whole span. So the drift bound is the residual spread divided by the
    # span - not a separate measurement, just the same fit read sideways.
    drift_ppb = (worst * 2.0 / cycle_hz) / span_s * 1e9 if span_s else 0.0

    print("")
    print("  bracket width             : %.0f ns worst, %.0f ns typical"
          % (max(brackets) * ns, sorted(brackets)[len(brackets) // 2] * ns))
    print("  slope                     : %.9f CM33 cycles per capture tick"
          % b)
    print("    nominal                 : %.9f  (%d / %d)"
          % (nominal, cycle_hz, tick_hz))
    print("    departure from nominal  : %+.1f ppb" % ((b / nominal - 1) * 1e9))
    print("  intercept (the offset)    : %.0f CM33 cycles at tick 0" % a)
    print("  residual                  : %.0f ns worst, %.0f ns rms  (%d points)"
          % (worst * ns, rms * ns, len(res)))
    print("  drift bound over %.1f s   : under %.0f ppb" % (span_s, drift_ppb))
    print("")
    print("  A merge built on this line is good to about %.0f ns." % (worst * ns))
    for name, period in (("12.5 MHz", 80.0), ("1 MHz", 1000.0)):
        print("    %-9s bus: one bit is %.0f ns - %s"
              % (name, period,
                 "inside the error bar" if worst * ns < period
                 else "SMALLER than the error bar"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
