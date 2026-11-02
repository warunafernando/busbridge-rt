"""How far behind the wire does the controller-side trace stamp its events?

clock_offset.py answers "where is the capture core's tick 0 in CM33 time", and
answers it to a few tens of nanoseconds. That is not the limit on `SNIFF:SRC
both`, and assuming it was would have been the easy mistake here.

The other limit is this one. A controller-side event is stamped when the CPU
gets round to it - when the address phase settles, when a byte is pushed into
the transmit FIFO - not when the corresponding bit reaches the wire. That lag
is real, it is not constant, and it is entirely independent of how well the two
clocks are aligned. Whatever it turns out to be is what `both` can claim, and
nothing better.

So this measures it directly. In one capture window:

  1. SYS:SYNC lays down markers, giving the CM33 -> CM7 line for that window.
  2. The controller-side trace is armed and real traffic is driven, so the
     same frames appear in both streams at once.
  3. Each frame's START is found in both: the trace's `S`, converted into
     capture ticks through the line, against the wire's own SDA fall while SCL
     is held high.

The difference is the lag. Its spread is the jitter, and the jitter is the
number that decides whether a merged stream can be trusted to order events
within a frame, between frames, or not at all.

Usage:  python trace_lag.py [rounds] [bus-hz]
"""
import sys

import bb
from acceptance import i3c_setup
from clock_offset import drain_edges, fit, parse_sync, unwrap

SCL_BIT = 0x01
SDA_BIT = 0x02


def sync_line(r, bursts=3, per=12, gap=2000):
    """Fit CM33 cycles against capture ticks using SYS:SYNC markers."""
    marks = []
    for _ in range(bursts):
        rep = r.cmd("SYS:SYNC i3c2.sda %d %d" % (per, gap))
        if not rep.startswith("OK"):
            raise RuntimeError(rep)
        marks += parse_sync(rep)
    return marks


def frame_starts(edges, idle_ticks):
    """Capture tick of the first edge of each burst of activity.

    Not "SDA falls while SCL is high", which is the textbook START and is the
    wrong tool here. Tried first and it over-counted by half: on a push-pull
    I3C bus SDA moves close enough to SCL's fall that a sampled capture sees
    plenty of SDA changes with SCL still reading high, and every one of them
    looks like a START to that rule.

    Bursts are unambiguous instead. Frames here are separated by tens of
    microseconds of a completely idle bus and nothing inside a frame is idle
    for even one bit time, so the gap threshold has an enormous margin either
    side and needs no decoding at all.
    """
    out = []
    prev_t = None
    for t, _ in edges:
        if prev_t is None or (t - prev_t) > idle_ticks:
            out.append(t)
        prev_t = t
    return out


def main():
    rounds = int(sys.argv[1]) if len(sys.argv) > 1 else 8
    hz     = int(sys.argv[2]) if len(sys.argv) > 2 else 1000000

    r, n = bb.rt(), bb.nu()
    r.timeout(30000)
    n.timeout(30000)

    print("== controller-side stamping lag, against the wire ==")
    i3c_setup(hz, 24000000)
    r.cmd("I3C:FREE")
    for c in ("SNIFF:SRC wire", "SNIFF:DECODE 0", "SNIFF:MODE oversample",
              "SNIFF:CH i3c d8 d9", "SNIFF:RATE %d" % hz, "SNIFF:CLEAR"):
        r.cmd(c)

    # One write/read pair costs about 200 edges at this rate, measured rather
    # than guessed - and the budget has to cover that, not a guess at it. An
    # under-sized budget is not a short capture: the declared maximum was hit
    # mid-traffic, the drain that followed timed out, and the board had to be
    # reset before it would open again.
    budget = 3 * 12 * 2 + rounds * 220 + 64
    armed = r.cmd("SNIFF:RAW:START %d" % budget)
    tick_hz = int(armed.split("tick_hz=")[1].split()[0])

    # Markers first, while the pad is still free and the bus is idle.
    marks = sync_line(r)

    # Then the trace alongside the capture already running. Setting the source
    # to self does not disturb the capture core - cmd_start() returns before
    # it would push a configuration - which is what lets both run at once.
    r.cmd("SNIFF:SRC self")
    r.cmd("SNIFF:START i3c")
    for _ in range(rounds):
        r.cmd("I3C:W 0x09 00 40")
        r.cmd("I3C:R 0x09 6")
    r.cmd("SNIFF:STOP")

    tr = r.cmd("SNIFF:TRACE? 250")
    if "t0=" not in tr:
        print("  SNIFF:TRACE? carries no t0; nothing can be placed. " + tr[:60])
        return 1
    t0       = int(tr.split("t0=")[1].split()[0])
    cycle_hz = int(tr.split("cycle_hz=")[1].split()[0])

    r.cmd("SNIFF:RAW:STOP")
    st = r.cmd("SNIFF:RAW:STAT?")
    if "full=0" not in st:
        print("  the edge ring filled - the window is not the whole run: " + st)
        return 1
    edges = drain_edges(r)

    # The line, from the markers in this same window.
    sda_trans = []
    prev = None
    for t, l in edges:
        cur = bool(l & SDA_BIT)
        if prev is not None and cur != prev:
            sda_trans.append((t, "rise" if cur else "fall"))
        prev = cur
    marker_trans = sda_trans[:len(marks)]
    if len(marker_trans) != len(marks):
        print("  only %d of %d markers landed in the capture"
              % (len(marker_trans), len(marks)))
        return 1

    cm7_m  = unwrap([t for t, _ in marker_trans])
    cm33_m = unwrap([int((a + b) / 2) for _, (a, b) in marks])
    a, b, res = fit(cm7_m, cm33_m)
    ns = 1e9 / cycle_hz
    print("  line from %d markers, residual %.0f ns worst"
          % (len(marks), max(abs(x) for x in res) * ns))

    # Wire frames, after the marker block.
    #
    # 100 us, chosen from the gap distribution rather than from the bit rate.
    # Frames arrive one USB command apart, which is about 300 us here; the
    # largest gap seen INSIDE a frame is 36 us, where i3c_claim_pads() re-muxes
    # the pad at the start of a transfer. Twenty bit times - the first,
    # rate-derived guess - is 20 us, which split that first frame in two and
    # produced nine frames where eight were driven.
    idle = int(100.0 * tick_hz / 1e6)
    traffic_edges = [(t, l) for (t, l) in edges if t > cm7_m[-1] + idle]
    wire_starts = frame_starts(traffic_edges, idle)

    # Trace STARTs, as absolute CM33 cycles, mapped onto capture ticks.
    toks = [m for m in tr.split() if "=" in m and m.split("=", 1)[0].isdigit()]
    trace_starts = [int(m.split("=")[0]) for m in toks
                    if m.split("=", 1)[1] == "S"]
    trace_ticks = [((t0 + d) - a) / b for d in trace_starts]

    print("  frames on the wire        : %d" % len(wire_starts))
    print("  frames in the trace       : %d" % len(trace_starts))
    if not wire_starts or not trace_starts:
        print("  nothing to compare.")
        return 1
    if len(wire_starts) != len(trace_starts):
        print("")
        print("  Counts differ, so the pairing would be a guess. Not pairing.")
        return 1

    lag_ns = [(tt - wt) * (1e9 / tick_hz)
              for tt, wt in zip(trace_ticks, wire_starts)]
    lo, hi = min(lag_ns), max(lag_ns)
    mean = sum(lag_ns) / len(lag_ns)

    print("")
    print("  lag, trace stamp after the wire's START:")
    print("    mean   %8.0f ns" % mean)
    print("    range  %8.0f ns to %.0f ns" % (lo, hi))
    print("    spread %8.0f ns   <- this is what bounds a merge" % (hi - lo))
    print("")
    bit_ns = 1e9 / hz
    print("  One bit at %.2f MHz is %.0f ns." % (hz / 1e6, bit_ns))
    if (hi - lo) < bit_ns:
        print("  The spread is under a bit time: events can be ordered within")
        print("  a frame.")
    else:
        print("  The spread is wider than a bit time, so a merged stream can")
        print("  order frames against each other but NOT events inside one.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
