"""A .cap for bus rates too fast to oversample, with its timing marked invented.

Oversampled capture - the mode that produces an edge list - needs four or more
samples per bit, and the intake tops out at 10 MHz on two taps. That is 1.7
samples per bit on a 6 MHz I3C bus and 0.8 on a 12.5 MHz one, so the edge list
those rates produce is not wrong so much as absent: addresses survive because
they are clocked slowest, payload does not. See cap_rates.py for the table.

Pin-clocked capture has no such limit. The bus clocks the shifter, one sample
per bit, and the decode is byte-exact at every rate to 12.5 MHz. What it does
not produce is an edge list: a sample is a bit, not a transition, and there is
no record of when anything moved. It cannot feed a waveform analyser.

So this reconstructs one. Every bit value, every byte, every START and STOP is
what the bus carried, decoded by the capture firmware and verified byte-exact.
Every *time* is manufactured here, by laying those values onto a uniform bit
period. The waveform is a drawing of the traffic, true in content and fictional
in timing, which is a genuinely useful thing to be as long as nobody mistakes
it for a recording.

That is the whole risk, and it is not small: a reconstructed waveform is
indistinguishable from a recorded one by inspection, and a measurement taken
off it - a clock rate, a duty cycle, a setup time, any jitter at all - returns
the bit period chosen below, confidently and with no sign that it is an echo of
an argument rather than a property of the bus. So the file says what it is, in
the session's `timing` field rather than in prose: the analyser prints a warning
whenever it opens one, prints it again on stdout above any waveform it draws,
and sets `timing_synthetic` on every measurement taken from it.

What is faithful here:
  - byte values, their order, and their ninth bits
  - START, repeated START and STOP, and where they fall between bytes
  - which transfers were reads and which were writes
  - a frame left unterminated by the capture staying unterminated

What is invented:
  - every timestamp
  - the bit period, and therefore every rate derived from it
  - the gap between transactions
  - clock duty cycle, setup and hold, and every edge's exact position

Usage:
    python pinclock_cap.py out.cap [bus-hz] [root-hz]
"""
import subprocess
import sys

import bb
import capture as cap
from acceptance import i3c_setup, PAT, clean

# bit0 is SCL and bit1 is SDA, which is the capture container's own order and
# the one the analyser reads channel names in.
SCL, SDA = 1, 2


class Wire:
    """Lay bit values onto a uniform clock, half a bit period at a time.

    The edge shapes are the simulator's (tools/simulator/src/protocols.rs) on
    purpose. A reconstruction that drew START or STOP differently from the
    generator the decoders are tested against would be testing the decoder's
    tolerance rather than reproducing the traffic.
    """

    def __init__(self):
        # Counted in ticks of half a bit, not in picoseconds. write_cap takes
        # ticks and scales them by the session's tick rate, so handing it
        # picoseconds scales them a second time - which it did, and produced a
        # capture whose transactions were a thousand times too far apart while
        # decoding perfectly, because every time was wrong by the same factor.
        self.t = 0
        self.scl = 1
        self.sda = 1
        # Contract 3: the first entry is the initial state, not a transition.
        # The bus idles with both lines released high.
        self.points = [(0, SCL | SDA)]

    def drive(self, scl, sda):
        self.t += 1
        if (scl, sda) == (self.scl, self.sda):
            return
        self.scl, self.sda = scl, sda
        self.points.append((self.t, (SCL if scl else 0) | (SDA if sda else 0)))

    def idle(self, bits=4):
        for _ in range(bits * 2):
            self.drive(1, 1)

    def start(self):
        self.drive(1, 1)
        self.drive(1, 0)
        self.drive(0, 0)

    def repeated_start(self):
        self.drive(0, 1)
        self.drive(1, 1)
        self.drive(1, 0)
        self.drive(0, 0)

    def bit(self, v):
        self.drive(0, v)
        self.drive(1, v)
        self.drive(0, v)

    def byte(self, value, ninth):
        for i in range(7, -1, -1):
            self.bit((value >> i) & 1)
        self.bit(ninth)

    def stop(self):
        self.drive(0, 0)
        self.drive(1, 0)
        self.drive(1, 1)


def read_events(r):
    """Drain the decoded events, as tokens."""
    out = []
    for _ in range(40):
        reply = r.cmd("SNIFF:READ? 256")
        more = "more=1" in reply
        out += clean(reply).split()
        if not more:
            break
    return out


def synthesise(events):
    """Turn decoded events back into a waveform.

    The ninth bit is recovered rather than assumed, and the two protocols put
    opposite meanings on the same flag. The firmware emits an address with its
    flag set when the ninth bit was LOW - that is an acknowledge - and emits a
    data or command byte with its flag set when the ninth bit was HIGH, because
    on I3C that bit is a T-bit driven by the transmitter and not an
    acknowledge at all. Reading one rule for both inverts every data byte's
    ninth bit, which a decoder then reports as a parity failure on traffic that
    was correct.
    """
    w = Wire()
    open_frame = False
    for tok in events:
        if tok == "S":
            if open_frame:
                w.stop()
            w.idle()
            w.start()
            open_frame = True
        elif tok == "Sr":
            w.repeated_start()
            open_frame = True
        elif tok == "P":
            w.stop()
            open_frame = False
        elif tok == "!":
            # A dropped-event marker. There is no waveform for "something was
            # lost", and drawing one would be the only outright lie available
            # here, so the hole stays a hole.
            continue
        elif len(tok) == 4 and tok[0] in "ADC" and tok[3] in "+-":
            value = int(tok[1:3], 16)
            flag = tok[3] == "+"
            ninth = (0 if flag else 1) if tok[0] == "A" else (1 if flag else 0)
            w.byte(value, ninth)
    # A frame the capture never saw closed stays open. Pin-clocked capture
    # strands a frame's last bits until the next frame clocks them, so the final
    # STOP of a window genuinely does not arrive - inventing one here would turn
    # a known limitation into a clean-looking capture.
    w.idle()
    return w.points


def capture_pinclocked(r, n, bus_hz, root_hz, runs=3):
    pp, daa = i3c_setup(bus_hz, root_hz)
    if "targets=1" not in daa:
        raise SystemExit("ENTDAA found no target: %s" % daa)
    n.cmd("EE:WR 0x40 " + PAT)
    for c in ("SNIFF:DMA 1", "SNIFF:DECODE 1", "SNIFF:MODE pinclock",
              "SNIFF:CH i3c d8 d9", "SNIFF:RATE %s" % pp,
              "SNIFF:CLEAR", "SNIFF:START"):
        r.cmd(c)
    for _ in range(runs):
        r.cmd("I3C:W 0x09 00 40")
        r.cmd("I3C:R 0x09 6")
    r.cmd("SNIFF:STOP")
    return int(pp), read_events(r)


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else "pinclock.cap"
    bus = int(sys.argv[2]) if len(sys.argv) > 2 else 12500000
    root = int(sys.argv[3]) if len(sys.argv) > 3 else 25000000

    r, n = bb.rt(), bb.nu()
    r.timeout(20000)
    n.timeout(20000)
    pp, events = capture_pinclocked(r, n, bus, root)
    bit_ps = 1000000000000 // pp
    points = synthesise(events)
    dev, fw = cap.ident(r)

    # tick_hz describes the timebase the file's times are on. Here that is the
    # reconstruction's own grid - one tick per half bit - and not any clock the
    # hardware counted, because saying otherwise would put a measured-looking
    # provenance on invented numbers.
    tick_hz = 2 * pp
    cap.write_cap(
        out, points, tick_hz, ["SCL", "SDA"], dev, fw,
        source="busbridge:USB pin-clocked, waveform reconstructed",
        timing={"Synthetic": {
            "why": ("The bus ran at %d Hz, which oversampled capture cannot "
                    "resolve - the intake allows 10 MHz on two taps, so %.1f "
                    "samples per bit against the four a transition needs. "
                    "Pin-clocked capture recovers the bits exactly and records "
                    "no transition times at all."
                    % (pp, 10000000.0 / pp)),
            "from": ("bytes and framing decoded on the capture core from a "
                     "pin-clocked capture at %d Hz, laid on a uniform bit "
                     "period" % pp),
            "bit_ps": bit_ps,
        }})
    print("%s: %d events -> %d edges, bit period %d ps (%d Hz bus)"
          % (out, len(events), len(points), bit_ps, pp))
    return out, events


def transfers(events):
    """The transfers the firmware decoded, as (address, direction, bytes).

    What the reconstruction is supposed to preserve, stated independently of
    the reconstruction so it can be compared against it.
    """
    out, cur = [], None
    for tok in events:
        if tok in ("S", "Sr"):
            if cur:
                out.append(cur)
            cur = None
        elif tok == "P":
            if cur:
                out.append(cur)
            cur = None
        elif len(tok) == 4 and tok[0] in "ADC" and tok[3] in "+-":
            value = int(tok[1:3], 16)
            if tok[0] == "A":
                if cur:
                    out.append(cur)
                cur = (value >> 1, "R" if value & 1 else "W", [])
            elif cur:
                cur[2].append(value)
    if cur:
        out.append(cur)
    return out


def check_round_trip(path, events, sniffer):
    """The analyser's decode of the reconstruction, against the firmware's.

    The only check available, because the values are all that survived the trip
    - but it is the one that matters. A reconstruction that drew the traffic
    wrongly would come back as different bytes, and one that drew it correctly
    cannot come back as anything else.

    A transfer the capture left unterminated is compared on the bytes it did
    carry: the last frame of a pin-clocked window is cut short by the mode
    itself, and demanding it match in full would fail the reconstruction for
    something that happened before it.
    """
    res = subprocess.run([sniffer, "decode", path, "--protocol", "i3c",
                          "--json"], capture_output=True, text=True)
    if res.returncode != 0:
        return False, "analyser failed: %s" % res.stderr.strip()[:200]
    import json
    got = []
    for t in json.loads(res.stdout).get("transactions", []):
        fields = {f["name"]: f for f in t.get("fields", [])}
        if "address" not in fields:
            continue
        data = [f["display"] for f in t.get("fields", [])
                if f["name"].startswith("data")]
        got.append((int(fields["address"]["value"]),
                    fields["direction"]["display"],
                    [int(d, 16) for d in data],
                    t.get("complete", True)))

    want = transfers(events)
    if len(got) != len(want):
        return False, ("%d transfers decoded, firmware saw %d"
                       % (len(got), len(want)))
    for i, ((ga, gd, gb, done), (wa, wd, wb)) in enumerate(zip(got, want)):
        if (ga, gd) != (wa, wd):
            return False, ("transfer %d: decoded 0x%02X %s, firmware 0x%02X %s"
                           % (i, ga, gd, wa, wd))
        if done and gb != wb:
            return False, ("transfer %d: decoded %s, firmware %s"
                           % (i, gb, wb))
        if not done and gb != wb[:len(gb)]:
            return False, ("transfer %d truncated and diverged: %s vs %s"
                           % (i, gb, wb))
    return True, "%d transfers, byte-exact against the firmware's own decode" \
                 % len(got)


if __name__ == "__main__":
    path, events = main()
    sn = r"E:\STMicro\sniffer\target\release\sniffer.exe"

    res = subprocess.run([sn, "decode", path, "--protocol", "i3c"],
                         capture_output=True, text=True)
    print("")
    for line in res.stderr.splitlines():
        if line.strip():
            print("  " + line.strip())
    print(res.stdout)

    ok, why = check_round_trip(path, events, sn)
    print("  round trip: %s - %s" % ("PASS" if ok else "FAIL", why))
    sys.exit(0 if ok else 1)
