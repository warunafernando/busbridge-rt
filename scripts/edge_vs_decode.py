"""Decode the edge list with the firmware's own rules, and compare.

The firmware decodes a capture byte-exactly. The analyser, handed the edge
list from the same capture, does not. Both claims are established, and between
them sits a question nobody has answered: is the edge list a faithful record of
what the decoder saw, or is it already short by the time it reaches the host?

In oversampled mode that question has an exact answer, because cap_consume()
feeds both paths from one place:

    if (st != last) {
        if (!cap.decode_off) { dec_sample(st, tick); }
        push_sample(st, tick);
    }

Same transitions, same ticks, same order, no filtering between them. So a
decoder that implements dec_sample() and dec_i3c.c faithfully, run over the
drained edge list, MUST reproduce SNIFF:READ? exactly. If it does, the edges
are whole and the fault is downstream in the analyser. If it does not, the
divergence names the first edge that went missing, and the hunt is upstream.

This is deliberately a transcription of the C and not an improvement on it.
Where the firmware has a quirk - the `settled` guard, the held START, the
eight-bit DAA framing - the quirk is copied. A better decoder here would prove
nothing: the point is to agree with that one or to say exactly where it stops
agreeing.

What it found, the first time it ran: the write matched and the read did not,
and the drain handed back 33 fewer edges than the firmware said it had. Those
were one fact. SNIFF:RAW:READ? 96 formats 1442 characters of hex and the host
library keeps 1199 of them, so seventeen records per call were cut off in
transit - and because the command had already advanced its cursor past them,
they could not be asked for again. The holes landed inside the read, so the
capture stayed well-formed and decoded to plausible, wrong bytes. Days went
into the protocol analyser, which was decoding exactly what it had been given.

Both ends are bounded now, and this is the check that says so.
"""
import struct
import sys

import bb
from acceptance import i3c_setup, PAT

DEC_CLK_BIT, DEC_DAT_BIT = 0, 1

ST_IDLE, ST_ADDR, ST_CCC, ST_DATA, ST_DAA = range(5)
I3C_BROADCAST, I3C_CCC_ENTDAA, I3C_CCC_IS_DIRECT = 0x7E, 0x07, 0x80
I3C_DAA_PAYLOAD = 8


class DecI3C:
    """dec_i3c.c, transcribed."""

    def __init__(self, emit):
        self.emit_fn = emit
        self.state = ST_IDLE
        self.in_frame = False
        self.bit_count = 0
        self.byte = 0
        self.read = False
        self.entdaa = False
        self.daa_left = 0
        self.skip = 0
        self.at_turnaround = False
        self.bytes_done = 0
        self.start_held = False
        self.start_held_repeated = False

    def emit(self, kind, value=0, flag=False):
        self.emit_fn(kind, value, flag)

    def start(self, repeated):
        if (repeated and self.at_turnaround and self.bit_count == 0
                and self.state in (ST_CCC, ST_DATA)):
            return
        self.start_held = True
        self.start_held_repeated = repeated
        self.state = ST_ADDR
        self.in_frame = True
        self.bytes_done = 0
        self.bit_count = 0
        self.byte = 0
        self.skip = 0
        if not repeated:
            self.entdaa = False
            self.daa_left = 0

    def stop(self):
        if self.in_frame and self.bytes_done == 1 and self.bit_count == 0:
            return "turnaround-suppressed"
        if not self.in_frame:
            return "no-frame-open"
        if self.start_held:
            self.start_held = False
            self.state = ST_IDLE
            self.in_frame = False
            self.bit_count = 0
            self.byte = 0
            self.entdaa = False
            self.daa_left = 0
            return "short-frame-dropped"
        self.emit("P")
        self.bytes_done = 0
        self.state = ST_IDLE
        self.in_frame = False
        self.bit_count = 0
        self.byte = 0
        self.entdaa = False
        self.daa_left = 0
        return "STOP-emitted"

    def finish_byte(self, ninth):
        value = self.byte
        if self.start_held:
            self.emit("Sr" if self.start_held_repeated else "S")
            self.start_held = False

        if self.state == ST_ADDR:
            self.read = (value & 1) != 0
            self.emit("A", value, ninth == 0)
            if (value >> 1) == I3C_BROADCAST and not self.read:
                self.state = ST_CCC
            elif self.entdaa and self.read and (value >> 1) == I3C_BROADCAST:
                self.state = ST_DAA
                self.daa_left = I3C_DAA_PAYLOAD
            else:
                self.state = ST_DATA
        elif self.state == ST_CCC:
            self.emit("C", value, ninth != 0)
            if value == I3C_CCC_ENTDAA:
                self.entdaa = True
                self.state = ST_ADDR
                self.skip = 1
                self.bit_count = 0
                self.byte = 0
                self.bytes_done += 1
                self.at_turnaround = True
                return
            self.state = ST_ADDR if (value & I3C_CCC_IS_DIRECT) else ST_DATA
        else:
            self.emit("D", value, ninth != 0)

        self.bit_count = 0
        self.byte = 0
        self.bytes_done += 1
        self.at_turnaround = True

    def bit(self, b):
        self.at_turnaround = False
        if not self.in_frame:
            return
        if self.state == ST_DAA:
            self.byte = ((self.byte << 1) | (b & 1)) & 0xFF
            self.bit_count += 1
            if self.bit_count >= 8:
                self.emit("D", self.byte, False)
                self.bit_count = 0
                self.byte = 0
                if self.daa_left > 0:
                    self.daa_left -= 1
                if self.daa_left == 0:
                    self.state = ST_DATA
            return
        if self.skip > 0:
            self.skip -= 1
            return
        if self.bit_count < 8:
            self.byte = ((self.byte << 1) | (b & 1)) & 0xFF
            self.bit_count += 1
            return
        self.finish_byte(b & 1)


class DecSample:
    """The edge half of dec_sample(), I3C branch."""

    def __init__(self, emit, trace):
        self.i3c = DecI3C(emit)
        self.trace = trace
        self.have_clk = False
        self.prev_clk = 0
        self.prev_dat = 0
        self.in_frame = False
        self.clk_rise_tick = 0
        self.have_rise = False

    def seed(self, state):
        self.prev_clk = (state >> DEC_CLK_BIT) & 1
        self.prev_dat = (state >> DEC_DAT_BIT) & 1
        self.have_clk = True

    def sample(self, state, tick, idx):
        clk = (state >> DEC_CLK_BIT) & 1
        dat = (state >> DEC_DAT_BIT) & 1
        if not self.have_clk:
            self.seed(state)
            return

        if clk == 1 and self.prev_clk == 0:
            self.clk_rise_tick = tick
            self.have_rise = True
        settled = (not self.have_rise) or ((tick - self.clk_rise_tick) > 1)

        if settled and clk == 1 and self.prev_clk == 1 and dat != self.prev_dat:
            if dat == 0:
                self.trace(idx, tick, "START" if not self.in_frame else "Sr")
                self.i3c.start(self.in_frame)
                self.in_frame = True
            elif self.in_frame:
                why = self.i3c.stop()
                self.trace(idx, tick, "STOP -> " + why)
                self.in_frame = False
        elif clk == 1 and self.prev_clk == 0 and self.in_frame:
            self.i3c.bit(dat)

        self.prev_clk = clk
        self.prev_dat = dat


# ---- bench ---------------------------------------------------------------

# Records per call. The firmware now refuses more than it can deliver, so this
# is its maximum rather than a number chosen to stay under a cliff.
CHUNK = 79


def drain_edges(r, want):
    edges, empties = [], 0
    for _ in range(600):
        toks = r.cmd("SNIFF:RAW:READ? %d" % CHUNK).split()
        if toks and toks[0].upper() == "OK":
            toks = toks[1:]
        if not toks:
            empties += 1
            if empties > 20:
                break
            continue
        empties = 0
        raw = bytes(int(x, 16) for x in toks)
        for i in range(0, len(raw) - 4, 5):
            edges.append((struct.unpack_from("<I", raw, i)[0], raw[i + 4]))
        if want and len(edges) >= want:
            break
    return edges


def drain_events(r):
    out = []
    for _ in range(40):
        ev = r.cmd("SNIFF:READ? 256")
        body = ev.split("more=", 1)[1] if "more=" in ev else ev
        more = body.split()[0] if body.split() else "0"
        toks = [t for t in body.split()[1:]
                if t not in ("framing=i3c-t-bit-not-ack",
                             "note=bits-exact-tail-truncates-under-8-bits")]
        out += toks
        if more == "0":
            break
    return out


def run(hz, root):
    r, n = bb.rt(), bb.nu()
    r.timeout(20000)
    n.timeout(20000)
    pp, daa = i3c_setup(hz, root)
    if "targets=1" not in daa:
        print("  ENTDAA did not find the target: %s" % daa)
        return None
    n.cmd("EE:WR 0x40 " + PAT)

    for c in ("SNIFF:DMA 1", "SNIFF:DECODE 1", "SNIFF:CH i3c d8 d9",
              "SNIFF:RATE %s" % pp, "SNIFF:CLEAR"):
        r.cmd(c)
    armed = r.cmd("SNIFF:RAW:START 8000")

    w = r.cmd("I3C:W 0x09 00 40")
    rd = r.cmd("I3C:R 0x09 6")
    r.cmd("SNIFF:RAW:STOP")

    stat = r.cmd("SNIFF:RAW:STAT?")
    want = int(stat.split("edges=")[1].split()[0]) if "edges=" in stat else 0

    fw = drain_events(r)
    edges = drain_edges(r, want)

    print("  bus=%s pp=%s  write=%s read=%s" % (hz, pp, w[:12], rd[:20]))
    print("  edges claimed=%d drained=%d   %s" % (want, len(edges), armed))
    return fw, edges


def mirror(edges):
    events, trace = [], []

    def emit(kind, value=0, flag=False):
        if kind in ("S", "Sr", "P"):
            events.append(kind)
        else:
            events.append("%s%02X%c" % (kind, value, "+" if flag else "-"))

    d = DecSample(emit, lambda i, t, what: trace.append((i, t, what)))
    if edges:
        d.seed(edges[0][1])
        for i, (t, st) in enumerate(edges[1:], start=1):
            d.sample(st, t, i)
    return events, trace


def compare(fw, mine):
    n = max(len(fw), len(mine))
    first = None
    for i in range(n):
        a = fw[i] if i < len(fw) else "-"
        b = mine[i] if i < len(mine) else "-"
        if a != b and first is None:
            first = i
    print("")
    print("  firmware : %s" % " ".join(fw))
    print("  from edges: %s" % " ".join(mine))
    if first is None:
        print("  IDENTICAL - %d events, the edge list carries the whole decode"
              % len(fw))
    else:
        print("  diverge at event %d: firmware %r, edges %r"
              % (first, fw[first] if first < len(fw) else None,
                 mine[first] if first < len(mine) else None))
    return first is None


if __name__ == "__main__":
    hz = int(sys.argv[1]) if len(sys.argv) > 1 else 1000000
    root = int(sys.argv[2]) if len(sys.argv) > 2 else 24000000
    print("== edge list vs firmware decode, same capture ==")
    got = run(hz, root)
    if got:
        fw, edges = got
        mine, trace = mirror(edges)
        same = compare(fw, mine)
        print("")
        print("  framing events seen in the edge list:")
        for i, t, what in trace:
            print("     edge %-5d t=%-10d %s" % (i, t, what))
        sys.exit(0 if same else 1)
    sys.exit(2)
