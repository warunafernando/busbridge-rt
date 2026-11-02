"""Count the inter-frame clock edges directly, from the raw sample stream.

Every attempt at the gap constant so far has inferred the edge count from how
the decoded bytes came out. That is indirect twice over: the decoder's own
framing decides where a byte starts, and the frame events arrive on a different
path to the bits. This reads the samples themselves.

In pin-clocked mode a sample IS a bit - the shifter is clocked by SCL, one
sample per rising edge - so the raw stream is the bit stream with nothing
interpreted. The transactions are known exactly, so their bit patterns can be
found in it and the distance between them measured.

  write  addr 0x12 +ACK, 0x00 +T, 0x40 +T          27 bits
  read   addr 0x13 +ACK, then 40 11 22 33 44 55    63 bits

Whatever sits between the last bit of one and the first bit of the next is the
gap, counted rather than guessed.
"""
import struct
import sys
import bb

PAT_BYTES = [0x40, 0x11, 0x22, 0x33, 0x44, 0x55]


def parity_t(b):
    """I3C T bit on a data byte: odd parity over the eight data bits."""
    return 1 ^ (bin(b).count("1") & 1)


def bits_of(byte, ninth):
    out = [(byte >> i) & 1 for i in range(7, -1, -1)]
    out.append(ninth)
    return out


def write_bits():
    b = bits_of(0x12, 0)                      # address, ACK pulled low
    for v in (0x00, 0x40):
        b += bits_of(v, parity_t(v))
    return b


def read_bits():
    """The T bit on a read is not parity.

    Parity is what a *writer* puts in the ninth bit. On a read the target drives
    it, and it means "more data follows" - 1 for every byte, including the last
    when the controller ends the transfer itself with RDTERM rather than the
    target ending it. The capture agrees: a read decodes as `D40+ D11+ D22+ ...`
    with every byte flagged, while a write of the same bytes shows `D00+ D40-`,
    which is parity.
    """
    b = bits_of(0x13, 0)
    for v in PAT_BYTES:
        b += bits_of(v, 1)
    return b


def raw_samples(r, maxbytes=400):
    """Pull the capture container and return the per-sample channel states.

    maxbytes has to stay inside BB_MSG_MAX_PAYLOAD: asking for more returns a
    range error, which reads as an empty capture rather than as a bad request.
    """
    blob = b""
    for _ in range(400):
        out = r.cmd("SNIFF:RAW? %d" % maxbytes)
        if not out.startswith("OK"):
            break
        hexes = out.split()[1:]
        if not hexes:
            break
        try:
            blob += bytes(int(h, 16) for h in hexes)
        except ValueError:
            break
    states, off = [], 0
    while off + 16 <= len(blob):
        typ, obs, ln, crc, t_ps = struct.unpack_from("<BBHIQ", blob, off)
        off += 16
        if ln > len(blob) - off:
            break
        if typ == 0x02:                        # BB_REC_SAMPLES
            for k in range(0, ln - 7, 8):
                dt, st = struct.unpack_from("<II", blob, off + k)
                states.append(st)
        off += ln
    return states


def find(seq, pat, start=0):
    n, m = len(seq), len(pat)
    for i in range(start, n - m + 1):
        if seq[i:i + m] == pat:
            return i
    return -1


def setup(hz, root):
    r, n = bb.rt(), bb.nu()
    n.cmd("EE:I3CCLK 0")
    n.cmd("EE:I3C OFF")
    n.cmd("EE:I3C ON")
    n.cmd("EE:FILL PATTERN")
    n.cmd("EE:WR 0x40 " + " ".join("%02X" % b for b in PAT_BYTES))
    for c in ("I3C:FREE", "I3C:PORT 2", "I3C:PAD 0", "I3C:HKEEP 0",
              "I3C:DISTO 0", "I3C:ODHPP 0", "I3C:ROOT %d" % root,
              "I3C:PULL 1", "I3C:FREQ %d 500000" % hz, "I3C:PPLOW 0",
              "I3C:SKEW 0", "I3C:RXTRIG 0", "I3C:RXDEFER 0"):
        r.cmd(c)
    r.cmd("I3C:ENTDAA")
    clk = r.cmd("I3C:CLK?")
    return clk.split("actual_pp=")[1].split()[0] if "actual_pp=" in clk else "?"


RATES = [(1000000, 24000000), (2000000, 24000000), (4000000, 16000000),
         (6000000, 12000000), (8000000, 16000000), (10000000, 20000000),
         (12500000, 25000000)]

if __name__ == "__main__":
    r, n = bb.rt(), bb.nu()
    r.timeout(15000)
    n.timeout(15000)
    W, RD = write_bits(), read_bits()
    print("write pattern %d bits, read pattern %d bits" % (len(W), len(RD)))
    print("")
    print("%-9s %-8s %-18s %s" % ("rate", "samples",
                                  "write to read", "read to write"))
    for hz, root in RATES:
        pp = setup(hz, root)
        # Decode off so the raw store runs; the decoder is not involved at all.
        for c in ("SNIFF:DMA 1", "SNIFF:DECODE 0", "SNIFF:MODE pinclock",
                  "SNIFF:CH i3c d8 d9", "SNIFF:RATE %s" % pp,
                  "SNIFF:CLEAR", "SNIFF:START"):
            r.cmd(c)
        for _ in range(4):
            r.cmd("I3C:W 0x09 00 40")
            r.cmd("I3C:R 0x09 6")
        r.cmd("I3C:W 0x09 00 40")
        r.cmd("SNIFF:STOP")
        st = raw_samples(r)
        # Channel 0 is SDA in pin-clocked mode (tap 0 is the clock).
        bits = [s & 1 for s in st]

        # both boundary directions, from the same capture
        wr, rw = [], []
        i = 0
        while True:
            wi = find(bits, W, i)
            if wi < 0:
                break
            ri = find(bits, RD, wi + len(W))
            if ri < 0:
                break
            wr.append(ri - (wi + len(W)))
            # and from the end of that read to the write that follows it
            wi2 = find(bits, W, ri + len(RD))
            if wi2 >= 0:
                rw.append(wi2 - (ri + len(RD)))
            i = ri + len(RD)
        print("%-9s %-8d %-18s %s" % (pp, len(bits),
                                      "W->R " + str(wr) if wr else "W->R none",
                                      "R->W " + str(rw) if rw else "R->W none"))
        sys.stdout.flush()
