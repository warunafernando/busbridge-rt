"""What ends each frame, and in what state.

Read frames come to nine bits at some rates where writes always come to
twenty-seven. The wire has been measured uniform - one edge between frames,
both directions, every rate - so whatever closes them early is in the decoder.
This reads the log it now keeps instead of inferring it from the bytes.
"""
import sys
import bb
import report as R

WHY = {0: "turnaround-suppressed", 1: "no-frame-open",
       2: "short-frame-dropped", 3: "STOP-emitted"}
SITE = {1: "oversample", 2: "pinclock"}
STATE = {0: "IDLE", 1: "ADDR", 2: "CCC", 3: "DATA", 4: "DAA"}


def unpack(w):
    return {
        "site": SITE.get((w >> 30) & 3, "?"),
        "why": WHY.get((w >> 28) & 3, "?"),
        "read": (w >> 27) & 1,
        "in_frame": (w >> 26) & 1,
        "state": STATE.get((w >> 23) & 7, "?"),
        "bit_count": (w >> 19) & 15,
        "bytes": (w >> 8) & 0x7FF,
        "held": (w >> 7) & 1,
    }


def run(r, n, hz, root, op, label):
    pp, _ = R.i3c_setup(hz, root)
    for c in ("SNIFF:DMA 1", "SNIFF:GAP 2", "SNIFF:DECODE 1",
              "SNIFF:MODE pinclock", "SNIFF:CH i3c d8 d9",
              "SNIFF:RATE %s" % pp, "SNIFF:CLEAR", "SNIFF:START"):
        r.cmd(c)
    for _ in range(4):
        op()
    r.cmd("SNIFF:STOP")
    st = r.cmd("SNIFF:STAT?")
    fb = st.split("framebits=")[1].split()[0] if "framebits=" in st else "?"
    print("  %-11s %-9s framebits=%s" % (label, pp, fb))
    if "closes=" in st:
        tail = st.split("closes=")[1].split()
        for tok in tail[1:]:
            if len(tok) != 8:
                break
            d = unpack(int(tok, 16))
            print("     %-10s %-22s read=%d frame=%d state=%-5s "
                  "bits=%-2d bytes=%-3d held=%d"
                  % (d["site"], d["why"], d["read"], d["in_frame"],
                     d["state"], d["bit_count"], d["bytes"], d["held"]))
    sys.stdout.flush()


if __name__ == "__main__":
    r, n = bb.rt(), bb.nu()
    r.timeout(12000)
    n.timeout(12000)
    for hz, root in [(1000000, 24000000), (6000000, 12000000),
                     (12500000, 25000000)]:
        print("")
        print("=== %d Hz requested ===" % hz)
        run(r, n, hz, root, lambda: r.cmd("I3C:W 0x09 00 40"), "writes only")
        run(r, n, hz, root, lambda: r.cmd("I3C:R 0x09 6"), "reads only")
