"""The .cap pipeline across bus rates, with the sample clock stated.

An oversampled capture needs samples per bit, and the intake clamp holds the
sample clock to 10 MHz on two taps however much is asked for. This sweeps the
requested clock at each bus rate, reports what the hardware actually delivered,
and decodes the resulting .cap against the traffic that was driven - so "it
failed" arrives with the samples-per-bit that explains it, rather than as a
verdict.

The useful column is the one saying what clock the engine settled on. Twice now
a rate has been argued about without that being checked, and both times the
answer was "not the one that was asked for".
"""
import os
import subprocess
import sys
import tempfile

import bb
import capture as cap
from acceptance import i3c_setup, PAT

SNIFFER = r"E:\STMicro\sniffer\target\release\sniffer.exe"
OUT = os.path.join(tempfile.gettempdir(), "bb_cap_rates.cap")

WANT_W = ["00", "40"]
WANT_R = PAT.split()

RATES = [(1000000, 24000000), (2000000, 24000000),
         (6000000, 12000000), (12500000, 25000000)]
ASK = [10000000, 20000000, 40000000]

r, n = bb.rt(), bb.nu()
r.timeout(20000)
n.timeout(20000)

print("%-10s %-10s %-10s %-6s %-5s %-5s %s"
      % ("bus", "asked", "got", "s/bit", "edges", "ovr", "decode"))
for hz, root in RATES:
    pp, daa = i3c_setup(hz, root)
    if "targets=1" not in daa:
        print("  %-9s ENTDAA found nothing: %s" % (hz, daa[:40]))
        continue
    pp = int(pp)
    for ask in ASK:
        n.cmd("EE:WR 0x40 " + PAT)
        r.cmd("SNIFF:RATE %d %d" % (pp, ask))

        def traffic():
            r.cmd("I3C:W 0x09 00 40")
            r.cmd("I3C:R 0x09 6")

        try:
            edges, tick_hz = cap.capture(r, "i3c", ["d8", "d9"], traffic,
                                         sample_hz=ask)
        except AssertionError as e:
            print("  %-9d %-10d TRUNCATED  %s" % (pp, ask, e))
            continue
        st = r.cmd("SNIFF:STAT?")
        ovr = int(st.split("overruns=")[1].split()[0]) if "overruns=" in st else -1
        dev, fw = cap.ident(r)
        cap.write_cap(OUT, edges, tick_hz, ["SCL", "SDA"], dev, fw)
        out = subprocess.run([SNIFFER, "decode", OUT, "--protocol", "i3c"],
                             capture_output=True, text=True).stdout
        body = [l for l in out.splitlines() if l.strip()]
        ok = (any("09" in l and " W " in l and all(b in l for b in WANT_W)
                  for l in body) and
              any("09" in l and " R " in l and all(b in l for b in WANT_R)
                  for l in body))
        got = [l for l in body if "PRIVATE" in l or "FRAMING" in l]
        print("  %-9d %-10d %-10d %-6.2f %-5d %-5d %s"
              % (pp, ask, tick_hz, tick_hz / pp if pp else 0,
                 len(edges), ovr, "EXACT" if ok else
                 ("; ".join(l.split("  ")[-1].strip() for l in got[:2])
                  or "nothing decoded")[:46]))
        sys.stdout.flush()
