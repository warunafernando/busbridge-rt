"""Acceptance run: I2C, I3C and the sniffer, end to end.

Every check that can be verified against the target's own memory is, over the
target's own USB link rather than over the bus under test. Checking a decode
against an expected string only proves the sniffer and the controller agree; it
does not prove either was right, and that is how an I2C test that wrote to the
wrong address passed for a long time.

Harness rules learned the hard way and encoded here:
  - SNIFF:RATE is firmware state that persists; set it every time.
  - A second ENTDAA on an addressed bus correctly finds nothing, so the target
    is cycled before each one.
  - The EEPROM address is two bytes wide.
  - Pin-clocked capture strands a frame's last byte until the next frame clocks
    it, so a capture ends with trailing traffic before it is read.

What this suite is for, and what it missed for a long time: every section but
sec_cap checks the decode the *firmware* produced, read back over SNIFF:READ?.
None of them writes a capture file or reads one. So when SNIFF:RAW:READ? began
losing seventeen edges per call to a reply line the host library silently cut,
this suite passed every run - the defect lived entirely in a path it did not
touch, and "ALL CHECKS PASSED" was true and told nobody anything. A suite that
covers the firmware is not the same as one that covers the system, and the
difference is invisible from inside it. sec_cap closes that path.
"""
import sys
import bb

PAT  = "40 11 22 33 44 55"
I2CW = "S AA0+ D00+ D00+ DDE+ DAD+ DBE+ DEF+ P"
# Pin-clocked capture strands a frame's last bits until the next frame clocks
# them, so the closing STOP of the final transaction in a window never arrives.
# The bytes are what this checks; the missing P is section 4.1, not a decode
# error.
I2CW_PIN = "S AA0+ D00+ D00+ DDE+ DAD+ DBE+ DEF+"
I3CR = "S A13+ D40+ D11+ D22+ D33+ D44+ D55+"
I3CW = "S A12+ D00+ D40-"
FAIL = []

RATES = [(1000000, 24000000), (2000000, 24000000), (4000000, 16000000),
         (6000000, 12000000), (8000000, 16000000), (10000000, 20000000),
         (12500000, 25000000)]


def chk(section, name, ok, detail=""):
    print("  %-46s %s  %s" % (name, "PASS" if ok else "FAIL", detail[:32]))
    if not ok:
        FAIL.append("%s / %s  %s" % (section, name.strip(), detail[:60]))
    sys.stdout.flush()


def clean(ev):
    b = ev.split("more=", 1)[1] if "more=" in ev else ev
    b = b[b.index(" "):] if " " in b else b
    for j in ("framing=i3c-t-bit-not-ack", "framing=i2c-ack",
              "note=bits-exact-tail-truncates-under-8-bits"):
        b = b.replace(j, "")
    return " ".join(b.split())


def stat(r):
    st = r.cmd("SNIFF:STAT?")
    g = lambda k: int(st.split(k)[1].split()[0]) if k in st else -1
    return g("overruns="), g("gaps="), g("dropped=")


def i3c_setup(hz, root, port=2):
    r, n = bb.rt(), bb.nu()
    n.cmd("EE:I3CCLK 0")
    n.cmd("EE:I3C OFF")
    n.cmd("EE:I3C ON")
    n.cmd("EE:FILL PATTERN")
    n.cmd("EE:WR 0x40 " + PAT)
    # SKEW / RXTRIG / RXDEFER used to be pinned here. They were only ever set
    # to the values the firmware already defaults to, guarding against state
    # left by a bench experiment - and the commands that could leave that state
    # have been removed, so there is nothing left to guard against.
    for c in ("I3C:FREE", "I3C:PORT %d" % port, "I3C:PAD 0", "I3C:HKEEP 0",
              "I3C:DISTO 0", "I3C:ODHPP 0", "I3C:ROOT %d" % root,
              "I3C:PULL 1", "I3C:FREQ %d 500000" % hz, "I3C:PPLOW 0",
              ):
        r.cmd(c)
    daa = r.cmd("I3C:ENTDAA")
    clk = r.cmd("I3C:CLK?")
    pp = clk.split("actual_pp=")[1].split()[0] if "actual_pp=" in clk else "?"
    return pp, daa


def sec_i2c():
    r, n = bb.rt(), bb.nu()
    print("")
    print("== I2C: controller and target ==")
    # Claim the bus and set a rate before asking anything of it. SCAN on a
    # port that has only just been selected, with whatever the previous run
    # left configured, answered "nothing here" on a bus that then passed every
    # transfer - a state artifact, not a discovery failure.
    r.cmd("I2C:BUS 2")
    r.cmd("I2C:FREQ 400000")
    r.cmd("I2C:PROBE 0x50")
    sc = r.cmd("I2C:SCAN")
    chk("i2c", "discovery: SCAN finds the target at 0x50", sc == "OK 50", sc)
    sa = r.cmd("I2C:SCANALL")
    chk("i2c", "SCANALL reports both wired buses", "i2c2 50" in sa, sa)
    for hz in (100000, 400000, 1000000):
        ok = 0
        for _ in range(5):
            n.cmd("EE:FILL PATTERN")
            r.cmd("I2C:FREQ %d" % hz)
            w = r.cmd("I2C:MEMW 0x50 0 2 DE AD BE EF")
            back = r.cmd("I2C:MEMR 0x50 0 2 4")
            tgt = n.cmd("EE:RD 0x00 4")
            if (w.startswith("OK") and back == "OK DE AD BE EF"
                    and tgt == "OK DE AD BE EF"):
                ok += 1
        chk("i2c", "%7d Hz  write+read, verified in target memory" % hz,
            ok == 5, "%d/5" % ok)


def sec_i2c_10bit():
    """Ten-bit addressing, verified in the target's own memory.

    The wire agreeing is not enough here and the project has been caught by
    exactly that before: a test wrote to the wrong address for the life of the
    project while the decode matched and passed 5/5. So every value written
    over 10-bit is read back through the target's own USB link as well.
    """
    r, n = bb.rt(), bb.nu()
    print("")
    print("== I2C: 10-bit addressing ==")
    r.cmd("I2C:BUS 2")
    for hz in (100000, 400000, 1000000):
        r.cmd("I2C:FREQ %d" % hz)
        n.cmd("EE:I2CADDR10 0x2A")
        pat = ["%02X" % ((hz // 100000 + i) & 0xFF) for i in range(4)]
        r.cmd("I2C:W10 0x2A 00 30 " + " ".join(pat))
        r.cmd("I2C:W10 0x2A 00 30")
        got = r.cmd("I2C:R10 0x2A 4")
        tgt = n.cmd("EE:RD 0x30 4")
        want = "OK " + " ".join(pat)
        # The target must answer its 10-bit address and stop answering its
        # 7-bit one - a device that ACKs everything would pass a one-sided test.
        here = r.cmd("I2C:PROBE10 0x2A")
        gone = r.cmd("I2C:PROBE 0x50")
        away = r.cmd("I2C:PROBE10 0x2B")
        n.cmd("EE:I2CADDR10 OFF")
        # Turning 10-bit off re-initialises the target's I2C peripheral, and a
        # probe that lands inside that window fails with probe-fifo-error
        # rather than answering. Reproducible at 1 MHz and not below, because
        # there the probe is quick enough to catch it. The assertion is
        # unchanged - the target must come back on its 7-bit address - it is
        # just not asked while it is still reconfiguring.
        back = ""
        for _ in range(5):
            back = r.cmd("I2C:PROBE 0x50")
            if back == "OK 1":
                break
        chk("i2c-10bit", "%8d Hz  write/read exact, addressing exclusive" % hz,
            got == want and tgt == want and "present" in here
            and gone == "OK 0" and "absent" in away and back == "OK 1",
            "rd=%s tgt=%s probe10=%s probe7=%s other=%s back=%s"
            % (got[:14], tgt[:14], here[-7:], gone, away[-6:], back))


def sec_i2c_sniff():
    r, n = bb.rt(), bb.nu()
    print("")
    print("== I2C: sniffing ==")
    for mode in ("oversample", "pinclock"):
        for hz in (100000, 400000, 1000000):
            ok = 0
            for _ in range(5):
                n.cmd("EE:FILL PATTERN")
                for c in ("SNIFF:DMA 1", "SNIFF:DECODE 1",
                          "SNIFF:MODE " + mode, "SNIFF:CH i2c d4 d5",
                          "SNIFF:RATE %d" % hz, "SNIFF:CLEAR", "SNIFF:START"):
                    r.cmd(c)
                r.cmd("I2C:FREQ %d" % hz)
                r.cmd("I2C:MEMW 0x50 0 2 DE AD BE EF")
                r.cmd("I2C:MEMW 0x50 0 2 DE AD BE EF")
                r.cmd("SNIFF:STOP")
                want = I2CW if mode == "oversample" else I2CW_PIN
                if clean(r.cmd("SNIFF:READ? 80")).startswith(want):
                    ok += 1
            o, g, d = stat(r)
            chk("i2c-sniff", "%-11s %7d Hz  exact decode" % (mode, hz),
                ok == 5, "%d/5 ovr=%d gaps=%d" % (ok, o, g))


def sec_i3c():
    r, n = bb.rt(), bb.nu()
    print("")
    print("== I3C: controller and target, every rate ==")
    for hz, root in RATES:
        pp, _ = i3c_setup(hz, root)
        daa = ccc = dccc = w = rd = 0
        for _ in range(6):
            n.cmd("EE:I3C OFF")
            n.cmd("EE:I3C ON")
            n.cmd("EE:WR 0x40 " + PAT)
            if "targets=1" in r.cmd("I3C:ENTDAA"):
                daa += 1
            if r.cmd("I3C:CCC:B 0x0A 11 22").startswith("OK"):
                ccc += 1
            if r.cmd("I3C:CCC:R 0x09 0x8E 1") == "OK 2E":
                dccc += 1
            if r.cmd("I3C:W 0x09 00 40").startswith("OK"):
                w += 1
            if r.cmd("I3C:R 0x09 6") == "OK " + PAT:
                rd += 1
        chk("i3c", "%8s Hz  entdaa/bcast/directed/write/read" % pp,
            min(daa, ccc, dccc, w, rd) == 6,
            "%d/%d/%d/%d/%d of 6" % (daa, ccc, dccc, w, rd))


def sec_i3c_ddr():
    """HDR-DDR: what the controller emits, since nothing here can answer it.

    The NUCLEO's I3C peripheral has no HDR-DDR message type - its controller
    types are RELEASE, HEADER, PRIVATE, DIRECT, LEGACY_I2C and CCC - and its
    GETCAPS advertises no HDR mode, so a DDR transfer cannot be acknowledged on
    this bench by anything. What can be checked is that the controller puts the
    right thing on the wire and that the bus survives it, and the entry sequence
    is plain SDR so our own decoder can read it.
    """
    r, n = bb.rt(), bb.nu()
    print("")
    print("== I3C: HDR-DDR (emission only - no target on this bench answers) ==")
    pp, _ = i3c_setup(1000000, 24000000)
    for c in ("SNIFF:DMA 1", "SNIFF:DECODE 1", "SNIFF:MODE oversample",
              "SNIFF:CH i3c d8 d9", "SNIFF:RATE 20000000",
              "SNIFF:CLEAR", "SNIFF:START"):
        r.cmd(c)
    wr = r.cmd("I3C:DDRW 0x09 12 34")
    r.cmd("SNIFF:STOP")
    ev = ""
    for _ in range(20):
        part = clean(r.cmd("SNIFF:READ? 250"))
        if not part.strip():
            break
        ev += " " + part
    # 0x7E broadcast (0xFC on the wire) then CCC 0x20, which is ENTHDR0 -
    # "enter HDR mode 0", HDR-DDR. Anything else means the transfer went out as
    # something other than DDR.
    chk("i3c-ddr", "entry is broadcast + ENTHDR0 (CCC 0x20)",
        wr == "OK" and "AFC" in ev and "C20" in ev, "wr=%s ev=%s" % (wr, ev.strip()[:40]))
    # A read has nobody to answer it; the controller must time out rather than
    # hang, and must leave the bus usable.
    rd = r.cmd("I3C:DDRR 0x09 2")
    sdr = r.cmd("I3C:W 0x09 00 40")
    back = r.cmd("I3C:R 0x09 6")
    chk("i3c-ddr", "unanswered read times out, bus still usable",
        rd.startswith("ERR") and sdr == "OK" and back == "OK " + PAT,
        "rd=%s sdr=%s back=%s" % (rd[:22], sdr, back[:20]))


def sec_i3c_ibi():
    r, n = bb.rt(), bb.nu()
    print("")
    print("== I3C: in-band interrupts ==")
    pls = ("AA", "55 66", "7E", "11 22 33", "A5 B6 C7 D8", "01", "FF 00", "5A")
    for hz, root in [(1000000, 24000000), (6000000, 12000000),
                     (12500000, 25000000)]:
        pp, _ = i3c_setup(hz, root)
        r.cmd("I3C:LISTEN 1")
        # Drain the queue completely, not once.
        #
        # The controller keeps a four-deep IBI queue that survives between
        # commands and between runs. Priming with a single read clears one
        # entry, so anything left behind by earlier work - a hot-join test, an
        # interrupted run - makes every reply here lag by that many and the
        # section reports 0/8 on a bus that is working perfectly. That cost an
        # afternoon of looking for a regression that was not there.
        n.cmd("I3C:TGT:IBI AA")
        for _ in range(20):
            if "none" in r.cmd("I3C:IBI?").split("|")[0]:
                break
        ok = bus = 0
        for pl in pls:
            n.cmd("I3C:TGT:IBI " + pl)
            want = "addr=0x09 len=%d %s" % (len(pl.split()), pl)
            if want in r.cmd("I3C:IBI?"):
                ok += 1
            r.cmd("I3C:W 0x09 00 40")
            if r.cmd("I3C:R 0x09 6") == "OK " + PAT:
                bus += 1
        r.cmd("I3C:LISTEN 0")
        chk("ibi", "%8s Hz  address+payload exact, bus intact" % pp,
            ok == len(pls) and bus == len(pls),
            "%d/%d ibi %d/%d bus" % (ok, len(pls), bus, len(pls)))


def sec_i3c_sniff():
    r, n = bb.rt(), bb.nu()
    print("")
    print("== I3C: sniffing, pin-clocked (the fast mode) ==")
    for hz, root in RATES:
        pp, _ = i3c_setup(hz, root)
        for c in ("SNIFF:DMA 1", "SNIFF:DECODE 1", "SNIFF:MODE pinclock",
                  "SNIFF:CH i3c d8 d9", "SNIFF:RATE %s" % pp,
                  "SNIFF:CLEAR", "SNIFF:START"):
            r.cmd(c)
        for _ in range(6):
            r.cmd("I3C:W 0x09 00 40")
            r.cmd("I3C:R 0x09 6")
        r.cmd("I3C:W 0x09 00 40")
        r.cmd("SNIFF:STOP")
        # Drain the whole capture, not the first window. Counting exact frames
        # is only meaningful against all of them.
        ev = ""
        for _ in range(40):
            part = clean(r.cmd("SNIFF:READ? 250"))
            if not part.strip():
                break
            ev += " " + part
        o, g, d = stat(r)
        # Every transaction that has traffic after it decodes exactly: six
        # reads of six, and six writes of seven. The seventh write is the last
        # thing on the bus, so its tail is still inside the shifter with
        # nothing to clock it out - confirmed by issuing two more writes, after
        # which eight of nine decode. That is the shifter tail, not framing.
        #
        # Asserted as counts rather than "the string appears somewhere",
        # which was the right test when two of six was the ceiling and would
        # now pass a serious regression without noticing.
        nr, nw = ev.count(I3CR), ev.count(I3CW)
        chk("i3c-sniff", "%8s Hz  lossless, every frame exact" % pp,
            o == 0 and g == 0 and d == 0 and nr == 6 and nw == 6,
            "ovr=%d gap=%d drop=%d rd=%d/6 wr=%d/6" % (o, g, d, nr, nw))

    print("")
    print("== I3C: sniffing, oversampled (exact below ~3 MHz) ==")
    # Two taps at 10 MHz sampling is four samples per bit at 2.5 MHz and under
    # three above 3.3 MHz, so this mode is asserted only where it can work.
    # Pin-clocked is the mode for a fast bus; see section 4.1.
    for hz, root in RATES[:2]:
        pp, _ = i3c_setup(hz, root)
        for c in ("SNIFF:DECODE 1", "SNIFF:MODE oversample",
                  "SNIFF:CH i3c d8 d9", "SNIFF:RATE %s" % pp,
                  "SNIFF:CLEAR", "SNIFF:START"):
            r.cmd(c)
        for _ in range(4):
            r.cmd("I3C:W 0x09 00 40")
            r.cmd("I3C:R 0x09 6")
        r.cmd("SNIFF:STOP")
        ev = clean(r.cmd("SNIFF:READ? 250"))
        o, g, d = stat(r)
        exact = ev.count(I3CR + " P")
        chk("i3c-ovs", "%8s Hz  every frame exact" % pp, exact >= 4,
            "%d/4 reads ovr=%d" % (exact, o))


def sec_cap():
    """The .cap pipeline: capture to a file, decode the file.

    Everything above this checks the decode the *firmware* produced, read back
    over SNIFF:READ?. Nothing above it writes a capture file or reads one, and
    that gap is not theoretical: SNIFF:RAW:READ? spent a long time losing
    seventeen edges per call to a reply line the host library silently cut, and
    this suite passed every time it ran, because the whole defect lived in a
    path it did not touch. "ALL CHECKS PASSED" was true and said nothing about
    it.

    So these four. The first is the one that would have caught that bug on the
    day it appeared, and it is first for that reason.
    """
    import os
    import subprocess

    import capture as capmod
    import edge_vs_decode as evd
    import pinclock_cap as pin

    r, n = bb.rt(), bb.nu()
    print("")
    print("== Capture files: the .cap pipeline end to end ==")

    sniffer = os.environ.get(
        "SNIFFER_EXE",
        os.path.join("E:", os.sep, "STMicro", "sniffer",
                     "target", "release", "sniffer.exe"))
    scratch = os.environ.get("TEMP", ".")
    have_analyser = os.path.isfile(sniffer)

    def decode(path):
        res = subprocess.run([sniffer, "decode", path, "--protocol", "i3c"],
                             capture_output=True, text=True)
        return res.stdout, res.stderr

    # ---- 1. the edge list carries the whole decode ----
    #
    # In oversampled mode cap_consume() feeds the firmware's decoder and the
    # sample ring from one place, same transitions and same ticks, so a
    # faithful transcription of that decoder run over the drained edge list has
    # to reproduce SNIFF:READ? exactly. Identical means the edges arrived
    # whole; a divergence names the first one lost. No analyser needed, which
    # is why it is the check that holds even on a machine with nothing else
    # built.
    pp, daa = i3c_setup(1000000, 24000000)
    n.cmd("EE:WR 0x40 " + PAT)
    for c in ("SNIFF:DMA 1", "SNIFF:DECODE 1", "SNIFF:CH i3c d8 d9",
              "SNIFF:RATE %s" % pp, "SNIFF:CLEAR"):
        r.cmd(c)
    r.cmd("SNIFF:RAW:START 8000")
    r.cmd("I3C:W 0x09 00 40")
    r.cmd("I3C:R 0x09 6")
    r.cmd("SNIFF:RAW:STOP")
    st = r.cmd("SNIFF:RAW:STAT?")
    want = int(st.split("edges=")[1].split()[0]) if "edges=" in st else 0
    fw = evd.drain_events(r)
    edges = evd.drain_edges(r, want)
    mine, _ = evd.mirror(edges)
    chk("cap", "edge list reproduces the firmware's own decode",
        fw == mine and len(fw) > 0,
        "fw=%d mine=%d edges=%d/%d" % (len(fw), len(mine), len(edges), want))

    # Separately from whether it decodes: every edge the firmware declared has
    # to arrive. A drain that quietly delivers fewer is the exact shape of the
    # bug above, and it is cheap to assert directly rather than hope the decode
    # notices.
    chk("cap", "every declared edge is delivered",
        want > 0 and len(edges) >= want,
        "declared=%d drained=%d" % (want, len(edges)))

    # ---- the boundary between the two ends ----
    #
    # The firmware will not format a reply longer than the host's line, and the
    # host reports an over-long line rather than cutting it. Each half is
    # checked where it lives - the host's in windows/tests/unit_test.c, which
    # needs no board - but the thing that actually broke was neither half: it
    # was the two disagreeing about where the limit is. That only shows at the
    # edge, so the edge is what this asks about.
    #
    # The largest reply the firmware will produce has to arrive whole. If the
    # limits ever drift apart, this is where it appears, as a reply that is not
    # a whole number of records.
    r.cmd("SNIFF:RAW:START 8000")
    r.cmd("I3C:W 0x09 00 40")
    r.cmd("I3C:R 0x09 6")
    r.cmd("SNIFF:RAW:STOP")
    big = r.cmd("SNIFF:RAW:READ? 79")
    toks = big.split()[1:] if big.startswith("OK") else []
    chk("cap", "the largest reply the firmware sends arrives whole",
        big.startswith("OK") and len(toks) % 5 == 0 and len(big) <= 1199,
        "%d bytes in %d chars" % (len(toks), len(big)))

    # And one record more is refused, naming the bound rather than truncating.
    # A firmware that accepted it would be back to formatting a line the host
    # cannot carry, which is the original defect exactly.
    over = r.cmd("SNIFF:RAW:READ? 80")
    chk("cap", "one record more is refused, not truncated",
        over.startswith("ERR") and "79" in over, over[:40])

    # The same bound on the container view, which had the same defect and the
    # same fix.
    raw_over = r.cmd("SNIFF:RAW? 400")
    chk("cap", "the container view is bounded too",
        raw_over.startswith("ERR") and "399" in raw_over, raw_over[:40])

    if not have_analyser:
        chk("cap", "analyser available to decode capture files", False,
            "not at %s - build it with cargo build --release" % sniffer[-30:])
        return

    # ---- 2. an oversampled capture decodes byte-exact from the file ----
    dev, fwv = capmod.ident(r)
    path = os.path.join(scratch, "bb_acceptance_oversampled.cap")
    capmod.write_cap(path, edges, 10000000, ["SCL", "SDA"], dev, fwv)
    out, _ = decode(path)
    chk("cap", "   1 MHz  oversampled capture decodes byte-exact",
        "0x09 W 00 40" in out and ("0x09 R " + PAT) in out,
        " ".join(out.split())[:40])

    # A recorded capture must NOT claim its timing was constructed, or the
    # warning that protects the reconstructions below means nothing.
    _, err = decode(path)
    chk("cap", "   a recorded capture is not marked synthetic",
        "SYNTHETIC" not in err, err.strip()[:40])

    # ---- 3. the pin-clocked reconstruction round-trips ----
    #
    # 12.5 MHz is four times too fast to oversample, so the waveform is rebuilt
    # from the decoded events. The values are all that survive, so the values
    # are what has to come back: the analyser's decode of the reconstruction
    # against the firmware's decode of the traffic that produced it.
    pp2, events = pin.capture_pinclocked(r, n, 12500000, 25000000)
    points = pin.synthesise(events)
    path2 = os.path.join(scratch, "bb_acceptance_pinclocked.cap")
    capmod.write_cap(
        path2, points, 2 * pp2, ["SCL", "SDA"], dev, fwv,
        source="busbridge:USB pin-clocked, waveform reconstructed",
        timing={"Synthetic": {"why": "acceptance run at %d Hz" % pp2,
                              "from": "pin-clocked decode",
                              "bit_ps": 1000000000000 // pp2}})
    ok, why = pin.check_round_trip(path2, events, sniffer)
    chk("cap", "12.5 MHz  reconstruction round-trips byte-exact", ok, why[:40])

    # ---- 4. and it says the timing is invented ----
    #
    # The one property that stops a reconstruction being read as a recording.
    # Checked on stderr, where every subcommand inherits it from open(), and on
    # the waveform's stdout, which is the output most likely to be redirected
    # somewhere the stderr warning does not follow.
    _, err2 = decode(path2)
    res = subprocess.run([sniffer, "show", path2, "--width", "40"],
                         capture_output=True, text=True)
    chk("cap", "   a reconstruction says its timing is invented",
        "SYNTHETIC" in err2 and "SYNTHETIC" in res.stdout,
        "decode=%s show=%s" % ("SYNTHETIC" in err2,
                               "SYNTHETIC" in res.stdout))


def sec_self():
    """SNIFF:SRC self - the controller-side trace, held against the wire.

    Every other sniffer section here measures the bus. This one does not: it
    records what our own I3C peripheral accepted and returned, with no taps and
    no capture core involved. That makes it the one source in this project that
    can be confidently wrong in a way nothing else notices, so the section is
    built around the comparison rather than around the feature.

    The equality check uses an OVERSAMPLED wire capture at 1 MHz, deliberately.
    A pin-clocked capture truncates the last frame of a window - open item 7 -
    so comparing against it would fail for a defect that has nothing to do with
    the trace. The pin-clocked stream is checked too, but as a prefix: a known
    disagreement whose shape is worth pinning, because if it ever stops being a
    clean prefix something new is wrong.

    The provenance checks are not decoration. A controller-side T-bit is
    computed from the byte it belongs to, so it agrees with that byte by
    construction and can never fail - and a host that compared it against a
    wire T-bit, found them equal and called that a check would have learned
    nothing while feeling it had. `tbits=computed` in the header and `~` in
    SNIFF:TRACE? are how that host is told, and they are checked both ways
    round: every data byte marked, no address byte marked. "All of them are
    marked" is also satisfied by marking everything.
    """
    S = "I3C self-trace"
    print("")
    print("== I3C: controller-side trace (SNIFF:SRC self) ==")
    r = bb.rt()

    pp, daa = i3c_setup(1000000, 24000000)
    if "targets=1" not in daa:
        chk(S, "a target is present", False, daa)
        return

    def traffic():
        r.cmd("I3C:W 0x09 00 40")
        r.cmd("I3C:R 0x09 6")

    def take_self():
        r.cmd("SNIFF:SRC self")
        r.cmd("SNIFF:START i3c")
        traffic()
        r.cmd("SNIFF:STOP")
        return r.cmd("SNIFF:READ? 120")

    raw = take_self()
    ev  = clean(raw)
    chk(S, "records say they came from the controller",
        "observed=controller" in raw and "tbits=computed" in raw
        and "addrack=observed" in raw, raw[:32])
    chk(S, "the frames driven are the frames traced",
        ev == I3CW + " P " + I3CR + " P", ev[:32])
    chk(S, "nothing was lost from the ring", "lost=0" in raw,
        raw.split("lost=")[1][:6] if "lost=" in raw else "?")
    chk(S, "a drained ring reads empty, not stale",
        clean(r.cmd("SNIFF:READ? 120")) == "", "re-read")

    # Against the wire, where equality is the claim worth making.
    selfev = clean(take_self())
    r.cmd("SNIFF:SRC wire")
    for c in ("SNIFF:DECODE 1", "SNIFF:MODE oversample", "SNIFF:CH i3c d8 d9",
              "SNIFF:RATE 1000000", "SNIFF:CLEAR", "SNIFF:START"):
        r.cmd(c)
    traffic()
    r.cmd("SNIFF:STOP")
    over = clean(r.cmd("SNIFF:READ? 120"))
    chk(S, "trace and oversampled wire agree token for token",
        selfev == over, "%d vs %d tokens" % (len(selfev.split()),
                                             len(over.split())))

    r.cmd("SNIFF:SRC wire")
    for c in ("SNIFF:DMA 1", "SNIFF:MODE pinclock", "SNIFF:CH i3c d8 d9",
              "SNIFF:RATE %s" % pp, "SNIFF:CLEAR", "SNIFF:START"):
        r.cmd(c)
    traffic()
    r.cmd("SNIFF:STOP")
    pinev = clean(r.cmd("SNIFF:READ? 120"))
    chk(S, "pin-clocked wire is a clean prefix of the trace",
        selfev.startswith(pinev) and pinev != selfev,
        "trace has %d more" % (len(selfev.split()) - len(pinev.split())))

    # What it cannot see is a hole, not a silence. A directed CCC read is two
    # SDK transfers, so it is two holes - one per frame this side cannot
    # describe. Collapsing them would understate how much is missing.
    r.cmd("SNIFF:SRC self")
    r.cmd("SNIFF:START i3c")
    r.cmd("I3C:CCC:R 0x09 0x8E 1")
    r.cmd("SNIFF:STOP")
    chk(S, "an untraceable CCC is marked, not dropped",
        clean(r.cmd("SNIFF:READ? 120")).split() == ["!", "!"], "GETBCR")

    # The case that found the one real defect in this source: an address with
    # nothing on it. The module raises MCTRLDONE for the address phase and sets
    # MERRWARN.NACK afterwards, so opening the frame at MCTRLDONE traced
    # `S AAA+` - acknowledged - for a transfer whose own reply correctly said
    # no-ack. The reply was right and the trace was confidently wrong, which is
    # the failure mode a controller-side source has and a wire capture does
    # not. Worth a check of its own for exactly that reason.
    r.cmd("SNIFF:SRC self")
    r.cmd("SNIFF:START i3c")
    r.cmd("I3C:W 0x55 AA")
    r.cmd("I3C:R 0x55 2")
    r.cmd("SNIFF:STOP")
    nack = clean(r.cmd("SNIFF:READ? 60"))
    chk(S, "a NACKed address traces as NACKed",
        nack.split() == ["S", "AAA-", "S", "AAB-"], repr(nack)[:32])

    # Timestamps, and the mark that says which flags are inferred.
    r.cmd("SNIFF:START i3c")
    traffic()
    r.cmd("SNIFF:STOP")
    tr    = r.cmd("SNIFF:TRACE? 40")
    marks = [t for t in tr.split() if "=" in t and t.split("=", 1)[0].isdigit()]
    toks  = [m.split("=", 1)[1] for m in marks]
    chk(S, "SNIFF:TRACE? pairs a time with a non-empty token",
        len(marks) == 14 and all(toks), "%d mark(s)" % len(marks))
    chk(S, "SNIFF:TRACE? peeks rather than drains",
        clean(r.cmd("SNIFF:READ? 120")).startswith(I3CW), "re-read")
    if marks:
        hzc = int(tr.split("cycle_hz=")[1].split()[0])
        d   = int(marks[-1].split("=")[0]) - int(marks[0].split("=")[0])
        chk(S, "event times advance across a transfer", d > 0,
            "%.1f us" % (d * 1e6 / hzc))
        chk(S, "every data byte is marked derived, no address is",
            all(t.endswith("~") for t in toks if t.startswith("D"))
            and not any(t.endswith("~") for t in toks if t.startswith("A")),
            "%d data, %d addr" % (sum(t[0] == "D" for t in toks),
                                  sum(t[0] == "A" for t in toks)))

    # `both` runs the two engines over one window. It used to refuse, because
    # nothing related the CM33's clock to the capture core's; clock_offset.py
    # measures that line and trace_lag.py measures what the controller's own
    # stamping adds to it. What is checked here is the cheap part - that both
    # engines really do run and describe the same frames - because the precise
    # part needs markers and a fit, and belongs in those tools.
    rc = r.cmd("SNIFF:SRC both")
    chk(S, "SNIFF:SRC both is accepted", rc.startswith("OK"), rc[:32])
    for c in ("SNIFF:DECODE 1", "SNIFF:MODE oversample", "SNIFF:CH i3c d8 d9",
              "SNIFF:RATE 1000000", "SNIFF:CLEAR", "SNIFF:START i3c"):
        r.cmd(c)
    traffic()
    r.cmd("SNIFF:STOP")
    wire_b = clean(r.cmd("SNIFF:READ? 120"))
    tr_b   = r.cmd("SNIFF:TRACE? 60")
    ctrl_b = " ".join(m.split("=", 1)[1].rstrip("~")
                      for m in tr_b.split()
                      if "=" in m and m.split("=", 1)[0].isdigit())
    chk(S, "both: the wire engine ran", wire_b.startswith(I3CW), wire_b[:32])
    chk(S, "both: the controller engine ran too",
        ctrl_b.startswith(I3CW), ctrl_b[:32])
    chk(S, "both: the two streams describe the same frames",
        wire_b == ctrl_b, "%d vs %d tokens" % (len(wire_b.split()),
                                               len(ctrl_b.split())))
    # SNIFF:STAT? emits the trace counters as a comment line AHEAD of its
    # reply. Printed after it they would not be part of that reply - they
    # would be the next line the host reads, and every reply after it would be
    # answered by the one before. Checked by reading a command after it.
    r.cmd("SNIFF:STAT?")
    chk(S, "both: STAT? leaves the reply stream in step",
        r.cmd("SYS:ID?").startswith("OK board="), "next reply")
    r.cmd("SNIFF:SRC wire")


def sec_soak():
    r, n = bb.rt(), bb.nu()
    print("")
    print("== Sustained: 12.5 MHz, sniffer and I2C running alongside ==")
    i3c_setup(12500000, 25000000)
    for c in ("SNIFF:DMA 1", "SNIFF:DECODE 1", "SNIFF:MODE pinclock",
              "SNIFF:CH i3c d8 d9", "SNIFF:RATE 12500000", "SNIFF:CLEAR",
              "SNIFF:START"):
        r.cmd(c)
    r.cmd("I2C:BUS 2")
    r.cmd("I2C:FREQ 400000")
    bad = i2cbad = 0
    N = 1000
    for i in range(N):
        r.cmd("I3C:W 0x09 00 40")
        if r.cmd("I3C:R 0x09 6") != "OK " + PAT:
            bad += 1
        if (i % 100) == 0:
            if not r.cmd("I2C:MEMW 0x50 0 2 DE AD BE EF").startswith("OK"):
                i2cbad += 1
    r.cmd("SNIFF:STOP")
    o, g, d = stat(r)
    usb = r.cmd("SYS:USB?")
    st = usb.split("stalls=")[1].split()[0] if "stalls=" in usb else "?"
    chk("soak", "%d I3C write/read pairs, every one exact" % N, bad == 0,
        "%d wrong" % bad)
    chk("soak", "interleaved I2C writes all accepted", i2cbad == 0,
        "%d bad" % i2cbad)
    chk("soak", "capture lossless throughout", o == 0 and g == 0 and d == 0,
        "ovr=%d gap=%d drop=%d" % (o, g, d))
    chk("soak", "USB clean", st == "0", "stalls=%s" % st)


if __name__ == "__main__":
    r, n = bb.rt(), bb.nu()
    r.timeout(15000)
    n.timeout(15000)
    print("board : " + r.cmd("SYS:ID?")[:64])
    print("target: " + n.cmd("SYS:ID?")[:64])
    sec_i2c()
    sec_i2c_10bit()
    sec_i2c_sniff()
    sec_i3c()
    sec_i3c_ddr()
    sec_i3c_ibi()
    sec_i3c_sniff()
    sec_cap()
    sec_self()
    sec_soak()
    print("")
    print("=" * 64)
    if FAIL:
        print("FAILURES (%d):" % len(FAIL))
        for f in FAIL:
            print("  - " + f)
    else:
        print("ALL CHECKS PASSED")
