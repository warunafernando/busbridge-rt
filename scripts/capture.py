"""Capture a scenario on the RT board and write it as a sniffer .cap file.

The analyser captures by owning the board, and here that board is also the
controller - one process cannot be both the watcher and the thing told to
drive. So traffic and capture come from one owner: arm the firmware's edge
capture, run the transactions, drain, and hand the analyser a file.

Format is core/file's: magic, then [type u8][len u32 le][crc32 u32 le][JSON],
with an index record last.
"""
import json
import struct
import time
import zlib

SESSION, SAMPLES, INDEX = 1, 2, 7

# Records per SNIFF:RAW:READ?, and the reason it is not simply the firmware's
# maximum. Each record prints as fifteen characters and the host library reads
# a reply line into 1200 bytes, so 79 records is what survives the trip. This
# used to ask for 96: the firmware formatted them, the library kept the first
# 1199 characters, and the seventeen records past the cut were lost - the
# command had already advanced its cursor, so they could not be asked for
# again. The resulting captures were well-formed and wrong, with holes inside
# read transfers, which read as a decoder fault for a long time.
READ_RECS = 79


def _rec(kind, obj):
    body = json.dumps(obj, separators=(",", ":")).encode()
    return struct.pack("<BII", kind, len(body), zlib.crc32(body) & 0xFFFFFFFF) + body


def write_cap(path, edges, tick_hz, channels, device, firmware,
              timing="Measured", source="busbridge:USB"):
    """Write an edge list as a .cap.

    `timing` goes into the session as the file's own statement about where its
    times came from, and defaults to the honest answer for a recorded capture.
    A caller that constructed its times passes the Synthetic form instead; see
    pinclock_cap.py. It is a field rather than a note in `source` because every
    tool downstream will answer timing questions from this file, and a free-text
    marker is something they would each have to think to parse.
    """
    ps_per_tick = 1_000_000_000_000 // max(tick_hz, 1)
    times, lines, wraps, prev = [], [], 0, None
    for t, l in edges:
        if prev is not None and t < prev:
            wraps += 1
        prev = t
        times.append((wraps * (1 << 32) + t) * ps_per_tick)
        lines.append(l)
    if times:
        base = times[0]
        times = [t - base for t in times]

    session = {
        "format_version": 1, "schema_version": 1,
        "source": source, "device_id": device,
        "firmware_version": firmware,
        "host_software": "busbridge-rt bench capture",
        "started_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "channels": channels, "tick_hz": tick_hz,
        "timing": timing,
    }
    # `end` is when the capture stopped, which is at or *after* the last
    # transition - not the same thing. A line that goes idle and stays there
    # produces no further edge, so taking the last transition as the end makes
    # every frame that ends in an idle level look truncated. One tick past the
    # last edge is the least this can honestly claim.
    last = times[-1] if times else 0
    chunk = {"id": 0, "start": times[0] if times else 0,
             "end": last + ps_per_tick, "times": times, "lines": lines}

    out = bytearray(b"SNIFCAP1" + struct.pack("<H", 1))
    out += _rec(SESSION, session)
    off = len(out)
    out += _rec(SAMPLES, chunk)
    out += _rec(INDEX, {
        "chunk_offsets": {"0": off}, "chunk_times": {"0": chunk["start"]},
        "first_time": chunk["start"], "last_time": chunk["end"],
        "edge_count": len(times), "gap_count": 0, "transaction_count": 0,
    })
    with open(path, "wb") as f:
        f.write(bytes(out))
    return len(times)


def capture(r, proto, taps, run, sample_hz=20000000, settle=0.05):
    r.cmd("SNIFF:CH %s %s" % (proto, " ".join(taps)))
    r.cmd("SNIFF:RATE %d" % sample_hz)
    armed = r.cmd("SNIFF:RAW:START 8000")
    tick_hz = int(armed.split("tick_hz=")[1].split()[0]) if "tick_hz=" in armed else 0
    time.sleep(settle)
    run()
    time.sleep(settle)
    r.cmd("SNIFF:RAW:STOP")
    stat = r.cmd("SNIFF:RAW:STAT?")
    want = int(stat.split("edges=")[1].split()[0]) if "edges=" in stat else 0
    edges = []
    empties = 0
    for _ in range(600):
        # The C library returns the reply payload with "OK " already stripped,
        # so there is no prefix token to drop. Dropping one anyway shifted
        # every record by a byte, which showed up as the top byte of each tick
        # toggling by 2^24 - the line mask, read one place too high.
        toks = r.cmd("SNIFF:RAW:READ? %d" % READ_RECS).split()
        if toks and toks[0].upper() == "OK":
            toks = toks[1:]
        if toks and len(toks) % 5:
            # A reply that is not a whole number of records has been cut in
            # transit. Dropping the stub quietly is what hid the line-length
            # bug for so long: the records before it are fine, so the capture
            # looks healthy and simply has a hole in it.
            raise AssertionError(
                "SNIFF:RAW:READ? returned %d bytes - not whole records. The "
                "reply was truncated; %d edges are unrecoverable."
                % (len(toks), len(toks) // 5 + 1))
        if not toks:
            # An empty read means "nothing right now", not "nothing left" -
            # the capture core hands over what it has to hand. Keep asking
            # until the declared count arrives or it stays silent.
            empties += 1
            if empties > 20:
                break
            continue
        empties = 0
        raw = bytes(int(x, 16) for x in toks)
        for i in range(0, len(raw) - 4, 5):
            edges.append((struct.unpack_from("<I", raw, i)[0], raw[i + 4]))
        if len(edges) >= want:
            break
    return edges, tick_hz


def ident(r):
    p = r.cmd("*IDN?").replace("OK ", "").split(",")
    return (p[3].strip() if len(p) > 3 else ""), (p[2].strip() if len(p) > 2 else "")
