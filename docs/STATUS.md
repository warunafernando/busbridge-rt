# BUSBRIDGE-RT — Current Status

RT1186 port, 1 November 2026. Everything below was measured on hardware in this
configuration; nothing is inferred from code reading alone.

```
RT1186 (FRDM-IMXRT1186)   controller + sniffer   USB VID_1209 PID_0003
NUCLEO-H533RE             I2C and I3C target     USB VID_1209 PID_0001
```

Trees: `E:\BusBridge-RT` (the port), `E:\STMicro` (the NUCLEO's firmware),
`E:\RT1186` (SDK, docs, test venv).

---

# 1. Headline

**I3C is complete at a true, measured 12.5 MHz**, in-band interrupts included.
ENTDAA, broadcast CCC, directed CCC, private write, private read and IBI all
pass; 2000 consecutive write/read pairs at 12.5 MHz return 2000/2000 exact.

**I2C is complete**, and now verified against the target's array rather than
only against the wire - which is what found a test that had been writing to the
wrong address for the life of the project.

**Three separate faults were found and fixed, each of which had been described
wrongly before.**

1. The controller's I3C pads were configured open-drain. I3C SDR data is
   push-pull, so every rising edge in a payload was made by a 40 kOhm internal
   pull-up. This was the receive duplication, the mid-range write stall and the
   12.5 MHz timeout, all at once.
2. The target assembled write frames by pairing "bytes taken so far" with "frame
   counter", two independent reads of state its own interrupt updates in
   between. About 7% of writes split or merged. A pointer write silently set the
   wrong pointer and the read after it returned the wrong data with no error.
3. The receive FIFO fault was never "the first byte repeated" - it was
   over-reception, more entries than the wire carried bytes.

**A read that over-receives is now an error, not a wrong answer**, and **the
controller can no longer be wedged by the bus** - every polled wait in the SDK's
I3C driver was unbounded.

**I2C is exact in oversampled capture** at 100 kHz, 400 kHz and 1 MHz on four
taps.

**Exact I3C capture reaches ~2.9 MHz**, up from ~2 MHz, after the intake budget
was measured properly rather than inferred.

**Pin-clocked capture is now exact at every rate from 1 to 12.5 MHz.** Every
transaction with traffic after it decodes to its exact byte string - forty-two
trials across seven rates, no variance - where before about half of them merged
into their neighbours and 4 and 6.06 MHz decoded no reads at all. The fault was
the anchor: frames were opened from the START flag, which is prompt while the
bits it precedes are not, so a frame could open a bit late and decode shifted.
Anchoring instead to the previous frame's end - known exactly in the decoder's
own bit stream - makes `SNIFF:GAP` mean what it says, and its right value is the
1 that was measured on the wire all along. See 4.1.

**Capture files now decode byte-exact, and did not before.** A `.cap` written
from a 1 MHz capture decodes to the traffic that produced it, 10 runs out of 10,
where the same test had been 0 of 10. The cause was not in the analyser, which
is where it was hunted for days: `SNIFF:RAW:READ? 96` formatted 1442 characters
of hex and the host library kept 1199 of them, cutting the line mid-record and
destroying seventeen edges per call that the command had already advanced past.
The holes fell inside read transfers, so the file stayed well-formed and decoded
to plausible, wrong bytes. Both ends are bounded now and the acceptance suite
covers the file path, which it never had - which is why it passed every run
while this was happening. See 5.3.

**Buses too fast to oversample can still be put in a capture file**, by rebuilding
the waveform from a pin-clocked decode. Byte-exact at 6.06 and 12.5 MHz, with the
timing marked invented in the file itself rather than in prose, because a
reconstruction is indistinguishable from a recording by inspection and every
duration read off one is an echo of the bit period it was drawn on. See 4.1.

---

# 2. What works

## 2.1 I2C — complete in oversampled capture

Oversampled capture is exact at 100 kHz, 400 kHz and 1 MHz, 5/5 at each speed
across repeated runs - and the payload is now verified in the target's array,
not only on the wire, 5/5 at every speed in both capture modes:

```
write      S AA0+ D00+ DDE+ DAD+ DBE+ DEF+ P
read-back  S AA0+ D00+ Sr AA1+ D00+ D01+ D02+ D03- P
```

Address, pointer, payload, repeated START, read data and the closing NACK - all
from two tap wires, confirmed by the target's own counters.

Pin-clocked capture of the same traffic drops the final byte and the STOP
(`n=6` against `n=8`, ending `... DBE+` instead of `... DEF+ P`). The reply says
so itself - `note=bits-exact-tail-truncates-under-8-bits` - and it is the known
tail-flush limitation in section 3, not a decode error: the last bits sit in the
FlexIO shifter with no further clock to push them out. Oversampled mode is the
one to use for exact I2C decode.

### 10-bit addressing

`I2C:W10`, `I2C:R10` and `I2C:PROBE10` carry a 10-bit address; the target
answers one via `EE:I2CADDR10`. Exact at 100 kHz, 400 kHz and 1 MHz, with every
value read back through the target's own USB link as well as off the wire -
the project has been caught by a wire-only check before.

The SDK cannot do this: `lpi2c_master_transfer_t.slaveAddress` is documented as
"the 7-bit slave address" and the blocking transfer shifts it into bits 7:1 with
the direction underneath, with no flag that widens it. So the transfer is built
from `LPI2C_MasterStart`/`Send`/`RepeatedStart`/`Receive`/`Stop`: a header byte
`11110-A9-A8-RW`, then `A7..A0` as the first data byte. A read needs the header
twice - written first, because the low byte has to go out before the target
knows it is addressed, then a repeated START re-sends it with the read bit set.

Separate commands rather than widening `I2C:W`, because the two are different
transfers and not different number ranges: a 10-bit address below 0x80 is legal
and would be indistinguishable from a 7-bit one if a single command guessed from
the value.

The acceptance suite checks addressing is *exclusive*, not just that the target
answers - a device that ACKs everything would pass a one-sided test. With 10-bit
on, 0x2A answers, 0x2B does not, and the 7-bit 0x50 stops answering; with it off,
0x50 comes back.

One measured quirk: turning 10-bit off re-initialises the target's I2C
peripheral, and a probe landing inside that window returns `probe-fifo-error`
instead of an answer. Reproducible at 1 MHz and not below, where the probe is
slow enough that the window has passed. It is a property of reconfiguring the
target mid-conversation, not of 10-bit addressing, and it clears on the next
attempt.

## 2.2 I3C — by operation

Two changes unlocked full rate, and they are independent.

**ODHPP off**, now the default and against the SDK's. It drives the open-drain
SCL HIGH pulse push-pull, so the pulse width follows the push-pull rate rather
than the open-drain one - about 83 ns at 6 MHz, 40 ns at 12.5. Every transfer
opens with an open-drain address header, so above roughly 6 MHz the target
stopped acknowledging its own address. A fast-bus symptom with a pulse-width
cause.

**Dynamic address assignment capped at ~3 MHz**, with the configured rate
restored afterwards. The DAA response is the target driving, and the target
cannot drive at 12.5 MHz.

**The controller's I3C pads driven push-pull** - `BB_PAD_I3C` rather than
`BB_PAD_I2C`. See section 4.1; this is what made reads work above 1 MHz.

Two settings that mattered before that fix and do not now: the target's pad
slew, and choosing the internal pull-up per rate. Both moved the same RC, and
with the pads driving properly the stock values work at every rate. The target's
default is back to ST's VERY_HIGH, re-measured at 5/5 on 6 and 12.5 MHz.

Measured, target freshly cycled before every ENTDAA, eight attempts each:

| operation | 1.07 | 1.96 | 4.00 | 6.06 | 8.00 | 10.0 | 12.5 MHz |
|---|---|---|---|---|---|---|---|
| ENTDAA | 8/8 | 8/8 | 8/8 | 8/8 | 8/8 | 8/8 | 8/8 |
| broadcast CCC | 8/8 | 8/8 | 8/8 | 8/8 | 8/8 | 8/8 | 8/8 |
| private write | 8/8 | 8/8 | 8/8 | 8/8 | 8/8 | 8/8 | 8/8 |
| private read | 8/8 | 8/8 | 8/8 | 8/8 | 8/8 | 8/8 | **8/8** |

**In-band interrupts work, payload included.** 30/30 exact at 1.07, 6.06 and
12.5 MHz, with 1-, 2-, 3- and 4-byte payloads and ordinary traffic interleaved,
and the bus healthy after every one.

Getting the payload right took two more fixes after the interrupt itself
worked, and both were flags that lie:

- **Every byte has to be waited for, not just the first.** The wait ran once
  before the drain loop, which then took whatever had landed and stopped at the
  first empty read. The symptom named the bug exactly: every one-byte payload
  was reported correctly and every two-byte payload came back as `len=1` holding
  the first byte. Not a late byte or a lost one - a byte nobody waited for.
- **`COMPLETE` is not a valid terminator here.** It is set while `MSTATUS.STATE`
  still reads `IbiRcv`, so the frame reports itself finished before the payload
  has landed. Using it turned the fault inside out: two-byte payloads started
  arriving whole and single-byte ones came back as `len=0`. It is also left set
  by the *previous* transfer, which is why the reported length depended on what
  the host happened to do last. Both ends are fixed - the stale flags are cleared
  before the request, and the wait is a bounded per-byte spin that trusts only
  the receive count.

**In-band interrupts work.** `I3C:LISTEN 1` now means the controller is
listening: `bb_i3c_poll()` services target-initiated requests from the command
loop and queues them, and `I3C:IBI?` reports address and payload. 5/5 at
12.5 MHz with the payload exact and the bus healthy afterwards 5/5.

Three things had to be right together, and each was wrong in a way that looked
like something else:

- Answering only when the host asked lost the transaction. A target that raises
  an IBI blocks until the controller answers, so it timed out and reported
  `ibi-refused hal=3` while the controller, asked afterwards, still found the
  latched request and read the right address off a bus nobody was driving.
- The plain ACK response takes no data, so the address was right and `len=0`:
  the data phase the target was waiting to drive never happened.
  `kI3C_IbiRespAckMandatory` is the one for a target whose BCR says it sends a
  byte.
- Closing with `i3c_recover()` tore down a healthy bus. It is the failure path -
  it flushes both FIFOs and force-exits - so every transfer after an interrupt
  came back `no-ack-for-0x09` until the next ENTDAA. A serviced IBI ends with a
  STOP, like any other frame.

On the target, an armed reply parks the HAL in `BUSY_TX` with nothing in flight,
and `HAL_I3C_Tgt_IBIReq()` refuses that with `HAL_BUSY` and the error code still
`NONE` - reported as `ibi-refused err=0x0`, a failure with no stated cause. The
reply is released first now, and the refusal reports the HAL state as well.

The full standard CCC set passes at 6 MHz and 12.5 MHz, broadcast and directed -
6/6 directed GETs and 4/4 broadcast in the regression:

```
broadcast  ENEC DISEC SETMWL SETMRL RSTDAA        all OK
directed   SETMWL SETMRL                          all OK
directed   GETPID    02 08 13 81 10 00
           GETBCR    2E        GETDCR    C6
           GETSTATUS 00 00     GETMWL/GETMRL 00 40
```

GETBCR and GETDCR match what ENTDAA reported, and GETMWL/GETMRL read back what
SETMWL/SETMRL had just written - so the directed path is carrying its target
address correctly, which was previously listed as unproven. Directed CCC reads
had been recorded as failing; that measurement was taken before the pad fix in
4.1 of the previous revision and the pad fix cured it.

Sustained: 2000 consecutive write/read pairs at 12.5 MHz with the sniffer and
an interleaved I2C load running, **2000/2000 exact, 0 wrong, 0 write errors**.

`I3C:CLK?` reports the actual push-pull rate for every row; none of these are
requested figures.

Writes are verified end to end, not merely acknowledged: the read that follows
each one returns the bytes the target actually holds, and the sniffer sees the
same six bytes on the wire.

Previously private reads were correct only at 1 MHz. The four working rates
above are new, and 6.06 MHz is six times the old ceiling.

## 2.3 Protocol decode

Private transfers, byte-exact, with real T-bit semantics:

```
I3C:W 0x09 00 40 11 22  ->  S A12+ D00+ D40- D11+ D22+ P
```

Broadcast CCCs, code and payload:

```
sent 0x01 -> S AFC+ C01- D11+ D22+ D33+ D44+ P
sent 0x0A -> S AFC+ C0A+ D11+ D22+ D33+ D44+ P
sent 0x1F -> S AFC+ C1F- D11+ D22+ D33+ D44+ P
```

Directed CCCs recognised as such: `I3C:CCC:W 0x09 0x90 11 22` -> `S AFC+ C90+ ...`

ENTDAA in full, identically in both capture modes:

```
S AFC+ C07- AFD+ D02- D08- D13- D81- D10- D00- D2E- DC6- D13-
```

Broadcast acked, command code, 0x7E+R header acked, eight bytes of PID/BCR/DCR,
then the assigned address with its parity bit.

## 2.4 Integrated system test

Controller, target, sniffer and USB simultaneously - 60 rounds:

```
i2c writes 60/60   i3c writes 60/60   sniffer reads 60 (1080 events)
i2c reads  60/60   i3c reads  60/60   overruns 0, gaps 0, usb errors 0
```

An earlier 100-round run was identical. The I3C in this test runs at 1 MHz,
which is where reads work.

## 2.5 Capture engine

| | |
|---|---|
| CM7 capture core | 792 MHz (ARM PLL), reported by `SNIFF:STAT? core_hz` |
| CM33 controller | 240 MHz |
| FlexIO root | 120 MHz (the part's maximum) |
| Intake budget | 20 M samples/s across taps, so 10 MHz per tap on two (`CAP_INTAKE_SAMPLES_PER_S`) |
| Framing | hardware-qualified by SCL (FlexIO `TIMENA=101b`) |
| Pin-clocked word | 8 bits |

---

# 3. Known limitations (understood, not defects)

**Pin-clocked capture truncates the last bits of a transfer, and that rotates
the next one.** A shifter emits nothing until its word fills, and FlexIO here
exposes no live timer counter. This was recorded as a bounded cost - the last
few bits - and it is not: the stranded bits contaminate the following frame. See
4.1, which has the measurements and what was tried. Oversampled capture is
unaffected.

**Oversampled capture cannot follow a fast bus.** The clamp allows 10 MHz per
tap on two taps, which is 1.7 samples per bit on a 6 MHz I3C bus and 0.8 on a
12.5 MHz one, against the four or more a transition needs to be resolved.
Measured through the whole file path, 1.07 and 1.96 MHz decode byte-exact and
6.06 and 12.5 MHz recover the address and lose the payload. Pin-clocked is the
high-rate mode; oversampled is the low-rate forensic one. For a capture *file*
at the fast rates there is a reconstruction - see 4.1 - which is a different
thing from this limitation being lifted.

**The PID's first two bytes differ from the controller's report, and both are
right.** The wire reads `02 08 ...` where the controller says `pid=0104...`. The
15-bit MIPI manufacturer ID is not byte aligned - the wire carries it
left-aligned, the controller packs it right-aligned. The SDK's own code shifts
that field right by one when it assembles the PID, which is independent
confirmation.

---

# 4. Open

Most of what follows is now closed, and kept because the measurements in it are
the reason the design is what it is. **This is what is actually still open** - eight of the eleven. #2, #10 and #11
were resolved and are kept struck through with their answers:

| # | Open item | Why it is not done | Where |
|---|---|---|---|
| 1 | **I3C1 (port 1) has never been exercised** | Needs two wires to the target's PB6/PB7. Everything firmware-side is now proven and the pad encoding checked; `i3c1_bringup.py` stops at ENTDAA with no target. | 4.2 |
| 2 | ~~`SNIFF:SRC self` / `both` - controller-side trace~~ | **Resolved.** Both implemented. The clock offset is measured with physical markers, the two clocks share a reference to 0.3 ppb, and the merge is good to about 1.5 us - enough to order frames, not events inside one. | 4.3 |
| 3 | **HDR-DDR data phase** | Entry is verified on the wire; nothing on this bench answers in DDR, so the data phase has never been observed. Needs a DDR-capable target. | 4.6 |
| 4 | **Hot-join target-side reporting** | An ST HAL limitation. The target reports a refusal for requests the controller answers correctly; three fixes were measured and none is right. | 4.6 |
| 5 | **I2C clock stretching** | Nothing on this bench stretches, so the path is untested rather than known good. | 3 |
| 6 | **`SYS:DMACAP` captures nothing** | Kept deliberately rather than removed. | 4.5 |
| 7 | **Pin-clocked capture truncates the last frame of a window** | A shifter emits nothing until its word fills, and this part exposes no live timer counter. Bounded and understood; it is why a reconstruction's final transfer reads `[incomplete]`. | 3, 4.1 |
| 8 | **Oversampled capture cannot follow a bus above ~3 MHz** | Arithmetic, not a defect: 1.7 samples per bit at 6 MHz. The pin-clocked reconstruction is a workaround for the *file* path, not a fix for this. | 3, 4.1 |
| 9 | **The intake clamp may be low, and the measurement is unresolved** | Raising it to 15 MHz/tap passed the toggle generator and made real captures worse. A next attempt needs a load at least as heavy as a 2 MHz bus with the controller active, and must check the decode, not the overrun counter. | 5.2 |
| 10 | ~~The I3C read-turnaround guard has never been seen to fire~~ | **Resolved.** Built in the simulator, shown load-bearing, and pinned by a test. Still not evidenced from this bench - see below. | 4.1 |
| 11 | ~~`BB_ERR_OVERFLOW` in the host library has no coverage~~ | **Resolved.** Covered by `windows/tests/unit_test.c`, which needs no board, and the firmware/library boundary is now checked by the suite. | 5.3 |

The two that were closed are written up below, because how they were closed
matters more than that they were.

**#10 is resolved, and the answer was "a case this bench cannot produce".**
Both halves were settled rather than argued.

*Does it happen here?* No. Every SDA move with the clock held in a clean capture
is accounted for - START, STOP, or a STOP's setup - at 1.07 and 1.96 MHz, ten of
ten each, with none at the byte boundary a handover would land on. The
`0x08 W 88 91 99 A2` the guard's comment used to cite as its evidence was the
reply-line corruption, not a turnaround.

*Is it load-bearing?* Yes, for the case it describes.
`read_address_with_handover()` in the simulator builds a target that releases
SDA under a held clock, with and without the data phase re-driving it low. With
the guard the read decodes whole; with the guard disabled the frame ends at its
address and the payload is discarded for want of a START - `0x09 R` where the
bus carried `0x09 R 40 11`. That was measured by disabling it, not predicted,
and it is the only test in the suite that fails when the guard goes.

The first attempt at the waveform was wrong in a way worth recording: emitting
the handover *after* a normally acknowledged byte adds a clock edge, so an extra
bit is sampled and the payload shifts by one. The test failed, but for a defect
the guard has nothing to do with. The handover is a change *within* the
acknowledge bit, not an addition to it, and a test built on the wrong waveform
would have proved nothing while looking like proof.

Kept, with the comment rewritten to say it is defended by construction rather
than by evidence, and that anyone deleting it should reproduce the test rather
than reason about it. A zero-length read ends at the same boundary the guard
fires on, so `a_zero_length_read_still_ends_where_it_ends` pins that it is not
swallowed.

**#11 is resolved, in the two places it actually lives.**

The firmware and the host library were fixed together - one refuses to send an
over-long line, the other refuses to truncate one silently - so with both in
place the second really is unreachable from the bench. It is covered where it
lives instead. `windows/tests/unit_test.c` drives `bb_winusb_read_line()`
directly, with a line staged in the handle's own receive buffer so no WinUSB
call happens and no board is needed. `cap` is a parameter, so overflow is
exercised with a small one rather than by staging 1200 characters: the same code
path, and it fits the 512-byte buffer a real packet arrives in. Ten assertions,
built by `windowsuild.ps1` alongside `api_test.exe`.

Verified by reverting rather than by reading: with the old truncating `return
len` put back, three of the ten fail and the harness exits non-zero - and the
reported value is `got 15`, which is the success-carrying-a-truncated-prefix
that cost the days in the first place.

One of those ten does not guard the old bug and says so: the stream-resync
check passes either way, because the old code also consumed to the newline. It
is there against a future simplification that returns at the overflow point,
which would leave the tail of one long line to be read as the next and
desynchronise every reply after it - worse than the truncation it replaced.

But neither half was the actual defect. **The two ends disagreeing about where
the limit is** was, and that only shows at the edge, so `sec_cap` now asks about
the edge: the largest reply the firmware will send arrives whole (395 bytes in
1187 characters, against the library's 1199), one record more is refused naming
the bound, and `SNIFF:RAW?` is bounded the same way. The twelve characters of
margin are the remainder of `(1199 - 2) / 3`, not slack.

## 4.1 Pin-clocked capture: lossless at every rate to 12.5 MHz

### Fixed: frames no longer rotate

Pin-clocked capture used to decode the first frame after arming and turn every
frame after it into rubbish, at every rate including 1 MHz. Three identical
writes came out as `S A12+ D00+ Sr P S A09+ D00-`.

The diagnosis that stood for most of this project - that the last bits of a
frame are stranded in a partly-filled shifter and lost - was only half right.
They are stranded, but they are **not lost**: they come out as the leading bits
of the next word, late but present and in order. The bit stream across frames is
complete. What was wrong was where the frame events landed in it.

`dec_i3c_start()` was applied the instant the hardware START flag was seen,
which reset the decoder's bit counter while the previous frame's tail was still
in front of it. Those bits were then read as the new frame's address and
everything after them shifted by a group. The STOP was already deferred to a
byte boundary for exactly this reason; the START was not.

Three changes, each measured:

- **Defer the START to a byte boundary**, like the STOP. The tail decodes as the
  frame it belongs to and the START lands on the first bit of the next one.
- **Apply framing between bits, not instead of one.** Both branches used to
  `continue`, discarding the bit they landed on - which for a STOP immediately
  followed by a START is the new frame's first bit, so its address decoded one
  bit late (0x12 read as 0x09).
- **Skip the inter-frame clock edges.** The stream is continuous within a frame
  but not across the gap: ending one transaction and starting the next moves SCL
  without carrying data. Swept: none skipped is two bits early, one is one bit
  early, two is exact.

### Fixed: the intake got fast enough to matter

Three costs, all per sample - and in this mode a sample is a bus clock:

| change | effect |
|---|---|
| ring index carried, not `s_write % 24576` (a 64-bit division, twice, per sample) | 4 -> 6 MHz |
| raw sample store skipped while decoding | 6 -> 8 MHz |
| `cap_shifter_count()` hoisted out of the per-sample loop | 8 MHz clean |

`SNIFF:DECODE 0` still records every raw sample; what is not available is both
at once at pin-clock rates.

### Measured now

| bus | overruns | decode |
|---|---|---|
| 1.07 MHz | 0 | exact, write and read |
| 4.00 MHz | 0 | exact |
| 6.06 MHz | 0 | exact |
| 8.00 MHz | 0 | exact |
| 12.5 MHz | 0 | exact, with the DMA intake (see below) |

### Full rate: fixed, by taking the intake off the CPU

The bottleneck was measured rather than argued, with the cycle counter on the
core that does the work. At 12.5 MHz a pin-clocked shifter word arrives every
640 ns - **486 cycles** at 792 MHz. The polled intake cost about **520**:

| | cycles/word |
|---|---|
| `cap_consume()` - unpack 8 bits and decode them | ~337 |
| shifter status read, settle, `SHIFTBUF` read | ~180 |
| **total** | **~520 against a 486 budget** |

The second row is three trips across the FlexIO bus before any decoding starts,
and it is the row that does not have to exist. `SNIFF:DMA 1` - now the default -
puts an eDMA channel on the shifter's own DMA request, writing words into a
512-entry ring in non-cacheable memory; the core reads them from RAM and looks
at one DMA register per poll instead of three FlexIO registers per word.

Measured over ten write/read rounds, pin-clocked:

| bus | polled | DMA |
|---|---|---|
| 1.07 MHz | 0 overruns | 0 overruns, identical decode |
| 6.06 MHz | 0 overruns | 0 overruns, identical decode |
| **12.5 MHz** | **5 overruns, no frame decodes** | **0 overruns, decodes as 1 MHz does** |

So 12.5 MHz capture now behaves exactly as 1 MHz capture does. `SNIFF:DMA 0`
restores the polled path.

Two things that were tried on the way and did not help, both measured: reducing
the per-poll and per-bit bookkeeping (the mailbox reads, the ring index, hoisted
loop invariants, a single framing predicate) moved the ceiling from 4 MHz to
8 MHz but no further, and `-flto` cut `cap_consume` from 418 to 336 cycles
because `dec_i3c_bit()` lives in another translation unit. Both are kept; neither
closes the last 1.5x on its own.

The framing timers are still read once per poll in the DMA path, not per word.
That cadence matters: checking them per word broke 1 MHz capture outright, and
checking them only on an idle pass let events sit too long. The framing path
qualifies an edge by reading the clock level out of FlexIO's pin register, which
is only correct while that level still holds, so it is timing-sensitive in a way
the data path is not.

### Fixed: the phantom frame at 12.5 MHz

A capture at 12.5 MHz grew a transaction that was never on the bus -
`... D55+ P S A25+ P` - with the real frames either side decoding exactly. It
was not a phantom in the end: `A25` is the next read's address, shifted by a
bit.

The cause is at the end of a read. The target releases SDA while the clock is
held high, and that is the shape of a STOP; the controller then emits the real
one. Two STOPs for one frame end. Each applied the inter-frame gap skip, so the
following transaction started two bits out. At 8 MHz the same event showed up
more mildly, as a doubled `P`.

Three rules, none of which needs a timing constant:

- **A START is held until its frame proves itself.** An I3C transaction opens
  with an address and its acknowledge, so a frame that closes before nine bits
  have been clocked did not open one. Holding the event until those bits arrive
  lets a frame that never happened be dropped whole rather than reported and
  then contradicted.
- **A STOP with nothing open is not a STOP.** The capture layer cannot filter
  this, because a phantom START has already told it a frame is open; the decoder
  knows whether one really is.
- **The gap skip belongs to a frame that really ended** - applied only when the
  frame just closed carried a whole address.

Measured after, twelve write/read rounds per rate, pin-clocked with the DMA
intake:

```
1.07 MHz   overruns 0    4.00 MHz   overruns 0    6.06 MHz   overruns 0
8.00 MHz   overruns 0    12.5 MHz   overruns 0
```

12.5 MHz is no longer distinguishable from 1 MHz: same overrun count, same
decode rate, same stray-event count. Whatever is left is not a full-rate
problem.

### Mostly fixed: spurious address-only frames

The decoder now logs every frame close with the state that decided it -
`dec_i3c_close_log()`, drained through `SNIFF:STAT?` - after four hypotheses
about the wire had each been measured and found not to explain anything.

That log answered it in one run. Writes close at `bytes=3` - address plus two
payload bytes - at every rate, always. Reads close at `bytes=7` when they work,
and when they do not the capture contains extra closes like this:

```
STOP-emitted   read=0 frame=1 state=DATA bits=0 bytes=1
```

A frame containing one byte and nothing else, in a **read-only** workload. It is
the handover at the end of a read: the target releases SDA while the clock is
held, the detector reports a START, nine misaligned bits are assembled into
something that looks like an address, and a STOP closes it. Those nine bits
belong to the next transaction, so everything after shifts.

The guard is that a frame ending with only its address decoded is not a transfer
this controller issues - a read carries RDTERM of at least one byte, a write at
least one payload byte. Scoped to reads at first, which was too narrow: the log
showed the spurious frames arriving with `read=0`, because an address assembled
from misaligned bits has whatever direction bit the data happened to give it.

Measured after, six write/read rounds per run, four runs:

| bus | exact reads / writes |
|---|---|
| 1.07 MHz | 2/1  2/1  2/1  2/1 |
| 1.96 MHz | 2/1  2/1  2/1  2/1 |
| 4.00 MHz | 2/1  0/3  2/1  2/1 |
| 6.06 MHz | 0/3  0/3  0/3  0/3 |
| 8.00 MHz | 3/1  2/2  2/2  3/1 |
| 10.0 MHz | 2/2  2/2  3/1  1/3 |
| 12.5 MHz | 3/1  2/2  3/1  3/1 |

Those figures are the state *at this point in the work*, not the current one -
4.00 and 6.06 MHz were still failing here. They were fixed afterwards by the
framing-order change below, which is a different bug that this guard had been
masking part of.

4 MHz went from decoding no reads at all to decoding them. The cost is that a
genuine address-only transaction - `I3C:PROBE` emits exactly one - no longer
appears in a pin-clocked capture; oversampled capture still shows it.

### Fixed: pin-clocked read decode, at every rate

Six trials per rate, counting reads that decode to their exact six-byte string,
before and after:

```
rate       before                 after                  reorders
1069518    [2,2,2,2,2,2]          [2,2,2,2,2,2]           0
1960784    [0,2,2,2,2,2]          [2,2,2,2,2,2]           1
4000000    [0,0,0,0,2,0]          [2,2,2,2,2,2]           6
6060606    [0,0,0,0,0,0]          [2,2,2,2,2,2]           6
8000000    [0,2,1,0,0,0]          [1,2,2,3,3,3]           3
10000000   [3,2,3,2,2,3]          [2,3,3,3,1,3]           0
12500000   [3,3,3,3,3,3]          [3,3,2,3,3,2]           0
```

The `reorders` column is the count of boundaries where the new condition
actually fired. It is non-zero at exactly the rates that improved and zero at
the two top rates - where the code path is therefore bit-identical to before, so
their trial-to-trial differences are the usual variance rather than a
regression. 6.06 MHz went from never decoding a read to decoding every one.

**What it was.** A STOP and a START can both be waiting to be applied at the
same byte boundary. STOP-then-START is right for an ordinary back-to-back
boundary. It is wrong for one case: when the pending START is the START *of the
very frame the pending STOP closes*, applying the STOP first hands it to a
decoder whose frame was never opened. It is dropped as `no-frame-open`, the
frame runs on into the next one, and a seven-byte read closes at eight bytes
having absorbed nine extra bits - which is the shape 6.06 MHz showed all along.

**Two earlier attempts failed, and why.** Applying them in raw flag-arrival
order took 1 to 6 MHz from decoding reads to decoding none: "the START flag
arrived first" is the *ordinary* case, not the rare one, because a deferred STOP
cannot be applied until the next frame's clocking delivers the previous frame's
stranded tail, so the next START is nearly always already flagged. The condition
fired on almost every boundary.

Ordering them by `cap.tick` then found nothing at all - the two stamps came out
equal at four of seven rates and differed by exactly 8 or 16 at the rest, and
the conclusion drawn was that the ordering information did not exist. **That
conclusion was wrong, and the error is worth recording.** Those numbers are word
boundaries because `cap.tick` advances as words are *consumed*, while
`cap_frame_poll()` runs many times per word. The flags were always sampled
finely; the clock stamping them was coarse. Reading a quantised measurement as
an absent signal is an easy mistake to make twice.

**The fix.** Stamp both flags with `DWT->CYCCNT` - free-running at the core
clock, already read by the profiling in `cap_poll()`, one register read - and
apply the START first when it was stamped strictly earlier than the STOP.
Signed difference, so counter wrap is a non-event. This resolves the poll
cadence rather than the word cadence, which is the resolution the flags actually
have, and it fires on the failing case and only on it.

No change to the intake was needed. Sampling the framing flags per bit inside
the word loop - the change this looked like it needed - would have cost eight
times the register traffic to recover a resolution the poll loop already had.
The DMA queue high-water mark is 1 to 3 words (`availmax`), so the consumed bits
lag the live flags by 8 to 24 bits, which is another reason reading live flags
against queued data would not have helped.

`bothpend`, `reorder` and `availmax` in `SNIFF:STAT?` are the instrumentation
that settled this and are kept.

### Fixed: pin-clocked capture is exact at every rate to 12.5 MHz

Every transaction that has traffic after it decodes to its exact byte string, at
every rate. Six trials per rate, reads exact of 6 and writes exact of 7:

```
1069518   6/6 6/6 6/6 6/6 6/6 6/6
1960784   6/6 6/6 6/6 6/6 6/6 6/6
4000000   6/6 6/6 6/6 6/6 6/6 6/6
6060606   6/6 6/6 6/6 6/6 6/6 6/6
8000000   6/6 6/6 6/6 6/6 6/6 6/6
10000000  6/6 6/6 6/6 6/6 6/6 6/6
12500000  6/6 6/6 6/6 6/6 6/6 6/6
```

Forty-two trials, no variance. The seventh write is the last thing on the bus,
so its tail is still inside the shifter with nothing to clock it out; issuing two
more writes decodes eight of nine, which confirms it is the shifter tail and not
framing.

**The fault was the anchor, not the arithmetic.** A frame was opened from the
START flag. That flag is prompt while the bits it precedes are not - a shifter
emits nothing until its word fills - so the distance between the flag and the
frame's real first bit varied with how many bits the *previous* frame stranded.
Worse, until the flag arrived no frame was open, so bits that did arrive were
discarded by the decoder and the frame opened however late the flag happened to
be. The symptom was a whole frame shifted left by one bit:

```
expect  S A13+ D40+ D11+ D22+ D33+ D44+ D55+
got     S A26+ D81- D23- D45- D67- D89- DAB-      0x26 == 0x13 << 1
```

**The previous frame's end is a far better anchor**, because it is known in the
decoder's own bit stream rather than in flag time: a frame closing on a unit
boundary means that boundary *is* its last bit. The wire gap after it was
measured directly from the raw samples - one clock edge, every rate, both
directions - so the next frame's first bit is a fixed, known distance away.
`frm.anchor_open` carries that: set once the gap has been skipped, it opens the
next frame on the very next bit whether or not a START flag has turned up. A
flag arriving late is then consumed rather than applied, since applying it would
reset the decoder mid-frame and undo the alignment just established.

**`SNIFF:GAP` now means what it says.** Its right value is **1**, the measured
wire gap, at every rate - and it is now the default. That the decoder used to
want 2 or 3 where the wire had 1 was the standing clue that something else was
wrong: the extra skip was padding for a variable delay, not for anything on the
bus. A constant that has to be tuned per rate is usually a symptom.

The acceptance suite now asserts exact frame *counts* - six reads of six and six
writes of six - rather than "the expected string appears somewhere", which was
the right test when two of six was the ceiling and would now pass a serious
regression without noticing.

#### What was tried first, and why each failed

Four approaches came before this one. They are recorded because each is a
plausible-looking idea that costs a day.

| approach | result |
|---|---|
| cancel a pending START when a STOP flag arrives | rejects all six setup falls, but a read's *turnaround* raises a STOP-shaped edge too, and it cancels real STARTs - 4 to 8 MHz stop decoding reads |
| hold a START 16 bus clocks and replay its buffered bits | correct frame *count* (13 of 13) but positions from the START flag, the wrong anchor |
| key the replay lead on the direction that just closed | circular - that direction is read back from a frame which, on the misaligned cases, is itself wrong |
| decoder-level alignment search over a buffered frame | ran clean, every frame found a candidate, and exact decodes did not move - the missing bit had already been discarded upstream by the gap skip, and no search over the bits that arrive can recover one dropped before them |

Two real faults were found along the way and are fixed independently of the
anchor: the idle STOP release pushed a raw `BB_EV_STOP` without telling the I3C
decoder, so the last frame of a burst was never closed; and the capture layer
cleared its own `in_frame` on every STOP it applied, including the read
turnarounds `dec_i3c_stop()` deliberately suppresses, which silently dropped one
STOP per read. `frm.in_frame` now follows `dec_i3c_in_frame()` instead of
assuming.

### Oversampled capture is still the exact low-rate mode

Exact frame after frame to 2.9 MHz on two taps at 10 MHz sampling, and at every
I2C speed on four taps at 5 MHz. Above about 4 MHz it runs out of samples per
bit, which is what pin-clocked mode exists for.

Measured through the whole `.cap` pipeline - capture, write the file, decode it
with the analyser - against known I3C traffic, two taps:

| bus | sample clock | samples/bit | decode |
|---|---|---|---|
| 1.07 MHz | 10 MHz | 9.4 | exact |
| 1.96 MHz | 10 MHz | 5.1 | exact |
| 6.06 MHz | 10 MHz | 1.7 | address only; payload wrong |
| 12.5 MHz | 10 MHz | 0.8 | address only; payload wrong |

The sample clock is 10 MHz at every row because the intake clamp sets it, and
asking for 20 or 40 MHz changes nothing. **6 and 12.5 MHz cannot go through this
pipeline oversampled**, and the reason is arithmetic rather than a defect: a
transition needs four or more samples to be resolved and there are 1.7 and 0.8.

### Fast rates: a reconstruction, with the timing marked invented

`scripts/pinclock_cap.py` writes a `.cap` for the rates oversampling cannot
reach. Pin-clocked capture is byte-exact at every rate to 12.5 MHz but records
no transition times at all - a sample is a bit - so the waveform is rebuilt from
the decoded events by laying them on a uniform bit period.

Every byte, every ninth bit, every START, repeated START and STOP is what the
bus carried. Every *timestamp* is manufactured. A frame the capture left
unterminated stays unterminated rather than being tidied up, and a dropped-event
marker draws nothing.

That distinction is the entire risk, because a reconstructed waveform is
indistinguishable from a recorded one by inspection, and any timing read off it
- clock rate, duty cycle, setup, jitter - returns the chosen bit period rather
than a property of the bus. So the file says what it is, in a `timing` field on
the session rather than in prose:

- `Timing::Synthetic { why, from, bit_ps }`, defaulting to `Measured` so every
  capture written before the field reads as what it is.
- The analyser prints a warning whenever it opens one - in `open()`, so every
  subcommand inherits it, not just the two that obviously draw time.
- `show` prints it again on **stdout** above the waveform, because that view is
  the one most likely to be redirected or pasted somewhere the stderr warning
  does not follow.
- `measure()` sets `timing_synthetic`, alongside `gap_overlap`, on every
  measurement taken from such a capture. Flagged rather than withheld: counting
  edges and comparing payloads over a reconstruction is still reasonable.

Verified by round trip - the analyser's decode of the reconstruction must equal
the firmware's own decode of the traffic that produced it, which the script
asserts rather than prints. Byte-exact at 6.06 MHz and 12.5 MHz.

### The suite now covers the file path, which it did not

`acceptance.py` gained a `sec_cap` section. Every other section checks the
decode the *firmware* produced, read back over `SNIFF:READ?`; none of them wrote
a capture file or read one. That is why the suite passed every run while
`SNIFF:RAW:READ?` was losing seventeen edges per call - the defect lived
entirely in a path the suite did not touch, and ALL CHECKS PASSED was true and
uninformative. Six checks:

- the drained edge list reproduces the firmware's own decode, which is the one
  that would have caught the transport bug on the day it appeared and needs no
  analyser to run;
- every edge the firmware declared is delivered, asserted directly rather than
  left for the decode to notice;
- a 1 MHz oversampled capture decodes byte-exact *from the file*;
- a recorded capture is **not** marked synthetic, so the mark means something;
- the 12.5 MHz reconstruction round-trips byte-exact;
- and it says its timing is invented, on both `decode`'s stderr and `show`'s
  stdout.

Checked for sensitivity rather than assumed: dropping a single edge from the
middle of a read makes the first check diverge, and the corrupted decode it
produces - `D45- D67- D89-` against `D22+ D33+ D44+` - is the same plausible
garbage the real bug produced. A missing analyser is reported as a failure
naming the path, not skipped.

`SNIFF:RAW:START` still forces oversampled mode. The edge list it produces is a
recording; this is a drawing, and they are kept apart deliberately.

### Fixed: the edge list the analyser sees is now the one the capture made

Saved `.cap` files decoded to plausible, wrong bytes for multi-frame sequences,
with extra framing events where bits were missing. None of it was framing: the
drain was losing seventeen edges per call to a truncated reply line, and the
analyser was decoding faithfully what it had been handed. See 5.3 - the method
that settled it is `scripts/edge_vs_decode.py`, which decodes the drained edge
list with the firmware's own rules and compares against `SNIFF:READ?` from the
same capture. Identical means the edges are whole; a divergence names the first
one lost.

Now byte-exact 10 of 10 at 1 MHz, where the same test had been 0 of 10. One
genuine analyser defect did come out of it once the data could be trusted: the
fall that sets up a STOP at the end of a read has the shape of a START, and was
being reported as a frame with no byte in it. The capture firmware had always
dropped it (`dec_i3c_stop`, `short-frame-dropped`); the analyser now applies the
same rule and records the suppressed frame as an event rather than a
transaction. The simulator could not produce that waveform, which is why the
golden corpus never caught it - `stop_from_high()` now generates it.

## 4.2 I3C1 has never been exercised - two wires, not the obvious ones

Everything measured here is port 2 (I3C2, AD_18/AD_19, Arduino A5/A4). Port 1 is
I3C1 on AON_16/AON_15, J3.12/J3.10 on the motor connector - a different
controller, in a different IOMUXC block, with a different pad-control layout.
Three independent things that can be wrong, so the firmware side was taken as
far as it goes without wires. `scripts/i3c1_bringup.py` runs it and reports
where it stopped.

**What is already proven, on the bench:**

- The firmware selects the port correctly. `I3C:PORT 1` muxes the AON pads and
  `I3C:PAD` reads back `0x0F` at `0x443C00B4`/`B0`.
- **The pad encoding is right, and asymmetric with port 2.** In the AON block
  `0x0F` is `SRE+DSE+PUE+PUS` - fast slew, high driver, and a genuine internal
  pull-up. In the main block the same constant has `PULL[3:2] = 11`, which is
  *No Pull*. So `I3C:PULL 1` really enables a pull-up on port 1 and does nothing
  of the kind on port 2, and anything that works on port 2 because of external
  pull-ups says nothing about port 1. Confirmed by reading the live register.
- **The J3 pads are wired and working** - RT's LPI2C2 shares them and finds the
  EEPROM at `0x50` over them, so J45 is already in the header position.

**And the trap.** That working J3 net cannot be reused. The target picks its
EEPROM bus with `default_bus()`, which deliberately takes the first bus that
does *not* share pins with I3C - so its EEPROM is on **I2C2, PB10/PB12**, while
its I3C target is on **PB6/PB7**, which is the net RT's A5/A4 already drive. The
J3 wires land on pins with no I3C function at all. Reusing them looks obviously
right and cannot work.

**The bench as it is**, read off the boards rather than assumed:

```
RT J3.12 / J3.10  (AON_16 / AON_15)  --  NUCLEO PB10 (D6) / PB12 (CN10-16)
                                         the target's I2C2, EEPROM at 0x50
RT A5 / A4        (AD_18 / AD_19)    --  NUCLEO PB6 (D15) / PB7 (D14)
                                         the target's I3C1
RT D14 / D15      (LPI2C3, bus 3)    --  not connected
```

**The two wires: move the NUCLEO end of the existing J3 pair.** Leave the RT end
alone.

| Wire | RT end (do not touch) | Move this end from | To |
|---|---|---|---|
| SCL | J3.12, `GPIO_AON_16` | NUCLEO PB10 / D6 | **NUCLEO PB6 / D15** |
| SDA | J3.10, `GPIO_AON_15` | NUCLEO PB12 / CN10-16 | **NUCLEO PB7 / D14** |

PB6/PB7 already carry the I3C bus that A5/A4 drive, so this joins an existing
net and its pull-ups. Both RT controllers are then on one bus, which is fine -
only one is claimed at a time and `I3C:FREE` releases the other's pads.

**Adding a second wire to J3.12/J3.10 instead would be wrong**, and this
document said to do it for one revision. The RT pad already has a wire on it
going to the target's I2C2; a second one to PB6 would short the target's I2C2
net to its I3C net, so driving either bus would drive both. There is one pad
pair and one wire each, so I3C1 and LPI2C2 are exclusive in the wiring exactly
as they are in the mux.

**What it costs while they are moved:** RT I2C bus 2 loses the EEPROM, so the
I2C sections of the acceptance suite fail. Either move the wires back
afterwards, or keep I2C alive on another bus - `EE:I2C 3` puts the target's
EEPROM on its I2C3, PA8/D7 and PC9/D10, which RT's unused bus 3 on D14/D15
could reach with two more wires.

Sniffing port 1 needs two further wires from the same net to taps `d4`/`d5`,
which are unallocated and on FLEXIO1 like `d8`/`d9`.

After that, `i3c1_bringup.py` should pass end to end, and port 1 can be folded
into the acceptance rate sweep - `i3c_setup()` already takes a port argument.

## 4.3 Controller-side trace - `self` and `both`, with the offset measured

`SNIFF:SRC self` records what our own I3C peripheral accepted and returned, as
it is driven. No taps, no FlexIO, no capture core - it works while the bus is
in use because it *is* the bus in use. `firmware/src/bb_trace.c` is the ring,
`bb_trace.h` is where the limits are written down, and the hooks are in the
register-level transfer paths in `bb_i3c.c`.

The tag plumbing for `observed=controller` had been complete for a long time
before anything generated a record to put it on, so selecting `self` used to
label FlexIO *wire samples* as controller-observed - which is precisely the
confusion the tag exists to prevent. That is why it refused rather than shipped.

### What a controller-side record means, and what it does not

| | |
|---|---|
| START / STOP | our own framing requests completing |
| address byte and its ACK | **observed** - MERRWARN.NACK is the only place on this board that knows whether a target answered |
| data byte on a read | **observed** - the peripheral took it off the wire |
| data byte on a write | it reached the transmit FIFO. Not that it was clocked out |
| the ninth bit of a data byte | **computed, never observed** |

That last row has teeth. On I3C SDR the ninth bit is a T-bit driven by the
transmitter - odd parity on a write, end-of-data on a read - and the controller
generates it in hardware on a write and consumes it on a read, exposing
neither. So it is worked out from the byte and its position, which means it
agrees with that byte by construction and **can never fail**. A host that
checked it against a wire T-bit and found them equal would have learned exactly
nothing. `SNIFF:READ?` says `tbits=computed addrack=observed` in its header and
`SNIFF:TRACE?` puts `~` after every derived token, so that host is told.

CCC and ENTDAA go through the SDK's blocking transfer, which does not expose
its bytes, so they emit `!` - the hole token the host library has always read
as "the stream is missing something here". A directed CCC read is two
transfers, so it is two holes. Emitting nothing would have been worse than
emitting a hole: a host merging this against a wire capture would find frames
on the wire with no controller record beside them and conclude the controller
never sent them.

### Measured, against the wire

The point of this source is that it can disagree with the bus, so the first
thing established was that it agrees when it should. A private write and a
six-byte read, 1 MHz, port 2:

| against | result |
|---|---|
| oversampled wire capture | **14 tokens, identical** |
| pin-clocked wire capture | wire is a clean prefix; the trace has two tokens more |

The pin-clocked difference is open item 7 - a shifter emits nothing until its
word fills, so the last frame of a window loses its tail. The trace has no
shifter, so it keeps `D55+ P`. Checked as a prefix rather than ignored: a known
disagreement whose shape is pinned is worth as much as an agreement, because if
it ever stops being a clean prefix something new is wrong.

### One real defect, found by pointing it at an address nobody answers

`I3C:W 0x55 AA` traced as `S AAA+` - address **acknowledged** - while the
command's own reply correctly said `no-ack-for-0x55`. The module raises
MCTRLDONE for the address phase and sets MERRWARN.NACK after it, and the frame
was being opened at MCTRLDONE.

This is the failure mode a controller-side source has and a wire capture does
not: not missing data, but a confident wrong answer, in the one record on this
side that is genuinely observed. Every frame that *worked* agreed in both
directions, so nothing else in the suite would have caught it.

The frame is now held until the first fact that settles it - a NACK appears, or
the module takes a byte. A NACK arriving *after* bytes have been accepted is a
different event, a write abort, and `A+` is correct there and left alone.
`I3C:W 0x55` and `I3C:R 0x55` now trace `S AAA- S AAB-`, and acceptance checks
it.

### `both` - the offset, measured

`both` runs the two engines over one window: the capture core samples the taps,
the controller records its own transfers, and each keeps its own true tag.
`SNIFF:READ?` gives the wire stream and `SNIFF:TRACE?` the controller stream.

It refused for a long time, and the reason was real. The trace stamps with
`bb_cycles()`, SysTick-derived on the CM33 at 240 MHz; the capture core counts
its own sample clock from the moment *it* started. Nothing related the two. A
merge on an assumed offset would interleave the streams wrongly while looking
entirely reasonable - the same confusion `observed=` exists to prevent,
arriving by a different door.

**Asking the capture core for its tick over the link cannot answer it.** The
round trip is tens of microseconds, so its own jitter swamps the figure; and
`cap.tick` is advanced by software draining shifters, so it lags real time by
however deep the queue is. Both errors are far larger than a bit time.

So the marker is physical. `SYS:SYNC` drives an edge on a pad wired to the taps
with `bb_cycles()` read either side of the store, and the capture engine
timestamps that same edge in its own ticks. One edge, two clocks, and the only
uncertainty is the width of that bracket - about 690 ns, reported rather than
averaged away.

#### What the fit says

`clock_offset.py`, 720 markers over 30.3 s:

| | |
|---|---|
| slope | 24.000000007 CM33 cycles per capture tick |
| nominal | 24.0 exactly (240 MHz / 10 MHz) |
| departure | **+0.3 ppb** |
| residual | 42 ns rms, 831 ns worst (one interrupted bracket) |
| drift | **under 55 ppb over 30 s** |

The slope is nominal to within a part in three billion, which is not two
clocks that happen to agree - it is two PLLs off one 24 MHz reference. **There
is no drift to track.** Only the intercept needs measuring, and it has to be
measured per capture because tick 0 is wherever that capture started.

A shorter 11.5 s run gives 29 ns rms and 90 ns worst, which is the figure to
quote for the alignment itself: the 831 ns above is one bracket that got
interrupted, and the bracket width is reported so such points can be seen.

#### The limit is not the clocks

That was the easy mistake waiting here, and the measurement that mattered is
`trace_lag.py`: how far behind the wire does the controller stamp its own
events? It runs both engines over one window and compares each frame's START
in the two streams.

The first answer was **27.9 us late with 19.2 us of spread** - nineteen bit
times at 1 MHz, 240 at 12.5. Not a clock problem at all. It was the cost of a
deliberate choice: the frame is held until its ACK is settled (4.3 above), so
the event was being stamped nine bit times after the START on a write and
seventeen on a read.

Both halves are gettable. `bb_trace_push_at()` carries a timestamp taken
earlier, and `bb_cycles()` is now read immediately before the MCTRL write that
emits the START. The ACK is still settled before the event is pushed; only the
time comes from the right moment.

| | mean | spread |
|---|---|---|
| stamped at push time | +27891 ns | 19184 ns |
| stamped at the MCTRL write | **-693 ns** | **692 ns** |

The mean is now *negative*, which is the right sign: the controller writes the
register a little before the bus moves. At 12.5 MHz it is -932 ns with 1276 ns
of spread.

#### What a merge can claim

`merge_both.py` runs it end to end - markers, both engines, one ordered stream
with provenance on every line. Six frames, 1 MHz:

```
  wire (us)    ctrl (us)    delta      frame
  0.0          -0.4         -0.40      write
  402.6        401.9        -0.73      read
  857.7        856.2        -1.47      write
  ...
  worst frame disagreement : 1467 ns
```

**Frames: yes, by a wide margin.** They are ~400 us apart and the error is
about 1.5 us.

**Events inside a frame: no.** A bit is 1000 ns at 1 MHz and 80 ns at 12.5, and
the error bar is larger than both. The tools say so rather than leaving it to
be discovered, and `merge_both.py` prints frame-level ordering only.

If the two ever disagree about how many frames happened, `merge_both.py` stops
and says so instead of pairing them up. That disagreement is the whole reason
the source exists; quietly aligning past it would throw away the finding.

### Commands

```
SNIFF:SRC wire|self|both    which engines run
SNIFF:READ? [n]             events - the wire's, or the trace's under `self`
SNIFF:TRACE? [n]            the trace with timestamps; `~` marks a derived flag
SNIFF:STAT?                 under `self` or `both`, a `# trace` line first
SYS:SYNC <signal> [n] [us]  markers both cores can timestamp
```

Two things that bite in that order. `SNIFF:RAW:START` restarts the capture
core, so it arms the trace too under `both` and must be the last thing to
touch the core - arming the trace afterwards with `SNIFF:START` discards every
marker laid down since, and the fit then comes out 30 ms wrong while looking
entirely ordinary. And `SNIFF:TRACE?` is a peek: it does not consume, so the
usual order is look there, then drain with `SNIFF:READ?`.

### Two bugs found while building this, both mine

**A stack overflow that looked like a USB fault.** `SNIFF:TRACE?` took the
board down at about fifty events - it would still enumerate and refuse to open
- and `bb_trace_peek()` was being asked for `done + take` entries into a
32-entry array, so it wrote past the end the moment `done` reached 32. The
symptom points nowhere near the cause, and the first fix written for it (one
buffered reply line instead of one bulk transfer per token) was aimed at the
wrong thing. That buffering is kept, because emitting hundreds of tiny
transfers against a `bb_usb_write()` that silently drops what it cannot hand
over is a hazard worth removing - but the comments say which of the two was
the diagnosis and which was precaution.

**A truncation marker that did not fit.** Once replies were buffered, a line
that filled exactly would drop its last token *and* the ` truncated=1` that
was supposed to report it. `line_reset()` now holds characters back for the
trailing note, because a marker that is itself the thing that does not fit is
the failure it exists to report, arriving through the report.

`scripts/self_trace.py` is the standalone bring-up; `sec_self()` in
`acceptance.py` is the same ground in the suite.

## 4.4 The capture self-tests cannot run in pin-clocked mode - now say so

`SNIFF:TEST` and `SNIFF:BENCH` drive one pin from the internal toggle generator
and measure what the capture engine makes of it. Pin-clocked capture spends
tap 0 on the bus clock and shifts tap 1 against it, so one driven pin gives the
shifters nothing to shift on.

Both now refuse in that mode rather than answering wrongly:

```
pinclock    TEST  -> ERR 4 test-needs-oversample-pinclock-has-no-bus-clock-to-shift-on
pinclock    BENCH -> ERR 4 bench-needs-oversample-pinclock-has-no-bus-clock-to-shift-on
oversample  TEST  -> OK driven=145 expected=144 seen=143 ...
oversample  BENCH -> OK cost_ns=100 max_bus_hz=2500000 ...
```

`SNIFF:BENCH` used to report `flexio-config-rejected`, naming the wrong
subsystem, and `SNIFF:TEST` used to return `seen=63` against 144 driven - which
reads like a capture fault rather than a mode that cannot answer the question.
Both are fixed; this entry stays because the underlying limitation is real and
the workaround is to use oversampled mode.

In oversampled mode both work, and the bench's own figure corroborates the
measured intake: `cost_ns=100 max_bus_hz=2500000` against ~2.9 MHz measured
exact.

## 4.5 SYS:DMACAP captures nothing

The remains of the GPIO-plus-eDMA capture experiment, measured at roughly a
sixth of FlexIO's rate and rejected on that basis. The channel arms - ERQ set,
ICR programmed for a request per edge - and completes no transfers, in any order
of pin and channel setup. Kept so the negative result stays reproducible; its
reply now says `note=retired-experiment-captures-nothing` so a zero is not read
as a measurement.

## 4.6 Protocol features not implemented

- **I2C clock stretching is untested.** This target does not stretch, so there
  is nothing on the bench to exercise it against.
- **HDR-DDR - the controller emits it; nothing on this bench can answer.**
  `I3C:DDRW` and `I3C:DDRR` drive HDR-DDR, and the entry sequence is verified on
  the wire by our own capture:

  ```
  SDR write   S A12- P
  DDR write   S AFC+ C20- DA2- D98-
  ```

  `AFC` is 0x7E shifted up - the broadcast address - and `C20` is CCC 0x20,
  ENTHDR0, "enter HDR mode 0", which is HDR-DDR. The words after it are the DDR
  frame, which the SDR decoder cannot frame and reports as whatever the bits
  happen to make. The transfer also produces a materially different waveform:
  89 SCL edges against 56 for the same payload in SDR.

  The controller path needed almost nothing - the peripheral's `MCTRL.TYPE`
  selects the message type and `i3c_write_typed()`/`i3c_read_typed()` already
  took it as a parameter, so these are the SDR commands with
  `kI3C_TypeI3CDdr` in place of `kI3C_TypeI3CSdr`.

  **The target cannot receive it, and that is structural rather than a matter of
  effort.** The STM32H5's I3C controller message types are RELEASE, HEADER,
  PRIVATE, DIRECT, LEGACY_I2C and CCC - there is no HDR-DDR type - and the only
  HDR thing its HAL exposes is `HAL_I3C_HDR_EXIT_PATTERN`, for escalating a
  fault. The target's own GETCAPS returns 0x00, advertising no HDR mode. No
  jumper or firmware change fixes that; it needs a different target device.

  So the acceptance suite checks what is checkable: that a DDR write goes out as
  broadcast + ENTHDR0, that an unanswered DDR read times out rather than hanging,
  and that the bus is still usable afterwards. Both pass. Verifying the data
  phase needs a target that implements the mode, and an HDR-DDR decoder in the
  sniffer - which is only worth building once there is something to point it at.
- **Hot-join - the controller answers it; the target's request is unreliable.**
  Six consecutive requests, the controller answered every one and the target was
  addressable after every one:

  ```
  seen=1..6   answered=1..6   last=assigned   read OK      6/6
  ```

  Three faults were fixed to get there, and the third is the important one.

  **Bus-initiated events were only serviced when a host asked.** `bb_i3c_poll()`
  was called from `I3C:IBI?` and `I3C:HJ?` and from nowhere else - not from the
  profile poll, not from the main loop. That works by accident for an in-band
  interrupt, because a host reads one right after causing it. It cannot work for
  a hot-join: the target sends its request and waits synchronously for an
  address while the controller sits idle until somebody types a command. Watched
  directly, the target reported `addr=none` the instant its request timed out
  and `addr=0x09` as soon as the next `I3C:HJ?` happened to run the poll. It is
  now called from `bb_profile_poll()`, and costs a listening check plus one
  register read when nothing is pending.

  **The IBI type was decoded against the wrong table.** `MSTATUS.IBITYPE` is
  `0b00 NONE, 0b01 IBI, 0b10 CR, 0b11 HJ`. The SDK's `i3c_ibi_type_t` is a
  *driver* enum with different numbering (`kI3C_IbiNormal` = 0,
  `kI3C_IbiHotJoin` = 1) and the raw field was compared against it, so every
  ordinary interrupt was labelled a hot-join - the counter read 108 on a bus
  that had never seen one. Named constants now carry the register's encoding.

  **A hot-join left the controller waiting for a byte that never comes.** The
  request must be issued with `IBIRESP = AckMandatory`, because the type is not
  knowable until the arbitration header has gone out, and a hot-join has no
  mandatory byte. The controller stalled in `kI3C_MasterStateIbiAck` - state 6,
  "waiting on IBI ACK/NACK decision" - and every transfer afterwards returned
  `entdaa-busy`, including the ENTDAA that would have answered the request. A
  plain ACK is now emitted once the type is known, and the frame closed
  regardless of the state it left behind.

  `I3C:HJ?` reports `seen`/`answered`/`last`; `I3C:HJAUTO 0` stops the
  controller answering, for a bus observer that wants to watch a hot-join
  without taking part. DAA is shared with `I3C:ENTDAA` through
  `i3c_daa_assign()` rather than reimplemented.

  **What is still wrong is on the target.** `HAL_I3C_Tgt_HotJoinReq()` is
  synchronous and its request is spent after one use: a second call without a
  re-init returns `HAL_I3C_ERROR_TIMEOUT` having put nothing on the wire. Three
  target-side fixes were measured and none is right - waiting on the HAL's own
  validity flag (never comes up), waiting on the DAUPD interrupt flag (better:
  two of four joined cleanly), and re-arming per call (uniformly worse, 0 of 6).
  The DAUPD wait is kept because it is honest; the re-arm is not. The target
  therefore reports a refusal for requests the controller answers correctly, and
  verifying the feature means reading `I3C:HJ?` and then addressing the target,
  which is what the measurement above does.

## 4.7 Bench instruments - pruned

Eight commands that existed only to sweep a value during debugging have been
removed:

```
I3C:SKEW  I3C:RXWIDE  I3C:RXGAP  I3C:RXTRIG  I3C:RXDEFER  I3C:RI2C
EE:I3CSLEW  EE:I3CLOG?
```

Kept, and why: `I3C:RXLOG?` and `I3C:TXLOG?` each found a fault that reasoning
had not. `I3C:PAD`, `I3C:HKEEP`, `I3C:DISTO` and `I3C:ODSTOP` map to real
silicon configuration a different bus may need. `EE:I3CCLK` selects the target's
clock source and the acceptance suite uses it.

**What the usage survey changed about the plan.** The first pass at this list
was a judgement about which knobs looked like scaffolding. Grepping the harness
for each one said something different: `I3C:SKEW`, `I3C:RXTRIG` and `I3C:RXDEFER`
were being set by the acceptance suite itself, which looked like a reason to
keep them. They were not - every one was being set to the value the firmware
already defaults to, as a guard against state left behind by a bench
experiment. Removing the commands removes the only thing that could leave that
state, so the guards went too and `i3c_setup()` is three lines shorter.

`I3C:RXGAP` and `I3C:RXWIDE` were referenced by nothing at all, in the firmware
or in any script.

The measured findings these knobs produced are in section 5 with their numbers.
The knobs themselves were the instrument, not the result, and a command table is
a user interface rather than a record of how something was debugged.

---

# 5. What was tried

## 5.1 Architectural alternatives — all measured, all rejected

| Idea | Result |
|---|---|
| Faster clocks | Already at silicon maximum: CM7 792 MHz, CM33 240, FlexIO 120 |
| GPIO edge to eDMA instead of FlexIO | **6x too slow.** Complete at 2 M edges/s, dropping at 4 M; 12.5 MHz needs 25 M |
| Split capture and decode across cores | **25%**, and backwards - decode would land on the slower, busier core |

The eDMA path *works* - `RGPIO_ICR.IRQC=0b0011` raises a per-pin request and the
channel copies `PDIR` - it is simply far too slow. Two incidental findings worth
keeping: **eDMA here will not self-trigger** (a channel linked to itself, and two
channels linked to each other, both sit with ACTIVE and DONE clear and no error),
and **it cannot write CM33 TCM**, which is where ordinary `.bss` lands.

**What actually moved the ceiling was a build flag.** The capture core was
compiled `-O0`: `CMakeLists.txt` scoped `-Og` to the bridge profile for ITCM fit
and excluded the capture core because it "has 255 KB of its own". Correct answer
to a question about size; nobody asked about speed.

| | -O0 | -O2 |
|---|---|---|
| 5 MHz/tap | 0/6, `DBE` decoded `D8E` | **6/6 exact** |
| 6 MHz/tap | 0/6, garbage framing | **6/6 exact** |
| 6 MHz I3C pin-clocked | 8-9 overruns | **0 overruns** |

Three architectural changes were measured and rejected before anyone looked at
how the code was compiled.

## 5.2 Fixes that did not work

- **Smaller pin-clocked words** to cure overruns. No effect: 8-bit and 32-bit
  gave identical counts despite a 4x difference in drain frequency, which proved
  the cost was per-sample decode rather than shifter draining.
- **The SDK's `daaBaudRate` parameter**, which exists for exactly the DAA rate
  problem and reprograms MCONFIG inside the call. 2/10. Changing rate through the
  path every other rate change uses is slower and works.
- **A 6 MHz cap for DAA.** Sound reasoning, wrong number: a cap is a *request*,
  and from the 25 MHz root a 12.5 MHz bus needs, 6 MHz resolves to 6.25 - the one
  rate already measured as marginal. 4/10. At 3 MHz it is 10/10.
- **`I3C:PPLOW`, higher roots, memory barriers, a register-level read** - all
  against the receive defect in 4.1. None moved it.
- **A framing guard counting consecutive decoder calls with SCL high.**
  Suppressed *all* framing including real STARTs, because in oversample mode the
  decoder is fed transitions rather than samples.
- **The same guard in capture ticks.** Correct in principle; does not recover the
  missed restart.
- **Gating a FlexIO timer's decrement from SCL.** Does not work on this part -
  the timer counts nothing. Gating its *enable* does, and is what the hardware
  framing uses now.
- **Raising the intake clamp from 20 M total to 15 MHz per tap**, to get faster
  buses through the oversampled `.cap` pipeline. The sweep looked convincing:
  with the clamp lifted, `SNIFF:TEST 500` gave zero overruns at 15 MHz/tap on
  both two and four taps, and 605/5519 overruns at 20 MHz - identical on the
  polled and eDMA intakes, which said the binding constraint was per channel and
  the `sample_hz * tap_count` model was an artifact of the old -O0 core. Four
  taps carried 60 M against a budget holding them to 20 M.

  Then it was measured against the bus instead of the generator, and a 2 MHz
  I3C capture went from exact at 10 MHz sampling to **wrong** at 15, with
  overruns at both; 1 MHz picked up overruns where it had none. More samples per
  bit and a worse decode is an intake that cannot keep up. The toggle generator
  drives one pin; a real bus moves two, with the controller driving them from
  this chip and USB alongside.

  The constant's own comment had already recorded this trap in its previous
  life - *"the conditions turned out to matter more than the number"* - and the
  sweep walked into it anyway. Reverted; both comments now say so. It would not
  have helped regardless: even at 15 MHz/tap a 6 MHz bus gets 2.5 samples per
  bit. **A clean overrun counter from a synthetic load is not evidence the rate
  is sustainable.** Check the decode.

## 5.3 Bugs found and fixed

**The reply line was silently truncated in transit, and the records it carried
were destroyed.** The counterpart to the `bb_reply_okf` bug below, at the other
end of the wire, and far more expensive.

`SNIFF:RAW:READ? 96` asks for ninety-six five-byte records. The firmware
formatted all ninety-six - 1442 characters of hex - because `RAW_MAX_RECS` was
`BB_MSG_MAX_PAYLOAD / RAW_REC_BYTES`, which is how many records fit in one
message *to the capture core* and has nothing to do with how many fit in the
answer *to the host*. The host library reads a reply line into a 1200-byte
buffer and kept the first 1199 characters without complaint, cutting the line
mid-record. The host parsed the 399 whole tokens that survived into 79 edges and
dropped the ruined one. Seventeen edges per call were gone - and unlike a short
read they could not be re-requested, because the command had already advanced
its cursor past them.

What made it cost days was the shape of the damage. The holes fell *inside* a
read transfer, so the edge list stayed well-formed: correct header, plausible
deltas, no gap marker. It decoded to `0x08 W 88 91 99 A2` where the bus had
carried `0x09 R 40 11 22 33 44 55`, with two extra framing events where the
missing bits had been. Everything pointed at the protocol analyser, which was
faithfully decoding exactly what it had been handed, and successive attempts to
fix its framing all failed for the same reason - there was nothing wrong with
it.

Found by `scripts/edge_vs_decode.py`, which settles the question by
construction. In oversampled mode `cap_consume()` feeds both paths from one
place - `dec_sample(st, tick)` then `push_sample(st, tick)`, same transitions,
same ticks, nothing between them - so a faithful transcription of `dec_sample()`
and `dec_i3c.c` run over the drained edge list *must* reproduce `SNIFF:READ?`.
It reproduced the write exactly and diverged on the read's first byte, which
said the edge list was already short before the analyser ever saw it. A chunk
sweep then put the cliff at exactly 80 records, and the reply at a hard 1199
characters however many were asked for.

Fixed in three places, because the defect needed all three to be harmless:

- `bb_winusb_read_line()` returns `BB_ERR_OVERFLOW` instead of a short line that
  looks complete. It reads on to the newline first - returning at the overflow
  would leave the tail to be read as the next line, desynchronising every reply
  after it.
- `BB_REPLY_BYTES_MAX` (399) bounds what `bb_reply_ok_bytes()` will send, and it
  reports rather than truncates. `RAW_MAX_RECS` and `SNIFF:RAW?` derive from it,
  so a command can no longer promise more than the line can carry.
- `capture.py` raises on a reply that is not a whole number of records instead of
  discarding the stub.

Afterwards: the mirror decode is identical to the firmware's, 3 runs of 3 at
1 MHz; `.cap` captures decode byte-exact 10 times out of 10, where the same test
had been 0 of 10; acceptance unchanged.

The lesson is the one in 5.5, in its sharpest form yet: **a wrong value out of a
decoder is not evidence of a decoder bug.** It is also worth noting that the fix
to `bb_reply_okf` below ended with "worth checking any other `vsnprintf` in the
tree against the same question" - the right instinct, aimed one function too
narrowly. The question is not about `vsnprintf`. It is: *can this path lose data
without saying so?* `bb_reply_ok_bytes` streams, so it looked immune; the limit
was at the far end, in another repository.

**Long replies were silently truncated.** `bb_reply_okf()` formatted into a
256-byte buffer, and `vsnprintf` truncates to fit without reporting it. Nothing
in the path checked the return value, so every layer believed the reply had been
formatted and sent. `SNIFF:STAT?` had grown past 255 characters one counter at a
time, and its newest tokens were simply absent from the string - a measurement
reading them got "field not present", which is indistinguishable from "the
firmware does not have this field". The tell was that a token that had worked
earlier in the session stopped appearing without anyone touching it.

Fixed by sizing the buffer to the transport's own limit (`BB_MSG_MAX_PAYLOAD`,
480 - a line that would not fit here would not fit on the wire either) and by
appending `...TRUNC` when the format still overflows. The second half matters
more than the first: the bug was never the size, it was that overflowing was
indistinguishable from succeeding. Worth checking any other `vsnprintf` in the
tree against the same question.

**The I3C pads were open-drain.** `i3c_claim_pads()` asked for `BB_PAD_I2C`
whenever the internal pull-up was wanted, and that mode sets ODE. I3C needs the
pull-up for its open-drain address header, so the choice looked reasonable; but
SDR data is push-pull, and an ODE pad cannot drive it. Found by sweeping the pad
register raw, 32 values, at a rate that was failing - the twelve that worked
were exactly those with drive or slew set and ODE clear, and the shipping
encoding was not among them. Reasoning from the field map would not have found
it: this part has two SW_PAD_CTL layouts and the project had already applied the
wrong one once.

**The target split write frames.** `bb_ee_i3c_poll()` accumulated bytes into a
buffer and interpreted them whenever the write counter moved - two independent
reads of state the event handler updates in between. A pass that caught only the
first byte before seeing the counter move applied that byte alone, dropped it as
short, and left the rest to be prepended to the next frame. `00 40` became `00`
then `40,00,40`, so a pointer write set the pointer to 0 and stored 0x40 as data
there. About 7% of writes at 12.5 MHz. Fixed by recording the frame boundary in
the receive ring at the moment the frame ends, so a frame is either wholly
available or not available.

**Every SDK I3C wait was unbounded.** `I3C_RETRY_TIMES` was never defined, so
`while (txCount < byteCounts)` and its siblings had no exit. A target that
stopped clocking hung the CM33 with no fault and no timeout, recoverable only by
reflashing - which is why early trials this round cost a programming cycle each.
The codebase had already made this argument for LPI2C and LPUART; I3C was
missed.

**`I3C:ROOT` could stall the peripheral.** 400 MHz into the I3C functional clock
stops the board answering USB at all. Now refused above 100 MHz, and the reply
reports whether the configured rates still fit the chosen root.


**In the RT1186 port:**

| Bug | Symptom it produced |
|---|---|
| Tap pads loaded the bus from reset | I3C only worked *while the sniffer ran* |
| Pad encoding wrong for every `GPIO_AD` pin (`PULL[3:2]` vs `PUE`/`PUS`, split by IOMUXC instance not address range) | `I3C:PULL 1` reported success and configured nothing |
| No recovery after a failed I3C transfer | First failure honest, every one after it `busy` for ever |
| I3C pads never re-claimed per operation | Transfers read their own stale GPIO level |
| SDK device list is append-only | One target counted 1, 2, 3 … 8 |
| `I3C:FREQ?` echoed the request | Six requested rates reported while the bus ran at two |
| `SNIFF:PINS?` always said `levels=00` | Taps parked on a FlexIO ALT; RGPIO cannot read that |
| Every NAK blamed `0x7E` | A read NAKed at 0x09 pointed at dynamic addressing |
| Framing qualified in software | Phantom restarts above 400 kHz |
| Capture core compiled `-O0` | Half the sampling ceiling |
| CCC formatter used `ev->value` (always `ev[0]`, a START) | Every command code decoded `0x00` |
| ENTDAA payload read nine bits per byte | PID/BCR/DCR plausible and wrong |
| ODHPP left at the SDK default | No I3C above ~6 MHz |

**In the NUCLEO's firmware** (four files):

| Bug | Symptom |
|---|---|
| DMA progress counted as a completed read | Permanent abort/re-arm loop; `reads=29765` on an idle bus |
| `BUSY_TX` excluded from the target event handler | Every write to an armed target silently dropped |
| Re-arm fired a full `HAL_I3C_Abort_IT` + 1 ms wait | Target deaf during it; next write lost |
| Read completion no longer reached the HAL | Window never rebuilt; later reads answered from the first one's pointer |
| DMA abort only in the armed case | Second read returned the *tail* of the previous window |

## 5.4 Test-harness artifacts that produced false conclusions

**The I2C test wrote to the wrong address for the whole project.** The target's
array is 1 KiB and its address is two bytes wide. Every run of the I2C
regression used `I2C:MEMW 0x50 0 1 DE AD BE EF` - a one-byte address - so `DE`
was consumed as the low address byte and `AD BE EF` was stored at 0xDE. The wire
decode matched the expected string exactly and the test passed 5/5, for a
transfer that wrote nothing where it claimed to.

It survived because the test checked the decode and never the data. Reading the
array back afterwards - over the target's own USB link, not over the bus under
test - is one line and would have caught it at any point. Every "I2C is
complete" claim before this rested on the sniffer agreeing with the controller
about a transfer both had wrong.

**A second ENTDAA finds nothing, and that is correct.** Repeating ENTDAA without
putting the target back reports `targets=0` and reads as a failure; the target
already has its address. Cost a run that showed 1/8 on a bus that was working
perfectly.


Four of these were written into documentation before being caught.

- **The rate sweep ran `I3C:ENTDAA` twice.** The first assigns an address; the
  second finds nothing, because an addressed target no longer participates in
  DAA. Reported as "2, 3 and 4 MHz fail" and "the failures are non-monotonic".
  Both false. It also made ODHPP look like it had no effect, which delayed the
  real fix by a long way.
- **The query helper broke on a partial line**, truncating event tokens. Made
  *working* I2C sniffing look completely broken.
- **`SNIFF:READ?` called before `SNIFF:STOP`** returned `n=0`, read as "the
  controller never transmitted". It had.
- **`levels=00`** believed as evidence the bus was stuck low. The diagnostic read
  RGPIO for pads muxed to FlexIO and always returned zero.
- **A grep matched the wrong line**, turning a failing 6 MHz read into "read=OK".
- **`reload.ps1` reports "board up"** from the first VID_1209 device it finds -
  the NUCLEO - whether or not the RT1186 is running. Use
  `scratchpad/rt-reload.sh`, which checks for PID_0003.

**A stale IBI queue read as a broken IBI path.** The controller keeps a
four-deep IBI queue that survives between commands and between runs. The
acceptance suite primed it with a single `I3C:IBI?`, which clears one entry - so
anything left behind by earlier work made every reply in that section lag by
that many, and it reported `0/8 ibi` on a bus that was working perfectly. Manual
IBIs at the same moment returned the right payloads on the first read, which is
what showed it was the queue and not a race. The suite now drains until the
queue answers `none`. Worth remembering as a shape: a test that depends on
shared device state being empty will pass until the first time somebody uses
that device for something else.

## 5.5 The pattern worth carrying forward

Eleven of the bugs above presented as hardware or wiring faults. None was.

Three times a symptom was misdiagnosed, a workaround was built on the
misdiagnosis, and the workaround then had to be removed once the real cause
appeared. A "turnaround suppression" rule was added to explain a command code
decoding as `0x00`; the real cause was one character in the formatter.

Two rules earned the hard way:

- **A wrong value out of a decoder is not evidence of a decoder bug.** The output
  path can lie just as easily and is cheaper to check. Charged twice now: once
  for a command code that read as `0x00` because of one character in a
  formatter, and once for a capture that decoded to wrong bytes because the
  transport was cutting its reply line. The second cost days and three failed
  attempts to fix framing that was never broken.
- **When two implementations disagree, make them decode the same input.** The
  firmware and the analyser were each argued about on their own for a long time.
  One script that ran both over the same edges - `edge_vs_decode.py` - located
  the fault in a single run, because "identical" and "diverges at event 6" are
  answers, where "the analyser looks wrong" is a hypothesis.
- **A negative result can outlive the bug that produced it.** ODHPP was recorded
  as "no effect" while a broken harness was masking it, and that note then sat in
  two documents arguing against re-testing the thing that turned out to be the
  whole problem.

---

# 6. Commands added

```
I3C:CLK?               root, requested/actual pp and od, ppbaud, odbaud,
                       i2cbaud, odhpp, raw MCONFIG
I3C:ROOT <hz>          choose the I3C functional clock (OscRc400M + divider)
I3C:ODHPP [0|1]        drive the open-drain SCL high pulse push-pull, or not
I3C:PPLOW [n]          stretch the push-pull SCL low period
I3C:SKEW [n]           MCONFIG.SKEW
I3C:RXWIDE [0|1]       read the receive FIFO through MRDATAH instead of MRDATAB
I3C:RXGAP [n]          NOPs between receive-FIFO pops
I3C:RXTRIG [n]         MDATACTRL.RXTRIG, the receive FIFO trigger level
I3C:RXDEFER [0|1]      drain the receive FIFO only after the frame completes -
                       the measurement that took software out of the loop
I3C:RXLOG?             the receive timeline of the last read: count before
                       START, per-pop value/RXCOUNT/spin, and what the frame
                       left behind after COMPLETE
I3C:TXLOG?             the same for the last write, including which wait it
                       stopped in
I3C:RI2C <addr> <n>    the same read issued as a legacy I2C transfer, to tell
                       an SDR framing fault from the shared receive path
EE:I3CSLEW [0..3]      (target) the output speed of the target's I3C pads
I3C:TRACE?             controller registers frozen at the moment a transfer failed
I3C:DAADBG             dynamic address assignment driven straight from registers
SNIFF:DECODE [0|1]     capture samples without decoding - a measurement tool
SNIFF:STAT? core_hz    the capture core's own clock, asked on that core
SYS:PINS? .../cfg=     the live SW_PAD_CTL value per pin
SYS:GPIO <sig> <0|1>   hold one named pin at a level
SYS:PULSE <sig> [n]    drive n low/high pairs on one named pin
SYS:DMACAP [ARM [pin]] the GPIO-to-eDMA experiment (prototype, negative result)
```

`SYS:GPIO` and `SYS:PULSE` are the ones to remember for bench work: diagnosing
wiring from transfer behaviour is ambiguous because a transfer moves both lines
at once. One named pin, one peripheral-free toggle, a known edge count, watched
on one tap, settles it in a round.

---

# 7. Operating notes

- **Reads must run at 1 MHz** until 4.1 is fixed. Higher rates return the first
  byte repeated, or time out.
- **ODHPP stays off.** It is the default now; turning it on caps the bus at
  ~6 MHz.
- **Dynamic address assignment is capped internally** and restores the configured
  rate. Nothing to do at the host.
- **Use `rt-reload.sh`**, not `reload.ps1`, to confirm the RT1186 actually came
  back.

---

# 8. Where the detail lives

- `E:\BusBridge-RT\docs\PROGRESS.md` - full narrative, ~2900 lines
- `E:\BusBridge-RT\docs\SPEC.md` - command surface
- `E:\BusBridge-RT\docs\PINS.md` - pin map and the three jumpers (J31, J35, J45)
- `G:\RT1186_Full_Rate_I3C_Controller_Sniffer_Plan_v2.md` - the plan, revised
  against hardware

The bench tools, and which question each answers:

| Script | Question |
|---|---|
| `scripts\acceptance.py` | Does all of it still work? 66 checks, including the file path and both capture sources. |
| `scripts\edge_vs_decode.py` | Is the edge list the host received the one the capture made? Decodes it with the firmware's own rules and compares. |
| `scripts\cap_rates.py` | Which bus rates survive the `.cap` pipeline, and what sample clock did the engine actually settle on? |
| `scripts\pinclock_cap.py` | A capture file for rates too fast to oversample, with the timing marked invented and the round trip asserted. |
| `scripts\i3c1_bringup.py` | Is I3C1 alive? Takes port 1 as far as it goes without wires, and names what is missing. |
| `scripts\self_trace.py` | Does the controller-side trace say the same thing as the wire - and does it admit what it cannot see? |
| `scripts\clock_offset.py` | Where does the capture core's tick 0 sit in CM33 time, and do the two clocks drift apart? |
| `scripts\trace_lag.py` | How far behind the wire does the controller stamp its own events? The number that bounds a merge. |
| `scripts\merge_both.py` | A `SNIFF:SRC both` capture merged into one ordered stream, with its error bar stated. |
| `scripts\bb.py` | Board access by serial. Restores the `OK ` prefix the C library strips. |

`acceptance.py` now reaches across repositories: `sec_cap` needs the analyser
built at `E:\STMicro\sniffer\target\release\sniffer.exe`, overridable with
`SNIFFER_EXE`. That is the honest shape for a system whose two halves have to
agree - the reply-line bug was a disagreement between exactly those halves - but
it means the suite cannot be run to completion from the firmware tree alone.
