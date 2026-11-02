# RT1186 I3C controller: receive FIFO over-reception

**Status: RESOLVED - not a silicon defect. Kept for the method, not the
conclusion.** The cause was local: the controller's own pads were configured
open-drain (`BB_PAD_I2C`, which sets ODE) while I3C SDR data is push-pull, so
every rising edge in the payload was made by a ~40 kOhm internal pull-up instead
of the driver. A dedicated `BB_PAD_I3C` mode - pull-up, high drive, fast slew,
ODE clear - gives 5/5 exact reads with zero extra FIFO entries at every rate
from 1 MHz to 12.5 MHz. **Do not send this to NXP.**

The measurements below are still accurate as a description of what the fault
looked like, and sections 2 to 4 are worth keeping: they are how a symptom that
had been called "the first byte repeated" for months turned out to be
over-reception, which is what made the real cause findable.

**Date:** 13 September 2026

---

## 1. One-line statement

> On a private SDR read, the I3C controller's receive FIFO gains **more entries
> than there are data bytes on the wire**. The extra entries are duplicates of
> bytes that were genuinely received, in order. The bus engine itself is
> correct: it clocks exactly the requested number of bytes and terminates on
> RDTERM.

Every previous description of this defect - including two documents in this
repository - called it "the first byte repeated". That was wrong, and it was
wrong because the FIFO was never drained past the requested length. The bytes
after the first are not corrupt; they are the start of a longer, duplicated
sequence.

## 2. The measurement that shows it

Six-byte private read of `40 11 22 33 44 55` at 6.06 MHz push-pull.

```text
requested            6 bytes
wire (FlexIO tap)    S A13+ D40+ D11+ D22+ D33+ D44+ D55+     <- 6 bytes, T bits correct
popped (6 entries)   40 40 11 11 22 22
still in FIFO        33 44 44 55
total pushed         10 entries
```

`I3C:RXLOG?` for the same transfer:

```text
pre=0 n=6 type=0 post=4 extra=4 tail=33,44,44,55
  0:val=40,rxcount=2,spin=1   1:val=40,rxcount=3,spin=0
  2:val=11,rxcount=2,spin=0   3:val=11,rxcount=3,spin=0
  4:val=22,rxcount=2,spin=0   5:val=22,rxcount=1,spin=0
```

`pre=0` answers the obvious question: the FIFO was empty before START.

## 3. It is not software

With `I3C:RXDEFER 1` the CPU touches nothing until `MSTATUS.COMPLETE` is set.
The FIFO then already holds **ten** entries at the first pop, in the same order:

```text
0:val=40,rxcount=10,spin=0  1:val=40,rxcount=9,spin=0  2:val=11,rxcount=8,spin=0 ...
```

So the duplication happens on the push side with no concurrent reads at all.
That removes, in one measurement, the whole class of explanations previously
swept one knob at a time: pop spacing, memory barriers, FIFO trigger level,
byte versus half-word read port, RDTERM armed atomically with START, PPLOW,
SKEW, and the root clock.

## 4. It is not misframing

If the receiver were slipping bit boundaries - counting 8 where the wire has 9
(data plus T bit) - the FIFO would hold bytes assembled from the right bits at
the wrong offsets. Taking the real bit stream for `40 11 22 33 44 55` with
correct odd-parity T bits and re-framing it at every offset for both 8- and
9-bit strides reproduces no prefix of `40 40 11 11 22 22`. The bytes arrive
intact and in order; some arrive twice.

## 5. What does move it

Two levers, and both are electrical rather than protocol.

| lever | effect |
|---|---|
| Target pad output slew (STM32H5 `GPIO_SPEED_FREQ_*`) | VERY_HIGH duplicates at 6 MHz; HIGH and MEDIUM give `extra=0` and an exact read |
| Controller internal pull-up (`I3C:PULL`) | On: writes stall at 2-4 MHz. Off: 2-4 MHz is exact. At 6 MHz it inverts - on is exact, off duplicates |

A working point that moves with bus rate, and inverts between two RC-changing
controls, is the signature of a signal-integrity optimum rather than a logic
fault. This bench has no proper I3C termination: jumper wires, internal
pull-ups, no series damping.

That matches the public NXP Community report for the LPC860, where a controller
returned `AA AA BB BB` for `AA BB CC DD` with the wire verified correct on a
scope, and the behaviour changed when a probe's capacitance was added to SCL.

## 6. Working configuration on this bench

| push-pull | internal pull-up | target slew | ENTDAA | CCC | write | read |
|---|---|---|---|---|---|---|
| 1.07 MHz | on  | HIGH | pass | pass | pass | pass |
| 1.96 MHz | off | HIGH | pass | pass | pass | pass |
| 4.00 MHz | off | HIGH | pass | pass | pass | pass |
| 6.06 MHz | on  | HIGH | pass | pass | pass | pass |
| 8.00 MHz | either | any | pass | pass | pass | **fail** |
| 12.5 MHz | either | any | pass | pass | pass | **fail** |

At 12.5 MHz the read returns `MERRWARN=0x00100000` - TIMEOUT, "the module has
stalled too long in a frame" - at every open-drain rate from 100 kHz to 1 MHz.

## 7. What the controller now does about it

A read that over-receives returns

```text
ERR 4 read-fifo-over-received-4-extra-entries-see-I3C:RXLOG?
```

rather than `OK` and six plausible-looking wrong bytes. Silently returning
corrupt data is the worst failure mode available to a bus tool, and it is what
this defect produced for as long as it went unrecognised.

## 8. Minimal reproduction for NXP

```text
board     FRDM-MIMXRT1186
target    one I3C target, dynamic address 0x09
transfer  one 6-byte private SDR read, polling, no DMA, no USB
```

Show, for the same target data and the same wire:

```text
1.07 MHz   MRDATAB x6 = 40 11 22 33 44 55   RXCOUNT after COMPLETE = 0
6.06 MHz   MRDATAB x6 = 40 40 11 11 22 22   RXCOUNT after COMPLETE = 4
```

and the accompanying `MCONFIG`, `MCTRL`, `MSTATUS`, `MERRWARN`, `MDATACTRL`,
root clock, PPBAUD, ODBAUD and RDTERM.

The question to ask is narrow:

> Under what input conditions can the I3C controller receive path push a byte
> into the RX FIFO more than once, given that the bus engine clocks exactly
> RDTERM bytes and the wire data is verified correct?

RT1180 Rev. 5 errata (June 2026) lists ERR052086, ERR052343 and ERR052344 for
I3C. None describes this: ERR052086 is a DMA last-byte issue, and this is
polling with more entries than bytes.

## 9. Not yet done - needs hands on the bench

- oscilloscope at the controller's own SCL and SDA pads, at 1 MHz (passing) and
  6 MHz with VERY_HIGH slew (failing), looking for multiple threshold crossings
- 22/33/47/68 ohm series damping, target-driven SDA first
- shortest-wire rebuild with paired grounds
- I3C1 versus I3C2: port 1 is AON_16/AON_15 and port 2 is AD_18/AD_19, so the
  A/B needs two jumpers moved
- a second, independent I3C target
