# BUSBRIDGE-RT

A USB bus controller and protocol sniffer on the NXP FRDM-IMXRT1186, built so
that the two jobs do not fight each other: the Cortex-M33 drives the buses and
the USB link, and the Cortex-M7 does nothing but capture.

That split is the whole design. The STM32 version this was ported from decoded
in a GPIO edge interrupt, and all three of its limits came from that one
choice — about 450 ns per bus clock, a ceiling near 1 MHz, and native I3C bits
read a position late because the handler sampled SDA after SCL had already
fallen. Capture here is hardware, on the other core, clocked by the bus's own
clock line.

```
FRDM-IMXRT1186   CM33 @ 240 MHz  controller + USB      VID_1209 PID_0003
                 CM7  @ 792 MHz  FlexIO capture engine
NUCLEO-H533RE    I2C and I3C target under test         VID_1209 PID_0001
```

## What is measured

Everything below was taken off hardware in this configuration. Nothing here is
inferred from reading the code.

- **I3C complete at a true 12.5 MHz**, in-band interrupts included. ENTDAA,
  broadcast CCC, directed CCC, private write, private read and IBI all pass;
  2000 consecutive write/read pairs at 12.5 MHz return 2000/2000 exact.
- **I2C complete**, 7-bit and 10-bit, verified against the target's own array
  rather than only against the wire — which is what caught a test that had been
  writing to the wrong address for the life of the project.
- **Pin-clocked capture lossless and byte-exact at every rate to 12.5 MHz**,
  zero overruns, with an eDMA intake that takes the shifter drain off the CPU.
- **Oversampled capture exact below about 3 MHz**, where it also sees framing,
  idle time and glitches that a pin-clocked capture cannot.
- **Three capture sources.** `wire` samples the taps. `self` records what our
  own controller accepted and returned, with no taps at all. `both` runs the
  two over one window, each keeping its own provenance tag.
- `scripts/acceptance.py` — **66 checks**, all passing.

### What is not

SPI, UART, MDIO and CAN have command surfaces in the bridge profile, carried
over from the STM32 build. Nothing on this bench exercises them, so they are
not in the acceptance suite and this README does not claim them.

HDR-DDR entry is verified on the wire but no target here answers in DDR, so the
data phase has never been observed. `docs/STATUS.md` §4 lists the eight open
items with what each is actually blocked on.

## Provenance is a first-class field

A capture record says where it came from, and the distinction survives to the
decoder. `wire` means FlexIO latched the line at a clock edge. `controller`
means our own peripheral's FIFO said so — true, useful, cheap, and **not the
same thing**. They differ exactly when something is wrong, which is when a
sniffer earns its keep.

The same rule runs through the rest of it. A controller-side T-bit is computed
from the byte it belongs to, so it agrees by construction and can never fail —
the reply says `tbits=computed` so nobody mistakes that for a passing check. A
frame the trace cannot describe emits a hole rather than nothing. A capture
file reconstructed from pin-clocked data carries `Timing::Synthetic` and warns
at every exit, because that mode records no transition times and the timing in
such a file is invented.

## The bench console

`gui/` is a window for this board: the capture engine is what it is built
around, not a tab bolted onto something else.

```powershell
cd gui
cargo build --release
target
eleaseusbridge-rt-gui.exe
```

Five tabs, in the order the instrument is used - pick a board, set the capture
up and run it, look at where the records came from, write or open a file, run
the bench measurements - with a console underneath all of them, because every
control is a command line and hiding that from someone debugging their own
firmware helps nobody.

It talks to both boards. An STM32H5 answers everything in the console; the
capture tabs are shown **disabled with the reason** rather than hidden, because
a window that silently looks different on two boards teaches nobody why.

```powershell
target
eleaseusbridge-rt-gui.exe --smoke 975a6193
```

The Capture tab draws a **live waveform** from the same capture, off the raw
edge list. Two things about it are worth knowing before reading one:

- In oversampled mode the axis is time, and the gap between two edges is a
  measurement. In pin-clocked mode a tick **is a bus clock**, so the spacing
  says what the bus did and not when - the panel labels the axis differently
  and says so rather than leaving it to be inferred.
- Pin-clocked capture skips the raw sample store while it decodes, which is
  what bought it 6 to 8 MHz. The waveform and the decoder cannot both run in
  that mode, so the checkbox refuses the combination instead of drawing an
  empty trace that looks like a dead bus.

`--smoke` drives the window's own code paths with no window at all: the real
worker, the real configuration ordering, the real reply parsing, against real
hardware. A GUI is the one part of this project nothing else can check - the
firmware has `acceptance.py`, the host library has `unit_test.c`, the analyser
has its golden corpus, and a window has a person clicking, which is not a test
and does not run twice the same way. It needs both boards, as the suite does.

It links the host C library statically from the STM32H5 tree; point
`BUSBRIDGE_LIB_DIR` elsewhere for a checkout somewhere else.

## Building and running

Windows, PowerShell, with the MCUXpresso SDK and an arm-none-eabi toolchain.

```powershell
scripts\build.ps1                 # bridge image, CM33
scripts\build.ps1 -Profile capture # capture image, CM7
scripts\build.ps1 -Pair            # bootable two-core image (XIP)
```

```powershell
scripts\reload.ps1 -Build          # build the pair, program it, restart into it
scripts\flash.ps1                  # load the bridge into TCM and run
```

`reload.ps1` is the one that removes the power cycles: an XIP image has to be
entered by the boot ROM, and a debug reset does not redo that.

```powershell
python scripts\acceptance.py       # the whole suite, both boards attached
```

## The bench tools, and the question each answers

| script | question |
|---|---|
| `acceptance.py` | Does all of it still work? 66 checks, including the file path and both capture sources. |
| `self_trace.py` | Does the controller-side trace say the same thing as the wire — and does it admit what it cannot see? |
| `clock_offset.py` | Where does the capture core's tick 0 sit in CM33 time, and do the two clocks drift apart? |
| `trace_lag.py` | How far behind the wire does the controller stamp its own events? The number that bounds a merge. |
| `merge_both.py` | A `SNIFF:SRC both` capture merged into one ordered stream, with its error bar stated. |
| `edge_vs_decode.py` | Is the edge list the host received the one the capture made? |
| `cap_rates.py` | Which bus rates survive the `.cap` pipeline, and what sample clock did the engine settle on? |
| `pinclock_cap.py` | A capture file for rates too fast to oversample, timing marked invented and the round trip asserted. |
| `i3c1_bringup.py` | Is I3C1 alive? Takes port 1 as far as it goes without wires, and names what is missing. |

## Layout

```
firmware/profiles/bridge/   CM33: the bus controller and the SNIFF command surface
firmware/cm7/               CM7: the capture engine and the on-device decoders
firmware/shared/            the record format and the CM33 <-> CM7 protocol
firmware/src/               board layer, USB, command parser, controller trace
docs/STATUS.md              the entry point: what works, what is open, and why
docs/SPEC.md                the command surface
docs/PINS.md                pin map and the three jumpers
docs/PROGRESS.md            the full narrative, including the wrong turns
scripts/                    build, flash and the bench tools above
```

## The other two repositories

- [busbridge-stm32h5](https://github.com/warunafernando/busbridge-stm32h5) —
  the original STM32H5 build this was ported from. It is both the reference and
  the I2C/I3C target this is tested against, and it holds the Windows host
  library the scripts talk through.
- [busbridge-sniffer](https://github.com/warunafernando/busbridge-sniffer) —
  the Rust waveform analyser that decodes the `.cap` files. `acceptance.py`
  reaches across to it: the `.cap` section needs
  `sniffer.exe` built there, overridable with `SNIFFER_EXE`. That is the honest
  shape for a system whose two halves have to agree — a disagreement between
  exactly those halves was the worst bug this project had — but it does mean
  the suite cannot be run to completion from this tree alone.

## A note on the documentation

`docs/STATUS.md` and `docs/PROGRESS.md` record the measurements behind each
decision, including the ones that were wrong first. A reply line silently
truncated by 243 characters sent weeks of work at the protocol analyser, which
was faithfully decoding exactly what it had been given; an intake clamp was
raised on evidence from a toggle generator and made real captures worse. Those
are written down with what was measured and why the conclusion was wrong,
because that is the part which is expensive to rediscover.

## Licence

MIT. See [LICENSE](LICENSE).
