# Build progress

Phase plan is in `SPEC.md` section 7. **Nothing here has been on hardware** — by
agreement, the board is tested at the end. Every claim below is "compiles and
links", which is the only claim that can honestly be made yet.

## Images

```
scripts\build.ps1 -Profile bridge       # CM33: the bus controller
scripts\build.ps1 -Profile peripheral   # CM33: the device under test
scripts\build.ps1 -Profile capture      # CM7:  the sniffer's capture engine
scripts\build.ps1 -Profile bridge -Memory flexspi_nor_debug   # bootable XIP
```

| Image | Core | text | data | of region |
|---|---|---|---|---|
| bridge | CM33 | 121,152 | 16,240 | **93%** of 127 KB TCM |
| peripheral | CM33 | 77,632 | 5,736 | 60% |
| capture | CM7 | 29,136 | 198,008 | 76% of 256 KB DTCM (the ring) |
| bridge, XIP | CM33 | 120,416 | — | 1.4% of 8 MB QSPI NOR |

**102 commands** in the bridge image, across `SYS: I2C: I3C: SPI: MDIO: UART:
CAN: SNIFF: NET:`. The bridge is at **93% of CM33 TCM** after the USB stack went
in — close enough that the next addition may not fit. The XIP variant is at 1.4%
of 8 MB QSPI NOR, so a bigger controller image has somewhere to go.

---

## Phase 0 — skeleton — DONE

`bb_cmd.c`, `bb_io.c` and `main.c` ported **unchanged** except `SYS:CLK?`, on the
strength of a ~140-line `bb_hal.h` shim (status enum, tick, NVIC spellings, an
i.MX-shaped pin type). New: board layer, LPUART console, the board directory.

**Board headers are `bb_board_def.h` / `bb_pinmap.inc`**, not `board.h` /
`pinmap.inc`: the SDK ships its own `board.h` for this board and two files of
that name on one include path is a coin flip resolved by search order.

**The board is chosen by include path, not a `-D` path macro.** The first
attempt passed `-DBB_BOARD_HEADER="…/board.h"`; a quoted path through CMake into
ninja into the compiler is at the mercy of whose quoting wins, and it silently
produced an image where *none* of the board macros existed. Include path cannot
be got wrong that way, and it keeps the STM32 tree's property that adding a
board means adding a directory.

**`PROJECT_BOARD_PORT_PATH` must be relative to the SDK root**, because the SDK
appends it to `SdkRootDirPath`. Computed with `file(RELATIVE_PATH)`, guarded
with a clear error if the two trees are on different drives — which that scheme
genuinely cannot express.

**`SYS:CLK?` keeps its field names but `pclk1` means something else.** The
RT1180 clocks each peripheral from its own root; there is no shared APB. The
field carries the cycle-counter rate, which is what a host actually wants.

**`SYS:BANK*` answers `unsupported`.** No bankable internal flash here — the
part boots XIP from external QSPI NOR. Added `BB_SWAP_UNSUPPORTED` so a host can
tell "never going to work here" from the STM32's recoverable refusals.
`bb_image_crc32` is kept real: the capture format needs it.

**The ELE must be pinged every 24 hours or the SoC resets.** No STM32
equivalent; it would present as "the board reboots if left over a weekend".
Pinged once a minute from the heartbeat in both host-facing images.

---

## Phase 1 — the IOMUXC tap experiment — DONE, answer is no

**Question:** can a FlexIO input be pointed at a pad already muxed to LPI2C,
giving a zero-wire internal tap of a live controller pin?

**No, and it is a hard no at the register level.** Every FlexIO mux define has an
empty input-select field:

```
IOMUXC_GPIO_EMC_B1_14_FLEXIO1_FLEXIO14  0x42A10048, 0x8, 0, 0, 0x42A10290
IOMUXC_GPIO_AD_15_FLEXIO2_FLEXIO15      0x42A10148, 0x8, 0, 0, 0x42A10390
                                                          ^  ^  no SELECT_INPUT
```

Compare a bus signal, which has one — and note it selects among *three pads for
LPI2C3*, i.e. where the bus lives, not who observes it:

```
IOMUXC_GPIO_EMC_B2_00_LPI2C3_SCL   0x42A100B8, 0x9, 0x42A105E8, 0x0, ...
IOMUXC_GPIO_EMC_B2_19_LPI2C3_SCL   0x42A10104, 0x8, 0x42A105E8, 0x1, ...
IOMUXC_GPIO_AD_18_LPI2C3_SCL       0x42A10154, 0x9, 0x42A105E8, 0x2, ...
```

FlexIO has no pad selection at all: `FLEXIO1_FLEXIO14` exists on exactly one pad.
And `MUX_MODE` is single-valued, confirmed by the 11 pads that offer both a bus
function and FlexIO and can still only be one at a time.

This needed no bench. Finding it in the header rather than after wiring a board
is the cheap version.

### The finding that replaces it

Losing the internal tap forced a better mechanism than "the firmware writes down
what it sent": **eDMA the controller's own FIFO into the capture ring.** LPI2C,
LPSPI and LPUART all have DMA-capable FIFOs and there are 64 eDMA channels. Byte
truth about our own traffic, no CPU cost, no pins — and it works while the bus is
in use because it *is* the bus in use. Three sources, honestly distinguished:

| `SNIFF:SRC` | Mechanism | Pins | Sees |
|---|---|---|---|
| `wire` | FlexIO, timer clocked from the bus clock | tap pads | everything on the bus, including other masters |
| `self` | eDMA from our own controller FIFO | none | every byte we sent or received, `observed=controller` |
| `both` | merged on timestamp | tap pads | wire truth cross-checked against intent |

`observed=controller` stays tagged all the way to the decoder. A FIFO says what
the peripheral accepted, not what the wire did — and those differ exactly when
something is wrong, which is when a sniffer earns its keep.

One accident of the IOMUX makes the wire case tidy: `GPIO_AD_17/18/19` (A2/A4/A5)
each offer I3C2, LPI2C3 **and** FlexIO2. With LPI2C3 on D14/D15 and FlexIO2
tapping A4/A5, self-tapping I2C is two jumpers between adjacent headers.

---

## Phase 2 — bus masters — DONE

| Bus | Instance | Pins | Notes |
|---|---|---|---|
| I2C ×3 | LPI2C1/2/3 | D0-D1, J3, D14-D15 | bus 1 off by default: its pads are the console |
| I3C ×2 | I3C1, I3C2 | J3, A4-A5 | **the STM32 could only ever drive one** |
| SPI | LPSPI4 | D10-D13 | software CS, so it can be held across commands |
| MDIO | bit-banged | D8-D9 | 377 lines ported with 6 hardware touch points changed |
| UART | LPUART4 | D2-D3 | framed and transparent modes |

**`bb_mdio.c` ported nearly verbatim** — it was already GPIO-level, so all the
clause-22/45 framing, turnaround handling and the "nothing answered vs 0xFFFF"
distinction came across untouched.

**`bb_i2c_timing.c` is gone entirely.** The STM32 needed 89 lines solving ST's
TIMINGR fields against the spec's setup and hold limits. LPI2C takes a baud rate
and derives its own dividers. `I2C:TIMING` now means "write MCCR0 raw" and the
usage string says MCCR0, so nobody feeds it an STM32 value.

**Two I3C controllers, so `I3C:PORT` is new.** The STM32 porting notes flagged
that `BB_I3C_PORT_COUNT` counted pin *pairs* of one I3C1 and that a part with
two I3C peripherals "cannot use the second without parameterising `bb_i3c.c`,
which references `I3C1` directly throughout". That is exactly the change made:
nothing in the new file names an instance.

**The STM32's 64 MHz I3C workaround did not need carrying over.** There, I3C had
to be clocked from HSI at 64 MHz rather than PCLK1 at 250 MHz because the timing
fields were 8 bits per phase and a 1.3 µs legacy-I2C low period needed 325
counts. The cost was that 100 kHz legacy devices could not share the I3C bus.
NXP's I3C takes three separate baud rates and derives its own dividers, so
`I3C:BUS mixed` is real here rather than a compromise.

**Target-role commands moved.** `I3C:ROLE target` and `I3C:TGT:*` answer
`unsupported` in the bridge image and name the peripheral image. On the STM32
they lived in a file shared by both profiles; a controller and a target want the
same peripheral and the same pins, and one image doing both is how a bus ends up
with two things driving SCL. They are listed in the table so a host gets "wrong
image" rather than "unknown command", which would be indistinguishable from old
firmware.

---

## Phase 3/4 — capture engine — PARTIAL, and precisely so

**Done:** the capture record format (`shared/bb_capture.h`), the CM33↔CM7 link,
the `SNIFF:*` command surface, the CM7 image, record serialisation with CRCs,
gap records, and the three-way `SNIFF:SRC`.

**Not done: the FlexIO timer and shifter programming itself.** `cap_start()`
returns `BB_CAP_ERR_FLEXIO` and every `SNIFF:START` says so. This is the one part
of the design that must not be guessed at, and it is left explicitly
unimplemented rather than filled in with plausible register writes — because a
wrong timer configuration produces a capture that looks entirely plausible and is
shifted by one bit, which is *exactly* the STM32 I3C defect arrived at from the
other direction. It wants the FlexIO chapter of `IMXRT1180RM` and a loopback
self test to prove it.

`SNIFF:BENCH` reports zeros and an error rather than a number derived from clock
arithmetic. No performance figure is claimed until it has been measured on this
board with the method stated.

### The three inherited contracts are honoured

1. **Picoseconds since capture start, in a u64.** Not ticks — a tick count means
   nothing without the rate, and that rate belongs to one device. Every record
   carries `ps_per_tick` so the host converts without knowing this board.
2. **Length-prefixed CRC'd records.** A file cut short by a crash reads up to the
   first record that does not check out, everything before it kept.
3. **The first sample entry is the initial state, not a transition.** Carried in
   the META record. Without it a decoder reads the first release of SDA as a
   STOP that never happened — which is what the STM32's first capture did.

### Design changes made while building

**No RPMsg.** The spec named it because the SDK ships it. Building the link, a
simpler thing was enough and easier to be sure of: strict request/response, one
message in flight, CM33-initiated. RPMsg's virtqueues, buffer pools and name
service all support multiple concurrent bidirectional endpoints — none of which
is wanted, and every one of which is state two cores can disagree about. The
transport is two buffers and two ready flags, about 80 lines.

**No doorbell interrupt.** MU1 belongs to MCMGR, which is what starts the CM7;
borrowing its GP channels would make this depend on an implementation detail of
somebody else's middleware. The CM7 polls one non-cacheable word instead. That
is affordable *here* precisely because the capture path is FlexIO plus eDMA and
spends no CPU per edge — it would not have been on the STM32, where the capture
*was* the CPU.

**Cache maintenance is designed out, not managed.** The shared region is in
OCRAM2 marked non-cacheable by the MPU on both cores. A protocol that is only
correct when both sides remember to invalidate and flush is one that works until
it does not.

**The link being absent is a first-class state.** `bb_link_up()` false makes
every `SNIFF:*` answer "capture core not running (`reason`) load-the-cm7-image"
immediately rather than hanging. A sniffer whose commands block forever when the
other core is missing is indistinguishable from one that is quietly wrong.

**The ring does not wrap** and the size was got wrong once: 32768 entries is a
round 256 KB and the linker refused it by 1400 bytes, because stack and heap live
in the same region. Now 24576 entries / 192 KB. "The whole of DTCM" is never the
available figure.

---

## Phase 5 — CAN — DONE

FlexCAN1 to the on-board transceiver U13. New to this project entirely: the
NUCLEO had no transceiver, so the Rust sniffer's CAN decoder has never had real
traffic to decode.

Listen-only is a **hardware** mode here — the receiver acknowledges nothing and
transmits nothing, so attaching to a live bus cannot disturb it. That is a
stronger guarantee than the two-wire buses get, where "passive" is a property of
how the firmware is written.

`CAN:TX` failure is reported as *"no-ack-bus-may-have-one-node"* rather than a
generic error, because a good CRC with no acknowledge is the commonest reason a
CAN link will not come up — the same distinction the Rust decoder makes.

**Two board conditions, reported in `CAN:CFG`'s own reply:** jumpers J44/J46 must
be at 1-2, and **J22 is DNP** — the transceiver is fitted and wired but there is
no header to plug a bus into. That is reported on every configure, not just at
boot, because it is the most likely reason for "configured fine, no traffic" and
a host that reconnects will not have seen the banner.

---

## Peripheral profile — DONE for I2C and SPI

`bb_ee.c` and `bb_ee_cmd.c` ported **verbatim** — 340 lines with zero HAL
references, because the EEPROM core was already written to know nothing about any
bus.

| Door | State |
|---|---|
| I2C | LPI2C3 target at 0x50 on D14-D15. Clock stretching on every phase. |
| SPI | LPSPI4 target on D10-D13, `0x03`/`0x02`/`0x05`. CS **sampled, not interrupted** — reported as `cs=polled`, and a frame shorter than the poll interval is missed. |
| I3C | not implemented, reports absent with the reason |
| MDIO | not implemented, reports absent with the reason |

The two missing ones report absent rather than answering incorrectly, and for a
device under test that is the whole point: a target that acknowledges ENTDAA and
then misbehaves leaves a bus full of devices at addresses nobody agreed on. When
MDIO's target is written, FlexIO is almost certainly the right mechanism rather
than an edge interrupt — so it is better built after the capture engine exists
than before it, twice.

---

## Phase 6 — Ethernet/TSN/EtherCAT — bounded first slice DONE

Scoped as "later phase" by agreement, and the spec says the whole of it deserves
its own document. What is built is the one part that is genuinely bounded and
impossible any other way: **`NET:*`, the on-board PHYs over hardware MDIO.**

`docs/PINS.md` established that `EMDIO` reaches none of the 22 Arduino pads. The
corollary is the useful half — hardware MDIO *can* reach the two Motorcomm
YT8531 gigabit PHYs (U21, U29) and nothing else can. Without this they are
unreachable from the command protocol.

`NET:PORTS?` `NET:SCAN` `NET:PHY:R` `NET:PHY:W` `NET:LINK?`. PHY addresses are
found by scanning rather than asserted, following the pin map's discipline:
publish what was found, do not hardcode what was assumed. `NET:LINK?` reads BMSR
twice, because its link bit is latching-low on most PHYs and reporting the first
read makes a link that came up look down exactly once.

No frames, no TSN scheduling, no EtherCAT. Each needs the switch and the ENETC
datapath brought up. Reading a PHY's identity and link state is what you want
working first: if MDIO to the PHY does not work, nothing above it will, and the
debugging time goes into the wrong layer.

---

## USB high speed — DONE (firmware and host), untested

Asked for as "update the windows driver to get the max USB speed". The driver
package turned out not to be the lever; two other things were.

### The INF is not the lever

`driver/busbridge.inf` sets `PortSubClass` and a `serenum` upper filter — that is
the **CDC serial** device, the ST-LINK VCP path retired at 692 round trips a
second. WinUSB needs no INF at all; the device's Microsoft OS 2.0 descriptors
bind it. There is no registry policy in that package that raises any ceiling.

### What did limit throughput

| # | Limit | Where | Fix |
|---|---|---|---|
| 1 | **One transfer in flight.** A 512-byte overlapped read, waited on, then the next. USB is scheduled per frame, so one outstanding transfer means at most one packet served per frame — the link rate is irrelevant. | host | 4 queued transfers |
| 2 | **512-byte buffer.** On full speed that batched 8 packets of 64; on high speed it is *one* packet of 512, so the same buffer batches an eighth as much work. | host | 64 KB reads, clamped to `MAXIMUM_TRANSFER_SIZE` |
| 3 | `BULK_MAX_PACKET 64`, hardcoded "full speed bulk is fixed at 64" | firmware | 512 at high speed, patched to the enumerated speed |
| 4 | No `RAW_IO` — WinUSB buffers and re-splits in the kernel | host | `RAW_IO` on the IN pipe |

**(1) is the biggest, and it is a host change.** On full speed it is the whole
story behind 110 kB/s. On high speed the same code would reach roughly 8× that
and stop, because the bottleneck moves without going away.

### What will not improve 40×

High speed is a 40× link — 480 Mbit/s against 12. **Round trips will not improve
40×.** A round trip is one OUT transfer, the firmware's response, one IN
transfer, and its floor is frame timing: 1 ms frames become 125 µs microframes,
which is **8×**. Expect round trips in that order and bulk payload much more.

Worth writing down because the natural benchmark — commands per second —
measures the thing that improves least. Reporting it as though it measured the
link is how a successful change gets recorded as a failure.

Numbers to beat, measured on the STM32 at full speed: **5864 round trips/s,
113.1 kB/s payload.** No prediction offered for what this reaches.

### Firmware: `firmware/src/bb_usb.c` (was a stub)

Real device now. Four things decide whether it comes up at high speed at all:

- `kUSB_ControllerEhci0`, not KHCI. KHCI is the full-speed controller.
- `bcdUSB 0x0201`. Windows only requests a BOS descriptor — how MS OS 2.0
  descriptors are advertised — from a device claiming 2.01 or later. Declaring
  2.00 makes the whole WinUSB binding path silently not happen.
- A **device qualifier** descriptor. Omitting it is legal only for a
  full-speed-only device, and Windows reads its absence as "cannot do high
  speed" — so a capable device enumerates at full speed and every measurement
  afterwards is of the wrong thing.
- Bulk `wMaxPacketSize` patched to match the speed actually enumerated, with the
  endpoint opened at the same figure. Out of step and the device enumerates,
  answers a few packets, then stalls like a firmware hang.

`SYS:USB?` reports the enumerated speed and packet size in force. **Check it
before trusting any throughput number** — a device that quietly came up at full
speed is the likeliest reason a high-speed figure comes out low.

### The bug that compiling could not catch

The first version registered the device with `USB_DeviceInit()` and handled
vendor requests through a function called `USB_DeviceProcessVendorRequest()`.
It compiled and linked cleanly. It would never have enumerated.

The SDK's ch9 handler routes GET_DESCRIPTOR **and** vendor requests through
`USB_DeviceClassCallback()`, which forwards to `configList->deviceCallback`. A
device brought up with `USB_DeviceInit()` alone never receives them — nothing
would have answered the very first descriptor request. And the invented vendor
hook was dead code, so the linker garbage-collected the MS OS 2.0 descriptor set
along with it, which would have meant WinUSB never binding either.

Caught by grepping the linked image for the `WINUSB` compatible-ID string and
finding it absent. Now brought up through `USB_DeviceClassInit()` with **zero
classes registered** — the class loop is a no-op at `count = 0`, so it costs
nothing and puts the callback where the stack actually calls it. String
descriptors were missing too, and the serial number is now derived from the
die's unique ID, because that is what lets a host pick one board out of several
on a bench.

### Host: `host/windows/bb_winusb_hs.c`

Drop-in replacement for the STM32 tree's `windows/src/bb_winusb.c` — same entry
points, same handle fields. **The STM32 tree is not modified.**

Two things the transfer queue makes newly possible to get wrong, both handled:

- **Completions are reaped in submission order**, not by whichever event fires
  first. Bulk transfers on one pipe complete in order; taking them out of order
  reorders the byte stream, which for a line-oriented protocol shows up as
  garbled replies *under load only*.
- **`flush` aborts and drains the queue**, not just the reassembly buffer.
  Queued transfers otherwise hold stale bytes that surface as the first line of
  the *next* command's reply — a one-command-behind bug that queueing introduces
  if flush is not written for it.

`bb_winusb_transport_info_hs()` reports in-flight count, read size and whether
`RAW_IO` was accepted, so a measurement can be attributed to a configuration.

### The connector, and a size warning

USB **OTG1 on J63**, not the MCU-Link debug USB on J23 — different connectors,
different silicon. A second cable is needed.

The bridge image is now at **93% of the 127 KB CM33 TCM.** The XIP variant is at
1.4% of 8 MB QSPI NOR, so a bigger controller image has somewhere to go, but the
TCM build is close to full and the next addition may not fit.

---

---

## FIRST HARDWARE RUN — the bridge image works over the console

`scripts\flash.ps1 -Profile bridge`, then commands over COM19. **This is no
longer "compiles and links".** Verified output:

```
# BUSBRIDGE 0.1.0 (bridge) on FRDM-IMXRT1186 ready
# 3 I2C, 2 I3C, SPI, MDIO, UART, CAN, sniffer, PHYs

SYS:ID?        -> OK board=FRDM-IMXRT1186 ... expect=0x1186 checked=0 profile=bridge
SYS:CLK?       -> OK sysclk=240000000 hclk=240000000 pclk1=240000000
SYS:STAT?      -> OK uptime_ms=9897 rx_overruns=0 twowire=i2c i2c_buses=3
I2C:PORTS?     -> OK i2c1=AON_09/AON_08/needs-J41-J43-and-BB_I2C1_ENABLE/100000Hz
                     i2c2=AON_16/AON_15/idle/100000Hz i2c3=EMC_B2_19/EMC_B2_20/ready/100000Hz
I3C:PORT?      -> OK p1=AON_16/AON_15 p2=AD_18/AD_19*
SNIFF:TAPS?    -> OK d4=pad/flexio1.14 ... a2=pad/flexio2.17
SNIFF:STAT?    -> ERR 8 stat capture-core-not-running (cm7-not-running) load-the-cm7-image
SPI:CFG?       -> OK hz=1000000 mode=0 width=8 order=msb cs=manual cspol=1 idle=FF
UART:CFG?      -> OK baud=115200 bits=8 parity=N stop=1 flow=none rx=1 overruns=0
CAN:CFG?       -> OK bitrate=500000 listen=0 ext=0 state=idle rx=0 tx=0 dropped=0
NET:PORTS?     -> OK eth0=U21/YT8531/J56A eth2=U29/YT8531/J56B ecat0=J57A ...
SYS:PINS? i2c3 -> OK i2c3.scl=GPIO_EMC_B2_19/D14/J2-18 i2c3.sda=GPIO_EMC_B2_20/D15/J2-20
I2C:SCAN       -> OK i2c3 none
```

Clock tree, console, command pump, pin map, all five bus drivers' configuration
paths, and the honest "capture core not running" refusal: all confirmed on
silicon. `I2C:SCAN` returning `none` is correct with nothing wired.

### Two real bugs, neither findable by compiling

**1. I3C divide-by-zero — hard fault before the banner.**
`i3c_apply()` set `baudRate_Hz.i2cBaud = 0` on a pure I3C bus, which reads as
"no legacy I2C here" and is how the field is documented. But
`I3C_MasterSetBaudRate()` divides by it unconditionally, with no guard:

```c
divEven = (sourceClock_Hz / i2cBaud_HZ) / (2 * (ppBaud+1) * (odBaud+1));
```

`DIV_0_TRP` is set on this Cortex-M33, so it trapped to `DefaultISR`. Symptom:
the board loaded, ran, and went silent before printing anything. Found by
bisecting `bb_profile_init()` with breakpoints — `bb_i2c_init` completed,
`bb_i3c_init` was entered, and the next step landed in `DefaultISR`.
Fix: always pass a computable rate.

**2. 1 KB stack — the same edit the STM32 tree had to make.**
`m_stack_size` defaults to `0x400` in the device linker script and the startup
code programs `MSPLIM`, so overflow traps instead of corrupting.
`bb_reply_okf()` alone puts a 256-byte format buffer and a 264-byte line buffer
on the stack.

The symptom was the instructive part: the banner printed, a command arrived, the
tokeniser ran correctly (`line="SYS:ID?"`, `status=7`), the handler was entered
— and then it vanished into `DefaultISR` inside `bb_board_silicon()`, a function
that does nothing but `memset` a struct. **A function that cannot fault,
faulting, is what points at the stack rather than at the code.**
Fix: `--defsym=__stack_size__=0x2000` in the board port's `reconfig.cmake`,
matching the STM32 tree's 1 KB → 8 KB.

## USB — working, high speed, WinUSB bound

Cable in **J63**. Verified on hardware:

```
Windows : USB\VID_1209&PID_0003  Status OK  Class USBDevice
          compat  USB\MS_COMP_WINUSB   inf winusb.inf   service WINUSB
          DeviceInterfaceGUIDs {1D4B2E5F-9C3A-4F8B-B0E7-2A6C8D3F5A91}
pipes   : 0x01 OUT / 0x81 IN, maxpacket 512  (high speed)
device  : state=configured transport=usb dppu=480 frame=512 resets=1
```

Measured with the STM32 tree's own `bench.exe`, so the comparison is the same
binary against both boards:

| | STM32H5 | FRDM-IMXRT1186 |
|---|---|---|
| round trips | 5864 /s | **9856 /s** |
| payload, 256 B both ways | 113.1 kB/s | **158.3 kB/s** |

Still the *old* host path - one transfer at a time, 512-byte reads. Nothing here
uses `host/windows/bb_winusb_hs.c` yet, which is where the queued transfers and
64 KB reads live. Take these as the floor.

Multi-packet replies (`I2C:PORTS?`, `SNIFF:TAPS?`, the 1140-byte `SYS:PINS?`)
arrive complete.

### Two bugs, and neither was in the descriptors

The device would appear in Device Manager and then fail. It took a while because
**every descriptor was correct the whole time** - the problem was in what the
host was allowed to ask for, and how the answer was shaped.

**1. BOS support was compiled out.** `usb_device_ch9.c` guards the entire
`USB_DESCRIPTOR_TYPE_BOS` branch behind `USB_DEVICE_CONFIG_LPM_L1`:

```c
#if (defined(USB_DEVICE_CONFIG_LPM_L1) && (USB_DEVICE_CONFIG_LPM_L1 > 0U))
    else if (USB_DESCRIPTOR_TYPE_BOS == descriptorType)
```

That symbol is absent from the generated `usb_device_config.h` and is not a
Kconfig option, so `GET_DESCRIPTOR(BOS)` never reached the device callback. The
BOS descriptor is what advertises that a Microsoft OS 2.0 descriptor set exists;
without it, the whole driver-free WinUSB binding path is dead code that links
cleanly and is never called. Fixed with `-DUSB_DEVICE_CONFIG_LPM_L1=1` in
`firmware/CMakeLists.txt`.

The name is actively misleading: nothing here wants link power management. It is
simply the only guard the SDK puts around BOS.

**2. The MS OS 2.0 set was nested; it has to be flat.** The first version used
set header → configuration subset → function subset → features. Every length and
type field in it was correct - verified byte by byte out of the linked image
against the spec - and the host fetched all 178 bytes:

```
vnd_type=0xC0  vnd_index=7  vnd_wlen=178  vnd_sent=178
```

Windows then silently declined to act on it: no `MS_COMP_WINUSB` compatible id,
no `DeviceInterfaceGUIDs` registry value, `CM_PROB_FAILED_INSTALL`
(`0xC0000490`, NOT_FOUND) and nothing in `setupapi.dev.log`. Flattening it - the
compatible id and registry property directly under the set header, 162 bytes -
made it bind immediately.

The STM32 tree already had it flat, with a comment saying so. Reading the
reference implementation being ported from would have been faster than deriving
it from the spec, which is the actual lesson here.

### How it was found

Aggregate counters said "stopped somewhere after the descriptors", which was not
enough. What resolved it was recording the *order* of events rather than the
count - `SYS:USB:TRACE?` keeps a shift register of 4-bit event codes:

```
events=1223122312231223   ->  reset, dev, dev, cfg   x4, then give up
state=1 (Address)         ->  SET_ADDRESS did land; that theory was wrong
```

and then, once BOS worked, comparing against a device known to bind **on this
same machine**: the STM32 board's own device nodes, still in the registry,
carrying `MS_COMP_WINUSB` and `winusb.inf`. Diffing against a working reference
beat reasoning about the spec.

A hypothesis that did *not* pan out, recorded because it looked convincing:
`device_cb` left `st = kStatus_USB_Error` for `GetConfiguration`,
`GetInterface`, `SetInterface` and `SetRemoteWakeup`, which ch9 turns into an
EP0 stall. Handling all four was correct and is kept - but the counters came
back byte-identical, so it was not the cause of anything.

### Other changes this needed

| Change | Why |
|---|---|
| `idProduct` 0x0002 → **0x0003** | Windows keys the device node on VID + PID + serial. With no admin rights the stale nodes from the failing runs could not be removed, so a fresh PID was the only way to get an uncached node. |
| `bcdDevice` → 1.03 | Same reason; kept for the record. |
| `bInterfaceSubClass`/`Protocol` 0xFF/0xFF → **0x00/0x00** | Matches the STM32 build, whose nodes on this machine bind WinUSB. Removes a difference with no upside. |
| `iSerialNumber` gated on a real UID | `bb_uid_word()` returns zero on every word because the OCOTP read is unimplemented, so every board would claim the same serial and collapse into one device node - two boards could not be used on one machine. `iSerialNumber = 0` is the spec's "no serial", and hosts then derive a unique per-port id. Starts publishing a real serial with no other change once OCOTP lands. |

## GUI — built and running against this board

The GUI in the STM32 tree needs no changes: it links `busbridge_static.lib` and
finds devices by interface GUID, so the identity work above is all it needed.

Built with the target directory pointed out of the STM32 tree, so that tree stays
untouched as required:

```powershell
$env:CARGO_TARGET_DIR  = 'E:/BusBridge-RT/build/gui-target'
$env:BUSBRIDGE_LIB_DIR = 'E:/STMicro/busbridge/windows/build/x64'
cargo build --release            # in E:/STMicro/busbridge/gui
```

`build/gui-target/release/busbridge-gui.exe` starts and runs - window title
"BUSBRIDGE Bench Console", responding. Whether its device panel shows a live
connection is on screen rather than something measurable from here; what *is*
measured is that the same host library the GUI links opens this board by GUID
and runs commands over the bulk pipes (`enumerate.exe`, `bench.exe`,
`api_test.exe` above).

Two known gaps in it:

- The **Firmware tab will not work.** It drives pyOCD, and pyOCD cannot attach
  to this part at all - the RT1180 is ADIv6 / CoreSight SoC-600 with
  base-address access ports, and pyOCD enumerates ADIv5 indices. Use
  `scripts/flash.ps1` (LinkServer) instead.
- The **sniffer tabs will report unsupported**, because the CM7 capture engine
  is not implemented. That is the same 22 `api_test` failures, not a GUI problem.

Still to do on the host side: swap to `host/windows/bb_winusb_hs.c` for the
queued transfers and 64 KB reads. That needs a host C compiler, which is not
installed on this machine - `busbridge_static.lib` here is the prebuilt
single-transfer library, so every figure in this document is a floor rather than
a ceiling.

## Host library conformance — 123 of 148

The prebuilt host tools in the STM32 tree (`windows/build/x64/`) find devices by
interface GUID rather than by VID/PID, so they drive this board unmodified.
`enumerate.exe`:

```
USB  (7&5edeed&0&4)
    BUSBRIDGE FRDM-IMXRT1186 firmware 0.1.0
    MCU id 000000000000000000000000, core 240 MHz
```

`api_test.exe` went from **39 passed / 106 failed** to **123 passed / 25 failed**.
Running a real host library against the firmware found five bugs that neither
compiling nor the console could: the console is line-oriented and forgiving, and
a human reading `OK 100000` cannot tell it apart from the `OK hz=100000` the
protocol requires.

### The pad configuration bug — one cause, six symptoms

The largest one. `bb_hal.h` defined pad modes as single numeric SW_PAD_CTL
values. But this part has **two pad register layouts**, and which applies depends
on where the pad is:

```
GPIO_EMC_B1_*, GPIO_EMC_B2_*   PDRV[1:0], PULL[3:2], ODE[4]
GPIO_AD_*, GPIO_AON_*          SRE[0], DSE[1], PUE[2], PUS[3], ODE[4]
```

They disagree in exactly the bits that matter. A weak pull-up is `PULL=01` -
bit 2 alone - on an EMC pad, and `PUE|PUS` - bits 2 and 3 - on an AD pad. So
bits 2 and 3 together mean *pull up* on one layout and *no pull at all* on the
other. The original values (`BB_PAD_I2C = 0x131E`) were RT1170-shaped and
configured every EMC pad as open drain with no pull.

What that produced:

| Symptom | Why |
|---|---|
| `MDIO:C22:R` returned 0x0000 on an idle bus | a released open-drain line floats instead of pulling up, so every bit reads 0 |
| `I2C:PROBE` → `probe-busy`, and every transfer after it | SCL and SDA never rise, so LPI2C's bus-busy flag stays set forever |
| `I2C:SCAN` completing in 0.4 ms | 112 addresses "failing" instantly rather than being probed |

None of those looks like a pad problem. Each looks like the layer above it -
a dead PHY, a stuck peripheral, an empty bus - which is why it survived a
working console and a clean build.

The fix makes `BB_PAD_*` an *intent* (`BB_PAD_I2C`, `BB_PAD_PUSHPULL`,
`BB_PAD_INPUT`, `BB_PAD_INPUT_PU`) and encodes it per pad in `pad_encode()` in
`bb_board.c`, choosing the layout from the pad's config register address. The
group boundaries come from `fsl_iomuxc.h`, where EMC_B1 and EMC_B2 are
contiguous. Callers name what they want and cannot get the layout wrong; a pin
added later is encoded correctly without anyone knowing this exists.

After it, `I2C:SCAN` takes 13.2 ms - 112 real transactions at 400 kHz - and the
four `peripheral busy` failures and the MDIO floating read all went away
together.

`bb_pin_gpio()` also passed SION = 0, so a pad configured as an output could not
be read back at all. Every bit-banged open-drain protocol depends on exactly
that: drive high to release, then sample. It is now unconditional.

### An unbounded wait that hung the firmware

`I2C:WR` on a busy bus killed the device outright, and the host saw the *next*
command time out - so the search started at `I2C:RECOVER`, which was innocent.
Reproducing it one command at a time was what separated the two.

`cmd_write_read()` issued `LPI2C_MasterStop()` whenever the write phase failed.
But `kStatus_LPI2C_Busy` comes from the bus-busy check that
`LPI2C_MasterTransferBlocking()` makes *before* sending anything, so there was no
transaction to end - and `LPI2C_MasterStop()` then waits for a STOP condition
that cannot appear on a bus which is already stuck. Its wait loop, unless
`I2C_RETRY_TIMES` is defined, is a bare `while (result == kStatus_Success)`.

Fixed twice over: the STOP is skipped when the transfer never started, and
`-DI2C_RETRY_TIMES=1000000` bounds every polled wait in `fsl_lpi2c.c`. The second
half matters more than the first. A bus tool's whole job is to talk to hardware
that may be stuck, absent or mis-clocked, so it must always return an answer -
an unbounded wait anywhere below the command loop is a defect, not a tuning
choice.

Related, found while reading: `I2C:TIMING` writes a raw value to MCCR0, and
`CLKLO` or `CLKHI` at zero means SCL never completes a cycle. That is
unrecoverable rather than merely wrong, so those two values are now refused. The
host API defines the argument as an STM32 `TIMINGR` word, which lands zeros there
much of the time.

`*_Deinit()` was also being called on already-gated peripherals in I2C, I3C, SPI
and UART: it gates the clock as its last act and resets registers as its first,
so a second call writes to an unclocked peripheral and stalls the AHB with no
fault. `*_Init()` ungates and resets on its own, so the deinit is now guarded on
the peripheral being configured. This was found while chasing the hang above and
is *not* what caused it - worth saying, because the guards look like the fix and
are not.

### Reply formats the console could not reveal

Five commands replied truthfully and unparseably. Each is a named-field or
structural contract the host library depends on:

| Command | Was | Now |
|---|---|---|
| `I2C:FREQ`, `I2C:FREQ?` | `OK 100000` | `OK hz=100000 mccr0=0x...` |
| `I2C:BUS?` | `OK 2` | `OK bus=2` |
| `I2C:SCAN` | `OK i2c3 none` | a bare hex byte list |
| `I2C:PORTS?` | readable text | `count=n active=i idx:pins,freq,up;...` |
| `I3C:TARGETS?` | inline list | `targets=n` plus one `#` line per target |
| `I3C:IBI?` | `listening=0 n=0` | `none` |

`I2C:SCANALL` keeps the readable form - that is what someone at a terminal
wants - and both go through one `scan_collect()` so they cannot disagree about
what is on the bus. `I2C:PORTS?` loses the "why is this bus unavailable" text,
because the parser takes fields from the right and free text breaks it; nothing
is lost, since `bus_ready()` names the reason in the error when a host actually
tries to use the bus, which is when it matters.

### The 25 that still fail, and why none is a defect

**22 are the capture engine.** `SNIFF:*` and the capture tests report "not
supported by this firmware" because the CM7 FlexIO timer and shifter programming
in `cm7/capture_main.c` is not written. `cap_start()` returns
`BB_CAP_ERR_FLEXIO` deliberately. This is the honest answer and the largest piece
of remaining work.

**3 are board differences the test cannot know about:**

- `core clock: expected 250 MHz, got 240000000` - the test hardcodes the
  STM32H5's clock. The CM33 runs at 240 MHz.
- `bb_pins: expected the whole map` - the test looks for a signal prefixed
  `i3c.`, which assumes one I3C bus. This board has two, published as `i3c1.*`
  and `i3c2.*`. Renaming them to satisfy the test would make the map less
  accurate; prefix filtering with `i3c` still matches both, which is what the
  GUI does.
- `bb_rs_set_config rtscts` - LPUART4's flow-control pads are not brought out to
  the headers, so `UART:CFG ... rtscts` refuses rather than accepting a setting
  it would silently ignore.

## Capture engine — written and building, not yet proven on silicon

`cm7/capture_main.c` now programs FlexIO instead of returning
`BB_CAP_ERR_FLEXIO`. Both images build (capture 33.7 KB of 255 KB text, bridge
97% of its 127 KB). **It has not captured a bus yet** - the two-core bring-up
below blocks that, and nothing here should be read as a measurement.

### Uniform sampling, not a pin-clocked shifter — and why that is a change

The spec's table pairs each protocol with a timer clocked from the bus's own
clock line: SCL clocks the timer, a shifter latches SDA on the edge itself. That
is the right shape for *decoded* capture and it is bit-exact by construction.

It cannot produce this container's records. A pin-clocked shifter yields bit
values in bus-clock order with nothing to say when any of them happened, and the
format's first contract is picosecond timestamps. Filling `dt_ticks` with a count
of bus clocks would satisfy the struct while silently changing what the field
means to every host that already reads it - the worst available option, and the
spec is explicit that the three container contracts are not being renegotiated.

So the raw path samples every tap uniformly from an internal clock and compresses
runs to transitions in software, which is what the dedicated core buys. That
keeps `dt_ticks` genuinely temporal, serves every protocol with one
configuration, and hands the host the whole waveform - START and STOP
conditions, glitches, setup violations - rather than only the bits a
protocol-specific decoder expected. It also fixes the STM32's I3C off-by-one, by
a different route than the spec proposed: there is no software read chasing an
edge, so there is no window to be late in.

The pin-clocked configuration is still the right implementation for the
decoded-events mode. `fsl_flexio_camera.c` is a working template for it - a
pin-clocked parallel receive shifter using
`kFLEXIO_TimerDecSrcOnPinInputShiftPinInput`.

### The parts worth knowing about

**A parallel window, not one pin per shifter.** One receive shifter reads
`width` consecutive pins per load, and `width` must divide 32 or samples straddle
the shifter word. It is rounded up to 1, 2, 4 or 8 and anchored at the *highest*
tap, because anchoring low runs off the end of the instance - FlexIO2's taps are
15..17 and a 4-wide window anchored at 15 would want pin 18. Taps need not be
adjacent or ordered: `bit_of_tap[]` maps each to its slot so `state` bit *i* is
always tap *i* in `SNIFF:CH` order, which is what keeps the host's channel
numbering matching `SNIFF:PINS?`.

**`parallelWidth` is width minus one.** Confirmed against
`fsl_flexio_camera.c`, which sets `FLEXIO_CAMERA_PARALLEL_DATA_WIDTH - 1U`.

**`CAP_OLDEST_IS_HIGH_SLOT` is a named constant, not a shift expression.** A
receive shifter loads each group at the low end and moves the register up, so the
first group of a full word sits in the most significant slot. Getting it backwards
produces a capture that looks entirely plausible with each group of samples
time-reversed - the same class of defect as the STM32 I3C bug, reached from the
other direction.

**Polling, not interrupts.** The shifter flag is polled from the main loop. This
core runs nothing else, so an interrupt buys nothing and costs a vector, a
priority and a concurrency question against the mailbox. It also makes the
failure visible: when sampling outruns the loop the shifter raises its error flag,
which becomes a real gap record rather than a quietly wrong capture. eDMA is what
lifts the ceiling and is the next step.

**The initial state is not an edge.** The first sample establishes
`last_state` and is not emitted, so a bus that was already idle high does not
appear to have risen at t=0. That is contract 3, and the phantom first edge is a
mistake the STM32 build actually made.

### The self test drives the window it samples

A transmit shifter configured for the same parallel window drives those pads
while the receive shifter reads them; the pads are muxed to FlexIO with SION, so
the input path sees what FlexIO itself is driving. No jumper wires, and the path
under test is the real capture path.

It also cannot repeat the STM32's specific failure, where generator and capture
shared a core so asking for a shorter bit period just made the generator slower -
that test passed at a nominal 3.3 MHz while never exceeding 202 kHz. Here the
generator is hardware with its own divider. `SNIFF:TEST` reports
`driven/expected/seen/sample_hz`, not a verdict, for the same reason: a pass/fail
cannot show how close it came. `SNIFF:BENCH` sweeps the generator faster until the
shifter overruns and reports the last clean rate, which is a property of this
core's polling loop - exactly the figure eDMA would improve.

One honest limit: generator and sampler are the same peripheral, so a shared
misunderstanding of slot order cancels out. The self test proves the engine
captures what is driven, at the rate claimed, in the right order *relative to the
generator* - not that `CAP_OLDEST_IS_HIGH_SLOT` matches a third party's bus. Only
a real bus settles that.

### Two real bugs found on the way

**The link could not survive the controller restarting.** The capture core raised
`cm7_alive` once at boot, and `bb_link_init()` clears it - correctly, since the
controller cannot know whether a capture core exists. So reloading the
controller, which is the thing one does constantly while developing and also what
happens on a watchdog or a USB reset, orphaned a perfectly healthy capture core
for good. The handshake had to be level-triggered from the capture side, not edge.

**"capture-core-not-running" could not be acted on.** It was true but did not
distinguish a core that never started from one running and failing to handshake,
and those need opposite fixes. There is now a heartbeat counter in shared memory
and `SNIFF:LINK?` samples it twice:

```
SNIFF:LINK? -> OK magic=0x42425254 expect=0x42425254 alive=0 ticks=0 delta=0
               verdict=cm7-halted-or-never-started
```

That verdict is what turned a guess into a fact.

## The two-core bring-up problem — the blocker, stated precisely

Nothing releases the CM7. That is a hole rather than an omission: the capture
core can only be started by a debugger, so `SNIFF:*` fails on any board that has
not had a second LinkServer session pointed at it.

What is established:

| Question | Answer | How |
|---|---|---|
| Does the capture image run? | **Yes** | loaded alone, `p/x $pc` refused with "Selected thread is running" - the core is executing, and `pc_after_load` was the entry point |
| Is the FlexIO clock there? | **Yes, 48 MHz** | `CLOCK_GetRootClockFreq(kCLOCK_Root_Flexio1)` on the running core; the board's `clock_config.c` sets SYS_PLL3_DIV2 / 5 |
| Can the CM7 mux its own taps? | **No** | `Cannot access memory at address 0x42a10048` from the CM7. IOMUXC is not in its map, which confirms the design's split rather than contradicting it - the controller must mux the pads, and it now does, in `push_config()` |
| Can the controller release the CM7? | **Not yet** | `SNIFF:CM7` runs the SDK's own RT1180 sequence and the enclave does not answer: `tsr=0x000000FF rsr=0x00000000` - all eight TX slots free, the command word accepted, no response |

The two debug connects also fight each other, which is why no load order works:
the CM7 connect script halts the M33 to wake the M7, so loading capture second
kills a running controller; and a normal M33 connect resets the SoC, which
re-asserts M7 reset and stops the capture core. `LinkServer gdbserver --attach`
avoids the reset, but it rejects `monitor halt`, so the image cannot be loaded
through it.

The likeliest reading of the silent enclave is that the debugger's connect script
has already released the M7 once and the enclave declines a second request. That
is a guess and is labelled as one.

### The route forward

The SDK's own multicore examples show the pattern, and it is the one that also
makes the product standalone: **the controller embeds the capture image, copies
it into the CM7's ITCM, and then releases the core.**
`MCMGR_StartCore()` ignores the boot address it is given on this part - the M7
boots from its own VTOR, which is why the capture profile links to ITCM
`0x00000000` - so the release itself is the short part. Copying is what makes it
safe: the controller currently has no way to see what is in ITCM, which is
exactly why `SNIFF:CM7` is a deliberate command rather than something
`bb_link_init()` does at boot over whatever happens to be there.

The obstacle is space, not design. The bridge is at 97% of its 127 KB of TCM and
the capture image is 34 KB, so embedding it needs the bridge moved to the
XIP/flash layout. That is a build-system change rather than a one-liner, and it
is the next piece of work.

### New commands

| Command | Purpose |
|---|---|
| `SNIFF:RATE <bus-hz> [sample-hz]` | the sample rate sets both edge resolution and how fast the ring fills; giving only the bus rate picks 16x oversampling |
| `SNIFF:RATE?` | what was asked for; `SNIFF:STAT?` reports what the divider achieved |
| `SNIFF:LINK?` | raw mailbox words and a verdict, for a link that is not up |
| `SNIFF:CM7` | release the capture core |

`SNIFF:STAT?` also now reports `sample_hz` and `overruns`.

## The bootable pair — built, and one step from running

`scripts\build.ps1 -Pair` now produces the two-core image in one command:

```
[1/3] capture core (cm7)                      33704 B  of 255 KB   12.91%
[2/3] embedding the capture image             34732 bytes, SP 0x20040000, PC 0x0000047D
[3/3] controller (cm33, XIP, image embedded)  160544 B of 8147 KB   1.92%
```

The controller carries the capture image, copies it into the CM7's ITCM, and
releases that core - the SDK's own multicore pattern, and the only configuration
in which both cores run without a debugger.

### What this needed

**The XIP target, because the RAM one cannot carry the image.** `m_text` is 127 KB
of TCM and the controller already fills it to 96%. The RAM linker script stores
`.data` initialisers in `m_text` too, so a 34 KB array costs 34 KB of TCM whether
it is `const` or not - the first attempt overflowed `m_text` by exactly that. The
SDK's own `core1_image` section has the same problem: it is placed `> m_text`.
`flexspi_nor` has 8 MB, where a `const` blob stays in flash and costs no RAM at
all. So the embed is an explicit `BB_EMBED_CM7` option that only the XIP build
passes, and the RAM build keeps working with `SNIFF:CM7` reporting that it
carries no image.

**The ITCM window, and the clock that makes it real.** The CM7 boots from its own
vector table at `0x00000000`, and that memory appears at `0x303C0000` in the
controller's map - `CORE1_BOOT_ADDRESS` from the SDK's board support for this
board, paired there with `CORE1_KICKOFF_ADDRESS 0x0`. It is checked rather than
trusted: the copy is read back before the core is released, so a wrong window is
reported as `itcm-window-readback-mismatch` instead of starting the CM7 on
whatever else lives there.

The window is not a plain aperture. Writing it from a debug session with the M7
held in reset fails outright - `Cannot access memory at address 0x303c0000` -
because it is the M7's own TCM and an unclocked core does not answer for it. So
`CLOCK_EnableClock(kCLOCK_M7)` comes first, before the copy. The SDK's example
gets away without it because its board init has already enabled the clock.

**A generator that refuses to produce a bad blob.** `scripts/mkcm7image.py`
checks the first two words of the binary - initial SP and reset vector - and
fails the build if the Thumb bit is missing or the reset vector lies outside the
image, which is what a binary not linked to `0x00000000` looks like. A stale or
wrong blob would be copied into ITCM and executed, a failure with no useful
symptom, so it is caught at build time instead.

### Where it stopped

The image programs and verifies. Read back from flash after programming:

```
FCB       @0x04000400  0x42464346  ("FCFB", the FlexSPI config block tag)
container @0x04001000  0x8700a000
app vector@0x0400B000  SP 0x20020000  reset 0x0400B47D
```

All correct. But after reset the CM33 sits at `0x100157EC` - boot ROM. It never
enters the image.

Two candidates, and this is not yet settled:

- **The boot-mode DIP.** `100` is FlexSPI quad SPI NOR and `001` is the serial
  downloader used for TCM/RAM debugging. Every image this project has run so far
  was loaded into RAM by a debugger, which works in either mode, so the switch
  position has not mattered until now and cannot be inferred from anything that
  has worked.
- **Container authentication.** RT1180 boots through a container that the
  EdgeLock Enclave checks. If this part is not in an open lifecycle, an unsigned
  container is refused - which would look exactly like this.

**Trying to run an XIP image through the debugger instead does not work.**
LinkServer programs it, but reads of the flash region return zero immediately
afterwards and the target ends up in a state gdb cannot drive. The ROM has to do
the booting, because the ROM is what configures FlexSPI.

### A hazard worth recording

While chasing the CM7 release, `SNIFF:CM7` sent the raw ELE command word
`0x17d20106` through the S3MU mailbox - the sequence copied from the SDK's MCMGR
port. The enclave never answered (`tsr=0x000000FF rsr=0x00000000`: command
accepted, no response), and some time after that the debug access port stopped
responding entirely:

```
Nc: DAP Speed test unexecuted or failed
Pc: (100) Target Operation Failed
Nn(05). Wire ACK Fault in DAP access
```

It does not recover in software - `LinkServer flash erase` gets
`Wire Ack Fault - target connected?` on every attempt. A power cycle is needed.

The enclave gates debug authentication on this part, so poking it with a raw
command sequence is a plausible way to lose the debug port, and that is the risk
taken by hand-rolling the sequence instead of linking MCMGR. It was chosen to
save ~3 KB in an image at 97% of TCM - a reasonable trade on size, and a poor one
on blast radius. In the XIP configuration the size argument no longer applies:
there are 8 MB, and MCMGR should be used.

## Both cores running, and the capture engine capturing

The bootable pair works. One image in flash, power on, and the controller starts
the capture core itself:

```
SNIFF:CM7   -> OK started
SNIFF:LINK? -> OK magic=0x42425254 alive=1 ticks=13470 delta=10083
               beacon=0x43374245 bticks=26732 bdelta=13249 saw_magic=1 verdict=up
SNIFF:STAT? -> OK run=0 proto=none ... ps_per_tick=1000000 sample_hz=1000000
```

And the FlexIO engine captures:

```
SNIFF:CH i2c d4 d5   -> OK
SNIFF:RATE 400000    -> OK
SNIFF:TEST           -> OK driven=1024 seen=21865 sample_hz=6000000 overruns=0
SNIFF:STAT?          -> OK entries=21865 coincident=217174 pending=174920
                          ps_per_tick=166666 sample_hz=6000000 overruns=0
```

239,039 raw samples compressed to 21,865 transition records, nothing lost. The
records decode as the format specifies:

```
hdr: type=02 SAMPLES  observed=00 wire  length=0x00F0  crc32=0x4AB7D918
     t_ps=0x07786D60 (125.65 us)
samples (dt_ticks, state):
     16,0   8,1   8,0   16,1   10,0   6,1   16,0   12,1   4,0   16,1 ...
```

Each full period sums to 32 ticks - 5.33 us at 6 MHz, about 188 kHz against a
generator running at roughly 200 kHz - and the varying 8/8, 10/6, 12/4 splits are
the aliasing that uniform sampling is supposed to show against an unrelated
clock. Timestamps, CRC, transition compression and the initial-state contract all
behave.

## Getting there: four things held the CM7, and only one was where I looked

Every round of this looked identical from the outside - "the capture core did not
start" - and the four causes needed unrelated fixes. Worth writing down in the
order they actually blocked, because the order I guessed in was nothing like it.

**1. The TCM is ECC memory and has to be initialised before it is touched.**
`Prepare_CM7()` in the SDK's `system_MIMXRT1186_cm33.c` clears all four CM7 TCM
windows by DMA, sets the initial VTOR, powers the PHY LDO / ARM PLL / DCDC, and
releases the core from boot reset through SRC. It has to run from
`SystemInitHook()`, before `main()`. Without it the 34 KB copy into
`0x303C0000` bus faulted on its *first write*, taking the command loop down while
USB stayed enumerated - which reads as an access-permission problem and is not
one.

**2. Nothing gave the M7 a root clock.** Both places in the board's
`clock_config.c` that set `kCLOCK_Root_M7` sit inside `#if (__CORTEX_M == 7)`, so
a controller running `BOARD_BootClockRUN()` configures its own root and leaves the
M7's alone - and the capture image can only set its own once it is already
executing. The controller now points it at OscRC400M before releasing; the capture
image switches itself to the ARM PLL for its 800 MHz once running. **This was the
actual blocker**: with `m7_root_hz=400000000` the core started immediately.

**3. The enclave was never silent; my reader was wrong.** `ENABLE_APC`
(`0x17D20106`) is what authorises the application core, and clearing CPU_WAIT
without it leaves the core held. My first version wrote that word at the S3MU
mailbox and read two response words back - which is what the SDK's MCMGR port
does inline - and reported the enclave as dead. It is not: the enclave may send
the *host* a command of its own before answering, so a reader that assumes the
next two words are its response waits forever for one queued behind something it
never serviced. `ELE_BaseAPI_EnableAPC()` handles that, and returns success
(`apc_status=0`). A `SNIFF:ELE?` ping confirmed the mailbox was fine all along.

**4. The mailbox was cached on both sides.** `BOARD_ConfigMPU()` deliberately
leaves OCRAM2 (`0x20500000`) without an MPU region on either core, so it falls
through to the background system address map - which its own comment describes as
"normal, write back, write/read allocate". The controller's writes sat in XCACHE
and the capture core read stale zeros. The SDK has a mechanism for exactly this
and I had missed it: `__USE_SHMEM` compiles in the non-cacheable region over the
mailbox (region 9 on the CM33, region 12 on the CM7), and the linker symbol
`__use_shmem__` makes the linker export the addresses it needs. Both halves are
required. That also replaced a hand-rolled MPU override I had added to the CM7
port the round before - which duplicated this and used the very region number the
SDK's path programs.

### What made these separable

Two diagnostics, and neither was optional in hindsight:

- **A beacon in the CM7's own ITCM**, read back through the same `0x303C0000`
  window the image is copied through. The mailbox is shared, cached differently
  on each side, and the thing most likely to be broken - so every diagnostic
  built on it fails with it, and "never started" looks exactly like "running and
  blind to the mailbox". The beacon told those apart in one reading:
  `beacon=0x43374245 bticks=248402 bdelta=47954 saw_magic=0` is a core that is
  very much alive and cannot see its mailbox.
- **`SNIFF:CM7?`**, reporting the release path's own registers. `wait_before=1
  wait_after=0`, `src_scr=0x00000001`, `initvtor=0x00000000`,
  `itcm_w0=0x20040000 itcm_w1=0x0000047D` - every one of those was correct while
  the core still would not run, which is what pointed at the clock rather than at
  anything I had written.

The link also had to be made self-healing along the way: the capture core raised
`cm7_alive` once at boot and `bb_link_init()` clears it, so reloading the
controller - the thing one does constantly, and what a watchdog or USB reset does
- orphaned a healthy capture core permanently.

### A regression this work introduced

`api_test` still scores 123/25, but two of those failures changed character and
that is a regression, not a wash:

```
bb_i2c_read with no device      expected no acknowledge, got bus or peripheral error
bb_i2c_transfer with no device  expected no acknowledge, got bus or peripheral error
```

They reported a clean NACK before this change set and now report a bus error -
arbitration-lost, bit-error or pin-low - on the same empty bus. Reproducible
across runs, so not a leftover from the capture self test driving pads.

Two candidates, both introduced here and neither yet eliminated:

- **`Prepare_CM7()` writes `PHY_LDO->CTRL0` and `DCDC->REG3`.** It has to, to
  bring up the CM7's power domain, but those are rails - and an I2C bus held up
  only by ~40 kOhm internal pull-ups is exactly the kind of thing that notices a
  changed rail or slew.
- **The controller now runs from QSPI flash rather than TCM.** Instruction
  fetches go through a cache over external flash, which changes the timing of
  the LPI2C driver's polled loops.

Both mean "nothing on the bus" in practice, so the impact is small - but the
distinction between NACK and a bus error is one this firmware deliberately makes
elsewhere, and it should not have moved.

### Two open details

- **`SNIFF:TEST`'s `expected` count is a floor, not a target.** The generator
  timer runs with `timerDisable = Never`, so it keeps shifting after its buffer
  drains and the pins keep toggling for as long as capture runs: 1024 loads
  driven, 21865 transitions captured. Both numbers are right; comparing them for
  equality was not. The pass test is now "captured transitions and lost none",
  which is what this test can actually establish.
- **Only one of the two window pins toggles.** `state` alternates 0 and 1, so
  tap `d4` (FlexIO1 pin 14) moves and `d5` (pin 15) stays low, although the
  generator pattern drives the whole 2-pin window. Either pin 15's pad mux is not
  taking or the parallel transmit is not driving both. It does not affect the
  engine - the capture is correct for the pin that moves - but a two-wire
  protocol needs both, so it is the next thing to chase.

## The second channel, and the end of the power cycles

Both fixed. Two channels now capture:

```
SNIFF:TEST -> OK driven=64 seen=180 overruns=0 moved=11 pins=11 taps=2
               sample_hz=6000000 pwidth=0 timcmp=0x00003F03
```

```
META:    proto=01 i2c  channel_count=02  ps_per_tick=166666
SAMPLES: dt=5,state=3   dt=24,state=0   dt=67,state=3   dt=9,state=0 ...
```

`state` alternates 0 and 3 - both channels moving together, which is what the
generator drives. The uneven dt values are that generator's software timing, not
jitter in the capture.

### Parallel receive was the wrong mechanism, and it took measurement to say so

The engine used one shifter with `PWIDTH` set, sampling a window of consecutive
pins. It only ever captured the pin at its pin select, and three separate
measurements were needed before that was a fact rather than a suspicion:

| Measurement | Result | What it ruled out |
|---|---|---|
| Per-pin drive/read through FlexIO's `PIN` register | `pins=11` | the pad, the mux, SION - both pads work |
| Sampler with one window pin driven at a time | `state_lo=1 state_hi=0` | wiring; the sampler genuinely does not see the second pin |
| `SHIFTCFG` / `SHIFTCTL` / `TIMCMP` read back | `pwidth=1`, receive, PINSEL=14, 6 MHz | a field that did not take - the configuration was correct all along |

Everything correct and the second pin still unsampled. So each tap now gets its
own ordinary serial receive shifter, all driven by the one timer - the
configuration every SPI, UART and I2S driver in the SDK uses. They shift on the
same edges and their status flags assert together, so a poll that reads all of
them reads one instant across every channel; `cap_poll()` requires *all* tap
shifters to be full and reads them in a single pass, because draining one at a
time would let the channels slide apart by a word and produce a capture that
looks plausible with the channels skewed in time.

The cost is a shifter per channel, so the tap count is now bounded by the
peripheral's shifters rather than a window width. 32 samples per word instead of
16 two-bit groups, so the intake rate also halves for the same sample clock.

### The self test was testing one channel out of two

Worth separating from the bug it hid. `seen > 0` was the pass criterion, and one
working channel satisfies it - so a completely dead second channel read as
success for several rounds. The generator was a transmit shifter, which drives
the single pin at its pin select, so it could not have exercised the others
anyway.

It is now this core toggling *every* tap pin together through FlexIO's software
pin output, and the pass test is per channel: `moved` must have a bit for every
tap. The trade is that a software delay loop is not a clock divider, so the rate
is approximate - which is why nothing in the reply claims a bus frequency and the
capture's own timestamps carry what was actually achieved.

Two other self-test mistakes of mine, both of which reported as engine failures:

- The per-pin probe ran *while* the sampler was live. It deliberately stalls for
  microseconds waiting for pads to settle, which overruns a shifter filling every
  couple of microseconds. It now runs before capture starts.
- After the probes I hand-reset the counters but not `running` or `full` - and
  `push_sample()` stops the hardware when the ring fills, so a probe that tripped
  that left the sampler disabled and the generator run captured nothing.
  `cap_start()` now does the restart, because it is the one place that knows
  every piece of state that has to be consistent.

## Reloading without a power cycle

`scripts\reload.ps1`. This is worth its own section because it was costing a
physical power cycle per iteration.

An XIP image has to be entered by the boot ROM, since the ROM is what configures
FlexSPI - and a debug reset does not redo that. `LinkServer flash load` programs
the image and leaves the core halted in ROM; connecting a gdbserver resets but
still does not re-boot from flash. So for a long stretch the only way to run a
freshly programmed image was to physically power the board.

`LinkServer run <device> <elf>` does both halves: programs *and* resets the
target into the image. Verified rather than assumed - after it, `SYS:STAT?`
reported `uptime_ms=35007` on a board that had been up far longer, and the
capture core was back to not-started. It then sits monitoring the target, which
is not wanted, so the script kills it once the board enumerates; the board keeps
running because it executes from its own flash and LinkServer was only watching.

A bootloader would also have solved this, and is still the right answer for
field updates without a debugger - but it is a separate piece of work, and for
the development loop this is a two-line script instead.

## Host conformance: 139 of 142

From 125/23 to **139 passed / 3 failed**, stable across eight consecutive runs.
The three that remain are one hardware fact and two board differences, described
at the end; everything else is fixed.

The pass count did not move when the last two failures cleared, and that is not a
miscount: those two checks are bare `fail()` calls rather than `expect_ok()`, so
they print a line when they fail and nothing when they pass. Fixing them removed
two lines from the total instead of adding two passes.

### A regression I reported that was not one

I recorded earlier that `Prepare_CM7()`'s PHY_LDO/DCDC writes or the move to XIP
had turned two I2C tests from a clean NACK into a bus error. **That was wrong.**
Run directly, every I2C path reports NACK correctly - plain reads, `I2C:MEMR`,
`I2C:WR`, and the whole api_test sequence replayed by hand including the raw
`MCCR0` write and the 112-address scan.

What those two runs had in common was that they immediately followed
`SNIFF:TEST`, which drives pads and leaves the capture core running. That is the
disturbance. Worth knowing as an interaction, and worth correcting plainly: I
attributed it to rails and a linker change on the strength of two adjacent runs,
which was not enough evidence for the claim.

### The UID was reading the wrong peripheral

`bb_uid_word()` read `0x47510C00`. `OCOTP_FSB` is at `0x47518000`, and the UID
lives in `OTP_SHADOW_PARTA[12..15]`. So every word came back zero - and nothing
complained, because a UID of zeros looks exactly like a part whose fuses are not
programmed. It quietly cost the USB serial number, which is derived from it.

Now read through `SILICONID_GetID()`, and the result shows up in three places at
once:

```
SYS:ID?  -> uid=23135000CBB968993B306018975A6193
Windows  -> USB\VID_1209&PID_0003\975A61933B306018CBB9689923135000
bench    -> USB bulk, board CBB968993B306018975A6193
```

The device now advertises a real serial number instead of falling back to a
port-derived instance id, which is what makes two of these boards usable on one
machine.

**The chip revision genuinely is not available.** `checked` stays false, and now
for a stated reason rather than a TODO: the register that would answer it is
`ANADIG_MISC`'s `MISC_DIFPROG`, and while `PERI_ANADIG_MISC.h` exists and defines
the fields, MIMXRT1189 never instantiates the block - no base address, no
pointer. Reporting the build's own expectation as a measurement would defeat the
point of the flag.

### The capture core now starts itself

`bb_link_init()` starts it, which was always the plan - the note left when
`SNIFF:CM7` was made a deliberate command said so: *"When the capture image is
embedded and copied by this core, that changes and the call can move into
bb_link_init()."* That is now true, so it moved.

The result is ignored on purpose. A controller that refused to finish booting
because the capture core did not come up would take USB, the console and all five
bus drivers with it, for a subsystem that reports its own absence perfectly well.

Fourteen host failures were nothing but this: api_test never calls `SNIFF:CM7`,
so every `SNIFF:*` command answered "capture-core-not-running" on a board that
was entirely capable of it - which is exactly how a host library and a GUI see it.

### Four things that refused instead of doing the obvious

A pattern, and worth naming as one. Each of these read as the careful choice and
each made a legitimate request fail for a state the caller could not see:

| Was | Now |
|---|---|
| `SNIFF:START` on a running capture → BUSY | restarts it |
| `BB_MSG_CONFIG` on a running capture → BUSY | stops, then reconfigures - the new config may use different pins and a different rate, so continuing under the old one would produce records the metadata no longer describes |
| `SNIFF:TEST` with no taps named → PINS | takes the protocol's default taps |
| `SNIFF:TEST` on a running capture → BUSY, then (worse) stopped it | injects into that capture and leaves it running |

The last one took two attempts in opposite directions. Refusing broke the
inject; stopping broke the caller's later `SNIFF:STOP`. The sequence a host
actually uses is `START`, `TEST`, `READ`, `STOP` - inject a known pattern, then
read what the capture made of it - so the self test now owns the capture only
when nothing else does, and the per-channel `moved` check applies only in that
case: injecting into someone else's capture cannot be held to a scope that was
not ours to set.

Default taps also needed care: a protocol given to `SNIFF:START` re-derives them,
because SPI needs four where I2C needs two and carrying two into SPI would
configure a channel that cannot decode.

### Two unbounded waits, of the same family as the I2C one

`UART_RETRY_TIMES` now bounds every polled wait in `fsl_lpuart.c`, for a specific
reason. `LPUART_WriteBlocking()` waits for the transmitter to drain; with flow
control on and nothing wired to CTS the pad pulls high, `CTS_B` is active low, so
that reads as "peer not ready" and the transmitter correctly refuses to send. The
wait then never ends. Found precisely that way: `UART:CFG` with `rtscts`
succeeded and the *next* command timed out, because the reply to it could not be
transmitted. `uart_send()` returns a status now and the callers report
`tx-timeout cts-not-asserted`.

The link had the same shape of bug. `bb_link_request()` used a flat 50 ms, which
is right for a configure or a drain and badly wrong for the self test and the
bench - `bench_run()` calls `selftest_run()` twelve times. `SNIFF:BENCH` timed
out, the timeout marked the link down, and every `SNIFF:*` after it reported
"capture-core-not-running" until the 500 ms re-probe. The capture core was
healthy the whole time; its beacon never stopped. Those two operations now get
8 s.

And one in the self test itself: the generator's wait between edges idled instead
of collecting. At a 20 us half period that is ~12 us of nothing while the
shifters complete a word every 5.3 us, so they overran and the test failed on
`overruns != 0` - the engine keeping up fine and the test refusing to collect
from it. The wait now calls `cap_poll()`.

### UART hardware flow control works, and my reason for refusing it was wrong

`UART:CFG ... rtscts` used to answer `rtscts-pads-not-on-headers`, on the basis
that LPUART4's flow-control pads were not brought out. They are:
`GPIO_EMC_B1_14` carries `LPUART4_CTS_B` and `GPIO_EMC_B1_15` carries
`LPUART4_RTS_B`, and both are header pins - D4 on J1-10, D5 on J1-12.

They are shared with sniffer taps d4 and d5, and that is declared rather than
hidden: asking for flow control takes those pads from the capture engine, because
a pad serves one peripheral at a time. The alternative is worse -
`LPUART4_RTS_B` is otherwise only on `GPIO_EMC_B1_22`, which is D13, which is
`spi.sck` - so sharing with one subsystem beats splitting the conflict across
two. The spec allows exactly this.

Nothing is un-muxed when flow control is turned off, and that is deliberate: the
first version handed the pads back as GPIO inputs and hung the controller on
every `UART:CFG` *without* `rtscts`, which is the common case and the first thing
a host does. Testing only the rtscts path is what let that through. Handing them
back is not needed anyway - `push_config()` muxes the taps it wants when the
sniffer claims them.

### Two of my own fixes made it worse and were reverted

Both are worth recording, because both were me chasing a test rather than fixing
a defect:

- **Renaming `i3c1.*`/`i3c2.*` to `i3c.p1.*`/`i3c.p2.*`** to satisfy the test's
  `i3c.` prefix took the suite from 125/23 to **110/189**. Those signals share
  pins with `i2c2.*`, and the *header text* (`J3-or-i3c1`) is what declares the
  sharing to the host. Renaming the signal without the header broke ~150
  duplicate-pin checks - a check that matters considerably more than a prefix.
- **The flow-control un-mux path**, above.

### The five that remain, and why each is not a defect

- **`core clock: expected 250 MHz, got 240000000`** - the test hardcodes the
  STM32H5's clock. The CM33's maximum is 240 MHz; reporting 250 would be a lie.
- **`bb_pins: expected the whole map`** - the test looks for a signal prefixed
  `i3c.`, which assumes one I3C bus. This board has two, published as `i3c1.*`
  and `i3c2.*`. See above for what renaming them costs. Prefix filtering with
  `i3c` still matches both, which is what the GUI does.
- **`bb_pins filtered: expected two sniffer taps`** - the test expects the
  STM32's two. This board publishes seven.
- ~~**`capture the self test: the decoded event stream is wrong`** and
  **`expected address byte 0xA0`**~~ - **now implemented and passing.** See
  "Decoded events" below.

### One cost worth stating

The capture core now runs from boot, and it polls the mailbox in a
non-cacheable loop. That is constant OCRAM traffic, and the transport figures
moved with it: **9555 round trips/s and 148.8 kB/s**, against 9889 and 157.7
measured with the core halted. About 5%, for a subsystem that is now always
available. Both numbers are from the STM32 tree's own `bench.exe`.

## Decoded events: the second capture mode

The spec describes two capture modes and only one existed. There is now an I2C
decoder on the capture core, fed from the same intake that fills the sample ring,
so a host can read the waveform with `SNIFF:RAW?` and the transaction with
`SNIFF:READ?` and the two cannot disagree - one pass, one source of truth.

```
SNIFF:TEST 20000 -> OK driven=78 seen=77 sample_hz=6000000 overruns=0 moved=11
SNIFF:READ? 64   -> OK n=6 more=0 S AA0+ D11+ D22+ D33+ P
```

The self test drives a real frame now - START, address 0xA0, three data bytes each
acknowledged, STOP - rather than a square wave, because a square wave proves the
sampler saw both channels and proves nothing about a decoder. A passing self test
therefore exercises pads, sampler, intake, decoder and the token rendering on the
other core, end to end.

### I took a command that already meant something

The first version of this returned raw record bytes from `SNIFF:READ?`. That
command already had a meaning - decoded event tokens, parsed by `bb_sniff_read()`
and `bb_sniff_token()` since the STM32 build - so redefining it broke every host
built against the protocol while looking like new capability. The spec is explicit
and I had read past it: *new capability arrives as additional commands, never as
changes to existing ones*. `SNIFF:READ?` is back to what it always meant and the
raw records moved to `SNIFF:RAW?`.

### Two bugs, and both had been passing the self test

**The sample order was reversed.** `CAP_OLDEST_IS_HIGH_SLOT` was 1, and the
comment justifying it described how a *parallel* receive shifter loads whole
groups at the low end. That was true of the parallel design this engine started
as and false of the per-tap serial shifters that replaced it - the constant
outlived the reasoning that set it, and nothing rechecked it during the redesign.

The symptom is worth naming because it is not a dead capture. Every 32-sample word
came out time-reversed, so the stream sawtoothed backwards at each word boundary
and re-crossed every real edge two or three times: 206 transitions for 78 driven
edges. The waveform still looked like a waveform. The decode was nonsense -
`S P S A63- D00+ ...`, eleven data bytes where three were sent.

**`dec_reset()` was spending a real transition on initialisation.** The decoder is
fed transitions, and a fresh decoder used the next one to establish its reference
level. On an idle-high I2C bus the next transition *is* the START, so the START was
swallowed, `in_frame` never set, and every clock edge after it was discarded: the
frame decoded to nothing at all. The initial level is not the decoder's to guess -
the intake already tracks it, because the sample contract makes the first record
the initial state rather than a transition - so it now comes from there.

### The self test was already computing the check that would have caught it

`out->expected` - one transition per driven edge - has been in the reply and on the
wire the whole time, and nothing ever compared it against `seen`. The only
questions the test asked were whether *anything* was seen and whether every channel
moved, and time-reversed samples satisfy both. So a reversed sample order passed.

That is the second defect this test has waved through by asking too little of
numbers it already had; the first was a dead second channel passing because
`seen > 0`. It now asserts an invariant rather than a tolerance: both generators
write the pin data register once per counted edge and the pins hold constant
between writes, so the sampler cannot observe more transitions than there were
writes. 206 against 78 fails that immediately.

## Four taps: a real ceiling, found through three wrong theories

A four-tap SPI self test timed out at 8 s and left every following command
reporting "capture-core-not-running ... load-the-cm7-image". It was none of those
things. `SNIFF:LINK?` reported **phase 0x70** - the run had *completed*, and the
reply was merely late.

Worth recording what the misleading report cost, because the message sent the
search to the wrong subsystem twice over. A timeout returned
`BB_CAP_ERR_UNSUPPORTED`, so a *late* answer was reported as an unimplemented one,
and because a timeout also marks the link down the reply advised loading an image
onto a core that was running and answering its beacon. Both now say what they
mean, and a timeout quotes the beacon's tick count and phase rather than
interpreting them.

Then the numbers, once a failed self test was made to report the measurements it
already returned:

```
4 taps @ 6 MHz  = 24 M samples/s -> overruns=7974, ~4% of words lost
4 taps @ 4 MHz  = 16 M samples/s -> overruns=0 over 1.4 M samples
2 taps @ 6 MHz  = 12 M samples/s -> overruns=0
```

`moved=0xF`, `pin_probe=0xF`, `seen=64` for `driven=64`: **all four channels
capture perfectly.** The engine was choosing a sample rate its own intake could
not consume, then failing on `overruns != 0` and reporting
"flexio-config-rejected" - which names the wrong subsystem again. The FlexIO
configuration was correct.

The cost per sample is per channel, so the ceiling belongs on `sample_hz *
tap_count`. 16 M samples/s was the budget when this was written; a later sweep
with the FlexIO root raised bracketed the knee properly and moved it to **15 M** -
see "The FlexIO clock was never configured" below. Two channels stay at
6 MHz exactly where they were; four drop to 4 MHz and pass. This is the polled
intake's ceiling, not the hardware's - eDMA is what raises it.

Three of my theories were wrong before the measurement: a `words[]` overrun (the
array is `[8]`), a per-protocol sample rate (`nominal_hz` is 400 kHz for every
protocol), and per-tap array overflow (also `[8]`). The phase markers settled it
in one build.

### A drain loop that waited for the bus to go quiet

The self test's tail drain ran up to 20000 polls and stopped when a poll produced
no new records - that is, when the bus went quiet. Releasing the taps leaves pads
with no pull floating, and floating pads keep transitioning, so it ran the full
20000, and each poll drains up to 64 word-sets. Over eight seconds of work waiting
for something that was never going to happen.

"Until the bus is quiet" is the wrong question for a sniffer in any case: this
engine exists to capture buses that are busy, and on a live bus that loop would
have taken its worst case every single time. What is wanted is the tail of the
generated pattern, which is at most a word-set or two.

## Shared pads were taken and never handed back

Sniffing MDIO's pads left MDIO broken until the board was reset, and it failed
quietly in the worst way - a floating read returned `0x0000` instead of `0xFFFF`,
which is a perfectly plausible register value and indistinguishable from a PHY
that answered.

**Which subsystems actually share pads, because I got this wrong once.** I wrote
that this affected I2C too, and named `a0`/`a1` as "i2c1's own pads". They are not:

```
taps    d4/d5 = EMC_B1_14/15   d8/d9 = EMC_B1_17/16   a0/a1/a2 = AD_15/16/17
MDIO    MDC   = EMC_B1_17 (d8)  MDIO = EMC_B1_16 (d9)   <- shared
UART4   CTS   = EMC_B1_14 (d4)  RTS  = EMC_B1_15 (d5)   <- shared
I2C     AON_09/08, AON_16/15, EMC_B2_19/20              <- shares nothing
```

`AD_*` and `AON_*` are different pads on this part, which is what I skipped over.
No I2C bus overlaps a tap, so no capture configuration can take an I2C pad. The
I2C re-claim below is therefore symmetry and cheap insurance, not a fix for an
observed conflict - and the I2C failure I originally attributed to pad theft was
the latched LPI2C fault described in the next section, which is a completely
different cause with a completely different fix.

Muxing at configure time is not enough on a board where pads are shared. The rule
the rest of the board already follows is that whoever is about to use a pad claims
it - the capture engine muxes taps when it claims them, the UART does the same for
flow control - and I2C and MDIO were not holding up their end. Both now re-mux
their own pads per operation, which is a couple of register writes against a
transaction measured in microseconds.

## One NACK could take the I2C bus down until reset

The suite degraded run over run - 3 failures, then 4, then 5 - and a reload
cleared it. `bb_i2c_transfer` reported a bus error where the target had simply
never answered.

The LPI2C halts while a sticky error flag is set. On a write NACK the code issued
`LPI2C_MasterStop()` to release a bus left open for a repeated start, but with
`FEF` latched the controller would not execute it - so the STOP never completed and
the flag stayed set, and every following transfer reported the *new* transfer as
broken rather than the old one.

It presented as frequency-dependent and is not:

```
before:  bus 2 @ 400 kHz -> 1 read-nack, then 19 write-fifo-error
         bus 3 @ 400 kHz -> 1 read-nack, then 19 write-fifo-error
after:   both @ 400 kHz  -> 20 read-nack, 20 of 20
```

Both buses, which is what ruled out a pull-up or a marginal bus. At 100 kHz the
NACK path recovered on its own, which is why it hid: `bb_i2c_config()` leaves
400 kHz behind, so the *next* suite run started there. Every error path now leaves
the controller in a known state, and the STOP is sent only when the bus is
actually still held.

## Failed self tests now report their measurements

The capture core fills the whole diagnostic block before it decides pass or fail
and returns it either way, and the controller threw it away on failure and printed
the bare error code. That is backwards - a self test that passes needs no
explanation, and the one that fails is exactly the one whose numbers are worth
having. Diagnosing the four-tap failure above meant adding temporary
instrumentation to recover figures the reply already carried.

## The FlexIO clock was never configured, and I quoted the default as a limit

I told the user the sampling ceiling was 24 MHz and called it a hardware ceiling.
It was a divider choice, and not even ours.

`cap_src_hz()` only ever *read* `kCLOCK_Root_Flexio1/2`; nothing in this project
set it. What it returned was the SDK board config's choice — `PLL_480_DIV2`
(240 MHz) with `div = 5`, which is 48 MHz. Divided by FlexIO's own minimum timer
divider of 2, that gives the 24 MHz I reported. None of it was silicon.

**IMXRT1180RM Rev 10, Table 135 (Clock Roots)** is the authority:

```
flexio1_clk_root / flexio2_clk_root
  Max Freq   120 MHz (HSRUN)   120 MHz (RUN)   60 MHz (UDRUN)
  mux  00 OSC_RC_24M   01 OSC_RC_400M   10 PLL_480_DIV2   11 PLL_1G_DIV5
```

So the root maxes at 120 MHz — reachable from the mux the board already selects,
by changing `div` from 5 to 2. `CAP_ROOT_DIV` now does that, and the self test
reports it:

```
src_hz=120000000       (was 48000000)
sample ceiling         60 MHz per channel, up from 24 MHz
```

The user's instinct was right and my number was an artifact I never questioned.
Worth recording as a pattern rather than a one-off: this is the third time in
this port that a value I read out of a working system got quoted as a constraint
— the others being the reversed `CAP_OLDEST_IS_HIGH_SLOT` comment that outlived
the design it described, and `-DCMAKE_C_FLAGS` "not working" when the real fault
was an unset `ARMGCC_DIR`.

### What the clock raise does and does not buy

It does **not** raise the usable rate today, and that is the important half. The
binding limit is the CM7's polled intake, measured by lifting the clamp and
sweeping the requested rate now that the finer divider grid can bracket a knee:

| taps × rate | total | result |
|---|---|---|
| 2 × 6.000 MHz | 12.0 M/s | overruns 0 |
| 2 × 6.667 MHz | 13.3 M/s | overruns 0 |
| 2 × 7.500 MHz | 15.0 M/s | overruns 0 |
| 4 × 4.000 MHz | 16.0 M/s | overruns 0 |
| 2 × 8.571 MHz | 17.1 M/s | **overruns 6372** |
| 2 × 60.00 MHz | 120 M/s | **never finished** |

`CAP_INTAKE_SAMPLES_PER_S` is now **15 M**, the highest total verified clean, and
it is deliberately the two-tap figure: a word-set completes every 3.7 us at
2×8.571 MHz against 8 us at 4×4 MHz, so the cost has a per-word-set term as well
as a per-sample one, and two taps is the harder case.

The 60 MHz row is the one to remember. It did not report overruns and fail — it
ground on until the link timed out at 8 s with the beacon stopped at phase `0x11`
inside the generator, because every settle iteration drained 64 word-sets that
refilled immediately. A sampler that outruns its intake badly enough stops
looking like data loss and starts looking like a hung core.

What the raise *does* buy: a finer divider grid (which is what made the sweep
above possible at all), and a ceiling that is now 60 MHz per channel waiting for
an intake that can use it.

### This reprioritises eDMA

I had eDMA on the open list as a throughput improvement. It is the gate on I3C:

```
I3C SDR push-pull ceiling     12.5 MHz   (I3C:FREQ accepts up to 12500000)
sampling needed for decode    ~4-5x      -> 50-60 MHz
shifters can do               60 MHz/ch  -> enough, now that the root is raised
polled intake can do          7.5 MHz/ch at two taps -> 8x short
eDMA moves                    2 shifters x 60 MHz / 32 samples per word
                              = 3.75 M words/s per channel, ~30 MB/s
```

30 MB/s of word moves is unremarkable for eDMA, and it is the difference between
"I3C cannot be sniffed on this part" — which is what I told the user — and "I3C
can be sniffed at full push-pull rate". The correct statement was never that the
silicon could not do it, only that the current intake could not.

## SNIFF:SRC was mislabelling provenance

`SNIFF:SRC` accepted `wire`, `self` and `both`. Only `wire` was ever implemented,
and the other two did not fail — they set a flag that was passed to the capture
core and stamped into every record header as `observed`, while nothing generated
controller-side records and nothing branched on the value.

So `SNIFF:SRC self` returned FlexIO **wire samples labelled
`observed=controller`**. The spec is explicit that this tag exists so that "no
tool can silently confuse what was inferred with what was sampled"; the mechanism
built to prevent that confusion was manufacturing it, invisibly, in any saved
capture.

Both now refuse with the reason, which is the same choice `bb_ee_i3c.c` makes for
the unimplemented I3C target:

```
SNIFF:SRC wire -> OK
SNIFF:SRC self -> ERR 8 self-not-implemented controller-side-tracing-absent
                        would-tag-wire-samples-observed=controller
```

A subsystem reporting its own absence is worth far more than one reporting
success and lying about where its data came from.

## SNIFF:BENCH was reporting the generator, not the capture

Re-running the bench after the clock change turned up two wrong numbers and one
bug, and the pattern is the same as the self test's: figures that were measured
honestly and then labelled as something else.

```
before:  cost_ns=166 max_bus_hz=17857142 dma_bytes_per_s=616
after:   cost_ns=166 max_bus_hz=1500000  dma_bytes_per_s=1804687
                     sample_hz=6000000   limited_by=sampler
```

**`max_bus_hz` was the generator's speed, not the capture's.** The sweep runs the
self test at progressively shorter half-periods and stops when it overruns - but
its floor is `half_ns >= 20`, so it stops when *this core cannot flip pins any
faster*, and reports that. 17.8 MHz is `(1e9/28)/2`: the last step before the
floor. Meanwhile the sampler was running at 6 MHz, and a 6 MHz sampler cannot
represent a 17.8 MHz bus at all - it aliases. `overruns` stayed zero throughout
because the sampler was keeping up with *itself*, and every channel still moved,
so nothing in the self test contradicted the claim.

That is the STM32 build's mistake in mirror image. There the generator was
*slower* than the nominal rate and the test reported the nominal figure; here the
generator was *faster* than the sampler and the test reported the generator.
Both publish a rate the capture cannot deliver.

The answer is now bounded by `sample_hz / 4` - two samples per bus clock is
Nyquist for the clock alone, and a decoder also needs the data line to settle
within each half cycle, so two is not enough to decode with - and `limited_by`
says which constraint bound it, because "the generator could not go faster" and
"the sampler could not follow" are different answers.

It is also tap-sensitive now, which it was not before:

| taps | sample_hz | max_bus_hz | limited_by |
|---|---|---|---|
| 2 | 6.00 MHz | 1.50 MHz | sampler |
| 4 | 3.75 MHz | 937.5 kHz | sampler |

Identical answers for two and four taps were the clue: the sample rates differ by
1.6x, so a measurement of the capture could not possibly have matched.

**`dma_bytes_per_s` was the size of one run.** `r.seen * sizeof(bb_sample_t)` -
77 x 8 = 616 bytes, reported as a per-second rate for a capture moving megabytes
a second. Nothing noticed because a small number in a diagnostic field looks
like a small number. It divides by the elapsed sample time now: 1.8 MB/s at two
taps, 3.0 MB/s at four.

**And `SNIFF:BENCH` never pushed a pending configuration.** `SNIFF:CH` records
the choice locally and marks it dirty; `SNIFF:TEST` sends it, and bench did not -
so `SNIFF:CH` then `SNIFF:BENCH` on a freshly booted board reached the capture
core with no taps and returned `pins-not-flexio-or-already-in-use`, which names a
pin fault for an unsent message and sends the reader to check their wiring.

It hid because it only fails on a *fresh* board - any earlier `SNIFF:TEST` pushes
the configuration and every bench after that works. My own first bench
measurement was taken that way, after a long run of other commands, which is
exactly why I did not see it.

## Pin-clocked capture: the bus clocks the sniffer

`SNIFF:MODE <oversample|pinclock>` — a new command, so nothing that worked before
behaves differently, and `oversample` remains the default.

In pin-clocked mode the bus's own clock line is the shift clock: `TIMDEC = 110b`,
*"decrement counter on pin input (rising edges), shift clock equals pin input"*,
with tap 0 as the timer's pin. One bit per bus clock, sampled at the instant the
bus itself nominates as valid. Verified bit-exact against the generated frame,
three runs running:

```
driven:   0xA0 +ack  0x11 +ack  0x22 +ack  0x33...
expected: 1010000000001000100010001000011
captured: 1010000000001000100010001000011   match
```

### Why it matters

```
                          max bus rate   why
oversample, polled         1.5 MHz       intake caps sampling; /4 to decode
oversample + eDMA         ~15   MHz      60 MHz sampling / 4
pin-clocked                30   MHz      FLEXIO_CLK/4, IMXRT1180RM 60.6.3
I3C's own ceiling          12.5 MHz      RM 65.3.2
```

Pin-clocked clears I3C's maximum with 2.4x to spare, and it does so while
producing roughly a fifth of the data: one bit per bus clock instead of four or
more samples per bit. A 12.5 MHz I3C bus is ~390 K words/s, against the ~470 K
the *current polled intake* already sustains — so full-rate I3C capture no longer
depends on eDMA at all. That reverses the priority I set earlier in the day, and
the reversal came from changing how we sample rather than how fast.

### What it cannot do, and why that is the bus's fault

START and STOP are SDA transitions *while SCL is high*. No clock edge exists at
those moments, so a shifter clocked by SCL cannot see them — the decoder is
disabled in this mode and `SNIFF:READ?` returns no events. `SNIFF:RAW?` carries
the exact bit stream.

That is a property of I2C/I3C framing, not a shortcoming of the configuration,
which is why this is a mode rather than a replacement. A complete high-rate
sniffer needs both: pin-clocked shifters for the bits, and something watching SDA
between clocks for the framing.

Because the mode persists like proto and taps do, a host that has never heard of
`SNIFF:MODE` can meet a board someone left pin-clocked and get a clean, empty,
unexplained answer. The read reply now names the reason in a field the existing
parser already ignores.

### Three wrong fixes before the right one

The first capture came back **bit-perfect and offset by two**. Everything after
the offset matched exactly, which is the signature of a word-boundary problem
rather than a sampling one — the bits were right, the frame was misaligned.

1. **Drive the taps to idle before `cap_start()`.** Cannot work: `cap_start()`
   calls `flexio_program()`, which runs `FLEXIO_Init()` and clears the pin
   registers, so the output enable necessarily happens after the capture is live.
   It added an edge instead of removing one — two spurious bits became three.
2. **Reset the software counters at the frame boundary**, as `dec_reset()` already
   did for the decoder. No effect whatsoever: the stale bits were inside the
   *shifter*, not the ring.
3. **Disable and re-enable FlexIO, and drain SHIFTBUF.** Also no effect. Neither
   clears a shifter's internal shift register — only writing `SHIFTCTL` does.

What actually mattered was the **timer**. It counts clock edges and loads
`SHIFTBUF` at its compare, so a timer that has already counted two edges needs
only thirty more and delivers a word of two stale bits followed by thirty real
ones. Resetting shifters without resetting the timer leaves the arithmetic
exactly as wrong as before. `flexio_program()` reprograms both — and then the
outputs have to be re-driven, *while FlexIO is still disabled*, so the re-drive
edge lands before counting starts.

Three of those four attempts produced byte-identical captured output, which I
briefly took as evidence the firmware was not being flashed. It was: changing the
driven payload from `0x11` to `0x55` put `10101010` exactly where expected. Worth
remembering as a diagnostic — when output will not move, change an input rather
than assuming the toolchain is lying.

### The last bit: contract 3, in pin-clocked form

One phantom bit survived all of that. FlexIO synchronises every pin input to its
own clock (RM 60.3.3.2) and that synchroniser starts at zero, so enabling the
module while the clock line sits idle high latches a 0->1 transition
indistinguishable from a rising edge. Exactly one, deterministically, because it
can only happen once per enable.

So the first pin-clocked sample establishes synchroniser state and is dropped —
which is the existing capture contract restated for this mode. The oversampled
path spends its first sample the same way, so that a bus already idle does not
appear to rise at t=0. Same rule, same place, different phantom.

### The framing channel, and full decode in pin-clocked mode

Pin-clocked capture cannot see START or STOP from the data shifter, because those
are SDA transitions while SCL is *high* and no clock edge exists at that moment.
It now sees them, and assembles the bits into bytes, so a pin-clocked capture
decodes a whole transaction:

```
SNIFF:MODE pinclock ; SNIFF:TEST ; SNIFF:READ?
  ->  S AA0+ D11+ D22+ D33+ P
```

Identical to what oversampled mode reports, obtained by clocking off the bus
itself rather than by sampling faster than it. Verified on every tap pair and
both FlexIO instances.

**How it works.** Two extra timers count SDA edges, one per direction -
`pinPolarity` ActiveLow inverts SDA, so a rising count on an inverted input is a
fall on the wire. Their status flags say *when* SDA moved and which way; software
reads SCL from FlexIO's pin register to say whether the clock was being held. A
fall with SCL high is a START, a rise with SCL high is a STOP. The data shifter's
bits are then assembled into bytes between those boundaries.

**The qualification is software, and that is the honest limit.** It is only
correct while the clock line still holds the level it had at the edge. START and
STOP hold SCL high until the bus moves again - at ordinary rates thousands of
poll iterations, at 12.5 MHz I3C push-pull about 40 ns, far below what polling
resolves. So **framing is reliable at moderate rates and is not at full rate**,
while the *bit* capture is hardware-exact at any rate the bus can run. That
asymmetry is the honest state of this mode and is reported in the reply rather
than left for a user to discover.

### What the hardware would not do, after eight attempts

The intended design was to let hardware do the qualification: trigger the framing
timer from SCL so it can only count while the clock is high. **A trigger-gated
FlexIO timer does not decrement on its pin input on this part.** Ungated, the
same timers count SDA edges perfectly; gated, they count nothing - at either
trigger polarity, and across eight combinations of enable, disable, reset and
compare. The RM's own TRGSEL encoding (`TRGSRC=1`, `2*N` for pin N) and the
internal trigger source were verified against the register documentation, so the
configuration matched the manual and the behaviour did not match the
configuration.

One real bug was found on the way there and is kept: **`triggerSource` must be
`Internal` for a pin trigger.** "External" reads like the right choice for a
signal arriving on a pad and actually means a trigger from outside the module;
the SDK's camera driver, which gates a pin-clocked shifter with another pin, uses
Internal. That moved the gate from "never fires" to "fires on every SCL cycle
regardless of SDA" - which was not a detection either, but was progress.

A shifter-based version came first and was abandoned after four attempts. Each
failure was a real property of shifters rather than a wiring mistake, and they
are worth recording because they recur:

1. **One shifter cannot see both directions** - it shifts on the polarity it
   selects, so `OnPositive` never sees a START. Perfect bits, zero events.
2. **Two shifters on one timer does not work either** - SHIFTBUF loads on the
   *timer's* compare, so both load on every edge and the one whose polarity did
   not match loads stale data.
3. **Draining one shifter and then the other reorders the stream** - a STOP
   processed before its own START finds the frame closed and is discarded.
4. **Strict alternation deadlocks** - one missed edge and the drain waits forever
   for a direction that will not come.

All four were fixed and the STOP still never appeared, with instrumentation
showing exactly one SCL-high sample where the frame contains two. The decisive
experiment was driving a frame with *no STOP at all*: the single event persisted,
which identified it as the START and ruled out every hypothesis about ordering.

### Three ordering problems between two clock domains

Framing events and data bits arrive on completely different schedules - a timer
flag is known at once, a shifter word only when 32 bits have accumulated - and
each mismatch cost a piece of the transaction:

- **The STOP arrived before the bytes it terminates.** Pushing it on detection
  put it ahead of its own frame's last bytes *and* closed the frame, so the byte
  assembler discarded them: a four-byte transaction decoded as three with the
  STOP in the middle.
- **Releasing it on the first empty shifter was too early.** A data shifter is
  empty between every pair of words, so that closed the frame mid-transaction and
  lost the byte straddling the word boundary.
- **Holding it until the bits ran out was too late.** Whatever the bus clocked
  next assembled into bytes that were never in the transaction - the self test's
  own flush clocks decoded as `D7F- DFF- DFF-`.

The rule that satisfies all three is to let the byte in progress finish and close
the frame on the next byte boundary: everything the STOP terminated is decoded,
and nothing after it is.

The generator also drives clocks after the frame to push the data shifter's tail
out, because a shifter emits whole 32-bit words and this frame is 45 bits. Those
clocks carry no data and fall outside the frame, so they cost raw samples and
nothing else. On a live bus the same tail exists and the next transaction's
clocks push it out; a bus that stops mid-word holds its last few bits until it
moves again.


### Reported honestly

A bus-clocked capture has no sample rate of its own, and none is invented.
`timebase_set()` substitutes 1 MHz for a zero argument — a sensible default for a
capture that needs *some* timebase, and a wrong answer to publish as a
measurement — so the fields are set directly and `sample_hz=0` travels with
`mode=pinclock`, telling a host the ticks are bus clocks rather than sample
periods.

The self test's per-channel check also had to move from counting taps to counting
*shifters*: pin-clocked mode spends tap 0 on the timer, so a two-tap capture has
one channel that can move, and asking for two failed a capture that was working.

## Sniffing a live bus, on two boards, at last

The distinguishing feature of this port - watching a bus while something else
drives it - is proven on hardware. A NUCLEO-H533RE running the STM32 BusBridge
controller drove I2C; the RT1186 captured it through tap wires and decoded it.
Every result cross-checks against what the driving board independently reported:

```
NUCLEO drove                        RT1186 captured        check
I2C:R 0x50 2        -> nack         S AA1- P               0x50<<1|1, NACK agrees
I2C:W 0x50 DE AD..  -> nack         S AA0- P               write bit; and correctly
                                                           no data after a NACKed address
I2C:PROBE 0x20/3C/68                S A40- P S A78- P
                                    S AD0- P               0x40 / 0x78 / 0xD0
```

Until now every capture in this project had been of a pattern this firmware
generated itself, on pads it drove itself. This is the first time it has measured
something it did not produce.

### The default sample rate was too high, and the self test could never have found it

The first full sweep failed everywhere above 100 kHz, and failed *systematically*
rather than randomly - which is what made it worth chasing. The first transaction
of each pair decoded perfectly every time; the second lost its address every
time.

```
400 kHz bus, two taps, ten runs each:
  6.0 MHz sampling (16x nominal, the default)   0/10 exact, overruns
  5.0 MHz sampling                              5/5  exact, clean
  4.0 MHz sampling                              5/5  exact, clean
```

So the intake budget was wrong, and wrong in the direction that looks like
success in a lab. `CAP_INTAKE_SAMPLES_PER_S` was 15 M, measured by driving the
self test's own pattern with nothing else competing for the core. Against a real
bus - a second board mastering, USB traffic flowing, the console being polled -
the limit is **10 M**, a third lower. The self test could not have found this,
because the conditions it measures under are not the conditions the product runs
in.

The load also depends on transition density rather than sample rate alone: 12 M
samples/s was clean at a 100 kHz bus and lossy at 400 kHz, because the faster bus
pushes more entries through the ring for the same rate. A single samples/s figure
is a proxy, not a model, and it is now set where the proxy holds for the busiest
case measured.

With the budget corrected, the default rate derivation clamps itself and the
whole matrix passes:

| mode | 100 kHz | 400 kHz | 1 MHz |
|---|---|---|---|
| oversample | **10/10** | **10/10** | **10/10** |
| pinclock | framing only | framing only | framing only |

1 MHz had previously failed at every sample rate tried; it passes now because the
clamp picks a rate the intake can actually keep up with rather than the highest
the divider allows.

### Pin-clocked mode needs traffic with something in it

`pinclock` reports framing and no bytes on this bus, and the reason is
structural rather than a defect. With no target fitted, every transaction NACKs
at the address and is nine bits long - and a shifter emits nothing until
thirty-two bits have accumulated, so the bits never reach software. The same mode
decodes a complete transaction when there is one to decode:

```
SNIFF:MODE pinclock ; SNIFF:TEST ; SNIFF:READ?
  ->  S AA0+ D11+ D22+ D33+ P
```

That is the self test's 45-bit frame, captured bit-exact off its own clock. The
limitation to state plainly is that pin-clocked capture has bit-granularity
latency: transactions shorter than a shifter word produce framing without bytes
until more traffic arrives.

### What the wiring actually is

Worth recording because it took a while to establish, and both boards had to be
interrogated to be sure:

```
connected:      NUCLEO bus pins  <->  RT1186 tap pins        (traffic decodes)
not connected:  RT1186 bus pins  <->  anything               (zero tap edges)
```

Evidence, from both ends: the RT1186's own I2C produced **zero** tap transitions
on every one of its three buses; the target image's transaction counters stayed
at zero through all of it; and its I3C ENTDAA returned
`protocol-ce2-no-ack-for-7E`. Meanwhile the NUCLEO's traffic decoded cleanly on
the same taps. So the RT1186 can watch this bus but cannot yet drive it, and the
RT1186-master/STM32-target direction remains untested for that reason rather than
any firmware one.

### Two boards on one host

Both boards present the same WinUSB interface GUID, and the first `*IDN?` of the
session came back `BUSBRIDGE,NUCLEO-H533RE` - the test client had silently been
talking to the wrong board. It now selects by device path.

The same applies to `api_test.exe`, which calls `bb_open(0)` and takes whichever
enumerates first: with both boards attached it tests the **NUCLEO**, where it
scores 141/0 on its native board. The RT1186's own 139/3 stands from runs made
with only that board attached. The host library does expose `bb_find_devices()`
and `bb_device_serial()`, so selecting a board is possible - the test just does
not do it.

## I3C, end to end, on two boards

The RT1186 masters an I3C bus, discovers a real target on another board, and
captures the traffic. This is the configuration the port was built for.

```
RT1186 I3C:ENTDAA  ->  OK targets=1  09:bcr=2E,dcr=C6,pid=010413811000
NUCLEO target      ->  i3c=r11984/w0/t749
```

A dynamic address assigned by the controller, and the target's own counters
moving to confirm it from the other end.

> **Corrected later in the port - the rates in this section are wrong.**
> `I3C:FREQ` accepted any number and reported it back unchanged, so "1, 4, 8 and
> 12.5 MHz" is a list of *requests*, not of bus rates. The divider is
> `FCLK / 2 / (PPBAUD + 1)` with `FCLK = 24 MHz`, so the only rates that exist
> are 12, 6, 4, 3, 2.4, 2 MHz and down: "8 MHz" ran at 6 and "12.5 MHz" ran at
> 12. Measured properly afterwards, ENTDAA is 3/3 at 1, 2, 3 and 6 MHz and 0/3
> at 12 MHz. See "I3C: six bugs, none of them where they looked" below.

### Capture at full rate

Pin-clocked capture keeps up with all of it:

| I3C rate | pin-clocked | overruns |
|---|---|---|
| 1 MHz | 127 bits | 0 |
| 4 MHz | 127 bits | 0 |
| 8 MHz | 127 bits | 0 |
| **12.5 MHz** | **127 bits** | **0** |

12.5 MHz is I3C SDR's ceiling on this part (RM 65.3.2), so the bit capture is no
longer the constraint on anything. That is what the whole pin-clocked exercise
was for, and it holds where oversampling cannot: the same sweep in oversampled
mode degrades from a rich capture at 1 MHz to almost nothing at 12.5 MHz, because
the intake clamp caps sampling near 5 MHz and there is no oversampling ratio left
to decode with.

### The addresses decode; the framing does not

Oversampled at 1 MHz, the decoder gets the addresses exactly right:

```
ENTDAA   ->  S AFC+ D06- P S AFC+ D07+ Sr AFD+ D02+ ...
                AFC = 0x7E<<1, the I3C broadcast address, acknowledged
write    ->  S A12+ P          0x12 = 0x09<<1, the assigned dynamic address
read     ->  S A13- P          0x13 = 0x09<<1|1
```

Everything after the address is approximate, and the reason is the one flagged
long before this bench session: **the I3C path still runs the I2C decoder.**
I3C's T-bit in place of the ninth-bit acknowledge shifts every subsequent byte
boundary, so the payload bytes are plausible and wrong. `SNIFF:RAW?` carries the
exact bits; `SNIFF:READ?` should not be trusted for I3C payload until a real I3C
decoder exists.

Pin-clocked framing degrades at these rates too, and that is by construction: the
"was SCL high when SDA moved" test is software-qualified, and at 12.5 MHz the
clock-high window is about 40 ns - far below what polling resolves. Bits exact,
framing approximate, and the reply says so.

### What the bench cost, and what it proved about the pin map

Three rounds of wiring were needed, and each failure was localised by
measurement rather than inspection - driving one end of a net and watching each
tap alone, with the sample count reported alongside the edge count so that "no
signal" could be told apart from "the sampler never ran".

The one board modification that mattered: **J31 and J35 default to pins 1-2**,
which routes `GPIO_AD_18`/`GPIO_AD_19` to the on-board EtherCAT I2C devices
(U38/U40) and leaves Arduino A5/A4 electrically dead. Moving them to 2-3 is what
made the RT1186's I3C reachable at all, and no amount of wiring to A4/A5 could
have worked before that. The symptom was diagnostic once understood: the error
moved from `no-ack-for-7E` to `entdaa-busy` when only one of the two lines was
connected, because a floating I3C SDA with no pull reads low and the controller
declines to start.

`GPIO_EMC_B2_19`/`EMC_B2_20` - D14/D15, the I2C master pins - have **no jumper**,
which is worth recording as a negative result: no board modification affects
them, so an open there is always physical.

## The full triangle: master, target and sniffer, on two boards

The RT1186 masters an I2C bus, a NUCLEO-H533RE answers as a target at 0x50, and
the RT1186 captures the exchange off the wire. A complete memory write and
read-back, decoded from the bus:

```
S AA0+ D00+ DDE+ DAD+ DBE+ DEF+ P            write DE AD BE EF, every byte acked
S AA0+ D00+ Sr AA1+ D00+ D01+ D02+ D03- P    repeated-start read, last byte nacked
```

Address, memory pointer, payload, repeated START, read data, and the closing NACK
that ends a read - all of it recovered from two tap wires, and confirmed from the
other side by the target's own counters (`i2c=r6/w3/t5`).

| mode | 100 kHz | 400 kHz | 1 MHz |
|---|---|---|---|
| oversample | **5/5** | **5/5** | **5/5** |
| pinclock | partial | framing only | framing only |

### The intake budget, measured three times and wrong twice

`CAP_INTAKE_SAMPLES_PER_S` has now been set from measurement three times, and each
correction came from testing under conditions closer to real use:

```
15 M   self test driving its own pattern, nothing else running
10 M   a second board mastering, USB traffic flowing        -> 400 kHz clean
 8 M   a real target answering, multi-byte transfers        -> 1 MHz clean
```

The last step is the instructive one. At 10 M a 1 MHz bus decoded `DBE` as `DBF` -
a single bit, in one byte, repeatably. Five samples per bit leaves no margin for a
sample arriving late, and the failure is a corrupted value rather than a missing
one, which is the kind that gets believed. Forcing 4 MHz sampling (8 M across two
taps) decoded the same transfer perfectly every time.

The cost of being conservative here is oversampling ratio at low bus rates, which
is the cheap direction to be wrong in - a 100 kHz bus is sampled 40x either way.
The expensive direction is a plausible wrong byte.

### Pin-clocked byte decode needs longer transactions than I2C provides

Pin-clocked capture is exact at the bit level and remains the only way to follow
a 12.5 MHz I3C bus, but its *byte* decode truncates on short I2C transfers: a
shifter emits nothing until 32 bits accumulate, and a six-byte transfer is 54
bits, so the tail sits in the shifter until further traffic pushes it out. Running
transfers back to back does not rescue it either - the deferred-STOP logic closes
each frame at a byte boundary, so the pieces arrive split across frames.

That is a real limitation of shifter granularity rather than a decoder bug, and
the fix is a smaller pin-clocked word rather than anything in the byte assembler.

### Three jumpers, and the bench lesson

Most of the bring-up time went to wiring, and the pattern is worth recording
because it cost hours and looked like firmware every time:

```
J31  GPIO_AD_18  -> LPI2C3_eCAT_SCL (U38/U40)   default 1-2, header needs 2-3
J35  GPIO_AD_19  -> LPI2C3_eCAT_SDA (U38/U40)   default 1-2, header needs 2-3
J45  GPIO_AON_15 -> ETH0_RST                    default 1-2, header needs 2-3
```

All three tie a pad to an on-board function and leave the Arduino/J3 header pin
electrically dead. No wiring to those pins can work until the jumper moves, and
the symptom is indistinguishable from a broken wire or a dead peripheral. `J31`
and `J35` gate the I3C pair; `J45` gates I2C2's SDA, and was the last thing
standing between a working I2C master and eight rounds of "still zero edges".

`GPIO_AON_16` (I2C2 SCL) and `GPIO_EMC_B1_*` (the taps) go straight to their
connectors with no jumper, which is why those worked immediately and made the
asymmetry so confusing.

### What made the difference: SYS:PULSE and SYS:GPIO

Diagnosing this from transfer behaviour was slow and ambiguous, because a transfer
moves both lines at once and cannot say which pin is which. Two commands added
during the session turned it into a measurement:

```
SYS:PULSE <signal> [count]   drive one named pin as a GPIO, n low/high pairs
SYS:GPIO  <signal> <0|1>     hold one named pin at a level
```

`SYS:PULSE i2c2.scl 64` producing exactly 128 edges on one tap and 0 on the other
is unambiguous in a way no protocol-level test was: one pin, one peripheral-free
toggle, a known count. Every remaining wiring question was settled by it within a
round or two, after several that were not.

Holding a line low and watching the *other board* report `i2c-read-error` gave a
second, independent channel - with the working line as a control in the same run,
so the method proved itself each time it was used.

## Closing out: what was fixed, and one defect characterised rather than fixed

### The intake budget, corrected a third time - and this failure lied

```
15 M   self test driving its own pattern, nothing else running
10 M   a second board mastering, USB traffic flowing
 8 M   a real target answering, multi-byte transfers          <- current
```

At 10 M a 1 MHz bus decoded `DBE` as `DBF`. One bit, in one byte, repeatably. Five
samples per bit leaves no margin for a sample arriving late, and the result is a
*plausible wrong value* rather than a visible gap - the kind that gets believed and
acted on. At 8 M the same transfer is 5/5 exact at 100 kHz, 400 kHz and 1 MHz.

Being conservative here costs oversampling ratio at low bus rates, which is the
cheap direction to be wrong in: a 100 kHz bus is sampled 40x either way.

### The I3C ninth bit is not an acknowledge, and now says so

I2C's ninth bit is an acknowledge driven by the *receiver*, so `+` means a device
answered. I3C SDR replaces it with a T-bit driven by the *transmitter* - parity on
a write, end-of-data on a read - so the identical `+` means something entirely
different and nothing acknowledged anything.

The byte values were never affected: both framings are nine bits and the first
eight decode identically, which is why I3C addresses were always right (`AFC` is
`0x7E<<1`, the broadcast). Only the flag's meaning changed, silently.

`SNIFF:READ?` now reports `framing=i3c-t-bit-not-ack` on an I3C capture. Reported
rather than re-rendered: the token format is the host library's contract, and a
different one for I3C would break every existing parser. Naming it in a field they
already ignore says the true thing without changing what they read.

### Pin-clocked word boundaries: found and fixed

Pin-clocked capture was corrupting one bit per shifter word. It is fixed, and the
cause is worth stating because it hid behind its own symptom.

**The load and the shift were racing on the same clock edge.** SHIFTBUF is latched
when the timer reaches its compare; with both the timer and the shifter acting on
SCL's rising edge, the latch happened before that edge's sample had entered the
shifter. So the final position of every word arrived empty and read as zero.

That is why it looked intermittent. A word whose true last bit was *already* zero
looked perfect - of four words in a test transfer, two were wrong and two were
right by luck, which reads as a flaky capture and is not one. Dumping the words
side by side against a known bit stream is what showed every word ending in a
zero:

```
word 0  last bit expected 1 -> got 0    wrong
word 1  last bit expected 0 -> got 0    right by coincidence
word 2  last bit expected 0 -> got 0    right by coincidence
word 3  last bit expected 1 -> got 0    wrong
```

**The fix is to put the reload on the opposite edge from the sample.** The timer
now counts SCL's *falling* edge (`pinPolarity` ActiveLow) while the shifters take
the negative edge of that inverted clock - which is SCL rising, exactly where I2C
and I3C define the data as valid, and half a bit period before the reload.

```
before:  159 of 162 bits, 2 wrong - both bit 31 of their word, both 1 read as 0
after:   159 of 162 bits, all correct, three runs running
```

The byte decode followed: a sixteen-byte write now decodes seventeen of eighteen
bytes exactly, the last one being the stranded partial word rather than an error.

**Three hypotheses were wrong first**, and are recorded so they are not retried:

1. *A shorter word reduces the stranded tail.* It made things worse - 8 bits
   corrupted three bytes of five, 16 corrupted one - because a shorter word
   creates more reloads and every reload was dropping a sample. The size was
   never the problem.
2. *The boundary sample is a duplicate of the next word's first.* Dropping it
   broke alignment and lost real data, so it was not a duplicate.
3. *The read races the shifter's settle.* A barrier and a delay before reading
   SHIFTBUF changed nothing and made the error deterministic rather than
   intermittent - which pointed at the hardware edge relationship instead, and
   that turned out to be right.

What remains is latency, not correctness: a shifter still emits nothing until it
fills, so a transfer shorter than 32 bus clocks produces framing without bytes,
and the last partial word waits for traffic that may not come.


## The RAM target, and one open question

### The RAM target links again

```
                     before      after
bridge RAM        100.38% *      67.11%     -Og
bridge XIP           2.01%        2.02%     -O0, unchanged
peripheral RAM      62.04%       62.04%     -O0, unchanged
capture (CM7)       13.68%       13.68%     -O0, unchanged

* overflowed m_text by 496 bytes; did not link
```

The SDK gives every `*_debug` configuration `-g -O0` and every `*_release` `-Os`
(`arch/arm/target/targets.cmake`). That is right for the XIP configurations, which
have 8 MB and use 2% of it, and impossible for `debug`, which links into 127 KB of
ITCM. `firmware/CMakeLists.txt` now appends `-Og` for that one configuration
through the SDK's own `mcux_add_armgcc_configuration`, so gcc's last `-O` wins and
`-g` is untouched.

`-Og` rather than `-Os` because this is still the debug configuration and the one a
debugger is pointed at, so the optimization that exists to stay debuggable is the
one to reach for. `-Os` is the fallback if `-Og` stops fitting.

**Correcting what this document said last revision.** It claimed that passing
`-DCMAKE_C_FLAGS=-Os` "does not work - it displaces the flags `armgcc.cmake` sets
and configuration fails". That was wrong, and wrong in a way worth naming: the
configure failed because I invoked `west build` directly instead of through
`build.ps1`, and `build.ps1` is what sets `ARMGCC_DIR`. The error was
`***Please set ARMGCC_DIR in environment variables***` at `armgcc.cmake:24` - a
missing environment variable, nothing to do with flag precedence. I read a
toolchain-detection failure as evidence about flag handling and wrote it down as a
finding.

The scope is deliberate, and it is the interesting part:

- **bridge only.** The peripheral profile fits at -O0 and the capture core has
  255 KB of its own.
- **the RAM configuration only.** Optimizing the CM7 would change the cost of
  `cap_poll()` and of the settle loops that pace the generator - which is exactly
  what the 16 M samples/s intake budget measures. It would invalidate that number
  rather than improve it.

The consequence to know about: the RAM and XIP images are now built at different
optimization levels, so they are genuinely different binaries. That is what a
127 KB budget costs.

Verified running, not just linking. The RAM image scores **124 passed / 24 failed**
against `api_test`, where every failure beyond the usual three is a `SNIFF:*` test -
that build carries no capture image, so it has no sniffer. The XIP pair still
scores 139/3 across three runs and still decodes
`S AA0+ D11+ D22+ D33+ P`.

### A hang the RAM target exposed: reading an unclocked core's memory

`SNIFF:LINK?` hung the controller outright on the RAM build. Not slow - dead: USB
stayed enumerated because that runs from an interrupt, and no command was ever
serviced again. A flash-and-reset did not clear it either, which fits a stalled bus
master not completing across a soft reset; it took a power cycle.

The beacon lives in the *capture core's* ITCM, reached through this core's window
at `0x303C0000`, and an unclocked core does not answer for its own memory. The read
does not fault and does not return garbage - it stalls the AHB indefinitely, with
no fault and no handler. That is the same trap `i2c_apply()` documents for an
ungated LPI2C, and it is worth stating twice because nothing about the expression
`b[0]` suggests it can hang.

What makes it more than a curiosity is which command hit it. The RAM target is
built without an embedded capture image, so it never starts that core - and
`SNIFF:LINK?`, whose entire job is to report whether the core is alive, read the
beacon to find out and killed the device instead of answering. A diagnostic that
takes the board down when the thing it diagnoses is absent is worse than no
diagnostic.

Every read of that core's memory is now gated on its clock, and the verdict
distinguishes absent from silent:

```
RAM build:  verdict=cm7-unclocked-no-image-in-this-build   (was: hung)
XIP build:  verdict=up   beacon=0x43374245 bdelta=10620
```

The zeros reported alongside that verdict are the function declining to read
unclocked memory, not a measurement, which is why callers ask
`bb_link_cm7_clocked()` rather than inferring it from a zero.

This was latent in the XIP build too. It never showed there because the capture
core is always started, but any failure of that start sequence would have reached
the same read.

### Transport round trips are down and I have not established why

```
now:              7767 /s, 143.1 kB/s   (spread 7252-7832 over five runs)
documented:       9555 /s, 148.8 kB/s
```

The payload figure moved about 4% and the round-trip figure about 19%, which
points at per-command latency rather than bandwidth. Round trips are `SYS:BTN?` -
a GPIO read - and nothing in this round of work touches that path.

The likely explanation is that the two numbers are not measuring the same binary:
the earlier figure predates the capture image being embedded, and the RAM build
runs from TCM while the XIP build fetches from flash. That is a plausible cause
and not a demonstrated one, so it is recorded as an open question rather than
explained away. The spread is tight and repeatable, so whatever it is, it is real
and not measurement noise.

Both figures still sit well above the STM32 tree's 5864 / 113.1 from the same
`bench.exe`.

## What to do first on hardware

1. **`scripts\flash.ps1 -Profile bridge`**, then `SYS:ID?` and `SYS:PINS?` over
   COM19. Proves the clock tree, console and command loop.
2. **Check the D14/D15 label question** (`docs/PINS.md` §4a). UM12450 labels
   J2.18 as SDA and J2.20 as SCL; the IOMUX has them the other way for LPI2C3.
   The map follows the silicon. One `I2C:SCAN` against a known device settles it.
3. **`SNIFF:*` will say the capture core is not running** until the CM7 image is
   loaded, and will then say `flexio-config-rejected` — both are the truth.
4. **USB works; the cable goes in J63** — not the MCU-Link debug USB at J23.
   `SYS:USB?` should report `dppu=480 frame=512` and `state=configured`. The
   prebuilt host tools in `E:/STMicro/busbridge/windows/build/x64/` drive this
   board unmodified — `enumerate.exe`, `bench.exe`, `api_test.exe` — because
   they search on the interface GUID, not VID/PID. The next measurement worth
   taking is `bb_winusb_hs.c` against the 9856 /s and 158.3 kB/s above.
   Note that a benchmark run while something else saturates the host CPU reads
   roughly half of that; it is host-bound, not device-bound.
5. **`bb_board_silicon()` reports `checked=false`** — it does not read OCOTP yet.
   Asserting the build's own expectation as if it were a measurement would defeat
   the point of the check.

## I3C: six bugs, none of them where they looked

Every I3C failure in this port pointed at wiring. None of them was wiring. The
symptoms were `entdaa-busy`, `no-ack-for-7E` and a target count that climbed on
its own, and the causes were six separate defects in this firmware, each of
which produced a bus-shaped symptom.

### 1. The taps were holding the bus - the one that cost the most

`bb_sniff_init()` left the tap pads at their reset default until a capture
claimed them. `d8`/`d9` are the I3C tap pair on this board, and EMC pads do not
reset high-Z, so a tap nobody had asked for was loading the I3C net from power-on.

The tell was absurd and took two sessions to notice: **ENTDAA worked only while
the sniffer was running.** Starting a capture was the one thing that set those
pads to a no-pull input.

```
sniffer stopped, taps on d8/d9 -> no-ack-for-7E
sniffer started on d8/d9       -> OK targets=1
sniffer started on d4/d5       -> no-ack-for-7E   (d8/d9 still loading)
```

That last line is what proved it: starting a capture on *other* pins does not
help, so it was never about clocks, timing or settling. Taps are parked as GPIO
inputs at init now. A tap is a passive observer of someone else's bus from
reset, or it is not passive at all.

### 2. The pad encoding was wrong for every GPIO_AD pin

There are two IOMUXC blocks and their `SW_PAD_CTL` registers are different
registers. AON has `PUE[2]` + `PUS[3]`, so a pull-up is both bits set. The main
block has one `PULL[3:2]` field where `01` is up and **`11` is No Pull**.

`pad_encode()` split on the EMC address range, which put GPIO_AD on the AON
encoding. `BB_PAD_I2C` therefore emitted `0x1E` on AD pads - `PULL = 11`, no
pull at all - on the two pads carrying I3C2. `I3C:PULL 1` reported success and
configured nothing.

The header offers no protection: `PERI_IOMUXC.h` defines `PUE`/`PUS` *and*
`PULL` over the same two bits with opposite meanings, and `DSE`/`PDRV` over bit 1
the same way. Either spelling compiles. The split is by IOMUXC instance now,
which is the thing that actually determines the layout.

### 3. A failed transfer left the controller wedged

The SDK refuses a transfer unless `MSTATUS.STATE` is IDLE or NORMACT, and
nothing in it returns the state machine to IDLE after a transfer it aborted. A
NAKed ENTDAA parks the part in the DAA state, so the *first* failure was
reported honestly and every attempt after it came back `busy` regardless of the
bus.

"Busy" reads as another controller or a stuck target. It was a stale register in
this part. `i3c_recover()` now emits STOP from NORMACT, FORCEEXIT from anything
else non-idle, and clears MERRWARN and the FIFOs.

### 4. SYS:GPIO and SYS:PULSE stole the I3C pads and never gave them back

Both deliberately re-mux a pad to GPIO - that is what makes them useful for
continuity checks - and nothing handed it back. I2C already re-claimed its pads
per operation; I3C only muxed on first configure, so every transfer after a
wiring check ran with its pins still on GPIO and read its own stale output level
instead of the bus. `port_ready()` calls `i3c_claim_pads()` now.

### 5. The target list was cumulative, so one target counted as eight

`I3C_MasterGetDeviceListAfterDAA()` reports a module-global append-only table.
Re-running ENTDAA against a single target reported 1, 2, 3 ... up to the 8-entry
ceiling, one more each pass:

```
before: {targets=1: 1, targets=2: 1, ... targets=7: 1, targets=8: 3}
after : {targets=1: 10}
```

That reads as a bus filling with phantom devices, and it is the number a host
would trust to decide what is out there. `I3C_MasterClearDeviceCount()` before
each DAA; `RSTDAA` already did this, which is what made the omission visible.

### 6. Two diagnostics that lied

`I3C:FREQ?` echoed the requested rate, so a sweep of six rates reported six
rates while the bus ran at two. It reports `act_pp`/`act_od` computed from
MCONFIG now, alongside the request - the gap between them is the interesting part.

`SNIFF:PINS?` reported `levels=00` for every tap, including taps on a bus that
was demonstrably idling high, because parking the pads on their FlexIO ALT left
`bb_pin_read()` reading an RGPIO register that does not follow a peripheral mux.
It was briefly believed here as proof that the bus was stuck low. A diagnostic
that always says zero is worse than no diagnostic.

A NAK was also always reported as `no-ack-for-7E`, which is true of ENTDAA and
false of a private transfer. A read from a target at 0x09 that nobody answered
blamed the broadcast address. It names the address actually sent now:

```
I3C:R 0x09 4  ->  ERR 5 read protocol-ce2-no-ack-for-0x09
I3C:R 0x55 2  ->  ERR 5 read protocol-ce2-no-ack-for-0x55
```

### Where I3C stands, measured

ENTDAA, 3 attempts per rate, in both capture modes, and with the sniffer off:

| requested | actual | ENTDAA |
|---|---|---|
| 1 MHz | 1 MHz | 3/3 |
| 2 MHz | 2 MHz | 3/3 |
| 3 MHz | 3 MHz | 3/3 |
| 6 MHz | 6 MHz | 3/3 |
| 12 MHz | 12 MHz | **0/3** |

A dedicated sweep also puts `PPBAUD = 2` (4 MHz) at 0/5 while 6 MHz passes,
which is not monotonic in rate and is not explained. It is not a documented
erratum - the three I3C errata for this part (ERR052086, ERR052343, ERR052344)
cover DMA FIFO reads, target early termination and absent controller clock
stalling, and none mentions MCONFIG, the dividers or DAA. Recorded as measured
and unexplained rather than tidied into a speed limit it does not fit.

A parity theory - "even PPBAUD fails" - fitted the first six points exactly and
was wrong: PPBAUD 4, 6 and 8 all pass 3/3.

### The payload path is limited by the target, and the sniffer proved it

A private write is accepted; a private read is refused, and the capture shows
which end refuses it:

```
I3C:W 0x09 00 00  ->  OK                          wire: S A12+ D00- D00- P
I3C:R 0x09 4      ->  ERR read no-ack-for-0x09     wire: S A13- P
```

`A12`/`A13` are `0x09 << 1` write and read. The controller puts both on the bus
correctly; the target ACKs the write and NAKs the read. That matches the STM32
target's own design note - a read is answered from a window loaded in advance
because an I3C target cannot stall a read - so an unloaded window has to be
refused at the address.

The NUCLEO runs the STM32 build, which this port does not own, so this is a
bench limitation rather than an RT1186 one. Worth recording that the sniffer is
what settled it: neither endpoint could say which of them was refusing, and one
capture did.

> One caution from getting this wrong first: `SNIFF:READ?` returned `n=0` for
> the same read and was briefly taken as "the controller never transmitted".
> The capture had not been stopped before reading. Stop the capture before
> concluding anything from an empty one.

### Also corrected: the target on the other board

An earlier note here read the RT tree's `bb_ee_i3c.c` - a deliberate
not-implemented stub - and concluded the NUCLEO had no I3C target, then
attributed a `i3c=r11984/w0/t749` counter line to it. The NUCLEO runs the STM32
build, which has a real 259-line I3C target; its `EE:I3C` reports
`addr=assigned` after ENTDAA and needs `EE:I3C ON` first. Two files with the
same name, one of them not the one running.

## Both sides: the I3C payload path, and framing done in hardware

With the STM32 tree opened up, the remaining I3C failures turned out to be four
more defects - three on the target, one on the controller's capture engine.

### The target counted DMA progress as a read, and never stopped re-arming

`bb_ee_i3c_poll()` decided "a read consumed the window" from
`bb_i3c_tgt_tx_consumed()`, which reports how far the DMA has got. The DMA
starts filling the transmit FIFO the moment a reply is armed, with no controller
on the bus, so the count went non-zero immediately and every pass of the pump
counted a read nobody performed and armed again. Arming flushes the FIFO and
aborts if the peripheral is not idle, so the target lived in abort-and-re-arm and
NAKed any read that arrived while it was there.

```
idle bus, before:  EE:I3C -> reads=29765 writes=0
idle bus, after :  EE:I3C -> reads=0     writes=0
```

`bb_i3c_tgt_read_frames()` exposes `tgt_reads`, which the transmit-complete path
increments - a read that actually happened. The pump re-arms on that, on a write
having moved the pointer, or on the reply being lost, and never on DMA progress.

### An armed reply made the target deaf to writes

This is the one that mattered. The target's event handler ran only while the HAL
handle was in `LISTEN` or `READY`, standing aside whenever "the HAL owns
something". But `HAL_I3C_Tgt_Transmit_DMA()` parks the handle in `BUSY_TX` for as
long as a reply sits armed - which for a target that must answer a read at any
moment is essentially always.

So every private write arriving at an armed target was skipped: not drained, not
counted, reported as an error with an empty `SER`. Write-then-read - what every
EEPROM driver does - could never work, while a write to a target with nothing
armed worked perfectly. That asymmetry is what identified it:

```
nothing armed:  I3C:TGT:RX? -> OK DE AD BE EF    writes=1 errors=0
reply armed  :  write lost                        writes=0 errors=2
```

`BUSY_TX` is now serviced. This module never arms a HAL *receive*, so the receive
path is unambiguously its own whatever the transmit side is doing.

Two consequences had to be handled with it. Re-arming an untouched reply is not
an abort - the old code saw `BUSY_TX`, called `HAL_I3C_Abort_IT()` and waited out
a millisecond on every reload, during which the next write was dropped; stopping
the channel and flushing is the whole teardown. And frame-complete for a *read*
no longer reaches the HAL, so `HAL_I3C_TgtTxCpltCallback()` never runs: a frame
that delivered nothing with a reply armed is now counted as the read it is, or
the reply stays marked armed for ever and every later read answers from the first
one's pointer.

The DMA abort also had to become unconditional. A read that took part of a window
leaves the channel pushing the rest into the FIFO, and flushing does not stop it -
it refills. Reading four bytes twice from one address returned the right bytes
once and the tail of the window the second time.

### Framing is hardware-qualified now

The capture engine qualified "was SCL high when SDA moved" in software, which
raced the bus and lost: at 400 kHz and above ordinary data edges became a stream
of phantom restarts.

```
before, 1 MHz:  S Sr Sr Sr Sr Sr Sr P S Sr P
after,  1 MHz:  S AA0+ D00+ DDE+ DAD+ DBE+ P
```

Gating the timer's *decrement* from SCL was tried before and does not work on this
part - that result stands. Gating its *enable* does: `TIMENA = 101b` starts the
timer on a pin edge only while the trigger is high, so the timer can only start on
an SDA edge that happened while SCL was held. The flag is the qualified event, and
the software read is gone.

With the word-boundary fix already in place, the pin-clocked word also drops from
32 bits to 8 - previously 8-bit words corrupted three bytes of five, and now they
are exact. The stranded tail goes from up to 31 bits to at most 7, and the reply
says `note=bits-exact-tail-truncates-under-8-bits` rather than claiming a software
qualification that no longer exists.

Recovering the last 7 bits is not possible here: FlexIO on this part exposes no
live timer counter, so there is no way to learn how many bits are sitting in a
partly-filled shifter.

### Where I3C ends up

Full round trip, both directions, confirmed on the wire and from the target:

```
I3C:W 0x09 00 40 11 22 33 44   OK
I3C:W 0x09 00 40               OK    wire: S A12+ D00- D40+ P
I3C:R 0x09 4                   OK 11 22 33 44
                                     wire: S A13+ D11- D22- D33- D44- Sr P
repeated                       OK 11 22 33 44   (stable, not stale)
target                         reads=7 writes=6 errors=0
```

ENTDAA by rate, 3 attempts each, both capture modes:

| actual pp | ENTDAA |
|---|---|
| 1 MHz | 3/3 |
| 2 MHz | 0/3 |
| 3 MHz | 0/3 |
| 4 MHz | 0/5 |
| 6 MHz | 3/3 |
| 12 MHz | 0/3 |

The capture says what fails. At 1 MHz the DAA phase completes:

```
S AFC+ D07+ Sr AFD+ D02+ D10+ D4E+ D08- D00+ D05- DB1- D09- Sr AFD- P
```

0x7E acked, ENTDAA CCC 0x07, repeated START, the target's PID, the assigned
address 0x09. At 2 MHz the repeated-START phase never completes:

```
S AFC+ D07+ D7E- ...
```

and the controller reports no ACK on the second 0x7E. That is the shape
**ERR052344** describes - controller clock stalling is not implemented on this
part, and its stated workaround for "I3C SDR write followed with repeated start
such as read" is to set the SCL frequency to suit the target. ENTDAA is exactly
that shape, and 1 MHz is what this target wants.

What does not fit that explanation is 6 MHz passing 3/3 while 2, 3 and 4 MHz fail.
It is reproducible in both capture modes and across rebuilds, and it is not
monotonic in rate, so it is recorded as measured and unexplained rather than
folded into a speed limit it contradicts. Enabling the target's internal pull-up
as well as the controller's changes nothing, so it is not drive strength.

## Can a GPIO-triggered eDMA replace FlexIO? Measured: no

The idea was to drop FlexIO and have a GPIO edge trigger a DMA channel that
copies `RGPIO2->PDIR` into a ring - capture with the core removed entirely, and
every pin of the port sampled at once. The silicon supports the mechanism:
`RGPIO_ICR[n].IRQC = 0b0011` raises a DMA request per pin on either edge,
`IRQS` picks request 0 or 1, and `kDma4RequestMuxGPIO2Request0` routes it. The
taps are all on RGPIO2, so nothing needed rewiring.

It works, and it is far too slow.

`SYS:DMACAP` arms the path and reports what the engine caught. Against a
deterministic burst - a write to an absent address is START plus nine clocks,
18 edges, at any bus rate:

| bus rate | edges/s | captured (18 expected) |
|---|---|---|
| 1 MHz | 2 M | 21, 21, 21 |
| 2 MHz | 4 M | 13, 16, 13 |
| 3 MHz | 6 M | 13 |
| 6 MHz | 12 M | 12 |
| 12 MHz | 24 M | 4 |

Complete at 2 M edges/s and already dropping at 4 M. A 12.5 MHz I3C bus needs
25 M edges/s, so the path is short by roughly a factor of six. A software-started
transfer measured 2.34 M/s from the other direction, which agrees: the per-request
cost is a few hundred nanoseconds however the request arrives.

That kills the redesign, and it is worth having killed it for the price of one
command rather than a rewrite. **FlexIO is not the bottleneck and replacing it
does not help** - the shifters already keep up; it is the per-sample work behind
them that does not.

Two things that cost time and are worth recording:

- **eDMA here will not self-trigger.** A channel linked to itself, and then two
  channels linked to each other, both sat with ACTIVE and DONE clear and no
  error, TCD read back exactly as written. The engine declines silently. Only an
  external request or a software start per minor loop actually runs.
- **The DMA cannot write CM33 TCM**, which is where ordinary `.bss` lands in this
  image: `CH_ES` destination-bus-error, zero transfers. The buffer has to be in
  OCRAM, and it has to get there through `AT_NONCACHEABLE_SECTION_ALIGN` - naming
  the section by hand gives GCC a PROGBITS section, so a zero-filled buffer
  became 2 KB of initialised data at an OCRAM address and the image stopped
  programming with "No flash configured at the specified address".

The prototype has one unresolved defect: every other arm captures nothing, with
the arm-time registers byte-identical between the run that works and the run
that does not. Arming twice in a row fails every time, so it is re-arm state
rather than anything the previous capture left on the bus. The measurements above
come from fresh arms and are consistent within a rate; the defect does not
affect the conclusion, and the conclusion is negative anyway.

### What would actually raise the ceiling

Not the capture path. The decode does ~66-132 CM7 cycles per sample, and at
792 MHz - the ARM PLL rate, which `SNIFF:STAT?` now reports as `core_hz` - that
is the whole budget at 6 MHz and beyond. Decoding in bulk rather than per sample
is the change with headroom behind it; every capture-side idea measured here is
already keeping up.

## Decode on one core, capture on the other? Measured: real, but small

`SNIFF:DECODE 0` turns the on-device decoder off and captures samples only. It
exists to answer exactly one question - what does decode cost, and therefore
what could splitting it off possibly buy - and the answer is now measured rather
than argued.

Identical controlled traffic (1 MHz I2C, a six-byte write, 108 edges), intake
forced to 5 MHz per tap, three runs each:

```
decode on    128-130 entries    1 overrun     every run
decode off   142    entries     0 overruns    every run
```

Decode is the cost, and it is not marginal: with it the capture loses about a
dozen transitions and reports an overrun; without it the same traffic is taken
cleanly. That confirms the premise - capture alone keeps up where capture plus
decode does not.

The payoff is smaller than the premise suggests. Pushed to 6 MHz per tap, both
configurations overrun:

```
6 MHz/tap, decode on    112-114 entries    1 overrun
6 MHz/tap, decode off   124-130 entries    1 overrun
```

So the ceiling moves from about 4 MHz per tap to about 5 - call it 8 M to 10 M
samples/s across two taps. Twenty-five percent, not a multiple.

That number explains every rate result in this port, which is the real reason to
trust it:

| capture | samples/s needed | observed |
|---|---|---|
| pin-clocked, 3 MHz bus | 6 M | clean |
| pin-clocked, 6 MHz bus | 12 M | overruns |
| pin-clocked, 12.5 MHz bus | 25 M | hopeless |

### Why the split should not go to the CM33

The obvious arrangement - CM7 captures, CM33 decodes - is backwards. The CM33
runs at 240 MHz against the CM7's 792, has no dual issue, and already owns USB
and every bus controller in the image. Moving decode there moves the bottleneck
onto the slower and busier of the two cores; the system gets worse, not better,
however much headroom the capture side gains.

The decoupling that does work is in time rather than across cores: capture raw
and decode afterwards, or ship raw to the host and decode there. `SNIFF:RAW?`
already carries the records for it, and a host has no real-time constraint at
all. What it costs is that the on-device decode is no longer live.

And it still does not reach full rate. Even perfect decoupling is the 25% above,
against the 3x needed for a 6 MHz bus and 6x for 12.5 MHz.

## The lever pulled: the capture core was never optimized

Every sampling limit in this project traces to one loop - the per-sample work in
`cap_consume()` - and that loop was compiled with **no optimization at all**.

`CMakeLists.txt` scoped `-Og` to the bridge profile, because the bridge has to
fit in 127 KB of ITCM and did not at `-O0`. The capture core was deliberately
left out, with the reasoning written down: it "has 255 KB of its own, so neither
needs this". That is a correct answer to a question about size and the wrong
question entirely. Nothing else in the build ever set an optimization level, so
the hot loop inherited the SDK's debug `-O0`.

The same note also argued that optimizing this core "would change the cost of
`cap_poll()` ... so it would invalidate the number rather than improve it". It
does invalidate the number. That is what a budget measured from this code is for.

`-O2` on the capture profile, nothing else changed:

| measurement | -O0 | -O2 |
|---|---|---|
| 4 MHz/tap, I2C 100 k / 400 k / 1 M | 6/6 exact | 6/6 exact |
| 5 MHz/tap | **0/6** - `DBE` decoded as `D8E` | **6/6 exact** |
| 6 MHz/tap | **0/6** - garbage framing | **6/6 exact** |
| 6 MHz I3C, pin-clocked | **8-9 overruns**, ~687 entries | **0 overruns**, 951 entries |

The intake budget goes from 8 M to **16 M samples/s**, held to the same standard
that set the old figure: the busiest case measured decodes exactly, repeatably.
Pin-clocked I3C is clean at 1, 2, 3 and 6 MHz where 6 MHz used to drop data.

Above 6 MHz per tap the sample rate stops rising with the budget - the divider is
no longer the binding constraint at these bus rates - so 16 M is where the
evidence runs out rather than where the core does. The real ceiling is higher and
untested.

### Why this was worth finding last

Three redesigns were measured and rejected before anyone looked at the compiler
flags:

- faster clocks - already at silicon maximum
- GPIO + eDMA instead of FlexIO - measured 6x too slow
- splitting capture and decode across the cores - measured 25%, and the wrong
  direction, since decode would land on the slower core

Each of those was a real measurement and each was worth having. But the thing
that actually moved the ceiling was a build flag, and it doubled what three
architectural changes could not. The lesson is cheap to state and was expensive
here: profile the build before redesigning around the profile.

I2C is unaffected by all of it - it was already exact at every speed it supports.
What changed is the headroom above it.
