# BUSBRIDGE-RT — specification

**Target:** FRDM-IMXRT1186 (MIMXRT1186CVJ8C, Cortex-M33 300 MHz + Cortex-M7 800 MHz)
**Source projects:** `E:\STMicro\busbridge` (11.4k lines firmware, STM32H5) and `E:\STMicro\sniffer` (19.3k lines Rust)
**Status:** draft for approval — nothing implemented yet

---

## 1. What this is

Two applications, as on the STM32:

1. **Bus controller** — a scriptable USB-attached multi-protocol bus master.
2. **Bus sniffer** — passive capture and decode of bus traffic.

With one capability the STM32 version does not have: **the sniffer can watch the controller's own ports while the controller is driving them.**

### 1.1 Decisions already taken

| Question | Answer |
|---|---|
| Meaning of "sniff ports while in use" | **Both** — self-monitoring of our own buses *and* faster non-intrusive capture of external buses, from one engine |
| Stack scope | **Firmware + keep the wire protocol.** The existing Python library, C DLL, Rust GUI and Rust sniffer keep working, with a board entry added |
| Buses | Parity set (I2C, I3C, SPI, MDIO, UART) **+ CAN** **+ Ethernet/TSN/EtherCAT** |

Keeping the ASCII one-line-per-command protocol is the single highest-leverage decision here: roughly 60k lines of working, tested host code (Python, C DLL, egui GUI, Rust sniffer with six decoders and a golden-capture corpus) continues to work against new silicon. New capability arrives as *additional* commands, never as changes to existing ones.

---

## 2. Why the RT1186 changes the design

The STM32 sniffer's three documented limits are all consequences of one choice — decoding in a GPIO edge interrupt:

| STM32H5 limit | Cause | RT1186 answer |
|---|---|---|
| ~450 ns per bus clock, ~1 MHz ceiling | interrupt entry + GPIO read + EXTI write per edge | FlexIO latches in hardware; no per-edge interrupt |
| Native I3C reads every bit one position late | SCL high phase is tens of ns; the handler reads the port after SCL has already fallen | FlexIO timer clocked *from SCL* latches SDA at the edge itself |
| Capture steals the core from everything else | one core | capture and decode live on the CM7; the controller keeps the CM33 |

Relevant silicon, confirmed from the SDK headers on this machine:

- **LPI2C ×4** (1, 2, 3, 6) — same four independent I2C buses as the STM32 build
- **I3C ×2** (I3C1, I3C2) — the STM32 could only ever drive one
- **LPSPI ×6**, **LPUART ×8**
- **FlexCAN ×2** (CAN1, CAN3); **CAN1 is wired to on-board transceiver U13 → J22**
- **FlexIO ×2** — the capture engine
- **eDMA, 64 channels**
- **ENETC EMDIO** — hardware MDIO, replacing the STM32's bit-banged clause 22/45
- **USB HS + USBPHY1** — 480 Mbit/s against the STM32's 12 Mbit/s full speed
- 2× gigabit TSN ports, 2× EtherCAT ports

---

## 3. Architecture

### 3.1 Core split

```
  CM33 @ 300 MHz  (boot core, owns ELE)     CM7 @ 800 MHz
  ------------------------------------      -----------------------------
  USB HS device + WinUSB                    FlexIO capture engine
  ASCII command parser and dispatch         eDMA ring management
  I2C / I3C / SPI / MDIO / UART / CAN       protocol decode
  masters                                   timestamping
  pin arbitration, pin map                  capture record framing
              |                                        |
              +------------ RPMsg-Lite / MCMGR --------+
```

The controller keeps a whole core. Capture cannot starve it, and decode cannot be starved by it — which is precisely what "sniff while in use" requires.

MCMGR and RPMsg-Lite are already in the SDK workspace, and the multicore boot path is verified working on this board (CM33 releases CM7; both cores debug).

### 3.2 Capture engine — per-protocol, hardware-clocked

Not one GPIO edge decoder. Each protocol gets a FlexIO configuration whose timer is clocked by the bus's own clock line, so data is latched in hardware at the correct instant:

| Protocol | FlexIO timer clock | Shifter inputs | Notes |
|---|---|---|---|
| I2C / I3C | SCL pin | SDA | fixes the I3C off-by-one-bit defect outright |
| SPI | SCK pin | MOSI, MISO | CS gates the timer, giving CS-bounded transfers free |
| MDIO | MDC pin | MDIO | turnaround visible as a real bit |
| UART | internal, 16x baud | RX (and TX) | oversampled, so baud can still be measured |
| CAN | **none — FlexCAN listen-only mode** | — | a real CAN controller receiving, not a bit-banger |

Two capture modes, as on the STM32, and the same distinction:

- **Decoded events** — transactions, unbounded length.
- **Raw edges/samples** — timestamped, DMA'd to a RAM ring, traceable back to samples. This is what the Rust `sniffer` project consumes.

The raw record format stays **compatible with the existing capture container**: picosecond `u64` timestamps, length-prefixed CRC'd records, first record is initial state not a transition, gaps recorded explicitly when a ring fills. Those three contracts are load-bearing in the Rust code and are not being renegotiated.

### 3.3 Sniffing our own ports — three mechanisms, in confidence order

**(a) External loopback taps — day one, guaranteed.**
Short jumpers from a controller pin to a FlexIO tap pin on the headers. Exactly how you would tap anyone else's bus; no silicon uncertainty. Costs header pins and a wire per signal.

**(b) Internal tap via IOMUXC — RESOLVED: not possible on this silicon.**
The idea was that IOMUXC's per-pad `SION` bit plus the *separate* `*_SELECT_INPUT` daisy-chain registers might let a FlexIO input select a pad whose `MUX_MODE` is already LPI2C, giving a zero-wire internal tap of a live controller pin.

**It cannot be done, and no bench test was needed to settle it.** The daisy-chain registers exist — 186 `SELECT_INPUT` groups in `PERI_IOMUXC.h` — but FlexIO is not among them:

```
CAN1-3  ECAT  FLEXPWM1-3  FLEXSPI1-2  I3C2  KPP  LPI2C3-6  LPSPI3-6
LPUART3-11  MIC  NETC  QTIMER1-8  SAI4  SINC1-2  SPDIF  USB  USBPHY1-2
USDHC1-2  XBAR1  XSPI
```

A FlexIO pin's input comes from its own fixed pad, decided by that pad's `MUX_MODE`, and there is no register to point it elsewhere. A pad is muxed to LPI2C **or** to FlexIO, never both. So **(a) is not a fallback, it is the answer**: wire-level self-sniffing costs two jumpers per bus, permanently.

Worth noting what made this look plausible when the spec was written: the daisy registers are real and several bus peripherals do have them. `LPI2C3-6` can choose their input pad, and `I3C2` has `I3C2_PIN_SCL_IN`/`I3C2_PIN_SDA_IN` while `I3C1` does not. That asymmetry suggests an untested possibility for I3C specifically — I3C1 mastering with I3C2 pointed at its pads as a monitor, which would be bit-exact at full push-pull rate rather than oversampled. Unproven, and recorded as an idea rather than a plan.

**(c) Controller-side transaction tracing — always available, clearly labelled.**
The bridge already knows what it transmitted and what came back. That can be emitted into the same capture stream at near-zero cost. It is **not** a wire measurement and will never be presented as one: such records carry a distinct source tag (`observed=controller` vs `observed=wire`) so no tool can silently confuse what was inferred with what was sampled. The STM32 project makes this same point; it is worth preserving.

### 3.4 Transport

USB high speed, WinUSB, Microsoft OS 2.0 descriptors — same approach as the STM32 build so Windows binds a signed in-box driver on first plug with nothing to install. The STM32 measured 5409 round trips/s and 110 kB/s on full speed; high speed should move payload substantially, which matters because raw capture readout is bandwidth-bound. **Targets will be set by measurement, not predicted here.**

The MCU-Link VCOM (COM19) stays as a console for bring-up, as on the STM32.

---

## 4. Protocol surface

Existing commands keep their exact syntax and semantics: `SYS:*`, `I2C:*`, `I3C:*`, `SPI:*`, `MDIO:*`, `UART:*`, `SNIFF:*`, `EE:*`.

Additions:

| Command | Purpose |
|---|---|
| `CAN:CFG <bitrate> [data-bitrate]` | FlexCAN1 on the on-board transceiver |
| `CAN:TX <id> <hex...>` / `CAN:RX?` | frames |
| `CAN:LISTEN <0\|1>` | listen-only sniffing |
| `SNIFF:SRC <wire\|self\|both>` | choose tap source — the new capability |
| `SNIFF:CH <proto> <pins...>` | bind a capture channel to a protocol and pins |
| `SNIFF:MODE <oversample\|pinclock>` | free-running sample clock, or shift on the bus clock |
| `SNIFF:RATE?` | measured achievable rate for the current configuration |
| `SNIFF:RAW?` | the capture's own record bytes, undecoded |
| `SYS:GPIO <signal> <0\|1>` | hold one named pin at a level, peripheral bypassed |
| `SYS:PULSE <signal> [count]` | drive n low/high pairs on one named pin |
| `NET:*` | Ethernet / TSN / EtherCAT (Phase 6, surface TBD) |

`SYS:GPIO` and `SYS:PULSE` exist because diagnosing wiring from transfer behaviour is ambiguous: a transfer moves both lines and cannot say which pin is which. One named pin, one peripheral-free toggle, a known edge count, watched on one tap, settles it in a round. They drive push-pull, so they must not be pointed at a pin something else is actively driving. Every peripheral re-claims its pads per operation, so a check leaves nothing behind.

Changed replies:

| Command | Change |
|---|---|
| `I3C:FREQ?` | adds `act_pp`, `act_od` and `fclk` — what the dividers actually produce, which is rarely what was asked for |
| `SYS:PINS?` | adds `/cfg=0xNN`, the live `SW_PAD_CTL` value for that pin |
| `I3C` NAK errors | name the address actually sent (`no-ack-for-0x09`) rather than always `7E` |
| `SNIFF:READ?` | carries `framing=i3c-t-bit-not-ack` on an I3C capture, and `note=bits-exact-tail-truncates-under-8-bits` in pinclock mode — framing is hardware-qualified by SCL, so only the sub-byte tail of a transfer is lost |

`SYS:PINS?` remains the single source of truth for the pin map, read by the GUI and both host libraries. `SYS:ID?` will report the RT1186 board and image.

---

## 5. Pin budget — the real constraint

FRDM-IMXRT1186 Arduino headers, from the schematic netlist (`docs/PINMAP-U19-MIMXRT1186.md`):

| Header | Signals |
|---|---|
| J1 (D0-D7) | `GPIO_AON_09`, `GPIO_AON_08`, `GPIO_EMC_B1_12..15`, `GPIO_AD_12`, `GPIO_AD_13` |
| J51 (A0-A5) | `GPIO_AD_15, 16, 17, 14, 19, 18` |
| J2 (D8-D15) | `GPIO_EMC_B1_17, 16, 25, 24, 23, 22`, `GPIO_EMC_B2_19` (SDA), `GPIO_EMC_B2_20` (SCL) |
| J52 | second 2x8 |

**Known conflicts to resolve during Phase 2:**

- `GPIO_AON_08/09` are D1/D0 *and* LPUART1 — the CM33 debug console. Using them as Arduino D0/D1 costs the console, exactly as PF0/PF1 cost I2C2 on the STM32.
- Every self-tap under mechanism (a) consumes a second pin per signal. Four I2C buses fully tapped is 8 pins on top of the 8 they already use.
- J60/SW5 boot mode, J40, J58, J59, J36 are board control, not free.

The STM32 project's rule stands: the pin map is published by the firmware and checked by a test that reads the source (`test_boards.py`), because a board once silently inherited another board's map. That test gets an RT1186 case.

---

## 6. Repository layout

New project root: **`E:\RT1186\busbridge-rt`** (decision point — see section 8).

```
BusBridge-RT/
  firmware/
    boards/frdm_imxrt1186/   board.cmake, board.h, pinmap.inc
    cm33/                    controller: USB, parser, bus masters
    cm7/                     capture: FlexIO, eDMA, decode
    shared/                  RPMsg protocol, capture record format
    src/                     board layer, console, command dispatch
  host/                      board entry only; library stays in E:\STMicro\busbridge\host
  docs/
  scripts/                   build.ps1, flash.ps1
```

Built against the MCUXpresso SDK west workspace already at `E:\RT1186\sdk`, using the verified toolchain (GCC 14.3, ninja, CMake) and LinkServer for flash/debug. The STM32 tree's `lib/` (ST HAL clone) has no counterpart — the MCUXpresso SDK plays that role and is already on disk.

The board-porting discipline from `docs/PORTING.md` carries over verbatim: three files per board, nothing outside the board directory edited.

---

## 7. Phases

| Phase | Deliverable | Proves |
|---|---|---|
| **0. Skeleton** | CM33 image, USB HS + WinUSB enumerating, `SYS:*` answering, console on COM19 | transport and command loop on new silicon |
| **1. Tap experiment** | Bench test of IOMUXC `SION` + `*_SELECT_INPUT` internal tap | decides 3.3(b) vs 3.3(a); scheduled early because it shapes the pin budget |
| **2. Bus masters** | I2C x4, I3C x2, SPI, hardware MDIO, UART; pin map published; conflicts resolved | parity with the STM32 controller |
| **3. Capture engine** | CM7 image, FlexIO + eDMA raw capture, RPMsg readout, `SNIFF:RAW:*` | the core new work |
| **4. Sniff-while-driving** | `SNIFF:SRC`, self-tap, decoded events, capture files the Rust decoders read unchanged | **the headline requirement** |
| **5. CAN** | FlexCAN1 master + listen-only sniff on the on-board transceiver | first real traffic for the existing CAN decoder |
| **6. Ethernet / TSN / EtherCAT** | `NET:*` | separate sub-project; scoped after Phase 5 |

Phases 0-2 are a port. Phases 3-4 are the new engineering. Phase 6 is large enough that it deserves its own spec once 0-5 are real.

---

## 8. Decisions (resolved)

| Question | Answer |
|---|---|
| Project root | **`E:\BusBridge-RT`** |
| Ethernet / TSN / EtherCAT | **Later phase** - stays Phase 6, scoped after Phase 5, does not shape earlier phases |
| Peripheral profile (second image) | **Yes, both images.** Port the `peripheral` profile too, so one RT1186 board can be the device under test for another - same two-image structure as the STM32 tree, selected by build profile |
| STM32 tree | **Untouched.** RT1186 is a separate project; no convergence |
| Pin sharing | **Permitted and expected.** The board is already built this way - see `PINS.md` section 2 |

### 8.1 Consequences of the two-image decision

The STM32 tree builds `bridge` and `peripheral` from one board layer, one USB stack
and one command parser, switched by `-DBB_PROFILE`. That structure ports directly.
Two RT1186-specific notes:

- `bb_ee.c` and its four port back-ends (`bb_ee_i2c/i3c/spi/mdio.c`, ~1.4k lines)
  become LPI2C/I3C/LPSPI target-mode drivers. The MDIO target has no hardware
  counterpart here (see `PINS.md` section 3) and stays bit-banged.
- The STM32 used dual-bank flash and `SYS:BANK:SWAP` to hold both images at once.
  The RT1186 boots from 16 MB of external QSPI NOR, so the equivalent is two images
  at different offsets selected by the boot container - a different mechanism for
  the same behaviour. `docs/BANKS.md` does not port; the command might.

---

## 9. What I am not promising

- No performance number is claimed until measured on this board. The STM32 project's habit of publishing measured figures with the method that produced them — and of recording two measurements that turned out to be wrong — is the standard to hold here.
- The internal-tap mechanism (3.3 b) is **resolved as impossible** — FlexIO has no input pad selection on this part. The spec never depended on it; mechanism (a) carries the feature.
- Phase 6 is not costed.
