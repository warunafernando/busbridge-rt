# Host transport for a high-speed device

`bb_winusb_hs.c` is a drop-in replacement for the STM32 tree's
`windows/src/bb_winusb.c`. Same entry points, same handle fields, so
`busbridge.c` and the public API are unchanged. **The STM32 tree is not
modified** — drop this file in beside the original and point the build at it,
or copy it over once it has been measured.

## The INF is not the lever

`driver/busbridge.inf` sets `PortSubClass` and a `serenum` upper filter. That is
the **CDC serial** device — the ST-LINK VCP path retired at 692 round trips a
second. WinUSB needs no INF at all; the device's Microsoft OS 2.0 descriptors
bind it. There is no registry policy in that package that raises any ceiling.

## What did limit throughput

In order of size:

| # | Limit | Fix |
|---|---|---|
| 1 | **One transfer in flight.** A 512-byte overlapped read, waited on, then the next. USB is scheduled per frame, so one outstanding transfer means at most one packet served per frame — the link rate is irrelevant. | `HS_IN_FLIGHT = 4` |
| 2 | **512-byte buffer.** On full speed that batched 8 packets of 64. On high speed it is *one* packet of 512, so the same buffer batches an eighth as much work. | 64 KB reads, clamped to `MAXIMUM_TRANSFER_SIZE` |
| 3 | **No `RAW_IO`.** WinUSB buffers and re-splits transfers in the kernel. | `RAW_IO` on the IN pipe |

(1) is the big one, and it is a host change, not a firmware one. On full speed it
is the whole story behind 110 kB/s; on high speed the same code would reach
roughly 8× that and stop, because the bottleneck moves without going away.

## What will not improve 40×

High speed is a 40× link — 480 Mbit/s against 12. **Round trips will not improve
40×.** A round trip is one OUT transfer, the firmware's response, and one IN
transfer, and its floor is frame timing: 1 ms frames become 125 µs microframes,
which is **8×**. Expect round trips in that order and bulk payload much more.

Worth stating because the natural benchmark — commands per second — measures the
thing that improves least. Reporting it as though it measured the link is how a
successful change gets recorded as a failure.

Numbers to beat, measured on the STM32 at full speed:

```
USB round trips   5864 /s        USB payload   113.1 kB/s
```

No prediction is offered for what this reaches. Measure it, and report the
method with it — `bb_winusb_transport_info_hs()` exists so a figure can be
attributed to a configuration rather than floating free.

## Two things the queue makes newly possible to get wrong

Both are handled, and both would only show up under load:

- **Completions are reaped in submission order**, not by whichever event fires
  first. Bulk transfers on one pipe complete in order; taking them out of order
  reorders the byte stream, which for a line-oriented protocol means garbled
  replies under load only.
- **`flush` aborts and drains the queue**, not just the reassembly buffer.
  Queued transfers otherwise hold stale bytes that surface as the first line of
  the *next* command's reply — a one-command-behind bug that queueing introduces
  if flush is not written for it.

## Firmware side

`E:\BusBridge-RT\firmware\src\bb_usb.c` is the matching device. The parts that
decide the speed:

- `kUSB_ControllerEhci0`, not KHCI. KHCI is the full-speed controller; choosing
  it caps the link at 12 Mbit/s no matter what the descriptors say.
- `bcdUSB 0x0201`. Windows only requests a BOS descriptor — how MS OS 2.0
  descriptors are advertised — from a device claiming 2.01 or later. Declaring
  2.00 makes the whole WinUSB binding path silently not happen.
- A **device qualifier** descriptor. Omitting it is legal only for a
  full-speed-only device, and Windows reads its absence as "cannot do high
  speed" — so a capable device enumerates at full speed and every measurement
  afterwards is of the wrong thing.
- Bulk `wMaxPacketSize` patched to 512 or 64 to match the speed the host
  actually enumerated at, and the endpoint opened with the same figure.

`SYS:USB?` reports the enumerated speed and the packet size in force. Check it
before trusting a throughput number: **a device that quietly came up at full
speed is the most likely reason a high-speed figure comes out low.**

## The connector

USB **OTG1 on J63**, not the MCU-Link debug USB on J23. Different connectors,
different silicon — J23 goes to the LPC55S69 that provides the VCOM console.
A second cable is needed.

---

## Connecting the existing GUI

**Not yet — nothing has been flashed.** But the path is short, and three
identifier mismatches that would each have broken it silently are now fixed.

### Three mismatches found and corrected

The host library enumerates with `SetupDiGetClassDevs` on **one interface GUID
and nothing else**. My first version of `bb_usb.c` invented its own.

| | was | now |
|---|---|---|
| Interface GUID | `{CBA28F6C-…}` invented | `{1D4B2E5F-9C3A-4F8B-B0E7-2A6C8D3F5A91}` — matches `BB_WINUSB_GUID` |
| idVendor | `0x1FC9` (NXP's — not ours to hand out PIDs from) | `0x1209` (pid.codes, as the STM32 build correctly used) |
| idProduct | `0x008A` | `0x0003`, deliberately ≠ the STM32 board's `0x0001` |

The GUID one is the important one: the device would have bound WinUSB perfectly
and the host would then have found **zero devices**, with no error message
anywhere to explain it.

The PID one is subtler. Windows caches a device's MS OS 2.0 descriptor set
against VID/PID/bcdDevice. Reusing `0x0001` with a different descriptor set gets
the *cached STM32 set* served instead — a device that binds WinUSB with the wrong
properties, explicable by reading neither firmware. The STM32 build hit the same
trap from the other side and bumped `bcdDevice` to 1.01 to escape it.

`0x0002` became `0x0003` during USB bring-up. Windows keys a device node on
VID + PID + serial number, and this firmware advertises no serial (the OCOTP UID
read is not implemented, and a serial that is identical on every unit is worse
than none — two boards would collapse into one node). Without admin rights the
stale nodes from the failing attempts could not be removed with `pnputil`, so a
fresh PID was the only way to get an uncached device node to test against.

The host library does not filter on VID or PID — it searches on the interface
GUID alone — so this change is invisible to it. That is worth knowing before
changing the PID again: the identity that must not drift is the GUID.

Verified in the linked image: GUID matches, VID/PID correct, `WINUSB`
compatible-ID present.

### The DLL does not need rebuilding

The existing `windows/build/x64/busbridge.dll` is protocol-compatible — it will
connect as-is, just with the old single-transfer transport (slower, but working).
`host/windows/bb_winusb_hs.c` is a speed change, not a compatibility one, so it
can wait until after the link is proven.

### Order of operations

1. `scripts\flash.ps1 -Profile bridge`, then `SYS:ID?` on **COM19**. This proves
   the clock tree, console and command loop with USB out of the picture.
2. Plug a second cable into **J63** (not J23). Windows should bind WinUSB with
   nothing installed. `windows\build\x64\enumerate.exe` should list the board.
3. Only then open the GUI.

Step 1 first, deliberately: if USB does not enumerate, having already proven the
command loop over the console tells you the problem is in the USB stack and not
somewhere in the other 9,000 lines.

### What will and will not work in the GUI

Every command the GUI issues on connect exists in the image: `SYS:ID?`,
`SYS:STAT?`, `SYS:CLK?`, `SYS:USB?`, `SYS:BTN?`, `SYS:BANK?`, `I2C:FREQ?`,
`I2C:SCAN`, `I3C:FREQ?`, `I3C:TARGETS?`, `SPI:CFG?`, `MDIO:CLK?`, `UART:CFG?`.

Two panels will misbehave, both expected:

- **The I3C target panel** (`I3C:TGT?`, `TGT:RX?`, `TGT:IBI`, `TGT:HJ`) answers
  `unsupported` — those belong to the peripheral image now, not the bridge. The
  commands are present so the GUI gets a definite "wrong image" rather than
  "unknown command", which would be indistinguishable from old firmware.
- **The Firmware tab will not work at all.** It drives pyOCD, and pyOCD cannot
  attach to this part — the RT1180 is ADIv6 and pyOCD enumerates ADIv5 access
  port indices. Use `scripts\flash.ps1`, which drives LinkServer.
