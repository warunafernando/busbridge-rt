# Pin analysis — FRDM-IMXRT1186

Derived from the board schematic netlist (`SCH-95302 Rev A4`, Allegro `pstxnet.dat`),
UM12450 Rev 3.0 Tables 20–23, and the SDK IOMUXC tables. Every claim below is
traceable to one of those three.

**One caveat on the IOMUX data:** the MCUXpresso SDK ships `fsl_iomuxc.h` only for
the **MIMXRT1189**. The RT1186 is the same family in a smaller BGA196 package, and
every pad quoted here is confirmed bonded on our part from the schematic — but the
ALT numbers should be re-checked against `IMXRT1180RM` before they are committed to
`board.h`. This is exactly the "alternate function belongs to the pin, not to the
peripheral" trap that `PORTING.md` warns about.

---

## 1. The headline: only 22 Arduino signal pins, and J52 has none

| Header | Arduino | Pad |
|---|---|---|
| J1.2 | D0 | `GPIO_AON_09` |
| J1.4 | D1 | `GPIO_AON_08` |
| J1.6 | D2 | `GPIO_EMC_B1_12` |
| J1.8 | D3 | `GPIO_EMC_B1_13` |
| J1.10 | D4 | `GPIO_EMC_B1_14` |
| J1.12 | D5 | `GPIO_EMC_B1_15` |
| J1.14 | D6 | `GPIO_AD_12` |
| J1.16 | D7 | `GPIO_AD_13` |
| J51.2 | A0 | `GPIO_AD_15` |
| J51.4 | A1 | `GPIO_AD_16` |
| J51.6 | A2 | `GPIO_AD_17` |
| J51.8 | A3 | `GPIO_AD_14` |
| J51.10 | A4 | `GPIO_AD_19` |
| J51.12 | A5 | `GPIO_AD_18` |
| J2.2 | D8 | `GPIO_EMC_B1_17` |
| J2.4 | D9 | `GPIO_EMC_B1_16` |
| J2.6 | D10 | `GPIO_EMC_B1_25` |
| J2.8 | D11 | `GPIO_EMC_B1_24` |
| J2.10 | D12 | `GPIO_EMC_B1_23` |
| J2.12 | D13 | `GPIO_EMC_B1_22` |
| J2.18 | D14 | `GPIO_EMC_B2_19` |
| J2.20 | D15 | `GPIO_EMC_B2_20` |

**J52 is power only** — NC, IOREF, RESET_b, 3V3, 5V, GND, GND, VIN. No signals.

Additional connectors that carry MCU signals:

- **J3** — motor card, 2×11. Carries `GPIO_AON_15..19` and `GPIO_SD_B1_03`. Fitted.
- **J53** — 2×11, reaches the MCU only through further passives; not yet mapped.
- **J61** — SRAMC, 2×25, **DNP**.
- **J39** — FlexSPI follower, 2×7, **DNP**.
- **J22** — CAN H/L, 1×3, **DNP** (see §5).

---

## 2. The board is already a pin-sharing matrix

Most MCU pads do not reach the Arduino header directly. They pass through a
**3-pin selection jumper** (pin 2 = MCU pad, pin 1 = on-board function,
pin 3 = header), then a series resistor. This is the board doing exactly what you
suggested — one pad, two destinations — in hardware:

| Jumper | MCU pad | Position 1–2 (on-board) | Position 2–3 (header) |
|---|---|---|---|
| J41 | `GPIO_AON_09` | LPUART1_RXD (CM33 console) | Arduino **D0** |
| J43 | `GPIO_AON_08` | LPUART1_TXD (CM33 console) | Arduino **D1** |
| J44 | `GPIO_AON_06` | **CAN1_TX** | SAI1_TX_BCLK → J51.9 |
| J46 | `GPIO_AON_07` | **CAN1_RX** | SAI1_MCLK → J51.11 |
| J47 | `GPIO_AON_04` | ETH2_INT_B | SAI1_TX_D0 → J51.5 |
| J48 | `GPIO_AON_05` | USB_OTG1_PWR | CAN_STBY |
| J45 | `GPIO_AON_15` | ETH0_RST_B | header |
| J27 | `GPIO_AD_14` | LPUART3_RXD | Arduino **A3** |
| J33 | `GPIO_AD_13` | LPUART3_TXD | Arduino **D7** |
| J30 | `GPIO_AD_12` | GPIO_BUTTON (SW4) | Arduino **D6** |
| J28 | `GPIO_AD_16` | ECAT_LINK0 | Arduino **A1** |
| J32 | `GPIO_AD_17` | ECAT_LINK1 | Arduino **A2** |
| J31 | `GPIO_AD_18` | LPI2C3_ECAT_SCL | Arduino **A5** |
| J35 | `GPIO_AD_19` | LPI2C3_ECAT_SDA | Arduino **A4** |
| J29 | `GPIO_AD_29` | EWM_OUT_B | header |

**Consequence:** the CM33 debug console (LPUART1 on D0/D1) is *not* a hard conflict
the way PF0/PF1 was on the NUCLEO. Move J41/J43 and those pads become Arduino D0/D1.
The cost is the console, which is why the design keeps LPUART3 available as an
alternative console on D7/A3 (J33/J27).

---

## 3. Where each bus can actually go

Every pin pair for every I2C/I3C instance on this package, cross-referenced against
whether the pad reaches a connector:

| Instance | Pin pair (SCL / SDA) | Reaches |
|---|---|---|
| **LPI2C1** | `AON_09` / `AON_08` (alt 6) | **D0 / D1** ✅ |
| LPI2C1 | `AON_07` / `AON_06` (alt 3) | J51.11 / J51.9 — collides with CAN1 |
| LPI2C1 | `AON_21` / `AON_20` (alt 2) | ❌ no connector |
| LPI2C1 | `AON_25` / `AON_24` (alt 1) | ❌ no connector |
| **LPI2C2** | `AON_16` / `AON_15` (alt 4) | **J3.12 / J3.10** ✅ (motor connector) |
| LPI2C2 | `AON_18` / `AON_17` (alt 3) | J3.16 / J3.14 ✅ |
| LPI2C2 | `AON_23` / `AON_22` (alt 1) | ❌ no connector |
| **LPI2C3** | `EMC_B2_19` / `EMC_B2_20` (alt 8) | **D14 / D15** ✅ — but see §4 |
| LPI2C3 | `AD_18` / `AD_19` (alt 9) | **A5 / A4** ✅ |
| LPI2C3 | `EMC_B2_00` / `EMC_B2_01` (alt 9) | ❌ no connector |
| **LPI2C6** | `B2_08` / `B2_09` (alt 6) | ❌ **no connector — unusable** |
| **I3C1** | `AON_16` / `AON_15` (alt 9) | **J3.12 / J3.10** ✅ |
| I3C1 | `AON_21` / `AON_20` (alt 3) | ❌ no connector |
| **I3C2** | `AD_18` / `AD_19` (alt 6) | **A5 / A4** ✅ — only option |

### What this means

**Four independent I2C masters does not port.** The STM32 build's headline feature
was four I2C buses on headers. Here:

- LPI2C6 cannot reach any connector at all.
- LPI2C2 is only reachable on the **motor connector J3**, not the Arduino headers.
- So the maximum on connectors is **three** — and only two on Arduino headers.

Two pairs are genuinely shared, which is where your "same pins for different ports"
comes in — both are the same situation as `BB_I3C_P1_SHARES_I2C` on the H533:

- `GPIO_AON_15/16` (J3.10/J3.12) → **LPI2C2 or I3C1**, never both.
- `GPIO_AD_18/19` (A5/A4) → **LPI2C3 or I3C2**, never both.

Because LPI2C3 has a second home at D14/D15, the conflict is avoidable:

> Put **LPI2C3 on D14/D15**, leaving **A4/A5 free for I3C2**. Then only the
> J3 pair has to be arbitrated (LPI2C2 vs I3C1), and the firmware does that at
> runtime exactly as `bb_i3c.c` already does.

### Other buses

| Bus | Pins | Note |
|---|---|---|
| **LPSPI4** | D10 PCS0, D11 SDO, D12 SDI, D13 SCK (alt 3) | clean, standard Arduino SPI position |
| LPSPI3 | A0 PCS1, A1 SCK, A2 PCS0, A4 SDI, A5 SDO (alt 7) | second SPI, but collides with I3C2 on A4/A5 |
| LPSPI2 | J3.12/14/16/18 (`AON_16..19`) | collides with LPI2C2 / I3C1 on J3.12 |
| **LPUART4** | D2 TX, D3 RX (alt 2) | first choice for the target UART |
| LPUART5 | D4 TX, D5 RX (alt 2) | |
| LPUART9 | D8 TX(D9), RX(D8) (alt 2) | |
| LPUART3 | D7 TX, A3 RX (alt 6) | alternative console if D0/D1 are taken |
| LPUART10 | A0 TX, A1 RX (alt 1) | |
| **CAN1** | `AON_06` TX, `AON_07` RX via J44/J46 | on-board transceiver U13 |
| **FlexIO1** | D2–D5 (FLEXIO12–15), D8–D13 (FLEXIO16,17,22–25) at **alt 8** | capture engine |
| **FlexIO2** | D6, D7, A0–A5 (FLEXIO12–19) at **alt 8** | capture engine |

**MDIO has no route to the headers.** `EMDIO` appears on none of the 22 Arduino
pads — the ENETC hardware MDIO is wired to the on-board PHYs. External MDIO must
therefore be FlexIO or GPIO bit-bang, as on the STM32. Hardware MDIO is available
only for talking to the board's own PHYs.

---

## 4. Two things that will cost an afternoon if not checked first

**(a) D14/D15 look swapped.** UM12450 Table 22 labels J2.18 as `D14/I2C_SDA` and
J2.20 as `D15/I2C_SCL`. But the IOMUX says:

```
GPIO_EMC_B2_19 (D14, silkscreen SDA)  alt8 = LPI2C3_SCL
GPIO_EMC_B2_20 (D15, silkscreen SCL)  alt8 = LPI2C3_SDA
```

Those are crossed. Either the UM table is wrong, the ALT numbers differ on the
RT1186 package, or LPI2C3 was never intended to be used there. **Verify against
IMXRT1180RM and on the bench before wiring anything to D14/D15.** This is the same
class of error as the CN5/CN9 jumper that cost an afternoon on the NUCLEO.

**(b) J22 is DNP.** The CAN transceiver U13 is fitted and CAN1 reaches it through
J44/J46, but the 3-pin CAN H/L connector **J22 is not populated**. CAN bring-up
needs a header soldered to J22, or flying leads to `CAN1_H` / `CAN1_L`. Worth
knowing before Phase 5 rather than during it.

---

## 5. Self-sniffing — what the pin budget actually allows

FlexIO sits at **alt 8** on nearly every Arduino pad. That is the good news. The
constraint is absolute: **a pad has one MUX_MODE.** A pad muxed to LPI2C3 is not
also muxed to FlexIO. So tapping our own bus needs one of:

**(a) Jumper wire to a second pad — works today, costs pins.**
Two wires per bus. Tapping the LPI2C3 bus on D14/D15 means wiring them across to,
say, D2/D3 (`FLEXIO1_FLEXIO12/13`). Budget: 2 header pins per 2-wire bus, 4 per SPI.

**(b) IOMUXC `SION` + `*_SELECT_INPUT` — unproven, would cost nothing.**
If the pad's input path can be forced on while it is muxed to LPI2C, and a FlexIO
input's `SELECT_INPUT` register can point at that same pad, we get a zero-wire
internal tap. **This is the Phase 1 experiment.** It is the difference between
"self-sniffing is free" and "self-sniffing costs half the header".

**(c) Controller-side tracing — free, but tagged.**
Always available, never presented as a wire measurement.

### Proposed default allocation (assuming (a); revise if (b) works)

| Function | Pins | Notes |
|---|---|---|
| LPI2C1 | D0 / D1 | move J41+J43; console goes to LPUART3 |
| LPI2C3 | D14 / D15 | Arduino I2C position — **check §4(a) first** |
| I3C2 | A4 / A5 | sole I3C2 option |
| LPI2C2 **or** I3C1 | J3.10 / J3.12 | runtime-arbitrated, shared pair |
| LPSPI4 | D10–D13 | standard SPI |
| LPUART (target) | D2 / D3 | LPUART4 |
| Console | D7 / A3 | LPUART3, via J33 + J27 |
| CAN1 | on-board transceiver | J44+J46 to position 1–2; fit J22 |
| **FlexIO taps** | D4, D5, D8, D9, A0, A1, A2 | 7 pads with alt-8 FlexIO left over |

Seven free FlexIO-capable pads is enough to tap one 2-wire bus **and** one 4-wire
SPI simultaneously, or three 2-wire buses. It is not enough to tap everything at
once — which is why `SNIFF:CH` in the spec binds a capture channel to a protocol
and a pin set at runtime rather than fixing it at build time.

---

## 6. Open item

`J53` (2×11, fitted) has not been mapped — its pins reach the MCU through more than
three hops of passives. If the allocation above proves too tight, J53 is the next
place to look for pins.
