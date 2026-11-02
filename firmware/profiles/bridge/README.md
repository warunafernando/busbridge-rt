# bridge profile

`_stm32_reference/` holds the STM32H5 implementations, kept verbatim as the
reference for the port. They are not compiled — nothing in this directory is in
the build until it has been rewritten against the NXP drivers.

Keeping them here rather than reading them out of `E:\STMicro` is deliberate:
the port is a translation, and a translation is much easier to check when the
original is beside it. They come out when the last of them is ported.

| STM32 file | ported to | phase |
|---|---|---|
| `bb_i2c.c` | LPI2C ×3 | 2 |
| `bb_i3c.c` (in `shared/`) | I3C ×2 | 2 |
| `bb_spi.c` | LPSPI4 | 2 |
| `bb_mdio.c` | bit-banged, unchanged in shape | 2 |
| `bb_uart.c` | LPUART4 | 2 |
| `bb_sniff.c` | **rewritten**, not ported — FlexIO + eDMA, not an edge interrupt | 3–4 |
| `bb_lcd*.c` | dropped — this board has no display | — |
