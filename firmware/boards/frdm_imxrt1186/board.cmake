# FRDM-IMXRT1186 — the toolchain facts that follow from the board.
#
# Everything else is derived: the -D define from the directory name, the SDK
# board name from BB_SDK_BOARD, the link target from BB_MEMORY. Unlike the STM32
# tree there is no linker script or startup file to name - the MCUXpresso SDK
# supplies both for the chosen device and memory target.

set(BB_SDK_BOARD    frdmimxrt1186)      # examples/_boards/<this>
set(BB_DEVICE       MIMXRT1186)
set(BB_XTAL_HZ      24000000UL)         # Y1, UM12450 section 2.2

# Which core each image runs on. The controller takes the CM33 because it is
# the boot core and owns the ELE; capture takes the CM7 for its clock.
set(BB_CORE_BRIDGE      cm33)
set(BB_CORE_PERIPHERAL  cm33)
set(BB_CORE_CAPTURE     cm7)

# Default link target. `debug` puts the image in CM33 code TCM at 0x0FFE0000,
# which is what you want while developing - nothing is written to flash and the
# boot ROM stays resident. `flexspi_nor_debug` produces a bootable XIP image at
# 0x04000000 with an FCB and boot container.
set(BB_MEMORY_DEFAULT   debug)
set(BB_MEMORY_XIP       flexspi_nor_debug)

# LinkServer, not pyOCD. pyOCD cannot drive this part: the RT1180 is an ADIv6 /
# CoreSight SoC-600 design whose APs are addressed by base address, and pyOCD
# enumerates ADIv5 indices, so it finds no cores. See E:/RT1186/README.md.
set(BB_LINKSERVER_DEVICE  "MIMXRT1186:FRDM-IMXRT1186")

# Debugging the CM7 needs a device config with cm7 as the primary core and the
# ITCM variant of the connect script, because the M7 AP is invisible until the
# core is woken into a landing zone at its own VTOR address.
set(BB_LINKSERVER_CM7_CFG "E:/RT1186/linkserver/FRDM-IMXRT1186-CM7-primary-ITCM.json")
