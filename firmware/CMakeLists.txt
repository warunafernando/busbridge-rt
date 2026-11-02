# BUSBRIDGE-RT firmware
#
# Built as a freestanding MCUXpresso SDK example: the sources live here, the
# SDK stays untouched at E:/RT1186/sdk/mcuxsdk. That is the same separation the
# STM32 tree kept between firmware/ and its lib/ clones of ST's HAL, and for the
# same reason - a vendor tree with local edits in it is a tree you cannot update.
#
#   west build -b frdmimxrt1186 E:/BusBridge-RT/firmware \
#       --toolchain armgcc --config debug -Dcore_id=cm33 \
#       -DBB_PROFILE=bridge -d E:/BusBridge-RT/build/bridge
#
# BB_PROFILE selects which image is built. The two share the board layer, the
# transports and the command parser; they differ in what they do with the pins.
# See firmware/inc/bb_profile.h.

cmake_minimum_required(VERSION 3.22.0)

include(${SdkRootDirPath}/cmake/extension/mcux.cmake)

set(BB_ROOT ${CMAKE_CURRENT_LIST_DIR})

if(NOT DEFINED BB_PROFILE)
    set(BB_PROFILE bridge)
endif()
# Three images, not two. `capture` is the CM7 half of the sniffer and shares the
# board layer and the capture format with the others but none of the command
# protocol - it answers the link, not the host.
if(NOT BB_PROFILE MATCHES "^(bridge|peripheral|capture)$")
    message(FATAL_ERROR "BB_PROFILE must be bridge, peripheral or capture, got '${BB_PROFILE}'")
endif()

# The board directory name is the -DBB_BOARD value, exactly as in the STM32
# tree: adding a board means adding a directory, not editing a list here.
if(NOT DEFINED BB_BOARD)
    set(BB_BOARD frdm_imxrt1186)
endif()
set(BB_BOARD_DIR ${BB_ROOT}/boards/${BB_BOARD})
if(NOT EXISTS ${BB_BOARD_DIR}/bb_board_def.h)
    message(FATAL_ERROR "no such board: ${BB_BOARD_DIR}/bb_board_def.h is missing")
endif()
include(${BB_BOARD_DIR}/board.cmake)

# The SDK appends PROJECT_BOARD_PORT_PATH to SdkRootDirPath, so it has to be
# relative to the SDK even though this tree lives outside it. Computed rather
# than written by hand, because a hand-written "../../.." silently stops being
# right the moment either tree moves.
if(NOT CMAKE_CURRENT_LIST_DIR MATCHES "^([A-Za-z]:)" OR NOT SdkRootDirPath MATCHES "^([A-Za-z]:)")
    set(BB_SAME_DRIVE TRUE)
else()
    string(REGEX MATCH "^([A-Za-z]:)" BB_DRIVE_APP ${CMAKE_CURRENT_LIST_DIR})
    string(REGEX MATCH "^([A-Za-z]:)" BB_DRIVE_SDK ${SdkRootDirPath})
    string(TOUPPER "${BB_DRIVE_APP}" BB_DRIVE_APP)
    string(TOUPPER "${BB_DRIVE_SDK}" BB_DRIVE_SDK)
    if(BB_DRIVE_APP STREQUAL BB_DRIVE_SDK)
        set(BB_SAME_DRIVE TRUE)
    else()
        set(BB_SAME_DRIVE FALSE)
    endif()
endif()
if(NOT BB_SAME_DRIVE)
    message(FATAL_ERROR
        "This tree (${CMAKE_CURRENT_LIST_DIR}) and the SDK (${SdkRootDirPath}) are on "
        "different drives. The SDK builds the board port path by appending to its own "
        "root, which cannot express that. Put both on one drive.")
endif()
file(RELATIVE_PATH BB_PORT_REL ${SdkRootDirPath} ${BB_BOARD_DIR}/port)

project(busbridge_rt LANGUAGES C CXX ASM
        PROJECT_BOARD_PORT_PATH ${BB_PORT_REL})

include(${SdkRootDirPath}/CMakeLists.txt)

# ---- sources -------------------------------------------------------------
if(BB_PROFILE STREQUAL "capture")
    # The capture core links none of the host transports or the command parser.
    # It talks to the controller core and to FlexIO, and that is all.
    mcux_add_source(
        BASE_PATH ${BB_ROOT}
        SOURCES
            cm7/capture_main.c
            cm7/dec_i3c.c       # the native I3C decoder, kept out of the I2C one
            src/bb_image.c      # for bb_image_crc32: every record carries one
    )
else()
    # Common to both host-facing images: the transports, the command pump, the
    # board layer.
    mcux_add_source(
        BASE_PATH ${BB_ROOT}
        SOURCES
            src/main.c
            src/bb_board.c
            src/bb_cmd.c
            src/bb_console.c
            src/bb_image.c
            src/bb_link.c
            src/bb_io.c
            src/bb_trace.c
            src/bb_usb.c
    )
endif()

# The profile's own sources. Listed per profile rather than globbed: a glob
# would silently pick up the _stm32_reference/ files that are deliberately not
# in the build yet.
if(BB_PROFILE STREQUAL "capture")
    # no profile sources: cm7/capture_main.c above is the whole image
elseif(BB_PROFILE STREQUAL "bridge")
    mcux_add_source(
        BASE_PATH ${BB_ROOT}
        SOURCES
            profiles/bridge/bridge.c
            profiles/bridge/bb_i2c.c
            profiles/bridge/bb_i3c.c
            profiles/bridge/bb_spi.c
            profiles/bridge/bb_mdio.c
            profiles/bridge/bb_uart.c
            profiles/bridge/bb_can.c
            profiles/bridge/bb_sniff.c
            profiles/bridge/bb_net.c
    )
else()
    mcux_add_source(
        BASE_PATH ${BB_ROOT}
        SOURCES
            profiles/peripheral/peripheral.c
            profiles/peripheral/bb_ee.c
            profiles/peripheral/bb_ee_cmd.c
            profiles/peripheral/bb_ee_i2c.c
            profiles/peripheral/bb_ee_spi.c
            profiles/peripheral/bb_ee_i3c.c
            profiles/peripheral/bb_ee_mdio.c
    )
endif()

mcux_add_include(
    BASE_PATH ${BB_ROOT}
    INCLUDES inc shared boards/${BB_BOARD}
)

# The RAM target is the only one with a memory budget, so it is the only one that
# is optimized.
#
# The SDK gives every *_debug configuration "-g -O0" (arch/arm/target/targets.cmake)
# and every *_release "-Os". That is right for the XIP configurations, which have
# 8 MB of flash and use 2% of it, and impossible for this one: `debug` links into
# 127 KB of ITCM, and at -O0 the bridge profile does not fit - it overflowed
# m_text by 496 bytes once the capture decoder and the event path landed.
#
# -Og rather than -Os on purpose. This is still the debug configuration and is
# what a debugger is pointed at, so the optimization that exists to stay
# debuggable is the one to reach for; -Os is the fallback if -Og stops fitting.
# Appended after the SDK's own flags, so gcc's last -O wins, and -g is untouched.
#
# Scoped deliberately, and the scope matters more than it looks:
#
#   - bridge only. The peripheral profile fits at -O0 (62%) and the capture core
#     has 255 KB of its own, so neither needs this.
#   - the RAM configuration only. The XIP pair is the primary build and the one
#     `reload.ps1` flashes, and its measurements - the 16 M samples/s intake
#     budget, the transport figures - were all taken at -O0. Optimizing the CM7
#     would change the cost of cap_poll() and the settle loops that pace the
#     generator, which is exactly what that budget measures, so it would
#     invalidate the number rather than improve it.
#
# The consequence to know about: the RAM and XIP images are now built at
# different optimization levels, so they are genuinely different binaries. That
# is what a 127 KB budget costs.
if(BB_PROFILE STREQUAL "bridge" AND CMAKE_BUILD_TYPE STREQUAL "debug")
    mcux_add_armgcc_configuration(CC "-Og")
endif()

# The capture core is built for speed, and until now it was not built for
# anything: it inherited the SDK's debug "-O0" because it has 255 KB to itself
# and nothing ever pushed on the size.
#
# Size was the wrong question. Every sampling limit in this project is the cost
# of the per-sample loop in cap_consume() - the intake budget, the pin-clocked
# overruns above 3 MHz, the whole ceiling - and that loop was compiled with no
# optimization at all. An earlier note here reasoned that optimizing this core
# "would change the cost of cap_poll() ... so it would invalidate the number
# rather than improve it". It does invalidate the number. That is the point: the
# budget is a measurement of this code, so making the code faster is supposed to
# move it, and the measurement is cheap to repeat.
#
# -O2 rather than -O3: the hot loop is bit tests and a comparison, not anything
# -O3's extra vectorization and unrolling would help, and -O2 keeps the image
# and the stack frames closer to what has been debugged.
#
# -flto because the hot loop crosses a translation unit.
#
# cap_consume() spends its per-bit budget calling dec_i3c_bit(), which lives in
# dec_i3c.c, so without link-time optimisation every bit pays a real call into
# another object file. Measured with the cycle counter: a pin-clocked word costs
# about 418 cycles for eight bits against a 507-cycle budget at 12.5 MHz, and
# the call is the part that cannot be optimised away from inside one file.
if(BB_PROFILE STREQUAL "capture")
    mcux_add_armgcc_configuration(CC "-O2 -flto")
    mcux_add_armgcc_configuration(LD "-flto -O2")
endif()

# The board header and its pin map are reached through these two defines rather
# than an #if chain, so nothing outside boards/<name>/ knows a board exists.
mcux_add_macro(
    CC "-DBB_BOARD_HEADER=\"${BB_BOARD_DIR}/board.h\" \
       -DBB_BOARD_PINMAP=\"${BB_BOARD_DIR}/pinmap.inc\" \
       -DBB_PROFILE_${BB_PROFILE}=1        -DUSB_DEVICE_CONFIG_LPM_L1=1        -DI2C_RETRY_TIMES=1000000        -DI3C_RETRY_TIMES=1000000        -DUART_RETRY_TIMES=1000000        -D__USE_SHMEM"
)

# UART_RETRY_TIMES does for fsl_lpuart.c what I2C_RETRY_TIMES does for LPI2C,
# and it is needed for a specific reason rather than symmetry.
#
# LPUART_WriteBlocking() waits for the transmitter to drain. With hardware flow
# control enabled and nothing wired to CTS, the pad pulls high - CTS_B is active
# low, so that reads as "peer not ready" - and the transmitter correctly refuses
# to send. The wait then never ends and the command loop is gone. That is not a
# flow-control bug: it is what flow control means. It is a bounded-wait bug.
#
# Found exactly that way: UART:CFG with rtscts succeeded and the *next* command
# timed out, because the reply to it could not be transmitted.
#
# I2C_RETRY_TIMES bounds every polled wait in fsl_lpi2c.c. Left undefined - the
# SDK default - those loops are bare `while (result == kStatus_Success)`, so any
# bus condition that stops SCL from ever completing a STOP or freeing the TX
# FIFO hangs the core with no fault and no timeout.
#
# I3C_RETRY_TIMES is the same define for fsl_i3c.c and was missed when the
# others went in, which cost real bench time: a private write that the target
# never acknowledged left I3C_MasterWaitForTxReady() spinning on
# `while (txCount < byteCounts)` with no exit, and the controller stopped
# answering USB entirely. It looked like a crash and was a wait. Recovery meant
# reflashing between attempts, so every measurement that needed a write cost a
# programming cycle.
#
# Both spellings matter: fsl_i3c.h only falls back to 0 when neither
# I3C_RETRY_TIMES nor CONFIG_I3C_RETRY_TIMES is defined, and 0 is what selects
# the unbounded variant of each loop.
#
# A bus tool is exactly the program that meets those conditions: its whole job
# is to talk to hardware that may be stuck, absent or mis-clocked. It must
# always come back with an answer, even an error one, so an unbounded wait
# anywhere below the command loop is a defect rather than a tuning choice.
#
# 1000000 iterations is a handful of milliseconds at 240 MHz - far longer than
# any legitimate wait at bus rates up to 1 MHz, including a target that clock
# stretches, and far shorter than a host timeout.

# USB_DEVICE_CONFIG_LPM_L1 is not a link-power-management feature request - it
# is the only guard the SDK puts around BOS descriptor support, and BOS is what
# carries the Microsoft OS 2.0 platform capability that makes Windows bind
# WinUSB with nothing installed.
#
# Without it, usb_device_ch9.c has no branch for USB_DESCRIPTOR_TYPE_BOS at all,
# so a host's GET_DESCRIPTOR(BOS) never reaches the device callback and is
# answered as an unsupported request. The device then enumerates and sits there
# with no driver bound and no error naming the cause. It took several rounds of
# descriptor bisection to find, because every descriptor involved was correct -
# the request simply never arrived.
#
# It is a compile definition rather than a Kconfig setting because the symbol is
# absent from the generated usb_device_config.h entirely (not defined to 0), so
# the SDK's `#if defined(...) && (... > 0)` guards pick this up cleanly.

# The capture core's image, carried by the controller.
#
# Generated by scripts/mkcm7image.py from the capture build's .bin, so the
# controller can copy it into the CM7's ITCM and release that core. Optional on
# purpose: without it the bridge still builds and SNIFF:CM7 reports that this
# build carries no capture image, which is a far better failure than a stale
# blob being copied into ITCM and run.
#
# 34 KB of .rodata, which is why the bridge is built for the flexspi_nor
# (XIP) target - it does not fit alongside the code in 127 KB of TCM.
#
# Gated, not automatic. In the RAM target the initialiser for .data is stored in
# m_text, so a 34 KB array costs 34 KB of the 127 KB of TCM whether it is const
# or not - and the controller already fills that to 96%. The RAM build simply
# cannot carry the image, so asking it to would turn a working development build
# into a link error.
#
# The flexspi_nor target has 8 MB and carries it comfortably. So the embed is an
# explicit option that scripts/build.ps1 passes only for that target, and the RAM
# build keeps working with SNIFF:CM7 reporting that this build has no image.
#
option(BB_EMBED_CM7 "Embed the capture core image (needs the flexspi_nor target)" OFF)

set(BB_CM7_IMAGE ${BB_ROOT}/generated/bb_cm7_image.c)
if(BB_EMBED_CM7 AND EXISTS ${BB_CM7_IMAGE})
    mcux_add_source(SOURCES generated/bb_cm7_image.c BASE_PATH ${BB_ROOT})
    mcux_add_macro(CC "-DBB_HAVE_CM7_IMAGE=1")
    message(STATUS "BUSBRIDGE: embedding capture image ${BB_CM7_IMAGE}")
else()
    mcux_add_macro(CC "-DBB_HAVE_CM7_IMAGE=0")
    message(STATUS "BUSBRIDGE: no embedded capture image "
                   "(BB_EMBED_CM7=${BB_EMBED_CM7}); SNIFF:CM7 will report so")
endif()

# __USE_SHMEM compiles in the MPU region that makes the CM33 <-> CM7 mailbox
# non-cacheable on both cores; the matching linker symbol __use_shmem__ is set in
# the board port's reconfig.cmake, and both halves are needed. See the note there.

include(${BB_BOARD_DIR}/port/reconfig.cmake OPTIONAL)

mcux_convert_binary(BINARY ${APPLICATION_BINARY_DIR}/${MCUX_SDK_PROJECT_NAME}.bin)
