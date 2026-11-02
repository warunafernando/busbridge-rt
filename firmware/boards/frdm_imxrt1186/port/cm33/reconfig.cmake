# Stack: 8 KB, not the device linker script's default 1 KB.
#
# This is the same deliberate edit the STM32 tree made, for the same reason and
# discovered the same way. The startup code programs MSPLIM, so a stack overflow
# traps rather than quietly corrupting - and the command handlers need more than
# 1 KB. `bb_reply_okf()` alone puts a 256-byte format buffer and a 264-byte line
# buffer on the stack.
#
# What it looked like before the fix: the board booted, printed its banner,
# accepted a command over the console, tokenised it correctly, entered the
# handler - and then vanished into DefaultISR inside `bb_board_silicon()`, a
# function that does nothing but memset a struct on the stack. A function that
# cannot fault, faulting, is what points at the stack rather than at the code.
#
# 0x2000 matches the STM32 build's choice. The pattern below is the SDK's own,
# from examples/_boards/frdmimxrt1186/component_examples/flash_component.
mcux_add_iar_configuration(
    LD "--config_def=__stack_size__=0x2000"
)
mcux_add_mdk_configuration(
    LD "--predefine=\"-D__stack_size__=0x2000\""
)
mcux_add_armgcc_configuration(
    LD "-Xlinker --defsym=__stack_size__=0x2000"
)

# The mailbox has to be non-cacheable on both cores, and the SDK already has a
# mechanism for saying so.
#
# BOARD_ConfigMPU() deliberately leaves OCRAM2 (0x20500000) without a region on
# either core, so it falls through to the background system address map - which
# its own comment describes as "normal, write back, write/read allocate". That is
# right for OCRAM used as ordinary RAM and wrong for a mailbox: the controller's
# writes sat in XCACHE and the capture core read stale zeros, so it spun forever
# on a magic it could never see while its own beacon counter climbed happily.
#
# `__USE_SHMEM` turns on the region that fixes it - region 9 on the CM33, region
# 12 on the CM7 - and the linker symbol `__use_shmem__` is what makes the linker
# script export the addresses it needs. Both are required: the C macro compiles
# the code in, the linker symbol gives it a non-zero region to configure.
#
# The region is m_ocram2_start for m_shmem_size, which is 0x20500000 for 256 KB -
# aligned and a power of two, as that code asserts.
#
# This replaces a hand-rolled MPU override that previously lived in the CM7 board
# port. Two mechanisms configuring one window is how you end up debugging the one
# that is not in effect; and that one used region 12, which is precisely the
# region the SDK's own path programs.
mcux_add_armgcc_configuration(
    LD "-Xlinker --defsym=__use_shmem__=1"
)
