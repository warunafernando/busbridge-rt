# Stack: 8 KB, not the device linker script's default 1 KB.
#
# The same edit the CM33 port needed, for the same reason, and found the same way
# a second time - which is the interesting part. The capture core's message
# handler builds its replies on the stack, and adding the decoded-event drain
# gave handle() a second 480-byte buffer alongside the raw one. That pushed the
# frame past 1 KB, the startup code programs MSPLIM so the overflow trapped, and
# the core simply stopped.
#
# What it looked like: SNIFF:TEST timed out after 8 s, and SNIFF:LINK? reported
# `alive=1 ticks=80968653 delta=0 bticks=80968653 bdelta=0`. Both counters
# frozen at the same value is a core that is not executing - not a slow reply,
# not a mailbox problem. Having the beacon there made that a one-command
# diagnosis instead of an afternoon.
#
# 0x2000 matches the controller's choice. A capture core whose stack is smaller
# than its largest reply is a core that works until a message grows.
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
