/*
 * Board port for the BUSBRIDGE-RT capture core, CM7.
 *
 * Deliberately thinner than the CM33's. This image drives no console and no
 * USB - the controller core owns both - so `BOARD_InitDebugConsole()` is not
 * called. That is not a saving, it is a correctness point: LPUART1 belongs to
 * the CM33, and two cores initialising one peripheral is how a console starts
 * dropping characters for reasons that look like a baud-rate problem.
 *
 * SysTick is not configured either. Nothing here needs a millisecond tick: the
 * capture timebase comes from FlexIO's own clock, and the link is polled rather
 * than timed.
 */
#include "app.h"
#include "board.h"
#include "clock_config.h"
#include "pin_mux.h"

void BOARD_InitHardware(void)
{
    BOARD_CommonSetting();
    /* This now includes a non-cacheable region over the CM33 <-> CM7 mailbox,
     * because the image is built with __USE_SHMEM and the matching linker
     * symbol. An earlier version overrode the MPU here by hand, duplicating what
     * that flag already does - and using the very region number it programs. */
    BOARD_ConfigMPU();
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
}
