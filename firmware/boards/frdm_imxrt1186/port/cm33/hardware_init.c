/*
 * Board port for BUSBRIDGE-RT on the FRDM-IMXRT1186, CM33.
 *
 * This is the seam between the SDK's generated setup and this project's board
 * layer. The SDK owns the clock tree, the MPU regions and the boot pin mux -
 * on the RT1180 those three plus the ELE and the TRDC all have to agree before
 * a peripheral answers, and re-deriving that by hand would be a second source
 * of truth that quietly drifts from the SDK's.
 *
 * Two deliberate differences from the SDK's own hello_world port:
 *
 *   - SysTick is not configured here. bb_board.c owns the tick, because the
 *     cycle counter is derived from the same reload value and the two cannot be
 *     set up in different places without disagreeing.
 *
 *   - the ELE keepalive is a function rather than a counter buried in the
 *     SysTick handler. It has to happen; it does not have to happen in an
 *     interrupt, and putting it in the heartbeat keeps the handler to one
 *     increment.
 */
#include "app.h"
#include "board.h"
#include "clock_config.h"
#include "pin_mux.h"

#include "fsl_ele_base_api.h"

void BOARD_InitHardware(void)
{
    BOARD_CommonSetting();
    BOARD_ConfigMPU();
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
}

/*
 * The ELE resets the SoC if it is not pinged within 24 hours. The SDK's demos
 * mostly ignore this because they do not run that long; a bus controller left
 * attached to a bench over a weekend is precisely the case that does.
 *
 * Pinged well inside the window rather than at it: the tick this is driven from
 * is a software counter off SysTick, and 23 hours of accumulated clock error is
 * not a number worth trusting to the last minute.
 */
void bb_ele_keepalive(void)
{
    (void)ELE_BaseAPI_Ping(MU_RT_S3MUA);
}
