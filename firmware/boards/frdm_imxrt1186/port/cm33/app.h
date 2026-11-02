#ifndef BB_APP_H
#define BB_APP_H

void BOARD_InitHardware(void);

/**
 * @brief Keep the EdgeLock Enclave awake.
 *
 * The RT1180 ELE must be pinged at least every 24 hours or the SoC resets.
 * There is no STM32 equivalent, and a bus controller left running on a bench
 * over a weekend is exactly the case that would hit it. Called from the
 * heartbeat, which already runs twice a second, so the interval is chosen for
 * margin rather than for cost.
 */
void bb_ele_keepalive(void);

#endif /* BB_APP_H */
