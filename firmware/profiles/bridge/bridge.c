/**
 * @file bridge.c
 * @brief The bus controller image: what it does with the board's pins.
 *
 * PHASE 0. The board layer, the transports and the command pump are real; the
 * buses are not here yet. `bb_profile_tables` is empty, so the only commands
 * this image answers are the SYS namespace that `bb_cmd.c` always adds - which
 * is exactly what Phase 0 is for: proving the clock tree, the console, the
 * command loop and the pin map on new silicon before any bus is layered on top.
 *
 * The buses arrive in Phase 2, one table at a time, and nothing else in the
 * tree changes when they do. That is the point of the profile indirection -
 * `main.c` is written against three hooks rather than against a list of buses.
 */
#include "bb_profile.h"
#include "bb_board.h"
#include "bb_can.h"
#include "bb_link.h"
#include "bb_sniff.h"
#include "bb_i2c.h"
#include "bb_i3c.h"
#include "bb_mdio.h"
#include "bb_spi.h"
#include "bb_uart.h"

#include "app.h"

const char *const bb_profile_name    = "bridge";
const char *const bb_profile_summary = "3 I2C, 2 I3C, SPI, MDIO, UART, CAN, sniffer, PHYs";

/* NULL-terminated rather than counted, so adding a bus touches this list and
 * nothing else - no length to keep in step. bb_cmd.c prepends SYS. */
const bb_cmd_t *const bb_profile_tables[] = {
    bb_cmd_i2c,
    bb_cmd_i3c,
    bb_cmd_spi,
    bb_cmd_mdio,
    bb_cmd_uart,
    bb_cmd_can,
    bb_cmd_sniff,
    bb_cmd_net,
    NULL
};

void bb_profile_init(void)
{
    bb_i2c_init();
    bb_i3c_init();
    bb_spi_init();
    bb_mdio_init();
    bb_uart_init();
    bb_can_init();
    bb_sniff_init();
}

void bb_profile_poll(void)
{
    /* Transparent UART bridge mode needs servicing faster than a command
     * arrives - it is a wire, not a request/response protocol. Everything else
     * here is synchronous and has nothing to poll. */
    bb_uart_poll();
    bb_can_poll();
    /*
     * In-band interrupts and hot-joins arrive when the target decides, not when
     * a host asks - so they have to be serviced here.
     *
     * This used to be driven only from I3C:IBI? and I3C:HJ?, which works for an
     * IBI because the host reads one after causing it, and does not work at all
     * for a hot-join: the target sends its request and waits synchronously for
     * an address, while the controller sits idle until somebody issues a
     * command. The target always timed out first, and the address only appeared
     * once the next I3C:HJ? happened to run the poll. Measured exactly that
     * way - `addr=none` immediately after the request, `addr=0x09` after the
     * controller was next poked.
     *
     * Cheap when nothing is pending: bb_i3c_poll() returns on a listening check
     * and then on one register read.
     */
    bb_i3c_poll();
    /* Probes the capture core while it is down, so SNIFF:* can say so at once
     * rather than each command paying a timeout. */
    bb_link_poll();
}

/*
 * The heartbeat, twice a second.
 *
 * The ELE keepalive lives here rather than in the SysTick handler. It has to
 * happen - the RT1180 resets the SoC if the enclave is not pinged within 24
 * hours, which is a failure mode the STM32 build had no equivalent of and which
 * would show up as a board that "randomly reboots over the weekend". It does
 * not have to happen in an interrupt.
 *
 * Pinged once a minute rather than once a day: the cost is a mailbox round trip
 * against a 500 ms heartbeat, and 23 hours of accumulated error on a software
 * tick counter is not something to bet an unattended run on.
 */
void bb_profile_tick(void)
{
    static uint32_t next_ping_ms;
    uint32_t        now = HAL_GetTick();

    if ((int32_t)(now - next_ping_ms) >= 0) {
        next_ping_ms = now + 60000u;
        bb_ele_keepalive();
    }
}

void bb_profile_note_command(const char *word, bool ok)
{
    /* This board has no display, so there is nowhere to show the last command.
     * The hook stays because the peripheral profile and any future board with a
     * panel use it, and because main.c should not have to know which. */
    (void)word;
    (void)ok;
}
