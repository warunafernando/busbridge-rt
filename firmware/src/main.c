/**
 * @file main.c
 * @brief Host transports and the command pump, for either profile.
 *
 * What an image does with the board's pins is bb_profile_*: the bridge profile
 * drives five buses from the host, the peripheral profile answers as a target
 * on them. Everything in this file is common to both - two host transports, one
 * command pump, a heartbeat.
 *
 * Two host transports are live at once and either can issue commands:
 *
 *   - the ST-LINK Virtual COM Port (USART3), which needs no firmware USB stack
 *     and works over the cable that powers the board;
 *   - the native USB device on the user USB-C connector (CN13), a vendor
 *     interface with bulk endpoints that Windows binds to WinUSB automatically.
 *
 * The bulk path is roughly a hundred times the raw bandwidth of the VCP and
 * costs one USB transaction per command instead of a UART round trip. Replies
 * follow whichever transport carried the command, which is what bb_io does.
 *
 * The main loop is deliberately a single-threaded command pump: bus
 * transactions are short and strictly request/response, so there is nothing to
 * gain from an RTOS and a lot of determinism to lose.
 */
#include "bb_board.h"
#include "bb_cmd.h"
#include "bb_console.h"
#include "bb_io.h"
#include "bb_profile.h"
#include "bb_usb.h"
#include "bb_version.h"

#define HEARTBEAT_MS  500u

/** Run one line from a transport, with replies routed back to it. */
static void dispatch(char *line, int status, bb_io_target_t from)
{
    if (status < 0 && status != -2) {
        return;
    }
    bb_io_select(from);
    bb_led_indicate(BB_LED_ACTIVITY, true);
    if (status == -2) {
        bb_reply_err(BB_ERR_TOO_LONG, "line-too-long");
    } else {
        bb_cmd_execute(line);
    }
    bb_led_indicate(BB_LED_ACTIVITY, false);
}

int main(void)
{
    /* Static rather than automatic: half a kilobyte of command line has no
     * business competing with the bus handlers for stack. */
    static char console_line[BB_CMD_MAX_LINE];
    static char usb_line[BB_CMD_MAX_LINE];
    uint32_t    last_beat = 0;

    bb_board_init();
    bb_console_init();
    bb_usb_init();
    bb_cmd_init();
    bb_profile_init();

    bb_io_select(BB_IO_CONSOLE);
    bb_console_printf("\r\n# %s %s (%s) on %s ready\r\n",
                      BB_PRODUCT, BB_VERSION, bb_profile_name, BB_BOARD);
    bb_console_printf("# %s\r\n", bb_profile_summary);
    bb_console_puts("# type HELP for the command list\r\n");

    for (;;) {
        dispatch(console_line,
                 bb_console_getline(console_line, sizeof console_line),
                 BB_IO_CONSOLE);

        if (bb_usb_ready()) {
            dispatch(usb_line,
                     bb_usb_getline(usb_line, sizeof usb_line),
                     BB_IO_USB);
        }

        bb_profile_poll();


        if (HAL_GetTick() - last_beat >= HEARTBEAT_MS) {
            last_beat = HAL_GetTick();
            bb_led_toggle(BB_LED_HEARTBEAT);
            bb_profile_tick();
        }
    }
}
