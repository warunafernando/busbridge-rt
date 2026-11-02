/**
 * @file profiles/bridge/bridge.c
 * @brief The bus-controller profile: what main.c used to hardcode.
 *
 * This is the original firmware's behaviour, unchanged - five buses driven from
 * the host, plus the sniffer and the on-board panel. It is a file rather than a
 * list in main.c so that a second profile costs nothing here.
 */
#include "bb_profile.h"

#include "bb_i2c.h"
#include "bb_i3c.h"
#include "bb_lcd.h"
#include "bb_mdio.h"
#include "bb_sniff.h"
#include "bb_spi.h"
#include "bb_uart.h"

const char *const bb_profile_name    = "bridge";
const char *const bb_profile_summary = "I3C/I2C/SPI/MDIO/serial controller";

void bb_profile_init(void)
{
    bb_i2c_init();
    bb_i3c_init();
    bb_spi_init();
    bb_mdio_init();
    bb_sniff_init();
    bb_lcd_init();
    bb_uart_init();
}

void bb_profile_poll(void)
{
    /* Transparent UART bridge mode runs its own inner pump. */
    bb_uart_poll();
}

void bb_profile_tick(void)
{
    /* The panel is refreshed on the heartbeat, not every pass: a repaint is
     * thousands of bus writes and the command pump has nothing to gain from
     * doing it faster than a human can read. */
    bb_lcd_tick();
}

void bb_profile_note_command(const char *word, bool ok)
{
    bb_lcd_note_command(word, ok);
}

const bb_cmd_t *const bb_profile_tables[] = {
    bb_cmd_i2c, bb_cmd_i3c, bb_cmd_spi, bb_cmd_mdio, bb_cmd_uart,
    bb_cmd_sniff, bb_cmd_lcd,
    NULL,
};
