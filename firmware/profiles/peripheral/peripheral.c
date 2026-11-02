/**
 * @file peripheral.c
 * @brief The device-under-test image: answers as a target, not a controller.
 *
 * The second of the two images the STM32 tree built, and the reason it is worth
 * building: with two boards, one runs `bridge` and drives a bus, the other runs
 * this and answers on it. Every controller command then has something real to
 * talk to, which is the difference between "the code path works against an
 * empty header" and "the signalling works".
 *
 * `bb_profile.h` explains why this cannot be one image with a mode switch, and
 * that reasoning holds harder here than on the STM32: a controller and a target
 * both want the same LPI2C instance and the same pads, and on this board there
 * are only three I2C buses to go round.
 *
 * ## What answers, and what does not
 *
 *   I2C   LPI2C3 target at 0x50 on D14/D15. Working.
 *   SPI   LPSPI4 target on D10-D13, 0x03/0x02/0x05 command set. Working, with
 *         CS sampled rather than interrupted on - see bb_ee_spi.c.
 *   I3C   not implemented. Reports absent with the reason.
 *   MDIO  not implemented. Reports absent with the reason.
 *
 * Two of four, and the two that are missing say so rather than answering
 * incorrectly. For a device under test that distinction is the whole point: a
 * target that acknowledges and then misbehaves is far more expensive to debug
 * than one that is plainly not there.
 */
#include "bb_board.h"
#include "bb_ee.h"
#include "bb_ee_ports.h"
#include "bb_profile.h"

#include "app.h"

const char *const bb_profile_name    = "peripheral";
const char *const bb_profile_summary = "I2C+SPI targets over 1 KB shared EEPROM";

const bb_cmd_t *const bb_profile_tables[] = {
    bb_cmd_ee,
    NULL
};

void bb_profile_init(void)
{
    /* The array first: a target back-end can be addressed the instant its
     * peripheral is enabled, and answering out of an uninitialised array would
     * be a race that only shows up on a busy bus. */
    bb_ee_init();

    bb_ee_i2c_init();
    bb_ee_spi_init();
    bb_ee_i3c_init();     /* both of these are no-ops that report absent */
    bb_ee_mdio_init();
}

void bb_profile_poll(void)
{
    /* SPI's frame boundary is a CS level here rather than an edge interrupt, so
     * it has to be sampled. The I2C target needs nothing: LPI2C stretches the
     * clock and its interrupt does the work. */
    bb_ee_spi_poll();
    bb_ee_i3c_poll();
}

void bb_profile_tick(void)
{
    static uint32_t next_ping_ms;
    uint32_t        now = HAL_GetTick();

    /* Same ELE keepalive as the bridge image, and for the same reason: the
     * RT1180 resets the SoC if the enclave is not pinged within 24 hours. A
     * device under test left attached over a weekend is exactly the case that
     * hits it, and a target that silently reboots mid-experiment would look
     * like a bus fault. */
    if ((int32_t)(now - next_ping_ms) >= 0) {
        next_ping_ms = now + 60000u;
        bb_ele_keepalive();
    }
}

void bb_profile_note_command(const char *word, bool ok)
{
    (void)word;
    (void)ok;
}
