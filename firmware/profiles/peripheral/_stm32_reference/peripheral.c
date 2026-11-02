/**
 * @file profiles/peripheral/peripheral.c
 * @brief The device-under-test profile: four bus targets over one EEPROM.
 *
 * This is the other half of a two-board test. One board runs the bridge profile
 * and drives a bus; this one answers on it. Everything the controller writes is
 * visible to the other three buses and to the host's own EE: commands, which is
 * what makes a cross-bus test - write over SPI, read over I3C - mean something.
 *
 * The four back-ends come up together where the board's pins allow it. On a
 * 64-pin part I2C1 and I3C1 are the same two pins, so the I2C target moves to
 * the first bus that is not shared; see bb_ee_i2c.c.
 */
#include "bb_profile.h"

#include "bb_ee.h"
#include "bb_ee_ports.h"

const char *const bb_profile_name    = "peripheral";
const char *const bb_profile_summary = "I3C/I2C/SPI/MDIO targets over 1 KB shared EEPROM";

void bb_profile_init(void)
{
    /* The array first: a back-end may answer a bus transaction before this
     * function returns, and answering from uninitialised memory would be a
     * confusing first impression. */
    bb_ee_init();

    bb_ee_i2c_init();
    bb_ee_i3c_init();
    bb_ee_spi_init();
    bb_ee_mdio_init();
}

void bb_profile_poll(void)
{
    /* Only the I3C target needs the pump. The other three are driven entirely
     * from their interrupts - see bb_ee_i3c.c for why that one cannot be. */
    bb_ee_i3c_poll();
}

void bb_profile_tick(void)
{
    /* Nothing on the heartbeat: no panel in this profile, and every target
     * answers from its own interrupt or from the pump. */
}

void bb_profile_note_command(const char *word, bool ok)
{
    /* Nothing to show it on. The counters in EE:STAT? are the record that
     * matters here, and they are kept by the thing being counted. */
    (void)word;
    (void)ok;
}

const bb_cmd_t *const bb_profile_tables[] = {
    bb_cmd_ee, bb_cmd_ee_i2c, bb_cmd_ee_i3c, bb_cmd_ee_spi, bb_cmd_ee_mdio,
    NULL,
};
