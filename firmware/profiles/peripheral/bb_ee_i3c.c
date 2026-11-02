/**
 * @file bb_ee_i3c.c
 * @brief I3C target — declared, not yet implemented, and says so.
 *
 * The STM32 build had a working I3C target (259 lines) and it was genuinely
 * useful: it is how the controller image's ENTDAA was exercised at all without
 * buying an I3C device. It should exist here too.
 *
 * It does not yet, and the reason is worth stating rather than hiding behind a
 * stub that returns success. An I3C target has to answer ENTDAA with its own
 * 48-bit provisional ID, accept the dynamic address the controller assigns,
 * distinguish a T-bit from an acknowledge on every ninth bit, and raise in-band
 * interrupts - and each of those is a place where "it linked and reported ready"
 * is much worse than "not implemented". A controller that runs ENTDAA against a
 * target which acknowledges 0x7E and then behaves incorrectly produces a bus
 * full of devices at addresses nobody agreed on, which is far harder to debug
 * than an empty bus.
 *
 * So this reports BB_STATE_ABSENT with the reason, `EE:PORTS?` shows it, and
 * nothing claims to be listening.
 *
 * Also to settle when it is written: which port. I3C1 shares its pads with I2C
 * bus 2 on the motor connector, and I2C bus 3 is already the I2C target - so the
 * I3C target would go on I3C2 (A4/A5), which shares nothing. See docs/PINS.md.
 */
#include "bb_ee.h"
#include "bb_ee_ports.h"

#include <stdio.h>
#include <string.h>

void bb_ee_i3c_init(void)
{
}

void bb_ee_i3c_ev_irq(void)
{
}

void bb_ee_i3c_er_irq(void)
{
}

bool bb_ee_i3c_enabled(void)
{
    return false;
}

void bb_ee_i3c_poll(void)
{
}

void bb_ee_i3c_describe(char *out, size_t cap)
{
    (void)snprintf(out, cap, "i3c not-implemented");
}

void bb_ee_i3c_status(bb_status_t *out)
{
    if (out == NULL) {
        return;
    }
    memset(out, 0, sizeof *out);
    (void)snprintf(out->pins,   sizeof out->pins,   "-");
    (void)snprintf(out->detail, sizeof out->detail, "not-implemented");
    out->state = BB_STATE_ABSENT;
}
