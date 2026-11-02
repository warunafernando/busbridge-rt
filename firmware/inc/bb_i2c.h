/**
 * @file bb_i2c.h
 * @brief LPI2C controllers — three independent buses.
 *
 * Bus 1 is LPI2C1 on D0/D1, bus 2 is LPI2C2 on the motor connector J3, bus 3 is
 * LPI2C3 on the Arduino I2C position D14/D15. Commands act on whichever bus
 * `I2C:BUS` selected; every bus keeps its own frequency, so selecting one never
 * disturbs another.
 *
 * Two board facts shape this and are worth knowing before wiring anything:
 *
 *   - Three buses, not the STM32's four. LPI2C6 reaches no connector on this
 *     package and LPI2C2 only reaches J3. `I2C:PORTS?` reports the truth.
 *   - Bus 1 is off by default. Its pads are the MCU-Link console. Enabling it
 *     means moving jumpers J41/J43 and building with `BB_I2C1_ENABLE=1`, which
 *     costs the console. Until then it reports itself unavailable with the
 *     reason, rather than accepting transfers that go nowhere.
 *
 * The default selection is bus 3, not bus 1 as on the STM32 - otherwise every
 * fresh board would refuse its first transfer.
 */
#ifndef BB_I2C_H
#define BB_I2C_H

#include <stdbool.h>
#include <stdint.h>

void bb_i2c_init(void);

/*
 * No interrupt vectors here.
 *
 * The STM32 build needed bb_i2c_ev_irq/bb_i2c_er_irq because ST's sequential
 * (repeated-start) API is interrupt-driven: a write-then-read had to be
 * assembled across two callbacks. LPI2C carries the sub-address in the transfer
 * itself and performs the repeated START in hardware, so a register read is one
 * blocking call and there is nothing for a vector to do.
 *
 * They are gone rather than stubbed: a vector that exists and does nothing
 * invites someone to wire it up.
 */

#endif /* BB_I2C_H */
