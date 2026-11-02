/**
 * @file bb_ee_ports.h
 * @brief The four target back-ends of the peripheral profile.
 *
 * Each one translates its own bus protocol into bb_ee_read()/bb_ee_write() and
 * owns nothing else. They are listed together here because the profile file has
 * to start them and the vectors have to reach them; they do not call each other,
 * with one exception - the I2C target asks the I3C target whether it is using
 * the shared pins, because on a 64-pin part that is a real conflict and a silent
 * bus is a bad way to find out.
 */
#ifndef BB_EE_PORTS_H
#define BB_EE_PORTS_H

#include "bb_cmd.h"
#include "bb_status.h"
#include <stdbool.h>
#include <stddef.h>

/* ---- I2C target --------------------------------------------------------- */
void bb_ee_i2c_init(void);
void bb_ee_i2c_ev_irq(unsigned bus);      /**< bus is 0-based */
void bb_ee_i2c_er_irq(unsigned bus);
void bb_ee_i2c_describe(char *out, size_t cap);
void bb_ee_i2c_status(bb_status_t *out);

/* ---- I3C target --------------------------------------------------------- */
void bb_ee_i3c_init(void);
void bb_ee_i3c_ev_irq(void);
void bb_ee_i3c_er_irq(void);
bool bb_ee_i3c_enabled(void);
void bb_ee_i3c_describe(char *out, size_t cap);
void bb_ee_i3c_status(bb_status_t *out);
/** Serviced from the command pump; the other three run from interrupts. */
void bb_ee_i3c_poll(void);

/* ---- SPI slave ---------------------------------------------------------- */
void bb_ee_spi_init(void);
void bb_ee_spi_irq(void);
void bb_ee_spi_cs_edge(void);
/* Polled on this board: RGPIO edge routing for the CS pad is not set up, so the
 * frame boundary is sampled rather than interrupted on. A frame shorter than
 * the poll interval is missed, and bb_ee_spi_describe() reports cs=polled so
 * that limitation stays visible rather than being assumed away. */
void bb_ee_spi_poll(void);
void bb_ee_spi_describe(char *out, size_t cap);
void bb_ee_spi_status(bb_status_t *out);
/** Which EXTI line the board's chip select pin is on. */
unsigned bb_ee_spi_cs_line(void);

/* ---- MDIO slave --------------------------------------------------------- */
void bb_ee_mdio_init(void);
void bb_ee_mdio_mdc_edge(void);
void bb_ee_mdio_describe(char *out, size_t cap);
void bb_ee_mdio_status(bb_status_t *out);
/** Which EXTI line the board's MDC pin is on. */
unsigned bb_ee_mdio_mdc_line(void);

/**
 * @brief One EXTI line fired.
 *
 * All sixteen vectors land here rather than the two this profile happens to
 * need being named in every board header. Which line belongs to the SPI chip
 * select and which to MDC is a board fact already in the pin map, so asking the
 * board for a vector name as well would be a third place to keep in step.
 */
void bb_ee_exti(unsigned line);

/* ---- command tables ----------------------------------------------------- */
extern const bb_cmd_t bb_cmd_ee[];
extern const bb_cmd_t bb_cmd_ee_i2c[];
extern const bb_cmd_t bb_cmd_ee_i3c[];
extern const bb_cmd_t bb_cmd_ee_spi[];
extern const bb_cmd_t bb_cmd_ee_mdio[];

#endif /* BB_EE_PORTS_H */
