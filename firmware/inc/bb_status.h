/**
 * @file bb_status.h
 * @brief One-line self-description for each bus, for anything that displays it.
 *
 * Each module answers for itself rather than having a display or a host walk
 * its internals. That keeps the description next to the state it describes, and
 * means the on-board screen and the host monitor cannot disagree about what a
 * bus is doing.
 */
#ifndef BB_STATUS_H
#define BB_STATUS_H

#include <stdbool.h>

typedef enum {
    BB_STATE_ABSENT = 0,   /**< the MCU has it, this board does not bring it out */
    BB_STATE_IDLE,          /**< configured or not, but nothing happening        */
    BB_STATE_ACTIVE,        /**< running: clocked, or carrying traffic           */
    BB_STATE_PROBLEM        /**< something needs attention                      */
} bb_state_t;

typedef struct {
    char       pins[26];    /**< short pin summary, e.g. "PB6/PB7"   */
    char       detail[26];  /**< clock, counts, whatever fits        */
    bb_state_t state;
} bb_status_t;

void bb_i2c_status(unsigned bus, bb_status_t *out);   /**< bus is 1-based */
void bb_i3c_status(bb_status_t *out);
void bb_spi_status(bb_status_t *out);
void bb_mdio_status(bb_status_t *out);
void bb_uart_status(bb_status_t *out);
void bb_sniff_status_line(bb_status_t *out);

#endif /* BB_STATUS_H */
