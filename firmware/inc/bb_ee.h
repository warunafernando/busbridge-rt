/**
 * @file bb_ee.h
 * @brief The one kilobyte every target on this board serves.
 *
 * The point of the peripheral profile is that I3C, I2C, SPI and MDIO are four
 * doors into the *same* array. Write 0xAA at 0x100 over SPI, read it back over
 * I3C, see it in the host's EE:RD - that is a test worth running, and it is
 * only worth running if there is exactly one copy of the data.
 *
 * So this module owns the bytes and the counters, and knows nothing about any
 * bus. Each target back-end translates its own protocol into bb_ee_read() and
 * bb_ee_write() calls and nothing more.
 *
 * RAM-backed, deliberately. A real EEPROM is non-volatile, but a device under
 * test that comes up with known contents after every reset is worth more than
 * one that remembers a previous experiment - and flash wear from a write loop
 * is a poor way to discover that. EE:SAVE is not offered for the same reason.
 */
#ifndef BB_EE_H
#define BB_EE_H

#include "bb_status.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/** 1 KB, so a 16-bit address covers it with room to spare. */
#define BB_EE_SIZE      1024u

/** Address mask. Access wraps rather than faulting, as a real EEPROM does. */
#define BB_EE_MASK      (BB_EE_SIZE - 1u)

/** Which door a byte came through. Counted per interface so that "did my write
 *  land?" has an answer that does not depend on reading the data back. */
typedef enum {
    BB_EE_IF_HOST = 0,   /**< the USB/console EE: commands */
    BB_EE_IF_I2C,
    BB_EE_IF_I3C,
    BB_EE_IF_SPI,
    BB_EE_IF_MDIO,
    BB_EE_IF_COUNT
} bb_ee_if_t;

void bb_ee_init(void);

/** Read one byte. @p addr wraps to the array. */
uint8_t bb_ee_read(bb_ee_if_t from, uint16_t addr);

/** Write one byte, unless this interface is write-protected. */
void    bb_ee_write(bb_ee_if_t from, uint16_t addr, uint8_t value);

/** Bulk access for the host commands, which have the whole array in view. */
void    bb_ee_read_block(bb_ee_if_t from, uint16_t addr, uint8_t *out, size_t len);
void    bb_ee_write_block(bb_ee_if_t from, uint16_t addr, const uint8_t *in, size_t len);

/**
 * @brief Read a byte without counting it.
 *
 * For a back-end's own bookkeeping - the I3C target preloads a window of bytes
 * it may never send, and counting those as reads would report traffic the
 * controller never performed.
 */
uint8_t bb_ee_peek(uint16_t addr);

/**
 * @brief Add to an interface's read count without touching the data.
 *
 * Only the I3C target needs this, and only because it learns after the fact how
 * many bytes left the peripheral - see bb_ee_i3c.c. Every other back-end counts
 * as it goes.
 */
void    bb_ee_note_bytes_read(bb_ee_if_t from, uint16_t count);

/** Fill every byte with @p value. */
void    bb_ee_fill(uint8_t value);

/**
 * @brief Restore the boot pattern: ee[i] = i & 0xFF.
 *
 * Not zeros. A ramp makes an off-by-one address pointer visible at a glance -
 * the byte at 0x123 reads 0x23 - which zeros cannot do, and that is exactly the
 * bug a bus test is looking for.
 */
void    bb_ee_pattern(void);

/** Refuse writes from one interface, to prove a read-only path stays read-only. */
void    bb_ee_protect(bb_ee_if_t which, bool on);
bool    bb_ee_protected(bb_ee_if_t which);

/** Per-interface activity. Zeroed by bb_ee_clear_stats(). */
typedef struct {
    uint32_t reads;          /**< bytes read          */
    uint32_t writes;         /**< bytes written       */
    uint32_t transactions;   /**< framed bus accesses */
    uint32_t rejected;       /**< writes refused by the protect flag */
    uint16_t last_addr;      /**< address of the last byte touched   */
} bb_ee_stats_t;

/** A back-end calls this once per framed bus transaction, for the counter. */
void                 bb_ee_note_transaction(bb_ee_if_t from);
const bb_ee_stats_t *bb_ee_stats(bb_ee_if_t which);
void                 bb_ee_clear_stats(void);

/** Short name for a reply, e.g. "i2c". */
const char *bb_ee_if_name(bb_ee_if_t which);

void bb_ee_status(bb_status_t *out);

#endif /* BB_EE_H */
