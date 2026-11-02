/**
 * @file profiles/peripheral/bb_ee.c
 * @brief The shared EEPROM image. See bb_ee.h for why it is RAM-backed.
 *
 * Every access goes through one of two functions, which is what lets the
 * counters be trusted: there is no path to the bytes that does not count.
 */
#include "bb_ee.h"

#include <stdio.h>
#include <string.h>

static uint8_t       ee[BB_EE_SIZE];
static bb_ee_stats_t stats[BB_EE_IF_COUNT];
static uint8_t       protect_mask;

static const char *const if_names[BB_EE_IF_COUNT] = {
    "host", "i2c", "i3c", "spi", "mdio"
};

void bb_ee_init(void)
{
    protect_mask = 0;
    bb_ee_clear_stats();
    bb_ee_pattern();
}

uint8_t bb_ee_read(bb_ee_if_t from, uint16_t addr)
{
    uint16_t a = (uint16_t)(addr & BB_EE_MASK);

    if (from < BB_EE_IF_COUNT) {
        stats[from].reads++;
        stats[from].last_addr = a;
    }
    return ee[a];
}

void bb_ee_write(bb_ee_if_t from, uint16_t addr, uint8_t value)
{
    uint16_t a = (uint16_t)(addr & BB_EE_MASK);

    if (from < BB_EE_IF_COUNT) {
        if ((protect_mask & (1u << from)) != 0u) {
            stats[from].rejected++;
            return;
        }
        stats[from].writes++;
        stats[from].last_addr = a;
    }
    ee[a] = value;
}

uint8_t bb_ee_peek(uint16_t addr)
{
    return ee[addr & BB_EE_MASK];
}

void bb_ee_note_bytes_read(bb_ee_if_t from, uint16_t count)
{
    if (from < BB_EE_IF_COUNT) {
        stats[from].reads += count;
    }
}

void bb_ee_read_block(bb_ee_if_t from, uint16_t addr, uint8_t *out, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        out[i] = bb_ee_read(from, (uint16_t)(addr + i));
    }
}

void bb_ee_write_block(bb_ee_if_t from, uint16_t addr, const uint8_t *in, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        bb_ee_write(from, (uint16_t)(addr + i), in[i]);
    }
}

void bb_ee_fill(uint8_t value)
{
    memset(ee, value, sizeof ee);
}

void bb_ee_pattern(void)
{
    for (size_t i = 0; i < BB_EE_SIZE; i++) {
        ee[i] = (uint8_t)i;
    }
}

void bb_ee_protect(bb_ee_if_t which, bool on)
{
    if (which < BB_EE_IF_COUNT) {
        if (on) {
            protect_mask |= (uint8_t)(1u << which);
        } else {
            protect_mask &= (uint8_t)~(1u << which);
        }
    }
}

bool bb_ee_protected(bb_ee_if_t which)
{
    return which < BB_EE_IF_COUNT && (protect_mask & (1u << which)) != 0u;
}

void bb_ee_note_transaction(bb_ee_if_t from)
{
    if (from < BB_EE_IF_COUNT) {
        stats[from].transactions++;
    }
}

const bb_ee_stats_t *bb_ee_stats(bb_ee_if_t which)
{
    return (which < BB_EE_IF_COUNT) ? &stats[which] : &stats[BB_EE_IF_HOST];
}

void bb_ee_clear_stats(void)
{
    memset(stats, 0, sizeof stats);
}

const char *bb_ee_if_name(bb_ee_if_t which)
{
    return (which < BB_EE_IF_COUNT) ? if_names[which] : "?";
}

void bb_ee_status(bb_status_t *out)
{
    uint32_t reads = 0;
    uint32_t writes = 0;

    for (unsigned i = 0; i < BB_EE_IF_COUNT; i++) {
        reads += stats[i].reads;
        writes += stats[i].writes;
    }

    snprintf(out->pins, sizeof out->pins, "%uB shared", (unsigned)BB_EE_SIZE);
    snprintf(out->detail, sizeof out->detail, "r%lu w%lu",
             (unsigned long)reads, (unsigned long)writes);
    out->state = (reads + writes) > 0u ? BB_STATE_ACTIVE : BB_STATE_IDLE;
}
