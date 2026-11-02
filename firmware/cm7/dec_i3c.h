/**
 * @file dec_i3c.h
 * @brief Native I3C SDR decoder - separate from the I2C one on purpose.
 *
 * The two protocols share electrical ancestry and almost nothing else at the
 * transaction level. The difference that matters most is the ninth bit: I2C
 * puts an acknowledge there, driven by the RECEIVER, while I3C SDR puts a
 * T-bit there, driven by the TRANSMITTER - parity on a write, end-of-data on a
 * read. Both framings are nine bits and the first eight decode identically, so
 * an I2C decoder pointed at an I3C bus produces addresses that are right and
 * payload that is plausible and wrong. That is the worst possible failure: it
 * looks believable.
 *
 * Kept in its own file rather than behind `if (proto == I3C)` branches in the
 * I2C decoder, because the state machines genuinely differ - CCCs, ENTDAA and
 * the broadcast address have no I2C counterpart - and interleaving them would
 * make both harder to reason about.
 */
#ifndef BB_DEC_I3C_H
#define BB_DEC_I3C_H

#include "bb_capture.h"
#include <stdbool.h>
#include <stdint.h>

/** How the decoder hands finished events back. */
typedef void (*dec_i3c_emit_fn)(bb_ev_kind_t kind, uint8_t value, bool flag);

void dec_i3c_init(dec_i3c_emit_fn emit);
void dec_i3c_reset(void);

/** Framing, from whatever detected it - these do not come from the bit stream. */
void dec_i3c_start(bool repeated);
void dec_i3c_stop(void);

/** One data bit, sampled where the bus said it was valid. */
void dec_i3c_bit(uint8_t bit);

/**
 * @brief Was the transfer that just closed a read?
 *
 * Taken from the address's direction bit. It matters to the capture layer
 * because a read ends differently on the wire: the target releases SDA while
 * the clock is held, which is one more clock edge than a write leaves behind.
 */
bool dec_i3c_was_read(void);

/** @brief Tag which call path is about to close a frame: 1 oversample, 2 pin-clocked. */
void dec_i3c_site(uint8_t site);

/**
 * @brief Drain the frame-close log.
 *
 * One word per close, packed: site, why it ended, and the decoder state at that
 * moment. Reading it clears it.
 */
unsigned dec_i3c_close_log(uint32_t *out, unsigned max);

/** True while a frame is open, for callers that gate byte assembly on it. */
bool dec_i3c_in_frame(void);

/** True between bytes. A deferred STOP has to land on a boundary, never
 *  part-way through a byte the bus really did send. */
bool dec_i3c_at_boundary(void);

#endif /* BB_DEC_I3C_H */
