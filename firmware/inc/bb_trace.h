/**
 * @file bb_trace.h
 * @brief Controller-side transaction trace - the generating half of
 *        `SNIFF:SRC self`.
 *
 * The tag plumbing for `observed=controller` was complete long before anything
 * generated a record to put it on: bb_sniff.c set the value, push_config()
 * passed it to the capture core, and the core stamped it into every record
 * header. So selecting `self` labelled FlexIO *wire samples* as
 * controller-observed, which is precisely the confusion the tag exists to
 * prevent, and `cmd_src()` refused rather than ship it. This file is what was
 * missing.
 *
 * WHAT A CONTROLLER-SIDE RECORD IS, AND IS NOT
 *
 * It is what our own I3C peripheral accepted or returned. It is not a
 * measurement of the bus. The two differ exactly when something is wrong,
 * which is when a sniffer earns its keep, so the difference is kept visible
 * rather than averaged away:
 *
 *   - **START / STOP** are the controller's own framing requests completing.
 *     Observed in the sense that MCTRLDONE came up for a request we issued.
 *   - **The address byte and its ACK** are genuinely observed. MERRWARN.NACK
 *     says whether a target answered, and nothing else on this core knows that.
 *   - **Data byte values** are observed, with different force in each
 *     direction. On a read the byte came out of the receive FIFO - the
 *     peripheral really did take it off the wire. On a write the byte went
 *     into the transmit FIFO, which says we handed it over, not that it was
 *     clocked out.
 *   - **The ninth bit of a data byte is NOT observed.** On I3C SDR it is a
 *     T-bit driven by the transmitter: odd parity on a write, end-of-data on a
 *     read. The controller hardware generates it on a write and consumes it on
 *     a read, and exposes neither. So the flag on a data event is *computed*
 *     from the byte and its position, and each event says so in `derived`.
 *
 * That last point is the one with teeth. A computed T-bit can never disagree
 * with the byte it was computed from, so it carries no evidence whatsoever,
 * and a host that compared a self-trace T-bit against a wire T-bit and found
 * them equal would have learned nothing while feeling it had. `derived` is how
 * that host is told, and `SNIFF:READ?` repeats it in the reply header.
 *
 * WHAT IS NOT TRACED, AND WHY IT IS A `!` RATHER THAN A SILENCE
 *
 * CCC and ENTDAA go through the SDK's blocking transfer rather than the
 * register-level paths here, so this file cannot describe their bytes. Leaving
 * them out silently would make a trace that is wrong rather than incomplete -
 * a host merging a self trace against a wire capture would see frames on the
 * wire with nothing beside them and conclude the controller never sent them.
 * So an untraceable frame pushes BB_EV_DROPPED, which is the `!` token the
 * host library has always read as "the stream has a hole here".
 *
 * CONCURRENCY
 *
 * There is none, and that is a constraint rather than an accident. Pushes
 * happen inside a bus transfer and drains happen inside a `SNIFF:*` command;
 * both run in the single command loop on the CM33, one after another. No lock
 * is taken because taking one would imply a concurrency that does not exist
 * and would be the only thing here claiming it. If this is ever pushed from an
 * interrupt - IBI service is the obvious candidate - that stops being true and
 * the ring needs revisiting, not a lock bolted on.
 *
 * TIME
 *
 * `t_cyc` is bb_cycles(), which is SysTick-derived at bb_cycle_hz() rather
 * than DWT->CYCCNT, for the reason given in bb_board.c: CYCCNT stops when a
 * debugger clears TRCENA on detach. It wraps at 2^32 cycles, about 17.9 s at
 * 240 MHz, so a trace armed for longer than that reports elapsed times that
 * have wrapped. Captures are milliseconds; this is documented rather than
 * guarded because a guard would cost a 64-bit accumulate on every event to fix
 * a case no capture reaches.
 *
 * This timebase is the CM33's. The capture core's is its own sample clock,
 * zeroed when IT started. Nothing here relates the two, which is exactly why
 * `SNIFF:SRC both` still refuses: merging two streams on timestamp needs a
 * known offset between the timebases and a bound on their relative drift, and
 * neither has been measured.
 */
#ifndef BB_TRACE_H
#define BB_TRACE_H

#include <stdbool.h>
#include <stdint.h>

#include "bb_capture.h"   /* bb_ev_kind_t - one event vocabulary, not two */

/*
 * 512 entries at 8 bytes. Sized against the traffic a single drain cycle sees
 * rather than against the RAM: a six-byte private read is nine events, so this
 * holds about fifty transfers between drains, and m_data has the room.
 */
#define BB_TRACE_RING  512u

typedef struct {
    uint32_t t_cyc;     /**< bb_cycles() when the event was recorded */
    uint8_t  kind;      /**< bb_ev_kind_t */
    uint8_t  value;     /**< the byte, for ADDR and DATA */
    uint8_t  flag;      /**< ninth bit: ACK on I2C, T on I3C */
    uint8_t  derived;   /**< 1 when `flag` was computed, not observed */
} bb_trace_ev_t;

/** Start recording. Does not clear; call bb_trace_clear() for that. */
void bb_trace_arm(void);
void bb_trace_disarm(void);
bool bb_trace_armed(void);

/** Drop everything held and reset the counters and the time origin. */
void bb_trace_clear(void);

/**
 * @brief Record one event, if armed.
 *
 * Cheap enough to call unconditionally from a transfer path: one load and a
 * branch when disarmed, which is what it is for every capture that is not a
 * self trace.
 */
void bb_trace_push(bb_ev_kind_t kind, uint8_t value, bool flag, bool derived);

/**
 * @brief Record one event with a timestamp taken earlier.
 *
 * For the case where WHEN something happened and WHETHER it worked become
 * known at different moments. A frame's START is one: the controller knows the
 * instant it asked for it, but not for another nine bit times whether the
 * address was acknowledged, and an event cannot be pushed before its flag is
 * settled without lying about the flag.
 *
 * Stamping at push time instead cost 28 microseconds of lag with 19 of spread,
 * measured against the wire by trace_lag.py - which is 19 bit times at 1 MHz
 * and 240 at 12.5. Carrying the earlier timestamp costs one word on the stack
 * and gets both halves right.
 */
void bb_trace_push_at(bb_ev_kind_t kind, uint8_t value, bool flag,
                      bool derived, uint32_t t_cyc);

/** Take up to @p max events out of the ring, oldest first. */
unsigned bb_trace_drain(bb_trace_ev_t *out, unsigned max);

/**
 * @brief Copy up to @p max events WITHOUT consuming them, skipping @p skip.
 *
 * `skip` is not a convenience. A caller walking the ring in batches has to say
 * where its window starts, and the first version of this had no way to - so
 * SNIFF:TRACE? asked for `done + take` entries into a 32-entry array and wrote
 * past the end of it the moment `done` reached 32. That smashed the stack at
 * exactly 42 events and took the board down with it, which looks like a USB
 * fault from the host and is nothing of the kind.
 */
unsigned bb_trace_peek(bb_trace_ev_t *out, unsigned max, unsigned skip);

unsigned bb_trace_pending(void);
uint32_t bb_trace_total(void);   /**< events pushed since the last clear */
uint32_t bb_trace_lost(void);    /**< events dropped because the ring was full */
uint32_t bb_trace_t0(void);      /**< bb_cycles() at the last clear or arm */

#endif /* BB_TRACE_H */
