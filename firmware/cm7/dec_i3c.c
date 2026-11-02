/**
 * @file dec_i3c.c
 * @brief Native I3C SDR decoder. See dec_i3c.h for why it is not the I2C one.
 */
#include "dec_i3c.h"

#include <string.h>

#define I3C_BROADCAST      0x7Eu   /* the address every I3C device answers */
#define I3C_CCC_ENTDAA     0x07u
#define I3C_CCC_IS_DIRECT  0x80u   /* bit 7 set = directed, clear = broadcast */

/* ENTDAA hands back 8 bytes per device before the controller assigns one. */
#define I3C_DAA_PAYLOAD    8u

typedef enum {
    ST_IDLE = 0,    /* between transactions */
    ST_ADDR,        /* the byte after START/Sr - ninth bit is an ACK */
    ST_CCC,         /* the byte after a broadcast-write address */
    ST_DATA,        /* payload - ninth bit is a T-bit */
    ST_DAA          /* inside ENTDAA: PID/BCR/DCR then the assigned address */
} dec_state_t;

static struct {
    dec_i3c_emit_fn emit;
    dec_state_t     state;
    bool            in_frame;
    uint8_t         bit_count;   /* 0..8 - eight data bits then the ninth */
    uint8_t         byte;
    bool            read;        /* direction of the current transfer */
    bool            entdaa;      /* an ENTDAA CCC has been seen this frame */
    uint8_t         daa_left;    /* bytes still expected in this DAA round */
    uint8_t         skip;        /* bits to discard before the next byte */
    /* At a byte boundary with nothing clocked since - where the controller may
     * turn the bus between open drain and push-pull, and where that turnaround
     * looks exactly like a repeated START to a two-wire detector. */
    bool            at_turnaround;
    /*
     * A START is not reported until the frame it opens proves itself.
     *
     * Releasing SDA at the end of a read moves the data line while the clock is
     * held high, and that is precisely the shape of a START - the hardware
     * detector is right to report it. At 12.5 MHz the handover is sharp enough
     * to be caught every time, and the capture grew a frame that was never on
     * the bus: `... D55+ P S A25+ P`, an address made of the previous frame's
     * stranded tail bits. At 8 MHz the same event shows up more mildly, as a
     * doubled `P`.
     *
     * The rule that separates them needs no timing: an I3C transaction opens
     * with an address and its acknowledge, so a START whose frame closes before
     * nine bits have been clocked did not open a transaction. Holding the event
     * back until those nine bits arrive lets a frame that never happens be
     * dropped whole, rather than reported and then contradicted.
     */
    uint16_t        bytes_done;   /* bytes completed in this frame */
    bool            start_held;
    bool            start_held_repeated;
} d;

void dec_i3c_init(dec_i3c_emit_fn emit)
{
    memset(&d, 0, sizeof d);
    d.emit = emit;
}

void dec_i3c_reset(void)
{
    d.start_held = false;

    dec_i3c_emit_fn keep = d.emit;

    memset(&d, 0, sizeof d);
    d.emit = keep;
}

/*
 * Whether the decoder still has a frame open.
 *
 * A caller cannot infer this from having called dec_i3c_stop(): a STOP in the
 * middle of a read is the bus turnaround rather than an end of frame, and it is
 * suppressed below with the frame left open. The capture side used to clear its
 * own `in_frame` on every STOP it applied, so after a turnaround the two
 * disagreed - and the read's *real* STOP then arrived at a capture layer that
 * believed nothing was open, and was dropped before ever reaching here. That
 * cost one STOP per read: thirteen on the wire, seven noted, six frames merged
 * into their neighbours. Ask, do not assume.
 */
bool dec_i3c_in_frame(void)
{
    return d.in_frame;
}

bool dec_i3c_at_boundary(void)
{
    return d.bit_count == 0u;
}

static void emit(bb_ev_kind_t kind, uint8_t value, bool flag)
{
    if (d.emit != NULL) {
        d.emit(kind, value, flag);
    }
}

void dec_i3c_start(bool repeated)
{
    /*
     * A restart cannot legitimately appear before the command code.
     *
     * After a broadcast-write address the very next thing on the bus is the
     * CCC - the protocol allows nothing else - so a repeated START reported
     * while the decoder is waiting for that byte is the open-drain to
     * push-pull turnaround, not framing. I3C releases SDA after the
     * acknowledge and drives the first command bit immediately, both with SCL
     * still high, which is indistinguishable from a START by looking at two
     * wires. The hardware detector reports it and is not wrong to.
     *
     * Left in, it resets the byte in progress and the command code is lost
     * entirely - pin-clocked captures showed `S AFC+ Sr ...` with no CCC at
     * all, and every byte after it shifted.
     *
     * The same turnaround happens after a *private* address, and that was not
     * covered. A private write of two bytes decoded as `S A12+ D00+ Sr P` - the
     * first payload byte, then a phantom restart where the second should be,
     * then nothing recognisable, at every rate including 1 MHz. It was recorded
     * as pin-clocked tail truncation, which it is not: no bits were lost, one
     * boundary was misread and the state machine went back to expecting an
     * address.
     *
     * The rule is a byte boundary with no payload bit after it. A restart that
     * genuinely belongs on the bus - before 0x7E+R in an ENTDAA, or before the
     * target address of a directed CCC - arrives after the command code, and a
     * read-after-write restart arrives after data bytes. Neither sits at the
     * boundary immediately following the byte that preceded it with nothing
     * clocked in between, which is what `at_turnaround` means.
     */
    if (repeated && d.at_turnaround && (d.bit_count == 0u) &&
        ((d.state == ST_CCC) || (d.state == ST_DATA))) {
        return;
    }

    d.start_held          = true;
    d.start_held_repeated = repeated;

    /*
     * A repeated START inside ENTDAA does not end the ENTDAA.
     *
     * The sequence is 0x7E+W, the ENTDAA CCC, then Sr, 0x7E+R, and only then do
     * the target's PID bytes arrive. Clearing the ENTDAA flag on the Sr - which
     * is what resetting all state here would do - loses the one piece of
     * context that tells those eight bytes apart from ordinary payload, and
     * they decode as a private read from the broadcast address.
     */
    d.state      = ST_ADDR;
    d.in_frame   = true;
    d.bytes_done = 0u;
    d.bit_count = 0u;
    d.byte      = 0u;
    d.skip      = 0u;
    if (!repeated) {
        d.entdaa   = false;
        d.daa_left = 0u;
    }
}

/*
 * Every frame close, with the state that decided it.
 *
 * Read frames come to nine bits at some rates where writes always come to
 * twenty-seven, and four hypotheses about the wire have each been measured and
 * found not to explain it. The wire is uniform - one edge between frames, both
 * directions, every rate - so what closes those frames is in here, and this
 * records it rather than reasoning about it.
 *
 * One word per close: which call path, whether it was suppressed and why, and
 * the decoder state at the moment it happened.
 */
#define DEC_CLOSE_LOG 8u

static uint32_t d_close[DEC_CLOSE_LOG];
static uint8_t  d_close_n;
static uint8_t  d_site;

void dec_i3c_site(uint8_t site)
{
    d_site = site;
}

static void close_note(uint8_t why)
{
    if (d_close_n < DEC_CLOSE_LOG) {
        d_close[d_close_n++] =
            ((uint32_t)(d_site & 3u) << 30) |
            ((uint32_t)(why & 3u) << 28) |
            ((uint32_t)(d.read ? 1u : 0u) << 27) |
            ((uint32_t)(d.in_frame ? 1u : 0u) << 26) |
            ((uint32_t)(d.state & 7u) << 23) |
            ((uint32_t)(d.bit_count & 15u) << 19) |
            ((uint32_t)(d.bytes_done & 0x7FFu) << 8) |
            ((uint32_t)(d.start_held ? 1u : 0u) << 7);
    }
}

unsigned dec_i3c_close_log(uint32_t *out, unsigned max)
{
    unsigned n = (d_close_n < max) ? d_close_n : max;

    for (unsigned i = 0; i < n; i++) {
        out[i] = d_close[i];
    }
    d_close_n = 0u;
    return n;
}

bool dec_i3c_was_read(void)
{
    return d.read;
}

void dec_i3c_stop(void)
{
    /*
     * The turnaround after a read's address is not a STOP.
     *
     * A private read hands the bus to the target once the address has been
     * acknowledged, and SDA moving while the clock is held is exactly the shape
     * of a STOP - so the hardware detector reports one, correctly, in the middle
     * of a transaction that has barely begun. A write turns the bus the other
     * way at the same point and produces a *restart* shape instead, which this
     * decoder already suppresses at a byte boundary; the STOP side was never
     * covered.
     *
     * Measured: with the decoder counting bits per frame against what the wire
     * carries, writes came to exactly 27 bits at every rate while reads came to
     * 9 or 18 instead of 63 - frames closed after one or two bytes. Nine bits is
     * an address and nothing else.
     *
     * A frame that ends with only its address decoded is not a transfer this
     * controller issues: a read carries RDTERM of at least one byte and a write
     * at least one payload byte. So a STOP there is the handover, and the frame
     * is dropped whole.
     *
     * Scoped to reads first, on the reasoning that only a read hands the bus
     * over. That was too narrow, and the frame-close log is what showed it: in
     * a read-only workload the spurious frames were arriving with `read=0`,
     * because their address was assembled from misaligned bits and its
     * direction bit came out whichever way the data fell. Nine bits of rubbish
     * can look like either direction. Dropping any address-only frame caught
     * them, and 4 MHz went from decoding no reads at all to decoding them.
     *
     * The cost is that a genuine address-only transaction - I3C:PROBE emits
     * exactly one - no longer appears in a pin-clocked capture. Oversampled
     * capture still shows it, and a probe is a bus operation nobody is usually
     * sniffing at 12 MHz.
     */
    if (d.in_frame && (d.bytes_done == 1u) && (d.bit_count == 0u)) {
        close_note(0u);
        return;
    }
    if (!d.in_frame) {
        /*
         * A STOP with nothing open is not a STOP.
         *
         * The same handover that produces a phantom START produces a second
         * STOP behind it: SDA rises while SCL is held when the target lets go,
         * and again when the controller ends the frame for real. Suppressing
         * only the phantom START left the trailing one to be reported on its
         * own - `... D55+ P P` - which is a transaction ending twice.
         *
         * The capture layer cannot filter it, because the phantom START had
         * already told it a frame was open. Here the decoder knows whether one
         * really is.
         */
        close_note(1u);
        return;
    }
    if (d.start_held) {
        /* The frame closed without completing an address: it was the handover
         * at the end of the last one, not a transaction. Drop both ends. */
        close_note(2u);
        d.start_held = false;
        d.state      = ST_IDLE;
        d.in_frame   = false;
        d.bit_count  = 0u;
        d.byte       = 0u;
        d.entdaa     = false;
        d.daa_left   = 0u;
        return;
    }
    close_note(3u);
    emit(BB_EV_STOP, 0u, false);
    d.bytes_done = 0u;
    d.state     = ST_IDLE;
    d.in_frame  = false;
    d.bit_count = 0u;
    d.byte      = 0u;
    d.entdaa    = false;
    d.daa_left  = 0u;
}

/** The ninth bit has arrived; decide what the byte was and what comes next. */
static void finish_byte(uint8_t ninth)
{
    uint8_t value = d.byte;

    /* The frame has produced a whole address byte, so the START that opened it
     * was real. Report it now, ahead of the byte it introduces. */
    if (d.start_held) {
        emit(d.start_held_repeated ? BB_EV_RESTART : BB_EV_START, 0u, false);
        d.start_held = false;
    }

    switch (d.state) {
    case ST_ADDR:
        /*
         * The ninth bit after an address IS an acknowledge, even on I3C - the
         * T-bit replaces it only for data. Reported inverted because the wire
         * is active low: a target pulls SDA down to accept.
         */
        d.read = (value & 1u) != 0u;
        emit(BB_EV_ADDR, value, ninth == 0u);

        if (((value >> 1) == I3C_BROADCAST) && !d.read) {
            /* Broadcast write: whatever follows is a command code. */
            d.state = ST_CCC;
        } else if (d.entdaa && d.read && ((value >> 1) == I3C_BROADCAST)) {
            /* 0x7E+R after an ENTDAA CCC - the target's turn to speak. */
            d.state    = ST_DAA;
            d.daa_left = I3C_DAA_PAYLOAD;
        } else {
            d.state = ST_DATA;
        }
        break;

    case ST_CCC:
        emit(BB_EV_CCC, value, ninth != 0u);
        if (value == I3C_CCC_ENTDAA) {
            /*
             * ENTDAA is followed by a restart and 0x7E+R, and the standard
             * allows nothing else there - so the decoder expects an address
             * next whether or not the restart was detected.
             *
             * It usually is not. Both capture paths miss that particular
             * restart: the controller turns the bus back to open drain for the
             * header, and the SDA movement does not present as a clean
             * SDA-while-SCL-high edge to either the software detector or the
             * hardware one. Waiting for framing that never arrives means the
             * header is read as CCC payload and every byte after it shifts.
             *
             * Expecting it from the protocol rather than from the wire is the
             * same reasoning that already says a CCC follows a broadcast write
             * address. It is a sequence the bus is not permitted to deviate
             * from.
             */
            d.entdaa = true;
            d.state  = ST_ADDR;
            /*
             * And discard one bit first.
             *
             * The restart occupies a bit position in the captured stream: the
             * capture clocks on SCL, and the turnaround into the header
             * contains a clock edge, so one sample arrives that is not a data
             * bit. Measured identically in both capture modes - the header read
             * as 0xFE instead of 0xFD, one bit early, whether the bits came
             * from a hardware-clocked shifter or an oversampled edge stream,
             * which is what says it is the bus and not the sampler.
             */
            d.skip = 1u;
            break;
        }
        /*
         * A directed CCC is followed by Sr and the target's address, so the
         * next byte is an address rather than payload - and the framing will
         * say so. A broadcast CCC carries its payload straight after.
         */
        d.state = ((value & I3C_CCC_IS_DIRECT) != 0u) ? ST_ADDR : ST_DATA;
        break;

    case ST_DAA:
        /* Unreachable: the DAA payload is framed in dec_i3c_bit(), because it
         * is not nine-bit framed at all. Kept so the switch stays total. */
        emit(BB_EV_DATA, value, ninth != 0u);
        break;

    case ST_DATA:
    default:
        /* T-bit, not an acknowledge - parity on a write, end-of-data on a
         * read. Passed through as the flag; the record's protocol says how to
         * read it. */
        emit(BB_EV_DATA, value, ninth != 0u);
        break;
    }

    d.bit_count = 0u;
    d.byte      = 0u;
    d.bytes_done++;
    /*
     * Set for every byte, not only the address.
     *
     * What it marks is "a byte boundary the bus may turn the drive mode at",
     * and the turnaround this guards against happens at the boundary after the
     * address. Keeping it general costs nothing and means a phantom restart
     * reported one byte later is caught too.
     */
    d.at_turnaround = true;
}

void dec_i3c_bit(uint8_t bit)
{
    d.at_turnaround = false;

    if (!d.in_frame) {
        /*
         * Bits with no START in front of them are a capture that began
         * mid-transaction. Counting them from an arbitrary point would produce
         * byte boundaries wrong by a rotation, which is exactly the plausible
         * wrong answer this decoder exists to avoid.
         */
        return;
    }

    /*
     * The ENTDAA payload is eight bits per byte. Everything else is nine.
     *
     * During dynamic address assignment the target drives its 48-bit
     * provisioned ID, BCR and DCR as 64 CONTIGUOUS bits - there is no T-bit
     * between them, because those bytes are arbitration data rather than a
     * transfer. Reading them nine bits at a time misframes every one, and the
     * damage compounds: each byte steals a bit from the next, so the values
     * come out plausible and entirely wrong.
     *
     * That is what the I2C-derived decoder did, and the evidence was in every
     * ENTDAA capture taken here: a target whose PID is 01 04 13 81 10 00 with
     * BCR 2E and DCR C6 decoded as 02 10 4E 08 00 05 B1 09. Eight bytes, right
     * count, right position, no gaps - nothing about it looks wrong. The
     * controller reported the PID correctly all along because it reads the
     * peripheral's registers and never goes near this path, so the two never
     * disagreed where anyone was looking.
     */
    if (d.state == ST_DAA) {
        d.byte = (uint8_t)((d.byte << 1) | (bit & 1u));
        d.bit_count++;
        if (d.bit_count >= 8u) {
            emit(BB_EV_DATA, d.byte, false);
            d.bit_count = 0u;
            d.byte      = 0u;
            if (d.daa_left > 0u) {
                d.daa_left--;
            }
            if (d.daa_left == 0u) {
                /* 64 bits done. The controller's address assignment follows,
                 * and that IS nine-bit framed. */
                d.state = ST_DATA;
            }
        }
        return;
    }

    if (d.skip > 0u) {
        d.skip--;
        return;
    }

    if (d.bit_count < 8u) {
        d.byte = (uint8_t)((d.byte << 1) | (bit & 1u));
        d.bit_count++;
        return;
    }

    finish_byte((uint8_t)(bit & 1u));
}
