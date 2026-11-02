/**
 * @file bb_sniff.c
 * @brief Passive I2C / I3C bus decoder on PG0 (SCL) and PG1 (SDA).
 *
 * Why GPIO capture and not a peripheral: neither the I2C nor the I3C block on
 * this part can sit on a bus as a silent observer.  Both are transaction
 * engines that must be addressed to report anything, and neither exposes the
 * raw bit stream.  So the sniffer watches two plain inputs with edge
 * interrupts and reconstructs the protocol in software.
 *
 * The decode is deliberately done in the handler rather than by sampling into
 * a buffer for the host to post-process.  A sampler needs four to eight times
 * the bit rate in memory bandwidth and gives you a few tens of milliseconds of
 * capture; decoding on the fly costs about forty cycles per bit and runs
 * indefinitely.  The trade is that it cannot see analogue faults - a slow rise
 * time or a marginal level looks like a clean bit here.  That is a scope's job
 * and this will never replace one.
 *
 * Timing budget: at 250 MHz there are 2500 core cycles per bit at 100 kHz and
 * 625 at 400 kHz, against roughly 40 cycles of handler.  1 MHz (125 cycles) is
 * the practical ceiling and is where dropped bits start to be plausible;
 * SNIFF:STAT? reports the worst handler latency it has seen so you can tell
 * rather than guess.
 *
 * I3C: the framing is decoded in I3C mode, where the ninth bit of a data byte
 * is a T-bit rather than an acknowledge, and 0x7E is recognised as the
 * broadcast address.  That works for the open-drain phases - ENTDAA, broadcast
 * CCCs, legacy I2C traffic - but NOT for 12.5 MHz push-pull SDR, which is
 * 80 ns per bit and out of reach of any interrupt-per-edge scheme on this
 * silicon.  See docs/SNIFFER.md for the arithmetic.
 */
#include "bb_sniff.h"
#include "bb_board.h"
#include "bb_status.h"
#include "bb_cmd.h"
#include "bb_i3c.h"
#include <stdio.h>
#include <string.h>

/**
 * Both handlers live in RAM.  Flash on this part runs with five wait states
 * behind a one-way instruction cache, and the handlers are entered from
 * whatever the main loop was doing, so they miss that cache routinely - the
 * measured cost per bus clock was 614 ns with the code in flash, which is
 * about 154 core cycles for maybe thirty instructions' worth of work.  Copying
 * them to SRAM removes the wait states from the one path in this firmware that
 * has a hard deadline.  SNIFF:BENCH measures the difference.
 */
#define SNIFF_FAST  __attribute__((section(".RamFunc"), noinline))

#define SNIFF_RING       4096u          /* events; 2 bytes each */
#define SNIFF_TEST_HALF_NS 20000u       /* default self-test half bit period */
#define SNIFF_TEST_MIN_NS    100u

/* Event kinds, as reported to the host. */
#define EV_START           1u
#define EV_RESTART         2u
#define EV_STOP            3u
#define EV_ADDR            4u
#define EV_DATA            5u

typedef struct {
    uint8_t kind;                       /**< EV_*, with 0x80 set when acked */
    uint8_t data;
} event_t;

static volatile event_t  ring[SNIFF_RING];
static volatile uint16_t head;          /* written by the ISRs */
static volatile uint16_t tail;          /* read by the command layer */
static volatile uint32_t dropped;

/* ---- raw edge capture --------------------------------------------------- */
/* A second, separate mode: the handlers record edges instead of decoding them.
 * See the note at the top of the file about why both exist.
 *
 * Sized to fill about a tenth of the smallest board's RAM. 8192 edges is
 * roughly 2 KB of I2C traffic at 400 kHz, or two milliseconds of a saturated
 * bus - enough to hold a transaction and its surroundings, which is what a
 * waveform view is for. Longer than that is a streaming problem and USB full
 * speed cannot carry it. */
#define SNIFF_RAW_MAX    8192u

typedef struct {
    uint32_t t;                         /**< TIM2 count, wraps every ~17 s   */
    uint8_t  lines;                     /**< bit0 SCL, bit1 SDA, after the edge */
} raw_edge_t;

static volatile raw_edge_t raw_ring[SNIFF_RAW_MAX];
static volatile uint32_t   raw_count;   /* edges recorded this capture */
static volatile uint32_t   raw_read;    /* how many the host has taken   */
static volatile bool       raw_mode;    /* handlers record instead of decoding */
static volatile bool       raw_full;    /* the buffer filled and capture stopped */
static volatile uint32_t   raw_limit = SNIFF_RAW_MAX;
static volatile uint32_t   raw_first_t;
static volatile uint32_t   raw_last_t;
/* The line state of the last record, so a handler can tell a transition from a
 * second handler reporting the same one. */
static volatile uint8_t    raw_last_lines;
/* Interrupts that arrived after both lines had already settled, so there was
 * nothing left to record. Counted rather than hidden: it is the measure of how
 * often two edges land inside one interrupt latency. */
static volatile uint32_t   raw_coincident;

/* Bit-level decode state, all touched only from the two handlers. */
static volatile bool     running;
static volatile bool     i3c_mode;
static volatile bool     strict_mode;   /* watch SDA even where it cannot matter */
static volatile bool     in_frame;      /* between START and STOP */

static volatile bool     first_byte;    /* next completed byte is the address */
static volatile uint8_t  shift;
static volatile uint8_t  nbits;

/* Statistics.  Deliberately no per-interrupt timing: reading TIM2 costs two
 * peripheral bus accesses, which on this part is a large fraction of the
 * handler itself, so instrumenting the hot path measured mostly the
 * instrumentation.  SNIFF:BENCH times the work from outside instead, where the
 * observer costs nothing. */
static volatile uint32_t n_starts;
/* Raw SCL rising edges seen inside a frame. Ground truth for "did the wire
 * carry the number of clocks the decoder assumed", which no amount of decoded
 * output can answer: a byte and a bit-shifted byte look equally plausible. */
static volatile uint32_t n_clocks;
/* Every SCL rise the handler sees while running, whether or not a frame was
 * open. n_edges - n_clocks is therefore the number of clocks that arrived with
 * nothing to attach them to, and the pair together are the only way to tell a
 * lost interrupt from a bit that was read at the wrong moment - decoded output
 * cannot distinguish them, because a shifted byte is still a plausible byte. */
static volatile uint32_t n_edges;
static volatile uint32_t n_stops;
static volatile uint32_t n_bytes;

/* Pin masks as plain constants: the handlers must not pay for a HAL call.
 * The cycle counter is read straight out of TIM2 for the same reason - a call
 * to bb_cycles() is a branch into another translation unit, and with no
 * link-time optimisation that is a flash fetch in the middle of the hot path. */
#define SCL_MASK   BB_SNIFF_SCL_PIN
#define SDA_MASK   BB_SNIFF_SDA_PIN
#define SNIFF_IDR  (BB_SNIFF_PORT->IDR)
#define NOW()      (TIM2->CNT)

/** Events waiting to be read.  The subtraction must wrap, not go negative. */
static inline unsigned queued(void)
{
    return (unsigned)((head + SNIFF_RING - tail) % SNIFF_RING);
}

static inline __attribute__((always_inline)) void push(uint8_t kind, uint8_t data)
{
    uint16_t next = (uint16_t)((head + 1u) % SNIFF_RING);

    if (next == tail) {
        dropped++;
        return;
    }
    ring[head].kind = kind;
    ring[head].data = data;
    head = next;
}

/* ---- interrupt handlers ------------------------------------------------ */

/**
 * @brief SCL edge: shift one bit in, or close the byte on the ninth.
 *
 * Only the rising edge is enabled, because that is where a two-wire bus
 * guarantees SDA is stable.
 */
/**
 * @brief Record one edge. Raw mode only.
 *
 * Everything here is a store: no decode state, no branch on protocol, nothing
 * that can be wrong about what the bus meant. The one decision is what to do
 * when the buffer is full, and it stops - a ring that overwrote its oldest
 * edges would silently change where the capture began, and the specification's
 * first rule is that loss is never silent.
 */
static inline __attribute__((always_inline)) void raw_push(uint32_t idr)
{
    uint32_t n = raw_count;
    uint8_t  lines = (uint8_t)(((idr & SCL_MASK) ? 1u : 0u)
                             | ((idr & SDA_MASK) ? 2u : 0u));

    /* Nothing changed since the last record, so this is not a transition.
     *
     * Two handlers watch one port. When both lines move within an interrupt
     * latency of each other, both read the IDR after both edges and see the
     * same thing - the first record is correct and complete, and the second
     * would be a phantom edge. Measured at twelve in ninety-two on the self
     * test, which is not rare.
     *
     * The cost is that a pulse shorter than the latency, which would appear as
     * two identical records, is invisible. It already was: a single post-edge
     * sample cannot express a line that went low and came back between reads. */
    if (n > 0u && lines == raw_last_lines) {
        raw_coincident++;
        return;
    }

    if (n >= raw_limit) {
        raw_full = true;
        running  = false;               /* stop, rather than overwrite */
        return;
    }
    uint32_t now = NOW();

    raw_ring[n].t     = now;
    raw_ring[n].lines = lines;
    raw_last_lines    = lines;
    if (n == 0u) {
        raw_first_t = now;
    }
    raw_last_t = now;
    raw_count  = n + 1u;
}

/**
 * @brief Record where the lines are, before anything moves them.
 *
 * The capture format's first entry is the initial state, not a transition, and
 * this is where it comes from. Without it a host cannot know the level before
 * the first edge: a record saying SDA is now low leaves open whether it fell or
 * whether SCL moved, and a decoder guessing at that reads the first release of
 * SDA as a STOP that never happened.
 */
static void raw_seed_initial_state(void)
{
    uint32_t idr = SNIFF_IDR;

    raw_ring[0].t     = NOW();
    raw_ring[0].lines = (uint8_t)(((idr & SCL_MASK) ? 1u : 0u)
                                | ((idr & SDA_MASK) ? 2u : 0u));
    raw_last_lines = raw_ring[0].lines;
    raw_first_t    = raw_ring[0].t;
    raw_last_t     = raw_ring[0].t;
    raw_count      = 1u;
}

SNIFF_FAST void bb_sniff_scl_irq(void)
{
    uint32_t idr = SNIFF_IDR;

    EXTI->RPR1 = SCL_MASK;
    if (raw_mode) {
        EXTI->FPR1 = SCL_MASK;
        if (running) {
            raw_push(idr);
        }
        return;
    }

    if (!running) {
        return;
    }

    bool sda = (idr & SDA_MASK) != 0u;

    n_edges++;
    if (in_frame) {
        n_clocks++;
    }

    if (!in_frame) {
        /* A clock with no frame open is not decodable and not worth guessing
         * at. An earlier version tried to recover a START from the SDA pending
         * flag here, on the theory that the first bit of an I3C frame was
         * being lost to interrupt ordering. Measurement disproved it - `edges`
         * and `clocks` agree exactly, so no edge is ever missed - and the
         * recovery invented STARTs from stale pending flags, which shifted the
         * self test's whole pattern by a bit. The one-bit offset seen on native
         * I3C traffic is a sampling-window problem; see docs/SNIFFER.md. */
        return;
    }

    if (nbits < 8u) {
        /* Inside a byte, no START or STOP can legally appear, so SDA is
         * masked for the rest of it.  SDA moves for every data bit while SCL
         * is low, each move costs an interrupt entry, and the handler's only
         * possible response is to notice SCL is low and return; masking those
         * out removes interrupts without removing information.  STRICT keeps
         * them, for hunting framing violations rather than reading traffic.
         *
         * Masking from the SECOND bit, not the first, and the reason is worth
         * spelling out: a STOP is framed by SCL going high before SDA is
         * released, and a repeated START by SCL going high before SDA is
         * pulled down.  Both therefore arrive as an SCL rising edge that is
         * indistinguishable from a first data bit.  Masking on that edge blinds
         * the sniffer to every STOP on the bus - which is exactly what the
         * loopback self test reported before this was moved a bit later. */
        if (nbits == 1u && !strict_mode) {
            EXTI->IMR1 &= ~(uint32_t)SDA_MASK;
        }
        shift = (uint8_t)((shift << 1) | (sda ? 1u : 0u));
        nbits++;
    } else {
        /* Ninth bit.  On I2C it is the target's acknowledge, active low.  On
         * I3C it is the T-bit for data and still an acknowledge for the
         * address, so the sense is only meaningful to the host, which knows
         * the mode; both are reported in the same flag. */
        uint8_t kind = first_byte ? EV_ADDR : EV_DATA;
        bool    acked = i3c_mode && !first_byte ? sda : !sda;

        push((uint8_t)(kind | (acked ? 0x80u : 0u)), shift);
        n_bytes++;
        first_byte = false;
        shift      = 0;
        nbits      = 0;

        if (!strict_mode) {
            /* Discard the setup edges that latched while the line was masked,
             * or the first thing seen on unmasking is a phantom START. */
            EXTI->RPR1  = SDA_MASK;
            EXTI->FPR1  = SDA_MASK;
            EXTI->IMR1 |= SDA_MASK;
        }
    }
}

/**
 * @brief SDA edge: a transition while SCL is high is a START or a STOP.
 *
 * Any SDA edge while SCL is low is ordinary data setup and is ignored, which
 * is most of them - hence the early exit before anything else is touched.
 */
SNIFF_FAST void bb_sniff_sda_irq(void)
{
    uint32_t idr = SNIFF_IDR;
    uint32_t rp  = EXTI->RPR1 & SDA_MASK;

    EXTI->RPR1 = SDA_MASK;
    EXTI->FPR1 = SDA_MASK;

    if (raw_mode) {
        if (running) {
            raw_push(idr);
        }
        return;
    }

    if (!running || (idr & SCL_MASK) == 0u) {
        return;
    }

    if (rp != 0u) {
        /* SDA released while SCL high: STOP. */
        if (in_frame) {
            push(EV_STOP, 0);
            n_stops++;
        }
        in_frame = false;
    } else {
        /* SDA pulled low while SCL high: START, or a repeated START if we
         * never saw the STOP that would have closed the previous frame. */
        push(in_frame ? EV_RESTART : EV_START, 0);
        n_starts++;
        in_frame   = true;
        first_byte = true;
        shift      = 0;
        nbits      = 0;
    }
}

/* ---- setup ------------------------------------------------------------- */

/** Put both taps back to high-impedance inputs with edge interrupts armed. */
static void pins_listen(void)
{
    GPIO_InitTypeDef g = {0};

    /* No internal pull-up: the bus under observation supplies its own, and a
     * 40k pull-up from this board would be one more thing loading a bus we
     * are supposed to be watching passively. */
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    /* Raw capture wants both edges of the clock as well: a waveform is the
     * high and the low, and a decoder reading edges off it needs to know how
     * long SCL was held. Decode mode takes only the rising edge, where a
     * two-wire bus guarantees SDA is stable, and pays one interrupt per bit
     * instead of two. */
    g.Pin  = SCL_MASK;
    g.Mode = raw_mode ? GPIO_MODE_IT_RISING_FALLING : GPIO_MODE_IT_RISING;
    HAL_GPIO_Init(BB_SNIFF_PORT, &g);

    g.Pin  = SDA_MASK;
    g.Mode = GPIO_MODE_IT_RISING_FALLING;
    HAL_GPIO_Init(BB_SNIFF_PORT, &g);
}

void bb_sniff_init(void)
{
    pins_listen();

    /* Above every other interrupt in the system: a late bit is a lost bit,
     * and nothing else here has a deadline measured in hundreds of cycles. */
    /* Both at the same preemption level, and the subpriority argument is
     * decoration: the HAL sets NVIC_PRIORITYGROUP_4, which is four bits of
     * preemption and none of subpriority. When both lines are pending at once
     * - which is the normal case for an I3C START and its first clock - the
     * NVIC picks the lower exception number, and that is the SCL tap.
     *
     * Raising SDA's preemption would not be an improvement: these two handlers
     * push into one event ring, so letting SDA interrupt SCL mid-byte would
     * reorder the bus, trading a wrong bit for a wrong sequence. The START
     * race is handled in bb_sniff_scl_irq() instead, where it can be resolved
     * without depending on which handler runs first. */
    HAL_NVIC_SetPriority(BB_SNIFF_SCL_IRQn, 0, 0);
    HAL_NVIC_SetPriority(BB_SNIFF_SDA_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(BB_SNIFF_SCL_IRQn);
    HAL_NVIC_EnableIRQ(BB_SNIFF_SDA_IRQn);
}

static void reset_state(void)
{
    /* Leave raw mode, if a raw capture left us in it.
     *
     * Here rather than in each caller, because that is exactly the mistake this
     * fixes: SNIFF:START cleared it and SNIFF:TEST did not, so a self test run
     * after a raw capture recorded edges instead of decoding and reported that
     * it had seen no traffic at all. Every decode path goes through this
     * function, so this is the one place it cannot be forgotten.
     *
     * pins_listen() is what puts the clock tap back to one interrupt per bit;
     * raw mode arms both edges. */
    if (raw_mode) {
        raw_mode = false;
        pins_listen();
    }

    /* Whatever state a capture left the mask in, listening starts with both
     * lines armed. */
    EXTI->RPR1  = SCL_MASK | SDA_MASK;
    EXTI->FPR1  = SCL_MASK | SDA_MASK;
    EXTI->IMR1 |= (uint32_t)(SCL_MASK | SDA_MASK);

    running    = false;
    head       = 0;
    tail       = 0;
    dropped    = 0;
    in_frame   = false;
    first_byte = false;
    shift      = 0;
    nbits      = 0;
    n_starts   = 0;
    n_clocks   = 0;
    n_edges    = 0;
    n_stops    = 0;
    n_bytes    = 0;
}

/* ---- self test --------------------------------------------------------- */

/** Busy-wait in nanoseconds off the TIM2 cycle counter. */
static void delay_ns(uint32_t ns)
{
    uint32_t ticks = (bb_cycle_hz() / 1000000u) * ns / 1000u;
    uint32_t t0    = NOW();

    while ((NOW() - t0) < ticks) { }
}

/**
 * @brief Drive a synthetic transaction on the sniff pins and decode it.
 *
 * The taps are switched to outputs while their EXTI lines stay armed -
 * HAL_GPIO_Init only touches the EXTI registers for interrupt modes, so
 * reconfiguring the pin direction leaves the trigger configuration intact.
 * The handlers then see edges exactly as they would from an external bus,
 * which exercises the whole path: pins, EXTI, decode, ring buffer, encoding.
 *
 * The drive is push-pull, not open-drain, on purpose.  Open-drain would leave
 * the rise time to the 40 kOhm internal pull-up - about a microsecond - and a
 * failure at a short bit period would then mean "the pull-up was too weak"
 * rather than "the decoder missed a bit", which is the opposite of what this
 * is for.  Push-pull is safe because these two pins are unconnected by
 * definition while the test runs, and that is also why the test must not be
 * run against a live bus.
 */
static void self_test(const uint8_t *bytes, unsigned count, bool addr_ack,
                      uint32_t half_ns)
{
    GPIO_InitTypeDef g = {0};

    g.Pin   = SCL_MASK | SDA_MASK;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(BB_SNIFF_PORT, &g);

    BB_SNIFF_PORT->BSRR = SCL_MASK | SDA_MASK;      /* both released, idle */
    delay_ns(half_ns * 2u);

    /* START: SDA low while SCL is high. */
    BB_SNIFF_PORT->BSRR = (uint32_t)SDA_MASK << 16;
    delay_ns(half_ns);

    for (unsigned i = 0; i < count; i++) {
        uint8_t byte = bytes[i];

        for (int bit = 7; bit >= 0; bit--) {
            BB_SNIFF_PORT->BSRR = (uint32_t)SCL_MASK << 16;      /* SCL low */
            if ((byte >> bit) & 1u) {
                BB_SNIFF_PORT->BSRR = SDA_MASK;
            } else {
                BB_SNIFF_PORT->BSRR = (uint32_t)SDA_MASK << 16;
            }
            delay_ns(half_ns);
            BB_SNIFF_PORT->BSRR = SCL_MASK;                      /* SCL high */
            delay_ns(half_ns);
        }

        /* Ninth bit: low is an acknowledge. */
        BB_SNIFF_PORT->BSRR = (uint32_t)SCL_MASK << 16;
        if (addr_ack) {
            BB_SNIFF_PORT->BSRR = (uint32_t)SDA_MASK << 16;
        } else {
            BB_SNIFF_PORT->BSRR = SDA_MASK;
        }
        delay_ns(half_ns);
        BB_SNIFF_PORT->BSRR = SCL_MASK;
        delay_ns(half_ns);
    }

    /* STOP: SDA released while SCL is high. */
    BB_SNIFF_PORT->BSRR = (uint32_t)SCL_MASK << 16;
    BB_SNIFF_PORT->BSRR = (uint32_t)SDA_MASK << 16;
    delay_ns(half_ns);
    BB_SNIFF_PORT->BSRR = SCL_MASK;
    delay_ns(half_ns);
    BB_SNIFF_PORT->BSRR = SDA_MASK;
    delay_ns(half_ns * 2u);

    pins_listen();
}

/* ---- commands ---------------------------------------------------------- */

/**
 * @brief SNIFF:RAW:START [max-edges] - record edges instead of decoding them.
 *
 * Leaves decode mode. The two are exclusive by construction: the handlers
 * check raw_mode first and return, so a capture cannot be half of each.
 */
static void cmd_raw_start(int argc, char **argv)
{
    uint32_t want = SNIFF_RAW_MAX;

    if (argc > 1 && (!bb_parse_u32_max(argv[1], SNIFF_RAW_MAX, &want)
                     || want == 0u)) {
        bb_reply_errf(BB_ERR_RANGE, "max-edges-1..%u", (unsigned)SNIFF_RAW_MAX);
        return;
    }

    running    = false;
    raw_mode   = true;
    raw_count  = 0;
    raw_read   = 0;
    raw_full   = false;
    raw_limit  = want;
    raw_coincident = 0;
    raw_last_lines = 0xFF;
    raw_first_t = 0;
    raw_last_t  = 0;

    /* Both edges of the clock, which decode mode does not need. */
    pins_listen();
    /* Cleared after the pins are configured, not before: reconfiguring a pin
     * sets its pending flag, and an edge from before the capture was armed is
     * not part of the capture. */
    EXTI->RPR1  = SCL_MASK | SDA_MASK;
    EXTI->FPR1  = SCL_MASK | SDA_MASK;
    raw_seed_initial_state();
    EXTI->IMR1 |= (uint32_t)(SCL_MASK | SDA_MASK);
    running     = true;

    bb_reply_okf("raw=1 initial=1 max=%lu tick_hz=%lu %s",
                 (unsigned long)raw_limit, (unsigned long)bb_cycle_hz(),
                 BB_SNIFF_PINS);
}

static void cmd_raw_stop(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    running = false;
    bb_reply_okf("edges=%lu full=%u", (unsigned long)raw_count,
                 raw_full ? 1u : 0u);
}

/**
 * @brief SNIFF:RAW:STAT? - what was captured, and whether it is all of it.
 *
 * `full` is the important field. A capture that filled the buffer ends where
 * the buffer ended, not where the bus went quiet, and a host that cannot tell
 * those apart will draw a gap as silence. It becomes a gap record in the
 * capture file.
 */
static void cmd_raw_stat(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    /* `initial=1` says the first record is the line state at arming rather
     * than an edge, which is what lets a host know the level before the first
     * transition. `coincident` is how many interrupts arrived with nothing left
     * to record - see raw_push(). */
    bb_reply_okf("raw=%u running=%u initial=%u edges=%lu read=%lu max=%lu "
                 "full=%u coincident=%lu first=%lu last=%lu tick_hz=%lu",
                 raw_mode ? 1u : 0u, running ? 1u : 0u,
                 raw_count > 0u ? 1u : 0u,
                 (unsigned long)raw_count, (unsigned long)raw_read,
                 (unsigned long)raw_limit, raw_full ? 1u : 0u,
                 (unsigned long)raw_coincident,
                 (unsigned long)raw_first_t, (unsigned long)raw_last_t,
                 (unsigned long)bb_cycle_hz());
}

/**
 * @brief SNIFF:RAW:READ? [max] - drain edges as five bytes each.
 *
 * Little-endian 32-bit timestamp then the line state, which is the layout the
 * host reads straight into its sample chunk. Absolute timestamps rather than
 * deltas: a delta stream cannot be resumed from the middle, and this reply is
 * one of many - a host draining eight thousand edges issues a hundred and
 * sixty commands and must be able to lose one without losing the timebase.
 */
static void cmd_raw_read(int argc, char **argv)
{
    uint8_t  out[BB_PAYLOAD_MAX];
    uint32_t want = (uint32_t)(sizeof out / 5u);
    size_t   n    = 0;

    if (argc > 1 && !bb_parse_u32_max(argv[1], sizeof out / 5u, &want)) {
        bb_reply_errf(BB_ERR_RANGE, "max-1..%u", (unsigned)(sizeof out / 5u));
        return;
    }

    while (n + 5u <= sizeof out && want > 0u && raw_read < raw_count) {
        uint32_t t     = raw_ring[raw_read].t;
        uint8_t  lines = raw_ring[raw_read].lines;

        out[n++] = (uint8_t)(t & 0xFFu);
        out[n++] = (uint8_t)((t >> 8) & 0xFFu);
        out[n++] = (uint8_t)((t >> 16) & 0xFFu);
        out[n++] = (uint8_t)((t >> 24) & 0xFFu);
        out[n++] = lines;
        raw_read++;
        want--;
    }

    if (n == 0u) {
        bb_reply_okf("empty remaining=0 full=%u", raw_full ? 1u : 0u);
        return;
    }
    bb_reply_ok_bytes(out, n);
}

/**
 * @brief SNIFF:RAW:TEST [half-ns] - drive a known pattern and capture it raw.
 *
 * The same generator the decoding self test uses, with the handlers recording
 * edges instead of decoding them. So the host can exercise the whole raw path -
 * pins, both EXTI edges, timestamps, the ring, the transport - against traffic
 * whose content it already knows, with nothing wired to the taps.
 *
 * Push-pull drive on two pins that are unconnected by definition while this
 * runs, which is why it must not be run against a live bus.
 */
static void cmd_raw_test(int argc, char **argv)
{
    static const uint8_t pattern[] = { 0xA0u, 0x5Au, 0x00u, 0xFFu };
    uint32_t half_ns = SNIFF_TEST_HALF_NS;

    if (argc > 1 && (!bb_parse_u32(argv[1], &half_ns) ||
                     half_ns < SNIFF_TEST_MIN_NS || half_ns > 1000000u)) {
        bb_reply_errf(BB_ERR_RANGE, "half-ns-%u..1000000",
                      (unsigned)SNIFF_TEST_MIN_NS);
        return;
    }

    running     = false;
    raw_mode    = true;
    raw_count   = 0;
    raw_read    = 0;
    raw_full    = false;
    raw_limit   = SNIFF_RAW_MAX;
    raw_coincident = 0;
    raw_last_lines = 0xFF;
    raw_first_t = 0;
    raw_last_t  = 0;

    /* Arm both edges of both taps first. The trigger configuration survives the
     * change of direction below - HAL_GPIO_Init only touches the EXTI registers
     * for interrupt modes - which is the same property self_test() relies on.
     * Leaving this out armed only the clock's rising edge, the mode decode uses,
     * and the capture came back with a third of its edges. */
    pins_listen();

    /* Drive both taps to idle before seeding the initial state.
     *
     * With nothing wired, an input tap floats - there is no internal pull-up,
     * deliberately, so that watching a bus does not load it. Seeding from a
     * floating pin recorded whatever it happened to read, and when the self
     * test then drove SDA high the decoder saw SDA rise while SCL was high and
     * reported a STOP that never happened. The self test knows the line state
     * because it drives it, so it says so. */
    {
        GPIO_InitTypeDef g = {0};

        g.Pin   = SCL_MASK | SDA_MASK;
        g.Mode  = GPIO_MODE_OUTPUT_PP;
        g.Pull  = GPIO_NOPULL;
        g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        HAL_GPIO_Init(BB_SNIFF_PORT, &g);
        BB_SNIFF_PORT->BSRR = SCL_MASK | SDA_MASK;      /* both idle high */
        delay_ns(half_ns);
    }

    EXTI->RPR1  = SCL_MASK | SDA_MASK;
    EXTI->FPR1  = SCL_MASK | SDA_MASK;
    raw_seed_initial_state();
    EXTI->IMR1 |= (uint32_t)(SCL_MASK | SDA_MASK);
    running     = true;

    uint32_t t_start = NOW();
    self_test(pattern, (unsigned)(sizeof pattern), true, half_ns);
    uint32_t elapsed = NOW() - t_start;

    running = false;

    /* The expected edge count is worth stating rather than leaving the host to
     * derive: one START (SDA fall), 45 clock pulses for five acked bytes at two
     * edges each, the SDA moves between them, and one STOP. An exact figure
     * would have to model when SDA is masked, so the host checks the pattern
     * decodes rather than that a number matched. */
    bb_reply_okf("edges=%lu full=%u coincident=%lu bytes=%u half_ns=%lu "
                 "elapsed_us=%lu tick_hz=%lu",
                 (unsigned long)raw_count, raw_full ? 1u : 0u,
                 (unsigned long)raw_coincident,
                 (unsigned)(sizeof pattern) + 1u,
                 (unsigned long)half_ns,
                 (unsigned long)(elapsed / (bb_cycle_hz() / 1000000u)),
                 (unsigned long)bb_cycle_hz());
}

static void cmd_raw_clear(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    running   = false;
    raw_mode  = false;
    raw_count = 0;
    raw_read  = 0;
    raw_full  = false;
    /* Back to one interrupt per bit rather than two. */
    pins_listen();
    bb_reply_ok();
}

static void cmd_start(int argc, char **argv)
{
    bool want_i3c    = false;
    bool want_strict = false;

    for (int i = 1; i < argc; i++) {
        if (bb_streq_ci(argv[i], "I3C")) {
            want_i3c = true;
        } else if (bb_streq_ci(argv[i], "STRICT")) {
            want_strict = true;
        } else if (!bb_streq_ci(argv[i], "I2C")) {
            bb_reply_err(BB_ERR_RANGE, "args-i2c-i3c-strict");
            return;
        }
    }
    reset_state();
    i3c_mode    = want_i3c;
    strict_mode = want_strict;
    running     = true;
    bb_reply_okf("mode=%s strict=%d scl=PG0/D65 sda=PG1/D64",
                 want_i3c ? "i3c" : "i2c", want_strict ? 1 : 0);
}

static void cmd_stop(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    running = false;
    bb_reply_okf("events=%u starts=%lu bytes=%lu dropped=%lu",
                 queued(),
                 (unsigned long)n_starts, (unsigned long)n_bytes,
                 (unsigned long)dropped);
}

static void cmd_clear(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    bool was = running;
    reset_state();
    running = was;
    bb_reply_ok();
}

/**
 * @brief Drain decoded events as a token stream.
 *
 * One token per event so a host can parse it with a split():
 *
 *     S        start            P        stop
 *     Sr       repeated start   !        events were dropped
 *     A50+     address, acked   D2A-     data byte, not acked
 *
 * In I3C mode the flag on a data byte carries the T-bit rather than an
 * acknowledge; SNIFF:STAT? reports which mode produced the capture.
 */
static void cmd_read_q(int argc, char **argv)
{
    char     out[900];
    size_t   used = 0;
    uint32_t want = 128;
    unsigned n    = 0;

    if (argc > 1 && (!bb_parse_u32(argv[1], &want) || want == 0u || want > 256u)) {
        bb_reply_err(BB_ERR_RANGE, "count-1..256");
        return;
    }

    if (dropped > 0u) {
        used += (size_t)snprintf(out + used, sizeof out - used, "! ");
        dropped = 0;
    }

    while (n < want && tail != head) {
        event_t e;

        e.kind = ring[tail].kind;
        e.data = ring[tail].data;
        tail   = (uint16_t)((tail + 1u) % SNIFF_RING);
        n++;

        /* Leave room for the longest token plus its separator. */
        if (used + 8u >= sizeof out) {
            break;
        }
        bool    acked = (e.kind & 0x80u) != 0u;
        uint8_t kind  = (uint8_t)(e.kind & 0x7Fu);

        switch (kind) {
        case EV_START:
            used += (size_t)snprintf(out + used, sizeof out - used, "S ");
            break;
        case EV_RESTART:
            used += (size_t)snprintf(out + used, sizeof out - used, "Sr ");
            break;
        case EV_STOP:
            used += (size_t)snprintf(out + used, sizeof out - used, "P ");
            break;
        case EV_ADDR:
        case EV_DATA:
            used += (size_t)snprintf(out + used, sizeof out - used, "%c%02X%c ",
                                     (kind == EV_ADDR) ? 'A' : 'D',
                                     (unsigned)e.data, acked ? '+' : '-');
            break;
        default:
            break;
        }
    }

    if (used > 0u && out[used - 1u] == ' ') {
        out[used - 1u] = '\0';
    } else {
        out[used] = '\0';
    }
    bb_reply_okf("n=%u more=%d %s", n, (tail != head) ? 1 : 0, out);
}

static void cmd_stat_q(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    bb_reply_okf("run=%d mode=%s strict=%d queued=%u starts=%lu stops=%lu "
                 "bytes=%lu clocks=%lu edges=%lu dropped=%lu",
                 running ? 1 : 0, i3c_mode ? "i3c" : "i2c", strict_mode ? 1 : 0,
                 queued(),
                 (unsigned long)n_starts, (unsigned long)n_stops,
                 (unsigned long)n_bytes, (unsigned long)n_clocks,
                 (unsigned long)n_edges, (unsigned long)dropped);
}

static void cmd_pins_q(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    bb_reply_okf("scl=PG0/D65 sda=PG1/D64 scl_level=%d sda_level=%d",
                 (SNIFF_IDR & SCL_MASK) ? 1 : 0,
                 (SNIFF_IDR & SDA_MASK) ? 1 : 0);
}

/**
 * @brief End-to-end self test with nothing wired up.
 *
 * Drives a transaction on the sniff pins and checks that the decoder recovered
 * exactly it.  This is the difference between "the code compiles" and "the
 * capture path works on this board": it exercises the GPIO, both EXTI lines,
 * both handlers, the ring buffer and the encoder.
 */
static void cmd_test(int argc, char **argv)
{
    static const uint8_t pattern[] = { 0xA0u, 0x5Au, 0x00u, 0xFFu };
    bool     saved_mode = i3c_mode;
    unsigned addrs = 0;
    unsigned datas = 0;
    unsigned starts = 0;
    unsigned stops = 0;
    uint8_t  got[8];
    unsigned ngot = 0;
    uint32_t half_ns = SNIFF_TEST_HALF_NS;

    if (argc > 1 && (!bb_parse_u32(argv[1], &half_ns) ||
                     half_ns < SNIFF_TEST_MIN_NS || half_ns > 1000000u)) {
        bb_reply_errf(BB_ERR_RANGE, "half-ns-%u..1000000", (unsigned)SNIFF_TEST_MIN_NS);
        return;
    }

    reset_state();
    i3c_mode = false;
    running  = true;

    /* Time the whole thing.  The generator and the handlers share one core, so
     * a requested bit period shorter than the handler cost simply stretches -
     * the test throttles itself and can never outrun the decoder.  That makes
     * pass/fail at a requested rate almost meaningless on its own; the elapsed
     * time is the measurement that means something, because it yields the real
     * cost per edge. */
    uint32_t t_start = NOW();
    self_test(pattern, (unsigned)(sizeof pattern), true, half_ns);
    uint32_t elapsed = NOW() - t_start;

    running = false;

    /* Walk the queue without consuming it.  The self test is the one capture
     * whose contents are known in advance, which makes it the most useful
     * thing a host can read back through the ordinary event API - so it would
     * be perverse for the check to eat it. */
    uint16_t cursor = tail;
    while (cursor != head) {
        uint8_t kind = (uint8_t)(ring[cursor].kind & 0x7Fu);
        uint8_t data = ring[cursor].data;
        cursor = (uint16_t)((cursor + 1u) % SNIFF_RING);

        switch (kind) {
        case EV_START:   starts++; break;
        case EV_STOP:    stops++;  break;
        case EV_ADDR:
            addrs++;
            if (ngot < sizeof got) { got[ngot++] = data; }
            break;
        case EV_DATA:
            datas++;
            if (ngot < sizeof got) { got[ngot++] = data; }
            break;
        default: break;
        }
    }
    i3c_mode = saved_mode;

    bool ok = starts == 1u && stops == 1u && addrs == 1u &&
              datas == (unsigned)(sizeof pattern) - 1u &&
              ngot == (unsigned)(sizeof pattern) &&
              memcmp(got, pattern, sizeof pattern) == 0;

    if (!ok) {
        bb_reply_errf(BB_ERR_BUS_STATE,
                      "self-test-failed%s at=%luHz starts=%u stops=%u addr=%u data=%u "
                      "got=%02X%02X%02X%02X",
                      /* The commonest cause of a failure here that is not the
                       * decoder's fault: the taps are jumpered to this board's
                       * own I3C pins and the controller is still holding them.
                       * Say so rather than leaving it to be rediscovered. */
                      bb_i3c_configured() ? "-i3c-is-driving-the-taps" : "",
                      (unsigned long)(500000000u / half_ns),
                      starts, stops, addrs, datas,
                      (ngot > 0u) ? got[0] : 0u, (ngot > 1u) ? got[1] : 0u,
                      (ngot > 2u) ? got[2] : 0u, (ngot > 3u) ? got[3] : 0u);
        return;
    }
    /* 4 bytes x 9 bits is 36 clocks, each one an SCL rising edge. */
    uint32_t mhz      = bb_cycle_hz() / 1000000u;
    uint32_t ns_total = elapsed / mhz * 1000u;
    uint32_t eff_hz   = (ns_total > 0u) ? (uint32_t)(36000000000ULL / ns_total) : 0u;

    bb_reply_okf("self-test-passed bytes=%u pattern=A05A00FF asked=%luHz "
                 "elapsed_us=%lu eff_hz=%lu",
                 ngot, (unsigned long)(500000000u / half_ns),
                 (unsigned long)(ns_total / 1000u), (unsigned long)eff_hz);
}

/**
 * @brief Measure the decode cost, and from it the fastest bus we can follow.
 *
 * The loopback test cannot answer "how fast a bus can this follow?" because the
 * generator and the handlers share one core: ask for a shorter bit period and
 * the generator just gets throttled by the interrupts it is causing, so the
 * test passes at any requested rate and tells you nothing.
 *
 * This runs the same waveform twice - once with both EXTI lines masked, once
 * with them armed - and takes the difference.  That isolates the interrupt and
 * decode cost from the cost of generating the edges, and on a real bus the
 * generating is somebody else's problem, so the difference per bit is the
 * number that bounds the bus rate.
 */
static void cmd_bench(int argc, char **argv)
{
    static const uint8_t pattern[] = { 0xA0u, 0x5Au, 0x00u, 0xFFu };
    const uint32_t clocks  = 36u;               /* 4 bytes x 9 bits */
    uint32_t       half_ns = 2000u;

    if (argc > 1 && (!bb_parse_u32(argv[1], &half_ns) ||
                     half_ns < SNIFF_TEST_MIN_NS || half_ns > 1000000u)) {
        bb_reply_errf(BB_ERR_RANGE, "half-ns-%u..1000000", (unsigned)SNIFF_TEST_MIN_NS);
        return;
    }

    uint32_t saved_imr = EXTI->IMR1;
    bool     saved_run = running;
    bool     saved_i3c = i3c_mode;
    bool     saved_str = strict_mode;

    strict_mode = (argc > 2) && bb_streq_ci(argv[2], "STRICT");

    /* Warm the paths first: the first pass through either arm pays flash and
     * cache costs that have nothing to do with steady-state decoding. */
    reset_state();
    running = true;
    self_test(pattern, (unsigned)(sizeof pattern), true, half_ns);

    /* Bare generator, no interrupts at all. */
    EXTI->IMR1 = saved_imr & ~(uint32_t)(SCL_MASK | SDA_MASK);
    uint32_t t0 = NOW();
    self_test(pattern, (unsigned)(sizeof pattern), true, half_ns);
    uint32_t bare = NOW() - t0;

    /* Same waveform, decoding live. */
    EXTI->IMR1 = saved_imr;
    reset_state();
    i3c_mode = false;
    running  = true;
    t0 = NOW();
    self_test(pattern, (unsigned)(sizeof pattern), true, half_ns);
    uint32_t decoded = NOW() - t0;

    bool used_strict = strict_mode;
    running     = saved_run;
    i3c_mode    = saved_i3c;
    strict_mode = saved_str;

    uint32_t mhz     = bb_cycle_hz() / 1000000u;
    uint32_t cost_ns = (decoded > bare) ? ((decoded - bare) * 1000u) / (mhz * clocks) : 0u;
    uint32_t max_hz  = (cost_ns > 0u) ? (uint32_t)(1000000000ULL / cost_ns) : 0u;

    bb_reply_okf("strict=%d bare_us=%lu decoded_us=%lu clocks=%lu cost_ns=%lu "
                 "max_bus_hz=%lu",
                 used_strict ? 1 : 0,
                 (unsigned long)(bare / mhz), (unsigned long)(decoded / mhz),
                 (unsigned long)clocks, (unsigned long)cost_ns,
                 (unsigned long)max_hz);
}

void bb_sniff_status_line(bb_status_t *out)
{
    if (out == NULL) {
        return;
    }
    snprintf(out->pins, sizeof out->pins, "%s", running ? "capturing" : "stopped");
    if (running) {
        snprintf(out->detail, sizeof out->detail, "%lu B %u q",
                 (unsigned long)n_bytes, queued());
    } else {
        snprintf(out->detail, sizeof out->detail, "-");
    }
    /* Dropped events are the one thing here worth flagging on a small screen:
     * capture data is gone and no later reading will bring it back. */
    out->state = (dropped > 0u) ? BB_STATE_PROBLEM
                                : (running ? BB_STATE_ACTIVE : BB_STATE_IDLE);
}

const bb_cmd_t bb_cmd_sniff[] = {
    { "SNIFF:BENCH", 0, 2, cmd_bench,  "SNIFF:BENCH [half-period-ns] [STRICT]" },
    { "SNIFF:START", 0, 2, cmd_start,  "SNIFF:START [I2C|I3C] [STRICT]" },
    { "SNIFF:RAW:START", 0, 1, cmd_raw_start, "SNIFF:RAW:START [max-edges]" },
    { "SNIFF:RAW:STOP",  0, 0, cmd_raw_stop,  "SNIFF:RAW:STOP" },
    { "SNIFF:RAW:STAT?", 0, 0, cmd_raw_stat,  "SNIFF:RAW:STAT?" },
    { "SNIFF:RAW:READ?", 0, 1, cmd_raw_read,  "SNIFF:RAW:READ? [max]" },
    { "SNIFF:RAW:CLEAR", 0, 0, cmd_raw_clear, "SNIFF:RAW:CLEAR" },
    { "SNIFF:RAW:TEST",  0, 1, cmd_raw_test,  "SNIFF:RAW:TEST [half-period-ns]" },
    { "SNIFF:STOP",  0, 0, cmd_stop,   "SNIFF:STOP" },
    { "SNIFF:READ?", 0, 1, cmd_read_q, "SNIFF:READ? [max-events]" },
    { "SNIFF:STAT?", 0, 0, cmd_stat_q, "SNIFF:STAT?" },
    { "SNIFF:PINS?", 0, 0, cmd_pins_q, "SNIFF:PINS?" },
    { "SNIFF:CLEAR", 0, 0, cmd_clear,  "SNIFF:CLEAR" },
    { "SNIFF:TEST",  0, 1, cmd_test,   "SNIFF:TEST [half-period-ns]" },
    { NULL, 0, 0, NULL, NULL }
};
