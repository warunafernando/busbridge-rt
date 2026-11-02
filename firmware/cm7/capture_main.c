/**
 * @file capture_main.c
 * @brief The capture core. Runs on the CM7 at 800 MHz and does nothing else.
 *
 * This image exists so that capture cannot compete with the bus controller. On
 * the STM32 they shared one core, and that single fact produced every limit the
 * sniffer had: ~450 ns of interrupt and decode per bus clock, a hard ceiling
 * around 2.1 MHz, and a self test whose generator throttled itself against its
 * own capture interrupts so that asking for 5 MHz produced 202 kHz.
 *
 * Here the split is physical. The CM33 runs the controller and USB; this core
 * runs FlexIO and eDMA. Neither can starve the other.
 *
 * ## Why FlexIO rather than edge interrupts
 *
 * FlexIO's timer can be **clocked from a pin**. Point it at SCL and a shifter
 * latches SDA on the edge itself, in hardware, before software knows anything
 * happened. That is not a speed optimisation - it is a correctness one, and it
 * fixes a specific STM32 defect:
 *
 *   Native I3C open-drain stretches the low period and keeps SCL high for tens
 *   of nanoseconds. By the time an interrupt handler read the port, SCL had
 *   fallen and SDA had already moved to the next bit, so every byte decoded as
 *   `(true << 1) | next_bit`. The 0x7E broadcast header read as 0xF8 instead of
 *   0xFC. The handler was not missing interrupts - the edge counts came out
 *   exactly right - it was reading a moment too late, every time.
 *
 * A pin-clocked shifter cannot be late. There is no window.
 *
 * ## What this core does not do
 *
 * It does not decode. It produces timestamped samples in the capture format and
 * the host decodes them, which is what the Rust `sniffer` project was built
 * around: every decoded field has to point at a sample range so a saved capture
 * can be re-decoded, tested against a golden corpus with no hardware, and made
 * to explain itself. Decoding here would throw that away to save a host a few
 * microseconds.
 */
#include "bb_capture.h"
#include "dec_i3c.h"

#include "fsl_clock.h"
#include "fsl_flexio.h"
/* fsl_edma.h (the edma4 variant) is pulled in when the DMA path lands. Not
 * included yet: an unused dependency on a driver this image does not touch
 * would make the component list claim more than the image does. */

#include <string.h>

void BOARD_InitHardware(void);

/* ---- shared memory ----------------------------------------------------- */
/* Must match bb_link.c exactly. Two declarations of one layout is a real risk,
 * so the struct lives here in the same shape and the magic word is checked
 * before anything is believed. */
#define BB_SHM_BASE   0x20500000u
#define BB_SHM_MAGIC  0x42425254u

typedef struct __attribute__((packed)) {
    uint32_t     magic;
    uint32_t     cm7_alive;
    /* Heartbeat, bumped every main-loop pass. See bb_link.c: it is what
     * separates a capture core that never started from one that is
     * running and failing to handshake. */
    uint32_t     cm7_ticks;
    uint32_t     req_ready;
    uint32_t     rep_ready;
    bb_msg_hdr_t req_hdr;
    uint8_t      req[BB_MSG_MAX_PAYLOAD];
    bb_msg_hdr_t rep_hdr;
    uint8_t      rep[BB_MSG_MAX_PAYLOAD];
} bb_shm_t;

static volatile bb_shm_t *const shm = (volatile bb_shm_t *)BB_SHM_BASE;

/*
 * A beacon in this core's own ITCM, written before anything else is trusted.
 *
 * The mailbox is the thing most likely to be broken - it is shared, it is
 * cached differently on each side, and it is what every other diagnostic here
 * depends on. So "is this core executing at all" gets an answer that does not
 * use it: a magic and a counter at a fixed ITCM offset, which the controller
 * reads through the same 0x303C0000 window it copied the image through. If the
 * copy worked, the read works.
 *
 * 0x00010000 is 64 KB into a 255 KB ITCM, well past a 34 KB image and never
 * touched by it. Prepare_CM7() fills ITCM with whatever the DMA source held, so
 * the magic is what distinguishes a live beacon from leftover bytes.
 */
#define BB_BEACON_ADDR   0x00010000u
#define BB_BEACON_MAGIC  0x43374245u   /* "C7BE" */

typedef struct {
    uint32_t magic;
    uint32_t ticks;      /**< main-loop passes */
    uint32_t saw_magic;  /**< 1 once the mailbox magic was observed */
    /*
     * How far the last long operation got. Written from the self test so that a
     * core which stops mid-operation can still say where - the beacon is in this
     * core's own ITCM and the controller reads it through the copy window, so it
     * survives exactly the situation the mailbox does not.
     */
    uint32_t phase;
} bb_beacon_t;

static volatile bb_beacon_t *const beacon = (volatile bb_beacon_t *)BB_BEACON_ADDR;

/* ---- the capture ring -------------------------------------------------- */
/*
 * 24576 sample entries at 8 bytes each: 192 KB of the CM7's 256 KB DTCM.
 *
 * The first attempt asked for 32768 entries - a round 256 KB - and the linker
 * refused it by 1400 bytes, because the stack and heap live in the same region.
 * Worth recording rather than quietly fixing: "the whole of DTCM" is never the
 * available figure, and a ring sized to exactly the region size is a ring that
 * links on one toolchain and not the next.
 *
 * For scale, the STM32 managed 8192 edges - a couple of milliseconds of a busy
 * bus. This is three times as many entries, and each carries a timestamp delta
 * as well as the line state.
 *
 * It does **not** wrap. When it fills, capture stops and a gap record is
 * written. A ring that overwrote its oldest entries would silently move where
 * the capture began, and every timestamp after that point would be relative to
 * a moment nobody recorded. The STM32 made the same choice and reported
 * `full=1`; this reports it as a real gap record with a reason.
 */
#define CAP_RING_ENTRIES  24576u

static bb_sample_t s_ring[CAP_RING_ENTRIES];
static volatile uint32_t s_write;      /* entries produced */
/*
 * The write position, carried rather than recomputed.
 *
 * push_sample() indexed the ring with `s_write % CAP_RING_ENTRIES`, and
 * CAP_RING_ENTRIES is 24576 - not a power of two - so every sample cost a real
 * division, twice. That is invisible at oversampled rates, where the intake is
 * clamped to 10 MHz anyway, and it is the whole ceiling in pin-clocked mode,
 * where the bus clocks a sample per bit: at 12.5 MHz the shifters overran and
 * transactions decoded correctly for five bytes and then fell apart.
 *
 * An index that increments and wraps costs an add and a compare.
 */
static uint32_t s_wr_idx;
static volatile uint32_t s_read;       /* entries drained  */

#define CAP_SAMPLES_PER_WORD  32u

/*
 * Samples per shifter load in pin-clocked mode.
 *
 * A shifter delivers nothing until it fills, so the word size *is* the capture's
 * latency, measured in bus clocks. At 32 a six-byte I2C transfer - 54 bits - put
 * one word through and left the remaining 22 bits inside the shifter, so the
 * decode stopped three bytes in and the tail never arrived: there was no further
 * traffic to push it out, and on a bus that goes idle after a transaction there
 * never is.
 *
 * Thirty-one, not thirty-two, and the missing one is the whole bug.
 *
 * SHIFTBUF is loaded when the timer reaches its compare, and that happens before
 * the final counted shift has entered the shifter - so a word programmed for 32
 * shifts arrives holding 31 samples and an empty most-significant position, and
 * the 32nd sample is simply gone. It reads as a zero every time.
 *
 * That is exactly what made it so hard to see: the last bit is always 0, so a
 * word whose true last bit happened to *be* 0 looked perfect. Of four words in a
 * test transfer, two were wrong and two were right by coincidence, which reads as
 * an intermittent fault and is not one. Dumping the words against the known bit
 * stream side by side is what showed every word ending in a zero.
 *
 * Loading every 31 shifts puts the boundary where the data actually ends: the
 * word carries samples 0..30 at bit positions 1..31, and the next word starts at
 * sample 31 with nothing skipped.
 *
 * Shortening the word was tried first and made things worse - 8 bits corrupted
 * three bytes of five, 16 corrupted one - which is consistent once the cause is
 * known: a shorter word creates more reloads, and every reload was dropping a
 * sample. The size was never the problem; the off-by-one at the reload was.
 *
 * Widening it was tried later, for throughput rather than correctness: 16 and 32
 * bits per pin-clocked word remove the overruns at 12.5 MHz entirely, because
 * there are two and four times fewer per-word costs. Both decode wrongly, and
 * the reason is worth recording. A frame that ends part-way through a word
 * leaves its tail inside the shifter; the deferred STOP fires at the decoder's
 * next byte boundary, which with a short word arrives after the tail has been
 * delivered and with a long one arrives before. The tail is then attributed to
 * the following frame and every byte after it shifts by a group.
 *
 * Eight is the width where the tail is short enough to land first. Fixing the
 * wide case needs the frame's true bit length, which nothing on this part
 * reports - there is no live timer counter (see the register map).
 */
#define CAP_PINCLOCK_BITS_PER_WORD  8u

/* SNIFF:GAP: inter-frame clock edges to skip. Set from the config, so it can be
 * compared against the count measured from the raw sample stream. */
static uint8_t s_gap_bits = 2u;

/* SNIFF:DMA, as the host asked for it. Read by cap_start(); see the DMA intake
 * further down. Declared here because cap_configure() sets it. */
static bool s_dma_want;

static struct {
    bool          running;
    bool          full;
    bb_proto_t    proto;
    bb_observed_t source;
    uint8_t       flags;
    uint8_t       taps[8];
    uint8_t       tap_count;
    uint8_t       mode;         /**< bb_cap_mode_t */
    uint8_t       bits_per_word;/**< samples a shifter accumulates before it loads */
    uint8_t       decode_off; /**< capture only - see bb_cfg_t.decode_off */
    uint32_t      entries;
    uint32_t      records;
    uint32_t      gaps;
    uint32_t      coincident;
    uint64_t      ps_per_tick;
    uint32_t      last_state;
    uint64_t      base_ticks;   /**< absolute tick of the next entry to drain */
    bool          meta_pending;

    /* Resolved hardware, from the CONFIG message. */
    uint8_t       flexio;       /**< 1 or 2 */
    uint8_t       pins[8];      /**< FlexIO pin per tap, in tap order */
    uint8_t       base_pin;     /**< first pin of the sampled window */
    uint8_t       width;        /**< window width in pins: 1, 2, 4 or 8 */
    uint8_t       shifts;       /**< parallel loads per 32-bit shifter word */
    uint8_t       bit_of_tap[8];/**< window bit index for each tap */
    uint32_t      nominal_hz;
    uint32_t      sample_hz_req;
    uint32_t      sample_hz;    /**< achieved, after integer division */
    uint32_t      overruns;
    uint32_t      changed_mask;
    uint64_t      tick;         /**< samples taken since start */
    uint64_t      last_tick;    /**< tick of the last entry pushed */
    bool          have_initial;
} cap;

/* ---- timebase ---------------------------------------------------------- */
/*
 * Contract 1 says timestamps are picoseconds, not ticks - a tick count means
 * nothing without the rate it was counted at, and that rate belongs to one
 * device. So the tick rate is captured once at configure time and every record
 * carries `ps_per_tick`, letting the host convert without knowing anything about
 * this board.
 */
/*
 * One tick is one sample period, not one FlexIO clock. The distinction matters:
 * sampling divides the FlexIO clock, so a tick measured in FlexIO clocks would
 * make every timestamp wrong by the divider. The real figure is only known once
 * the divider has been programmed, so this sets a placeholder and
 * flexio_program() overwrites it with what the hardware actually achieved.
 */
static void timebase_set(uint32_t sample_hz)
{
    if (sample_hz == 0u) {
        sample_hz = 1000000u;
    }
    cap.sample_hz = sample_hz;
    /* Picoseconds per tick. Integer division loses sub-picosecond precision,
     * which is far below anything a digital capture resolves. */
    cap.ps_per_tick = 1000000000000ull / (uint64_t)sample_hz;
}

static void timebase_init(void)
{
    timebase_set(1000000u);
}

/* Forward declarations: cap_configure() below has to be able to stop a running
 * capture, and the intake and the stop live further down with the engine. */
static void cap_poll(void);
static void cap_hw_stop(void);

/* ---- capture control --------------------------------------------------- */

static bb_cap_err_t cap_configure(const bb_cfg_t *cfg)
{
    /*
     * Reconfiguring a running capture stops it first, for the same reason
     * cap_start() restarts one: refusing makes a legitimate request fail for a
     * state the caller cannot see.
     *
     * SNIFF:START pushes a configuration before it starts, so with the refusal
     * in place a second SNIFF:START - the host library's ordinary way to switch
     * protocol - reported "peripheral busy" and changed nothing. Stopping is
     * also the only correct thing to do: the new configuration may use different
     * pins and a different sample rate, so continuing to capture under the old
     * one would produce records the metadata no longer describes.
     */
    if (cap.running) {
        cap_poll();          /* keep what the shifters already hold */
        cap_hw_stop();
    }
    if (cfg->proto == (uint8_t)BB_PROTO_NONE) {
        return BB_CAP_ERR_PROTO;
    }
    if (cfg->tap_count > 8u) {
        return BB_CAP_ERR_RANGE;
    }

    cap.proto     = (bb_proto_t)cfg->proto;
    cap.source    = (bb_observed_t)cfg->source;
    cap.mode      = cfg->mode;
    cap.decode_off    = cfg->decode_off;
    cap.bits_per_word = (cfg->mode == (uint8_t)BB_CAP_PINCLOCK)
                            ? (uint8_t)CAP_PINCLOCK_BITS_PER_WORD
                            : (uint8_t)CAP_SAMPLES_PER_WORD;
    cap.flags     = cfg->flags;
    cap.tap_count = cfg->tap_count;
    memcpy(cap.taps, cfg->taps, sizeof cap.taps);
    memcpy(cap.pins, cfg->pins, sizeof cap.pins);
    cap.flexio        = cfg->flexio;
    cap.nominal_hz    = cfg->nominal_hz;
    cap.sample_hz_req = cfg->sample_hz;
    s_dma_want        = (cfg->dma_intake != 0u);
    s_gap_bits        = cfg->gap_bits;

    timebase_init();
    s_write = s_read = 0u;
    s_wr_idx = 0u;
    cap.entries = cap.records = cap.gaps = cap.coincident = 0u;
    cap.full = false;
    cap.meta_pending = true;
    return BB_CAP_OK;
}

/* ---- the FlexIO capture engine ----------------------------------------- */
/*
 * ## Uniform sampling, not a pin-clocked shifter
 *
 * The spec's table puts a pin-clocked timer against each protocol - SCL clocks
 * the timer, a shifter latches SDA on the edge itself - and that is the right
 * shape for *decoded* capture: it is bit-exact by construction and cannot be
 * late, which is what fixes the STM32's I3C off-by-one.
 *
 * It cannot produce this container's records, and that is why it is not what
 * runs here. A pin-clocked shifter has no time reference: it yields bit values
 * in bus-clock order with nothing to say when any of them happened. The capture
 * format's first contract is picosecond timestamps, and the spec says the three
 * container contracts are not being renegotiated. Emitting `dt_ticks` counted in
 * bus clocks would satisfy the struct and silently change what the field means
 * to every host that already reads it - the worst of the options.
 *
 * So the raw path samples all tap lines uniformly from an internal clock and
 * compresses runs to transitions in software. That keeps `dt_ticks` genuinely
 * temporal, serves every protocol with one configuration, and gives the host the
 * whole waveform - START and STOP conditions, glitches, timing violations -
 * rather than only the bits a protocol-specific decoder expected to find. It
 * also fixes the I3C defect, by a different route: there is no software read
 * chasing an edge, so there is no window to be late in.
 *
 * The pin-clocked configuration remains the right implementation for the
 * decoded-events mode, which needs bit values and not timestamps.
 * fsl_flexio_camera.c is the working template - a pin-clocked parallel receive
 * shifter, `kFLEXIO_TimerDecSrcOnPinInputShiftPinInput`.
 *
 * ## Polling, not interrupts
 *
 * The shifter status flag is polled from the main loop. This core runs nothing
 * else, so an interrupt would buy nothing and cost a vector, a priority and a
 * concurrency question against the mailbox. Polling also makes the failure
 * visible instead of silent: if sampling outruns the loop the shifter raises its
 * error flag, which becomes a real gap record rather than a quietly wrong
 * capture. eDMA is the next step and is what lifts the rate ceiling.
 */

#define CAP_TIMER        0u
#define CAP_SHIFTER      0u

/*
 * The framing timer and shifter, used in pin-clocked mode only.
 *
 * A shifter clocked by SCL cannot see START or STOP, because those are SDA
 * transitions while SCL is *high* and no clock edge exists at that moment. So
 * the framing comes from the other direction: a timer clocked by SDA's edges,
 * with its shifter sampling SCL. Every sample it takes is "what was the clock
 * doing when the data line moved", and that is precisely the question START and
 * STOP are defined by - SCL high means a framing event, SCL low means an
 * ordinary data setup between clocks.
 *
 * Which of the two it was comes from parity rather than a second shifter. SDA
 * edges alternate, and the bus rests high, so edge 1 is a fall, edge 2 a rise,
 * and so on: a fall with SCL high is a START, a rise with SCL high is a STOP.
 * That costs one timer instead of two (one clocked on each polarity) and is
 * exact as long as no edge is dropped - which the shifter's own overrun flag
 * reports rather than hides.
 */
#define CAP_FRAME_TIMER  1u
/*
 * How few bus clocks between a START flag and a STOP flag mean the START was
 * the STOP's setup. One is what the wire shows; the tick is read between words
 * so it arrives quantised to 0 or 8. The shortest real frame is an address
 * plus its acknowledge plus one byte and T - twenty-seven bits, which
 * quantises to 24 - so anything at or below sixteen is the setup and nothing
 * else can be.
 */


/* SDA edges per framing word. See the timer configuration for the trade. */
#define CAP_FRAME_EDGES_PER_WORD  1u

/*
 * Where the oldest sample sits in a full shifter word.
 *
 * A serial receive shifter takes each sample in at the top and shifts right, so
 * after 32 samples the *oldest* is in the low slot and the word reads oldest
 * first. Measured, not deduced: with this set the wrong way the self test
 * reported 206 transitions for 78 driven edges, and with it right it reports 77
 * - one per edge, less the initial state, which is exact.
 *
 * It was previously set the other way, and the comment here justified it by how
 * a *parallel* receive shifter loads whole groups at the low end. That was true
 * of the parallel design this engine started as and false of the per-tap serial
 * shifters that replaced it; the constant survived the redesign that invalidated
 * its reasoning. The symptom is worth naming because it is not a dead capture:
 * every 32-sample word comes out time-reversed, so the stream sawtooths
 * backwards at each word boundary and re-crosses every real edge two or three
 * times. The waveform still looks like a waveform. The decode is nonsense.
 */
#define CAP_OLDEST_IS_HIGH_SLOT  0

static FLEXIO_Type *cap_hw(void)
{
    return (cap.flexio == 2u) ? FLEXIO2 : FLEXIO1;
}

/*
 * FlexIO root clock: the maximum the part allows, from the source the board
 * already selected.
 *
 * IMXRT1180RM Rev 10 Table 135 gives flexio1/2_clk_root a maximum of 120 MHz in
 * RUN mode (60 MHz underdrive), with mux options OSC_RC_24M, OSC_RC_400M,
 * PLL_480_DIV2 (240 MHz) and PLL_1G_DIV5 (200 MHz). The SDK's board clock config
 * picks PLL_480_DIV2 with div=5, which is 48 MHz - well inside the limit and
 * nowhere near it.
 *
 * Nothing in this project had ever set this root; cap_src_hz() only read it, and
 * the 48 MHz it returned got quoted as though it were a property of the silicon.
 * It is a divider choice. Same mux, div=2, gives the full 120 MHz.
 *
 * That doubles-and-a-half the sampling ceiling: FlexIO's own timer divider has a
 * minimum of 2, so the top sample rate goes from 24 MHz to 60 MHz. Whether it
 * can be *used* is a separate question answered by the intake clamp above - the
 * CPU has to consume what the shifters produce, and that, not the clock, is what
 * binds today.
 */
#define CAP_ROOT_DIV   2u        /* 240 MHz PLL_480_DIV2 / 2 = 120 MHz, the max */

/* Put the FlexIO root at its documented maximum before anything reads it. */
static void cap_clock_set(void)
{
    clock_root_config_t rc;

    rc.clockOff = false;
    rc.div      = CAP_ROOT_DIV;
    rc.mux      = (cap.flexio == 2u)
                      ? (uint8_t)kCLOCK_FLEXIO2_ClockRoot_MuxSysPll3Div2
                      : (uint8_t)kCLOCK_FLEXIO1_ClockRoot_MuxSysPll3Div2;
    CLOCK_SetRootClock((cap.flexio == 2u) ? kCLOCK_Root_Flexio2
                                          : kCLOCK_Root_Flexio1, &rc);
}

static uint32_t cap_src_hz(void)
{
    clock_root_t root = (cap.flexio == 2u) ? kCLOCK_Root_Flexio2
                                           : kCLOCK_Root_Flexio1;
    uint32_t hz = CLOCK_GetRootClockFreq(root);

    return (hz == 0u) ? 100000000u : hz;
}

/*
 * One receive shifter per tap, and why not one parallel shifter for all of them.
 *
 * The first version used FlexIO's parallel receive: a single shifter with
 * PWIDTH set, sampling a window of consecutive pins. It configured cleanly and
 * only ever captured the pin at its pin select.
 *
 * That was measured, not guessed. Driving the low pin of a two-pin window gave
 * state bit 0; driving the high pin gave nothing at all - while a direct probe
 * through FlexIO's own PIN register showed *both* pads driving and reading
 * perfectly, and a register read-back confirmed the configuration had taken:
 *
 *     pins=11  window=14+2  state_lo=1  state_hi=0
 *     shiftcfg=0x00010000 pwidth=1  shiftctl=0x00000E01  timcmp=0x00001F03
 *
 * PWIDTH is 1, receive mode is set, pin select is 14, the divider gives the
 * 6 MHz it claimed. Everything correct, second pin still not sampled. So the
 * parallel path is not doing what is wanted here, and rather than keep pushing a
 * mode that will not explain itself, each tap now gets its own ordinary serial
 * receive shifter - the configuration every SPI, UART and I2S driver in the SDK
 * uses.
 *
 * The cost is a shifter per channel, so the tap count is bounded by the
 * peripheral's shifters rather than by a window width. The gain is that each
 * channel is independent: they share one timer, so they sample on the same edges
 * and stay aligned, and there is no assumption about how a group is packed.
 *
 * 32 samples per shifter word rather than 16 two-bit groups, so the intake also
 * halves in rate for the same sample clock.
 */

/*
 * Total samples per second the polled intake can consume across all channels.
 *
 * Measured by sweeping the requested sample rate with the clamp lifted, once the
 * FlexIO root was raised to its 120 MHz maximum so the divider grid was fine
 * enough to bracket the knee:
 *
 *   2 taps x 7.500 MHz = 15.0 M samples/s   overruns 0
 *   4 taps x 4.000 MHz = 16.0 M samples/s   overruns 0
 *   2 taps x 8.571 MHz = 17.1 M samples/s   overruns 6372
 *   2 taps x 60.0  MHz =  120 M samples/s   never finished - see below
 *
 * 15 M was the highest total verified clean *under those conditions*, and the
 * conditions turned out to matter more than the number. Repeating the
 * measurement against a real bus - a second board driving I2C, USB traffic
 * flowing, the console being polled - moved the limit down by a third:
 *
 *   400 kHz bus, 2 taps x 6.0 MHz = 12 M samples/s  ->  0/5 decodes, overruns
 *   400 kHz bus, 2 taps x 5.0 MHz = 10 M samples/s  ->  5/5 decodes, clean
 *
 * The failure was not random loss, which is what made it worth chasing: the
 * first transaction decoded perfectly every time and the second lost its address
 * every time. The self test could not have found this, because it drives its own
 * pattern with nothing else competing for the core - so the honest figure is the
 * one measured with the machine doing what it actually does, and it is 10 M.
 *
 * The load also depends on transition density, not sampling alone: 12 M samples/s
 * was clean at a 100 kHz bus and lossy at 400 kHz, because the faster bus pushes
 * more entries through the ring for the same sample rate. A single samples/s
 * budget is therefore a proxy rather than a model, and it is set where the proxy
 * holds for the busiest case measured.
 *
 * Which is why it came down again. 10 M held for 400 kHz and still dropped a bit
 * at a 1 MHz bus - `DBE` decoded as `DBF` - because a megahertz bus at five
 * samples per bit leaves no margin for a sample arriving late. Forcing 4 MHz
 * (8 M/s across two taps) decoded the same transfer perfectly and repeatably, so
 * 8 M is the figure. The cost is oversampling ratio at low bus rates, which is
 * the cheap direction to be wrong in: a 100 kHz bus is sampled 40x either way.
 *
 * The 60 MHz row is worth keeping. It did not report overruns and fail - it
 * ground on until the link timed out at 8 s, with the beacon stopped at phase
 * 0x11 inside the generator, because every settle iteration drained 64 word-sets
 * that refilled immediately. A sampler that outruns its intake badly enough
 * stops looking like data loss and starts looking like a hung core.
 *
 * This is the polled intake's number and nothing else's. The shifters will run
 * to 60 MHz per channel.
 *
 * ---- doubled, after the real cause was found ----
 *
 * Everything above was measured against a capture core compiled at -O0. Nothing
 * had ever set an optimization level for this image: the build scoped -Og to the
 * bridge profile because that one has to fit in 127 KB, and left this core alone
 * on the grounds that it has 255 KB and does not need the space. Size was the
 * wrong question - this loop runs once per sample and is the only thing that
 * sets the sampling ceiling.
 *
 * At -O2 the same measurements, unchanged, give:
 *
 *   5 MHz/tap   6/6 exact at 100 kHz, 400 kHz and 1 MHz   (was DBE -> D8E)
 *   6 MHz/tap   6/6 exact at all three                    (was 0/6, garbage)
 *   6 MHz I3C, pin-clocked: 0 overruns, 951 entries       (was 8-9, ~687)
 *
 * So 16 M, held to the same standard as the old 8 M: the busiest case measured
 * decodes exactly, repeatably. The earlier note reasoned that optimizing this
 * core "would invalidate the number rather than improve it" - it does invalidate
 * it, which is what a budget measured from this code is supposed to do when the
 * code gets faster.
 *
 * Above 6 MHz/tap the divider stops being the binding constraint at these bus
 * rates, so 16 M is where the evidence runs out rather than where the core does.
 *
 * ---- re-measured at 15 MHz/tap, and put back. Do not repeat this. ----
 *
 * The clamp was lifted and swept again, because the note at the clamp said eDMA
 * was what would raise it and eDMA had since arrived. Against SNIFF:TEST 500 -
 * a 1 MHz toggle - the knee looked far higher than 20 M, and looked like a
 * per-tap limit rather than a total:
 *
 *   taps  per tap   total      overruns          (SNIFF:TEST 500)
 *   2     15 MHz     30 M         0
 *   2     20 MHz     40 M       605
 *   4     15 MHz     60 M         0
 *   4     20 MHz     80 M      5519
 *
 * Identical with the polled intake and the eDMA one. Four taps carrying 60 M
 * with no overruns, against a budget holding them to 20 M, is not a small
 * discrepancy, and a per-tap reading explained it: at -O2 the per-sample unpack
 * is no longer the bottleneck, so a cost model built on sample_hz * tap_count
 * no longer describes anything.
 *
 * It was wrong anyway. Set to 15 MHz/tap and run against the bus rather than
 * the generator, a 2 MHz I3C capture went from exact at 10 MHz sampling to
 * wrong at 15 - with overruns at both - and 1 MHz picked up overruns where it
 * had none. More samples per bit and a worse decode is the signature of an
 * intake that cannot keep up, measured at the one load that counts.
 *
 * The toggle generator drives one pin. A real I3C bus moves two, with the
 * controller driving them from this same chip and USB running alongside, so it
 * is comfortably more than twice the work per sample. The generator says what
 * the sampler can do; it does not say what the system can do while sampling.
 *
 * The note above had already written this down - "15 M was the highest total
 * verified clean *under those conditions*, and the conditions turned out to
 * matter more than the number" - and this sweep walked into it regardless, for
 * the second time in this file's history. 20 M stays. A next attempt needs a
 * load at least as heavy as a 2 MHz bus with the controller active, and must
 * check the decode rather than the overrun counter, which read zero throughout
 * the wrong measurement.
 *
 * Worth separating from the question that prompted it: raising this would not
 * have made fast buses oversamplable either way. Even at 15 MHz/tap a 6 MHz I3C
 * bus gets 2.5 samples per bit and a 12.5 MHz bus 1.2, against the four or more
 * a transition needs. Pin-clocked mode is what exists for those.
 */
#define CAP_INTAKE_SAMPLES_PER_S  20000000u


/* The shifters this capture is using: 0 .. tap_count-1. */
/*
 * How many shifters this capture uses.
 *
 * In pin-clocked mode tap 0 is not sampled - it *is* the clock, and it drives
 * the timer rather than a shifter. So a two-tap I2C capture has one shifter
 * carrying SDA, and `state` bit i is tap i+1. Oversampled mode samples every
 * tap, and `state` bit i is tap i.
 *
 * That asymmetry is the whole reason this is a function rather than a field: it
 * is consulted by the poll loop, the status mask and the consume loop, and
 * getting it wrong in one of the three produces a capture that is subtly
 * shifted by a channel rather than one that fails.
 */
static unsigned cap_shifter_count(void)
{
    if (cap.mode == (uint8_t)BB_CAP_PINCLOCK) {
        return (cap.tap_count > 1u) ? (unsigned)(cap.tap_count - 1u) : 0u;
    }
    return cap.tap_count;
}

/* Whether this configuration has a framing channel at all. */
static bool cap_has_frame(void)
{
    return (cap.mode == (uint8_t)BB_CAP_PINCLOCK) && (cap.tap_count >= 2u);
}

/* The data shifters only - the framing shifter is polled separately because it
 * is clocked by a different line and fills at an unrelated rate. */
static uint32_t cap_shifter_mask(void)
{
    unsigned n = cap_shifter_count();

    return (n >= 32u) ? 0xFFFFFFFFu : ((1uL << n) - 1u);
}

/* Every shifter this capture owns, data and framing, for reset and error
 * clearing where the distinction does not apply. */
static uint32_t cap_all_shifter_mask(void)
{
    unsigned n = cap_shifter_count();   /* framing uses timers, not shifters */

    return (n >= 32u) ? 0xFFFFFFFFu : ((1uL << n) - 1u);
}

static bb_cap_err_t cap_assign(void)
{
    if (cap.tap_count == 0u || cap.tap_count > 8u) {
        return BB_CAP_ERR_PINS;
    }
    /* Shifter i samples tap i, so `state` bit i is tap i in SNIFF:CH order with
     * no mapping table at all - which is what the host's channel numbering has
     * to agree with. */
    cap.shifts   = (uint8_t)CAP_SAMPLES_PER_WORD;
    cap.base_pin = cap.pins[0];
    cap.width    = cap.tap_count;
    return BB_CAP_OK;
}

/*
 * Program every capture shifter, and reset it in the process.
 *
 * Separate from flexio_program() because it is needed twice. Writing SHIFTCTL is
 * what actually clears a shifter's *internal* shift register - disabling and
 * re-enabling the FlexIO module does not, and neither does draining SHIFTBUF.
 * That distinction cost real time: a pin-clocked capture came back bit-perfect
 * but offset by two, because two setup edges were still sitting inside the
 * shifter and the first word arrived as two stale bits followed by thirty real
 * ones. Restarting the module left them exactly where they were.
 */
static void cap_program_shifters(FLEXIO_Type *base)
{
    flexio_shifter_config_t sc;

    /* Every tap shifter is driven by the one timer, which is what keeps the
     * channels aligned: they shift on the same edges and their status flags
     * assert together, so a poll that reads all of them reads one instant of the
     * bus across every channel. */
    for (unsigned i = 0, n = cap_shifter_count(); i < n; i++) {
        /* Pin-clocked mode spends tap 0 on the timer, so shifter i carries tap
         * i+1. See cap_shifter_count(). */
        unsigned tap = (cap.mode == (uint8_t)BB_CAP_PINCLOCK) ? (i + 1u) : i;

        memset(&sc, 0, sizeof sc);
        sc.timerSelect   = CAP_TIMER;
        /*
         * Pin-clocked shifters take the *negative* edge of the timer's shift
         * clock. That clock is SCL inverted (see the timer's pinPolarity), so a
         * negative edge on it is SCL rising - which is where I2C and I3C define
         * the data as valid, and half a bit period before the reload.
         */
        sc.timerPolarity = (cap.mode == (uint8_t)BB_CAP_PINCLOCK)
                               ? kFLEXIO_ShifterTimerPolarityOnNegitive
                               : kFLEXIO_ShifterTimerPolarityOnPositive;
        sc.pinConfig     = kFLEXIO_PinConfigOutputDisabled;   /* input only */
        sc.pinSelect     = cap.pins[tap];
        sc.pinPolarity   = kFLEXIO_PinActiveHigh;
        sc.shifterMode   = kFLEXIO_ShifterModeReceive;
        sc.parallelWidth = 0u;                 /* serial: one pin, one bit */
        sc.inputSource   = kFLEXIO_ShifterInputFromPin;
        sc.shifterStop   = kFLEXIO_ShifterStopBitDisable;
        sc.shifterStart  = kFLEXIO_ShifterStartBitDisabledLoadDataOnEnable;
        FLEXIO_SetShifterConfig(base, (uint8_t)i, &sc);
    }

}

static bb_cap_err_t flexio_program(void)
{
    FLEXIO_Type            *base = cap_hw();
    flexio_timer_config_t   tc;
    bb_cap_err_t            rc = cap_assign();

    if (rc != BB_CAP_OK) {
        return rc;
    }

    /*
     * Sample rate. Requested explicitly, or 16x the nominal bus rate - enough
     * to place an edge to about a sixteenth of a bit, which is what a decoder
     * needs to distinguish a setup violation from a slow rise.
     *
     * The divider is the low byte of TIMCMP, so it is 8 bits and even: the
     * shift clock is the timer output, which toggles twice per period. That caps
     * the division at 512 and the achieved rate is reported rather than the
     * requested one, because integer division rarely lands on a round number.
     */
    cap_clock_set();

    uint32_t src  = cap_src_hz();
    uint32_t want = cap.sample_hz_req;

    if (want == 0u) {
        want = (cap.nominal_hz != 0u) ? cap.nominal_hz * 16u : 8000000u;
    }
    /*
     * Clamp to what the intake can actually consume, which depends on how many
     * channels there are.
     *
     * The cost per sample is per channel: cap_consume() unpacks one bit per tap
     * per sample, so the load is sample_hz * tap_count, and the ceiling belongs
     * on that product rather than on the rate alone. Measured on this part, with
     * the toggle generator and a four-tap capture:
     *
     *   4 taps @ 6 MHz  = 24 M samples/s  ->  7974 overruns, ~4% of words lost
     *   4 taps @ 4 MHz  = 16 M samples/s  ->  0 overruns over 1.4 M samples
     *   2 taps @ 6 MHz  = 12 M samples/s  ->  0 overruns
     *
     * Those set the budget at 16 M, which was safe and not the edge: the
     * boundary was never approached from below with two taps, and two taps is
     * what an I2C or I3C capture actually uses. Probed later, with real bus
     * traffic rather than the toggle generator:
     *
     *   2 taps @ 10 MHz = 20 M samples/s  ->  0 overruns, decode exact
     *   2 taps @ 12 MHz = 24 M samples/s  ->  overruns, decode degrades
     *
     * So 20 M. It matters because the clamp is what sets the sample rate: at
     * 16 M a two-tap capture was held to 7.5 MHz, which is under four samples
     * per bit on a 2 MHz bus and the reason oversampled capture ran out there.
     *
     * Without this the engine chose a rate it could not sustain and then failed
     * its own self test on `overruns != 0`, reporting "flexio-config-rejected" -
     * which names the wrong subsystem entirely. The FlexIO configuration was
     * correct and every channel captured perfectly; the core simply could not
     * read the shifters fast enough. A configured rate that cannot be sustained
     * is not a configuration error, it is a rate that should not have been
     * chosen.
     *
     * This is the intake's ceiling, not the hardware's: the shifters will run to
     * 60 MHz per channel.
     *
     * It used to say that moving the intake to eDMA is what raises it. eDMA
     * arrived, and it does not: swept with the clamp lifted, the knee sits in
     * the same place on the polled path and the eDMA one, so whatever binds
     * here is not the transfer. An attempt to raise the number on the strength
     * of toggle-generator measurements made real captures worse and was
     * reverted - see the constant's own comment before trying again.
     */
    if (cap.tap_count > 0u) {
        uint32_t ceiling = CAP_INTAKE_SAMPLES_PER_S / cap.tap_count;

        if (want > ceiling) {
            want = ceiling;
        }
    }

    uint32_t div = (want >= src) ? 2u : ((src + want - 1u) / want);
    if (div < 2u)   { div = 2u; }
    if ((div & 1u) != 0u) { div++; }
    if (div > 512u) { div = 512u; }

    timebase_set(src / div);

    CLOCK_EnableClock((cap.flexio == 2u) ? kCLOCK_Flexio2 : kCLOCK_Flexio1);

    flexio_config_t fc;
    FLEXIO_GetDefaultConfig(&fc);
    fc.enableFlexio = false;          /* configured first, started last */
    FLEXIO_Init(base, &fc);

    memset(&tc, 0, sizeof tc);
    tc.triggerSelect   = 0u;
    tc.triggerPolarity = kFLEXIO_TimerTriggerPolarityActiveHigh;
    tc.triggerSource   = kFLEXIO_TimerTriggerSourceInternal;
    /* No pin: the sample clock is internal and must not appear on a pad. A
     * sniffer that drives one of its own taps is not a sniffer. */
    tc.pinConfig       = kFLEXIO_PinConfigOutputDisabled;
    tc.pinSelect       = 0u;
    tc.pinPolarity     = kFLEXIO_PinActiveHigh;
    tc.timerMode       = kFLEXIO_TimerModeDual8BitBaudBit;
    tc.timerOutput     = kFLEXIO_TimerOutputZeroNotAffectedByReset;
    tc.timerDecrement  = kFLEXIO_TimerDecSrcOnFlexIOClockShiftTimerOutput;
    /* Never reset, never disable, always enabled: a free-running sample clock.
     * Anything else would stop the timer at the first compare and sample once. */
    tc.timerReset      = kFLEXIO_TimerResetNever;
    tc.timerDisable    = kFLEXIO_TimerDisableNever;
    tc.timerEnable     = kFLEXIO_TimerEnabledAlways;
    tc.timerStop       = kFLEXIO_TimerStopBitDisabled;
    tc.timerStart      = kFLEXIO_TimerStartBitDisabled;
    /* High byte counts shift-clock edges - two per load - and the low byte is
     * the clock divider expressed the same way. */
    tc.timerCompare    = (((CAP_SAMPLES_PER_WORD * 2u) - 1u) << 8)
                       | ((div / 2u) - 1u);
    if (cap.mode == (uint8_t)BB_CAP_PINCLOCK) {
        /*
         * The bus clocks us. Tap 0 is SCL and becomes the timer's pin, so the
         * shifters advance exactly once per bus clock instead of oversampling a
         * free-running tick.
         *
         * Rising edges only, because I2C and I3C SDR define the data as valid
         * on SCL's rising edge - that is the instant the bus itself nominates,
         * and sampling there is what makes this bit-exact rather than merely
         * fast. `kFLEXIO_TimerDecSrcOnPinInputShiftPinInput` would take both
         * edges and give two samples per bit; that is the configuration HDR-DDR
         * would want, and is left for when HDR is implemented.
         *
         * Free-running, unlike the SDK's SPI-target template this is modelled
         * on. That one gates the timer on chip select - enable on trigger
         * rising, disable on compare - because a target knows when its transfer
         * begins. A sniffer does not: there is no select line and no start, only
         * a bus that has been running since before we were switched on. So the
         * timer is enabled always, reset never, disabled never, and simply
         * counts SCL edges forever.
         *
         * The compare value is the word boundary: 32 rising edges fill a shifter
         * and load SHIFTBUF, which is the same 32-samples-per-word packing the
         * oversampled path uses, so everything downstream is unchanged.
         */
        memset(&tc, 0, sizeof tc);
        /*
         * The START detector resets this timer, in hardware.
         *
         * A receive shifter emits nothing until its word fills, so a frame that
         * ends part-way through a word strands those bits: they sit at the front
         * of the next frame's first word and rotate every byte of it. Software
         * cannot repair that - three attempts at resetting the shifter from the
         * START flag all act microseconds late, on bits that are already in
         * flight, and narrowing the word to one bit is eight times the register
         * traffic and overruns below the rate it would replace.
         *
         * Timer 2 already detects START in hardware, with no latency at all.
         * Pointing this timer's reset at that timer's output puts the word
         * boundary back on the first bit of every transaction, at the instant
         * the transaction starts.
         *
         * This is a third trigger mechanism and worth stating separately,
         * because the second one does not work on this part: a trigger-gated
         * *decrement* counts nothing (see cap_has_frame()), while a
         * trigger-gated *enable* works and is what the framing timers use.
         * Reset-on-trigger is neither of those.
         */
        tc.triggerSelect   = cap_has_frame()
                                 ? FLEXIO_TIMER_TRIGGER_SEL_TIMn(CAP_FRAME_TIMER + 1u)
                                 : 0u;
        tc.triggerPolarity = kFLEXIO_TimerTriggerPolarityActiveHigh;
        tc.triggerSource   = kFLEXIO_TimerTriggerSourceInternal;
        tc.pinConfig       = kFLEXIO_PinConfigOutputDisabled;
        tc.pinSelect       = cap.pins[0];          /* SCL - the bus's own clock */
        /*
         * The timer counts the *falling* edge so that the load and the shift
         * happen on opposite edges of the bus clock.
         *
         * With both on the rising edge they race, and the shift loses: SHIFTBUF
         * is latched as the timer compares, before that edge's sample has entered
         * the shifter, so the final position of every word arrives empty and reads
         * as zero. It is invisible whenever the true bit happened to be zero,
         * which is why it looked intermittent - of four words in a test transfer,
         * two were wrong and two were right by luck.
         *
         * Inverting the timer's view of the pin puts the reload half a bit period
         * after the sample, by which time it has landed.
         */
        tc.pinPolarity     = kFLEXIO_PinActiveLow;
        tc.timerMode       = kFLEXIO_TimerModeSingle16Bit;
        tc.timerOutput     = kFLEXIO_TimerOutputZeroNotAffectedByReset;
        tc.timerDecrement  = kFLEXIO_TimerRisSrcOnPinInputShiftPinInput;
        tc.timerReset      = cap_has_frame()
                                 ? kFLEXIO_TimerResetOnTimerTriggerRisingEdge
                                 : kFLEXIO_TimerResetNever;
        tc.timerDisable    = kFLEXIO_TimerDisableNever;
        tc.timerEnable     = kFLEXIO_TimerEnabledAlways;
        tc.timerStop       = kFLEXIO_TimerStopBitDisabled;
        tc.timerStart      = kFLEXIO_TimerStartBitDisabled;
        tc.timerCompare    = (uint32_t)cap.bits_per_word - 1u;

        /*
         * There is no sample rate here and none is invented.
         *
         * The shift clock is the bus, so the rate is whatever the bus is doing,
         * and we do not know it until we have watched it. timebase_set()
         * substitutes 1 MHz for a zero argument - a sensible default for a
         * capture that needs *some* timebase, and a wrong answer to report as a
         * measurement - so the fields are set directly instead. A host reading
         * sample_hz=0 alongside mode=pinclock knows the ticks are bus clocks.
         */
        cap.sample_hz   = 0u;
        cap.ps_per_tick = 0u;
    }

    FLEXIO_SetTimerConfig(base, CAP_TIMER, &tc);

    cap_program_shifters(base);

    if (cap_has_frame()) {
        /*
         * Framing: two shifters clocked by SDA, each sampling SCL.
         *
         * A START is SDA falling while SCL is high and a STOP is SDA rising
         * while SCL is high, so "what was the clock doing when the data line
         * moved" answers both - and which direction moved is the shifter's own
         * identity, because each has a timer counting one direction only.
         * `pinPolarity` ActiveLow inverts SDA on the way in, so a rising count on
         * an inverted input is a fall on the wire.
         *
         * Not trigger-gated, and that is a finding rather than a preference. The
         * obvious design is to let hardware qualify "while SCL is high" by
         * triggering the timer from SCL, and on this part a trigger-gated timer
         * does not decrement on its pin input at all: ungated it counts SDA
         * edges perfectly, gated it counts nothing, at either trigger polarity
         * and in eight combinations of enable, disable and reset. The RM's own
         * TRGSEL encoding and TRGSRC=internal were verified against the register
         * documentation, so the configuration was right and the behaviour is
         * not what the register description implies. Software does the
         * qualification instead, which costs one comparison per SDA edge.
         *
         * One edge per word. A shifter delivers nothing until it fills, and
         * framing events are sparse - a 32-edge word meant the STOP that ends a
         * transaction sat unread until unrelated traffic arrived to push it out.
         */
        flexio_timer_config_t ft;

        for (unsigned f = 0; f < 2u; f++) {
            memset(&ft, 0, sizeof ft);
            /*
             * SCL is the trigger, and it gates the timer's ENABLE rather than
             * its decrement. That distinction is the whole fix.
             *
             * Gating the decrement - a timer that counts SDA edges only while
             * SCL is high - is the obvious design and does not work on this
             * part: ungated it counts SDA edges perfectly, gated it counts
             * nothing, at either trigger polarity and across eight combinations
             * of enable, disable and reset. That result stands and is why
             * software did the qualification for so long.
             *
             * The enable condition is a different mechanism and does work.
             * TIMENA = 101b enables the timer on a pin edge *provided the
             * trigger is high*, so the timer only ever starts on an SDA edge
             * that happened while SCL was held - which is the definition of
             * START and STOP. It then counts down from its compare, raises its
             * status flag and disables itself, ready to re-arm on the next
             * qualifying edge. The flag IS the qualified event.
             */
            ft.triggerSelect   = FLEXIO_TIMER_TRIGGER_SEL_PININPUT(cap.pins[0]);
            ft.triggerPolarity = kFLEXIO_TimerTriggerPolarityActiveHigh;
            ft.triggerSource   = kFLEXIO_TimerTriggerSourceInternal;
            ft.pinConfig       = kFLEXIO_PinConfigOutputDisabled;
            ft.pinSelect       = cap.pins[1];        /* SDA */
            /* An inverted input turns a fall on the wire into the rising edge
             * the enable condition looks for, so f==1 is the START detector. */
            ft.pinPolarity     = (f == 0u) ? kFLEXIO_PinActiveHigh  /* rises */
                                           : kFLEXIO_PinActiveLow;  /* falls */
            ft.timerMode       = kFLEXIO_TimerModeSingle16Bit;
            ft.timerOutput     = kFLEXIO_TimerOutputZeroNotAffectedByReset;
            /* Counts the FlexIO clock, not the pin: the pin's job here is to
             * start it, and the count only has to reach compare to raise the
             * flag. At 120 MHz that is tens of nanoseconds. */
            ft.timerDecrement  = kFLEXIO_TimerDecSrcOnFlexIOClockShiftTimerOutput;
            ft.timerReset      = kFLEXIO_TimerResetNever;
            ft.timerDisable    = kFLEXIO_TimerDisableOnTimerCompare;
            ft.timerEnable     = kFLEXIO_TimerEnableOnPinRisingEdgeTriggerHigh;
            ft.timerStop       = kFLEXIO_TimerStopBitDisabled;
            ft.timerStart      = kFLEXIO_TimerStartBitDisabled;
            ft.timerCompare    = 1u;                 /* shortest one-shot */
            FLEXIO_SetTimerConfig(base, (uint8_t)(CAP_FRAME_TIMER + f), &ft);

        }
    }

    FLEXIO_ClearShifterErrorFlags(base, cap_all_shifter_mask());
    return BB_CAP_OK;
}

/* ---- decoding ----------------------------------------------------------- */
/*
 * An I2C decoder, fed one sample at a time from the same intake that fills the
 * sample ring.
 *
 * Decoding here rather than on the host is the spec's second capture mode, and
 * running it off the same samples is what makes the two modes consistent: a host
 * can read the waveform with SNIFF:RAW? and the transaction with SNIFF:READ? and
 * they cannot disagree, because there is one source of truth and one pass over
 * it.
 *
 * The rules are the ones I2C is defined by, and each is stated because getting
 * any of them slightly wrong produces a decode that looks plausible:
 *
 *   START    SDA falls while SCL is high
 *   STOP     SDA rises while SCL is high
 *   bit      SDA is sampled on SCL's rising edge
 *   byte     eight bits, most significant first, then a ninth bit which is the
 *            acknowledgement - low means acknowledged
 *   address  the first byte after a START; every byte after that is data
 *
 * A repeated START - a START with no STOP before it - is a distinct event,
 * because a host reading `S A50+ D00+ S A51+ D..` needs to know the second
 * transaction was not preceded by the bus being released.
 *
 * Channel order comes from SNIFF:CH: tap 0 is the clock, tap 1 is the data. That
 * is the order proto_default_taps() assigns and the order the reply documents,
 * and it is the one thing here that is a convention rather than a rule.
 */
#define DEC_CLK_BIT   0u
#define DEC_DAT_BIT   1u

#define CAP_EVENT_ENTRIES  512u

static bb_event_t s_events[CAP_EVENT_ENTRIES];
static volatile uint32_t s_ev_write;
static volatile uint32_t s_ev_read;

static struct {
    bool     in_frame;    /**< between a START and its STOP */
    bool     expect_addr; /**< the next completed byte is an address */
    uint8_t  bit_count;   /**< bits shifted into `byte` so far */
    uint8_t  byte;
    bool     have_clk;    /**< a previous sample exists to compare against */
    uint8_t  prev_clk;
    uint8_t  prev_dat;
    uint32_t clk_rise_tick; /**< when SCL last rose - see the guard in dec_sample */
    bool     have_rise;
    uint32_t dropped;     /**< events lost because the ring was full */
} dec;

static void dec_reset(void)
{
    memset(&dec, 0, sizeof dec);
    s_ev_write = s_ev_read = 0u;
}

/*
 * Tell the decoder what the lines are sitting at, without calling it an edge.
 *
 * A decoder fed transitions must never spend a real transition on working out
 * where it started, and the first version did exactly that: dec_reset() cleared
 * `have_clk`, so the next transition was consumed purely to establish a
 * reference. On an idle-high I2C bus the next transition is the START - so the
 * START was swallowed, `in_frame` never set, and every clock edge after it was
 * discarded. The whole frame decoded to nothing.
 *
 * The level is not ours to guess: the intake already tracks it, because the
 * sample contract makes the first record the initial state rather than a
 * transition. So the reference comes from there, and both callers are places
 * where that state is known to be current.
 */
static void dec_seed(uint32_t state)
{
    dec.prev_clk = (uint8_t)((state >> DEC_CLK_BIT) & 1u);
    dec.prev_dat = (uint8_t)((state >> DEC_DAT_BIT) & 1u);
    dec.have_clk = true;
}

static void ev_push(bb_ev_kind_t kind, uint8_t value, bool ack)
{
    uint32_t used = s_ev_write - s_ev_read;

    if (used >= CAP_EVENT_ENTRIES) {
        /*
         * Counted, not overwritten. A dropped event is emitted as `!` when the
         * ring next drains, so a host sees a hole rather than a stream that
         * silently skips a byte - the same contract the sample path uses for a
         * gap, and for the same reason.
         */
        dec.dropped++;
        return;
    }
    s_events[s_ev_write % CAP_EVENT_ENTRIES].kind  = (uint8_t)kind;
    s_events[s_ev_write % CAP_EVENT_ENTRIES].value = value;
    s_events[s_ev_write % CAP_EVENT_ENTRIES].ack   = ack ? 1u : 0u;
    s_events[s_ev_write % CAP_EVENT_ENTRIES].pad   = 0u;
    s_ev_write++;
}

static void dec_sample(uint32_t state, uint32_t tick)
{
    uint8_t clk = (uint8_t)((state >> DEC_CLK_BIT) & 1u);
    uint8_t dat = (uint8_t)((state >> DEC_DAT_BIT) & 1u);

    if (!dec.have_clk) {
        /* Nothing to compare against yet: the first sample only establishes a
         * reference. Treating it as an edge would invent one at t=0, which is
         * the same phantom the sample path is careful to avoid. */
        dec.prev_clk  = clk;
        dec.prev_dat  = dat;
        dec.have_clk  = true;
        return;
    }

    /* START and STOP are SDA moving while SCL stays high. Checked before the
     * clock edge, because a sample where both moved is a malformed bus and
     * treating it as a data bit would put a bogus byte in the stream. */
    /*
     * Edge detection is shared; byte semantics are not.
     *
     * Finding START, STOP and the sampling instant is identical for both
     * protocols - it is electrical. What the bytes mean afterwards is not, so
     * I3C hands off to its own state machine from here and this function keeps
     * only the edge work. The alternative, branching on protocol inside the
     * byte assembler, is what dec_i3c.c exists to avoid.
     */
    bool i3c = (cap.proto == BB_PROTO_I3C);

    /*
     * When the clock last rose, in capture ticks.
     *
     * START and STOP move SDA in the MIDDLE of a settled clock-high period.
     * The edges either side of a ninth bit do not: the target releases SDA
     * after acknowledging, and the controller drives the next bit immediately
     * afterwards, and on I3C's push-pull phases both land close enough to the
     * clock edge to be sampled as "SDA moved while SCL was high".
     *
     * Read as framing, that phantom restart lands between the broadcast address
     * and the command code and resets the byte in progress - so every CCC
     * decoded as 0x00 whatever was sent, and ENTDAA lost the one byte that says
     * it is an ENTDAA.
     *
     * Measured in TICKS, not in decoder calls. An earlier attempt counted
     * consecutive calls with SCL high and suppressed all framing including real
     * STARTs, because in oversample mode the decoder is fed transitions rather
     * than samples - a run length counted in calls never exceeds one. The tick
     * is the capture's own timebase and does not have that problem.
     *
     * I3C only. I2C changes SDA far from the clock edge and has never needed
     * it, and adding a qualification to a path that is 5/5 exact at every speed
     * it supports would be risk without benefit.
     */
    if ((clk == 1u) && (dec.prev_clk == 0u)) {
        dec.clk_rise_tick = tick;
        dec.have_rise     = true;
    }

    bool settled = !i3c || !dec.have_rise ||
                   ((tick - dec.clk_rise_tick) > 1u);

    if (settled && (clk == 1u) && (dec.prev_clk == 1u) && (dat != dec.prev_dat)) {
        if (dat == 0u) {
            if (i3c) {
                dec_i3c_start(dec.in_frame);
            } else {
                ev_push(dec.in_frame ? BB_EV_RESTART : BB_EV_START, 0u, false);
            }
            dec.in_frame    = true;
            dec.expect_addr = true;
            dec.bit_count   = 0u;
            dec.byte        = 0u;
        } else if (dec.in_frame) {
            if (i3c) {
                dec_i3c_site(1u);
                dec_i3c_stop();
            } else {
                ev_push(BB_EV_STOP, 0u, false);
            }
            dec.in_frame  = false;
            dec.bit_count = 0u;
            dec.byte      = 0u;
        }
    } else if (i3c && (clk == 1u) && (dec.prev_clk == 0u) && dec.in_frame) {
        dec_i3c_bit(dat);
    } else if ((clk == 1u) && (dec.prev_clk == 0u) && dec.in_frame) {
        /* SCL rising: the data line is valid now, which is the whole reason a
         * pin-clocked or oversampled capture beats reading the port late. */
        if (dec.bit_count < 8u) {
            dec.byte = (uint8_t)((dec.byte << 1) | dat);
            dec.bit_count++;
        } else {
            /* The ninth bit: low is an acknowledgement. */
            ev_push(dec.expect_addr ? BB_EV_ADDR : BB_EV_DATA,
                    dec.byte, dat == 0u);
            dec.expect_addr = false;
            dec.bit_count   = 0u;
            dec.byte        = 0u;
        }
    }

    dec.prev_clk = clk;
    dec.prev_dat = dat;
}

/* ---- sample intake ------------------------------------------------------ */

static void cap_dma_stop(void);

static void cap_hw_stop(void)
{
    FLEXIO_Type *base = cap_hw();

    cap_dma_stop();

    FLEXIO_Enable(base, false);
    cap.running = false;
}

static void push_sample(uint32_t state, uint64_t at_tick)
{
    if ((s_write - s_read) >= CAP_RING_ENTRIES) {
        /*
         * The ring does not wrap. Overwriting the oldest entries would move
         * where the capture began without saying so, and a capture whose start
         * is unknown cannot be decoded against a golden corpus. Capture stops,
         * `full` is set, and the drain emits a gap record.
         */
        cap.full = true;
        cap_hw_stop();
        return;
    }

    uint64_t dt = at_tick - cap.last_tick;

    /*
     * `dt_ticks` is 32-bit. A longer gap than that - an idle bus between two
     * bursts, which is the common case rather than a theoretical one - is
     * carried by clamping here and letting the host see a maximal delta. The
     * alternative the header describes, a fresh SAMPLES record with a new
     * absolute base, belongs in the drain path where record boundaries are
     * decided; clamping keeps the ring entries uniform and never wraps silently.
     */
    if (dt > 0xFFFFFFFFull) {
        dt = 0xFFFFFFFFull;
    }
    s_ring[s_wr_idx].dt_ticks = (uint32_t)dt;
    s_ring[s_wr_idx].state    = state;
    if (++s_wr_idx >= CAP_RING_ENTRIES) {
        s_wr_idx = 0u;
    }
    s_write++;
    cap.entries++;
    cap.last_tick = at_tick;
}

/*
 * Framing state for pin-clocked capture.
 *
 * The events themselves come from two trigger-gated timers whose status flags
 * are the detection - see the framing timers in flexio_program(). All that is
 * left for software is whether a frame is currently open, which is what
 * separates a first START from a repeated one and stops a STOP being reported
 * for a frame that never began.
 */
static struct {
    bool in_frame;      /**< between a START and its STOP */
    bool next_rising;   /**< which half of the alternating edge stream is next */
    bool     stop_pending;  /**< STOP seen, waiting for its frame's data to drain */
    uint32_t stop_idle;     /**< consecutive polls with no new bits consumed */
    uint64_t stop_tick;     /**< sample tick when the STOP was last checked */
    /*
     * A START waits for its frame's data to drain too, for exactly the reason
     * a STOP does - and that it did not is what made pin-clocked capture lose
     * every frame after the first.
     *
     * The stranded bits at the end of a frame are not lost. A shifter emits
     * nothing until its word fills, so they stay in it and come out as the
     * leading bits of the next word - late, but present and in order. The bit
     * stream across frames is therefore complete; what is wrong is where the
     * frame events land in it. Applying a START the moment the timer flag is
     * seen resets the decoder's bit counter while the previous frame's tail is
     * still in front of it, so those bits are read as the new frame's address
     * and every byte after them is rotated.
     *
     * Deferred to a byte boundary, the same test the STOP already used, the
     * tail decodes as the frame it belongs to and the START lands on the first
     * bit of the next one.
     */
    bool     start_pending;
    bool     start_repeated;
    /*
     * Non-zero when a START or a STOP is waiting to be applied.
     *
     * The per-bit guard tested each separately, which is a load from this struct
     * per bus clock for a condition that is false through the whole body of
     * every transaction. One flag, maintained where they are set and cleared,
     * is one load.
     */
    uint32_t pending;
    /*
     * Bits consumed since the current frame's START was applied.
     *
     * A transaction opens with an address and its acknowledge, so a frame that
     * closes before nine bits have been clocked was never a transaction - which
     * is how the handover at the end of a read, where the target releases SDA
     * while the clock is held, is told apart from a real frame.
     */
    uint32_t frame_bits;
    /*
     * Clock edges between one transaction and the next that carry no data.
     *
     * Ending a transaction and beginning the next moves SCL without a bit
     * riding on it, and the data shifter counts every rising edge it sees. Two
     * is what a sweep at 1 MHz gives: none decodes the next address two bits
     * early, one decodes it one bit early, two is exact.
     *
     * It is not right everywhere - see docs/STATUS.md section 4.1. Removing it
     * entirely was tried, on the reasoning that the decoder already discards
     * bits outside a frame, and made every rate decode one bit early rather
     * than fixing the two that did. Two is the value that is right at the most
     * rates measured, and it stays until the mechanism is replaced rather than
     * retuned.
     */
    uint8_t  skip_bits;
    /*
     * Where the bits between two transactions actually go.
     *
     * The wire gap is one edge, measured from the raw sample stream. The
     * decoder needs two or three skipped. Those are different numbers, so bits
     * are being consumed somewhere between the STOP being flagged and the START
     * being applied, and modelling has already been wrong twice about where.
     *
     * `defer_bits` counts bits consumed while a STOP is pending - the deferral
     * waiting for a decoder byte boundary. `arm_bits` counts bits consumed
     * between that STOP being applied and the START being applied. Together
     * they account for every bit between the two frames.
     */
    uint16_t defer_bits;
    uint16_t arm_bits;
    /* Sample tick at which each framing flag was seen, so their order is
     * recorded rather than inferred from which block happened to run. */
    uint32_t start_at;
    uint32_t stop_at;
    /*
     * The same two events in bits rather than cycles.
     *
     * Two clocks, because the two questions need different units. CYCCNT
     * orders them; cap.tick counts the bus clocks between them, which is what
     * tells a STOP's setup fall from a real frame: the setup fall is one bus
     * clock ahead of its STOP, a genuine frame is twenty-seven or more.
     * Quantised to the word, so it reads as 0 or 8 for a setup and 24+ for a
     * frame - far enough apart to separate cleanly.
     */
    uint32_t start_tick;
    uint32_t stop_tick_at;
    /*
     * Where the next frame begins, measured from the last one's end.
     *
     * The START flag is a poor anchor. It is prompt while the bits it precedes
     * are not, so the distance between the flag and the frame's real first bit
     * varies with how many bits the previous frame stranded in the shifter -
     * and worse, until the flag arrives no frame is open, so the bits that do
     * arrive are discarded by the decoder and the frame opens however late the
     * flag happens to be. That is the variance behind a frame decoding one bit
     * short: `A13+ D40+ ...` arriving as `A26+ D81- ...`, which is the whole
     * frame shifted left by one.
     *
     * The previous frame's end is a much better anchor, because it is known in
     * the decoder's own bit stream rather than in flag time: when a frame
     * closes on a unit boundary, that boundary *is* its last bit. The wire gap
     * after it was measured directly from the raw samples - one clock edge,
     * every rate, both directions - so the next frame's first bit is a fixed,
     * known distance away. `anchor_open` carries that: set when the gap has
     * been skipped, it opens the next frame on the very next bit whether or not
     * a START flag has turned up yet.
     */
    bool     anchor_open;
    /* The current frame was opened by the anchor above, so a START flag
     * arriving late must be consumed rather than applied - applying it would
     * reset the decoder mid-frame and undo the alignment just established. */
    bool     opened_anchored;
    /* STARTs settled as a STOP's setup fall and discarded. One per read is the
     * figure this should reach. */
    uint16_t setup_starts;
    /* Direction of the frame that just closed. The lead to drop from the next
     * frame's replay depends on it - see the replay. */
    bool     prev_read;
    uint16_t frame_last;
    uint16_t defer_last;
    uint16_t arm_last;
    /*
     * How often both framing events are pending at once, and how often their
     * notes are separable in time when they are.
     *
     * The application order of a pending STOP against a pending START is the
     * whole of the remaining decode bug, and the two cases need different
     * answers: noted in the same poll the order is unknowable and physics says
     * STOP first, noted in different polls the order is known outright. These
     * count the two so the split can be seen per rate rather than assumed.
     */
    uint16_t bothpend_n;
    uint16_t reorder_n;
} frm;


static uint32_t s_dma_avail_max;    /* deepest the DMA queue was found */
/*
 * A STOP has to be set up, and setting it up looks exactly like a START.
 *
 * Releasing SDA high while SCL is held is a STOP, so SDA has to be *low* first.
 * When the last bit of a frame left SDA high the controller must pull it down,
 * and SDA falling while SCL is high is precisely what the START detector is
 * built to find. The detector is not wrong - the edge is real, and it is not a
 * START.
 *
 * Measured: 13 transactions (7 writes, 6 reads) raise 13 STOP flags and 19
 * START flags, at every rate. Six extra, one per read, none for the writes.
 * That is the whole discrepancy, and it is why thirteen transactions decoded as
 * six frames: each phantom START opened a frame at the end of a read that then
 * swallowed the transaction after it.
 *
 * Which reads take the extra edge is not a guess. An I3C read's ninth bit is
 * "more data follows", not parity, and the controller ends the transfer itself
 * with RDTERM - so the final T bit of a read is always 1 and SDA is always
 * high when the frame ends. A read therefore always needs the setup fall. A
 * write's final T is the parity of its last byte and is 0 here (`D40-`), so it
 * needs none. Direction is known at the STOP, which is what makes this causal
 * rather than fitted.
 *
 * Timing cannot do this job. The flags were stamped and measured: the gap
 * between a setup fall and its STOP is 800 to 2000 core cycles, which is the
 * *poll period* under traffic - about ten bit times at 6 MHz - not the sub-bit
 * separation the wire has. The framing detector is polled far more coarsely
 * than a bit, so a time window wide enough to catch every setup fall is also
 * wide enough to swallow genuine boundaries. The attempt is recorded in
 * docs/STATUS.md 4.1 so it is not repeated.
 */
static uint32_t s_frame_words[2];   /* [0] rising, [1] falling - diagnostic */
static uint32_t s_frame_hi[2];      /* of those, how many had SCL high */

static void frame_reset(void)
{
    memset(&frm, 0, sizeof frm);
    s_dma_avail_max  = 0u;
    s_frame_words[0] = s_frame_words[1] = 0u;
    s_frame_hi[0] = s_frame_hi[1] = 0u;
}

/*
 * The DMA intake, for pin-clocked capture.
 *
 * Measured with the cycle counter, a pin-clocked word costs about 520 cycles to
 * take in and decode, against a budget of 486 at 12.5 MHz - one shifter word
 * every 640 ns on a 792 MHz core. Roughly 180 of those cycles are not decoding
 * at all: they are the shifter status read, the settle before the buffer read,
 * and the buffer read itself, all across the FlexIO bus.
 *
 * eDMA removes every one of them. The engine moves SHIFTBUF into memory on the
 * shifter's own request, and this core reads words out of RAM and looks at a
 * DMA register once per poll rather than three FlexIO registers per word.
 *
 * Scoped to pin-clocked mode deliberately. It has one data shifter, so one
 * channel does it; oversampled capture has up to four and is nowhere near its
 * ceiling anyway.
 */
#define CAP_DMA_CH     7u      /* CM33 uses DMABENCH_CH for SYS:DMACAP */
#define CAP_DMA_WORDS  512u    /* 328 us of buffer at 12.5 MHz */

AT_NONCACHEABLE_SECTION_ALIGN(static uint32_t s_dma_buf[CAP_DMA_WORDS], 32);
static uint32_t s_dma_rd;      /* our index into the ring */
static bool     s_dma_on;

static void cap_dma_stop(void)
{
    if (!s_dma_on) {
        return;
    }
    DMA4->TCD[CAP_DMA_CH].CH_CSR = 0u;                    /* ERQ off */
    DMA4->TCD[CAP_DMA_CH].CH_INT = 1u;
    DMA4->TCD[CAP_DMA_CH].CH_ES  = 0x80000000u;
    cap_hw()->SHIFTSDEN = 0u;
    s_dma_on = false;
}

static bool cap_dma_start(FLEXIO_Type *base)
{
    volatile uint32_t *src = &base->SHIFTBUF[0];

    s_dma_rd = 0u;
    memset(s_dma_buf, 0, sizeof s_dma_buf);

    DMA4->TCD[CAP_DMA_CH].CH_CSR = 0u;
    DMA4->TCD[CAP_DMA_CH].CH_CSR = DMA4_CH_CSR_DONE_MASK;   /* w1c */
    DMA4->TCD[CAP_DMA_CH].CH_CSR = 0u;
    DMA4->TCD[CAP_DMA_CH].CH_INT = 1u;
    DMA4->TCD[CAP_DMA_CH].CH_ES  = 0x80000000u;
    DMA4->TCD[CAP_DMA_CH].CH_MUX = 37u;    /* FlexIO1 shifter 0 request */

    DMA4->TCD[CAP_DMA_CH].SADDR          = (uint32_t)(uintptr_t)src;
    DMA4->TCD[CAP_DMA_CH].SOFF           = 0;               /* one register */
    DMA4->TCD[CAP_DMA_CH].ATTR           = DMA4_ATTR_SSIZE(2u) |
                                           DMA4_ATTR_DSIZE(2u);
    DMA4->TCD[CAP_DMA_CH].NBYTES_MLOFFNO = 4u;
    DMA4->TCD[CAP_DMA_CH].SLAST_SDA      = 0;
    DMA4->TCD[CAP_DMA_CH].DADDR          = (uint32_t)(uintptr_t)s_dma_buf;
    DMA4->TCD[CAP_DMA_CH].DOFF           = 4;
    DMA4->TCD[CAP_DMA_CH].CITER_ELINKNO  = (uint16_t)CAP_DMA_WORDS;
    DMA4->TCD[CAP_DMA_CH].BITER_ELINKNO  = (uint16_t)CAP_DMA_WORDS;
    /* Rewind at major-loop completion: the buffer is a ring, and the engine
     * reloads CITER from BITER on its own, so the transfer never ends. */
    DMA4->TCD[CAP_DMA_CH].DLAST_SGA      = -(int32_t)(CAP_DMA_WORDS * 4u);
    DMA4->TCD[CAP_DMA_CH].CSR            = 0u;

    base->SHIFTSDEN = 1u;                  /* shifter 0 raises DMA requests */
    __DSB();
    DMA4->TCD[CAP_DMA_CH].CH_CSR = DMA4_CH_CSR_ERQ_MASK;
    s_dma_on = true;
    return true;
}

/** How many words the engine has written that we have not read. */
static uint32_t cap_dma_avail(void)
{
    uint32_t citer = DMA4->TCD[CAP_DMA_CH].CITER_ELINKNO & 0x7FFFu;
    uint32_t wr    = (citer > CAP_DMA_WORDS) ? 0u : (CAP_DMA_WORDS - citer);

    return (wr - s_dma_rd) & (CAP_DMA_WORDS - 1u);
}

static uint32_t s_idle_pass;

/*
 * Cycle profiling for the intake, on the core that runs it.
 *
 * DWT's cycle counter is the only honest way to answer "where does the 640 ns
 * go". Totals are shifted down by six so a long capture cannot wrap a 32-bit
 * field; the interesting numbers are ratios, not absolutes.
 */
static struct {
    uint32_t polls;
    uint32_t words;
    uint64_t cyc_poll;
    uint64_t cyc_word;
    uint32_t cyc_max;      /**< worst single cap_poll(), where the spikes are */
} s_prof;

static void prof_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT       = 0u;
    DWT->CTRL        |= DWT_CTRL_CYCCNTENA_Msk;
    memset(&s_prof, 0, sizeof s_prof);
}

static void cap_consume(const uint32_t *words)
{
    /*
     * A receive shifter fills from the top and shifts right, so after n shifts
     * the samples occupy the *most significant* n bits with the oldest at
     * 32 - n. With a full 32-bit word that reduces to bit k, which is what the
     * oversampled path has always used; with a short pin-clocked word the offset
     * is what stops the reader picking up stale bits below the data.
     */
    const uint32_t t0 = DWT->CYCCNT;
    const uint32_t n = cap.bits_per_word;
    /*
     * Hoisted out of the per-sample loop.
     *
     * cap_shifter_count() was called once per *sample*, and in pin-clocked mode
     * a sample is a bus clock: at 12.5 MHz that is a function call every 80 ns,
     * for a value that cannot change while a capture is running. Together with
     * the ring index, this is what the pin-clocked ceiling was made of.
     */
    const unsigned nsh = cap_shifter_count();
    /*
     * Loop-invariant state in locals, written back once.
     *
     * `cap` is a plain struct in memory, so `cap.mode`, `cap.decode_off`,
     * `cap.last_state` and `cap.changed_mask` were each loaded - and two of
     * them stored - on every bit. Measured with the cycle counter: a word cost
     * 384 cycles for eight bits, against a budget of 507 for the whole word at
     * 12.5 MHz. Nothing here can change while the loop runs.
     */
    const bool pinclk  = (cap.mode == (uint8_t)BB_CAP_PINCLOCK);
    const bool decode  = !cap.decode_off;
    const bool store   = !(pinclk && decode);
    uint32_t   last    = cap.last_state;
    uint32_t   changed = cap.changed_mask;
    /*
     * The tick in a register for the length of the word.
     *
     * It is a 64-bit field in a struct, so `cap.tick++` per bit is a load, an
     * add-with-carry and a store to DTCM. Nothing outside this loop reads it
     * while the loop runs.
     */
    uint64_t   tick    = cap.tick;

    for (uint32_t k = 0; k < n; k++) {
#if CAP_OLDEST_IS_HIGH_SLOT
        uint32_t bit = CAP_SAMPLES_PER_WORD - 1u - k;
#else
        uint32_t bit = (CAP_SAMPLES_PER_WORD - n) + k;
#endif
        uint32_t st = 0u;

        if (nsh == 1u) {
            /* The pin-clocked case, and the one that has to be fast: tap 0 is
             * the bus clock, so exactly one shifter carries data. */
            st = (words[0] >> bit) & 1u;
        } else {
            for (unsigned i = 0; i < nsh; i++) {
                if (((words[i] >> bit) & 1u) != 0u) {
                    st |= (1uL << i);
                }
            }
        }
        tick++;

        /*
         * Pin-clocked samples are *bits*, and every one of them is recorded.
         *
         * Oversampled capture stores transitions because a repeated level
         * carries no new information about the waveform. That reasoning is
         * exactly inverted here: a sample is taken only where the bus said the
         * data was valid, so two identical samples in a row are two identical
         * *data bits*, and discarding the second would delete payload rather
         * than redundancy. Two 0xFF bytes would arrive as one edge.
         *
         * The tick count means something different too - it counts bus clocks,
         * not sample periods - which is why the mode travels in the record
         * metadata rather than being inferred from the data.
         */
        if (pinclk) {
            /*
             * The first bit establishes the reference and is still recorded.
             *
             * Without this the XOR runs against whatever `last_state` held from
             * a previous capture, and sets `changed_mask` bits for channels that
             * never moved - a two-tap pin-clocked run reported both channels
             * moving when only one shifter exists, and the self test failed a
             * capture that was working perfectly. The sample itself is data
             * either way, so unlike the oversampled path it is pushed, not
             * swallowed.
             */
            if (!cap.have_initial) {
                /*
                 * The first pin-clocked sample is synchroniser state, not a bus
                 * bit, and is dropped - contract 3 in its pin-clocked form.
                 *
                 * FlexIO synchronises every pin input to its own clock, and that
                 * synchroniser starts at zero. Enabling the module while the
                 * clock line sits idle high therefore latches a 0->1 transition
                 * that is indistinguishable from a rising edge, and the capture
                 * comes back correct but shifted by one. It is exactly one,
                 * deterministically, because it can only happen once per enable.
                 *
                 * The oversampled path spends its first sample the same way and
                 * for a related reason - there it establishes the initial level
                 * so a bus already idle does not appear to rise at t=0. Same
                 * rule, same place, different phantom.
                 */
                cap.have_initial = true;
                last             = st;
                continue;
            }
            changed |= (st ^ last);
            last     = st;
            /*
             * Pin-clocked decode does not also store raw samples.
             *
             * A sample here is a bus clock, so storing one per bit is a ring
             * write every 80 ns at 12.5 MHz - and the ring write was the single
             * largest per-sample cost. Dropping it while decoding took the
             * exact-capture ceiling from 6 MHz to 8 MHz on its own.
             *
             * Nothing is lost that the caller cannot have: the events carry the
             * decode, and `SNIFF:DECODE 0` still records every raw sample. What
             * is not available is both at once at pin-clock rates, which is a
             * fair trade for a mode whose entire purpose is keeping up with a
             * fast bus.
             */
            if (store) {
                push_sample(st, tick);
            }

            /*
             * Assemble the bits into bytes, exactly as the oversampled decoder
             * does - but from samples the bus itself clocked rather than from
             * edges found in an oversampled stream.
             *
             * Only inside a frame: bits arriving with no START in front of them
             * are a capture that began mid-transaction, and counting them from
             * an arbitrary point would produce byte boundaries that are wrong by
             * a rotation. The framing timers set `in_frame`, so the byte counter
             * starts where the bus says the transaction does.
             */
            if ((frm.in_frame || frm.anchor_open) && !cap.decode_off) {
                /*
                 * A pending STOP closes the frame at the next byte boundary.
                 *
                 * It cannot close it immediately: the STOP is known the instant
                 * SDA rises, while the bits it terminates are still inside a
                 * shifter that emits whole words - so the last byte of the
                 * transaction arrives *after* its own STOP was detected, and
                 * closing on detection discarded it. Nor can the frame stay open
                 * until the bits run out, because whatever clocks the bus does
                 * next would assemble into bytes that were never in the
                 * transaction.
                 *
                 * Letting the byte in progress finish and closing on the
                 * boundary satisfies both: everything the STOP terminated is
                 * decoded, and nothing after it is.
                 */
                /*
                 * I3C gets its own decoder. The ninth bit is a T-bit rather
                 * than an acknowledge, CCCs change what the bytes after them
                 * mean, and ENTDAA has a shape no I2C transaction has - none of
                 * which the assembler below can express.
                 */
                bool i3c = (cap.proto == BB_PROTO_I3C);

                /*
                 * Apply the deferred framing, oldest first, and then decode
                 * this bit - do not consume it.
                 *
                 * Both branches used to `continue`, which threw the bit away.
                 * For a STOP immediately followed by a START - every
                 * back-to-back transaction - that is the first bit of the new
                 * frame, so its address decoded one bit late: 0x12 came out as
                 * 0x09. The frame events belong *between* bits, not instead of
                 * one.
                 */
                /*
                 * One predicate guards the whole framing block.
                 *
                 * These three tests run per *bit*, and two of them called
                 * dec_i3c_at_boundary() to do it - a function call every 80 ns
                 * at 12.5 MHz for a condition that is false almost always.
                 * Nothing is pending during the body of a transaction, which is
                 * where all the bits are.
                 */
                if (frm.pending != 0u) {
                /*
                 * Which of the two goes first, when both are waiting.
                 *
                 * STOP-then-START is right for an ordinary boundary and is what
                 * runs below. It is wrong for one case: a START noted strictly
                 * earlier than the STOP is the START of the very frame this
                 * STOP closes, so applying the STOP first hands it to a decoder
                 * whose frame was never opened. It is dropped as
                 * `no-frame-open` and the frame runs on into the next one - the
                 * seven-byte read that closes at eight bytes.
                 *
                 * Two earlier attempts at this failed for reasons worth
                 * keeping. Applying them in raw flag-arrival order took 1 to
                 * 6 MHz from decoding reads to decoding none, because "the
                 * START flag arrived first" is the ordinary case rather than
                 * the rare one. Then ordering them by cap.tick found nothing,
                 * because cap.tick advances per *word consumed* while this poll
                 * runs many times per word - so both stamps came out equal and
                 * every boundary looked simultaneous.
                 *
                 * Stamping with DWT->CYCCNT resolves the poll cadence instead
                 * of the word cadence, and the difference is the whole result:
                 * reorder counts are non-zero at 1.96, 4 and 6.06 MHz and zero
                 * at every rate that already decoded reliably. The condition
                 * fires on the failing case and only on it, which is what the
                 * flag-arrival version could not do.
                 *
                 * Signed difference, so counter wrap is a non-event.
                 */
                bool start_first =
                    frm.start_pending && frm.stop_pending &&
                    ((int32_t)(frm.start_at - frm.stop_at) < 0);


                if (frm.stop_pending && frm.start_pending &&
                    (i3c ? dec_i3c_at_boundary() : (dec.bit_count == 0u))) {
                    frm.bothpend_n++;
                    if (start_first) {
                        frm.reorder_n++;
                    }
                }
                if (frm.stop_pending && !start_first &&
                    (i3c ? dec_i3c_at_boundary() : (dec.bit_count == 0u))) {
                    if (i3c) {
                        frm.prev_read = dec_i3c_was_read();
                        dec_i3c_site(2u);
                        dec_i3c_stop();
                    } else {
                        ev_push(BB_EV_STOP, 0u, false);
                    }
                    frm.stop_pending    = false;
                    frm.opened_anchored = false;
                    /*
                     * Follow the decoder rather than assume. A STOP in the
                     * middle of a read is the bus turnaround, which dec_i3c
                     * suppresses and keeps its frame open for; clearing
                     * in_frame here regardless made the read's real STOP arrive
                     * at a layer that thought nothing was open, and it was
                     * dropped. See dec_i3c_in_frame().
                     */
                    frm.in_frame     = i3c ? dec_i3c_in_frame() : false;
                    frm.stop_idle    = 0u;
                    if (!i3c) {
                        frm.pending = frm.start_pending ? 1u : 0u;
                        continue;   /* the I2C assembler expects the old shape */
                    }
                    /* Only a frame that carried an address had a real end,
                     * and only a real end has a gap after it. */
                    if (frm.frame_bits >= 9u) {
                        /*
                         * One constant, not a rule about direction.
                         *
                         * Capturing one boundary type at a time says writes want
                         * 2 and reads want 3, at every rate and every payload
                         * length - so it looked like the skip should depend on
                         * the frame that just closed. Applying that made the
                         * ordinary mixed workload worse, not better: reads
                         * stopped decoding at 4 MHz and above.
                         *
                         * The two measurements are consistent if the dependency
                         * is on the frame about to *start* rather than the one
                         * that just ended - a single-boundary-type capture
                         * cannot tell those apart, and the mixed one gives them
                         * opposite answers. The direction of a frame that has
                         * not begun is not knowable here, so this stays a
                         * constant until the mechanism is understood rather than
                         * curve-fitted. SNIFF:GAP makes it a one-line change to
                         * test against a different bus.
                         */
                        frm.skip_bits = s_gap_bits;
                    }
                    /* What the decoder actually consumed for this frame,
                     * against the 27 or 63 bits the wire carries. */
                    frm.frame_last = (uint16_t)frm.frame_bits;
                    frm.frame_bits = 0u;
                    frm.pending    = (frm.skip_bits || frm.start_pending)
                                         ? 1u : 0u;
                }
                if (frm.skip_bits != 0u) {
                    if (--frm.skip_bits == 0u) {
                        /* The gap is behind us, so the next bit starts a
                         * frame. Keep the framing block live to open it. */
                        frm.anchor_open = true;
                        frm.pending     = 1u;
                    }
                    continue;       /* a clock edge, not a bit */
                }

                /*
                 * Open the frame here, a fixed distance from the last one's
                 * end, instead of whenever the START flag is applied.
                 */
                if (i3c && frm.anchor_open) {
                    frm.anchor_open     = false;
                    frm.opened_anchored = true;
                    if (!dec_i3c_in_frame()) {
                        dec_i3c_start(false);
                    }
                    frm.in_frame      = true;
                    frm.start_pending = false;
                    frm.frame_bits    = 0u;
                    frm.defer_last    = frm.defer_bits;
                    frm.arm_last      = frm.arm_bits;
                    frm.defer_bits    = 0u;
                    frm.arm_bits      = 0u;
                    frm.pending       = frm.stop_pending ? 1u : 0u;
                    dec.expect_addr   = true;
                    dec.bit_count     = 0u;
                    dec.byte          = 0u;
                }
                if (i3c && frm.start_pending && frm.opened_anchored) {
                    /* Already open, and opened from a better anchor than this
                     * flag. Consume it rather than restart the frame. */
                    frm.start_pending = false;
                    frm.pending       = (frm.stop_pending || frm.skip_bits)
                                            ? 1u : 0u;
                } else if (i3c && frm.start_pending && dec_i3c_at_boundary()) {
                    frm.opened_anchored = false;
                    dec_i3c_start(frm.start_repeated);
                    frm.start_pending = false;
                    frm.frame_bits    = 0u;
                    frm.defer_last    = frm.defer_bits;
                    frm.arm_last      = frm.arm_bits;
                    frm.defer_bits    = 0u;
                    frm.arm_bits      = 0u;
                    frm.pending       = (frm.stop_pending || frm.skip_bits)
                                            ? 1u : 0u;
                    dec.expect_addr   = true;
                    dec.bit_count     = 0u;
                    dec.byte          = 0u;
                }
                }
                if (i3c) {
                    if (frm.stop_pending) {
                        frm.defer_bits++;
                    } else if (frm.start_pending || frm.skip_bits) {
                        frm.arm_bits++;
                    }
                    frm.frame_bits++;
                    dec_i3c_bit((uint8_t)(st & 1u));
                } else if (dec.bit_count < 8u) {
                    dec.byte = (uint8_t)((dec.byte << 1) | (st & 1u));
                    dec.bit_count++;
                } else {
                    ev_push(dec.expect_addr ? BB_EV_ADDR : BB_EV_DATA,
                            dec.byte, (st & 1u) == 0u);
                    dec.expect_addr = false;
                    dec.bit_count   = 0u;
                    dec.byte        = 0u;
                }
            }
            continue;
        }

        if (!cap.have_initial) {
            /*
             * Contract 3: the first record is the initial state, not a
             * transition. The very first sample establishes it and is not
             * emitted as an edge - otherwise a bus that was already idle high
             * would appear to have risen at t=0, which is precisely the class of
             * phantom edge the STM32 build produced.
             */
            last             = st;
            cap.last_tick    = tick;
            cap.have_initial = true;
            /* Same sample, same purpose: it is the initial state for the
             * decoder too, so neither path has to treat it as an edge. */
            dec_seed(st);
            continue;
        }
        /*
         * The decoder sees transitions, not every sample.
         *
         * An earlier version fed it every sample, on the stated grounds that a
         * state machine over levels needs each level to know a clock is still
         * high. That reasoning was wrong: every condition this decoder tests is
         * an *edge* - SDA moving while SCL is high, SCL rising - and two
         * successive transitions carry exactly the same information as the run
         * of identical samples between them.
         *
         * It was also unaffordable. At 6 MHz the per-sample budget is 166 ns, and
         * consuming two channels plus a decode step does not fit in it - so the
         * intake could never catch up, and a self test that took 38 ms without
         * the decoder took over eight seconds with it. Transitions make it
         * roughly five orders of magnitude cheaper for the same result.
         */
        if (st != last) {
            /* Which channels moved, not just that something did. The locals are
             * the live copy for the whole of this function; see the hoist at the
             * top. Writing the struct here instead would be lost at the
             * write-back. */
            changed |= (st ^ last);
            last     = st;
            if ((cap.proto == BB_PROTO_I2C) || (cap.proto == BB_PROTO_I3C)) {
                if (!cap.decode_off) {
                    dec_sample(st, tick);
                }
            }
            push_sample(st, tick);
        } else {
            cap.coincident++;
        }
    }
    cap.tick         = tick;
    cap.last_state   = last;
    cap.changed_mask = changed;
    s_prof.cyc_word += (uint32_t)(DWT->CYCCNT - t0);
    s_prof.words++;
}

/*
 * Called from the main loop. Drains whatever the shifter has and notices when
 * sampling has outrun this loop.
 */
/*
 * The framing timers, drained once per poll.
 *
 * Lifted out of cap_poll() unchanged so the DMA intake can call it too: that
 * path reads no FlexIO shifter registers at all, but it still needs START and
 * STOP, which only these timers report.
 */
static void cap_frame_poll(FLEXIO_Type *base)
{
    if (cap_has_frame()) {
    /*
         * Framing: the timers say *when* SDA moved and which way; software says
         * what SCL was doing at that moment.
         *
         * The hardware-only version got as far as a reliable START and never a
         * STOP. Both framing timers count SDA edges correctly - that is measured
         * - and the shifter that samples SCL alongside them returns the right
         * level for a fall and never for a rise, with no configuration of
         * polarity, compare, load timing or bit position changing it. Rather
         * than keep searching that space, the level is read directly from
         * FlexIO's pin register when the edge flag comes up.
         *
         * The cost is honest and bounded: this is a *software* qualification, so
         * it is only correct while the clock line still holds the level it had at
         * the edge. START and STOP hold SCL high until the bus moves again, which
         * at ordinary bus rates is many thousands of poll iterations, and at
         * 12.5 MHz I3C push-pull is about 40 ns - far below what polling can
         * resolve. So framing is reliable at moderate rates and is not at full
         * rate, which is why `SNIFF:STATUS` reports it and why the *bit* capture
         * - which is hardware-exact at any rate the bus can run - is the part
         * this mode exists for.
         */
        uint32_t tf = FLEXIO_GetTimerStatusFlags(base);

        /*
         * No software SCL read any more. The timer only enables on an SDA edge
         * that occurred while SCL was high, so a flag already means "qualified"
         * - the check that used to live here raced the bus and lost above about
         * 1 MHz, turning ordinary data edges into a stream of phantom restarts.
         *
         * The s_frame_hi counters are kept and now equal s_frame_words by
         * construction. They stay because a divergence would mean the hardware
         * qualification is not doing what this comment claims.
         */
        /*
         * The STOP detector is read first, and the order is the point.
         *
         * Both flags come out of one TIMSTAT snapshot, so when a transaction
         * ends and the next begins between two polls, both are set and whichever
         * block runs first is treated as having happened first. The START block
         * ran first, so an ordinary boundary looked like "a START that preceded
         * its STOP" - which is impossible on the wire, and which made a STOP
         * arrive at a decoder whose frame had not been opened yet. It was then
         * dropped as `no-frame-open` and the frame it should have closed ran on
         * into the next one: a seven-byte read closing at eight bytes.
         *
         * A STOP always precedes the START after it. Reading it first costs
         * nothing and makes the common case come out in the order it happened.
         */
        if ((tf & (1uL << CAP_FRAME_TIMER)) != 0u) {
            FLEXIO_ClearTimerStatusFlags(base, 1uL << CAP_FRAME_TIMER);
            s_frame_words[0]++;
            if (frm.in_frame) {             /* SDA rose while SCL was held */
                /*
                 * Noted, not emitted yet.
                 *
                 * The two channels have very different latencies: a framing
                 * event is a timer flag and is known at once, while data bits
                 * arrive only when a shifter fills, 32 at a time. Pushing the
                 * STOP the moment it is detected therefore puts it *ahead* of
                 * the last bytes of its own frame in the event stream - and
                 * worse, closes the frame, so the byte assembler discarded them:
                 * a four-byte transaction decoded as three with the STOP in the
                 * middle. It is held until the data shifter has drained.
                 */
                s_frame_hi[0]++;
                /*
                 * Applying these in flag-arrival order was tried and is wrong.
                 *
                 * The window is real: frm.in_frame is set when the START *flag*
                 * is seen while the decoder's own in_frame is set only when that
                 * START is *applied*, and a STOP arriving between the two is
                 * dropped as `no-frame-open`, leaving a frame that runs on - a
                 * seven-byte read closing at eight.
                 *
                 * But "the START flag arrived first" is the ordinary case, not
                 * the rare one: a deferred STOP cannot be applied until the next
                 * frame's clocking delivers the previous frame's stranded tail,
                 * so the next START is nearly always already flagged by then.
                 * Acting on it took 1 to 6 MHz from decoding reads to decoding
                 * none.
                 */
                /*
                 * Cancelling a pending START here was tried and is wrong.
                 *
                 * The rule "a STOP flag cancels a START that has not been
                 * applied" rejects all six of a run's setup falls and looked
                 * like the answer. It is not, because a read raises a STOP-shaped
                 * edge at its *turnaround* too - the target takes the bus after
                 * the address is acknowledged - and that edge arrives while the
                 * read's own START may still be waiting for a byte boundary. The
                 * rule then cancels a perfectly real START, and 4 to 8 MHz stop
                 * decoding reads entirely.
                 *
                 * The turnaround and the setup fall cannot be told apart at flag
                 * time; only the decoder, which knows how many bytes the frame
                 * has produced, can tell them apart at all. See dec_i3c_stop().
                 */
                frm.stop_pending = true;
                /*
                 * Stamped with the cycle counter, not the sample tick.
                 *
                 * The first attempt used cap.tick and concluded the ordering
                 * was unrecoverable: the two stamps differed by exactly 0, 8 or
                 * 16 across every rate, which are word boundaries. That was a
                 * property of the clock, not of the events. cap.tick advances
                 * only as words are *consumed*, while this poll runs many times
                 * per word - so every flag seen between two words got the same
                 * number, and two events in that span looked simultaneous when
                 * they were not.
                 *
                 * DWT->CYCCNT is free-running at the core clock, is already
                 * read by the profiling in cap_poll, and costs one register
                 * read. It resolves the poll cadence rather than the word
                 * cadence, which is the resolution the flags actually have.
                 *
                 * Compared with a signed difference so counter wrap is a
                 * non-event rather than a once-every-five-seconds bug.
                 *
                 * Superseded note, kept because it is the wrong conclusion this
                 * replaced: the tick a framing flag is stamped with was a word
                 * boundary, not the event.
                 *
                 * This was added to order START against STOP instead of
                 * inferring the order from which block happened to run. It
                 * cannot: cap.tick advances per bit inside the word loop, but
                 * this runs once per word, after the loop. Measured across all
                 * seven rates, stop_at and start_at differ by exactly 0, 8 or
                 * 16 - the word quantum - and at four of the seven rates they
                 * are identical. Two flags raised inside one word carry no
                 * relative time, because the FlexIO timer status bits are
                 * sticky and are only read between words.
                 *
                 * So the order below is settled by physics rather than by
                 * timestamp, and it is not a guess: a START seen in the same
                 * word as a STOP must follow it, because the STOP belongs to
                 * the frame that was already open. The fields are kept because
                 * they are what showed the resolution was insufficient, and
                 * they will show it again if the intake ever changes.
                 */
                frm.stop_at      = DWT->CYCCNT;
                frm.stop_tick_at = (uint32_t)cap.tick;
                frm.pending      = 1u;
            }
        }
        if ((tf & (1uL << (CAP_FRAME_TIMER + 1u))) != 0u) {
            FLEXIO_ClearTimerStatusFlags(base, 1uL << (CAP_FRAME_TIMER + 1u));
            s_frame_words[1]++;
            {                               /* SDA fell while SCL was held */
                s_frame_hi[1]++;
                /*
                 * Nothing is reset here, and that is a settled limitation.
                 *
                 * A pin-clocked receive shifter emits nothing until its word
                 * fills, so a frame ending part-way through a word strands
                 * those bits inside it. They are not merely lost: they sit at
                 * the front of the next frame's first word, so every byte of
                 * the next transaction is rotated by however many were left.
                 * Three identical writes decode as
                 * `S A12+ D00+ Sr P S A09+ D00-` - the first frame right, the
                 * second shifted.
                 *
                 * Four ways of fixing it were measured and none works:
                 *
                 *   clear SMOD at the START       alignment unchanged
                 *   full SetShifterConfig         alignment unchanged
                 *   flush at the STOP by running  second frame differently
                 *     the shift clock free        wrong, not right
                 *   one bit per word              correct by construction and
                 *                                 eight times the register
                 *                                 traffic - overruns from
                 *                                 1.96 MHz, against 2.9 MHz
                 *                                 exact in oversampled mode
                 *
                 * The first three share a cause: this flag is seen microseconds
                 * after the event, by which time bits of the new frame are
                 * already in the shifter, so anything done here acts on live
                 * data. A fix has to reset the shifter from the START detector
                 * in hardware, and on this part a trigger-gated timer does not
                 * decrement on its pin input at all - see cap_has_frame().
                 *
                 * Use oversampled capture where exactness matters; it is exact
                 * frame after frame to 2.9 MHz. See docs/STATUS.md section 4.1.
                 */
                if (cap.proto == BB_PROTO_I3C) {
                    /* Noted, not applied: the previous frame's tail is still
                     * ahead of this point in the bit stream. See frm. */
                    /* Repeated only if no STOP stands between: at flag time
                     * in_frame is still true for the frame the pending STOP is
                     * about to close, and taking it at face value labelled
                     * every ordinary START as a restart. */
                    /*
                     * The same-poll half: the STOP detector is read first, so
                     * when both edges land in one poll the STOP is already
                     * pending by the time this runs and the falling edge in
                     * front of it is that STOP's setup.
                     */
                    {
                    /*
                     * Held, not pending.
                     *
                     * Every I3C read ends with its STOP being set up - SDA is
                     * high after a T bit that always reads 1, so it must be
                     * pulled down before it can be released, and that fall
                     * while SCL is held is exactly what this detector finds.
                     * Six reads produced six extra STARTs against thirteen
                     * correct STOPs, at every rate, and an oversampled capture
                     * of the wire shows the edge plainly.
                     *
                     * Such a START cannot be told from a real one when it is
                     * seen - the two are the same edge, the same distance from
                     * the transaction boundary, and three different guards
                     * tried at three different sites each caught about half.
                     * What separates them is only what happens next: a real
                     * START is followed by a frame, a setup fall by its STOP
                     * one bus clock later. So the decision waits for that.
                     */
                    frm.start_repeated = frm.in_frame && !frm.stop_pending;
                    frm.start_pending  = true;
                    frm.start_at       = DWT->CYCCNT;
                    frm.start_tick     = (uint32_t)cap.tick;
                    frm.pending        = 1u;
                    }
                } else {
                    ev_push(frm.in_frame ? BB_EV_RESTART : BB_EV_START, 0u, false);
                    dec.expect_addr = true;
                    dec.bit_count   = 0u;
                    dec.byte        = 0u;
                }
                frm.in_frame = true;
            }
        }
    }

    /*
     * A deferred STOP is released once the data shifters hold nothing more, so
     * it lands after the bytes it terminates rather than before them.
     */
    if (frm.stop_pending) {
        /*
         * Released only once the data has genuinely stopped arriving, not the
         * first time the shifter happens to be empty.
         *
         * A data shifter is empty between every pair of words, so releasing on
         * the first empty poll closes the frame in the middle of it - the bytes
         * in the *next* word are then discarded, which cost the fourth byte of a
         * four-byte transaction. Requiring several consecutive empty polls
         * distinguishes "between words" from "no more words", which is the
         * distinction that matters and the only one available without knowing
         * how many bits were in flight when the STOP arrived.
         */
        /*
         * "Has any bit arrived since last time" is the test, not "is the shifter
         * empty right now". A shifter is empty between every pair of words, and
         * during a run of clocks those gaps are long enough to look like the end
         * of the data - which released the STOP mid-transaction and cost the
         * byte that straddled the word boundary. The sample tick only advances
         * when bits are actually consumed, so it distinguishes a gap from a
         * stop.
         */
        if (cap.tick != frm.stop_tick) {
            frm.stop_tick = cap.tick;
            frm.stop_idle = 0u;
        } else if (++frm.stop_idle >= 20000u) {
            /*
             * The idle release has to close the *decoder's* frame too.
             *
             * This path exists for the STOP that ends the last frame of a
             * burst, where no further clocking will ever arrive to reach the
             * byte boundary the normal release waits for. It pushed a raw
             * BB_EV_STOP and cleared the local flags - which is the whole story
             * for I2C, and only half of it for I3C, whose decoder keeps its own
             * `in_frame`. That decoder was never told, so it held the frame
             * open and the next transaction's bytes were appended to it.
             *
             * That is what merged the frames: thirteen transactions decoded as
             * six or seven, with `closes=7` against `hwstop=13`. Every STOP the
             * hardware saw was correct and about half of them never reached the
             * decoder. It is also why a frame ending mid-unit was the one that
             * went missing - reaching a 9-bit boundary is exactly the condition
             * this path exists to bypass.
             */
            bool i3c_idle = (cap.proto == BB_PROTO_I3C);

            if (i3c_idle) {
                frm.prev_read = dec_i3c_was_read();
                dec_i3c_site(2u);
                dec_i3c_stop();
            } else {
                ev_push(BB_EV_STOP, 0u, false);
            }
            /* Same bookkeeping the boundary release does, for the same
             * reasons: only a frame that carried an address had a real end,
             * and only a real end has a gap after it. */
            if (frm.frame_bits >= 9u) {
                frm.skip_bits = s_gap_bits;
            }
            frm.frame_last   = (uint16_t)frm.frame_bits;
            frm.frame_bits   = 0u;
            frm.stop_pending    = false;
            frm.opened_anchored = false;
            frm.in_frame        = i3c_idle ? dec_i3c_in_frame() : false;
            frm.stop_idle    = 0u;
        }
    }

}

static void cap_poll(void)
{
    FLEXIO_Type *base;
    uint32_t     mask;
    unsigned     budget = 64u;
    uint32_t     t0;

    if (!cap.running) {
        return;
    }
    t0 = DWT->CYCCNT;
    s_prof.polls++;
    base = cap_hw();
    mask = cap_shifter_mask();

    /*
     * The idle poll reads one register, not three.
     *
     * Measured: a poll that drains nothing cost about 220 cycles, and at
     * 12.5 MHz a shifter word arrives every 640 ns - roughly 500 cycles. That
     * idle cost sits directly between "a word became ready" and "the word was
     * read", because a poll already in flight has to finish first. Three
     * peripheral reads of a FlexIO that is not on the core's own bus is most of
     * it.
     *
     * Only the shifter status has to be read every time; it is the one that
     * says whether there is work. The error flags and the framing timers latch
     * until they are cleared, so nothing is lost by looking at them when there
     * is a word to drain - which is whenever the bus is clocking - or, as a
     * safety net for a STOP that ends the last frame of a burst, every
     * sixteenth idle pass.
     */
    if (s_dma_on) {
        /*
         * The engine has already moved the words. Nothing here touches FlexIO
         * except the framing timers, and those are read once per poll rather
         * than once per word.
         */
        uint32_t avail = cap_dma_avail();

        /*
         * How deep the DMA queue actually gets under traffic.
         *
         * words-per-poll does not answer this: the poll loop runs on an idle
         * bus too, and idle polls dominate the ratio. The high-water mark is
         * the honest measure of how far the consumed bits lag the live framing
         * flags, which is what decides whether reading those flags against
         * queued data means anything.
         */
        if (avail > s_dma_avail_max) {
            s_dma_avail_max = avail;
        }


        /*
         * Framing once per poll, every poll: the cadence the polled intake has.
         * Per-word was tried and changed nothing measurable, because a poll at
         * these rates finds about one word anyway.
         */
        cap_frame_poll(base);

        if (avail == 0u) {
            s_prof.cyc_poll += (uint32_t)(DWT->CYCCNT - t0);
            return;
        }
        while ((avail-- > 0u) && (budget-- > 0u)) {
            uint32_t words[1];

            words[0] = s_dma_buf[s_dma_rd];
            s_dma_rd = (s_dma_rd + 1u) & (CAP_DMA_WORDS - 1u);
            cap_consume(words);
            if (!cap.running) {
                break;
            }
        }
        s_prof.cyc_poll += (uint32_t)(DWT->CYCCNT - t0);
        return;
    }

    if (((FLEXIO_GetShifterStatusFlags(base) & mask) != mask) &&
        ((++s_idle_pass & 0x0Fu) != 0u)) {
        s_prof.cyc_poll += (uint32_t)(DWT->CYCCNT - t0);
        return;
    }

    if ((FLEXIO_GetShifterErrorFlags(base) & mask) != 0u) {
        /* A word was completed before the previous one was read: samples were
         * lost. Recorded as an overrun gap rather than stitched over, because a
         * capture with an unmarked discontinuity is worse than one with a hole
         * in it. */
        FLEXIO_ClearShifterErrorFlags(base, mask);
        cap.overruns++;
        cap.gaps++;
    }

    /*
     * All tap shifters or none. They share the timer, so they fill together;
     * reading them in one pass is what keeps the channels on the same sample.
     * Draining one shifter at a time would let the channels slide apart by a
     * word, which produces a capture that looks plausible and has the channels
     * skewed in time.
     *
     * Bounded so a fast bus cannot starve the mailbox - the controller core has
     * to be able to ask this one to stop.
     */
    /*
     * The framing shifter drains on its own schedule, and must.
     *
     * The all-or-none rule below exists because the data shifters share one
     * timer and fill together, so reading them in one pass keeps the channels on
     * the same sample. The framing shifter is in a different clock domain
     * entirely - it advances on SDA edges, not SCL - so it fills at a completely
     * unrelated rate, and waiting for it to be ready alongside the others would
     * stall both. It is a separate event stream, not another channel of the same
     * instant, and it is drained as one.
     */
    cap_frame_poll(base);

    while ((budget-- > 0u) &&
           ((FLEXIO_GetShifterStatusFlags(base) & mask) == mask)) {
        uint32_t words[8];

        /*
         * Let the last sample settle before reading the word.
         *
         * The status flag asserts as the shifter loads SHIFTBUF, and reading it
         * in the very next instruction occasionally caught the final bit before
         * it had propagated: across 159 pin-clocked bits of a known pattern,
         * exactly two were wrong, both at bit 31 of their word, and both a 1 read
         * as 0. Never a 0 read as 1, and never a bit anywhere else in the word -
         * which is the signature of a read racing a settle rather than of lost or
         * misaligned samples.
         *
         * A barrier plus a few cycles costs nothing next to the word period and
         * is far cheaper than the alternative, which is a capture that is right
         * about thirty-one bits in thirty-two and therefore believable when it is
         * wrong.
         */
        __DSB();
        for (volatile unsigned d = 0; d < 4u; d++) { }

        for (unsigned i = 0, n = cap_shifter_count(); i < n; i++) {
            words[i] = base->SHIFTBUF[i];   /* the read clears the flag */
        }
        cap_consume(words);
        if (!cap.running) {
            break;              /* the ring filled inside cap_consume */
        }
    }
    {
        uint32_t d = (uint32_t)(DWT->CYCCNT - t0);

        s_prof.cyc_poll += d;
        if (d > s_prof.cyc_max) {
            s_prof.cyc_max = d;
        }
    }
}

static bb_cap_err_t cap_start(void)
{
    /*
     * Starting an already-running capture restarts it rather than refusing.
     *
     * Refusing was the first behaviour and it reads as the careful choice, but
     * it is wrong for what the command means: a host asking to start a capture
     * wants a capture running from now, and it has no way to know whether some
     * earlier command left one going. The host library has no stop-then-start
     * idiom either - bb_sniff_start() just starts - so BUSY made a legitimate
     * request fail for a reason the caller could not have avoided.
     *
     * Everything a restart has to reset is already reset below, which is what
     * makes this safe rather than merely convenient.
     */
    if (cap.running) {
        cap_hw_stop();
    }
    if (cap.tap_count == 0u && cap.proto != BB_PROTO_CAN) {
        return BB_CAP_ERR_PINS;
    }
    if (cap.proto == BB_PROTO_CAN) {
        /* FlexCAN listen-only on the controller core captures CAN. Two engines
         * for one bus could disagree, so this one declines it. */
        return BB_CAP_ERR_UNSUPPORTED;
    }

    bb_cap_err_t rc = flexio_program();
    if (rc != BB_CAP_OK) {
        return rc;
    }

    s_write = s_read = 0u;
    s_wr_idx = 0u;
    cap.entries = cap.records = cap.coincident = 0u;
    cap.gaps = cap.overruns = 0u;
    cap.changed_mask = 0u;
    dec_reset();
    dec_i3c_init(ev_push);   /* same emit path; a different state machine */
    frame_reset();
    cap.tick = cap.last_tick = 0u;
    cap.base_ticks   = 0u;
    cap.full         = false;
    cap.have_initial = false;
    cap.meta_pending = true;

    prof_init();
    if (s_dma_want && (cap.mode == (uint8_t)BB_CAP_PINCLOCK) &&
        (cap_shifter_count() == 1u)) {
        (void)cap_dma_start(cap_hw());
    }
    FLEXIO_Enable(cap_hw(), true);
    cap.running = true;
    return BB_CAP_OK;
}

static bb_cap_err_t cap_stop(void)
{
    if (!cap.running) {
        return BB_CAP_ERR_NOT_RUNNING;
    }
    /* Drain what the shifter already holds before switching it off, so the
     * last word captured is not thrown away by the act of stopping. */
    cap_poll();
    cap_hw_stop();
    return BB_CAP_OK;
}

/* ---- self test and bench ----------------------------------------------- */
/*
 * The generator drives the same pin window the sampler reads.
 *
 * That is the whole trick, and it is what makes a self test possible on this
 * board with no jumper wires: a transmit shifter configured for the same
 * parallel window drives those pads, and the receive shifter sees the pad state
 * because the pads are muxed to FlexIO with SION. Zero wires, and the path under
 * test is the real capture path rather than a simulation of it.
 *
 * It also avoids the STM32 build's specific failure. There the generator and the
 * capture shared a core, so asking for a shorter bit period simply made the
 * generator slower - the test "passed" at a nominal 3.3 MHz while never
 * exceeding 202 kHz, and reported the nominal figure. Here the generator is
 * hardware and its rate is set by a divider, so it cannot be throttled by the
 * cost of capturing it. `from_selftest` is still reported, because the number
 * comes from our own generator rather than a real bus, and a caller is entitled
 * to know which kind of figure they are reading.
 *
 * One honest limit: generator and sampler are the same peripheral, so a shared
 * misunderstanding of the parallel slot order would cancel out and the test
 * would still pass. It proves the engine captures what is driven, at the rate
 * claimed, in the right order *relative to the generator* - not that
 * CAP_OLDEST_IS_HIGH_SLOT matches a third party's bus. Only a real bus settles
 * that, which is why the constant is named rather than buried.
 */

/*
 * Drive and read each tap pad directly, using FlexIO's own pin registers.
 *
 * PINOUTE enables a software output on a pin and PINOUTD sets its level, both
 * independent of any shifter; PIN reads the live input. So this answers one
 * question per pad - "can FlexIO drive this pin and see the result" - and it is
 * the question the whole-channel self test cannot answer.
 *
 * A pad that fails here is not muxed to FlexIO (or is held by something else).
 * A pad that passes here while its channel never moves in the capture means the
 * pad is fine and the shifter is not driving that member of its parallel group,
 * which is a completely different fix.
 *
 * The delays are for the pad, not the register: an open-drain pad with only a
 * weak internal pull needs time to actually reach the level before the read
 * means anything.
 */
static void probe_pins(bb_selftest_t *out)
{
    FLEXIO_Type *base = cap_hw();

    out->pin_probe = 0u;
    for (unsigned i = 0; i < cap.tap_count; i++) {
        uint32_t bit = 1uL << cap.pins[i];
        bool     hi;
        bool     lo;

        base->PINOUTE |= bit;

        base->PINOUTD |= bit;
        for (volatile unsigned d = 0; d < 2000u; d++) { }
        hi = (FLEXIO_ReadPinInput(base) & bit) != 0u;

        base->PINOUTD &= ~bit;
        for (volatile unsigned d = 0; d < 2000u; d++) { }
        lo = (FLEXIO_ReadPinInput(base) & bit) == 0u;

        /* Released, so the probe cannot leave a tap driven into a live bus. */
        base->PINOUTE &= ~bit;

        if (hi && lo) {
            out->pin_probe |= (1uL << i);
        }
    }
}

/* Every tap pin, as a FlexIO software-output mask. */
static uint32_t cap_tap_pin_mask(void)
{
    uint32_t m = 0u;

    for (unsigned i = 0; i < cap.tap_count; i++) {
        m |= 1uL << cap.pins[i];
    }
    return m;
}

/*
 * The generator: this core toggling every tap pin together, through FlexIO's
 * software pin output.
 *
 * Not a transmit shifter, and that is deliberate twice over. A transmit shifter
 * drives the single pin at its pin select, and with a shifter per tap for
 * capture there is no spare shifter per channel to pair with it - so a shifter
 * generator could only ever drive one tap. Driving one tap is exactly the blind
 * spot that let a dead second channel pass as success for several rounds.
 *
 * Toggling all taps together means every channel must move, which makes the
 * per-channel result a real check rather than a formality.
 *
 * The cost is honest and worth stating: the rate is a software delay loop, not a
 * clock divider, so it is approximate. That is why nothing here claims a bus
 * frequency - the entries and the capture's own timestamps carry the rate that
 * was actually achieved.
 */
static uint32_t gen_toggle(uint32_t edges, uint32_t half_ns)
{
    FLEXIO_Type *base = cap_hw();
    uint32_t     mask = cap_tap_pin_mask();
    uint32_t     settle;
    uint32_t     n;

    /* Roughly half_ns of delay. The CM7 runs from the ARM PLL by the time this
     * executes and a volatile loop iteration is a few cycles, so this is an
     * order-of-magnitude control rather than a calibrated one - deliberately
     * not dressed up as a frequency. */
    settle = half_ns / 8u;
    if (settle == 0u) {
        settle = 1u;
    }

    base->PINOUTE |= mask;
    for (n = 0; n < edges; n++) {
        base->PINOUTD ^= mask;

        /*
         * The wait between edges services the intake instead of idling.
         *
         * The first version spun a volatile counter and polled once per edge.
         * At a slow edge rate that starves the sampler: a 20 us half period is
         * about 12 us of doing nothing, while the shifters complete a word every
         * 5.3 us at 6 MHz - so they overran, the self test failed on
         * `overruns != 0`, and it looked like the engine could not keep up when
         * in fact the test was refusing to collect from it.
         *
         * Polling here also makes the delay depend on what cap_poll() costs,
         * which is why the rate this produces is approximate and why nothing
         * reports it as a bus frequency.
         */
        for (uint32_t d = 0; d < settle; d++) {
            cap_poll();
            if (!cap.running) {
                break;
            }
        }
        if (!cap.running) {
            n++;
            break;
        }
    }
    /* Released, so the self test cannot leave the taps driven. */
    base->PINOUTE &= ~mask;
    base->PINOUTD &= ~mask;
    return n;
}

/*
 * An I2C frame, bit-banged onto the clock and data taps.
 *
 * A square wave was enough to prove the sampler saw both channels, and is no use
 * at all for proving the decoder works - so for a protocol that has a decoder
 * the generator drives something the decoder can decode: START, an address
 * byte, three data bytes each acknowledged, STOP.
 *
 * That is also the waveform the wire protocol's own conformance expectation is
 * written against - one address, three data, one stop, opening with a START -
 * so a passing self test now means the whole path works end to end: pads,
 * sampler, intake, decoder and the token rendering on the other core.
 *
 * The generator drives the acknowledgement bits itself, because it is driving
 * both lines and there is no target on the bus. So `+` here means "the generator
 * asserted ack", not "something answered" - worth knowing before reading a self
 * test as evidence about a device.
 */
static uint32_t gen_edge_count;

static void gen_set(uint32_t clk_bit, uint32_t dat_bit,
                    bool clk, bool dat, uint32_t settle)
{
    FLEXIO_Type *base = cap_hw();
    uint32_t     v    = base->PINOUTD & ~(clk_bit | dat_bit);

    if (clk) { v |= clk_bit; }
    if (dat) { v |= dat_bit; }
    base->PINOUTD = v;
    gen_edge_count++;

    /* The wait services the intake rather than idling - see gen_toggle(). */
    for (uint32_t d = 0; d < settle; d++) {
        cap_poll();
        if (!cap.running) {
            return;
        }
    }
}

static void gen_i2c_byte(uint32_t clk_bit, uint32_t dat_bit,
                         uint8_t byte, uint32_t settle)
{
    for (int b = 7; b >= 0; b--) {
        bool bit = ((byte >> b) & 1u) != 0u;

        /* Data changes while the clock is low, and is sampled on the rising
         * edge. Setting it the other way round is how you get a decode that is
         * off by one bit - the exact defect this whole engine exists to avoid. */
        gen_set(clk_bit, dat_bit, false, bit, settle);
        gen_set(clk_bit, dat_bit, true,  bit, settle);
    }
    /* The ninth bit, driven low: acknowledged. */
    gen_set(clk_bit, dat_bit, false, false, settle);
    gen_set(clk_bit, dat_bit, true,  false, settle);
}

static uint32_t gen_i2c_frame(uint32_t half_ns)
{
    FLEXIO_Type   *base    = cap_hw();
    uint32_t       clk_bit = 1uL << cap.pins[0];
    uint32_t       dat_bit = 1uL << cap.pins[1];
    uint32_t       settle  = half_ns / 8u;
    static const uint8_t payload[3] = { 0x11u, 0x22u, 0x33u };

    if (settle == 0u) {
        settle = 1u;
    }
    gen_edge_count = 0u;

    beacon->phase = 0x10u;

    /*
     * Idle level first, output enable second. The other order glitches.
     *
     * PINOUTD holds whatever it held last - zero after a reset - so enabling the
     * output before setting the level drives both lines low for as long as it
     * takes to get to the next write. The sampler sees that, and a decoder
     * reading it gets a spurious edge before the frame even starts: the first
     * attempt produced `S P S` ahead of the address and then decoded the address
     * as 0x63 instead of 0xA0, because the bit counter had already been
     * desynchronised by edges that were not part of any transaction.
     */
    base->PINOUTD |= (clk_bit | dat_bit);
    base->PINOUTE |= (clk_bit | dat_bit);

    /* Settle at idle, both lines high, before anything is decoded. */
    gen_set(clk_bit, dat_bit, true, true, settle);
    gen_set(clk_bit, dat_bit, true, true, settle);

    /*
     * Start the decoder from here, so the stream contains the frame and nothing
     * else. Everything before this point - enabling the output, the pads
     * settling, whatever the previous test left them doing - is setup, and a
     * host reading SNIFF:READ? after a self test should see the transaction that
     * was driven, not the noise around it.
     */
    /*
     * Start the record at the frame, not at the setup.
     *
     * Enabling a pad output is itself an edge - the pin leaves whatever level a
     * floating input was reading - and in pin-clocked mode a shifter cannot tell
     * that edge from a bus clock. The first captured frame was bit-perfect and
     * shifted by one, with a spurious sample at the front, for exactly that
     * reason.
     *
     * Driving the taps to idle *before* cap_start() does not fix it: cap_start()
     * calls flexio_program(), which runs FLEXIO_Init() and resets the pin
     * registers, so the enable necessarily happens after the capture is live. I
     * tried that first and it added an edge rather than removing one.
     *
     * So the record is reset here instead, at the point where setup ends and the
     * transaction begins - the same place and the same reasoning as dec_reset()
     * below, which has been doing this for the decoder all along.
     */
    /*
     * The shifters are restarted, not just the counters.
     *
     * Zeroing the ring is not enough on its own, and the first attempt at this
     * proved it: the bits from the setup edges were already *inside* the
     * shifter, so the 32-bit word boundary stayed offset by exactly as many
     * edges as the setup had produced, and the capture came back shifted no
     * matter how the software counters were reset. Disabling and re-enabling
     * FlexIO discards the partial word and restarts the timer, so the next
     * rising edge is bit 0 of a fresh word.
     *
     * The *timer* has to be reset too, and that is the part that took three
     * attempts. It counts clock edges and loads SHIFTBUF when it reaches its
     * compare, so a timer that has already counted two edges needs only thirty
     * more - and delivers a word of two stale bits followed by thirty real ones.
     * Clearing the shifters alone left the capture bit-perfect and offset by
     * exactly two, which looks like a framing bug and is really an arithmetic
     * one. flexio_program() reprograms both.
     *
     * Done while the lines are idle and nothing is clocking, which is the only
     * moment where dropping a partial word is guaranteed to drop nothing real.
     */
    FLEXIO_Enable(base, false);
    (void)flexio_program();              /* timer *and* shifters, both reset */

    /*
     * Re-drive the taps while FlexIO is still disabled.
     *
     * flexio_program() runs FLEXIO_Init(), which clears the pin output
     * registers - so the generator loses its drive and, left there, produces no
     * clock at all: the next run captured nothing but a metadata record. Putting
     * the outputs back is therefore required, and doing it *before* the module
     * is re-enabled is what makes the re-drive edge free. The pins settle at
     * idle high with nothing counting, and the first edge the timer ever sees is
     * the first clock of the frame.
     */
    base->PINOUTD |= (clk_bit | dat_bit);
    base->PINOUTE |= (clk_bit | dat_bit);
    for (volatile unsigned d = 0; d < 2000u; d++) { }

    FLEXIO_ClearShifterErrorFlags(base, cap_all_shifter_mask());
    FLEXIO_Enable(base, true);

    s_write = s_read = 0u;
    s_wr_idx = 0u;
    cap.entries      = 0u;
    cap.tick         = 0u;
    cap.last_tick    = 0u;
    cap.have_initial = false;
    cap.changed_mask = 0u;

    dec_reset();
    dec_i3c_init(ev_push);   /* same emit path; a different state machine */
    frame_reset();
    dec_seed(cap.last_state);

    /* START: SDA falls while SCL stays high. */
    gen_set(clk_bit, dat_bit, true, false, settle);

    /* 0xA0 - a write to target 0x50, which is the address the protocol's
     * conformance expectation names. */
    beacon->phase = 0x11u;
    gen_i2c_byte(clk_bit, dat_bit, 0xA0u, settle);
    for (unsigned i = 0; i < 3u; i++) {
        beacon->phase = 0x20u + i;
        gen_i2c_byte(clk_bit, dat_bit, payload[i], settle);
    }
    beacon->phase = 0x30u;

    /* STOP: bring the clock high with data low, then release data. */
    gen_set(clk_bit, dat_bit, false, false, settle);
    gen_set(clk_bit, dat_bit, true,  false, settle);
    gen_set(clk_bit, dat_bit, true,  true,  settle);

    /*
     * Clock the data shifter out after the frame.
     *
     * A shifter emits whole 32-bit words and nothing sooner, so a frame whose
     * bit count is not a multiple of 32 leaves its tail inside - and this frame
     * is 45 bits, so the last byte and its acknowledge sat in the shifter and
     * the capture decoded three bytes of the four driven.
     *
     * The clocks are driven with SDA held high and after the STOP, so they carry
     * no data and fall outside any frame: `in_frame` is false, so the byte
     * assembler ignores them entirely and they cost nothing but raw samples. On
     * a live bus the same tail exists and the next transaction's clocks push it
     * out, which is the ordinary case; a bus that stops mid-word holds its last
     * few bits until it moves again.
     */
    for (unsigned c = 0; c < 32u; c++) {
        gen_set(clk_bit, dat_bit, false, true, settle / 8u + 1u);
        gen_set(clk_bit, dat_bit, true,  true, settle / 8u + 1u);
    }

    /*
     * Flush the framing shifters with one harmless edge in each direction.
     *
     * A framing shifter loads SHIFTBUF on the edge *after* the one it sampled,
     * so every channel runs exactly one edge behind and the last edge of a frame
     * is still inside the shifter when the frame ends. On the falling channel
     * that costs nothing - the last fall is an acknowledge bit with the clock
     * down. On the rising channel the last rise is the STOP, so the event that
     * ends the transaction was the one permanently stuck: the capture reported a
     * START and nothing else, with eight words read where nine edges happened.
     *
     * SCL is taken down first and left there, so neither of these edges is a
     * data bit - the data shifter is clocked by SCL's *rising* edges - and both
     * read as ordinary setups with the clock low, so they generate no events of
     * their own. They exist purely to push the real last sample out.
     *
     * On a live bus the same one-edge lag applies and there is nothing to flush
     * with: the final STOP of the last transaction stays in the shifter until
     * the bus moves again. That is a property of clocking a shifter from the
     * signal it is measuring, and it is why this is stated here rather than
     * quietly handled.
     */
    gen_set(clk_bit, dat_bit, false, true,  settle);
    gen_set(clk_bit, dat_bit, false, false, settle);
    gen_set(clk_bit, dat_bit, false, true,  settle);

    /*
     * Released at idle high, not pulled low. These pads have no pull, so
     * whatever level they are left at is where they stay - and leaving an I2C
     * pair low reads as a bus held busy by the next thing that looks at it.
     */
    base->PINOUTE &= ~(clk_bit | dat_bit);
    beacon->phase = 0x40u;
    return gen_edge_count;
}

static bb_cap_err_t selftest_run(uint32_t half_ns, uint32_t words,
                                 bb_selftest_t *out)
{
    /*
     * Two ways in, and the difference is who owns the capture.
     *
     * Standalone - nothing running - this configures, probes the pads, starts a
     * capture, drives the pattern and stops again. That is what SNIFF:TEST on
     * its own should do, and it is how the engine gets exercised from cold.
     *
     * With a capture already running, it drives the pattern into *that* capture
     * and leaves it running. This is the sequence a host actually uses:
     * SNIFF:START, SNIFF:TEST, SNIFF:READ, SNIFF:STOP - inject a known pattern,
     * then read and decode what the capture made of it. Earlier versions
     * refused when running (so the inject failed) and then stopped when running
     * (so the caller's later SNIFF:STOP failed instead). Both broke the same
     * sequence from opposite ends; neither was what the command means.
     */
    bool owned = !cap.running;

    if (cap.tap_count == 0u) {
        return BB_CAP_ERR_PINS;
    }

    if (owned) {
        /*
         * Probe the pads before capture starts, not during it.
         *
         * probe_pins() deliberately holds this core for microseconds at a time
         * waiting for a pad to actually reach a level - and a sampler running at
         * several MHz completes a shifter word every couple of microseconds, so
         * probing with capture live overruns the shifter every time. The first
         * version did exactly that and the self test failed with zero entries,
         * which reads as a broken engine and was a broken test.
         *
         * It is also why the probe is skipped entirely when someone else owns
         * the capture: it needs the pads and the intake to itself. `moved` still
         * answers the per-channel question, which is the one that matters.
         */
        bb_cap_err_t rc = flexio_program();

        if (rc != BB_CAP_OK) {
            return rc;
        }
        /*
         * Phase markers through the run, readable from the other core.
         *
         * The beacon lives in this core's ITCM and the controller can read it
         * while this core is mid-operation, so it answers "where did it stop"
         * without a debugger - the only question that matters when a request
         * does not come back. 0x01 probing, 0x02 starting, 0x03 sampling, 0x04
         * generating (0x10-0x40 within the I2C frame), 0x50 generated, 0x60
         * drained, 0x70 returned.
         *
         * They earned their keep immediately: a four-tap self test that timed
         * out reported phase 0x70, which said the run had *completed* and the
         * reply was merely late. The obvious reading without that was a wedged
         * core, and the search would have started in the wrong place.
         */
        beacon->phase = 0x01u;
        FLEXIO_Enable(cap_hw(), true);
        probe_pins(out);
        FLEXIO_Enable(cap_hw(), false);

        beacon->phase = 0x02u;
        rc = cap_start();
        if (rc != BB_CAP_OK) {
            return rc;
        }
    }

    beacon->phase = 0x03u;
    FLEXIO_Enable(cap_hw(), true);

    /*
     * A decodable frame where there is a decoder, a square wave otherwise.
     *
     * The square wave still has a job: for SPI, UART and MDIO there is nothing
     * here that decodes, so the only question the self test can answer is
     * whether every channel is sampled - and toggling them all together is the
     * cleanest way to ask it.
     */
    uint32_t edges;

    beacon->phase = 0x04u;
    if ((cap.proto == BB_PROTO_I2C) || (cap.proto == BB_PROTO_I3C)) {
        edges = gen_i2c_frame(half_ns);
    } else {
        edges = gen_toggle(words, half_ns);
    }
    beacon->phase = 0x50u;

    /*
     * Drain the samples the pattern left in flight, and no more.
     *
     * The bound is what matters here. This used to run up to 20000 polls and
     * break when a poll produced no new records - that is, when the bus went
     * quiet - and each poll drains up to 64 word-sets of 32 samples per tap. On
     * a bus that never goes quiet it therefore ran the full 20000, which took
     * over eight seconds and tripped the link timeout. It showed up on the
     * toggle generator because releasing the taps leaves pads with no pull
     * floating, and floating pads keep producing transitions - so the exit
     * condition was waiting for something that was never going to happen.
     *
     * "Until the bus is quiet" is the wrong question for a sniffer in any case:
     * the whole point of this engine is capturing buses that are busy, and on a
     * live bus the old loop would have taken its worst case every single time.
     *
     * What is actually wanted is the tail of the generated pattern, and that is
     * a known, small quantity: at most a word-set or two per shifter, where one
     * cap_poll() drains up to 64. So a handful of polls is sufficient and 64 is
     * already generous. The early break still exits immediately in the common
     * case where the intake is empty.
     */
    for (uint32_t i = 0; i < 64u; i++) {
        uint32_t before  = s_write;
        uint32_t before_e = s_ev_write;

        cap_poll();
        /*
         * Progress means *either* ring moved.
         *
         * Watching only the sample ring stops the drain as soon as the data
         * shifters run dry - and the framing shifters are in a different clock
         * domain, so they are still holding edges at that moment. The last two
         * SDA edges of a frame were left unread every time, and the second of
         * those is the STOP: the whole reason a frame ends. The self test
         * reported a START and no STOP, with 16 framing words processed out of
         * the 18 the generator drives.
         */
        if (((s_write == before) && (s_ev_write == before_e)) || !cap.running) {
            break;
        }
    }
    beacon->phase = 0x60u;
    if (owned) {
        (void)cap_stop();
    }
    beacon->phase = 0x70u;

    out->driven_loads = edges;
    out->expected     = (edges > 0u) ? (edges - 1u) : 0u;
    out->moved        = cap.changed_mask;
    out->ticks        = (uint32_t)cap.tick;
    out->seen         = cap.entries;
    out->sample_hz    = cap.sample_hz;
    out->src_hz       = cap_src_hz();
    out->frame_rise   = (s_frame_words[0] << 8) | (s_frame_hi[0] & 0xFFu);
    out->frame_fall   = (s_frame_words[1] << 8) | (s_frame_hi[1] & 0xFFu);
    out->overruns     = cap.overruns;
    out->pin_live     = FLEXIO_ReadPinInput(cap_hw());
    out->shiftcfg     = cap_hw()->SHIFTCFG[CAP_SHIFTER];
    out->shiftctl     = cap_hw()->SHIFTCTL[CAP_SHIFTER];
    out->timcmp       = cap_hw()->TIMCMP[CAP_TIMER];
    out->base_pin     = cap.base_pin;
    out->width        = cap.width;

    if (out->driven_loads == 0u) {
        return BB_CAP_ERR_FLEXIO;
    }
    /*
     * Every channel moved, transitions were seen, and nothing was lost.
     *
     * The per-channel test is the one that matters: the generator drives all
     * taps together, so a channel that did not move is a channel the capture
     * cannot see - which is precisely the failure that passed unnoticed while
     * the test only asked whether *anything* moved.
     */
    /* Per *shifter*, not per tap: pin-clocked mode spends tap 0 on the timer, so
     * a two-tap capture has one channel that can move and asking for two would
     * fail a working capture. */
    unsigned nch  = cap_shifter_count();
    uint32_t want = (nch >= 32u) ? 0xFFFFFFFFu : ((1uL << nch) - 1u);

    if (out->seen == 0u || out->overruns != 0u) {
        return BB_CAP_ERR_FLEXIO;
    }
    /*
     * No more transitions than the generator could physically have produced.
     *
     * This is an invariant rather than a tolerance: both generators write the
     * pin data register once per counted edge - gen_toggle() XORs the mask,
     * gen_set() writes a level - and the pins hold constant between writes, so
     * the sampler cannot observe two transitions from one write. The allowance
     * covers the edge from enabling the output driver, which moves the pads
     * before the first counted write.
     *
     * `expected` has been computed and reported here all along and nothing ever
     * checked it, which is how a reversed sample order passed this test: the
     * only questions asked were whether *anything* was seen and whether every
     * channel moved, and time-reversed samples satisfy both. That is the second
     * time this test has passed a real defect by asking too little of the
     * numbers it already had.
     *
     * Owned captures only, for the same reason the per-channel check is: when
     * injecting into someone else's capture, `entries` counts their samples too
     * and the bound is not ours to apply.
     */
    if (owned && (out->seen > (out->driven_loads + (out->driven_loads / 2u) + 2u))) {
        return BB_CAP_ERR_FLEXIO;
    }
    /*
     * The per-channel check applies only to a capture this call owns.
     *
     * Injecting into someone else's capture cannot be held to it: their
     * configuration may cover taps this generator is not driving, and their
     * counters started before the pattern did. `moved` is still reported either
     * way, so the caller can see it - it just is not grounds for failing a run
     * whose scope was not ours to set.
     */
    if (owned && out->moved != want) {
        return BB_CAP_ERR_FLEXIO;
    }
    return BB_CAP_OK;
}

/*
 * The ceiling, measured rather than derived.
 *
 * The generator is driven progressively faster until the shifter overruns or
 * entries stop keeping up, and the last clean rate is reported. That is a
 * property of this core's polling loop, not of the sampler, which is exactly
 * the figure worth knowing: it is what eDMA will move.
 */
static bb_cap_err_t bench_run(uint32_t start_half_ns, bb_bench_t *out)
{
    uint32_t half_ns = (start_half_ns < 20u) ? 20u : start_half_ns;
    uint32_t best_hz = 0u;
    uint32_t best_bytes = 0u;

    if (cap.tap_count == 0u) {
        return BB_CAP_ERR_PINS;
    }

    for (unsigned step = 0; step < 12u; step++) {
        bb_selftest_t r;
        memset(&r, 0, sizeof r);

        bb_cap_err_t rc = selftest_run(half_ns, 32u, &r);
        if (rc != BB_CAP_OK || r.overruns != 0u) {
            break;
        }
        /* One full window flip per load, so the equivalent bus clock is half
         * the load rate - a clock line completes a cycle every two flips. */
        uint32_t load_hz = (half_ns > 0u) ? (1000000000u / half_ns) : 0u;
        best_hz = load_hz / 2u;

        /*
         * A genuine rate, not the size of one run.
         *
         * This was `r.seen * sizeof(bb_sample_t)` and called
         * `dma_bytes_per_s` - the total bytes the run produced, reported as a
         * per-second figure. It read as 616 B/s for a capture moving tens of
         * kilobytes a second, and nothing noticed because a small number in a
         * diagnostic field looks like a small number.
         */
        if ((r.ticks != 0u) && (r.sample_hz != 0u)) {
            uint64_t bytes = (uint64_t)r.seen * sizeof(bb_sample_t);

            best_bytes = (uint32_t)((bytes * r.sample_hz) / r.ticks);
        } else {
            best_bytes = 0u;
        }

        if (half_ns <= 20u) {
            break;
        }
        half_ns = (half_ns * 2u) / 3u;
        if (half_ns < 20u) {
            half_ns = 20u;
        }
    }

    /*
     * Bound the answer by what the sampler can actually represent.
     *
     * The sweep stops when the generator can go no faster - its floor is
     * half_ns >= 20 - so on its own it reports how fast *this core can flip
     * pins*, which on a 120 MHz FlexIO root came out as 17.8 MHz while the
     * sampler was running at 6 MHz. A 6 MHz sampler cannot represent a 17.8 MHz
     * bus at all; it aliases. `overruns` stays zero throughout because the
     * sampler is keeping up with *itself*, and every channel still moves, so
     * nothing in the self test contradicts the claim.
     *
     * Reporting it unbounded would repeat the STM32 build's mistake in mirror
     * image. There the generator was slower than the nominal rate and the test
     * reported the nominal figure; here the generator is faster than the
     * sampler and the test reported the generator. Both publish a number the
     * capture cannot deliver.
     *
     * Four samples per bus clock is the bound used - two is Nyquist for the
     * clock alone, and a decoder needs to see the data line settle within each
     * half-cycle, so two is not enough to decode with. `limited_by` says which
     * constraint bound the result, because "the generator could not go faster"
     * and "the sampler could not follow" are different answers and a caller
     * choosing hardware needs to know which one it got.
     */
    uint32_t sampler_hz = cap.sample_hz / 4u;

    if (best_hz > sampler_hz) {
        best_hz          = sampler_hz;
        out->limited_by  = 0u;      /* sampler */
    } else {
        out->limited_by  = 1u;      /* generator */
    }
    out->sample_hz       = cap.sample_hz;
    out->max_bus_hz      = best_hz;
    out->dma_bytes_per_s = best_bytes;
    out->from_selftest   = 1u;
    /*
     * cost_ns is the sample period, not a decode cost, and the difference is
     * the whole point of this engine.
     *
     * On the STM32 this figure was the ~450 ns of interrupt entry, GPIO read and
     * EXTI write that every single edge cost the CPU - which is why its ceiling
     * was around 1 MHz. Here nothing is spent per edge: FlexIO latches in
     * hardware and the core only unpacks whole words. So the honest equivalent
     * is the sample period, which is what actually bounds edge resolution.
     */
    out->cost_ns = (cap.sample_hz != 0u) ? (1000000000u / cap.sample_hz) : 0u;
    return (best_hz == 0u) ? BB_CAP_ERR_FLEXIO : BB_CAP_OK;
}

/* ---- record serialisation ---------------------------------------------- */
/*
 * Records are built into the reply buffer on demand rather than into a second
 * staging area. The ring holds samples; a record is a header plus a run of
 * them, and building it at drain time means the ring stores no framing overhead
 * and the reply is never larger than the transport allows.
 */
static size_t emit_meta(uint8_t *out, size_t cap_bytes)
{
    if (cap_bytes < sizeof(bb_rec_hdr_t) + sizeof(bb_meta_t)) {
        return 0u;
    }
    bb_rec_hdr_t hdr;
    bb_meta_t    m;

    memset(&m, 0, sizeof m);
    m.proto         = (uint8_t)cap.proto;
    m.channel_count = cap.tap_count;
    m.flags         = cap.flags;
    m.ps_per_tick   = cap.ps_per_tick;
    /* Contract 3: the initial state, not a transition. Without it a decoder
     * cannot know what the lines were doing before the first edge, and the
     * STM32 build proved what that costs - the first release of SDA read as a
     * STOP that never happened. */
    m.initial_state = cap.last_state;

    hdr.type     = (uint8_t)BB_REC_META;
    hdr.observed = (uint8_t)cap.source;
    hdr.length   = (uint16_t)sizeof m;
    hdr.crc32    = bb_image_crc32((const uint8_t *)&m, sizeof m);
    hdr.t_ps     = 0u;

    memcpy(out, &hdr, sizeof hdr);
    memcpy(out + sizeof hdr, &m, sizeof m);
    cap.records++;
    return sizeof hdr + sizeof m;
}

static size_t emit_gap(uint8_t *out, size_t cap_bytes, uint8_t reason)
{
    if (cap_bytes < sizeof(bb_rec_hdr_t) + sizeof(bb_gap_t)) {
        return 0u;
    }
    bb_rec_hdr_t hdr;
    bb_gap_t     g;

    memset(&g, 0, sizeof g);
    g.reason = reason;
    /* lost_estimate stays 0: when the ring fills we genuinely do not know how
     * many entries the bus produced while capture was stopped, and inventing a
     * number would be worse than admitting it. The Rust side treats 0 as
     * "unknown" rather than "none". */
    g.lost_estimate = 0u;

    hdr.type     = (uint8_t)BB_REC_GAP;
    hdr.observed = (uint8_t)cap.source;
    hdr.length   = (uint16_t)sizeof g;
    hdr.crc32    = bb_image_crc32((const uint8_t *)&g, sizeof g);
    hdr.t_ps     = 0u;

    memcpy(out, &hdr, sizeof hdr);
    memcpy(out + sizeof hdr, &g, sizeof g);
    cap.records++;
    return sizeof hdr + sizeof g;
}

static size_t drain(uint8_t *out, size_t want)
{
    size_t used = 0u;

    if (want > BB_MSG_MAX_PAYLOAD) {
        want = BB_MSG_MAX_PAYLOAD;
    }
    if (cap.meta_pending) {
        size_t n = emit_meta(out, want);
        if (n == 0u) {
            return 0u;
        }
        cap.meta_pending = false;
        used += n;
    }
    if (cap.full && s_read == s_write) {
        used += emit_gap(out + used, want - used, BB_GAP_RING_FULL);
        cap.full = false;
        cap.gaps++;
        return used;
    }

    /* One SAMPLES record per drain, holding as many entries as fit. */
    uint32_t avail = s_write - s_read;
    if (avail == 0u) {
        return used;
    }
    size_t room = (want > used + sizeof(bb_rec_hdr_t))
                      ? (want - used - sizeof(bb_rec_hdr_t)) : 0u;
    uint32_t n = (uint32_t)(room / sizeof(bb_sample_t));
    if (n == 0u) {
        return used;
    }
    if (n > avail) {
        n = avail;
    }

    bb_rec_hdr_t hdr;
    uint8_t     *payload = out + used + sizeof hdr;
    for (uint32_t i = 0; i < n; i++) {
        memcpy(payload + i * sizeof(bb_sample_t),
               &s_ring[(s_read + i) % CAP_RING_ENTRIES],
               sizeof(bb_sample_t));
    }
    hdr.type     = (uint8_t)BB_REC_SAMPLES;
    hdr.observed = (uint8_t)cap.source;
    hdr.length   = (uint16_t)(n * sizeof(bb_sample_t));
    hdr.crc32    = bb_image_crc32(payload, hdr.length);
    hdr.t_ps     = cap.base_ticks * cap.ps_per_tick;
    memcpy(out + used, &hdr, sizeof hdr);

    /*
     * Advance the absolute base past the entries just handed over. The record
     * header carries the absolute time of its first entry and the entries carry
     * deltas from it, so the base for the *next* record is this one plus the
     * deltas in it. Recomputing it from a running counter instead would drift
     * apart from the deltas the host is accumulating, which is the one error
     * this format cannot detect.
     */
    for (uint32_t i = 0; i < n; i++) {
        cap.base_ticks += s_ring[(s_read + i) % CAP_RING_ENTRIES].dt_ticks;
    }
    s_read += n;
    cap.records++;
    return used + sizeof hdr + hdr.length;
}

/* ---- message handling -------------------------------------------------- */

static void reply(bb_cap_err_t st, const void *payload, size_t len)
{
    if (len > BB_MSG_MAX_PAYLOAD) {
        len = BB_MSG_MAX_PAYLOAD;
    }
    if (payload != NULL && len > 0u) {
        memcpy((void *)shm->rep, payload, len);
    }
    shm->rep_hdr.op     = (uint8_t)(shm->req_hdr.op | (uint8_t)BB_MSG_REPLY);
    shm->rep_hdr.status = (uint8_t)st;
    shm->rep_hdr.length = (uint16_t)len;
    shm->rep_hdr.seq    = shm->req_hdr.seq;
    __DSB();
    shm->req_ready = 0u;
    shm->rep_ready = 1u;
    __DSB();
}

static void handle(void)
{
    switch ((bb_msg_op_t)shm->req_hdr.op) {
    case BB_MSG_PING: {
        /* Identifies the core and the format version, so a mismatched pair of
         * images is caught at the handshake rather than by a capture that
         * decodes to nonsense. */
        static const char id[] = "cm7-capture-1";
        reply(BB_CAP_OK, id, sizeof id);
        break;
    }
    case BB_MSG_CONFIG: {
        bb_cfg_t cfg;
        if (shm->req_hdr.length < sizeof cfg) {
            reply(BB_CAP_ERR_RANGE, NULL, 0u);
            break;
        }
        memcpy(&cfg, (const void *)shm->req, sizeof cfg);
        reply(cap_configure(&cfg), NULL, 0u);
        break;
    }
    case BB_MSG_START:
        reply(cap_start(), NULL, 0u);
        break;
    case BB_MSG_STOP:
        reply(cap_stop(), NULL, 0u);
        break;
    case BB_MSG_CLEAR:
        dec_reset();
    dec_i3c_init(ev_push);   /* same emit path; a different state machine */
        s_write = s_read = 0u;
    s_wr_idx = 0u;
        cap.entries = cap.coincident = 0u;
        cap.full = false;
        reply(BB_CAP_OK, NULL, 0u);
        break;
    case BB_MSG_STAT: {
        bb_stat_t st;
        memset(&st, 0, sizeof st);
        st.running       = cap.running ? 1u : 0u;
        st.proto         = (uint8_t)cap.proto;
        st.source        = (uint8_t)cap.source;
        st.full          = cap.full ? 1u : 0u;
        st.entries       = cap.entries;
        st.records       = cap.records;
        st.bytes_pending = (s_write - s_read) * (uint32_t)sizeof(bb_sample_t);
        st.gaps          = cap.gaps;
        st.coincident    = cap.coincident;
        st.ps_per_tick   = cap.ps_per_tick;
        st.sample_hz     = cap.sample_hz;
        st.overruns      = cap.overruns;
        /* Asked here, on the core it describes. SystemCoreClock is whatever the
         * startup left behind; the root clock is what the core is actually
         * fed. */
        st.core_hz       = CLOCK_GetRootClockFreq(kCLOCK_Root_M7);
        st.prof_polls    = s_prof.polls;
        st.prof_words    = s_prof.words;
        st.prof_cyc_poll = (uint32_t)(s_prof.cyc_poll >> 6);
        st.prof_cyc_word = (uint32_t)(s_prof.cyc_word >> 6);
        st.prof_cyc_max  = s_prof.cyc_max;
        st.defer_bits    = frm.defer_last;
        st.arm_bits      = frm.arm_last;
        st.frame_bits    = frm.frame_last;
        st.start_at      = frm.start_at;
        st.stop_at       = frm.stop_at;
        st.bothpend_n    = frm.bothpend_n;
        st.reorder_n     = frm.reorder_n;
        st.dma_avail_max = s_dma_avail_max;
        /* How many framing events the hardware raised, against how many
         * transactions the bus carried. Separates "the detector missed
         * it" from "the decoder dropped it", which no other counter
         * here distinguishes. [0] is the STOP detector, [1] the START. */
        st.setup_starts  = frm.setup_starts;
        st.stops_seen    = s_frame_words[0];
        st.stops_noted   = s_frame_hi[0];
        st.starts_seen   = s_frame_words[1];
        {
            /* Through a local: bb_stat_t is packed, so taking the address of a
             * member inside it is an unaligned pointer. */
            uint32_t log[8];
            unsigned ln = dec_i3c_close_log(log, 8u);

            st.close_n = (uint8_t)ln;
            for (unsigned i = 0; i < ln; i++) {
                st.close_log[i] = log[i];
            }
        }
        reply(BB_CAP_OK, &st, sizeof st);
        break;
    }
    case BB_MSG_EVENTS: {
        /*
         * Events are drained as a count plus that many fixed-size records, and
         * the controller renders them as protocol tokens. Splitting it that way
         * keeps the text formatting on the core that owns the wire protocol and
         * keeps this core free of it.
         */
        uint32_t want = 64u;

        if (shm->req_hdr.length >= sizeof want) {
            memcpy(&want, (const void *)shm->req, sizeof want);
        }

        uint8_t          buf[BB_MSG_MAX_PAYLOAD];
        bb_events_hdr_t *hdr  = (bb_events_hdr_t *)buf;
        bb_event_t      *out  = (bb_event_t *)(buf + sizeof *hdr);
        uint32_t         room = (BB_MSG_MAX_PAYLOAD - sizeof *hdr) / sizeof(bb_event_t);
        uint32_t         n    = 0u;

        if (want > room) { want = room; }

        /* A lost-event marker leads, so the hole appears before the events that
         * followed it rather than after. */
        if ((dec.dropped != 0u) && (want > 0u)) {
            out[n].kind = (uint8_t)BB_EV_DROPPED;
            out[n].value = 0u;
            out[n].ack = 0u;
            out[n].pad = 0u;
            n++;
            dec.dropped = 0u;
        }
        while ((n < want) && (s_ev_read != s_ev_write)) {
            out[n] = s_events[s_ev_read % CAP_EVENT_ENTRIES];
            s_ev_read++;
            n++;
        }
        hdr->count = (uint16_t)n;
        hdr->more  = (uint16_t)((s_ev_read != s_ev_write) ? 1u : 0u);
        reply(BB_CAP_OK, buf, sizeof *hdr + n * sizeof(bb_event_t));
        break;
    }
    case BB_MSG_READ: {
        uint32_t want = BB_MSG_MAX_PAYLOAD;
        if (shm->req_hdr.length >= sizeof want) {
            memcpy(&want, (const void *)shm->req, sizeof want);
        }
        uint8_t buf[BB_MSG_MAX_PAYLOAD];
        size_t  n = drain(buf, want);
        reply(BB_CAP_OK, buf, n);
        break;
    }
    case BB_MSG_SELFTEST: {
        uint32_t half_ns = 2500u;
        bb_selftest_t r;

        if (shm->req_hdr.length >= sizeof half_ns) {
            memcpy(&half_ns, (const void *)shm->req, sizeof half_ns);
        }
        memset(&r, 0, sizeof r);
        bb_cap_err_t st = selftest_run(half_ns, 64u, &r);
        reply(st, &r, sizeof r);
        break;
    }
    case BB_MSG_BENCH: {
        uint32_t   half_ns = 2500u;
        bb_bench_t b;

        if (shm->req_hdr.length >= sizeof half_ns) {
            memcpy(&half_ns, (const void *)shm->req, sizeof half_ns);
        }
        memset(&b, 0, sizeof b);
        bb_cap_err_t st = bench_run(half_ns, &b);
        reply(st, &b, sizeof b);
        break;
    }
    default:
        reply(BB_CAP_ERR_UNSUPPORTED, NULL, 0u);
        break;
    }
}

/* ---- main -------------------------------------------------------------- */

int main(void)
{
    BOARD_InitHardware();

    memset(&cap, 0, sizeof cap);
    timebase_init();

    /*
     * Announce only after the magic is there. The CM33 writes it before
     * releasing this core, so its absence means this image is running without a
     * controller - which happens when the CM7 is loaded by a debugger directly.
     * Waiting rather than proceeding avoids answering into memory nobody owns.
     */
    /* Beacon first: it must be visible even if the mailbox never is. */
    beacon->magic     = BB_BEACON_MAGIC;
    beacon->ticks     = 0u;
    beacon->saw_magic = 0u;
    __DSB();

    while (shm->magic != BB_SHM_MAGIC) {
        beacon->ticks++;
        __DSB();
    }
    beacon->saw_magic = 1u;
    __DSB();
    shm->cm7_alive = 1u;
    __DSB();

    uint32_t loop_pass = 0u;

    for (;;) {
        /*
         * Re-announce whenever the controller has cleared the flag.
         *
         * bb_link_init() sets cm7_alive to 0, correctly - the controller cannot
         * know whether a capture core exists. But announcing only once at boot
         * meant that restarting the controller orphaned a perfectly healthy
         * capture core for good: every SNIFF:* command then reported
         * "capture-core-not-running" while this image sat here polling.
         *
         * That is not a test-rig inconvenience. The controller is the image that
         * gets reloaded during development, and it is also the one that reboots
         * on a watchdog or a USB reset, so a link that cannot survive the
         * controller restarting is a link that breaks in normal use. The
         * handshake has to be level-triggered from this side, not edge.
         */
        /*
         * The heartbeat is not free, and it sits in the intake's way.
         *
         * Both counters live in non-cacheable memory, so each `++` is an
         * uncached read-modify-write - and they ran once per loop, which at
         * 12.5 MHz is once per shifter word, because a drain at that rate
         * usually finds a single word waiting. Two uncached round trips per
         * word is a large share of a 640 ns budget.
         *
         * Every 64th pass keeps the counters moving far faster than anything
         * that watches them needs - SNIFF:LINK? only asks whether the delta is
         * non-zero - and takes the cost out of the per-word path.
         */
        /*
         * Everything that is not the intake runs once per 64 drains.
         *
         * The heartbeat writes were amortised first, and the reads turned out to
         * matter just as much: `shm->magic`, `shm->cm7_alive` and
         * `shm->req_ready` all live in non-cacheable memory, so each is a bus
         * round trip that cannot be cached or reordered away, and they ran on
         * every pass. At 12.5 MHz a pass drains about one shifter word - the
         * words arrive every 640 ns and this loop is faster than that - so the
         * mailbox was being polled once per bus word, at three uncached reads a
         * time, out of a budget of roughly 500 cycles.
         *
         * 64 drains is well under a microsecond, so the mailbox still answers
         * far faster than a USB command can arrive.
         */
        /*
         * Every 4096th pass, not every 64th.
         *
         * With the idle poll down to about 100 cycles, 64 passes is roughly a
         * dozen shifter words at 12.5 MHz - so this block, which is several
         * uncached round trips and sometimes a whole mailbox transaction, landed
         * inside almost every burst and stalled the intake long enough to drop a
         * word. The steady-state cost fits the budget; it was the periodic spike
         * that did not.
         *
         * 4096 passes is still well under a millisecond, which is far faster
         * than any host command or liveness check needs.
         */
        if ((++loop_pass & 0xFFFu) == 0u) {
            shm->cm7_ticks += 4096u;
            beacon->ticks  += 4096u;

            if (shm->magic == BB_SHM_MAGIC && shm->cm7_alive == 0u) {
                shm->cm7_alive = 1u;
                __DSB();
            }
            if (shm->req_ready != 0u) {
                __DSB();
                handle();
            }
        }
        /* The capture intake. This core exists to do exactly this, which is why
         * polling rather than an interrupt is the right shape: there is nothing
         * to be preempted from, and an overrun becomes a recorded gap instead of
         * a silently wrong capture. */
        cap_poll();
    }
}
