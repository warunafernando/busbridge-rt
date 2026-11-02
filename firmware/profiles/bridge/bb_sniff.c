/**
 * @file bb_sniff.c
 * @brief SNIFF:* command surface — the controller side of the capture engine.
 *
 * **Rewritten, not ported.** The STM32's `bb_sniff.c` was 1102 lines of edge
 * interrupt and software decode, and every one of its limits came from that
 * shape: ~450 ns per bus clock, a ~1 MHz ceiling, and native I3C phases read
 * one bit late because the handler sampled SDA after SCL had already fallen.
 * None of that logic is worth translating. The capture happens in FlexIO on the
 * CM7 now, clocked by the bus's own clock line, and this file is the command
 * layer in front of it.
 *
 * The command surface is kept, because the host tools and the Rust `sniffer`
 * project speak it. Three additions:
 *
 *   `SNIFF:SRC <wire|self|both>`  the new capability - see below
 *   `SNIFF:CH <proto> <taps...>`  bind a channel to a protocol and tap pins
 *   `SNIFF:TAPS?`                 what tap pins this board has spare
 *
 * `SNIFF:SRC` is the answer to "sniff the ports while in use", and it has three
 * settings because the hardware gives three genuinely different things:
 *
 *   `wire`   FlexIO samples tap pads. Sees everything on the bus including
 *            other masters. Needs the taps wired - two jumpers for a two-wire
 *            bus - because FlexIO inputs have no pad selection on this silicon
 *            and so cannot be pointed at a pad a bus already owns.
 *   `self`   The controller's own transfers, recorded as they are driven. No
 *            pins at all, so it works while the bus is in use because it *is*
 *            the bus in use, and it needs no taps and no capture core. Tagged
 *            `observed=controller`, and that tag is load-bearing - a FIFO says
 *            what the peripheral accepted, not what the wire did. Implemented
 *            in bb_trace.c, which is also where the limits are written down:
 *            the ninth bit of a data byte is computed rather than observed,
 *            and CCC/ENTDAA frames are marked as holes rather than described.
 *   `both`   STILL NOT IMPLEMENTED, and now for one specific reason rather
 *            than for want of a source. Merging the two on timestamp needs the
 *            offset between the CM33's SysTick-derived counter and the capture
 *            core's sample clock, and a bound on their relative drift. Neither
 *            has been measured, and a merge built on an assumed offset would
 *            interleave the two streams wrongly while looking entirely
 *            plausible. cmd_src() refuses and says so.
 *
 * The tag plumbing was complete long before anything generated a record to put
 * it on - this file set the value, push_config() passed it down, the capture
 * core stamped it into every record - so selecting `self` used to label wire
 * samples as controller-observed, which is precisely the confusion the tag
 * exists to prevent.
 */
#include "bb_board.h"
#include "bb_capture.h"
#include "bb_cmd.h"
#include "bb_image.h"
#include "bb_io.h"
#include "bb_link.h"
#include "bb_sniff.h"
#include "bb_trace.h"

#include "fsl_ele_base_api.h"
#include "bb_status.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* The board's spare FlexIO-capable pads. */
typedef struct {
    const char *label;
    bb_pin_t    pin;
    uint8_t     flexio;    /**< 1 or 2 - a channel cannot straddle them */
    uint8_t     signal;    /**< FLEXIO pin number within that instance */
} bb_tap_t;

static const bb_tap_t s_taps[] = BB_SNIFF_TAPS;

static struct {
    bb_proto_t    proto;
    /*
     * Two flags rather than one source, because `both` is not a third kind of
     * observation - it is both engines running, each keeping its own true tag.
     * Folding it into bb_observed_t would have meant inventing a value for the
     * capture core to stamp on records that are still, individually, wire
     * samples or controller records. That is the confusion `observed=` exists
     * to prevent, arriving through the field itself.
     */
    bool          src_wire;
    bool          src_self;
    bool          strict;
    uint8_t       taps[8];
    uint8_t       tap_count;
    bool          configured;
    uint32_t      nominal_hz;   /**< bus rate the taps are expected to carry */
    uint32_t      sample_hz;    /**< 0 = let the capture core choose */
    bb_cap_mode_t mode;         /**< oversample or pin-clocked */
    bool          dma_intake;   /**< SNIFF:DMA - eDMA the pin-clocked intake */
    uint8_t       gap_bits;     /**< SNIFF:GAP - inter-frame clock edges */
} s = {
    .proto      = BB_PROTO_I2C,
    .src_wire   = true,
    .mode       = BB_CAP_OVERSAMPLE,
    .dma_intake = true,
    .gap_bits   = 1u,
    .nominal_hz = 400000u,
    /* d4 and d5, the I2C default. Set here so SNIFF:START works on a board
     * nobody has run SNIFF:CH on - which is how every existing host script and
     * the GUI arrive. */
    .taps       = { 0u, 1u },
    .tap_count  = 2u,
};

void bb_sniff_init(void)
{
    /*
     * Park every tap pad high-impedance at startup, not just the ones a capture
     * later claims.
     *
     * "A sniffer must not load the bus it is watching" was already the rule in
     * cap_configure(), but it only took effect once a capture started. Until
     * then the pads sat at their reset default, and the EMC pads do not reset
     * high-Z - they come up with a pull that fights whatever the bus has.
     *
     * On this board d8/d9 are the I3C tap pair, so a tap nobody had asked for
     * was holding the I3C net. The bus read low at the controller's own pads
     * and ENTDAA answered no-ack-for-7E, which points firmly at wiring or at
     * the target - and it recovered the instant a capture started, which is the
     * one thing that made the tap pads high-Z. Two separate bench sessions were
     * spent on that before the pattern showed up.
     *
     * A tap is a passive observer of someone else's bus from reset, or it is
     * not passive at all.
     */
    unsigned        count = 0u;
    const bb_tap_t *taps  = s_taps;

    count = (unsigned)(sizeof s_taps / sizeof s_taps[0]);
    for (unsigned i = 0; i < count; i++) {
        /*
         * GPIO input rather than the FlexIO alternate function, for two
         * reasons. It is unambiguously high-Z whether or not FlexIO is
         * clocked, and it keeps bb_pin_read() meaningful: RGPIO's input
         * register does not follow a pad muxed to a peripheral ALT, so parking
         * these on FlexIO made SNIFF:PINS? report levels=00 for every tap
         * including ones on a bus that was demonstrably idling high. A
         * diagnostic that always says zero is worse than none - it was briefly
         * read here as proof that the bus was stuck low.
         *
         * cap_configure() still moves the pads it needs to FlexIO when a
         * capture starts, and this is where they come back to afterwards.
         */
        bb_pin_gpio(&taps[i].pin, false, false, BB_PAD_INPUT);
    }

    bb_link_init();
}

/* ---- helpers ----------------------------------------------------------- */

/** Report a link or capture-core failure in the protocol's own terms. */
static bool cap_reply(bb_cap_err_t st, const char *what)
{
    if (st == BB_CAP_OK) {
        return true;
    }
    if (st == BB_CAP_ERR_TIMEOUT) {
        /*
         * Checked before the link-down branch, because a timeout marks the link
         * down and would otherwise be reported as a missing image.
         *
         * The distinction is the whole point: "no capture core" and "the capture
         * core took longer than the budget" call for opposite responses, and
         * conflating them told a caller to load an image onto a core that was
         * already running and answering its beacon.
         *
         * The beacon is quoted rather than interpreted. Its tick count and
         * phase are what distinguish a core that is wedged from one that is
         * merely slow, and a single reading cannot tell those apart on its own -
         * so this reports the numbers and leaves the reading to SNIFF:LINK?,
         * which can sample them twice. Phase in particular is a position in the
         * run, not a liveness flag: 0x40 means the generator finished.
         */
        uint32_t magic = 0u, bticks = 0u, saw = 0u;

        bb_link_beacon(&magic, &bticks, &saw);
        bb_reply_errf(BB_ERR_HAL, "%s timed-out (%s) bticks=%lu phase=0x%02lX",
                      what, bb_link_status(),
                      (unsigned long)bticks,
                      (unsigned long)bb_link_beacon_phase());
        return false;
    }
    if (!bb_link_up()) {
        /* Distinct from every other failure, and phrased so it is actionable:
         * the capture core is a separate image on a separate core, and the most
         * likely reason it is not answering is that nobody loaded it. */
        bb_reply_errf(BB_ERR_UNSUPPORTED,
                      "%s capture-core-not-running (%s) load-the-cm7-image",
                      what, bb_link_status());
        return false;
    }
    int code = (st == BB_CAP_ERR_BUSY)        ? BB_ERR_BUS_STATE
             : (st == BB_CAP_ERR_NOT_RUNNING) ? BB_ERR_BUS_STATE
             : (st == BB_CAP_ERR_RANGE)       ? BB_ERR_RANGE
             : (st == BB_CAP_ERR_UNSUPPORTED) ? BB_ERR_UNSUPPORTED
                                              : BB_ERR_HAL;
    bb_reply_errf(code, "%s %s", what, bb_cap_err_name(st));
    return false;
}

static bool parse_proto(const char *tok, bb_proto_t *out)
{
    if (bb_streq_ci(tok, "i2c"))  { *out = BB_PROTO_I2C;  return true; }
    if (bb_streq_ci(tok, "i3c"))  { *out = BB_PROTO_I3C;  return true; }
    if (bb_streq_ci(tok, "spi"))  { *out = BB_PROTO_SPI;  return true; }
    if (bb_streq_ci(tok, "mdio")) { *out = BB_PROTO_MDIO; return true; }
    if (bb_streq_ci(tok, "uart")) { *out = BB_PROTO_UART; return true; }
    if (bb_streq_ci(tok, "can"))  { *out = BB_PROTO_CAN;  return true; }
    return false;
}

/** How many taps a protocol needs, and it is worth refusing the wrong count. */
static unsigned proto_taps(bb_proto_t p)
{
    switch (p) {
    case BB_PROTO_I2C:
    case BB_PROTO_I3C:
    case BB_PROTO_MDIO: return 2u;   /* clock + data */
    case BB_PROTO_SPI:  return 4u;   /* sck, mosi, miso, cs */
    case BB_PROTO_UART: return 1u;   /* rx; tx optional */
    case BB_PROTO_CAN:  return 0u;   /* FlexCAN listen-only; no taps at all */
    default:            return 0u;
    }
}

/*
 * The taps a protocol gets when the host does not name any.
 *
 * The STM32 build had one fixed set of sniffer pins, so `SNIFF:START i2c` was
 * complete on its own. This board has seven taps across two FlexIO instances
 * and the first version required them to be named - which meant every
 * SNIFF:START from an existing host failed with no taps configured, and the
 * failure said "pins" rather than "you have to choose some".
 *
 * So a protocol now has a default: the first taps on FLEXIO1, in the order a
 * decoder expects them (clock first, then data). Naming taps explicitly still
 * overrides it, which is the point of SNIFF:CH - these are for the caller who
 * has not got an opinion yet.
 */
static unsigned proto_default_taps(bb_proto_t p, uint8_t *idx)
{
    /* d4, d5, d8, d9 are FLEXIO1 taps 0..3 in the board's table. A channel
     * cannot straddle the two instances, so the default never mixes them. */
    switch (p) {
    case BB_PROTO_I2C:
    case BB_PROTO_I3C:
    case BB_PROTO_MDIO:
        idx[0] = 0u; idx[1] = 1u;                 /* clock, data */
        return 2u;
    case BB_PROTO_SPI:
        idx[0] = 0u; idx[1] = 1u; idx[2] = 2u; idx[3] = 3u;
        return 4u;                                /* sck, mosi, miso, cs */
    case BB_PROTO_UART:
        idx[0] = 0u;
        return 1u;                                /* rx */
    default:
        return 0u;                                /* CAN uses no taps */
    }
}

static int find_tap(const char *label)
{
    for (unsigned i = 0; i < BB_SNIFF_TAP_COUNT; i++) {
        if (bb_streq_ci(label, s_taps[i].label)) {
            return (int)i;
        }
    }
    return -1;
}
static bool s_decode_off;   /* see SNIFF:DECODE */


static bb_cap_err_t push_config(void)
{
    bb_cfg_t cfg;

    memset(&cfg, 0, sizeof cfg);
    cfg.proto      = (uint8_t)s.proto;
    /* Always wire. The capture core samples pads; it has never produced a
     * controller-side record and must never be told it did. */
    cfg.source     = (uint8_t)BB_OBS_WIRE;
    cfg.mode       = (uint8_t)s.mode;
    cfg.dma_intake = s.dma_intake ? 1u : 0u;
    cfg.gap_bits   = s.gap_bits;
    cfg.decode_off = s_decode_off ? 1u : 0u;
    cfg.flags      = s.strict ? BB_META_FLAG_STRICT : 0u;
    cfg.tap_count  = s.tap_count;
    cfg.nominal_hz = s.nominal_hz;
    cfg.sample_hz  = s.sample_hz;
    memcpy(cfg.taps, s.taps, sizeof cfg.taps);

    /*
     * Resolve the taps to hardware here, and mux the pads here, because this
     * core owns the pin map. The capture core is told an instance and a list of
     * FlexIO pin numbers and needs to know nothing about the board.
     *
     * The pads go to their FLEXIO alternate function with SION set and no pull.
     * SION because a capture pin is an input whose mux mode is a peripheral
     * function, which is exactly the case the bit needs; no pull because a
     * sniffer must not load the bus it is watching - the internal ~40k would be
     * a second pull-up on someone else's carefully chosen pair.
     */
    for (unsigned i = 0; i < s.tap_count; i++) {
        const bb_tap_t *tp = &s_taps[s.taps[i]];

        cfg.pins[i] = tp->signal;
        if (i == 0u) {
            cfg.flexio = tp->flexio;
        }
        bb_pin_mux(&tp->pin, BB_PAD_INPUT, true);
    }

    return bb_link_request(BB_MSG_CONFIG, &cfg, sizeof cfg, NULL, 0u, NULL);
}

/* ---- commands ---------------------------------------------------------- */

/*
 * How the capture clocks itself. A new command, not a change to an existing one.
 *
 * `oversample` is the default and is what every existing host gets without
 * asking, so nothing that worked before behaves differently.
 *
 * `pinclock` takes the bus's own clock line - tap 0 - as the shift clock. One
 * bit per bus clock, sampled where the bus says the data is valid, which is
 * bit-exact rather than approximated and bounded by FlexIO's input
 * synchronisation at a quarter of the FlexIO clock: 30 MHz here, against I3C's
 * own 12.5 MHz ceiling. It also produces roughly a fifth of the data.
 *
 * The trade is framing, and it is worth stating at the point of choice rather
 * than burying in a manual. START and STOP are SDA moving while SCL is *high* -
 * no clock edge exists at those moments - so a shifter clocked by SCL cannot
 * see them, and the decoder is disabled in this mode. `SNIFF:RAW?` gives the
 * exact bit stream; `SNIFF:READ?` will report no events. That is a property of
 * the bus rather than of this implementation.
 */
/*
 * SNIFF:DECODE <0|1> - run the on-device decoder, or capture samples only.
 *
 * A measurement tool rather than a feature. Capture and decode both run on the
 * capture core, and the question of whether splitting them across the two cores
 * would help is really the question of what each half costs. Turning the
 * decoder off and watching the overrun count answers it directly.
 */
static void cmd_decode(int argc, char **argv)
{
    uint32_t on = 1u;

    if (argc > 1) {
        if (!bb_parse_u32_max(argv[1], 1u, &on)) {
            bb_reply_err(BB_ERR_RANGE, "expected-0-or-1");
            return;
        }
        s_decode_off = (on == 0u);
    }
    bb_reply_okf("decode=%u", s_decode_off ? 0u : 1u);
}

/*
 * SNIFF:DMA <0|1> - take the pin-clocked intake through eDMA.
 *
 * On by default, because it is strictly better where it applies. Measured over
 * ten write/read rounds, pin-clocked, against the polled intake:
 *
 *   1.07 MHz   polled 0 overruns   DMA 0 overruns   identical decode
 *   6.06 MHz   polled 0 overruns   DMA 0 overruns   identical decode
 *   12.5 MHz   polled 5 overruns   DMA 0 overruns   DMA decodes, polled does not
 *
 * At 12.5 MHz the polled intake cannot keep up: a shifter word arrives every
 * 640 ns, about 486 core cycles, and reading the shifter status, waiting for the
 * sample to settle and reading the buffer - three trips across the FlexIO bus -
 * costs roughly 180 of them before any decoding happens. The engine does that
 * work instead, and this core reads words out of RAM.
 *
 * 0 restores the polled path, which is what every rate up to 8 MHz was first
 * proven on and is still the fallback if a channel is ever unavailable.
 */
static void cmd_dma(int argc, char **argv)
{
    uint32_t n;

    if (argc > 1) {
        if (!bb_parse_u32_max(argv[1], 1u, &n)) {
            bb_reply_err(BB_ERR_RANGE, "dma-0..1");
            return;
        }
        s.dma_intake = (n != 0u);
        s.configured = false;
    }
    bb_reply_okf("dma=%u", s.dma_intake ? 1u : 0u);
}

/* SNIFF:GAP <n> - inter-frame clock edges to skip in I3C pin-clocked decode. */
static void cmd_gap(int argc, char **argv)
{
    uint32_t n;

    if (argc > 1) {
        if (!bb_parse_u32_max(argv[1], 8u, &n)) {
            bb_reply_err(BB_ERR_RANGE, "gap-0..8");
            return;
        }
        s.gap_bits = (uint8_t)n;
        s.configured = false;
    }
    bb_reply_okf("gap=%u", (unsigned)s.gap_bits);
}

static void cmd_mode(int argc, char **argv)
{
    (void)argc;
    if (bb_streq_ci(argv[1], "oversample")) {
        s.mode = BB_CAP_OVERSAMPLE;
    } else if (bb_streq_ci(argv[1], "pinclock")) {
        if (s.tap_count < 2u) {
            bb_reply_err(BB_ERR_ARGC, "pinclock-needs-a-clock-tap-and-a-data-tap");
            return;
        }
        s.mode = BB_CAP_PINCLOCK;
    } else {
        bb_reply_err(BB_ERR_RANGE, "oversample-or-pinclock");
        return;
    }
    s.configured = false;
    bb_reply_ok();
}

static void cmd_mode_q(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    bb_reply_okf("mode=%s decodes=%s",
                 (s.mode == BB_CAP_PINCLOCK) ? "pinclock" : "oversample",
                 (s.mode == BB_CAP_PINCLOCK)
                     ? "yes-bits-exact-tail-truncates-under-8-bits" : "yes");
}

/*
 * The tap source. `wire` and `self` are real; `both` still refuses.
 *
 * `self` is controller-side transaction tracing: the bridge already knows what
 * it transmitted and what came back, and emitting that into the event stream
 * costs almost nothing. The spec describes it as mechanism (c) and is emphatic
 * about one thing - such records must carry `observed=controller` so that "no
 * tool can silently confuse what was inferred with what was sampled". That tag
 * was plumbed end to end years before anything generated a record to put it
 * on; bb_trace.c is the half that was missing, and bb_trace.h is where the
 * limits of what a controller-side record can mean are written down.
 *
 * `both` is the one still refusing, and the reason is now specific rather than
 * "not written". The two streams carry different clocks: the trace stamps
 * events with bb_cycles(), SysTick-derived on this core, and the capture core
 * counts its own sample clock from the moment IT started. Interleaving them
 * needs the offset between those origins and a bound on their relative drift,
 * and neither has been measured. A merge built on an assumed offset would put
 * controller events on the wrong side of wire events while looking entirely
 * reasonable - which is the failure this whole tag exists to prevent, arriving
 * by a different door.
 *
 * Refusing until that is measured is the same choice bb_ee_i3c.c makes: a
 * subsystem that reports its own absence is worth far more than one that
 * reports success and lies about provenance.
 */
static void cmd_src(int argc, char **argv)
{
    (void)argc;
    if (bb_streq_ci(argv[1], "wire")) {
        s.src_wire   = true;
        s.src_self   = false;
        s.configured = false;
        bb_reply_ok();
        return;
    }
    if (bb_streq_ci(argv[1], "both")) {
        s.src_wire   = true;
        s.src_self   = true;
        s.configured = false;
        bb_reply_ok();
        return;
    }
    if (bb_streq_ci(argv[1], "self")) {
        s.src_wire   = false;
        s.src_self   = true;
        /* The capture core is not involved in a self trace, but it may still
         * be holding a configuration from a `wire` capture. Marking it dirty
         * means a later `SNIFF:SRC wire` re-pushes rather than starting on
         * whatever was last agreed. */
        s.configured = false;
        bb_reply_ok();
        return;
    }
    bb_reply_err(BB_ERR_RANGE, "wire-self-or-both");
}

/** True when SNIFF:READ? should drain the trace rather than the CM7. */
static bool self_trace(void)
{
    return s.src_self && !s.src_wire;
}

/** A short name for the pair, for the status lines. */
static const char *src_name(void)
{
    if (s.src_wire && s.src_self) { return "both"; }
    return s.src_self ? "controller" : "wire";
}


static void cmd_ch(int argc, char **argv)
{
    bb_proto_t proto;

    if (!parse_proto(argv[1], &proto)) {
        bb_reply_err(BB_ERR_RANGE, "proto-i2c-i3c-spi-mdio-uart-can");
        return;
    }
    unsigned need = proto_taps(proto);
    unsigned got  = (unsigned)(argc - 2);

    if (proto == BB_PROTO_CAN) {
        if (got != 0u) {
            /* CAN is captured by FlexCAN in listen-only mode, not by sampling
             * pads. Accepting taps for it would imply a capture path that does
             * not exist. */
            bb_reply_err(BB_ERR_ARGC, "can-needs-no-taps-uses-flexcan-listen-only");
            return;
        }
        s.proto     = proto;
        s.tap_count = 0u;
        s.configured = false;
        bb_reply_ok();
        return;
    }
    if (got == 0u) {
        /* None named: take the protocol's default set rather than refusing. */
        uint8_t def[8];
        unsigned n = proto_default_taps(proto, def);

        if (n == 0u) {
            bb_reply_errf(BB_ERR_ARGC, "%s-needs-taps", bb_proto_name(proto));
            return;
        }
        s.proto     = proto;
        s.tap_count = (uint8_t)n;
        memcpy(s.taps, def, sizeof s.taps);
        s.configured = false;
        bb_reply_ok();
        return;
    }
    if (got < need) {
        bb_reply_errf(BB_ERR_ARGC, "%s-needs-%u-taps", bb_proto_name(proto), need);
        return;
    }
    if (got > 8u) {
        bb_reply_err(BB_ERR_ARGC, "max-8-taps");
        return;
    }

    uint8_t  idx[8];
    unsigned instance = 0u;
    for (unsigned i = 0; i < got; i++) {
        int t = find_tap(argv[2 + i]);
        if (t < 0) {
            bb_reply_errf(BB_ERR_RANGE, "no-such-tap-%s see-SNIFF:TAPS?", argv[2 + i]);
            return;
        }
        /* A capture channel cannot straddle FLEXIO1 and FLEXIO2: they are
         * separate peripherals with separate timers, so one shifter cannot be
         * clocked by a pin on the other. Refused here rather than producing a
         * capture where half the channels are unrelated to the clock. */
        if (i == 0u) {
            instance = s_taps[t].flexio;
        } else if (s_taps[t].flexio != instance) {
            bb_reply_errf(BB_ERR_RANGE,
                          "taps-must-share-one-flexio %s-is-flexio%u",
                          argv[2 + i], (unsigned)s_taps[t].flexio);
            return;
        }
        idx[i] = (uint8_t)t;
    }

    s.proto     = proto;
    s.tap_count = (uint8_t)got;
    memcpy(s.taps, idx, sizeof s.taps);
    s.configured = false;
    bb_reply_ok();
}

static void cmd_taps_q(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    bb_io_puts("OK");
    for (unsigned i = 0; i < BB_SNIFF_TAP_COUNT; i++) {
        bb_io_printf(" %s=%s/flexio%u.%u", s_taps[i].label,
                     s_taps[i].pin.gpio != NULL ? "pad" : "pad",
                     (unsigned)s_taps[i].flexio, (unsigned)s_taps[i].signal);
    }
    bb_io_puts("\r\n");
}

/*
 * The sample rate is a capture parameter, not a constant, and it is worth being
 * explicit about the trade it makes. Sampling is uniform, so the rate sets both
 * the timing resolution of every edge and how fast the ring fills. Oversampling
 * a 400 kHz I2C bus 16x resolves edges to ~150 ns and fills 192 KB in a few
 * milliseconds of busy traffic; oversampling 4x stretches the capture window and
 * blurs the edges. Neither is right for every job, so the host chooses.
 *
 * Giving only the bus rate picks a default oversample, which is what a caller
 * who has not thought about it wants. Giving both is for a caller who has.
 */
static void cmd_cm7_start(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    switch (bb_link_start_cm7()) {
    case BB_CM7_STARTED:
        bb_reply_okf("started %s", bb_link_status());
        break;
    case BB_CM7_ALREADY_UP:
        bb_reply_okf("already-running %s", bb_link_status());
        break;
    case BB_CM7_STARTED_NO_ELE:
        /* Running, so this is a success - but the enclave stayed silent, and
         * that is worth reporting rather than smoothing over. */
        bb_reply_okf("started-without-ele-ack %s", bb_link_status());
        break;
    case BB_CM7_NO_IMAGE:
        bb_reply_err(BB_ERR_UNSUPPORTED,
                     "no-capture-image-in-this-build run-scripts-build.ps1-all");
        break;
    case BB_CM7_COPY_FAILED:
        /* The ITCM window did not read back what was written. Either the alias
         * is wrong for this part or that memory is not accessible from here -
         * both are worth naming, because neither is a capture-core problem. */
        bb_reply_err(BB_ERR_HAL, "itcm-window-readback-mismatch");
        break;
    case BB_CM7_ELE_TIMEOUT: {
        uint32_t tsr = 0u, rsr = 0u;
        bb_link_ele_state(&tsr, &rsr);
        bb_reply_errf(BB_ERR_TIMEOUT, "ele-did-not-answer tsr=0x%08lX rsr=0x%08lX",
                      (unsigned long)tsr, (unsigned long)rsr);
        break;
    }
    default:
        /* Released but silent: almost always an empty or wrong-addressed ITCM.
         * Named precisely because the fix differs - the image has to be loaded
         * to 0x00000000 with the CM7 connect script, and SNIFF:LINK? will show
         * whether the core is executing at all. */
        bb_reply_err(BB_ERR_HAL, "released-but-no-handshake see-SNIFF:LINK?");
        break;
    }
}

/*
 * The release path's own registers. Separate from SNIFF:LINK? because that
 * answers "is the capture core talking" and this answers "was it ever let go",
 * and conflating them produced a reply nobody could read.
 */
/*
 * Is the enclave talking to us at all?
 *
 * Worth its own command because "the capture core did not start" and "the
 * enclave never answered" were indistinguishable for several rounds, and they
 * need completely different fixes. A ping that works proves the mailbox, the
 * MU instance and the response handling are all sound, which narrows the next
 * question to the core itself.
 */
static void cmd_ele_q(int argc, char **argv)
{
    uint32_t fw_status = 0u;
    status_t ping      = ELE_BaseAPI_Ping(MU_RT_S3MUA);
    status_t sts       = ELE_BaseAPI_GetFwStatus(MU_RT_S3MUA, &fw_status);

    (void)argc;
    (void)argv;
    bb_reply_okf("ping=%s fw_status_read=%s fw_status=0x%08lX",
                 (ping == kStatus_Success) ? "ok" : "no-answer",
                 (sts  == kStatus_Success) ? "ok" : "no-answer",
                 (unsigned long)fw_status);
}

static void cmd_cm7_q(int argc, char **argv)
{
    uint32_t before = 0u, after = 0u, scr = 0u, w0 = 0u, w1 = 0u;
    uint32_t tsr = 0u, rsr = 0u;

    (void)argc;
    (void)argv;
    uint32_t root_hz = 0u;
    int      apc     = -1;

    bb_link_m7_state(&before, &after, &scr, &w0, &w1);
    bb_link_ele_state(&tsr, &rsr);
    bb_link_m7_clock(&root_hz, &apc);

    /* WAIT is bit 4 of M7_CFG and INITVTOR is bits 31:7 holding the vector
     * address shifted right by 7. Decoded rather than left as a raw word,
     * because the whole question is whether one particular bit changed. */
    bb_reply_okf("m7cfg_before=0x%08lX m7cfg_after=0x%08lX "
                 "wait_before=%lu wait_after=%lu initvtor=0x%08lX "
                 "src_scr=0x%08lX itcm_w0=0x%08lX itcm_w1=0x%08lX "
                 "ele_tsr=0x%08lX ele_rsr=0x%08lX "
                 "m7_root_hz=%lu apc_status=%d",
                 (unsigned long)before, (unsigned long)after,
                 (unsigned long)((before >> 4) & 1u),
                 (unsigned long)((after  >> 4) & 1u),
                 (unsigned long)((after >> 7) << 7),
                 (unsigned long)scr,
                 (unsigned long)w0, (unsigned long)w1,
                 (unsigned long)tsr, (unsigned long)rsr,
                 (unsigned long)root_hz, apc);
}

static void cmd_link_q(int argc, char **argv)
{
    uint32_t magic = 0u, alive = 0u, t1 = 0u, t2 = 0u;

    (void)argc;
    (void)argv;
    bb_link_raw(&magic, &alive, &t1);
    /* Sampled twice with a gap: a counter that is merely non-zero could be left
     * over from a previous run, and only a *moving* one proves the capture core
     * is executing now. */
    HAL_Delay(2u);
    bb_link_raw(NULL, NULL, &t2);

    /* The beacon is sampled twice as well, and it is the one that settles the
     * ambiguous case: a moving beacon with a still mailbox means the core is
     * alive and cannot see the mailbox, which is a cache or MPU problem rather
     * than a startup one. */
    uint32_t bmagic = 0u, b1 = 0u, b2 = 0u, saw = 0u;

    bb_link_beacon(&bmagic, &b1, &saw);
    HAL_Delay(2u);
    bb_link_beacon(NULL, &b2, NULL);

    /*
     * "No clock" is a different answer from "no beacon", and the verdict says
     * which. A build with no embedded capture image never starts that core, so
     * its beacon is not quiet - it does not exist, and the zeros reported above
     * are this function declining to read unclocked memory rather than a
     * measurement. Reporting that as "cm7-halted-or-never-started" would be
     * true but useless; the actionable fact is that this image cannot start it.
     */
    bool clocked     = bb_link_cm7_clocked();
    bool beacon_live = clocked && (bmagic == 0x43374245u) && (b2 != b1);

    bb_reply_okf("magic=0x%08lX expect=0x42425254 alive=%lu ticks=%lu delta=%lu "
                 "beacon=0x%08lX bticks=%lu bdelta=%lu saw_magic=%lu phase=0x%02lX "
                 "verdict=%s",
                 (unsigned long)magic, (unsigned long)alive,
                 (unsigned long)t2, (unsigned long)(t2 - t1),
                 (unsigned long)bmagic, (unsigned long)b2,
                 (unsigned long)(b2 - b1), (unsigned long)saw,
                 (unsigned long)bb_link_beacon_phase(),
                 (magic != 0x42425254u) ? "controller-did-not-init-mailbox"
                 : (alive != 0u)        ? "up"
                 : (t2 != t1)           ? "cm7-running-but-not-handshaking"
                 : beacon_live          ? "cm7-running-but-cannot-see-mailbox"
                 : !clocked             ? "cm7-unclocked-no-image-in-this-build"
                                        : "cm7-halted-or-never-started");
}

static void cmd_rate(int argc, char **argv)
{
    uint32_t bus_hz, sample_hz = 0u;

    if (!bb_parse_u32(argv[1], &bus_hz) || bus_hz < 1000u ||
        bus_hz > 50000000u) {
        bb_reply_err(BB_ERR_RANGE, "bus-hz-1k..50M");
        return;
    }
    if (argc > 2 && (!bb_parse_u32(argv[2], &sample_hz) ||
                     sample_hz < 100000u || sample_hz > 100000000u)) {
        bb_reply_err(BB_ERR_RANGE, "sample-hz-100k..100M");
        return;
    }
    s.nominal_hz = bus_hz;
    s.sample_hz  = sample_hz;
    s.configured = false;
    bb_reply_ok();
}

static void cmd_rate_q(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    /* `sample_hz=0` means "derived", and the achieved rate is not known until
     * the capture core has programmed the divider - SNIFF:STAT? reports what it
     * actually got, which can differ because the divider is an integer. */
    bb_reply_okf("bus_hz=%lu sample_hz=%lu note=stat-reports-achieved-rate",
                 (unsigned long)s.nominal_hz, (unsigned long)s.sample_hz);
}

static void cmd_start(int argc, char **argv)
{
    /* Optional protocol and STRICT, exactly as the STM32 accepted them, so an
     * existing host script keeps working without learning SNIFF:CH. */
    for (int i = 1; i < argc; i++) {
        bb_proto_t p;
        if (bb_streq_ci(argv[i], "strict")) {
            s.strict = true;
            s.configured = false;
        } else if (parse_proto(argv[i], &p)) {
            /*
             * A protocol *change* re-derives the taps; repeating the current
             * one does not.
             *
             * The count differs per protocol - SPI needs four where I2C needs
             * two - so switching without re-deriving would configure a channel
             * that cannot decode. Re-deriving unconditionally is what the host
             * library made happen, because bb_sniff_start() always names the
             * protocol: `SNIFF:CH i3c d8 d9` followed by the GUI's Start
             * silently became the seven default taps, tap 0 stopped being the
             * bus clock, and a pin-clocked capture of a working bus returned
             * `0 start(s) 0 byte(s)`. An explicit channel selection should
             * survive a start that changes nothing.
             */
            uint8_t  def[8];
            unsigned n = proto_default_taps(p, def);

            if ((p != s.proto) && (n != 0u)) {
                s.tap_count = (uint8_t)n;
                memcpy(s.taps, def, sizeof s.taps);
                s.configured = false;
            }
            if (p != s.proto) {
                s.proto      = p;
                s.configured = false;
            }
        } else {
            bb_reply_errf(BB_ERR_RANGE, "unexpected-%s", argv[i]);
            return;
        }
    }
    /*
     * A self trace arms the controller-side ring and stops here.
     *
     * No taps, no FlexIO, no capture core - that is the whole point of the
     * source. Pushing a configuration to the CM7 and starting it would claim
     * tap pads this capture never reads and leave a second engine running
     * behind a trace nobody drains from it, which is a good way to produce two
     * answers to one question.
     *
     * The protocol argument is still parsed above, because SNIFF:READ? uses
     * s.proto to say what the ninth bit means.
     */
    if (s.src_self) {
        bb_trace_clear();
        bb_trace_arm();
    }
    if (!s.src_wire) {
        /* No capture core involved: no taps, no FlexIO, nothing to start. */
        bb_reply_ok();
        return;
    }
    if (!s.configured) {
        bb_cap_err_t st = push_config();
        if (!cap_reply(st, "config")) {
            return;
        }
        s.configured = true;
    }
    if (!cap_reply(bb_link_request(BB_MSG_START, NULL, 0u, NULL, 0u, NULL),
                   "start")) {
        return;
    }
    bb_reply_ok();
}

static void cmd_stop(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    if (s.src_self) {
        /* Disarm only. What was recorded stays recorded, so the usual
         * stop-then-drain order works the same way it does for a wire
         * capture. */
        bb_trace_disarm();
    }
    if (!s.src_wire) {
        bb_reply_ok();
        return;
    }
    if (cap_reply(bb_link_request(BB_MSG_STOP, NULL, 0u, NULL, 0u, NULL), "stop")) {
        bb_reply_ok();
    }
}

static void cmd_clear(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    if (s.src_self) {
        bb_trace_clear();
    }
    if (!s.src_wire) {
        bb_reply_ok();
        return;
    }
    if (cap_reply(bb_link_request(BB_MSG_CLEAR, NULL, 0u, NULL, 0u, NULL), "clear")) {
        bb_reply_ok();
    }
}

/*
 * One token, in the format the host library has parsed since the STM32 build.
 *
 * Shared by the wire path and the self trace deliberately: the two sources
 * differ in what their records *mean*, which is what `observed=` in the reply
 * header is for, and not at all in how a byte is spelled. Two formatters would
 * be two places for that spelling to drift, and a host cannot tell a
 * drifted token from a wrong one.
 */
/*
 * One reply line, built in a buffer and written once.
 *
 * Honest about its own history, because the comment that was here first was
 * wrong. SNIFF:TRACE? took the board down at about fifty events - it would
 * still enumerate and would not open - and this buffering was written as the
 * fix. It was not the fault. The fault was a stack overflow in the peek that
 * fed this loop, recorded at bb_trace_peek(), and it is what the reset was
 * clearing each time.
 *
 * The buffering is kept on its own merits, which are a hazard rather than a
 * diagnosis: bb_usb_write() sends every call as its own bulk transfer, waits
 * for the previous one, and gives up silently - dropping the rest - if the
 * host has not drained within its spin. A reply emitted one token at a time
 * is hundreds of tiny transfers, and a tail dropped that way takes the newline
 * with it, which is the exact shape of the reply-line truncation this project
 * already paid for once. That path has not been shown to fire here; it has
 * been removed instead of measured, which is the cheaper of the two.
 *
 * So: append into one buffer, stop cleanly when it is full, write once. The
 * caller is told how many events it actually got, which is what `more=` and
 * `pending=` are for.
 *
 * Not reentrant, and does not need to be: every command runs to completion in
 * the single command loop before the next is read.
 */
static char   s_line[BB_REPLY_TEXT_MAX];
static size_t s_line_n;
static size_t s_line_cap;

/**
 * @brief Start a reply, holding @p reserve characters back for the end.
 *
 * The reserve is for whatever has to be said *after* the body - and it has to
 * be held back rather than hoped for. Without it the marker that says the line
 * was cut is itself the thing that does not fit, which is how a reply came
 * back one token short with nothing to say so: exactly the failure the marker
 * exists to report, arriving through the marker.
 */
static void line_reset(size_t reserve)
{
    s_line_n   = 0u;
    s_line_cap = sizeof s_line - reserve;
}

/** Append, and say whether it fitted. A false return ends the loop. */
static bool line_add(const char *s)
{
    size_t n = strlen(s);

    if ((s_line_n + n) >= s_line_cap) {
        return false;
    }
    memcpy(&s_line[s_line_n], s, n);
    s_line_n += n;
    return true;
}

/** Append into the reserve. For the trailing note only. */
static void line_add_reserved(const char *s)
{
    size_t n = strlen(s);

    if ((s_line_n + n) < sizeof s_line) {
        memcpy(&s_line[s_line_n], s, n);
        s_line_n += n;
    }
}

static bool line_addf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static bool line_addf(const char *fmt, ...)
{
    /* Big enough for the longest header any caller formats in one go. The
     * first version used 48 and silently cut SNIFF:TRACE?'s header mid-field,
     * which is a smaller version of the same sin as everything else here. */
    char    tmp[160];
    va_list ap;

    va_start(ap, fmt);
    int n = vsnprintf(tmp, sizeof tmp, fmt, ap);
    va_end(ap);
    if (n < 0) {
        return false;
    }
    return line_add(tmp);
}

static void line_flush(void)
{
    /* One write for the body, one for the terminator. Two transfers for a
     * whole reply rather than one per token, which is the entire point. */
    if (s_line_n != 0u) {
        bb_io_write(s_line, s_line_n);
    }
    bb_io_puts("\r\n");
}

/*
 * One token, in the format the host library has parsed since the STM32 build.
 *
 * Shared by the wire path and the self trace deliberately: the two sources
 * differ in what their records *mean*, which is what `observed=` in the reply
 * header is for, and not at all in how a byte is spelled. Two formatters would
 * be two places for that spelling to drift, and a host cannot tell a drifted
 * token from a wrong one.
 *
 * The token carries no separator of its own. SNIFF:READ? wants " S";
 * SNIFF:TRACE? wants "71276=S", because a space between the timestamp and the
 * token splits one field into two and any host counting whitespace-separated
 * fields reads the time as an event. That is not hypothetical - the first
 * version baked the space in, and the test that was supposed to check every
 * data byte was flagged derived passed while examining empty strings.
 */
static bool emit_token(uint8_t kind, uint8_t value, uint8_t flag)
{
    switch ((bb_ev_kind_t)kind) {
    case BB_EV_START:   return line_add("S");
    case BB_EV_RESTART: return line_add("Sr");
    case BB_EV_STOP:    return line_add("P");
    case BB_EV_DROPPED: return line_add("!");
    case BB_EV_CCC:
        return line_addf("C%02X%c", (unsigned)value, flag ? '+' : '-');
    case BB_EV_ADDR:
    case BB_EV_DATA:
        /* Four characters exactly - the host checks the length before it
         * parses the hex, so a three- or five-character token is silently
         * skipped rather than reported. */
        return line_addf("%c%02X%c",
                         (kind == (uint8_t)BB_EV_ADDR) ? 'A' : 'D',
                         (unsigned)value, flag ? '+' : '-');
    default:
        return true;
    }
}

/*
 * SNIFF:TRACE? - the controller-side trace with its timestamps, peeked.
 *
 * SNIFF:READ? gives the same events as tokens and CONSUMES them; this does not
 * consume, so the normal order is to look here and then drain there. Peeking
 * rather than draining is what lets the two coexist on one ring without a host
 * having to know which command ate its events.
 *
 * Times go out as raw counter deltas with the counter frequency beside them,
 * not as nanoseconds. That is the same choice bb_capture.h makes for
 * ps_per_tick and for the same reason: a converted figure hides the resolution
 * it was converted at, and this counter is SysTick-derived rather than a
 * free-running peripheral - its resolution is a property of this board, not of
 * the format. The delta is from the arm, is 32-bit, and wraps after 2^32
 * counts: about 17.9 s at 240 MHz.
 *
 * `~` after a token marks a flag this side computed rather than observed. On
 * I3C that is every data T-bit and no address ACK. bb_trace.h says why that
 * distinction is worth a character.
 */
static void cmd_trace_q(int argc, char **argv)
{
    uint32_t want = 64u;

    if (argc > 1 && !bb_parse_u32_max(argv[1], 256u, &want)) {
        bb_reply_err(BB_ERR_RANGE, "max-1..256");
        return;
    }
    if (!s.src_self) {
        bb_reply_err(BB_ERR_UNSUPPORTED,
                     "trace-is-controller-side-see-SNIFF:SRC-self-or-both");
        return;
    }

    unsigned pending = bb_trace_pending();
    unsigned n       = (want < pending) ? (unsigned)want : pending;
    uint32_t t0      = bb_trace_t0();

    /*
     * t0 absolutely, not just as the origin the deltas are from.
     *
     * The deltas alone cannot be placed against anything else on this board.
     * A merge with a wire capture needs the controller's events in the same
     * absolute bb_cycles() frame that SYS:SYNC brackets its markers in, and
     * that frame is t0 + delta. Without this the two streams can each be
     * internally consistent and still have no common point.
     */
    /* 16 held back for " truncated=1". */
    line_reset(16u);
    (void)line_addf("OK n=%u pending=%u cycle_hz=%lu armed=%u lost=%lu t0=%lu",
                    n, pending, (unsigned long)bb_cycle_hz(),
                    bb_trace_armed() ? 1u : 0u,
                    (unsigned long)bb_trace_lost(), (unsigned long)t0);

    unsigned done = 0u;
    bool     full = false;
    while ((done < n) && !full) {
        bb_trace_ev_t ev[32];
        unsigned      take = ((n - done) < 32u) ? (n - done) : 32u;
        /* The window starts where the last batch ended. Asking for
         * `done + take` into a 32-entry array is what wrote past the end of
         * it once `done` reached 32 - see bb_trace_peek(). */
        unsigned      got  = bb_trace_peek(ev, take, done);

        if (got == 0u) {
            break;
        }
        for (unsigned i = 0; i < got; i++) {
            /* Checked at the start of the mark, not after it: a timestamp
             * that fitted and a token that did not would print "71276=" as a
             * field of its own, which parses as an event with no kind. */
            size_t mark = s_line_n;

            if (!line_addf(" %lu=", (unsigned long)(ev[i].t_cyc - t0))
                || !emit_token(ev[i].kind, ev[i].value, ev[i].flag)
                || (ev[i].derived && !line_add("~"))) {
                s_line_n = mark;
                full = true;
                break;
            }
        }
        done += got;
    }
    if (full) {
        /* Said rather than left to be inferred from a count that is short.
         * The ring is untouched - this command peeks - so asking for fewer
         * gets the rest. */
        line_add_reserved(" truncated=1");
    }
    line_flush();
}

static void cmd_stat_q(int argc, char **argv)
{
    if (self_trace()) {
        /*
         * A self trace has no capture core behind it, so the wire engine's
         * counters would all be stale or zero. Reporting them would be worse
         * than useless: `entries=0 overruns=0` reads as a healthy capture that
         * happened to see nothing, which is exactly the wrong conclusion to
         * invite from a source that is working perfectly.
         */
        uint32_t span = bb_cycles() - bb_trace_t0();
        uint32_t hz   = bb_cycle_hz();
        uint32_t us   = (hz != 0u)
                      ? (uint32_t)(((uint64_t)span * 1000000u) / hz) : 0u;

        bb_reply_okf("run=%u src=%s proto=%s events=%lu pending=%u "
                     "lost=%lu span_us=%lu cycle_hz=%lu ring=%u",
                     bb_trace_armed() ? 1u : 0u, src_name(),
                     bb_proto_name(s.proto),
                     (unsigned long)bb_trace_total(), bb_trace_pending(),
                     (unsigned long)bb_trace_lost(), (unsigned long)us,
                     (unsigned long)hz, (unsigned)BB_TRACE_RING);
        return;
    }
    bb_stat_t    st;
    size_t       n  = 0u;
    bb_cap_err_t rc;

    (void)argc;
    (void)argv;
    rc = bb_link_request(BB_MSG_STAT, NULL, 0u, &st, sizeof st, &n);
    if (!cap_reply(rc, "stat")) {
        return;
    }
    if (n < sizeof st) {
        bb_reply_err(BB_ERR_HAL, "stat-short-reply");
        return;
    }
    /* `full` and `gaps` are reported rather than smoothed over. When the ring
     * fills, capture stops and a gap record is written - it does not wrap,
     * because a ring that overwrote its oldest entries would silently move
     * where the capture began. */
    /*
     * queued, starts, bytes and dropped are the names bb_sniff_status() parses,
     * and they are aliases onto figures this engine already keeps rather than
     * new counters: queued is the entries waiting, bytes is those entries in
     * bytes, dropped is the samples discarded as unchanged, and starts is the
     * records produced. The descriptive names stay alongside them, because
     * `dropped=217174` on an idle bus is alarming until you know it means
     * "compressed away".
     */
    /*
     * The frame-close log goes out as a comment line of its own, ahead of the
     * reply. bb_reply_okf() formats and *sends*; anything printed after it
     * lands outside the reply and is lost, which is how this arrived empty the
     * first time.
     */
    if (st.close_n != 0u) {
        bb_io_printf("# closes=%u", (unsigned)st.close_n);
        for (unsigned k = 0; k < st.close_n && k < 8u; k++) {
            bb_io_printf(" %08lX", (unsigned long)st.close_log[k]);
        }
        bb_io_puts("\r\n");
    }
    /*
     * The trace's counters go out as their own comment line AHEAD of the
     * reply, for the reason written three lines above about the close log:
     * bb_reply_okf() formats and *sends*, so anything printed after it is not
     * part of that reply - it is the next line the host reads, and every
     * reply after it is answered by the one before. This was written after it
     * for exactly one build, and the symptom was SNIFF:STAT? answering with
     * the previous command's reply.
     *
     * Separate from the reply rather than folded into it, too: `src=` there
     * is what the CAPTURE CORE says about its own records, and it is always
     * wire because that is all it can observe. Mixing controller-side figures
     * into that line would put the two provenances in one place with nothing
     * to tell them apart.
     */
    if (s.src_self) {
        bb_io_printf("# trace sel=%s armed=%u events=%lu pending=%u lost=%lu"
                     " t0=%lu cycle_hz=%lu",
                     src_name(), bb_trace_armed() ? 1u : 0u,
                     (unsigned long)bb_trace_total(), bb_trace_pending(),
                     (unsigned long)bb_trace_lost(),
                     (unsigned long)bb_trace_t0(),
                     (unsigned long)bb_cycle_hz());
        bb_io_puts("\r\n");
    }
    bb_reply_okf("run=%u proto=%s src=%s full=%u entries=%lu records=%lu "
                 "queued=%lu starts=%lu bytes=%lu dropped=%lu "
                 "pending=%lu gaps=%lu coincident=%lu ps_per_tick=%lu "
                 "sample_hz=%lu overruns=%lu core_hz=%lu "
                 "polls=%lu words=%lu cycpoll=%lu cycword=%lu cycmax=%lu "
                 "defer=%u arm=%u framebits=%u startat=%lu stopat=%lu "
                 "bothpend=%u reorder=%u availmax=%lu hwstart=%lu hwstop=%lu "
                 "setupstart=%u hwstopn=%lu",
                 (unsigned)st.running, bb_proto_name((bb_proto_t)st.proto),
                 bb_observed_name((bb_observed_t)st.source),
                 (unsigned)st.full,
                 (unsigned long)st.entries, (unsigned long)st.records,
                 (unsigned long)(st.bytes_pending / sizeof(bb_sample_t)),
                 (unsigned long)st.records,
                 (unsigned long)st.bytes_pending,
                 (unsigned long)st.coincident,
                 (unsigned long)st.bytes_pending, (unsigned long)st.gaps,
                 (unsigned long)st.coincident,
                 (unsigned long)st.ps_per_tick,
                 (unsigned long)st.sample_hz, (unsigned long)st.overruns,
                 (unsigned long)st.core_hz,
                 (unsigned long)st.prof_polls, (unsigned long)st.prof_words,
                 (unsigned long)st.prof_cyc_poll,
                 (unsigned long)st.prof_cyc_word,
                 (unsigned long)st.prof_cyc_max,
                 (unsigned)st.defer_bits, (unsigned)st.arm_bits,
                 (unsigned)st.frame_bits,
                 (unsigned long)st.start_at, (unsigned long)st.stop_at,
                 (unsigned)st.bothpend_n, (unsigned)st.reorder_n,
                 (unsigned long)st.dma_avail_max,
                 (unsigned long)st.starts_seen,
                 (unsigned long)st.stops_seen,
                 (unsigned)st.setup_starts,
                 (unsigned long)st.stops_noted);
}

/*
 * Decoded events, as the protocol's own tokens.
 *
 * This command had this meaning before this port existed, and an earlier version
 * of this file took it over for raw capture records - which broke every host
 * built against the protocol, because bb_sniff_read() parses `S`, `Sr`, `P`,
 * `!` and `A`/`D` with a hex byte and an ack flag. The spec is explicit about
 * this and I had ignored it: "New capability arrives as *additional* commands,
 * never as changes to existing ones." So the raw records moved to SNIFF:RAW?
 * and this went back to what it always meant.
 *
 * The two are the same capture seen two ways, from one pass over one sample
 * stream - so a host can read the transaction here and the waveform there and
 * they cannot disagree.
 */

/*
 * Drain the controller-side ring in the same shape a wire read has.
 *
 * The tokens are identical; everything that differs is in the header, and all
 * of it is about provenance:
 *
 *   observed=controller   these are our peripheral's records, not the bus's
 *   tbits=computed        the ninth bit of a data byte was worked out from the
 *                         byte, not seen - see bb_trace.h, which is emphatic
 *                         about why a check that cannot fail is worse than no
 *                         check at all
 *   addrack=observed      the one flag here that is real
 *   lost=N                events the ring dropped; each run of them also
 *                         appears in the stream as a `!`
 */
/*
 * Events per reply, bounded by the line rather than by the ring.
 *
 * Every token costs at most five characters - a space and four for `D40+` or
 * `C8E+` - and the header is under a hundred. (1152 - 100) / 5 = 210, so 200
 * is the promise with room left over. The bound belongs on what the command
 * is willing to offer, not on what it prints: a drain advances a cursor, so
 * anything cut at print time is gone for good. That is the same reasoning as
 * BB_REPLY_BYTES_MAX and the same bug it was written for.
 */
#define READ_SELF_MAX  200u

static void read_self(uint32_t want)
{
    unsigned pending = bb_trace_pending();
    unsigned n       = (want < pending) ? (unsigned)want : pending;

    if (n > READ_SELF_MAX) {
        n = READ_SELF_MAX;
    }

    /*
     * `more=` goes LAST, which is not cosmetic.
     *
     * Every host here splits a reply on "more=" and treats the first
     * space after it as the start of the tokens - acceptance.py's
     * clean() is the canonical one - so any field placed between
     * "more=" and the tokens has to be known to that splitter by name
     * or it parses as a token. The wire path got away with it because
     * its two extra fields were added to clean() at the same time.
     * Four new ones would have been four more strings to keep in step
     * across two trees, and the failure when they drift is a stray
     * "lost=0" read as a bus event. Ordering the header so the
     * existing splitter works untouched costs nothing and removes the
     * chance entirely.
     */
    /* 4 held back for a " !" that should never be needed. */
    line_reset(4u);
    (void)line_addf("OK n=%u observed=controller tbits=computed "
                    "addrack=observed lost=%lu%s more=%u",
                    n, (unsigned long)bb_trace_lost(),
                    (s.proto == BB_PROTO_I3C) ? " framing=i3c-t-bit-not-ack" : "",
                    (unsigned)(pending - n));
    /*
     * One space, always, even when no tokens follow.
     *
     * clean() takes everything from the first space after "more=", so
     * a reply that ends at "more=0" has no such space and the splitter
     * hands back the count itself - "0" parsed as a bus event. The
     * wire path never hits this because its framing note always
     * follows. An empty drain is the ordinary end of every capture, so
     * it is not an edge case worth leaving to the host to notice.
     */
    (void)line_add(" ");

    /* Chunked rather than drained whole: 256 events is 2 KB, which is a lot of
     * stack for a batch that is appended as it goes anyway. */
    while (n > 0u) {
        bb_trace_ev_t ev[32];
        unsigned      take = (n < 32u) ? n : 32u;
        unsigned      got  = bb_trace_drain(ev, take);

        if (got == 0u) {
            break;
        }
        for (unsigned i = 0; i < got; i++) {
            if (!line_add(" ")
                || !emit_token(ev[i].kind, ev[i].value, ev[i].flag)) {
                /*
                 * The buffer filled and these events are already out of the
                 * ring - a drain cannot be taken back. Saying so is the only
                 * honest option left: a short line with no warning is exactly
                 * the silent cut this mechanism was rebuilt to stop.
                 *
                 * It should not be reachable, because READ_SELF_MAX bounds n
                 * so the worst case fits. This is here because that bound is
                 * an argument and this is a fact.
                 */
                line_add_reserved(" !");
                line_flush();
                return;
            }
        }
        n -= got;
    }
    line_flush();
}

static void cmd_read_q(int argc, char **argv)
{
    uint8_t      buf[BB_MSG_MAX_PAYLOAD];
    size_t       n    = 0u;
    uint32_t     want = 64u;
    bb_cap_err_t rc;

    if (argc > 1 && !bb_parse_u32_max(argv[1], 256u, &want)) {
        bb_reply_err(BB_ERR_RANGE, "max-1..256");
        return;
    }
    if (self_trace()) {
        read_self(want);
        return;
    }
    rc = bb_link_request(BB_MSG_EVENTS, &want, sizeof want, buf, sizeof buf, &n);
    if (!cap_reply(rc, "read")) {
        return;
    }
    if (n < sizeof(bb_events_hdr_t)) {
        bb_reply_err(BB_ERR_HAL, "events-short-reply");
        return;
    }

    const bb_events_hdr_t *hdr = (const bb_events_hdr_t *)buf;
    const bb_event_t      *ev  = (const bb_event_t *)(buf + sizeof *hdr);
    unsigned               cnt = hdr->count;

    if ((sizeof *hdr + (size_t)cnt * sizeof(bb_event_t)) > n) {
        bb_reply_err(BB_ERR_HAL, "events-count-past-end");
        return;
    }

    /*
     * A pin-clocked capture says why it has no events rather than just having
     * none.
     *
     * The mode persists across commands, as proto and taps do, so a host that
     * has never heard of SNIFF:MODE can meet a board someone left pin-clocked
     * and get a clean, empty, entirely unexplained answer. Naming the reason in
     * a field the existing parser already ignores costs nothing and turns a
     * silent nothing into a pointer at SNIFF:RAW?.
     */
    /*
     * Say what the ninth bit of a data byte means, because it is not the same
     * thing on both protocols.
     *
     * I2C's ninth bit is an acknowledge driven by the *receiver*, so `+` means a
     * device answered. I3C SDR replaces it with a T-bit driven by the
     * *transmitter* - parity on a write, an end-of-data signal on a read - so the
     * same `+` means something entirely different and nothing acknowledged
     * anything. The byte values are unaffected, since both framings are nine bits
     * and the decoder recovers the first eight identically; it is only the flag
     * that changes meaning.
     *
     * Reported rather than re-rendered: the token format is the host library's
     * contract and a different one for I3C would break every existing parser.
     * Naming the framing in a field they already ignore says the true thing
     * without changing what they read.
     */
    /*
     * Buffered like the self path. This loop is bounded by the capture core's
     * message payload - 480 bytes is 120 events - and has never misbehaved,
     * so this is precaution, not repair: 120 events emitted one write at a
     * time is 240 bulk transfers for one reply, against a bb_usb_write() that
     * drops what it cannot hand over. Changed while the two paths were being
     * made to share a formatter anyway.
     */
    line_reset(4u);
    (void)line_addf("OK n=%u more=%u%s%s", cnt, (unsigned)hdr->more,
                    (s.proto == BB_PROTO_I3C) ? " framing=i3c-t-bit-not-ack" : "",
                    (s.mode == BB_CAP_PINCLOCK)
                        ? " note=bits-exact-tail-truncates-under-8-bits" : "");
    for (unsigned i = 0; i < cnt; i++) {
        if (!line_add(" ") || !emit_token(ev[i].kind, ev[i].value, ev[i].ack)) {
            line_add_reserved(" !");
            break;
        }
    }
    line_flush();
}

static void cmd_raw_q(int argc, char **argv)
{
    uint8_t      buf[BB_MSG_MAX_PAYLOAD];
    size_t       n = 0u;
    /* Bounded by the reply line, not by the link message - see RAW_MAX_RECS.
     * A full BB_MSG_MAX_PAYLOAD of container bytes prints as 1442 characters
     * and the host keeps 1199 of them. */
    uint32_t     want = BB_REPLY_BYTES_MAX;
    bb_cap_err_t rc;

    if (argc > 1 && !bb_parse_u32_max(argv[1], BB_REPLY_BYTES_MAX, &want)) {
        bb_reply_errf(BB_ERR_RANGE, "max-1..%u", (unsigned)BB_REPLY_BYTES_MAX);
        return;
    }
    rc = bb_link_request(BB_MSG_READ, &want, sizeof want, buf, sizeof buf, &n);
    if (!cap_reply(rc, "read")) {
        return;
    }
    /* Raw record bytes as hex. The host reassembles them into the capture
     * container - this end deliberately does no interpreting, because the whole
     * point of the format is that a saved capture can be re-decoded later and
     * tested against a golden corpus with no hardware present. */
    bb_reply_ok_bytes(buf, n);
}

static void cmd_pins_q(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    bb_io_printf("OK proto=%s src=%s strict=%u taps=",
                 bb_proto_name(s.proto), src_name(),
                 s.strict ? 1u : 0u);
    if (s.tap_count == 0u) {
        bb_io_puts("none");
    } else {
        for (unsigned i = 0; i < s.tap_count; i++) {
            bb_io_printf("%s%s", (i > 0u) ? "," : "", s_taps[s.taps[i]].label);
        }
    }
    /* Levels are what the STM32's SNIFF:PINS? reported and what separates "my
     * wiring is wrong" from "the bus is idle". A floating tap reads as an
     * arbitrary level, which is normal rather than a fault. */
    bb_io_puts(" levels=");
    for (unsigned i = 0; i < s.tap_count; i++) {
        bb_io_printf("%u", bb_pin_read(&s_taps[s.taps[i]].pin) ? 1u : 0u);
    }
    bb_io_puts("\r\n");
}

static void cmd_test(int argc, char **argv)
{
    /*
     * Pin-clocked mode cannot answer this question, so say so rather than
     * answering it wrongly.
     *
     * The self-test drives one pin from the internal generator and counts what
     * the capture engine saw. Pin-clocked capture spends tap 0 on the bus clock
     * and shifts tap 1 against it, so with one driven pin there is no shift
     * clock and the count comes back low - `seen=63` against 144 driven. That
     * reads like a capture fault and is a mode that has nothing to count.
     */
    if (s.mode == BB_CAP_PINCLOCK) {
        bb_reply_err(BB_ERR_BUS_STATE,
                     "test-needs-oversample-pinclock-has-no-bus-clock-to-shift-on");
        return;
    }

    uint32_t half_ns = 2500u;   /* 200 kHz, the STM32's default self test */

    if (argc > 1 && (!bb_parse_u32(argv[1], &half_ns) || half_ns < 50u)) {
        bb_reply_err(BB_ERR_RANGE, "half-ns-min-50");
        return;
    }
    if (!s.configured) {
        if (!cap_reply(push_config(), "config")) {
            return;
        }
        s.configured = true;
    }
    /* Drives the taps, so it must not be run against a live bus. Said in the
     * reply rather than only in the docs, because the docs are not what someone
     * reads at the moment they type it. */
    bb_selftest_t r;
    size_t        n  = 0u;
    bb_cap_err_t  rc = bb_link_request(BB_MSG_SELFTEST, &half_ns, sizeof half_ns,
                                       &r, sizeof r, &n);
    /*
     * A failed self test reports its measurements too.
     *
     * The capture core fills the whole diagnostic block before it decides pass
     * or fail, and returns it either way - and this used to throw it away on
     * failure and print the bare error code. That is backwards: a self test that
     * passes needs no explanation, and the one that fails is exactly the one
     * whose numbers are worth having. Debugging a four-tap failure meant adding
     * temporary instrumentation to recover figures the reply already carried.
     *
     * One formatter serves both, so the numbers are named identically whichever
     * way the run went and a caller can diff a good run against a bad one. Only
     * the verdict and the wrapper differ.
     */
    bool failed = (rc != BB_CAP_OK);

    if (failed && (!bb_link_up() || n < sizeof r)) {
        (void)cap_reply(rc, "selftest");   /* nothing to report but the cause */
        return;
    }
    if (!failed && !cap_reply(rc, "selftest")) {
        return;
    }
    if (n < sizeof r) {
        bb_reply_err(BB_ERR_HAL, "selftest-short-reply");
        return;
    }
    /*
     * Both counts, not a verdict. The capture core has already decided pass or
     * fail - a non-OK status got us out above - but reporting only that would
     * repeat the STM32 build's mistake, where a loopback test passed while
     * capturing a twentieth of the rate it claimed. driven/expected/seen lets a
     * reader see how close it was, and sample_hz says what resolution produced
     * the answer.
     */
    /* pins= is per tap in SNIFF:CH order, 1 where that pad drove high and low
     * and read back through FlexIO's own pin registers. A 0 there means the pad
     * is not reaching FlexIO at all, which no amount of shifter configuration
     * will fix. */
    char pins[10];
    unsigned n_pins = (s.tap_count < 8u) ? s.tap_count : 8u;

    for (unsigned i = 0; i < n_pins; i++) {
        pins[i] = ((r.pin_probe >> i) & 1u) ? '1' : '0';
    }
    pins[n_pins] = '\0';

    /* moved= is per tap, and it is the line to read: the generator drives every
     * tap together, so anything but all ones is a channel the capture cannot
     * see. */
    char moved[10];

    for (unsigned i = 0; i < n_pins; i++) {
        moved[i] = ((r.moved >> i) & 1u) ? '1' : '0';
    }
    moved[n_pins] = '\0';

    char det[BB_PAYLOAD_MAX];

    (void)snprintf(det, sizeof det,
                 "driven=%lu expected=%lu seen=%lu sample_hz=%lu src_hz=%lu frise=%lu ffall=%lu overruns=%lu "
                 "moved=%s pins=%s taps=%u first_pin=%u ticks=%lu pin_live=0x%08lX "
                 "shiftcfg=0x%08lX pwidth=%lu shiftctl=0x%08lX timcmp=0x%08lX "
                 "half_ns=%lu",
                 (unsigned long)r.driven_loads, (unsigned long)r.expected,
                 (unsigned long)r.seen, (unsigned long)r.sample_hz,
                 (unsigned long)r.src_hz,
                 (unsigned long)r.frame_rise, (unsigned long)r.frame_fall,
                 (unsigned long)r.overruns, moved, pins,
                 (unsigned)r.width, (unsigned)r.base_pin,
                 (unsigned long)r.ticks,
                 (unsigned long)r.pin_live,
                 (unsigned long)r.shiftcfg,
                 (unsigned long)((r.shiftcfg >> 16) & 0x1Fu),
                 (unsigned long)r.shiftctl, (unsigned long)r.timcmp,
                 (unsigned long)half_ns);

    if (failed) {
        bb_reply_errf(BB_ERR_HAL, "selftest %s %s", bb_cap_err_name(rc), det);
    } else {
        bb_reply_okf("%s note=drives-the-taps-not-for-a-live-bus", det);
    }
}

static void cmd_bench(int argc, char **argv)
{
    bb_bench_t   b;
    size_t       n = 0u;
    uint32_t     half_ns = 2500u;

    if (argc > 1 && !bb_parse_u32(argv[1], &half_ns)) {
        bb_reply_err(BB_ERR_NUMBER, "half-ns");
        return;
    }
    /*
     * Push a pending configuration first, exactly as SNIFF:TEST does.
     *
     * SNIFF:CH only records the choice locally and marks the configuration
     * dirty; something has to send it. This command did not, so `SNIFF:CH` then
     * `SNIFF:BENCH` on a freshly booted board reached the capture core with no
     * taps and came back "pins-not-flexio-or-already-in-use" - which names a
     * pin problem for what is really an unsent message, and sends the reader to
     * check their wiring.
     *
     * It hid because it only fails on a *fresh* board: any earlier SNIFF:TEST
     * pushes the configuration and every later bench then works. My own first
     * bench measurement was taken that way, after a long run of other commands.
     */
    if (!s.configured) {
        if (!cap_reply(push_config(), "config")) {
            return;
        }
        s.configured = true;
    }

    /*
     * Not meaningful in pin-clocked mode, and say so here.
     *
     * The bench drives one pin from the internal toggle generator and measures
     * what the capture engine makes of it. Pin-clocked capture spends tap 0 on
     * the bus clock and shifts tap 1 against it, so a single driven pin gives
     * the shifters nothing to shift on and the configuration is refused
     * downstream - as `flexio-config-rejected`, which names the wrong
     * subsystem. The FlexIO configuration is not the problem; asking a
     * bus-clocked capture to measure a source with no bus clock is.
     *
     * SNIFF:TEST has the same shape and does not fail, it just reports a low
     * `seen` - 63 against 144 driven - which reads like a capture fault rather
     * than a mode that cannot answer the question. Both are worth saying
     * plainly rather than leaving to be rediscovered.
     */
    if (s.mode == BB_CAP_PINCLOCK) {
        bb_reply_err(BB_ERR_BUS_STATE,
                     "bench-needs-oversample-pinclock-has-no-bus-clock-to-shift-on");
        return;
    }

    bb_cap_err_t rc = bb_link_request(BB_MSG_BENCH, &half_ns, sizeof half_ns,
                                      &b, sizeof b, &n);
    if (!cap_reply(rc, "bench")) {
        return;
    }
    if (n < sizeof b) {
        bb_reply_err(BB_ERR_HAL, "bench-short-reply");
        return;
    }
    /*
     * `from_selftest` is reported because of a specific STM32 lesson: a
     * pass/fail loopback test cannot find the ceiling. Its generator and its
     * capture shared a core, so a bit period shorter than the capture cost
     * simply stretched - it "passed" at a nominal 3.3 MHz while never actually
     * exceeding 202 kHz. A figure measured that way is not a bus rate, and the
     * reply says which kind of figure this is rather than leaving the reader to
     * assume the flattering one.
     */
    /*
     * cost_ns and max_bus_hz are the two fields bb_sniff_bench() parses; the
     * rest is extra and harmless. Both are named rather than positional, which
     * is what lets the reply grow without breaking an existing host - so
     * sample_hz and limited_by are appended rather than substituted.
     *
     * limited_by is the one to read. "The generator could not go faster" and
     * "the sampler could not follow" are different answers, and a caller sizing
     * this against a real bus needs to know which one it got.
     */
    bb_reply_okf("cost_ns=%lu max_bus_hz=%lu dma_bytes_per_s=%lu from_selftest=%u"
                 " sample_hz=%lu limited_by=%s",
                 (unsigned long)b.cost_ns,
                 (unsigned long)b.max_bus_hz,
                 (unsigned long)b.dma_bytes_per_s,
                 (unsigned)b.from_selftest,
                 (unsigned long)b.sample_hz,
                 (b.limited_by == 0u) ? "sampler" : "generator");
}

/* ---- status ------------------------------------------------------------ */

void bb_sniff_status_line(bb_status_t *out)
{
    if (out == NULL) {
        return;
    }
    memset(out, 0, sizeof *out);
    (void)snprintf(out->pins, sizeof out->pins, "%u taps", (unsigned)s.tap_count);
    (void)snprintf(out->detail, sizeof out->detail, "%s/%s %s",
                   bb_proto_name(s.proto), src_name(),
                   bb_link_up() ? "up" : "no-cm7");
    out->state = !bb_link_up() ? BB_STATE_ABSENT
                 : s.configured ? BB_STATE_ACTIVE
                                : BB_STATE_IDLE;
}

/* ---- the edge-list view (SNIFF:RAW:*) ---------------------------------- */
/*
 * A second presentation of the same capture, for the waveform analyser.
 *
 * That analyser wants an *edge list*: arm with a budget, wait, drain five-byte
 * records of a free-running 32-bit tick and a line mask, and be told the tick
 * rate so it can turn those into picoseconds itself. This firmware stores
 * something richer - the capture container, with record headers, CRCs and
 * per-entry deltas - and SNIFF:RAW? hands that out untouched.
 *
 * Neither is wrong and neither subsumes the other: the container is what makes
 * a capture re-decodable later against a golden corpus, and the edge list is
 * what a waveform view actually needs. So this converts between them rather
 * than either side changing format, and the conversion is the only code that
 * has to know both.
 *
 * Deltas become absolute here, because this is where a running total can live
 * across several READ? calls. The analyser is handed a counter that only goes
 * up and wraps at 32 bits, which is exactly what it already unwraps.
 */
#define RAW_REC_BYTES   5u                       /* LE32 tick + u8 lines */
/*
 * Records per READ?, bounded by the reply line rather than by the link message.
 *
 * It used to be BB_MSG_MAX_PAYLOAD / RAW_REC_BYTES - 96 - which is how many
 * records fit in one message to the capture core, and has nothing to do with
 * how many fit in the answer to the host. Ninety-six records print as 1442
 * characters; the host library's line buffer holds 1199 and used to keep that
 * prefix without complaint. So seventeen records per call were formatted,
 * dropped in transit, and never seen again - this command had already advanced
 * its cursor past them, so unlike a short read they could not be re-requested.
 *
 * The damage did not look like loss. The holes fell inside a read transfer, so
 * the edge list stayed well-formed and decoded to plausible, wrong bytes, with
 * two extra framing events where the missing bits had been. That sent the
 * investigation into the protocol analyser, which was faithfully decoding
 * exactly what it had been given.
 */
#define RAW_MAX_RECS    (BB_REPLY_BYTES_MAX / RAW_REC_BYTES)
/* A container record's payload cannot exceed one link message, so this holds
 * any partial record between calls. */
#define RAW_CARRY_MAX   BB_MSG_MAX_PAYLOAD

static uint64_t s_raw_tick;                 /* absolute tick, accumulated */
static uint64_t s_raw_ps_per_tick;          /* to re-base each record */
static uint8_t  s_raw_resumed;              /* carry starts mid-record */
/*
 * The line levels before the first transition, and whether they still need
 * emitting.
 *
 * An edge list says what changed, not what the bus was, so its consumer cannot
 * tell a falling edge from a rising one at the very first entry - and the very
 * first entry of an I3C frame is the START, SDA going low while SCL is held.
 * Captured here the START reads as "SDA is 0", which is a state and not an
 * event, so a decoder finds no START, no frame, and reports nothing at all
 * from a capture whose bits are provably correct.
 *
 * The container already carries the answer: contract 3 puts the initial state
 * in the META record that opens every capture. This lifts it out and emits it
 * as the first edge, one tick ahead of the first real transition, so the list
 * begins with the bus as it was and every edge after it has something to be a
 * change from.
 */
static uint8_t  s_raw_initial;
static bool     s_raw_initial_due;
static uint8_t  s_raw_carry[RAW_CARRY_MAX]; /* container bytes not yet parsed */
static uint16_t s_raw_carry_n;

static void raw_reset(void)
{
    s_raw_tick    = 0u;
    s_raw_carry_n = 0u;
    s_raw_resumed = 0u;
    s_raw_initial = 0u;
    s_raw_initial_due = false;
}

/** The capture clock - what the analyser needs to make time out of ticks. */
static uint32_t raw_tick_hz(void)
{
    bb_stat_t st;
    size_t    n = 0u;

    memset(&st, 0, sizeof st);
    if (bb_link_request(BB_MSG_STAT, NULL, 0u, &st, sizeof st, &n) != BB_CAP_OK) {
        return 0u;
    }
    return st.sample_hz;
}

/*
 * SNIFF:RAW:START [max-edges] - arm for an edge-list capture.
 *
 * Oversampled and undecoded, because that is what an edge list is: every
 * transition on the taps with no protocol opinion applied. Pin-clocked mode
 * samples on the bus clock and would hand the analyser bits rather than edges.
 */
static void cmd_raw_start(int argc, char **argv)
{
    uint32_t max_edges = 0u;

    if (argc > 1 && !bb_parse_u32(argv[1], &max_edges)) {
        bb_reply_err(BB_ERR_NUMBER, "max-edges");
        return;
    }

    s.mode       = BB_CAP_OVERSAMPLE;
    s.configured = false;
    if (!cap_reply(push_config(), "config")) {
        return;
    }
    s.configured = true;

    if (!cap_reply(bb_link_request(BB_MSG_CLEAR, NULL, 0u, NULL, 0u, NULL),
                   "clear")) {
        return;
    }
    raw_reset();
    if (!cap_reply(bb_link_request(BB_MSG_START, NULL, 0u, NULL, 0u, NULL),
                   "start")) {
        return;
    }
    /*
     * The edge-list view is a way of starting a capture, not a different
     * capture, so under `both` it arms the controller trace as well.
     *
     * Without this the only order that worked was SNIFF:START then
     * SNIFF:RAW:START, because this command restarts the capture core and so
     * threw away any markers laid down before it - and a merge aligned on
     * markers from a previous tick origin came out 30 ms wrong while looking
     * like an ordinary fit. Arming here puts the trace's origin before the
     * markers, where it has to be.
     */
    if (s.src_self) {
        bb_trace_clear();
        bb_trace_arm();
    }
    {
        bb_stat_t st;
        size_t    n = 0u;

        memset(&st, 0, sizeof st);
        if (bb_link_request(BB_MSG_STAT, NULL, 0u, &st, sizeof st, &n)
                == BB_CAP_OK) {
            s_raw_ps_per_tick = st.ps_per_tick;
        }
    }
    bb_reply_okf("tick_hz=%lu max_edges=%lu",
                 (unsigned long)raw_tick_hz(), (unsigned long)max_edges);
}

static void cmd_raw_stop(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    if (cap_reply(bb_link_request(BB_MSG_STOP, NULL, 0u, NULL, 0u, NULL),
                  "stop")) {
        bb_reply_ok();
    }
}

/*
 * SNIFF:RAW:STAT? - how many edges are waiting, and whether the ring filled.
 *
 * `edges` has to agree with what READ? will deliver, because the analyser loops
 * until it has that many; claiming more would leave it asking for records that
 * never come, which it then has to report as loss.
 */
static void cmd_raw_stat_q(int argc, char **argv)
{
    bb_stat_t st;
    size_t    n = 0u;

    (void)argc;
    (void)argv;
    memset(&st, 0, sizeof st);
    if (!cap_reply(bb_link_request(BB_MSG_STAT, NULL, 0u, &st, sizeof st, &n),
                   "stat")) {
        return;
    }
    bb_reply_okf("edges=%lu full=%u tick_hz=%lu",
                 (unsigned long)st.entries, (unsigned)(st.full ? 1u : 0u),
                 (unsigned long)st.sample_hz);
}

/*
 * SNIFF:RAW:READ? [records] - drain edges as five-byte records.
 *
 * Container records are pulled from the capture core and converted here. A
 * record can straddle two link messages, so whatever is left over is carried to
 * the next call rather than dropped: resynchronising on the wrong byte would
 * make every edge after it fiction, and fiction that looks like data.
 */
static void cmd_raw_read_q(int argc, char **argv)
{
    uint32_t want = RAW_MAX_RECS;
    uint8_t  out[RAW_MAX_RECS * RAW_REC_BYTES];
    uint32_t got = 0u;
    /* Set when the carry begins with a record this call already part-read, so
     * its base is not applied a second time. */
    uint8_t  resumed = s_raw_resumed;

    if (argc > 1 && !bb_parse_u32_max(argv[1], RAW_MAX_RECS, &want)) {
        bb_reply_errf(BB_ERR_RANGE, "records-1..%u", (unsigned)RAW_MAX_RECS);
        return;
    }
    if (want == 0u) {
        want = RAW_MAX_RECS;
    }

    while (got < want) {
        uint16_t off = 0u;

        /* Whatever complete records the carry already holds. */
        while ((got < want) &&
               ((uint32_t)(s_raw_carry_n - off) >= sizeof(bb_rec_hdr_t))) {
            bb_rec_hdr_t hdr;
            uint16_t     need;

            memcpy(&hdr, &s_raw_carry[off], sizeof hdr);
            need = (uint16_t)(sizeof hdr + hdr.length);
            if ((uint32_t)(s_raw_carry_n - off) < need) {
                break;                      /* payload not all here yet */
            }
            if ((hdr.type == (uint8_t)BB_REC_META) &&
                (hdr.length >= sizeof(bb_meta_t))) {
                bb_meta_t meta;

                memcpy(&meta, &s_raw_carry[off + sizeof hdr], sizeof meta);
                s_raw_initial     = (uint8_t)(meta.initial_state & 0xFFu);
                s_raw_initial_due = true;
            }
            if (hdr.type == (uint8_t)BB_REC_SAMPLES) {
                const uint8_t *pay  = &s_raw_carry[off + sizeof hdr];
                uint16_t       left = hdr.length;


                /*
                 * The bus as it was, one tick before anything moved. Emitted
                 * once, ahead of the first real sample, so the first genuine
                 * transition is readable as a transition.
                 */
                if (s_raw_initial_due && (got < want) &&
                    (left >= sizeof(bb_sample_t))) {
                    bb_sample_t first;
                    uint32_t    t0;
                    uint8_t    *r0;

                    memcpy(&first, pay, sizeof first);
                    t0 = (uint32_t)(s_raw_tick + first.dt_ticks - 1u);
                    r0 = &out[got * RAW_REC_BYTES];
                    r0[0] = (uint8_t)(t0 & 0xFFu);
                    r0[1] = (uint8_t)((t0 >> 8) & 0xFFu);
                    r0[2] = (uint8_t)((t0 >> 16) & 0xFFu);
                    r0[3] = (uint8_t)((t0 >> 24) & 0xFFu);
                    r0[4] = s_raw_initial;
                    got++;
                    s_raw_initial_due = false;
                }

                while ((got < want) && (left >= sizeof(bb_sample_t))) {
                    bb_sample_t smp;
                    uint32_t    t;
                    uint8_t    *r;

                    memcpy(&smp, pay, sizeof smp);
                    pay  += sizeof smp;
                    left -= (uint16_t)sizeof smp;

                    s_raw_tick += smp.dt_ticks;
                    t = (uint32_t)s_raw_tick;
                    r = &out[got * RAW_REC_BYTES];
                    r[0] = (uint8_t)(t & 0xFFu);
                    r[1] = (uint8_t)((t >> 8) & 0xFFu);
                    r[2] = (uint8_t)((t >> 16) & 0xFFu);
                    r[3] = (uint8_t)((t >> 24) & 0xFFu);
                    r[4] = (uint8_t)(smp.state & 0xFFu);
                    got++;
                }
                if (left != 0u) {
                    /*
                     * The caller's budget ran out mid-record. Keep the rest by
                     * rewriting the header in place to cover only what is
                     * unread, so the next call resumes exactly here.
                     */
                    uint16_t consumed = (uint16_t)(hdr.length - left);

                    hdr.length = left;
                    memcpy(&s_raw_carry[off + consumed], &hdr, sizeof hdr);
                    off = (uint16_t)(off + consumed);
                    resumed = 1u;
                    break;
                }
            }
            off = (uint16_t)(off + need);
        }

        if (off != 0u) {
            s_raw_carry_n = (uint16_t)(s_raw_carry_n - off);
            memmove(s_raw_carry, &s_raw_carry[off], s_raw_carry_n);
        }
        if (got >= want) {
            break;
        }

        /* Top up from the capture core. */
        {
            uint8_t  buf[BB_MSG_MAX_PAYLOAD];
            size_t   n    = 0u;
            uint32_t room = (uint32_t)(RAW_CARRY_MAX - s_raw_carry_n);

            if (room > BB_MSG_MAX_PAYLOAD) {
                room = BB_MSG_MAX_PAYLOAD;
            }
            if (room == 0u) {
                break;                      /* a record larger than the carry */
            }
            if (bb_link_request(BB_MSG_READ, &room, sizeof room,
                                buf, sizeof buf, &n) != BB_CAP_OK) {
                break;
            }
            if (n == 0u) {
                /*
                 * Nothing available *this* call, which is not the same as
                 * nothing left.
                 *
                 * The capture core hands over what it has to hand; a read that
                 * comes back empty while records are still queued behind it is
                 * ordinary. Latching that as "drained" ended the whole drain on
                 * the first such read - 164 of 197 edges delivered, the rest
                 * unreachable for the life of the capture, and a decoder left
                 * to make what it could of a frame that stopped mid-byte.
                 *
                 * This call returns what it has; the caller knows how many
                 * edges were declared and asks again.
                 */
                break;
            }
            memcpy(&s_raw_carry[s_raw_carry_n], buf, n);
            s_raw_carry_n = (uint16_t)(s_raw_carry_n + n);
        }
    }

    s_raw_resumed = resumed;
    bb_reply_ok_bytes(out, (size_t)got * RAW_REC_BYTES);
}

/* ---- table ------------------------------------------------------------- */

const bb_cmd_t bb_cmd_sniff[] = {
    { "SNIFF:SRC",    1, 1,    cmd_src,     "SNIFF:SRC <wire|self|both>" },
    { "SNIFF:CH",     1, 0xFF, cmd_ch,      "SNIFF:CH <proto> [taps...]" },
    { "SNIFF:TAPS?",  0, 0,    cmd_taps_q,  "SNIFF:TAPS?" },
    { "SNIFF:RATE",   1, 2,    cmd_rate,    "SNIFF:RATE <bus-hz> [sample-hz]" },
    { "SNIFF:RATE?",  0, 0,    cmd_rate_q,  "SNIFF:RATE?" },
    { "SNIFF:LINK?",  0, 0,    cmd_link_q,  "SNIFF:LINK?" },
    { "SNIFF:CM7",    0, 0,    cmd_cm7_start, "SNIFF:CM7" },
    { "SNIFF:CM7?",   0, 0,    cmd_cm7_q,     "SNIFF:CM7?" },
    { "SNIFF:ELE?",   0, 0,    cmd_ele_q,     "SNIFF:ELE?" },
    { "SNIFF:START",  0, 2,    cmd_start,   "SNIFF:START [proto] [STRICT]" },
    { "SNIFF:STOP",   0, 0,    cmd_stop,    "SNIFF:STOP" },
    { "SNIFF:READ?",  0, 1,    cmd_read_q,  "SNIFF:READ? [maxevents]" },
    { "SNIFF:RAW?",   0, 1,    cmd_raw_q,   "SNIFF:RAW? [maxbytes]" },
    { "SNIFF:TRACE?", 0, 1,    cmd_trace_q, "SNIFF:TRACE? [max]" },
    { "SNIFF:RAW:START", 0, 1, cmd_raw_start,  "SNIFF:RAW:START [edges]" },
    { "SNIFF:RAW:STOP",  0, 0, cmd_raw_stop,   "SNIFF:RAW:STOP" },
    { "SNIFF:RAW:STAT?", 0, 0, cmd_raw_stat_q, "SNIFF:RAW:STAT?" },
    { "SNIFF:RAW:READ?", 0, 1, cmd_raw_read_q, "SNIFF:RAW:READ? [records]" },
    { "SNIFF:MODE",   1, 1,    cmd_mode,    "SNIFF:MODE <oversample|pinclock>" },
    { "SNIFF:DMA",    0, 1,    cmd_dma,     "SNIFF:DMA [0|1]" },
    { "SNIFF:GAP",    0, 1,    cmd_gap,     "SNIFF:GAP [n]" },
    { "SNIFF:DECODE", 0, 1,    cmd_decode,  "SNIFF:DECODE [0|1]" },
    { "SNIFF:MODE?",  0, 0,    cmd_mode_q,  "SNIFF:MODE?" },
    { "SNIFF:STAT?",  0, 0,    cmd_stat_q,  "SNIFF:STAT?" },
    { "SNIFF:PINS?",  0, 0,    cmd_pins_q,  "SNIFF:PINS?" },
    { "SNIFF:CLEAR",  0, 0,    cmd_clear,   "SNIFF:CLEAR" },
    { "SNIFF:TEST",   0, 1,    cmd_test,    "SNIFF:TEST [half-period-ns]" },
    { "SNIFF:BENCH",  0, 2,    cmd_bench,   "SNIFF:BENCH [half-period-ns]" },
    { NULL, 0, 0, NULL, NULL }
};
