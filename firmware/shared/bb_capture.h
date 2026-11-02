/**
 * @file bb_capture.h
 * @brief The capture record format, and the CM33/CM7 protocol that carries it.
 *
 * Shared by both cores. This file is the seam of the whole sniffer: the CM33
 * asks for a capture and drains records, the CM7 produces them, and the Rust
 * `sniffer` project decodes them. Three contracts are inherited from that Rust
 * side and are **not renegotiable** - they were locked early there precisely so
 * that a new capture source could not quietly break every stored capture and
 * every golden test:
 *
 * 1. **Timestamps are picoseconds since the start of the capture, in a u64.**
 *    Not ticks. A tick count means nothing without the rate it was counted at,
 *    and that rate belongs to one device. u64 picoseconds is 213 days at a
 *    resolution far below anything a digital capture resolves.
 *
 * 2. **A capture is a stream of length-prefixed CRC'd records.** That framing
 *    is what makes a file cut short by a crash readable up to the first record
 *    that does not check out, with everything before it kept and usable.
 *
 * 3. **The first entry of a sample chunk is the initial state, not a
 *    transition.** A stream of transitions cannot say what the lines were doing
 *    before the first one. On the STM32 this was found the hard way: without it
 *    the decoder read the first release of SDA as a STOP that never happened.
 *
 * What is new here is the fourth field on every record: **where the record came
 * from**. The STM32 had one source, so it did not need to say. This build has
 * three, and one of them is inference rather than measurement.
 */
#ifndef BB_CAPTURE_H
#define BB_CAPTURE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* ---- sources ----------------------------------------------------------- */
/**
 * @brief Where a record's contents actually came from.
 *
 * This distinction is the reason the field exists, and it must survive all the
 * way to the decoder. `BB_OBS_WIRE` is a measurement: FlexIO latched the line
 * at a clock edge. `BB_OBS_CONTROLLER` is what our own peripheral's FIFO
 * accepted or returned - true, useful, cheap, and **not the same thing**. They
 * differ exactly when something is wrong, which is when a sniffer earns its
 * keep, so nothing downstream is allowed to confuse them.
 */
typedef enum {
    BB_OBS_WIRE       = 0,   /**< FlexIO sampled the pad */
    BB_OBS_CONTROLLER = 1,   /**< our own controller FIFO said so */
    BB_OBS_SYNTHETIC  = 2    /**< self-test pattern; never a real bus */
} bb_observed_t;

/* ---- record types ------------------------------------------------------ */
typedef enum {
    BB_REC_META    = 0x01,   /**< capture parameters, always the first record */
    BB_REC_SAMPLES = 0x02,   /**< a chunk of line-state samples */
    BB_REC_GAP     = 0x03,   /**< capture stopped and restarted; data was lost */
    BB_REC_EVENT   = 0x04,   /**< a decoded transaction event */
    BB_REC_END     = 0x05    /**< clean end of capture */
} bb_rec_type_t;

/** Which protocol a channel was configured to capture. */
typedef enum {
    BB_PROTO_NONE = 0,
    BB_PROTO_I2C,
    BB_PROTO_I3C,
    BB_PROTO_SPI,
    BB_PROTO_MDIO,
    BB_PROTO_UART,
    BB_PROTO_CAN
} bb_proto_t;

/* ---- record header ----------------------------------------------------- */
/*
 * Packed and little-endian, because the host reads it as bytes and a padded
 * struct would make the on-wire layout depend on the compiler. 16 bytes.
 */
typedef struct __attribute__((packed)) {
    uint8_t  type;        /**< bb_rec_type_t */
    uint8_t  observed;    /**< bb_observed_t */
    uint16_t length;      /**< payload bytes following this header */
    uint32_t crc32;       /**< over the payload only, so a truncated payload is
                               detectable without trusting the length */
    uint64_t t_ps;        /**< picoseconds since capture start */
} bb_rec_hdr_t;

/** Payload of BB_REC_META. */
typedef struct __attribute__((packed)) {
    uint8_t  proto;          /**< bb_proto_t */
    uint8_t  channel_count;  /**< lines sampled per entry */
    uint8_t  flags;
    uint8_t  reserved;
    uint32_t nominal_hz;     /**< configured bus rate, 0 if measured */
    uint64_t ps_per_tick;    /**< resolution of the capture clock */
    /* The initial line state, satisfying contract 3. One bit per channel, in
     * the order the channels were bound by SNIFF:CH. */
    uint32_t initial_state;
} bb_meta_t;

#define BB_META_FLAG_STRICT   0x01u  /**< every edge kept, nothing masked */
#define BB_META_FLAG_SELFTEST 0x02u

/** Payload of BB_REC_GAP. */
typedef struct __attribute__((packed)) {
    uint64_t resume_t_ps;    /**< when capture picked up again */
    uint32_t lost_estimate;  /**< entries lost, 0 when genuinely unknown */
    uint8_t  reason;
    uint8_t  reserved[3];
} bb_gap_t;

#define BB_GAP_RING_FULL   1u   /**< the DMA ring filled before it was drained */
#define BB_GAP_STOPPED     2u   /**< the host stopped and restarted capture */
#define BB_GAP_OVERRUN     3u   /**< FlexIO shifter overran */

/**
 * @brief One sample entry inside a BB_REC_SAMPLES payload.
 *
 * A timestamp delta rather than an absolute one: at picosecond resolution an
 * absolute u64 per edge would be eight bytes for information that is almost
 * always small, and a busy bus is mostly small deltas. The header carries the
 * absolute base and the deltas accumulate from it.
 *
 * `dt_ticks` being 32-bit caps one delta at about 4 billion ticks; when a gap
 * is longer than that the CM7 emits a fresh SAMPLES record with a new absolute
 * base rather than overflowing. That is a real case, not a theoretical one - an
 * idle bus between two bursts is exactly it.
 */
typedef struct __attribute__((packed)) {
    uint32_t dt_ticks;    /**< since the previous entry */
    uint32_t state;       /**< one bit per channel */
} bb_sample_t;

/* ---- the CM33 <-> CM7 protocol ----------------------------------------- */
/*
 * The controller core does not touch the capture hardware and the capture core
 * does not touch USB. They talk over RPMsg with the small request/response set
 * below, which is deliberately narrow: every extra message is a way for the two
 * cores to disagree about state.
 *
 * The alternative - shared memory with flags - was rejected because the two
 * cores have different caches and different views of the TRDC, and a protocol
 * that is only correct when both agree about cache maintenance is a protocol
 * that works until it does not.
 */
typedef enum {
    BB_MSG_PING       = 0x01,  /**< is the capture core alive, and what is it */
    BB_MSG_CONFIG     = 0x02,  /**< bind a channel to a protocol and pins */
    BB_MSG_START      = 0x03,
    BB_MSG_STOP       = 0x04,
    BB_MSG_STAT       = 0x05,
    BB_MSG_READ       = 0x06,  /**< drain up to N bytes of records */
    BB_MSG_CLEAR      = 0x07,
    BB_MSG_SELFTEST   = 0x08,  /**< drive a known pattern and capture it */
    BB_MSG_BENCH      = 0x09,  /**< measure the achievable rate */
    BB_MSG_EVENTS     = 0x0A,  /**< drain decoded transaction events */
    BB_MSG_REPLY      = 0x80   /**< OR'd into the reply's opcode */
} bb_msg_op_t;

#define BB_MSG_MAX_PAYLOAD  480u   /**< fits one RPMsg buffer with room for its
                                        own header; larger reads are paged */

typedef struct __attribute__((packed)) {
    uint8_t  op;          /**< bb_msg_op_t */
    uint8_t  status;      /**< 0 = ok, else a bb_cap_err_t */
    uint16_t length;      /**< payload bytes following */
    uint32_t seq;         /**< echoed, so a late reply cannot be mistaken for
                               the answer to the next question */
} bb_msg_hdr_t;

typedef enum {
    BB_CAP_OK = 0,
    BB_CAP_ERR_PROTO,        /**< no capture configuration for that protocol */
    BB_CAP_ERR_PINS,         /**< the requested pins are not FlexIO-capable, or
                                  are already spoken for by a bus */
    BB_CAP_ERR_BUSY,
    BB_CAP_ERR_NOT_RUNNING,
    BB_CAP_ERR_RANGE,
    BB_CAP_ERR_FLEXIO,       /**< the FlexIO configuration was rejected */
    BB_CAP_ERR_UNSUPPORTED,
    BB_CAP_ERR_TIMEOUT       /**< the capture core did not answer in time; it
                                  may still be working on the request */
} bb_cap_err_t;

/**
 * @brief Payload of BB_MSG_CONFIG.
 *
 * `flexio` and `pins` are resolved by the controller core rather than sent as
 * board table indices for the capture core to look up. The division is
 * deliberate and matches the rest of the design: the CM33 owns the pin map and
 * pad muxing, the CM7 owns the capture hardware. Shipping the board table to
 * both cores would give two places to describe one board, and `taps` is kept
 * only so the capture core can report back what it was told.
 */
/**
 * @brief How the capture clocks itself.
 *
 * Two genuinely different instruments, not a tuning knob.
 *
 * `oversample` runs a free-running sample clock and reads the taps on every
 * tick. It sees everything on the bus - levels, glitches, the idle time between
 * transactions, START and STOP - and it costs one sample per tick per channel
 * whether or not anything happened. Its ceiling is the sample rate, and a
 * decoder needs roughly four samples per bus clock, so the bus rate it can
 * follow is a quarter of what it samples at.
 *
 * `pinclock` takes the bus's own clock line as the shift clock: one bit per bus
 * clock, exactly where the bus says the data is valid. There is no oversampling
 * ratio, so the bus rate it can follow is bounded only by FlexIO's input
 * synchronisation - IMXRT1180RM 60.6.3 puts that at a quarter of the FlexIO
 * clock, which is 30 MHz here against I3C's own 12.5 MHz ceiling. It also
 * produces about a fifth of the data for the same traffic.
 *
 * Framing needs a separate mechanism, because START and STOP are SDA transitions
 * *while SCL is high* and no clock edge exists at those moments - a shifter
 * clocked by SCL cannot see them. Two extra timers count SDA edges, one per
 * direction, and software reads SCL when a flag comes up to decide whether the
 * clock was being held. That qualification is software, so framing is reliable
 * at moderate rates and not at full rate, while the bits stay hardware-exact at
 * any rate the bus can run.
 */
typedef enum {
    BB_CAP_OVERSAMPLE = 0,  /**< free-running sample clock; sees framing */
    BB_CAP_PINCLOCK   = 1   /**< shift on the bus clock; exact bits, no framing */
} bb_cap_mode_t;

typedef struct __attribute__((packed)) {
    uint8_t  proto;       /**< bb_proto_t */
    uint8_t  source;      /**< bb_observed_t: wire, controller, or both */
    uint8_t  flags;       /**< BB_META_FLAG_* */
    uint8_t  tap_count;
    uint8_t  taps[8];     /**< indices into the board's BB_SNIFF_TAPS table */
    uint32_t nominal_hz;  /**< bus rate the taps are expected to carry */
    uint8_t  flexio;      /**< 1 or 2 - the instance every tap is on */
    uint8_t  pins[8];     /**< FlexIO pin number per tap, in taps[] order */
    uint8_t  mode;        /**< bb_cap_mode_t */
    /*
     * 0 turns the on-device decoder off and captures samples only.
     *
     * Not a feature so much as a measurement: it is the only way to ask what
     * capture costs without decode, and therefore what splitting the two across
     * the cores could possibly buy. Taken from the reserved bytes so the
     * message layout is unchanged and an older host still works - it sends
     * zero, which is why the meaning is inverted on the wire and normalised in
     * cap_configure() rather than here.
     */
    uint8_t  decode_off;
    /*
     * Take the pin-clocked intake through eDMA instead of polling SHIFTBUF.
     *
     * Taken from the reserved byte, like decode_off, so the message layout does
     * not move and an older host still works by sending zero. Zero is the
     * polled path, which is the one every rate up to 8 MHz was measured on.
     */
    uint8_t  dma_intake;
    /*
     * Clock edges between one transaction and the next that carry no data.
     *
     * Measured directly from the raw sample stream rather than inferred from
     * how the bytes decode: with the decoder switched off, the bit patterns of
     * a known write and the read after it were located in the samples and the
     * distance between them counted. It is exactly one, at every rate from
     * 1 MHz to 12.5 MHz, every frame pair.
     *
     * For a long time the decoder wanted two or three where the wire had one,
     * and that discrepancy was the standing clue that something else was wrong.
     * It was: the frame was opened from the START flag, which arrives promptly
     * while the bits it precedes do not, so the extra skip was padding for a
     * variable delay rather than anything on the bus. With frames anchored to
     * the previous frame's end instead, the right value is one - the measured
     * one - at every rate, and it is one because that is what the wire carries.
     */
    uint8_t  gap_bits;
    uint32_t sample_hz;   /**< requested sample rate; 0 = derive from nominal */
} bb_cfg_t;

/**
 * @brief Payload of a BB_MSG_SELFTEST reply.
 *
 * Both the expected and the observed count, not a verdict. A self test that
 * returns only pass/fail cannot be used to find the ceiling, and the STM32
 * build's worst bug hid behind exactly that: its loopback test passed while
 * capturing a twentieth of the rate it claimed.
 */
typedef struct __attribute__((packed)) {
    uint32_t driven_loads;  /**< parallel loads the generator actually sent */
    uint32_t expected;      /**< a floor on the transitions those produce */
    uint32_t seen;          /**< entries the capture engine recorded */
    uint32_t sample_hz;     /**< the rate it sampled at */
    uint32_t src_hz;        /**< FlexIO root clock the rate was divided from */
    uint32_t frame_rise;    /**< framing words seen on SDA rising edges */
    uint32_t frame_fall;    /**< framing words seen on SDA falling edges */
    uint32_t overruns;
    /*
     * Per-tap result of driving and reading each pad directly, before any
     * shifter is involved: bit i is tap i in SNIFF:CH order.
     *
     * Added because a capture where one channel never moves has two very
     * different explanations - the pad is not muxed to FlexIO, or the shifter is
     * not driving that member of its parallel group - and the whole-channel
     * result cannot tell them apart. This can: it uses FlexIO's own PINOUTE /
     * PINOUTD / PIN registers, which bypass the shifters entirely.
     */
    uint32_t pin_probe;     /**< bit i: tap i drove high and low and read back */
    uint32_t pin_live;      /**< the PIN register after the run, for context */
    /*
     * The sampler's own registers, read back after configuration.
     *
     * Reported because "parallelWidth was set" is an assumption until the
     * register says so, and the difference between a field that did not take and
     * a peripheral that ignores it is a one-line fix versus a redesign.
     */
    uint32_t shiftcfg;
    uint32_t shiftctl;
    uint32_t timcmp;
    /*
     * Which channels actually changed during the run, bit i for tap i.
     *
     * This is the result that matters. A whole-capture "it saw transitions" was
     * satisfied by one working channel out of two for several rounds, because
     * nothing reported per channel - so a completely dead second channel looked
     * like success. Every tap is driven together, so anything short of all ones
     * here is a real failure.
     */
    uint32_t moved;
    /*
     * Samples taken during the run. Times the sample period this is the elapsed
     * duration, which is the figure that says whether a self test is slow
     * because the engine is slow or because the generator is - and there was no
     * way to tell them apart without it.
     */
    uint32_t ticks;
    uint8_t  base_pin;      /**< first pin of the sampled window */
    uint8_t  width;         /**< window width in pins */
    uint8_t  reserved[2];
} bb_selftest_t;

/**
 * @brief One decoded transaction event.
 *
 * The capture core decodes as well as samples, and this is the second of the two
 * modes the spec describes: raw samples for a host that wants the waveform,
 * decoded events for one that wants the transaction. Both come from the same
 * intake, so they cannot disagree about what happened - which is the reason to
 * decode here rather than run a second pass over the samples.
 *
 * The kinds are exactly the tokens the wire protocol already carries, because
 * this is not a new protocol: `S`, `Sr`, `P`, `!`, and `A`/`D` with a hex byte
 * and an ack flag. The host library has parsed those since the STM32 build and
 * nothing about it changes.
 */
typedef enum {
    BB_EV_START   = 1,   /**< S  - START condition */
    BB_EV_RESTART = 2,   /**< Sr - repeated START, no STOP in between */
    BB_EV_STOP    = 3,   /**< P  - STOP condition */
    BB_EV_ADDR    = 4,   /**< A  - the first byte after a START */
    /*
     * D - any byte after the address.
     *
     * The accompanying flag is the ninth bit, and what it MEANS depends on the
     * protocol: an acknowledge driven by the receiver on I2C, and a T-bit
     * driven by the transmitter on I3C - parity on a write, end-of-data on a
     * read. Same wire position, opposite party, unrelated meaning. The record
     * header carries the protocol; nothing in the event itself does.
     */
    BB_EV_DATA    = 5,
    BB_EV_DROPPED = 6,   /**< !  - events were lost; the stream has a hole */
    /*
     * I3C only, and added rather than overloaded BB_EV_DATA.
     *
     * A CCC code is not a data byte: it selects what every byte after it means,
     * so a host that cannot tell them apart cannot decode the rest of the frame
     * at all. Existing kinds keep the meaning they always had, so a host that
     * does not know this one still reads I2C captures correctly and sees an
     * unknown kind on I3C rather than a plausible wrong byte.
     */
    BB_EV_CCC     = 7    /**< C  - I3C common command code */
} bb_ev_kind_t;

typedef struct __attribute__((packed)) {
    uint8_t kind;    /**< bb_ev_kind_t */
    uint8_t value;   /**< the byte, for ADDR and DATA */
    uint8_t ack;     /**< 1 if the ninth bit was low, for ADDR and DATA */
    uint8_t pad;
} bb_event_t;

/** Payload of a BB_MSG_EVENTS reply: a count, then that many events. */
typedef struct __attribute__((packed)) {
    uint16_t count;
    uint16_t more;   /**< non-zero when the ring still holds events */
} bb_events_hdr_t;

/** Payload of a BB_MSG_STAT reply. */
typedef struct __attribute__((packed)) {
    uint8_t  running;
    uint8_t  proto;
    uint8_t  source;
    uint8_t  full;           /**< the ring filled and a gap was recorded */
    uint32_t entries;        /**< sample entries captured */
    uint32_t records;        /**< records produced */
    uint32_t bytes_pending;  /**< waiting to be drained */
    uint32_t gaps;
    uint32_t coincident;     /**< samples that changed nothing, dropped */
    uint64_t ps_per_tick;
    uint32_t sample_hz;      /**< the rate actually achieved, not requested */
    uint32_t overruns;       /**< shifter overruns: sampling outran the core */
    /*
     * The capture core's own clock, reported by the capture core.
     *
     * The intake ceiling is set by how fast this core can drain shifters, so
     * this is the number behind every sampling limit here - and it is not the
     * one the controller core can see. The CM33 hands the M7 a 400 MHz root to
     * start on and the capture image then switches itself to the ARM PLL, so
     * anything asked on the CM33 side reports the handoff value and not what
     * the core ended up running at.
     */
    uint32_t core_hz;        /**< capture core clock, measured on that core */
    /*
     * Where the intake's time actually goes, in core cycles.
     *
     * The pin-clocked ceiling is a budget question - at 12.5 MHz a shifter word
     * arrives every 640 ns, which is about 500 cycles at 792 MHz - and every
     * attempt to raise it by reasoning about which line looked expensive has
     * either helped by an unclear amount or not at all. These are measured with
     * the cycle counter on the core that does the work.
     */
    uint32_t prof_polls;     /**< cap_poll() calls */
    uint32_t prof_words;     /**< shifter words drained */
    uint32_t prof_cyc_poll;  /**< cycles in cap_poll(), total >> 6 */
    uint32_t prof_cyc_word;  /**< cycles in cap_consume(), total >> 6 */
    uint32_t prof_cyc_max;   /**< worst single cap_poll(), in cycles */
    uint16_t defer_bits;     /**< bits consumed while a STOP was pending */
    uint16_t arm_bits;       /**< bits consumed waiting to apply the START */
    uint16_t frame_bits;     /**< bits the decoder consumed for the last frame */
    uint8_t  close_n;        /**< entries in close_log */
    uint8_t  close_pad[3];
    /*
     * The last few frame closes, packed one per word.
     *
     * Read frames come to nine bits at some rates where writes always come to
     * twenty-seven, and the wire has been measured uniform, so what ends those
     * frames is in the decoder. This carries the state at the moment each one
     * ended rather than leaving it to be inferred.
     */
    uint32_t close_log[8];
    uint32_t start_at;       /**< sample tick the last START flag was seen */
    uint32_t stop_at;        /**< sample tick the last STOP flag was seen */
    uint16_t bothpend_n;     /**< boundaries with START and STOP both pending */
    uint16_t reorder_n;      /**< of those, START noted in an earlier poll */
    uint32_t dma_avail_max;  /**< deepest the DMA intake queue was found */
    uint16_t setup_starts;   /**< STARTs settled as a STOP's setup fall */
    uint16_t setup_pad;
    uint32_t starts_seen;    /**< START flags the hardware raised */
    uint32_t stops_seen;     /**< STOP flags the hardware raised */
    uint32_t stops_noted;    /**< of those, ones with a frame open to close */
} bb_stat_t;

/**
 * @brief Payload of a BB_MSG_BENCH reply.
 *
 * `max_bus_hz` is the point of the whole message. The STM32 build learned two
 * lessons here that are worth carrying:
 *
 *   - Instrumenting the capture path measures the instrumentation. Timing each
 *     interrupt from inside it made the cost look 17% worse than it was.
 *   - A loopback pass/fail test cannot find the ceiling, because the generator
 *     and the capture share a core and the generator simply stretches. It
 *     "passed" at a nominal 3.3 MHz while never exceeding 202 kHz.
 *
 * Neither trap applies the same way to a hardware-clocked capture - FlexIO does
 * not spend CPU per edge - but the second one still bites a self test, so the
 * bench reports the DMA-limited rate and says whether the figure came from a
 * self test or from real traffic.
 */
typedef struct __attribute__((packed)) {
    uint32_t max_bus_hz;
    uint32_t dma_bytes_per_s;
    uint8_t  from_selftest;   /**< 1 when the generator shared this core */
    uint8_t  reserved[3];
    /* The sample period in nanoseconds. Named cost_ns because that is
     * what the wire protocol and the host library call it - on the
     * STM32 it was the per-edge software cost, and here there is none,
     * so the sample period is the figure that bounds resolution. */
    uint32_t sample_hz;    /**< rate the sweep sampled at; max_bus_hz is
                                bounded by it, not just by the generator */
    uint8_t  limited_by;   /**< 0 sampler (Nyquist headroom), 1 generator */
    uint8_t  pad_lim[3];
    uint32_t cost_ns;
} bb_bench_t;

/* ---- helpers ----------------------------------------------------------- */

/** CRC-32, the same polynomial the Rust side uses. Defined in bb_image.c. */
uint32_t bb_image_crc32(const uint8_t *data, size_t len);

static inline const char *bb_proto_name(bb_proto_t p)
{
    switch (p) {
    case BB_PROTO_I2C:  return "i2c";
    case BB_PROTO_I3C:  return "i3c";
    case BB_PROTO_SPI:  return "spi";
    case BB_PROTO_MDIO: return "mdio";
    case BB_PROTO_UART: return "uart";
    case BB_PROTO_CAN:  return "can";
    default:            return "none";
    }
}

static inline const char *bb_observed_name(bb_observed_t o)
{
    switch (o) {
    case BB_OBS_WIRE:       return "wire";
    case BB_OBS_CONTROLLER: return "controller";
    default:                return "synthetic";
    }
}

static inline const char *bb_cap_err_name(bb_cap_err_t e)
{
    switch (e) {
    case BB_CAP_OK:               return "ok";
    case BB_CAP_ERR_PROTO:        return "no-capture-config-for-protocol";
    case BB_CAP_ERR_PINS:         return "pins-not-flexio-or-already-in-use";
    case BB_CAP_ERR_BUSY:         return "capture-already-running";
    case BB_CAP_ERR_NOT_RUNNING:  return "capture-not-running";
    case BB_CAP_ERR_RANGE:        return "out-of-range";
    case BB_CAP_ERR_FLEXIO:       return "flexio-config-rejected";
    case BB_CAP_ERR_TIMEOUT:      return "capture-core-did-not-answer-in-time";
    default:                      return "unsupported";
    }
}

#endif /* BB_CAPTURE_H */
