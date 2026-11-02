/**
 * @file bb_i3c.c
 * @brief I3C controllers — two of them, which the STM32 never had.
 *
 * Translated from `_stm32_reference/bb_i3c.c` (2381 lines against ST's HAL).
 * The command surface is unchanged, with one addition and one honest removal.
 *
 * **Added: `I3C:PORT <1|2>`.** This part has I3C1 and I3C2 and both reach a
 * connector, so port numbering here counts *controllers*. On the NUCLEO,
 * `BB_I3C_PORT_COUNT` counted pin *pairs* of one I3C1 - the STM32 porting notes
 * flagged that a part with two I3C peripherals could not use the second without
 * parameterising the whole file, because it referenced `I3C1` throughout. That
 * is exactly the change made here: nothing below names an instance.
 *
 * **Removed: target-role commands.** `I3C:ROLE target` and the `I3C:TGT:*`
 * family answer `unsupported` in this image and say where to find them. On the
 * STM32 they lived in a file shared between both profiles; here they belong to
 * the peripheral profile, which is the image whose job is to *be* a target. A
 * bus controller and a bus target want the same peripheral and the same pins,
 * and pretending one image can be both at once is how you get a bus with two
 * things driving SCL.
 *
 * **Not carried over: the 64 MHz clock workaround.** The STM32 had to clock I3C
 * from HSI at 64 MHz rather than PCLK1 at 250 MHz, because its timing fields
 * were 8 bits per phase and a 1.3 us legacy-I2C low period needed 325 counts,
 * which does not fit. The cost was that 100 kHz legacy devices could not share
 * the I3C bus. NXP's I3C takes three separate baud rates - I2C, push-pull and
 * open-drain - and derives its own dividers, so a mixed bus needs no such
 * trade. `I3C:BUS mixed` is therefore real here rather than a compromise.
 */
#include "bb_board.h"
#include "bb_cmd.h"
#include "bb_i3c.h"
#include "bb_image.h"
#include "bb_io.h"
#include "bb_status.h"
#include "bb_trace.h"

#include "fsl_clock.h"
#include "fsl_i3c.h"

#include <stdio.h>
#include <string.h>

/* I3C's reserved broadcast address. Every ENTDAA and broadcast CCC opens with
 * it, and "nobody answered 0x7E" is the single most common I3C bring-up
 * failure - no target present, or no pull-ups. */
#define I3C_BROADCAST_ADDR  0x7Eu

/*
 * The push-pull rate dynamic address assignment is run at.
 *
 * Not the fastest that works - the fastest that works with margin. The DAA path
 * is 5/5 at 1, 2, 3 and 6 MHz and 0/5 at 12.5, with the boundary bracketed at
 * 6.06 MHz passing, 6.25 MHz marginal at 1/5 and 6.45 MHz failing.
 *
 * Six looked like the obvious cap and gave 4 successes in 10, because the cap
 * is a *request* and the dividers land where they land: from the 25 MHz root a
 * 12.5 MHz bus needs, 6 MHz resolves to 6.25 - the one rate measured as
 * marginal. Three resolves to 3.125 MHz from that root and 3 MHz from a 12 MHz
 * one, both comfortably inside the working range whatever the root happens to
 * be.
 *
 * Discovery is a handful of transactions at startup; there is nothing to gain
 * by running it near a boundary.
 */
#define I3C_DAA_MAX_PP_HZ  3000000u

/* The highest I3C functional clock this driver will select. See cmd_root(). */
#define I3C_ROOT_MAX_HZ    100000000u

#define I3C_MAX_TARGETS     8u

/* Dynamic addresses handed out by ENTDAA. Chosen high so they cannot collide
 * with the static addresses of legacy I2C devices on a mixed bus, which live
 * at the low end in practice. */
static const uint8_t k_daa_pool[I3C_MAX_TARGETS] = {
    0x09u, 0x0Au, 0x0Bu, 0x0Cu, 0x0Du, 0x0Eu, 0x0Fu, 0x10u
};

typedef struct {
    I3C_Type     *base;
    bb_pin_t      scl;
    bb_pin_t      sda;
    const char   *pins;
    clock_root_t  root;
    unsigned      shares_i2c_bus;   /**< 0 = shares nothing */
} i3c_port_t;

static const i3c_port_t s_port[BB_I3C_PORT_COUNT] = {
    {
        .base = BB_I3C_P1_INSTANCE,
        .scl  = BB_I3C_P1_SCL,
        .sda  = BB_I3C_P1_SDA,
        .pins = BB_I3C_P1_PINS,
        .root = kCLOCK_Root_I3c1,
        .shares_i2c_bus = BB_I3C_P1_SHARES_I2C,
    },
    {
        .base = BB_I3C_P2_INSTANCE,
        .scl  = BB_I3C_P2_SCL,
        .sda  = BB_I3C_P2_SDA,
        .pins = BB_I3C_P2_PINS,
        .root = kCLOCK_Root_I3c2,
        .shares_i2c_bus = BB_I3C_P2_SHARES_I2C,
    },
};

static struct {
    unsigned          port;          /**< 0-based */
    uint32_t          pp_hz;
    uint32_t          od_hz;
    uint32_t          i2c_hz;
    bool              mixed;         /**< legacy I2C devices on the bus too */
    bool              configured;
    bool              pull_internal;
    bool              odhpp;
    uint8_t           pplow;
    uint8_t           skew;
    uint8_t           hkeep;         /**< MCONFIG.HKEEP, the high-keeper */
    bool              disto;         /**< MCONFIG.DISTO, disable in-frame timeout */
    bool              odstop_force;  /**< override the mixed-bus default */
    bool              odstop_set;
    uint32_t          pad_raw;       /**< SW_PAD_CTL override, 0 = leave alone */
    bool              rx_wide;
    bool              rx_defer;
    uint32_t          rx_gap;
    uint8_t           rx_trig;
    bool              listening;
    uint8_t           target_count;
    i3c_device_info_t targets[I3C_MAX_TARGETS];
} s = {
    .port          = 1u,            /* port 2: I3C2 on A4/A5, shares nothing */
    .pp_hz         = 4000000u,
    .od_hz         = 1000000u,
    .i2c_hz        = 400000u,
    .mixed         = false,
    .pull_internal = false,
    /*
     * Off, against the SDK's default, because it is what full-rate costs.
     *
     * ODHPP drives the open-drain SCL HIGH pulse push-pull, so its width is set
     * by the push-pull rate rather than the open-drain one: about 83 ns at
     * 6 MHz and 40 ns at 12.5 MHz. Every transfer begins with an open-drain
     * address header, so a target that needs longer than that to sample SCL
     * stops acknowledging its own address as the push-pull rate rises. It
     * presents as a fast-bus failure and is a pulse-width one.
     *
     * Measured: at a true 12.5 MHz, ODHPP on gives no-ack-for-0x09 on a private
     * write and no-ack-for-0x7E on a broadcast CCC; ODHPP off gives 5/5 on both.
     * ENTDAA is unaffected at 1, 2, 3 and 6 MHz either way, so nothing is traded
     * for it.
     *
     * This was tested earlier and recorded as having no effect, which was wrong
     * - that measurement was taken while a test harness was running ENTDAA
     * twice and reporting the second, empty result. The negative finding
     * outlived the bug that produced it.
     */
    .odhpp         = false,
    .pplow         = 0u,
    .skew          = 0u,
    .rx_wide       = false,
    .rx_gap        = 0u,
    .rx_trig       = 0u,
};

static const i3c_port_t *pt(void)
{
    return &s_port[s.port];
}

/* ---- configuration ----------------------------------------------------- */

/*
 * The address the last transfer put on the bus, so a NAK can name it.
 *
 * Every NAK used to be reported as "no-ack-for-7E". That is true of ENTDAA and
 * of a broadcast CCC, and false of a private read or write, which never send
 * 0x7E at all - a read from a target at 0x09 that nobody answered reported a
 * problem with the broadcast address instead, and sent the reader looking at
 * dynamic address assignment rather than at the target they had just addressed.
 */
static uint8_t s_last_addr = I3C_BROADCAST_ADDR;

/*
 * What the controller looked like at the moment a transfer gave up.
 *
 * The 12.5 MHz ENTDAA failure reports no-ack-for-0x7E while the capture shows
 * the broadcast address being acknowledged on the wire - so the error string is
 * the SDK's interpretation and not an observation. This records the registers
 * behind that interpretation, at the instant it was made, so the two can be
 * compared instead of argued about.
 *
 * Phases are coarse on purpose. The question this has to answer first is not
 * "which line failed" but "did the controller get as far as having the ACK",
 * and a handful of markers around the transfer answers that.
 */
typedef enum {
    I3C_PH_IDLE      = 0x00u,
    I3C_PH_ENTER     = 0x10u,  /* command entered              */
    I3C_PH_READY     = 0x11u,  /* port configured, pads claimed */
    I3C_PH_PRE_DAA   = 0x14u,  /* about to call into the SDK    */
    I3C_PH_POST_DAA  = 0x1Eu,  /* SDK returned                  */
    I3C_PH_ERROR     = 0xE0u
} i3c_phase_t;

static struct {
    uint8_t  phase;
    int32_t  status;      /* the SDK status_t that was classified */
    uint32_t mstatus;
    uint32_t merrwarn;
    uint32_t mctrl;
    uint32_t mdatactrl;
    uint32_t mconfig;
    uint32_t root_hz;
    uint8_t  addr;        /* what s_last_addr held */
} s_trace;

/*
 * The receive timeline, one entry per FIFO pop.
 *
 * Every software lever has come back byte-identical, so the question is no
 * longer which knob but what the FIFO actually looked like while the bytes
 * were being taken out of it. `pre` answers plan v3 section 11 - the count
 * before START, which must be zero - and the per-pop rows answer section 4's
 * "RXCOUNT sequence": how many entries were queued at the moment each byte was
 * taken, and how long the CPU waited for it.
 *
 * `spin` is the useful column. A duplicate that arrives after a real wait is a
 * FIFO that advanced and re-presented stale data; a duplicate that arrives
 * with no wait at all is a count that was already non-zero and a read port
 * that never moved on. The two have different causes and the same symptom.
 */
#define I3C_RXLOG_MAX 32u

static struct {
    uint8_t  pre;               /**< RXCOUNT before START - expected 0 */
    uint8_t  n;
    uint8_t  type;              /**< MCTRL.TYPE the read was issued with */
    uint8_t  post;              /**< RXCOUNT once the frame has closed */
    uint8_t  extra;             /**< entries drained after the frame closed */
    uint8_t  tail[I3C_RXLOG_MAX];
    uint8_t  val[I3C_RXLOG_MAX];
    uint8_t  cnt[I3C_RXLOG_MAX];
    uint32_t spin[I3C_RXLOG_MAX];
} s_rxlog;

/*
 * The transmit timeline, the same instrument pointed the other way.
 *
 * A private write times out for bus rates between roughly 1.5 and 4 MHz while
 * working below and above that band, and the wire shows the address ACKed with
 * no data bytes after it at all. MSTATUS sits in NormAct with TXNOTFULL set and
 * COMPLETE clear, and MERRWARN is empty - so the module accepted the address,
 * has room for data, reports no error, and does not clock anything out.
 *
 * Where the SDK's blocking write gives up, this records which wait it was.
 */
static struct {
    uint8_t  pre;               /**< TXCOUNT before START */
    uint8_t  n;
    uint8_t  stage;             /**< how far the write got - i3c_tx_stage_t */
    uint8_t  val[I3C_RXLOG_MAX];
    uint8_t  cnt[I3C_RXLOG_MAX];
    uint32_t spin[I3C_RXLOG_MAX];
    uint32_t spin_start;
    uint32_t spin_done;
    uint32_t mstatus_end;
    uint32_t merrwarn_end;
} s_txlog;

/* Defined below with the other register-level transfer paths. */
static status_t i3c_write_typed(uint8_t addr, const uint8_t *buf, uint32_t len,
                                uint32_t type);

typedef enum {
    I3C_TX_ENTER = 0u,
    I3C_TX_STARTED,             /**< MCTRLDONE seen, address phase over */
    I3C_TX_FED,                 /**< every byte handed to the FIFO */
    I3C_TX_COMPLETE,            /**< COMPLETE seen */
    I3C_TX_STOPPED
} i3c_tx_stage_t;

static void trace_mark(uint8_t phase)
{
    s_trace.phase = phase;
}

/** Freeze the controller's registers. Reads only - nothing here clears state. */
static void trace_snap(status_t st)
{
    I3C_Type *base = pt()->base;

    s_trace.status    = (int32_t)st;
    s_trace.mstatus   = base->MSTATUS;
    s_trace.merrwarn  = base->MERRWARN;
    s_trace.mctrl     = base->MCTRL;
    s_trace.mdatactrl = base->MDATACTRL;
    s_trace.mconfig   = base->MCONFIG;
    s_trace.root_hz   = CLOCK_GetRootClockFreq(pt()->root);
    s_trace.addr      = s_last_addr;
}

/* ---- controller-side trace (SNIFF:SRC self) ---------------------------- */
/*
 * The generating half of `SNIFF:SRC self`. Every call here is a no-op unless
 * that source has armed the ring - one load and a branch inside
 * bb_trace_push() - which is why the transfer paths call them unconditionally
 * instead of guarding each site and making the hot path read as if tracing
 * were its main business.
 *
 * Only the register-level paths in this file are instrumented. CCC and ENTDAA
 * go through the SDK's blocking transfer, which does not expose the bytes it
 * moves, so xfer() marks a hole rather than describing a frame it cannot see.
 * bb_trace.h says why a hole beats a silence.
 */

/**
 * @brief The T-bit a write carries: odd parity over the eight data bits.
 *
 * Computed, never observed. The controller generates this bit in hardware and
 * reports it nowhere, so the only way to put a ninth bit on a write event is
 * to work it out from the byte. The convention matches both wire decoders in
 * this project - dec_i3c.c and the Rust i3c decoder - where T is 1 when the
 * byte holds an even number of ones, making the nine bits together odd.
 *
 * Being computed from the byte, it agrees with the byte by construction and
 * can never fail. That is the whole reason every event it fills in is pushed
 * with `derived` set: a host that checked this against a wire T-bit and found
 * them equal would have learned exactly nothing, and is entitled to be told so
 * rather than left to discover it.
 */
static bool i3c_write_tbit(uint8_t b)
{
    return (__builtin_popcount((unsigned)b) % 2) == 0;
}

/**
 * @brief START and the address byte, stamped when they were asked for.
 *
 * `at` is bb_cycles() read immediately before the MCTRL write that emits the
 * START, not the moment this is called. The two are far apart on purpose: the
 * frame cannot be opened until the ACK is settled, which is nine bit times
 * later for a write and seventeen for a read, and pushing earlier would mean
 * guessing the flag. Stamping at push time instead put the START event 28 us
 * after the wire with 19 us of spread - measured by trace_lag.py - which is
 * nineteen bit times at 1 MHz. Carrying the earlier timestamp costs a word.
 *
 * The address byte takes the same stamp. It follows the START immediately on
 * the wire, well inside what this can resolve, and claiming to tell them apart
 * would be precision this does not have.
 */
static void trace_frame_open(uint8_t addr, bool read, bool acked, uint32_t at)
{
    bb_trace_push_at(BB_EV_START, 0u, false, false, at);
    /* `acked` comes from MERRWARN.NACK, so it is observed rather than derived:
     * whether a target answered is something only this core knows. */
    bb_trace_push_at(BB_EV_ADDR, (uint8_t)((addr << 1) | (read ? 1u : 0u)),
                     acked, false, at);
}

static void trace_frame_close(void)
{
    bb_trace_push(BB_EV_STOP, 0u, false, false);
}

/**
 * @brief Open the frame once, at the first moment its ACK is actually settled.
 *
 * MCTRLDONE is not that moment, and assuming it was put a wrong answer in the
 * trace. On this part the module raises MCTRLDONE for the address phase and
 * sets MERRWARN.NACK afterwards, so a transfer to an absent target traced as
 * `S AAA+` - address acknowledged - while the command itself correctly
 * reported `no-ack-for-0x55`. The reply was right and the trace was confidently
 * wrong, which is the one failure mode a controller-side source has that a wire
 * capture does not.
 *
 * So the event is held until the first fact that settles it: either a NACK
 * appears, or the module takes a byte. A NACK arriving AFTER bytes have been
 * accepted is a different event - the target answered and then the write was
 * aborted - so `A+` is correct in that case and is left alone.
 *
 * Found by tracing a write to an address with nothing on it, which is worth
 * recording: both directions agreed on every frame that worked.
 */
static void trace_addr_once(bool *done, uint8_t addr, bool read, bool acked,
                            uint32_t at)
{
    if (!*done) {
        *done = true;
        trace_frame_open(addr, read, acked, at);
    }
}

/** A frame happened that this trace cannot describe. See bb_trace.h. */
static void trace_hole(void)
{
    bb_trace_push(BB_EV_DROPPED, 0u, false, false);
}

/*
 * Re-mux the pads on every operation, not just the first.
 *
 * SYS:GPIO and SYS:PULSE deliberately steal a pad to drive it as a GPIO, and
 * they leave it that way - that is what makes them useful for wiring checks.
 * Nothing handed the pad back, so the next I3C transfer ran with its pins
 * still muxed to GPIO: the controller read its own stale output level instead
 * of the bus and refused to start, reporting the line as busy.
 *
 * The symptom pointed hard at wiring, and was not wiring. Re-claiming here is
 * cheap (two register writes against a transfer) and makes the peripheral own
 * its pins at the moment it needs them rather than at some earlier time whose
 * assumptions may no longer hold. i2c_claim_pads() exists for the same reason.
 */
static void i3c_claim_pads(void)
{
    /* BB_PAD_I3C, never BB_PAD_I2C: the latter sets ODE, and an open-drain pad
     * cannot drive the push-pull half of an I3C frame. See the note in
     * bb_hal.h - that one bit was the high-rate ceiling. */
    uint32_t pad = s.pull_internal ? BB_PAD_I3C : BB_PAD_PUSHPULL;

    if (s.pad_raw != 0u) {
        /* Mux first so the pad belongs to I3C, then overwrite the pad control
         * the mux helper just chose. Order matters: IOMUXC_SetPinConfig() runs
         * inside bb_pin_mux(). */
        bb_pin_mux(&pt()->scl, pad, true);
        bb_pin_mux(&pt()->sda, pad, true);
        *(volatile uint32_t *)pt()->scl.cfg_reg = s.pad_raw;
        *(volatile uint32_t *)pt()->sda.cfg_reg = s.pad_raw;
        return;
    }
    bb_pin_mux(&pt()->scl, pad, true);
    bb_pin_mux(&pt()->sda, pad, true);
}

static bool i3c_apply(void)
{
    i3c_master_config_t cfg;

    /*
     * I3C drives SCL push-pull itself, so the pads are push-pull rather than
     * open drain - but SDA still has to be readable while the controller
     * drives it, hence SION on both. On a mixed bus the open-drain phases rely
     * on the same read-back.
     *
     * The internal pull-up is off by default and settable through I3C:PULL. It
     * is roughly 40 kOhm: enough to hold a short bench wire high at low speed,
     * nowhere near what a real I3C bus needs. The STM32 build made the same
     * choice for the same reason, and the same warning applies.
     */
    i3c_claim_pads();

    I3C_MasterGetDefaultConfig(&cfg);
    cfg.enableMaster              = kI3C_MasterOn;
    cfg.baudRate_Hz.i3cPushPullBaud  = s.pp_hz;
    cfg.baudRate_Hz.i3cOpenDrainBaud = s.od_hz;
    /*
     * i2cBaud is ALWAYS non-zero, even on a pure I3C bus where no legacy device
     * will ever use it.
     *
     * The first version set it to 0 on a pure bus, which reads as "no I2C here"
     * and is how the field is documented. It hard-faulted the board on the very
     * first hardware run. I3C_MasterSetBaudRate divides by it unconditionally
     * and with no guard:
     *
     *   divEven = (sourceClock_Hz / i2cBaud_HZ) / (2 * (ppBaud+1) * (odBaud+1));
     *
     * so 0 is a divide-by-zero, and on this Cortex-M33 DIV_0_TRP is set, so it
     * traps to DefaultISR. The symptom was a board that loaded, ran, and went
     * silent before printing its banner - with nothing in the build to suggest
     * why. What legacy timing the register ends up holding is irrelevant on a
     * pure bus; that it is computable is not.
     */
    cfg.baudRate_Hz.i2cBaud          = s.i2c_hz;
    /* Open-drain STOP only matters on a mixed bus, where a legacy device has to
     * see a STOP it recognises. */
    cfg.enableOpenDrainStop = s.mixed;
    /*
     * ODHPP drives the open-drain SCL HIGH pulse push-pull, so its width is set
     * by the push-pull rate and not by the open-drain rate: about 83 ns at
     * 6 MHz and 40 ns at 12.5 MHz. The address header of every transfer is an
     * open-drain phase, so a target that needs longer than that to sample SCL
     * stops acknowledging its own address as the push-pull rate rises - which
     * looks like a fast-bus failure and is really a pulse-width one.
     */
    cfg.enableOpenDrainHigh = s.odhpp;

    /*
     * The high-keeper, which nothing in this project has ever set.
     *
     * The SDK default is kI3C_MasterHighKeeperNone, documented as "use PUR to
     * hold SCL high" - it assumes a proper external pull-up resistor. This
     * bench has none: jumper wires and a ~40 kOhm internal pull-up at best. The
     * high-keeper is the on-chip alternative, actively holding the line rather
     * than relying on a resistor that is not there, which is the right thing to
     * try on a bus whose working point moves with its RC.
     */
    cfg.hKeep = (i3c_master_hkeep_t)s.hkeep;

    /*
     * DISTO disables the module's in-frame stall timeout.
     *
     * At 12.5 MHz a private read returns MERRWARN=0x00100000 - TIMEOUT, "the
     * module has stalled too long in a frame" - at every open-drain rate from
     * 100 kHz to 1 MHz. The SDK field for this is even documented as "whether
     * to disable timeout to prevent the ERRWARN". Off by default, because a
     * timeout that fires is usually telling the truth.
     */
    cfg.disableTimeout = s.disto;

    if (s.odstop_set) {
        cfg.enableOpenDrainStop = s.odstop_force;
    }

    /* Only release a peripheral that is actually clocked - see the long note in
     * bb_i2c.c's i2c_apply(). A *_Deinit() gates the clock as its last act and
     * resets registers as its first, so deinitialising twice writes to an
     * unclocked peripheral and stalls the AHB with no fault raised. *_Init()
     * ungates and resets on its own, so skipping the deinit is safe. */
    if (s.configured) {
        I3C_MasterDeinit(pt()->base);
    }
    I3C_MasterInit(pt()->base, &cfg, CLOCK_GetRootClockFreq(pt()->root));

    /* After init, because I3C_MasterSetBaudRate() clears PPLOW on its way past
     * and would undo this. */
    pt()->base->MCONFIG = (pt()->base->MCONFIG & ~(I3C_MCONFIG_PPLOW_MASK |
                                                   I3C_MCONFIG_SKEW_MASK)) |
                          I3C_MCONFIG_PPLOW(s.pplow) |
                          I3C_MCONFIG_SKEW(s.skew);

    s.configured   = true;
    s.target_count = 0u;
    return true;
}

void bb_i3c_init(void)
{
    (void)i3c_apply();
}

/* ---- error mapping ----------------------------------------------------- */
/*
 * The STM32 build's most useful diagnostic was naming the 0x7E no-ACK case
 * rather than reporting a generic protocol error, because that one message
 * answers "why does ENTDAA fail on my new board" in a single line. It is kept
 * verbatim: `protocol-ce2-no-ack-for-7E`.
 */
/*
 * Put the controller back in IDLE after a failed transfer.
 *
 * The SDK refuses any new transfer unless MSTATUS.STATE is IDLE or NORMACT, and
 * reports kStatus_I3C_Busy when it is not - but nothing in the SDK returns the
 * state machine to IDLE after a transfer it aborted. A NAKed ENTDAA leaves the
 * part in the DAA state, so the FIRST failure is reported honestly and every
 * attempt after it comes back "busy" no matter what the bus is doing.
 *
 * That misdiagnoses badly. "Busy" reads as a bus problem - another controller,
 * a target holding SDA - and sends you to the wiring, when the actual cause is
 * a stale register in this part. It cost a long bench session here.
 *
 * NORMACT means a transfer is genuinely open, so a STOP ends it. Anything else
 * non-IDLE is a state machine stranded mid-operation, where FORCEEXIT is the
 * documented escape (RM 65.7.1.4: exits DAA/HDR and returns the bus to idle).
 * Errors and FIFOs are cleared either way, since a stale MERRWARN bit is read
 * as a fresh error by the next transfer.
 */
static void i3c_recover(void)
{
    I3C_Type *base = pt()->base;
    uint32_t  state;

    state = (base->MSTATUS & I3C_MSTATUS_STATE_MASK) >> I3C_MSTATUS_STATE_SHIFT;
    if (state != (uint32_t)kI3C_MasterStateIdle) {
        uint32_t req = (state == (uint32_t)kI3C_MasterStateNormAct)
                           ? (uint32_t)kI3C_RequestEmitStop
                           : (uint32_t)kI3C_RequestForceExit;

        base->MCTRL = (base->MCTRL & ~(I3C_MCTRL_REQUEST_MASK | I3C_MCTRL_DIR_MASK |
                                       I3C_MCTRL_RDTERM_MASK)) |
                      I3C_MCTRL_REQUEST(req);

        /* Bounded: a wedged bus must not turn a failed command into a hang. The
         * count is arbitrary but generous - the request completes in a few bus
         * clocks at any supported rate. */
        for (uint32_t i = 0u; i < 100000u; i++) {
            if ((base->MSTATUS & I3C_MSTATUS_MCTRLDONE_MASK) != 0u) {
                break;
            }
        }
    }

    I3C_MasterClearErrorStatusFlags(base, base->MERRWARN);
    I3C_MasterClearStatusFlags(base, base->MSTATUS);
    base->MDATACTRL |= I3C_MDATACTRL_FLUSHTB_MASK | I3C_MDATACTRL_FLUSHFB_MASK;
}

static bool i3c_reply(status_t st, const char *what)
{
    if (st != kStatus_Success) {
        /* Before i3c_recover(), which clears MERRWARN and the FIFOs - the
         * things most worth seeing. */
        trace_snap(st);
        trace_mark(I3C_PH_ERROR);
        i3c_recover();
    }

    switch (st) {
    case kStatus_Success:
        return true;
    case kStatus_I3C_Nak:
        bb_reply_errf(BB_ERR_NACK, "%s protocol-ce2-no-ack-for-0x%02X%s", what,
                      (unsigned)s_last_addr,
                      (s_last_addr == I3C_BROADCAST_ADDR) ? "-broadcast" : "");
        return false;
    case kStatus_I3C_Timeout:
        bb_reply_errf(BB_ERR_TIMEOUT, "%s-timeout", what);
        return false;
    case kStatus_I3C_Busy:
        bb_reply_errf(BB_ERR_BUS_STATE, "%s-busy", what);
        return false;
    case kStatus_I3C_IBIWon:
        /* Not a failure: a target raised an in-band interrupt and won the bus
         * mid-transfer. The transfer has to be retried, and saying so is more
         * useful than calling it an error. */
        bb_reply_errf(BB_ERR_BUS_STATE, "%s-ibi-won-retry", what);
        return false;
    case kStatus_I3C_SlaveCountExceed:
        bb_reply_errf(BB_ERR_RANGE, "%s-too-many-targets-max-%u",
                      what, (unsigned)I3C_MAX_TARGETS);
        return false;
    case kStatus_I3C_WriteAbort:
        bb_reply_errf(BB_ERR_NACK, "%s-write-aborted-by-target", what);
        return false;
    case kStatus_I3C_SdrParityError:
    case kStatus_I3C_HdrParityError:
        bb_reply_errf(BB_ERR_BUS_STATE, "%s-parity-error", what);
        return false;
    case kStatus_I3C_MsgError:
        if (s_rxlog.extra != 0u) {
            bb_reply_errf(BB_ERR_BUS_STATE,
                          "%s-fifo-over-received-%u-extra-entries-see-I3C:RXLOG?",
                          what, (unsigned)s_rxlog.extra);
        } else {
            bb_reply_errf(BB_ERR_BUS_STATE, "%s-msg-error", what);
        }
        return false;
    case kStatus_I3C_Term:
        bb_reply_errf(BB_ERR_BUS_STATE, "%s-terminated-early", what);
        return false;
    default:
        bb_reply_errf(BB_ERR_HAL, "%s-error-%d", what, (int)st);
        return false;
    }
}

/*
 * The same readiness check without the reply.
 *
 * port_ready() reports its own failure, which is right for a command handler
 * and wrong anywhere else: the poll answers a hot-join outside any command, and
 * an error emitted there lands in the middle of whatever reply the host was
 * actually reading.
 */
static bool port_ready_quiet(void)
{
    if (!s.configured && !i3c_apply()) {
        return false;
    }
    i3c_claim_pads();
    return true;
}

static bool port_ready(void)
{
    if (!port_ready_quiet()) {
        bb_reply_err(BB_ERR_HAL, "i3c-init-failed");
        return false;
    }
    return true;
}

/*
 * Dynamic address assignment, with no reply of its own.
 *
 * Extracted from I3C:ENTDAA so the hot-join responder can use the same code:
 * answering a hot-join *is* running DAA, and a second implementation of it
 * would be a second set of bugs. The rate handling matters and is why this is
 * shared rather than reimplemented - the DAA engine fails above about 6 MHz in
 * a way the ordinary address path does not, so the sequence is run at a capped
 * rate and the configured one restored afterwards.
 */
static status_t i3c_daa_assign(uint8_t *count_out)
{
    uint8_t  pool[I3C_MAX_TARGETS];
    uint32_t saved_pp = s.pp_hz;
    bool     lowered  = false;
    status_t st;

    memcpy(pool, k_daa_pool, sizeof pool);
    s_last_addr = I3C_BROADCAST_ADDR;   /* DAA does not go through xfer() */
    I3C_MasterClearDeviceCount(pt()->base);

    if (s.pp_hz > I3C_DAA_MAX_PP_HZ) {
        s.pp_hz      = I3C_DAA_MAX_PP_HZ;
        s.configured = false;
        if (!port_ready_quiet()) {
            s.pp_hz = saved_pp;
            return kStatus_Fail;
        }
        lowered = true;
    }

    st = I3C_MasterProcessDAASpecifiedBaudrate(pt()->base, pool,
                                               (uint32_t)sizeof pool, NULL);
    if (lowered) {
        /* Back to the configured rate whatever DAA did - a failed discovery
         * must not silently leave the bus running slow. */
        s.pp_hz      = saved_pp;
        s.configured = false;
        (void)port_ready_quiet();
    }

    {
        uint8_t            count = 0u;
        i3c_device_info_t *list  =
            I3C_MasterGetDeviceListAfterDAA(pt()->base, &count);

        if (count > I3C_MAX_TARGETS) {
            count = I3C_MAX_TARGETS;
        }
        if ((list != NULL) && (count > 0u)) {
            memcpy(s.targets, list, (size_t)count * sizeof s.targets[0]);
        }
        s.target_count = count;
        if (count_out != NULL) {
            *count_out = count;
        }
    }
    return st;
}

/* ---- raw transfers ----------------------------------------------------- */

static status_t xfer(uint8_t addr, i3c_direction_t dir, i3c_bus_type_t type,
                     void *data, size_t len, uint32_t flags)
{
    s_last_addr = addr;
    i3c_master_transfer_t t;

    memset(&t, 0, sizeof t);
    t.flags        = flags;
    t.slaveAddress = addr;
    t.direction    = dir;
    t.busType      = type;
    t.data         = data;
    t.dataSize     = len;
    t.ibiResponse  = kI3C_IbiRespNack;

    /*
     * A frame the controller-side trace cannot describe.
     *
     * Every CCC and ENTDAA comes through here, and I3C_MasterTransferBlocking()
     * moves the bytes inside the SDK where nothing can see them. Emitting
     * nothing would be worse than emitting a hole: a host merging this trace
     * against a wire capture would find frames on the wire with no controller
     * record beside them and conclude the controller never sent them.
     */
    bb_trace_push(BB_EV_DROPPED, 0u, false, false);

    return I3C_MasterTransferBlocking(pt()->base, &t);
}

/* ---- commands: bus configuration --------------------------------------- */

static void cmd_port(int argc, char **argv)
{
    uint32_t n;

    (void)argc;
    if (!bb_parse_u32_max(argv[1], BB_I3C_PORT_COUNT, &n) || n < 1u) {
        bb_reply_errf(BB_ERR_RANGE, "port-1..%u", (unsigned)BB_I3C_PORT_COUNT);
        return;
    }
    s.port       = (unsigned)(n - 1u);
    s.configured = false;
    if (!i3c_apply()) {
        bb_reply_err(BB_ERR_HAL, "i3c-init-failed");
        return;
    }
    /* Say what it cost. Port 1 shares its pads with I2C bus 2, so selecting it
     * has just taken them - the host should not have to infer that from the pin
     * map. */
    if (pt()->shares_i2c_bus != 0u) {
        bb_reply_okf("port=%lu pins=%s took-i2c%u",
                     (unsigned long)n, pt()->pins, pt()->shares_i2c_bus);
    } else {
        bb_reply_okf("port=%lu pins=%s", (unsigned long)n, pt()->pins);
    }
}

static void cmd_port_q(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    bb_io_puts("OK");
    for (unsigned i = 0; i < BB_I3C_PORT_COUNT; i++) {
        bb_io_printf(" p%u=%s%s", i + 1u, s_port[i].pins,
                     (i == s.port) ? "*" : "");
    }
    bb_io_puts("\r\n");
}

static void cmd_freq(int argc, char **argv)
{
    uint32_t pp, od;

    if (!bb_parse_u32(argv[1], &pp) || pp < 100000u || pp > 12500000u) {
        /* 12.5 MHz is I3C SDR's push-pull ceiling. */
        bb_reply_err(BB_ERR_RANGE, "pp-100000-12500000");
        return;
    }
    od = s.od_hz;
    if (argc > 2 && (!bb_parse_u32(argv[2], &od) || od < 50000u || od > 4000000u)) {
        bb_reply_err(BB_ERR_RANGE, "od-50000-4000000");
        return;
    }
    s.pp_hz = pp;
    s.od_hz = od;
    s.configured = false;
    if (!i3c_apply()) {
        bb_reply_err(BB_ERR_HAL, "freq-apply-failed");
        return;
    }
    bb_reply_ok();
}

/*
 * Report what the hardware actually produces, not what was asked for.
 *
 * The dividers are small integers - ppFreq = FCLK / 2 / (PPBAUD + 1) with
 * PPBAUD in 0..15 - so only a handful of rates exist and a request lands on
 * whichever one is nearest below. Echoing the request back made every rate look
 * honoured: a sweep of 8, 9, 10, 11, 12 and 12.5 MHz reported six rates while
 * the bus ran at two, and the step between them was read as a failure at
 * "11 MHz" that was really the divider jumping.
 *
 * act_pp/act_od are computed from MCONFIG, so they describe the bus. The
 * requested values are still reported, because the gap between the two is the
 * interesting part.
 */
/*
 * I3C:CLK? - every number between the clock tree and the wire.
 *
 * A requested rate and a generated rate are different values, and reporting one
 * as the other cost this port a documented mistake: a sweep of six push-pull
 * rates was recorded as six rates while the bus ran at two, because the dividers
 * are small integers and a request lands on whichever one is nearest below.
 *
 * Everything here is read back from MCONFIG and the clock tree rather than
 * remembered from the request, and the raw divider fields are printed too -
 * when a rate misbehaves, which PPBAUD and ODBAUD it actually landed on is the
 * first thing worth knowing and cannot be inferred from the frequency alone.
 */
/*
 * I3C:ROOT <hz> - choose the functional clock the dividers work from.
 *
 * Nothing set this before: the peripheral ran on whatever the board clock
 * config left on the root, which is OscRc24M at 24 MHz. That single number
 * decides every rate the part can produce, because push-pull is
 * root / 2 / (PPBAUD + 1) and PPBAUD is a small integer - so at 24 MHz the
 * only rates that exist are 12, 6, 4, 3, 2.4, 2 and down. A request for
 * 12.5 MHz could not be honoured and was silently served as 12, and there is
 * no rate at all between 6 and 12 to probe with.
 *
 * OscRc400M with a chosen divider fixes both: 400/16 gives exactly 25 MHz and
 * therefore exactly 12.5 MHz, the SDR ceiling, and other divisors open up the
 * gap between 6 and 12 that 24 MHz cannot reach.
 *
 * It is an RC oscillator, not a PLL, so the rate it produces is close rather
 * than exact. That is the right way round for a bus with a maximum: landing
 * slightly under 12.5 MHz is legal and landing over it is not.
 *
 * Re-initialisation is forced rather than optional. MCONFIG's dividers were
 * computed from the old root, and leaving them would run the bus at a rate
 * nobody asked for while every report claimed the new one.
 */
/*
 * I3C:ODHPP <0|1> - drive the open-drain SCL high pulse push-pull, or not.
 *
 * On by default because that is the SDK's default and what every measurement
 * here was taken with. Off makes the open-drain phases genuinely open drain, so
 * the high pulse is set by the open-drain rate instead of the push-pull one.
 */
static void cmd_odhpp(int argc, char **argv)
{
    uint32_t on = 1u;

    if (argc > 1) {
        if (!bb_parse_u32_max(argv[1], 1u, &on)) {
            bb_reply_err(BB_ERR_RANGE, "expected-0-or-1");
            return;
        }
        s.odhpp      = (on != 0u);
        s.configured = false;   /* MCONFIG carries it */
    }
    bb_reply_okf("odhpp=%u", s.odhpp ? 1u : 0u);
}

static void cmd_root(int argc, char **argv)
{
    uint32_t want = 0u;

    if (!bb_parse_u32_max(argv[1], 400000000u, &want) || want == 0u) {
        bb_reply_err(BB_ERR_RANGE, "root-hz-1..400000000");
        return;
    }

    /*
     * A ceiling, enforced before the register is written.
     *
     * OscRc400M divided by 1 puts 400 MHz into the I3C functional clock, and
     * the peripheral does not survive it: the board stops answering USB
     * entirely and needs reflashing. That is not a bus failure to be reported,
     * it is a configuration that must not be applied - so it is refused rather
     * than clamped, because a caller who asked for 400 MHz should be told the
     * number was wrong instead of being handed a different one.
     *
     * 100 MHz is well above anything this driver needs: 12.5 MHz push-pull,
     * the fastest SDR rate there is, wants a 25 MHz root.
     */
    if (want > I3C_ROOT_MAX_HZ) {
        bb_reply_errf(BB_ERR_RANGE,
                      "root-max-%lu-hz-higher-stalls-the-peripheral",
                      (unsigned long)I3C_ROOT_MAX_HZ);
        return;
    }

    uint32_t div = (400000000u + (want / 2u)) / want;
    if (div == 0u) {
        div = 1u;
    }
    if (div > 256u) {
        div = 256u;
    }

    clock_root_config_t rc;
    memset(&rc, 0, sizeof rc);
    rc.clockOff = false;
    rc.mux      = 1u;      /* OscRc400M, on both I3C roots */
    rc.div      = div;
    CLOCK_SetRootClock(pt()->root, &rc);

    s.configured = false;  /* MCONFIG must be recomputed from the new source */

    /*
     * Say whether the configured bus rates still fit, rather than leaving it to
     * be discovered by a transfer.
     *
     * The push-pull rate is root / 2 / (PPBAUD+1) with PPBAUD in four bits, so
     * a root below twice the requested rate cannot reach it at all and a root
     * more than 32 times it cannot divide down to it. The open-drain low period
     * is an eight-bit count, so the reachable floor there is root / 2 / 256.
     *
     * A root outside those bounds used to be accepted silently: MCONFIG was
     * recomputed with whatever the divider arithmetic produced, the bus ran at
     * a rate nobody asked for, and the failure surfaced later as a target that
     * would not acknowledge. Worse, some combinations stalled the peripheral
     * outright and the only way back was a reflash - that part is now handled
     * by the bounded waits in fsl_i3c.c, but a wrong rate accepted without
     * comment is still a wrong rate.
     */
    uint32_t got   = CLOCK_GetRootClockFreq(pt()->root);
    uint32_t pp_lo = got / 2u / 16u;       /* PPBAUD at its maximum */
    uint32_t pp_hi = got / 2u;             /* PPBAUD == 0           */
    uint32_t od_lo = got / 2u / 256u;
    const char *fit = "ok";

    if (s.pp_hz > pp_hi) {
        fit = "pp-too-fast-for-this-root";
    } else if (s.pp_hz < pp_lo) {
        fit = "pp-too-slow-for-this-root";
    } else if (s.od_hz < od_lo) {
        fit = "od-too-slow-for-this-root";
    }

    bb_reply_okf("root_hz=%lu div=%lu requested=%lu pp_range=%lu..%lu "
                 "od_min=%lu fit=%s",
                 (unsigned long)got, (unsigned long)div, (unsigned long)want,
                 (unsigned long)pp_lo, (unsigned long)pp_hi,
                 (unsigned long)od_lo, fit);
}

/*
 * I3C:DAADBG - dynamic address assignment driven straight from the registers.
 *
 * Runbook experiment 5, and the point is not to have another DAA: it is to see
 * MERRWARN at the moment it first goes non-zero. The SDK's DAA loop calls
 * I3C_MasterCheckAndClearError() every iteration, which classifies the error
 * and then CLEARS the register - so by the time a caller sees a failure the
 * evidence is gone. That is why the first trace here read merrwarn=0 on a
 * transfer the hardware had definitely marked as NACKed; the zero was the SDK
 * tidying up, not the absence of an error.
 *
 * This runs the same sequence - emit ProcessDAA, drain the eight bytes the
 * target drives, write the assigned address when the controller says it is
 * between devices - and on the first sign of trouble stops and reports the raw
 * registers instead of interpreting them.
 *
 * Deliberately minimal. No retries, no recovery, no device list: those are the
 * SDK's job and duplicating them would reintroduce the thing being measured.
 */
static void cmd_daadbg(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    if (!port_ready()) {
        return;
    }

    I3C_Type *base = pt()->base;

    /* Start from a known state, by hand. */
    I3C_MasterClearErrorStatusFlags(base, base->MERRWARN);
    I3C_MasterClearStatusFlags(base, base->MSTATUS);
    base->MDATACTRL |= I3C_MDATACTRL_FLUSHTB_MASK | I3C_MDATACTRL_FLUSHFB_MASK;

    uint32_t st0     = base->MSTATUS;
    uint8_t  rx[8];
    uint32_t rx_n    = 0u;
    uint32_t devices = 0u;
    uint32_t iters   = 0u;
    bool     ctrldone = false;

    /* Captured at the first non-zero MERRWARN and never overwritten. */
    uint32_t err_first = 0u;
    uint32_t err_stat  = 0u;
    uint32_t err_iter  = 0u;
    uint32_t err_rx    = 0u;

    base->MCTRL = (base->MCTRL & ~(I3C_MCTRL_REQUEST_MASK | I3C_MCTRL_DIR_MASK |
                                   I3C_MCTRL_RDTERM_MASK)) |
                  I3C_MCTRL_REQUEST(kI3C_RequestProcessDAA);

    while (iters++ < 200000u) {
        uint32_t status = base->MSTATUS;
        uint32_t err    = base->MERRWARN;

        if ((err != 0u) && (err_first == 0u)) {
            err_first = err;
            err_stat  = status;
            err_iter  = iters;
            err_rx    = rx_n;
            break;            /* stop on the first sign, registers intact */
        }

        if (!ctrldone || (rx_n < 8u)) {
            uint32_t rxcount = (base->MDATACTRL & I3C_MDATACTRL_RXCOUNT_MASK)
                               >> I3C_MDATACTRL_RXCOUNT_SHIFT;
            if ((rxcount != 0u) && (rx_n < 8u)) {
                rx[rx_n++] = (uint8_t)(base->MRDATAB & 0xFFu);
            }
            if ((status & I3C_MSTATUS_MCTRLDONE_MASK) != 0u) {
                I3C_MasterClearStatusFlags(base, I3C_MSTATUS_MCTRLDONE_MASK);
                ctrldone = true;
            }
        } else if ((((status & I3C_MSTATUS_STATE_MASK) >> I3C_MSTATUS_STATE_SHIFT)
                        == (uint32_t)kI3C_MasterStateDaa) &&
                   ((status & I3C_MSTATUS_BETWEEN_MASK) != 0u)) {
            base->MWDATAB = k_daa_pool[devices];
            base->MCTRL = (base->MCTRL & ~(I3C_MCTRL_REQUEST_MASK |
                                           I3C_MCTRL_DIR_MASK |
                                           I3C_MCTRL_RDTERM_MASK)) |
                          I3C_MCTRL_REQUEST(kI3C_RequestProcessDAA);
            devices++;
            ctrldone = false;
            rx_n     = 0u;
            if (devices >= 1u) {
                break;        /* one device is enough to answer the question */
            }
        }
    }

    uint32_t st_end = base->MSTATUS;
    uint32_t er_end = base->MERRWARN;

    bb_reply_okf("devices=%lu iters=%lu rx_n=%lu "
                 "rx=%02X%02X%02X%02X%02X%02X%02X%02X "
                 "mstatus_begin=0x%08lX mstatus_end=0x%08lX merrwarn_end=0x%08lX "
                 "first_err=0x%08lX at_iter=%lu with_mstatus=0x%08lX after_rx=%lu "
                 "nacked=%u",
                 (unsigned long)devices, (unsigned long)iters,
                 (unsigned long)rx_n,
                 rx[0], rx[1], rx[2], rx[3], rx[4], rx[5], rx[6], rx[7],
                 (unsigned long)st0, (unsigned long)st_end,
                 (unsigned long)er_end,
                 (unsigned long)err_first, (unsigned long)err_iter,
                 (unsigned long)err_stat, (unsigned long)err_rx,
                 ((err_first & I3C_MERRWARN_NACK_MASK) != 0u) ? 1u : 0u);

    i3c_recover();
}

/*
 * I3C:PPLOW <n> - stretch the push-pull SCL low period by n clocks.
 *
 * I3C_MasterSetBaudRate() masks PPLOW out and never writes it back, so the
 * push-pull clock always comes out symmetric: high and low the same width. That
 * is legal and it leaves the target the minimum possible time to present the
 * next bit before the controller samples it.
 *
 * Which is what a private read looks like when it fails here. Above about 1 MHz
 * every byte after the first comes back as a copy of the first, while the
 * capture shows the target driving the correct bytes on the wire - so the data
 * is there and the controller is latching the same one repeatedly. The receive
 * FIFO is not the problem either: it fills to four or five entries and drains
 * normally, it just holds the same value in every slot.
 *
 * Stretching the low period moves the controller's sampling point later
 * relative to the target's drive without changing the push-pull rate the bus
 * is nominally running at.
 */
static void cmd_pplow(int argc, char **argv)
{
    uint32_t n = 0u;

    if (argc > 1) {
        if (!bb_parse_u32_max(argv[1], 255u, &n)) {
            bb_reply_err(BB_ERR_RANGE, "pplow-0..255");
            return;
        }
        s.pplow = (uint8_t)n;
        if (s.configured) {
            I3C_Type *base = pt()->base;

            base->MCONFIG = (base->MCONFIG & ~I3C_MCONFIG_PPLOW_MASK) |
                            I3C_MCONFIG_PPLOW(s.pplow);
        }
    }
    bb_reply_okf("pplow=%u mconfig=0x%08lX", (unsigned)s.pplow,
                 s.configured ? (unsigned long)pt()->base->MCONFIG : 0uL);
}




/* I3C:HKEEP <0..3> - MCONFIG.HKEEP: none, wired-in, passive SDA, passive both. */
static void cmd_hkeep(int argc, char **argv)
{
    static const char *const name[] = {
        "none", "wired-in", "passive-sda", "passive-sda-scl"
    };
    uint32_t n;

    if (argc > 1) {
        if (!bb_parse_u32_max(argv[1], 3u, &n)) {
            bb_reply_err(BB_ERR_RANGE, "hkeep-0..3");
            return;
        }
        s.hkeep = (uint8_t)n;
        s.configured = false;          /* re-init picks it up */
    }
    bb_reply_okf("hkeep=%u %s", (unsigned)s.hkeep, name[s.hkeep]);
}

/* I3C:DISTO <0|1> - MCONFIG.DISTO, the module's in-frame stall timeout. */
static void cmd_disto(int argc, char **argv)
{
    uint32_t n;

    if (argc > 1) {
        if (!bb_parse_u32_max(argv[1], 1u, &n)) {
            bb_reply_err(BB_ERR_RANGE, "disto-0..1");
            return;
        }
        s.disto = (n != 0u);
        s.configured = false;
    }
    bb_reply_okf("disto=%u", s.disto ? 1u : 0u);
}

/* I3C:ODSTOP <0|1> - emit the STOP at open-drain speed, overriding I3C:BUS. */
static void cmd_odstop(int argc, char **argv)
{
    uint32_t n;

    if (argc > 1) {
        if (!bb_parse_u32_max(argv[1], 1u, &n)) {
            bb_reply_err(BB_ERR_RANGE, "odstop-0..1");
            return;
        }
        s.odstop_force = (n != 0u);
        s.odstop_set   = true;
        s.configured   = false;
    }
    bb_reply_okf("odstop=%u overridden=%u",
                 s.odstop_set ? (s.odstop_force ? 1u : 0u) : (s.mixed ? 1u : 0u),
                 s.odstop_set ? 1u : 0u);
}

/*
 * I3C:PAD [hex] - write SW_PAD_CTL for both I3C pads directly.
 *
 * The target's pad slew turned out to decide whether this controller's receive
 * FIFO duplicates entries. The controller's own pads are the other half of that
 * link and have never been varied: bb_pin_mux() picks one of three fixed
 * encodings and nothing chooses drive strength or slew.
 *
 * Rather than trust a field map - this part has two SW_PAD_CTL layouts and the
 * project has already been bitten once by applying the wrong one - this writes
 * the register raw so the bus can say which bits matter. 0 restores the normal
 * encoding.
 */
static void cmd_pad(int argc, char **argv)
{
    uint32_t n;

    if (argc > 1) {
        if (!bb_parse_u32_max(argv[1], 0xFFu, &n)) {
            bb_reply_err(BB_ERR_RANGE, "pad-0..0xFF");
            return;
        }
        s.pad_raw = n;
        if (s.configured) {
            i3c_claim_pads();
        }
    }
    bb_reply_okf("pad=0x%02lX scl_cfg=0x%08lX sda_cfg=0x%08lX "
                 "scl_now=0x%08lX sda_now=0x%08lX",
                 (unsigned long)s.pad_raw,
                 (unsigned long)pt()->scl.cfg_reg,
                 (unsigned long)pt()->sda.cfg_reg,
                 (unsigned long)(*(volatile uint32_t *)pt()->scl.cfg_reg),
                 (unsigned long)(*(volatile uint32_t *)pt()->sda.cfg_reg));
}



static void cmd_trace_q(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    uint32_t state = (s_trace.mstatus & I3C_MSTATUS_STATE_MASK)
                     >> I3C_MSTATUS_STATE_SHIFT;
    uint32_t txc   = (s_trace.mdatactrl & I3C_MDATACTRL_TXCOUNT_MASK)
                     >> I3C_MDATACTRL_TXCOUNT_SHIFT;
    uint32_t rxc   = (s_trace.mdatactrl & I3C_MDATACTRL_RXCOUNT_MASK)
                     >> I3C_MDATACTRL_RXCOUNT_SHIFT;

    bb_reply_okf("phase=0x%02X status=%d addr=0x%02X "
                 "mstatus=0x%08lX state=%lu between=%u "
                 "merrwarn=0x%08lX mctrl=0x%08lX mdatactrl=0x%08lX "
                 "tx=%lu rx=%lu mconfig=0x%08lX root_hz=%lu",
                 (unsigned)s_trace.phase, (int)s_trace.status,
                 (unsigned)s_trace.addr,
                 (unsigned long)s_trace.mstatus, (unsigned long)state,
                 ((s_trace.mstatus & I3C_MSTATUS_BETWEEN_MASK) != 0u) ? 1u : 0u,
                 (unsigned long)s_trace.merrwarn,
                 (unsigned long)s_trace.mctrl,
                 (unsigned long)s_trace.mdatactrl,
                 (unsigned long)txc, (unsigned long)rxc,
                 (unsigned long)s_trace.mconfig,
                 (unsigned long)s_trace.root_hz);
}

static void cmd_clk_q(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    if (!s.configured) {
        bb_reply_okf("requested_pp=%lu requested_od=%lu actual=not-configured",
                     (unsigned long)s.pp_hz, (unsigned long)s.od_hz);
        return;
    }

    uint32_t cfg     = pt()->base->MCONFIG;
    uint32_t ppbaud  = (cfg & I3C_MCONFIG_PPBAUD_MASK) >> I3C_MCONFIG_PPBAUD_SHIFT;
    uint32_t odbaud  = (cfg & I3C_MCONFIG_ODBAUD_MASK) >> I3C_MCONFIG_ODBAUD_SHIFT;
    uint32_t i2cbaud = (cfg & I3C_MCONFIG_I2CBAUD_MASK) >> I3C_MCONFIG_I2CBAUD_SHIFT;
    bool     odhpp   = (cfg & I3C_MCONFIG_ODHPP_MASK) != 0u;
    uint32_t root_hz = CLOCK_GetRootClockFreq(pt()->root);

    uint32_t act_pp = root_hz / 2u / (ppbaud + 1u);
    uint32_t act_od = odhpp ? ((2u * act_pp) / (odbaud + 2u))
                            : (act_pp / (odbaud + 1u));

    bb_reply_okf("root_hz=%lu requested_pp=%lu actual_pp=%lu "
                 "requested_od=%lu actual_od=%lu "
                 "ppbaud=%lu odbaud=%lu i2cbaud=%lu odhpp=%u mconfig=0x%08lX",
                 (unsigned long)root_hz,
                 (unsigned long)s.pp_hz, (unsigned long)act_pp,
                 (unsigned long)s.od_hz, (unsigned long)act_od,
                 (unsigned long)ppbaud, (unsigned long)odbaud,
                 (unsigned long)i2cbaud, odhpp ? 1u : 0u,
                 (unsigned long)cfg);
}

static void cmd_freq_q(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    if (!s.configured) {
        bb_reply_okf("pp=%lu od=%lu i2c=%lu act=not-configured",
                     (unsigned long)s.pp_hz, (unsigned long)s.od_hz,
                     (unsigned long)(s.mixed ? s.i2c_hz : 0u));
        return;
    }

    uint32_t cfg    = pt()->base->MCONFIG;
    uint32_t ppbaud = (cfg & I3C_MCONFIG_PPBAUD_MASK) >> I3C_MCONFIG_PPBAUD_SHIFT;
    uint32_t odbaud = (cfg & I3C_MCONFIG_ODBAUD_MASK) >> I3C_MCONFIG_ODBAUD_SHIFT;
    bool     odhpp  = (cfg & I3C_MCONFIG_ODHPP_MASK) != 0u;
    uint32_t fclk   = CLOCK_GetRootClockFreq(pt()->root);

    uint32_t act_pp = fclk / 2u / (ppbaud + 1u);
    uint32_t act_od = odhpp ? ((2u * act_pp) / (odbaud + 2u))
                            : (act_pp / (odbaud + 1u));

    bb_reply_okf("pp=%lu od=%lu i2c=%lu act_pp=%lu act_od=%lu fclk=%lu",
                 (unsigned long)s.pp_hz, (unsigned long)s.od_hz,
                 (unsigned long)(s.mixed ? s.i2c_hz : 0u),
                 (unsigned long)act_pp, (unsigned long)act_od,
                 (unsigned long)fclk);
}

static void cmd_bus(int argc, char **argv)
{
    (void)argc;
    if (bb_streq_ci(argv[1], "pure")) {
        s.mixed = false;
    } else if (bb_streq_ci(argv[1], "mixed")) {
        s.mixed = true;
    } else {
        bb_reply_err(BB_ERR_RANGE, "pure-or-mixed");
        return;
    }
    s.configured = false;
    if (!i3c_apply()) {
        bb_reply_err(BB_ERR_HAL, "bus-apply-failed");
        return;
    }
    bb_reply_ok();
}

static void cmd_pull(int argc, char **argv)
{
    uint32_t on;

    (void)argc;
    if (!bb_parse_u32_max(argv[1], 1u, &on)) {
        bb_reply_err(BB_ERR_RANGE, "0-or-1");
        return;
    }
    s.pull_internal = (on != 0u);
    s.configured    = false;
    if (!i3c_apply()) {
        bb_reply_err(BB_ERR_HAL, "pull-apply-failed");
        return;
    }
    bb_reply_ok();
}

static void cmd_free(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    /*
     * Let go of the pins entirely: inputs, no pull.
     *
     * An I3C controller drives SCL push-pull, so two initialised controllers on
     * one wire fight each other and the loser is whichever line you were trying
     * to read. A released pin contributes nothing - which is what a board needs
     * when it is listening rather than talking, and what makes a two-board rig
     * possible at all.
     */
    if (s.configured) {
        I3C_MasterDeinit(pt()->base);
    }
    bb_pin_gpio(&pt()->scl, false, false, BB_PAD_INPUT);
    bb_pin_gpio(&pt()->sda, false, false, BB_PAD_INPUT);
    s.configured = false;
    bb_reply_ok();
}

/* ---- commands: dynamic address assignment ------------------------------ */

static void cmd_entdaa(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    trace_mark(I3C_PH_ENTER);
    if (!port_ready()) {
        return;
    }
    trace_mark(I3C_PH_READY);
    trace_mark(I3C_PH_PRE_DAA);

    uint8_t  count = 0u;
    status_t st    = i3c_daa_assign(&count);

    if (!i3c_reply(st, "entdaa")) {
        return;
    }

    /* One line per target would be several replies; the protocol is one reply
     * per command, so they are streamed into a single OK line the way the pin
     * map is. The 48-bit PID is the vendor ID and part number as ENTDAA
     * reported them. */
    bb_io_printf("OK targets=%u", (unsigned)count);
    for (unsigned i = 0; i < count; i++) {
        bb_io_printf(" %02X:bcr=%02X,dcr=%02X,pid=%04X%08lX",
                     (unsigned)s.targets[i].dynamicAddr,
                     (unsigned)s.targets[i].bcr,
                     (unsigned)s.targets[i].dcr,
                     (unsigned)s.targets[i].vendorID,
                     (unsigned long)s.targets[i].partNumber);
    }
    bb_io_puts("\r\n");
}

static void cmd_rstdaa(int argc, char **argv)
{
    uint8_t ccc = 0x06u;    /* RSTDAA, broadcast */

    (void)argc;
    (void)argv;
    if (!port_ready()) {
        return;
    }
    status_t st = xfer(I3C_BROADCAST_ADDR, kI3C_Write, kI3C_TypeI3CSdr,
                       &ccc, 1u, kI3C_TransferDefaultFlag);
    if (!i3c_reply(st, "rstdaa")) {
        return;
    }
    I3C_MasterClearDeviceCount(pt()->base);
    s.target_count = 0u;
    bb_reply_ok();
}

static void cmd_targets_q(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    /*
     * `targets=<n>` on the OK line, then one `#` comment line per target.
     *
     * Not a style choice: bb_i3c_targets() reads the count with
     * bb_field_int(payload, "targets", ...) and then takes the target details
     * from the *comment* lines the transport collected, one target per line.
     * An inline list on the OK line - however complete - leaves the host with
     * a count it cannot match to any target, which it reports as a malformed
     * reply.
     */
    bb_io_printf("OK targets=%u", (unsigned)s.target_count);
    bb_io_puts("\r\n");

    for (unsigned i = 0; i < s.target_count; i++) {
        /*
         * payload= is the 8-byte ENTDAA payload as one 64-bit hex value with
         * the *first byte the target sent* in the low bits - the order the
         * host undoes to recover the 48-bit provisioned id. ENTDAA sends the
         * PID most-significant byte first, then BCR, then DCR:
         *
         *   bits  0..47  PID bytes in transmission order (PID[47:40] lowest)
         *   bits 48..55  BCR
         *   bits 56..63  DCR
         *
         * The SDK hands the PID back already split into its MIPI fields, so it
         * is reassembled here: bits 47..33 are the manufacturer id and bits
         * 31..0 the part number, with bit 32 the provisioned-id type.
         *
         * Untested against real silicon - nothing I3C is wired to the board -
         * so this is the format the host documents rather than a measurement.
         */
        uint64_t pid = ((uint64_t)s.targets[i].vendorID << 33) |
                       (uint64_t)s.targets[i].partNumber;
        uint64_t raw = 0;

        for (unsigned b = 0; b < 6u; b++) {
            raw |= (uint64_t)((pid >> (8u * (5u - b))) & 0xFFu) << (8u * b);
        }
        raw |= (uint64_t)s.targets[i].bcr << 48;
        raw |= (uint64_t)s.targets[i].dcr << 56;

        /* ibi is BCR bit 1, "IBI request capable". slot is this table index -
         * the firmware has no other notion of a hardware slot, and saying so
         * beats inventing one. */
        bb_io_printf("# addr=0x%02X bcr=0x%02X dcr=0x%02X ibi=%u slot=%u"
                     " payload=%08lX%08lX",
                     (unsigned)s.targets[i].dynamicAddr,
                     (unsigned)s.targets[i].bcr,
                     (unsigned)s.targets[i].dcr,
                     (unsigned)((s.targets[i].bcr >> 1) & 1u),
                     i,
                     (unsigned long)(uint32_t)(raw >> 32),
                     (unsigned long)(uint32_t)(raw & 0xFFFFFFFFu));
        bb_io_puts("\r\n");
    }
}

/* ---- commands: CCC ----------------------------------------------------- */

static void cmd_ccc_b(int argc, char **argv)
{
    uint32_t ccc;
    uint8_t  buf[BB_PAYLOAD_MAX];
    size_t   len = 0u;

    if (!bb_parse_u32_max(argv[1], 0xFFu, &ccc)) {
        bb_reply_err(BB_ERR_RANGE, "ccc-0x00-0xFF");
        return;
    }
    /* A broadcast CCC code has bit 7 clear by definition; a direct one has it
     * set. Refusing the wrong class here is better than emitting a frame the
     * bus will interpret as the other kind. */
    if ((ccc & 0x80u) != 0u) {
        bb_reply_err(BB_ERR_RANGE, "broadcast-ccc-must-be-under-0x80");
        return;
    }
    buf[len++] = (uint8_t)ccc;
    size_t extra = 0u;
    if (argc > 2) {
        if (!bb_parse_bytes(argc, argv, 2, &buf[1], sizeof buf - 1u, &extra)) {
            bb_reply_errf(BB_ERR_TOO_LONG, "payload-max-%u",
                          (unsigned)(BB_PAYLOAD_MAX - 1u));
            return;
        }
    }
    if (!port_ready()) {
        return;
    }
    if (i3c_reply(xfer(I3C_BROADCAST_ADDR, kI3C_Write, kI3C_TypeI3CSdr,
                       buf, len + extra, kI3C_TransferDefaultFlag), "ccc-b")) {
        bb_reply_ok();
    }
}

static void cmd_ccc_w(int argc, char **argv)
{
    uint32_t addr, ccc;
    uint8_t  buf[BB_PAYLOAD_MAX];
    size_t   extra = 0u;

    if (!bb_parse_u32_max(argv[1], 0x7Fu, &addr) ||
        !bb_parse_u32_max(argv[2], 0xFFu, &ccc)) {
        bb_reply_err(BB_ERR_RANGE, "addr7-and-ccc");
        return;
    }
    if ((ccc & 0x80u) == 0u) {
        bb_reply_err(BB_ERR_RANGE, "direct-ccc-must-be-0x80-or-above");
        return;
    }
    if (argc > 3 &&
        !bb_parse_bytes(argc, argv, 3, buf, sizeof buf, &extra)) {
        bb_reply_errf(BB_ERR_TOO_LONG, "payload-max-%u", (unsigned)BB_PAYLOAD_MAX);
        return;
    }
    if (!port_ready()) {
        return;
    }

    /*
     * A direct CCC is two frames: the code broadcast to 0x7E, then a repeated
     * START addressing the target. Sending the code without the repeated START
     * would leave the bus expecting an address it never gets.
     */
    uint8_t code = (uint8_t)ccc;
    status_t st = xfer(I3C_BROADCAST_ADDR, kI3C_Write, kI3C_TypeI3CSdr,
                       &code, 1u, kI3C_TransferNoStopFlag);
    if (!i3c_reply(st, "ccc-w-header")) {
        (void)I3C_MasterStop(pt()->base);
        return;
    }
    st = xfer((uint8_t)addr, kI3C_Write, kI3C_TypeI3CSdr,
              buf, extra, kI3C_TransferDefaultFlag);
    if (i3c_reply(st, "ccc-w")) {
        bb_reply_ok();
    }
}

static void cmd_ccc_r(int argc, char **argv)
{
    uint32_t addr, ccc, count;
    uint8_t  buf[BB_PAYLOAD_MAX];

    (void)argc;
    if (!bb_parse_u32_max(argv[1], 0x7Fu, &addr) ||
        !bb_parse_u32_max(argv[2], 0xFFu, &ccc)) {
        bb_reply_err(BB_ERR_RANGE, "addr7-and-ccc");
        return;
    }
    if ((ccc & 0x80u) == 0u) {
        bb_reply_err(BB_ERR_RANGE, "direct-ccc-must-be-0x80-or-above");
        return;
    }
    if (!bb_parse_u32_max(argv[3], BB_PAYLOAD_MAX, &count) || count == 0u) {
        bb_reply_errf(BB_ERR_RANGE, "count-1..%u", (unsigned)BB_PAYLOAD_MAX);
        return;
    }
    if (!port_ready()) {
        return;
    }

    uint8_t  code = (uint8_t)ccc;
    status_t st   = xfer(I3C_BROADCAST_ADDR, kI3C_Write, kI3C_TypeI3CSdr,
                         &code, 1u, kI3C_TransferNoStopFlag);
    if (!i3c_reply(st, "ccc-r-header")) {
        (void)I3C_MasterStop(pt()->base);
        return;
    }
    st = xfer((uint8_t)addr, kI3C_Read, kI3C_TypeI3CSdr,
              buf, count, kI3C_TransferDefaultFlag);
    if (i3c_reply(st, "ccc-r")) {
        bb_reply_ok_bytes(buf, count);
    }
}

/* ---- commands: private transfers --------------------------------------- */

static void cmd_write(int argc, char **argv)
{
    uint32_t addr;
    uint8_t  buf[BB_PAYLOAD_MAX];
    size_t   len;

    if (!bb_parse_u32_max(argv[1], 0x7Fu, &addr)) {
        bb_reply_err(BB_ERR_RANGE, "addr7");
        return;
    }
    if (!bb_parse_bytes(argc, argv, 2, buf, sizeof buf, &len) || len == 0u) {
        bb_reply_errf(BB_ERR_TOO_LONG, "payload-1..%u", (unsigned)BB_PAYLOAD_MAX);
        return;
    }
    if (!port_ready()) {
        return;
    }
    if (i3c_reply(i3c_write_typed((uint8_t)addr, buf, (uint32_t)len,
                                  (uint32_t)kI3C_TypeI3CSdr), "write")) {
        bb_reply_ok();
    }
}

/*
 * A private read driven from the registers, because the SDK's loop pops the
 * receive FIFO faster than it refills at rate.
 *
 * Symptom: every byte after the first comes back as a copy of the first. The
 * capture shows the target driving the right data - `S A13+ D40+ D11+ D22+ ...`
 * on the wire while the controller reports `40 40 40 40`. One-byte reads are
 * always correct; two or more are not, and only above about 1 MHz.
 *
 * That is the shape of a stale read rather than a bus fault. At 1 MHz the bytes
 * arrive slowly enough that the wait for RXCOUNT separates every pop in time. At
 * 2 MHz the whole transfer is already in the FIFO before software starts, the
 * pops run back to back, and MRDATAB is read again before the peripheral has
 * presented the next byte.
 *
 * So each pop is fenced and RXCOUNT is re-read from the peripheral in between -
 * never cached across a byte. The barrier is the point: without it the compiler
 * and the write buffer are both entitled to do exactly what went wrong.
 */
/*
 * One transmit path, bounded and instrumented, mirroring i3c_read_typed().
 *
 * The SDK's blocking write is correct but opaque: when it fails, every wait
 * inside it looks the same from outside. Here each stage is recorded, so a
 * failure says whether the address went out, whether the bytes reached the
 * FIFO, and which wait ran out.
 */
static status_t i3c_write_typed(uint8_t addr, const uint8_t *buf, uint32_t len,
                                uint32_t type)
{
    I3C_Type *base = pt()->base;
    uint32_t  guard;

    if ((len == 0u) || (len > 255u)) {
        return kStatus_InvalidArgument;
    }

    s_last_addr    = addr;
    s_txlog.n      = 0u;
    s_txlog.stage  = (uint8_t)I3C_TX_ENTER;

    bool     addr_traced = false;
    uint32_t t_start     = 0u;

    I3C_MasterClearErrorStatusFlags(base, base->MERRWARN);
    I3C_MasterClearStatusFlags(base, base->MSTATUS);
    base->MDATACTRL |= I3C_MDATACTRL_FLUSHTB_MASK;
    s_txlog.pre = (uint8_t)((base->MDATACTRL & I3C_MDATACTRL_TXCOUNT_MASK)
                            >> I3C_MDATACTRL_TXCOUNT_SHIFT);

    /* The last thing read before the module is told to drive the START, so
     * it is as close to the wire as this core can get. */
    t_start = bb_cycles();
    base->MCTRL = I3C_MCTRL_REQUEST(kI3C_RequestEmitStartAddr) |
                  I3C_MCTRL_TYPE(type) |
                  I3C_MCTRL_DIR(0u) |
                  I3C_MCTRL_ADDR(addr);

    for (guard = 0u; guard < 500000u; guard++) {
        if ((base->MERRWARN & I3C_MERRWARN_NACK_MASK) != 0u) {
            s_txlog.spin_start = guard;
            trace_addr_once(&addr_traced, addr, false, false, t_start);
            return kStatus_I3C_Nak;
        }
        if ((base->MSTATUS & I3C_MSTATUS_MCTRLDONE_MASK) != 0u) {
            break;
        }
    }
    s_txlog.spin_start = guard;
    if (guard >= 500000u) {
        return kStatus_I3C_Timeout;
    }
    I3C_MasterClearStatusFlags(base, I3C_MSTATUS_MCTRLDONE_MASK);
    s_txlog.stage = (uint8_t)I3C_TX_STARTED;

    for (uint32_t i = 0u; i < len; i++) {
        bool     room = false;
        uint32_t txc  = 0u;

        for (guard = 0u; guard < 500000u; guard++) {
            uint32_t err = base->MERRWARN;

            if ((err & (I3C_MERRWARN_NACK_MASK |
                        I3C_MERRWARN_WRABT_MASK)) != 0u) {
                s_txlog.merrwarn_end = err;
                /* Before any byte was taken, so this NACK is the address's. */
                trace_addr_once(&addr_traced, addr, false, false, t_start);
                return ((err & I3C_MERRWARN_NACK_MASK) != 0u)
                           ? kStatus_I3C_Nak : kStatus_I3C_WriteAbort;
            }
            __DMB();
            txc = (base->MDATACTRL & I3C_MDATACTRL_TXCOUNT_MASK)
                  >> I3C_MDATACTRL_TXCOUNT_SHIFT;
            if ((base->MSTATUS & I3C_MSTATUS_TXNOTFULL_MASK) != 0u) {
                room = true;
                break;
            }
        }
        if (s_txlog.n < I3C_RXLOG_MAX) {
            s_txlog.val[s_txlog.n]  = buf[i];
            s_txlog.cnt[s_txlog.n]  = (uint8_t)txc;
            s_txlog.spin[s_txlog.n] = guard;
            s_txlog.n++;
        }
        if (!room) {
            s_txlog.mstatus_end  = base->MSTATUS;
            s_txlog.merrwarn_end = base->MERRWARN;
            /* No NACK and no room: the frame went somewhere this trace cannot
             * describe, which is a hole rather than an absent frame. */
            trace_hole();
            return kStatus_I3C_Timeout;
        }

        /* The module has room, so it accepted the address: the first point
         * at which that is a fact rather than an assumption. */
        trace_addr_once(&addr_traced, addr, false, true, t_start);

        /* The last byte goes to the byte-end port: that is what tells the
         * module the message is finished and to emit the T bit and stop
         * clocking, and it is the only difference between the two ports. */
        if ((i + 1u) == len) {
            base->MWDATABE = buf[i];
        } else {
            base->MWDATAB = buf[i];
        }
        /*
         * Weaker than it looks, and deliberately recorded at the weaker point.
         *
         * This says the byte reached the transmit FIFO, not that it was
         * clocked onto the bus - the two differ for exactly the write stall
         * this path was built to diagnose, where the address is ACKed and no
         * data follows. Stamping it after COMPLETE would make the trace agree
         * with the wire in precisely the case worth disagreeing about.
         */
        bb_trace_push(BB_EV_DATA, buf[i], i3c_write_tbit(buf[i]), true);
    }
    s_txlog.stage = (uint8_t)I3C_TX_FED;

    for (guard = 0u; guard < 500000u; guard++) {
        uint32_t err = base->MERRWARN;

        if ((err & (I3C_MERRWARN_NACK_MASK | I3C_MERRWARN_WRABT_MASK)) != 0u) {
            s_txlog.spin_done    = guard;
            s_txlog.merrwarn_end = err;
            return ((err & I3C_MERRWARN_NACK_MASK) != 0u)
                       ? kStatus_I3C_Nak : kStatus_I3C_WriteAbort;
        }
        if ((base->MSTATUS & I3C_MSTATUS_COMPLETE_MASK) != 0u) {
            break;
        }
    }
    s_txlog.spin_done    = guard;
    s_txlog.mstatus_end  = base->MSTATUS;
    s_txlog.merrwarn_end = base->MERRWARN;
    if (guard >= 500000u) {
        return kStatus_I3C_Timeout;
    }
    I3C_MasterClearStatusFlags(base, I3C_MSTATUS_COMPLETE_MASK);
    s_txlog.stage = (uint8_t)I3C_TX_COMPLETE;

    if ((base->MSTATUS & I3C_MSTATUS_STATE_MASK) ==
        (uint32_t)kI3C_MasterStateNormAct) {
        base->MCTRL = (base->MCTRL & ~(I3C_MCTRL_REQUEST_MASK |
                                       I3C_MCTRL_DIR_MASK |
                                       I3C_MCTRL_RDTERM_MASK)) |
                      I3C_MCTRL_REQUEST(kI3C_RequestEmitStop);
        for (guard = 0u; guard < 500000u; guard++) {
            if ((base->MSTATUS & I3C_MSTATUS_MCTRLDONE_MASK) != 0u) {
                break;
            }
        }
        I3C_MasterClearStatusFlags(base, I3C_MSTATUS_MCTRLDONE_MASK);
        /* Inside the request, not after it: this claims a STOP only when one
         * was actually asked for and the module said it was done. */
        trace_frame_close();
    }
    s_txlog.stage = (uint8_t)I3C_TX_STOPPED;

    /* Leave no receive residue for whatever runs next. A write should not be
     * able to put anything in the receive FIFO, and on this part it can. */
    base->MDATACTRL |= I3C_MDATACTRL_FLUSHFB_MASK;

    return kStatus_Success;
}

/*
 * One receive path, parameterised by bus type.
 *
 * `type` is kI3C_TypeI3CSdr for a normal private read and kI3C_TypeI2C for
 * plan v3 section 10's localisation test: the same I3C block, the same FIFO
 * and the same read port, driving a legacy I2C transfer instead of an SDR
 * one. If the duplication follows the block rather than the protocol, it sits
 * below SDR handling, in the shared receive path - which is a different (and
 * much narrower) question to put to NXP than "I3C reads do not work".
 */
static status_t i3c_read_typed(uint8_t addr, uint8_t *buf, uint32_t len,
                               uint32_t type)
{
    I3C_Type *base = pt()->base;
    uint32_t  guard;

    if ((len == 0u) || (len > 255u)) {
        return kStatus_InvalidArgument;
    }

    s_last_addr = addr;

    bool     addr_traced = false;
    uint32_t t_start     = 0u;

    I3C_MasterClearErrorStatusFlags(base, base->MERRWARN);
    I3C_MasterClearStatusFlags(base, base->MSTATUS);

    s_rxlog.n    = 0u;
    s_rxlog.type = (uint8_t)type;
    s_rxlog.pre  = (uint8_t)((base->MDATACTRL & I3C_MDATACTRL_RXCOUNT_MASK)
                             >> I3C_MDATACTRL_RXCOUNT_SHIFT);

    /*
     * Empty the receive FIFO and then check that it is empty.
     *
     * A single FLUSHFB is a request, not a fact, and the difference showed up
     * in a thousand-read soak: 62 reads came back shifted by two, returning
     * `00 40 22 33 44 55` where `00 40` is the payload of the pointer write
     * that preceded them. Residue from the previous frame landed in the FIFO
     * just after the flush and just before the data, so the read took two
     * stale entries first and the real payload came out displaced.
     *
     * Flushing until the count actually reads zero closes that window. If it
     * will not empty, the bus is in a state this transfer cannot trust, and
     * saying so beats returning six bytes that are almost right.
     */
    for (guard = 0u; guard < 1000u; guard++) {
        base->MDATACTRL |= I3C_MDATACTRL_FLUSHFB_MASK;
        __DMB();
        if (((base->MDATACTRL & I3C_MDATACTRL_RXCOUNT_MASK)
                 >> I3C_MDATACTRL_RXCOUNT_SHIFT) == 0u) {
            break;
        }
    }
    if (guard >= 1000u) {
        return kStatus_I3C_Busy;
    }

    /* START, address, read direction, and terminate after len bytes. */
    t_start = bb_cycles();
    base->MCTRL = I3C_MCTRL_REQUEST(kI3C_RequestEmitStartAddr) |
                  I3C_MCTRL_TYPE(type) |
                  I3C_MCTRL_DIR(1u) |
                  I3C_MCTRL_ADDR(addr) |
                  I3C_MCTRL_RDTERM(len);

    for (guard = 0u; guard < 500000u; guard++) {
        uint32_t st = base->MSTATUS;

        if ((base->MERRWARN & I3C_MERRWARN_NACK_MASK) != 0u) {
            trace_addr_once(&addr_traced, addr, true, false, t_start);
            return kStatus_I3C_Nak;
        }
        if ((st & I3C_MSTATUS_MCTRLDONE_MASK) != 0u) {
            break;
        }
    }
    if (guard >= 500000u) {
        return kStatus_I3C_Timeout;
    }
    I3C_MasterClearStatusFlags(base, I3C_MSTATUS_MCTRLDONE_MASK);

    /*
     * Optionally take nothing out of the FIFO until the frame is over.
     *
     * The duplication tracks FIFO occupancy, not rate. At about 1 MHz the CPU
     * waits for each byte, RXCOUNT is 1 at every pop and nothing is duplicated;
     * at 6 MHz the CPU never waits, RXCOUNT climbs to nine, and a six-byte read
     * yields ten entries while the wire carries exactly six. The common factor
     * is that pops and pushes overlap.
     *
     * So: let the transfer finish first, then drain. If the FIFO then holds
     * exactly len correct bytes, the extra entries were caused by reading the
     * port while the receive path was writing it, and the fix is to stop doing
     * that. If it still holds more, the push side duplicates on its own and no
     * amount of software care will help.
     */
    if (s.rx_defer) {
        for (guard = 0u; guard < 2000000u; guard++) {
            if ((base->MERRWARN & I3C_MERRWARN_NACK_MASK) != 0u) {
                trace_addr_once(&addr_traced, addr, true, false, t_start);
                return kStatus_I3C_Nak;
            }
            if ((base->MSTATUS & I3C_MSTATUS_COMPLETE_MASK) != 0u) {
                break;
            }
        }
    }

    for (uint32_t i = 0u; i < len; i++) {
        bool     got  = false;
        bool     pair = false;   /* the half-word port took two bytes */
        uint32_t rxc  = 0u;

        for (guard = 0u; guard < 500000u; guard++) {
            if ((base->MERRWARN & I3C_MERRWARN_NACK_MASK) != 0u) {
                trace_addr_once(&addr_traced, addr, true, false, t_start);
                return kStatus_I3C_Nak;
            }
            /* Read the counter afresh every time round - the whole defect is a
             * value that was true a moment ago. */
            __DMB();
            rxc = (base->MDATACTRL & I3C_MDATACTRL_RXCOUNT_MASK)
                  >> I3C_MDATACTRL_RXCOUNT_SHIFT;
            if (rxc != 0u) {
                got = true;
                break;
            }
        }
        if (s_rxlog.n < I3C_RXLOG_MAX) {
            s_rxlog.cnt[s_rxlog.n]  = (uint8_t)rxc;
            s_rxlog.spin[s_rxlog.n] = guard;
        }
        if (!got) {
            if (s_rxlog.n < I3C_RXLOG_MAX) {
                s_rxlog.val[s_rxlog.n] = 0u;
                s_rxlog.n++;
            }
            trace_hole();
            return kStatus_I3C_Timeout;
        }

        /* An entry is in the FIFO, so the target answered the address: the
         * first point at which that is a fact rather than an assumption. */
        trace_addr_once(&addr_traced, addr, true, true, t_start);

        /*
         * Let the FIFO advance before the next pop.
         *
         * The evidence for this is a read that returned 77 77 77 77 77 88: five
         * copies of the first byte and then the correct next one. The last pop
         * is the one furthest in time from the first, and it is the only one
         * that saw fresh data. Barriers alone did nothing, which fits - they
         * order accesses without spacing them.
         */
        for (uint32_t d = 0u; d < s.rx_gap; d++) {
            __NOP();
        }

        __DMB();
        if (s.rx_wide && ((len - i) >= 2u)) {
            /*
             * The half-word port, as a way of telling a byte-port fault from
             * corruption that happens before the FIFO is read at all. If
             * MRDATAB repeats a byte and MRDATAH does not, the fault is in the
             * byte read port; if both repeat, whatever goes wrong happens
             * before either port sees the data.
             */
            uint32_t h = base->MRDATAH;

            buf[i]     = (uint8_t)(h & 0xFFu);
            buf[i + 1] = (uint8_t)((h >> 8) & 0xFFu);
            i++;
            pair = true;
        } else {
            buf[i] = (uint8_t)(base->MRDATAB & 0xFFu);
        }
        if (s_rxlog.n < I3C_RXLOG_MAX) {
            s_rxlog.val[s_rxlog.n] = buf[i];
            s_rxlog.n++;
        }
        /*
         * The strongest evidence this side produces: the peripheral really did
         * take these bytes off the wire. `i` has already moved to the second
         * of a pair when the half-word port was used, so buf[i - 1] is the
         * first and both are pushed - unlike s_rxlog above, which records only
         * one of each pair.
         *
         * Untested, and said so rather than left to look exercised: rx_wide is
         * false with no command left to set it, so the half-word port is not
         * reachable on this build and this branch has never run. It is written
         * to be right if the port is ever brought back, not because it was
         * measured.
         *
         * The ninth bit is derived, like every ninth bit here. On a read the
         * T-bit is driven by the target and means "more data follows"; the
         * controller consumes it and reports it nowhere. These reads terminate
         * at RDTERM, where the target keeps driving 1 through the last byte,
         * so 1 is what the wire carries and 1 is what is filled in.
         */
        if (pair) {
            bb_trace_push(BB_EV_DATA, buf[i - 1u], true, true);
        }
        bb_trace_push(BB_EV_DATA, buf[i], true, true);
        __DMB();
    }

    /* The read terminates itself at RDTERM; close the frame. */
    for (guard = 0u; guard < 500000u; guard++) {
        if ((base->MSTATUS & I3C_MSTATUS_COMPLETE_MASK) != 0u) {
            break;
        }
    }
    I3C_MasterClearStatusFlags(base, I3C_MSTATUS_COMPLETE_MASK);

    /*
     * What is still in the FIFO now that the frame is over.
     *
     * This is the measurement that reframed the whole defect. A six-byte read
     * that duplicates bytes does not leave an empty FIFO: the per-pop counts
     * climb to nine while six bytes are being taken out, and entries are still
     * there after COMPLETE. The controller is not re-presenting one byte from a
     * stalled read port - it is receiving more entries than there are bytes on
     * the wire, and the duplicates are those extra entries.
     *
     * Draining them here rather than flushing blind is what makes the count
     * meaningful, and it leaves the FIFO genuinely empty for the next transfer.
     */
    s_rxlog.post  = (uint8_t)((base->MDATACTRL & I3C_MDATACTRL_RXCOUNT_MASK)
                              >> I3C_MDATACTRL_RXCOUNT_SHIFT);
    s_rxlog.extra = 0u;
    bool over = false;
    while (((base->MDATACTRL & I3C_MDATACTRL_RXCOUNT_MASK)
                >> I3C_MDATACTRL_RXCOUNT_SHIFT) != 0u) {
        uint8_t b = (uint8_t)(base->MRDATAB & 0xFFu);

        if (s_rxlog.extra < I3C_RXLOG_MAX) {
            s_rxlog.tail[s_rxlog.extra] = b;
        }
        over = true;
        if (++s_rxlog.extra >= 255u) {
            break;
        }
    }

    if ((base->MSTATUS & I3C_MSTATUS_STATE_MASK) ==
        (uint32_t)kI3C_MasterStateNormAct) {
        base->MCTRL = (base->MCTRL & ~(I3C_MCTRL_REQUEST_MASK |
                                       I3C_MCTRL_DIR_MASK |
                                       I3C_MCTRL_RDTERM_MASK)) |
                      I3C_MCTRL_REQUEST(kI3C_RequestEmitStop);
        for (guard = 0u; guard < 500000u; guard++) {
            if ((base->MSTATUS & I3C_MSTATUS_MCTRLDONE_MASK) != 0u) {
                break;
            }
        }
        I3C_MasterClearStatusFlags(base, I3C_MSTATUS_MCTRLDONE_MASK);
        /* Inside the request, not after it: this claims a STOP only when one
         * was actually asked for and the module said it was done. */
        trace_frame_close();
    }

    /*
     * Over-reception is a wrong answer, not a warning.
     *
     * When the frame pushes more entries than the wire carried bytes, the
     * values handed back are a subset of a longer sequence: the transfer looks
     * like a success and the payload is silently wrong. That is the worst
     * failure a bus tool can have, and it is exactly what this defect produced
     * for months - `OK 40 40 11 11 22 22` for a wire carrying 40 11 22 33 44 55.
     *
     * The count is known exactly by the time the frame closes, so say so. A
     * caller that wants to see the damage anyway has I3C:RXLOG?, which still
     * holds every entry.
     */
    if (over) {
        return kStatus_I3C_MsgError;
    }

    return kStatus_Success;
}

static void read_cmd_common(char **argv, uint32_t type, const char *what)
{
    uint32_t addr, count;
    uint8_t  buf[BB_PAYLOAD_MAX];

    if (!bb_parse_u32_max(argv[1], 0x7Fu, &addr)) {
        bb_reply_err(BB_ERR_RANGE, "addr7");
        return;
    }
    if (!bb_parse_u32_max(argv[2], BB_PAYLOAD_MAX, &count) || count == 0u) {
        bb_reply_errf(BB_ERR_RANGE, "count-1..%u", (unsigned)BB_PAYLOAD_MAX);
        return;
    }
    if (!port_ready()) {
        return;
    }
    if (i3c_reply(i3c_read_typed((uint8_t)addr, buf, count, type), what)) {
        bb_reply_ok_bytes(buf, count);
    }
}

static void cmd_read(int argc, char **argv)
{
    (void)argc;
    read_cmd_common(argv, (uint32_t)kI3C_TypeI3CSdr, "read");
}


/* I3C:TXLOG? - the transmit timeline of the last write. */
static void cmd_txlog_q(int argc, char **argv)
{
    static const char *const stage[] = {
        "enter", "started", "fed", "complete", "stopped"
    };

    (void)argc;
    (void)argv;

    bb_io_printf("OK pre=%u n=%u stage=%s spin_start=%lu spin_done=%lu "
                 "mstatus=0x%08lX merrwarn=0x%08lX",
                 (unsigned)s_txlog.pre, (unsigned)s_txlog.n,
                 (s_txlog.stage < 5u) ? stage[s_txlog.stage] : "?",
                 (unsigned long)s_txlog.spin_start,
                 (unsigned long)s_txlog.spin_done,
                 (unsigned long)s_txlog.mstatus_end,
                 (unsigned long)s_txlog.merrwarn_end);
    for (unsigned i = 0; i < s_txlog.n; i++) {
        bb_io_printf(" %u:val=%02X,txcount=%u,spin=%lu", i,
                     (unsigned)s_txlog.val[i], (unsigned)s_txlog.cnt[i],
                     (unsigned long)s_txlog.spin[i]);
    }
    bb_io_puts("\r\n");
}

/* I3C:RXLOG? - the receive timeline of the last read. */
static void cmd_rxlog_q(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    bb_io_printf("OK pre=%u n=%u type=%u post=%u extra=%u",
                 (unsigned)s_rxlog.pre, (unsigned)s_rxlog.n,
                 (unsigned)s_rxlog.type, (unsigned)s_rxlog.post,
                 (unsigned)s_rxlog.extra);
    if (s_rxlog.extra != 0u) {
        unsigned shown = (s_rxlog.extra < I3C_RXLOG_MAX) ? s_rxlog.extra
                                                         : I3C_RXLOG_MAX;
        bb_io_printf(" tail=");
        for (unsigned i = 0; i < shown; i++) {
            bb_io_printf("%s%02X", i ? "," : "", (unsigned)s_rxlog.tail[i]);
        }
    }
    for (unsigned i = 0; i < s_rxlog.n; i++) {
        bb_io_printf(" %u:val=%02X,rxcount=%u,spin=%lu", i,
                     (unsigned)s_rxlog.val[i], (unsigned)s_rxlog.cnt[i],
                     (unsigned long)s_rxlog.spin[i]);
    }
    bb_io_puts("\r\n");
}

static void cmd_probe(int argc, char **argv)
{
    uint32_t addr;
    bool     as_i2c = (argc > 2) && bb_streq_ci(argv[2], "i2c");

    if (!bb_parse_u32_max(argv[1], 0x7Fu, &addr)) {
        bb_reply_err(BB_ERR_RANGE, "addr7");
        return;
    }
    if (!port_ready()) {
        return;
    }
    /* A legacy device on a mixed bus answers I2C framing, not I3C SDR, so the
     * probe has to be told which to speak. Probing an I2C device with SDR
     * framing reports "absent" for a device that is right there. */
    status_t st = xfer((uint8_t)addr, kI3C_Write,
                       as_i2c ? kI3C_TypeI2C : kI3C_TypeI3CSdr,
                       NULL, 0u, kI3C_TransferDefaultFlag);
    if (st == kStatus_Success) {
        bb_reply_okf("1");
    } else if (st == kStatus_I3C_Nak) {
        bb_reply_okf("0");
    } else {
        (void)i3c_reply(st, "probe");
    }
}

/* ---- commands: in-band interrupts -------------------------------------- */

static void cmd_listen(int argc, char **argv)
{
    uint32_t on;

    (void)argc;
    if (!bb_parse_u32_max(argv[1], 1u, &on)) {
        bb_reply_err(BB_ERR_RANGE, "0-or-1");
        return;
    }
    if (!port_ready()) {
        return;
    }
    s.listening = (on != 0u);
    bb_reply_ok();
}

/*
 * Service one in-band interrupt, if a target has raised one.
 *
 * A target that wants attention drives SDA low while the bus is idle; the
 * controller latches that as MSTATUS.SLVSTART. The response is an AutoIbi
 * request, which emits START and the broadcast address and lets the target
 * arbitrate its own address onto the bus. What comes back is the winning
 * address in MSTATUS.IBIADDR, plus a mandatory data byte if the target's BCR
 * says it sends one.
 *
 * Polled from I3C:IBI? rather than from an interrupt. SLVSTART latches and the
 * target holds the line until it is serviced, so nothing is missed by asking
 * late - and a bench tool that only reports what the host asked about is easier
 * to reason about than one with a background handler racing the command loop.
 *
 * @return true when an IBI was serviced, with its address and payload in `out`.
 */
static bool i3c_ibi_service(uint8_t *addr, uint8_t *buf, uint32_t cap_len,
                            uint32_t *got, uint32_t *status_out)
{
    I3C_Type *base = pt()->base;
    uint32_t  st   = base->MSTATUS;
    uint32_t  guard;

    *got        = 0u;
    *addr       = 0u;
    *status_out = st;

    if ((st & I3C_MSTATUS_SLVSTART_MASK) == 0u) {
        return false;
    }

    /*
     * Clear COMPLETE and IBIWON before emitting, not just SLVSTART.
     *
     * Both are left set by whatever transfer ran before this one, and both are
     * what the waits below test. A stale COMPLETE makes the completion wait
     * return before the interrupt has been serviced at all, and then makes the
     * payload wait give up before the bytes arrive - so the reported length
     * depended on what the host happened to do last.
     *
     * That is exactly how it presented: with the same target sending the same
     * payloads, `55 66` came back whole, `22 33` came back as one byte, and a
     * single `AA` came back as none. Nothing about the interrupt varied; the
     * flag left over from the previous command did.
     */
    I3C_MasterClearStatusFlags(base, I3C_MSTATUS_SLVSTART_MASK |
                                     I3C_MSTATUS_COMPLETE_MASK |
                                     I3C_MSTATUS_IBIWON_MASK |
                                     I3C_MSTATUS_MCTRLDONE_MASK);
    I3C_MasterClearErrorStatusFlags(base, base->MERRWARN);
    base->MDATACTRL |= I3C_MDATACTRL_FLUSHFB_MASK;

    /*
     * Acknowledge *with* a mandatory byte.
     *
     * kI3C_IbiRespAck accepts the interrupt and takes no data, which is right
     * only for a target whose BCR says it sends none. This one sets BCR bit 2
     * and sends a payload; answering with the plain ACK left the address
     * correct and `len=0`, and the target's own request still timed out because
     * the data phase it was waiting to drive never happened.
     */
    base->MCTRL = (base->MCTRL & ~(I3C_MCTRL_REQUEST_MASK |
                                   I3C_MCTRL_IBIRESP_MASK)) |
                  I3C_MCTRL_REQUEST(kI3C_RequestAutoIbi) |
                  I3C_MCTRL_IBIRESP(kI3C_IbiRespAckMandatory);

    for (guard = 0u; guard < 500000u; guard++) {
        st = base->MSTATUS;
        if ((st & (I3C_MSTATUS_COMPLETE_MASK |
                   I3C_MSTATUS_IBIWON_MASK)) != 0u) {
            break;
        }
        if ((base->MERRWARN & I3C_MERRWARN_NACK_MASK) != 0u) {
            break;
        }
    }
    *status_out = st;
    *addr = (uint8_t)((st & I3C_MSTATUS_IBIADDR_MASK)
                      >> I3C_MSTATUS_IBIADDR_SHIFT);

    /*
     * Wait for each byte, not just the first.
     *
     * The payload arrives on the bus after the address has won arbitration, so
     * a drain that runs the instant IBIWON is seen finds an empty FIFO. That was
     * fixed by waiting - once, before the loop - and the loop then took whatever
     * had landed and stopped at the first empty read. One byte always had; the
     * second had not yet.
     *
     * The symptom was exact and is what named the bug: every one-byte payload
     * was reported correctly, and every two-byte payload came back as `len=1`
     * holding the first byte. Not a late byte, not a lost one - a byte nobody
     * waited for.
     *
     * The wait is a bounded spin per byte and nothing else. COMPLETE looks like
     * the right terminator and is not: it is set while MSTATUS.STATE still reads
     * IbiRcv, so the frame reports itself finished before the payload has
     * landed. Trusting it turned the fault inside out - two-byte payloads
     * started arriving whole and single-byte ones came back as `len=0`.
     *
     * A byte at 12.5 MHz is nine bits, about 720 ns, which is a handful of
     * register reads. The spin below is generous against that and costs one
     * trailing wait per interrupt when the payload ends - microseconds, paid
     * once, on an event that is rare by construction.
     */
/*
 * MSTATUS.IBITYPE, as the *hardware* encodes it.
 *
 * Not the SDK's i3c_ibi_type_t, which is a driver-level enum with different
 * numbering - kI3C_IbiNormal is 0 there and kI3C_IbiHotJoin is 1, while the
 * register uses 0 for "no IBI" and puts a normal IBI at 1. Comparing the raw
 * field against the driver enum labelled every ordinary in-band interrupt a
 * hot-join; the counter read 108 of them on a bus that had never seen one.
 *
 * From the reference manual's own field description:
 *   0b00 NONE   0b01 IBI   0b10 CR   0b11 HJ
 */
#define I3C_IBITYPE_NONE 0u
#define I3C_IBITYPE_IBI  1u
#define I3C_IBITYPE_CR   2u
#define I3C_IBITYPE_HJ   3u

#define I3C_IBI_BYTE_SPIN 400u

    while (*got < cap_len) {
        bool have = false;

        for (guard = 0u; guard < I3C_IBI_BYTE_SPIN; guard++) {
            if (((base->MDATACTRL & I3C_MDATACTRL_RXCOUNT_MASK)
                     >> I3C_MDATACTRL_RXCOUNT_SHIFT) != 0u) {
                have = true;
                break;
            }
        }
        if (!have) {
            break;                  /* nothing more is coming */
        }
        buf[(*got)++] = (uint8_t)(base->MRDATAB & 0xFFu);
    }

    I3C_MasterClearStatusFlags(base, I3C_MSTATUS_COMPLETE_MASK |
                                     I3C_MSTATUS_IBIWON_MASK);

    /*
     * Close the frame with a STOP, not with i3c_recover().
     *
     * recover() is the failure path: it clears MERRWARN, flushes both FIFOs and
     * force-exits when the controller is not in a normal state. Running it after
     * a *successful* IBI tore down a bus that was fine - every transfer after an
     * interrupt came back no-ack-for-0x09 until the next ENTDAA. An IBI that was
     * serviced correctly ends like any other frame.
     */
    /*
     * A hot-join is closed whatever state it left behind.
     *
     * The NormAct test is right for an ordinary IBI and not sufficient here. A
     * hot-join carries no mandatory byte, but the request has to be issued with
     * IBIRESP = AckMandatory because the type is not knowable until the
     * arbitration header has already gone out - so the controller waits for a
     * byte that never comes, and the frame is still open when this is reached.
     * The state is then not NormAct, no STOP is emitted, and the bus stays
     * busy: the very next ENTDAA - including the one that would have answered
     * the request - fails with `entdaa-busy`, which is exactly how this
     * presented.
     *
     * Emitting the STOP anyway costs nothing when the frame is already closed,
     * and is the difference between answering a hot-join and wedging the bus
     * with one.
     */
    {
        uint32_t ibitype = (*status_out & I3C_MSTATUS_IBITYPE_MASK)
                           >> I3C_MSTATUS_IBITYPE_SHIFT;
        bool     norm;

        /*
         * A hot-join needs its acknowledge spelled out.
         *
         * The request is issued with IBIRESP = AckMandatory, because the type
         * is not knowable until the arbitration header has already gone out -
         * and a hot-join has no mandatory byte to send. The controller is left
         * in kI3C_MasterStateIbiAck, "waiting on IBI ACK/NACK decision", with
         * the decision never made. MSTATUS read 0x...C6 - state 6 - and every
         * following transfer returned `entdaa-busy`, including the ENTDAA that
         * was meant to answer the request.
         *
         * A plain ACK is the decision a hot-join wants: acknowledged, no byte
         * expected. Emitted explicitly here, after the type is finally known,
         * which is the earliest point at which it can be.
         */
        if (ibitype == I3C_IBITYPE_HJ) {
            base->MCTRL = (base->MCTRL & ~(I3C_MCTRL_REQUEST_MASK |
                                           I3C_MCTRL_IBIRESP_MASK)) |
                          I3C_MCTRL_REQUEST(kI3C_RequestIbiAckNack) |
                          I3C_MCTRL_IBIRESP(kI3C_IbiRespAck);
            for (guard = 0u; guard < 500000u; guard++) {
                if ((base->MSTATUS & I3C_MSTATUS_MCTRLDONE_MASK) != 0u) {
                    break;
                }
            }
            I3C_MasterClearStatusFlags(base, I3C_MSTATUS_MCTRLDONE_MASK);
        }

        norm = ((base->MSTATUS & I3C_MSTATUS_STATE_MASK) ==
                (uint32_t)kI3C_MasterStateNormAct);

        if (norm || (ibitype == I3C_IBITYPE_HJ)) {
            base->MCTRL = (base->MCTRL & ~(I3C_MCTRL_REQUEST_MASK |
                                           I3C_MCTRL_DIR_MASK |
                                           I3C_MCTRL_RDTERM_MASK)) |
                          I3C_MCTRL_REQUEST(kI3C_RequestEmitStop);
            for (guard = 0u; guard < 500000u; guard++) {
                if ((base->MSTATUS & I3C_MSTATUS_MCTRLDONE_MASK) != 0u) {
                    break;
                }
            }
            I3C_MasterClearStatusFlags(base, I3C_MSTATUS_MCTRLDONE_MASK);
        }
    }
    return true;
}

/*
 * The last few in-band interrupts, filled from the command loop.
 *
 * Servicing only when the host asks does not work, and the measurement says so:
 * a target that raises an IBI blocks until the controller answers, so with
 * nothing polling it times out and gives up - `ibi-refused hal=3` on the target
 * while the controller, asked afterwards, still found the latched request and
 * read the right address off a bus nobody was driving any more. Both ends
 * behaved correctly and the transaction was still lost.
 *
 * `I3C:LISTEN 1` has to mean the controller is listening. It now does.
 */
#define I3C_IBI_QUEUE 4u


static struct {
    uint8_t  addr;
    uint8_t  len;
    uint8_t  ibitype;    /* MSTATUS.IBITYPE: 0 normal, 1 hot-join, 2 ctrl req */
    uint8_t  data[4];
    uint32_t mstatus;
} s_ibi[I3C_IBI_QUEUE];
static uint8_t  s_ibi_head, s_ibi_tail;
static uint32_t s_ibi_total, s_ibi_lost;
/*
 * Hot-joins seen, counted apart from ordinary in-band interrupts.
 *
 * They arrive down the same path - a target pulls SDA low outside a frame and
 * the controller sees SLVSTART - and MSTATUS.IBITYPE is the only thing that
 * tells them apart. A hot-join carries no mandatory byte and means "I have no
 * dynamic address, give me one", so it needs an ENTDAA rather than a payload
 * read. Counted separately because an IBI total that silently includes them
 * hides the one event a host most wants to act on.
 */
static uint32_t s_hj_total;
/* A hot-join has been acknowledged and is waiting for its address. */
static bool     s_hj_pending;
/* Assign one automatically (I3C:HJAUTO). On by default: a hot-join that is
 * acknowledged and then left without an address leaves the target worse off
 * than if it had been ignored - it has given up its old address and has no new
 * one. */
static bool     s_hj_auto = true;
static status_t s_hj_status;     /* result of the last assignment */
static uint8_t  s_hj_count;      /* targets the bus held after it */
static uint32_t s_hj_done;       /* hot-joins answered with an address */

/*
 * Answer a hot-join by giving the target an address.
 *
 * A hot-join is a target with no dynamic address asking for one, and the
 * acknowledgement alone does not supply it - the controller has to run DAA.
 * Until this existed the controller acknowledged the request and did nothing
 * else, which leaves the target strictly worse off than being ignored: it has
 * already given up whatever address it had and now has none.
 *
 * Run from the poll rather than from where the request is noticed, so the
 * assignment is not attempted inside the path that is still draining the IBI,
 * and quietly, because the poll runs outside any command and must not put a
 * reply on the wire. The outcome is recorded for I3C:HJ? instead.
 */
static void i3c_hj_service(void)
{
    uint8_t  count = 0u;
    status_t st;

    s_hj_pending = false;
    if (!port_ready_quiet()) {
        s_hj_status = kStatus_Fail;
        return;
    }
    st          = i3c_daa_assign(&count);
    s_hj_status = st;
    s_hj_count  = count;
    if (st == kStatus_Success) {
        s_hj_done++;
    }
}

void bb_i3c_poll(void)
{
    uint8_t  addr = 0u, buf[4];
    uint32_t got = 0u, st = 0u;
    uint8_t  next;

    if (!s.listening || !s.configured) {
        return;
    }
    if ((pt()->base->MSTATUS & I3C_MSTATUS_SLVSTART_MASK) == 0u) {
        return;                     /* the common case, and cheap */
    }
    if (!i3c_ibi_service(&addr, buf, sizeof buf, &got, &st)) {
        return;
    }

    s_ibi_total++;
    next = (uint8_t)((s_ibi_head + 1u) % I3C_IBI_QUEUE);
    if (next == s_ibi_tail) {
        s_ibi_lost++;               /* queue full: count it, never overwrite */
        return;
    }
    s_ibi[s_ibi_head].addr    = addr;
    s_ibi[s_ibi_head].len     = (uint8_t)got;
    s_ibi[s_ibi_head].ibitype = (uint8_t)((st & I3C_MSTATUS_IBITYPE_MASK)
                                          >> I3C_MSTATUS_IBITYPE_SHIFT);
    s_ibi[s_ibi_head].mstatus = st;
    if (s_ibi[s_ibi_head].ibitype == I3C_IBITYPE_HJ) {
        s_hj_total++;
        /* A hot-join is a request for an address, and answering it is the
         * controller's job. Deferred to the poll below so the assignment does
         * not run inside the interrupt-shaped path that noticed it. */
        s_hj_pending = true;
    }
    for (uint32_t i = 0; i < got && i < sizeof s_ibi[0].data; i++) {
        s_ibi[s_ibi_head].data[i] = buf[i];
    }
    s_ibi_head = next;

    /* The request has been taken; now answer it. Last, so the event is already
     * queued for the host whether or not the assignment succeeds. */
    if (s_hj_pending && s_hj_auto) {
        i3c_hj_service();
    }
}

/*
 * I3C:HJ? - hot-joins seen, and what was done about them.
 *
 * I3C:HJAUTO 0 stops the controller answering them, which is what a bus
 * observer wants: the request is still counted and reported, but no address is
 * assigned and nothing is put on the wire.
 */
static void cmd_hj_q(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    if (!port_ready()) {
        return;
    }
    bb_i3c_poll();
    bb_reply_okf("seen=%lu answered=%lu auto=%u pending=%u targets=%u "
                 "last=%s",
                 (unsigned long)s_hj_total, (unsigned long)s_hj_done,
                 (unsigned)(s_hj_auto ? 1u : 0u),
                 (unsigned)(s_hj_pending ? 1u : 0u),
                 (unsigned)s_hj_count,
                 (s_hj_done == 0u) ? "none"
                                   : ((s_hj_status == kStatus_Success)
                                      ? "assigned" : "daa-failed"));
}

static void cmd_hj_auto(int argc, char **argv)
{
    uint32_t v;

    if (argc > 1) {
        if (!bb_parse_u32_max(argv[1], 1u, &v)) {
            bb_reply_err(BB_ERR_RANGE, "hjauto-0-or-1");
            return;
        }
        s_hj_auto = (v != 0u);
    }
    bb_reply_okf("hjauto=%u", (unsigned)(s_hj_auto ? 1u : 0u));
}

static void cmd_ibi_q(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    if (!port_ready()) {
        return;
    }
    /*
     * "none" is the protocol word for an empty queue, and bb_i3c_ibi_poll()
     * tests for it before looking for addr= and len=. Anything else lacking
     * those two fields is a malformed reply to the host - which is what
     * "listening=0 n=0" was, however truthful it read.
     */
    if (!s.listening) {
        bb_reply_okf("none");
        return;
    }
    bb_i3c_poll();                  /* catch one that arrived just now */

    if (s_ibi_tail != s_ibi_head) {
        unsigned i = s_ibi_tail;

        /*
         * addr, len and then the payload bytes, with nothing between them.
         *
         * This reply is positional: a reader takes len= and then reads that
         * many bytes from the tokens that follow. A `type=` field inserted
         * between len= and the payload was read as the first payload byte and
         * turned every IBI check from 8/8 into 0/8 - the bus was fine and the
         * parse was not. New fields go after the payload, with the other
         * trailing counters.
         */
        bb_io_printf("OK addr=0x%02X len=%u",
                     (unsigned)s_ibi[i].addr, (unsigned)s_ibi[i].len);
        for (unsigned k = 0; k < s_ibi[i].len; k++) {
            bb_io_printf(" %02X", (unsigned)s_ibi[i].data[k]);
        }
        bb_io_printf(" type=%s mstatus=0x%08lX total=%lu lost=%lu hotjoin=%lu",
                     (s_ibi[i].ibitype == I3C_IBITYPE_HJ) ? "hotjoin"
                     : ((s_ibi[i].ibitype == I3C_IBITYPE_CR) ? "ctrlreq"
                        : ((s_ibi[i].ibitype == I3C_IBITYPE_IBI) ? "ibi"
                           : "none")),
                     (unsigned long)s_ibi[i].mstatus,
                     (unsigned long)s_ibi_total, (unsigned long)s_ibi_lost,
                     (unsigned long)s_hj_total);
        bb_io_puts("\r\n");
        s_ibi_tail = (unsigned char)((s_ibi_tail + 1u) % I3C_IBI_QUEUE);
        return;
    }
    /* IBIs are reported as a count and the last address seen rather than a
     * queue. A queue would need a handler on this port, and the capture engine
     * in Phase 3 is the right place for a timestamped stream of bus events -
     * duplicating it here would give two answers that could disagree. */
    /*
     * No IBI queue exists on this port by design - see the note above - so
     * there is never a pending IBI to report and "none" is the honest answer.
     * The controller status still goes out, as a comment: it is what someone
     * debugging a silent IBI line wants, and a comment cannot confuse the
     * parser.
     */
    uint32_t status = I3C_MasterGetStatusFlags(pt()->base);

    bb_io_printf("# i3c-status=0x%08lX listening=1 slvstart=0 total=%lu "
                 "lost=%lu hotjoin=%lu",
                 (unsigned long)status, (unsigned long)s_ibi_total,
                 (unsigned long)s_ibi_lost, (unsigned long)s_hj_total);
    bb_io_puts("\r\n");
    bb_reply_okf("none");
}

/* ---- commands: role ---------------------------------------------------- */

static void cmd_role(int argc, char **argv)
{
    (void)argc;
    if (bb_streq_ci(argv[1], "controller")) {
        if (!port_ready()) {
            return;
        }
        bb_reply_ok();
        return;
    }
    if (bb_streq_ci(argv[1], "target")) {
        /* Deliberate, and different from the STM32 build. There, one shared
         * file could switch role in either image. Here the target role belongs
         * to the peripheral image, because a controller and a target want the
         * same peripheral and the same pins - one image doing both is how a bus
         * ends up with two things driving SCL. */
        bb_reply_err(BB_ERR_UNSUPPORTED,
                     "target-role-is-the-peripheral-image-build-BB_PROFILE=peripheral");
        return;
    }
    bb_reply_err(BB_ERR_RANGE, "controller-or-target");
}

static void cmd_role_q(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    bb_reply_okf("controller");
}

/* ---- status ------------------------------------------------------------ */

void bb_i3c_status(bb_status_t *out)
{
    if (out == NULL) {
        return;
    }
    memset(out, 0, sizeof *out);
    bb_image_copy_str(out->pins, sizeof out->pins, pt()->pins, strlen(pt()->pins));
    (void)snprintf(out->detail, sizeof out->detail, "p%u %luk %s n=%u",
                   s.port + 1u, (unsigned long)(s.pp_hz / 1000u),
                   s.mixed ? "mix" : "pure", (unsigned)s.target_count);
    out->state = !s.configured ? BB_STATE_ABSENT
                 : (s.target_count > 0u) ? BB_STATE_ACTIVE
                                         : BB_STATE_IDLE;
}

const char *bb_i3c_port_pins(unsigned port)
{
    if (port < 1u || port > BB_I3C_PORT_COUNT) {
        return NULL;
    }
    return s_port[port - 1u].pins;
}

/*
 * HDR-DDR, the high-data-rate mode that clocks on both edges.
 *
 * The controller can emit it: the peripheral's MCTRL.TYPE selects the message
 * type and i3c_write_typed()/i3c_read_typed() already take it, so these are the
 * SDR commands with kI3C_TypeI3CDdr in place of kI3C_TypeI3CSdr.
 *
 * Nothing on this bench can answer one. The NUCLEO's I3C peripheral has no
 * HDR-DDR message type at all - its controller types are RELEASE, HEADER,
 * PRIVATE, DIRECT, LEGACY_I2C and CCC, and the only HDR thing its HAL exposes
 * is an exit pattern for escalating a fault. The target's own GETCAPS returns
 * 0x00, advertising no HDR mode. So a DDR transfer here is expected to go out
 * correctly and be answered by nobody, and these commands exist to drive a
 * target that can, and to let the sniffer show what the controller puts on the
 * wire in the meantime. See docs/STATUS.md 4.6.
 */
static void cmd_ddr_write(int argc, char **argv)
{
    uint32_t addr;
    uint8_t  buf[BB_PAYLOAD_MAX];
    size_t   len;

    if (!bb_parse_u32_max(argv[1], 0x7Fu, &addr)) {
        bb_reply_err(BB_ERR_RANGE, "addr7");
        return;
    }
    if (!bb_parse_bytes(argc, argv, 2, buf, sizeof buf, &len) || len == 0u) {
        bb_reply_errf(BB_ERR_TOO_LONG, "payload-1..%u", (unsigned)BB_PAYLOAD_MAX);
        return;
    }
    if (!port_ready()) {
        return;
    }
    if (i3c_reply(i3c_write_typed((uint8_t)addr, buf, (uint32_t)len,
                                  (uint32_t)kI3C_TypeI3CDdr), "ddrwrite")) {
        bb_reply_ok();
    }
}

static void cmd_ddr_read(int argc, char **argv)
{
    uint32_t addr, count;
    uint8_t  buf[BB_PAYLOAD_MAX];

    (void)argc;
    if (!bb_parse_u32_max(argv[1], 0x7Fu, &addr)) {
        bb_reply_err(BB_ERR_RANGE, "addr7");
        return;
    }
    if (!bb_parse_u32_max(argv[2], BB_PAYLOAD_MAX, &count) || count == 0u) {
        bb_reply_errf(BB_ERR_RANGE, "count-1..%u", (unsigned)BB_PAYLOAD_MAX);
        return;
    }
    if (!port_ready()) {
        return;
    }
    if (i3c_reply(i3c_read_typed((uint8_t)addr, buf, count,
                                 (uint32_t)kI3C_TypeI3CDdr), "ddrread")) {
        bb_reply_ok_bytes(buf, count);
    }
}

/* ---- table ------------------------------------------------------------- */

/*
 * The target-role commands are listed so a host discovers them and gets a
 * definite "wrong image" rather than "unknown command" - which would be
 * indistinguishable from an old firmware. They all land on cmd_role's refusal
 * path through cmd_tgt_unsupported.
 */
static void cmd_tgt_unsupported(int argc, char **argv)
{
    (void)argc;
    bb_reply_errf(BB_ERR_UNSUPPORTED,
                  "%s-is-the-peripheral-image-build-BB_PROFILE=peripheral",
                  argv[0]);
}

const bb_cmd_t bb_cmd_i3c[] = {
    { "I3C:PORT",     1, 1,    cmd_port,       "I3C:PORT <1|2>" },
    { "I3C:PORT?",    0, 0,    cmd_port_q,     "I3C:PORT?" },
    { "I3C:FREQ",     1, 2,    cmd_freq,       "I3C:FREQ <pp-hz> [od-hz]" },
    { "I3C:FREQ?",    0, 0,    cmd_freq_q,     "I3C:FREQ?" },
    { "I3C:CLK?",     0, 0,    cmd_clk_q,      "I3C:CLK?" },
    { "I3C:TRACE?",   0, 0,    cmd_trace_q,    "I3C:TRACE?" },
    { "I3C:DAADBG",   0, 0,    cmd_daadbg,     "I3C:DAADBG" },
    { "I3C:ROOT",     1, 1,    cmd_root,       "I3C:ROOT <hz>" },
    { "I3C:ODHPP",    0, 1,    cmd_odhpp,      "I3C:ODHPP [0|1]" },
    { "I3C:PPLOW",    0, 1,    cmd_pplow,      "I3C:PPLOW [n]" },
    { "I3C:HKEEP",    0, 1,    cmd_hkeep,      "I3C:HKEEP [0..3]" },
    { "I3C:DISTO",    0, 1,    cmd_disto,      "I3C:DISTO [0|1]" },
    { "I3C:ODSTOP",   0, 1,    cmd_odstop,     "I3C:ODSTOP [0|1]" },
    { "I3C:PAD",      0, 1,    cmd_pad,        "I3C:PAD [hex]" },
    { "I3C:BUS",      1, 1,    cmd_bus,        "I3C:BUS <pure|mixed>" },
    { "I3C:ENTDAA",   0, 0,    cmd_entdaa,     "I3C:ENTDAA" },
    { "I3C:RSTDAA",   0, 0,    cmd_rstdaa,     "I3C:RSTDAA" },
    { "I3C:TARGETS?", 0, 0,    cmd_targets_q,  "I3C:TARGETS?" },
    { "I3C:CCC:B",    1, 0xFF, cmd_ccc_b,      "I3C:CCC:B <ccc> [hex...]" },
    { "I3C:CCC:W",    2, 0xFF, cmd_ccc_w,      "I3C:CCC:W <addr> <ccc> [hex...]" },
    { "I3C:CCC:R",    3, 3,    cmd_ccc_r,      "I3C:CCC:R <addr> <ccc> <count>" },
    { "I3C:W",        2, 0xFF, cmd_write,      "I3C:W <addr> <hex...>" },
    { "I3C:R",        2, 2,    cmd_read,       "I3C:R <addr> <count>" },
    { "I3C:DDRW",     2, 0xFF, cmd_ddr_write,  "I3C:DDRW <addr> <hex...>" },
    { "I3C:DDRR",     2, 2,    cmd_ddr_read,   "I3C:DDRR <addr> <count>" },
    { "I3C:RXLOG?",   0, 0,    cmd_rxlog_q,    "I3C:RXLOG?" },
    { "I3C:TXLOG?",   0, 0,    cmd_txlog_q,    "I3C:TXLOG?" },
    { "I3C:PROBE",    1, 2,    cmd_probe,      "I3C:PROBE <addr> [i2c]" },
    { "I3C:LISTEN",   1, 1,    cmd_listen,     "I3C:LISTEN <0|1>" },
    { "I3C:IBI?",     0, 0,    cmd_ibi_q,      "I3C:IBI?" },
    { "I3C:HJ?",      0, 0,    cmd_hj_q,       "I3C:HJ?" },
    { "I3C:HJAUTO",   0, 1,    cmd_hj_auto,    "I3C:HJAUTO [0|1]" },
    { "I3C:ROLE",     1, 1,    cmd_role,       "I3C:ROLE <controller|target>" },
    { "I3C:ROLE?",    0, 0,    cmd_role_q,     "I3C:ROLE?" },
    { "I3C:FREE",     0, 0,    cmd_free,       "I3C:FREE" },
    { "I3C:PULL",     1, 1,    cmd_pull,       "I3C:PULL <0|1>" },
    { "I3C:TGT?",     0, 0,    cmd_tgt_unsupported, "I3C:TGT? (peripheral image)" },
    { "I3C:TGT:ADDR?",0, 0,    cmd_tgt_unsupported, "I3C:TGT:ADDR? (peripheral image)" },
    { "I3C:TGT:ID",   1, 2,    cmd_tgt_unsupported, "I3C:TGT:ID (peripheral image)" },
    { "I3C:TGT:TX",   1, 0xFF, cmd_tgt_unsupported, "I3C:TGT:TX (peripheral image)" },
    { "I3C:TGT:RX?",  0, 1,    cmd_tgt_unsupported, "I3C:TGT:RX? (peripheral image)" },
    { "I3C:TGT:IBI",  0, 0xFF, cmd_tgt_unsupported, "I3C:TGT:IBI (peripheral image)" },
    { "I3C:TGT:HJ",   0, 0,    cmd_tgt_unsupported, "I3C:TGT:HJ (peripheral image)" },
    { NULL, 0, 0, NULL, NULL }
};
