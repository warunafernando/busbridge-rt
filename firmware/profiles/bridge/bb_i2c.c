/**
 * @file bb_i2c.c
 * @brief LPI2C controllers — three independent buses.
 *
 * Translated from the STM32H5 version in `_stm32_reference/bb_i2c.c`. The
 * command surface is byte-for-byte the same so the existing Python library, C
 * DLL and GUI keep working; what changed underneath is everything.
 *
 * Three notes on how this differs from the STM32 build, all of them properties
 * of the hardware rather than choices:
 *
 * 1. **Three buses, not four.** LPI2C6 reaches no connector on this package
 *    and LPI2C2 reaches only the motor connector. `I2C:PORTS?` reports what is
 *    actually there, including that bus 1 needs jumpers moved, because the
 *    STM32 experience was that a bus which answers "ok" while unwired is worse
 *    than one that refuses.
 *
 * 2. **No TIMINGR to compute.** The STM32 build had `bb_i2c_timing.c` solving
 *    ST's timing fields against the spec's setup and hold limits. LPI2C takes a
 *    baud rate and derives its own dividers, so that whole file is gone and
 *    `I2C:TIMING` now means "write MCCR0 raw" - the nearest equivalent, and
 *    documented as such rather than silently reinterpreted.
 *
 * 3. **Recovery is the driver's, not a hand-rolled clock pulse.** LPI2C detects
 *    a target holding SDA and reports `kStatus_LPI2C_PinLowTimeout`, and a
 *    re-init releases the pins. The STM32 version toggled SCL nine times as a
 *    GPIO to walk a stuck target out of its bit; that is still what
 *    `I2C:RECOVER` does here, because a peripheral reset does not clock a
 *    target that is mid-byte.
 */
#include "bb_board.h"
#include "bb_cmd.h"
#include "bb_io.h"
#include "bb_image.h"
#include "bb_status.h"

#include "fsl_clock.h"
#include "fsl_lpi2c.h"

#include <stdio.h>
#include <string.h>

/* ---- the buses --------------------------------------------------------- */

typedef struct {
    LPI2C_Type        *base;
    bb_pin_t           scl;
    bb_pin_t           sda;
    const char        *pins;
    clock_root_t       root;
    uint32_t           freq_hz;    /**< requested, not measured */
    bool               configured;
    bool               available;  /**< false when the board says the pads are
                                        spoken for - bus 1 without its jumpers */
    const char        *unavailable_why;
} i2c_bus_t;

/* Per-bus frequency was proven independent on the STM32 and the host library
 * relies on it: setting bus 2 to 400 kHz must not disturb bus 3. Each entry
 * carries its own rate for that reason. */
static i2c_bus_t s_bus[BB_I2C_BUS_COUNT] = {
    {
        .base = BB_I2C1_INSTANCE,
        .scl  = BB_I2C1_SCL,
        .sda  = BB_I2C1_SDA,
        .pins = BB_I2C1_PINS,
        .root = kCLOCK_Root_Lpi2c0102,
#if BB_I2C1_ENABLE
        .available = true,
        .unavailable_why = NULL,
#else
        .available = false,
        /* Not a fault. These pads are the MCU-Link console; taking them costs
         * the transport you are most likely reading this over. */
        .unavailable_why = "needs-J41-J43-and-BB_I2C1_ENABLE",
#endif
    },
    {
        .base = BB_I2C2_INSTANCE,
        .scl  = BB_I2C2_SCL,
        .sda  = BB_I2C2_SDA,
        .pins = BB_I2C2_PINS,
        .root = kCLOCK_Root_Lpi2c0102,
        .available = true,
    },
    {
        .base = BB_I2C3_INSTANCE,
        .scl  = BB_I2C3_SCL,
        .sda  = BB_I2C3_SDA,
        .pins = BB_I2C3_PINS,
        .root = kCLOCK_Root_Lpi2c0304,
        .available = true,
    },
};

/* 1-based on the wire, 0-based here. Defaults to bus 3 rather than bus 1,
 * because on this board bus 3 is the Arduino I2C position and bus 1 is off
 * unless its jumpers have been moved. The STM32 default of bus 1 would have
 * meant every fresh board answered "unavailable" to its first transfer. */
static unsigned s_selected = 2u;

#define BB_I2C_DEFAULT_HZ  100000u

static i2c_bus_t *sel(void)
{
    return &s_bus[s_selected];
}

/* ---- configuration ----------------------------------------------------- */

/* Open drain with the weak internal pull-up, and SION set so the peripheral can
 * read back the line it is driving - which an open-drain bus must do to see ACKs
 * and to detect arbitration loss. Without SION the controller drives blind and
 * every transfer NACKs. */
static void i2c_claim_pads(const i2c_bus_t *b)
{
    bb_pin_mux(&b->scl, BB_PAD_I2C, true);
    bb_pin_mux(&b->sda, BB_PAD_I2C, true);
}

static bool i2c_apply(i2c_bus_t *b)
{
    if (!b->available) {
        return false;
    }

    i2c_claim_pads(b);

    lpi2c_master_config_t cfg;
    LPI2C_MasterGetDefaultConfig(&cfg);
    cfg.baudRate_Hz = (b->freq_hz != 0u) ? b->freq_hz : BB_I2C_DEFAULT_HZ;

    /*
     * Deinit only when the peripheral is known to be clocked.
     *
     * LPI2C_MasterDeinit() gates the LPI2C clock as its last act, and
     * LPI2C_MasterReset() - its first act - writes peripheral registers. So
     * calling it on an already-gated instance writes to an unclocked
     * peripheral, which on this part stalls the AHB indefinitely: no fault, no
     * handler, the core simply stops. That is exactly what I2C:RECOVER did,
     * because it deinits the peripheral itself before pulsing the pins and then
     * calls this function to bring it back.
     *
     * It is not needed anyway: LPI2C_MasterInit() ungates the clock and issues
     * its own reset. The guard is here rather than at the call site so every
     * path through this function is safe, including the first configuration of
     * a bus that has never been initialised.
     */
    if (b->configured) {
        LPI2C_MasterDeinit(b->base);
    }
    LPI2C_MasterInit(b->base, &cfg, CLOCK_GetRootClockFreq(b->root));

    b->freq_hz   = cfg.baudRate_Hz;
    b->configured = true;
    return true;
}

void bb_i2c_init(void)
{
    for (unsigned i = 0; i < BB_I2C_BUS_COUNT; i++) {
        s_bus[i].freq_hz    = BB_I2C_DEFAULT_HZ;
        s_bus[i].configured = false;
    }
    /* Only the selected bus is brought up at boot. Configuring all three would
     * claim the motor-connector pads and bus 1's console pads before anyone
     * asked for them. */
    (void)i2c_apply(sel());
}

/* ---- error mapping ----------------------------------------------------- */
/*
 * The wire protocol's ten error codes are the host library's contract, so the
 * mapping matters more than it looks. A NACK is code 5 and must stay code 5:
 * `I2C:PROBE` on an empty bus is a NACK, and the host turns that into "nobody
 * answered" rather than an exception.
 */
/*
 * Leave the controller usable after a failed transfer.
 *
 * The LPI2C halts while a sticky error flag is set: with FEF latched it will not
 * execute another command, so every following transfer fails immediately and
 * reports the *new* transfer as broken rather than the old one. That turns one
 * failure into an unbounded run of them, which is how a single NACK could take
 * the bus down until the board was reset.
 *
 * It showed up as frequency-dependent and it is not. At 100 kHz a NACK left the
 * peripheral clean; at 400 kHz the first transfer NACKed correctly and the next
 * nineteen returned "write-fifo-error" - on both buses, which is what ruled out
 * a pull-up or a marginal bus and pointed here. The suite showed it as slowly
 * accumulating failures, because bb_i2c_config() leaves 400 kHz behind and the
 * next run starts there.
 *
 * Only touched when the peripheral is known to be clocked. Writing LPI2C
 * registers on a gated instance stalls the AHB on this part with no fault and no
 * handler - the same trap i2c_apply() documents.
 */
static void i2c_clear_fault(i2c_bus_t *b)
{
    if (!b->available || !b->configured) {
        return;
    }
    LPI2C_MasterClearStatusFlags(b->base, kLPI2C_MasterClearFlags);
    /* Self-clearing. The SDK resets both FIFOs the same way in its own error
     * path, which is the precedent for doing it here. */
    b->base->MCR |= (LPI2C_MCR_RRF_MASK | LPI2C_MCR_RTF_MASK);
}

static bool i2c_reply(status_t st, const char *what)
{
    if (st != kStatus_Success) {
        /* Every error path, not just the ones that looked like they needed it:
         * the next command is entitled to a controller in a known state
         * whichever way this one failed. */
        i2c_clear_fault(sel());
    }
    switch (st) {
    case kStatus_Success:
        return true;
    case kStatus_LPI2C_Nak:
        bb_reply_errf(BB_ERR_NACK, "%s-nack", what);
        return false;
    case kStatus_LPI2C_Timeout:
        bb_reply_errf(BB_ERR_TIMEOUT, "%s-timeout", what);
        return false;
    case kStatus_LPI2C_PinLowTimeout:
        /* Distinct from a plain timeout on purpose: this one says a line is
         * being held low, which points at a stuck target or a missing pull-up
         * rather than at a target that is merely slow. I2C:RECOVER is the next
         * thing to try, and the message says so by naming the condition. */
        bb_reply_errf(BB_ERR_BUS_STATE, "%s-pin-low-try-recover", what);
        return false;
    case kStatus_LPI2C_ArbitrationLost:
        bb_reply_errf(BB_ERR_BUS_STATE, "%s-arbitration-lost", what);
        return false;
    case kStatus_LPI2C_BitError:
        bb_reply_errf(BB_ERR_BUS_STATE, "%s-bit-error", what);
        return false;
    case kStatus_LPI2C_FifoError:
        bb_reply_errf(BB_ERR_HAL, "%s-fifo-error", what);
        return false;
    case kStatus_LPI2C_Busy:
        bb_reply_errf(BB_ERR_BUS_STATE, "%s-busy", what);
        return false;
    default:
        bb_reply_errf(BB_ERR_HAL, "%s-error-%d", what, (int)st);
        return false;
    }
}

/** Refuse politely if the selected bus has nowhere to go on this board. */
static bool bus_ready(void)
{
    i2c_bus_t *b = sel();

    if (!b->available) {
        bb_reply_errf(BB_ERR_UNSUPPORTED, "i2c%u-unavailable %s",
                      s_selected + 1u, b->unavailable_why);
        return false;
    }
    if (!b->configured && !i2c_apply(b)) {
        bb_reply_errf(BB_ERR_HAL, "i2c%u-init-failed", s_selected + 1u);
        return false;
    }
/*
 * The bus re-claims its own pads before every operation.
 *
 * Symmetry and cheap insurance rather than a fix for an observed conflict, and
 * the distinction is worth stating so nobody trusts this to be load-bearing.
 *
 * No I2C bus on this board shares a pad with a capture tap: the buses are on
 * AON_09/08, AON_16/15 and EMC_B2_19/20, and the taps are on EMC_B1_14..17 and
 * AD_15..17. I originally wrote that sniffing "i2c1's own pads" broke a transfer
 * and fixed it here. That was wrong - AD_* and AON_* are different pads on this
 * part - and the failure I was chasing was the latched FIFO error that
 * i2c_clear_fault() deals with. MDIO is the subsystem that genuinely shares pads
 * with taps, MDC and MDIO being d8 and d9, and bb_mdio.c re-claims for that
 * reason.
 *
 * It stays because a pad map is a board fact and this file should not depend on
 * one: re-muxing costs two register writes against a transaction measured in
 * microseconds, and a future board that does share these pads needs no change
 * here.
 */
    i2c_claim_pads(b);
    return true;
}

/* ---- raw transfers ----------------------------------------------------- */

static status_t xfer(uint8_t addr7, lpi2c_direction_t dir,
                     void *data, size_t len, uint32_t flags)
{
    lpi2c_master_transfer_t t;

    memset(&t, 0, sizeof t);
    t.flags          = flags;
    t.slaveAddress   = addr7;
    t.direction      = dir;
    t.subaddressSize = 0u;
    t.data           = data;
    t.dataSize       = len;

    return LPI2C_MasterTransferBlocking(sel()->base, &t);
}

/*
 * Ten-bit addressing, built by hand.
 *
 * The SDK cannot do it: `lpi2c_master_transfer_t.slaveAddress` is documented as
 * "the 7-bit slave address" and LPI2C_MasterTransferBlocking() shifts it into
 * bits 7:1 with the direction underneath. There is no flag that widens it.
 *
 * On the wire a 10-bit transfer is an ordinary one with a two-byte address: a
 * header byte 11110-A9-A8-RW followed by A7..A0 as the first data byte. A read
 * needs both - the header can only carry a write, because the low byte has to
 * go out before the target knows it is being addressed - so it is written
 * first, then a repeated START re-sends the header with the read bit set.
 *
 * The header's top five bits are the reserved 11110 block, which is why 10-bit
 * addressing costs 7-bit addresses 0x78-0x7B and why nothing else may use them.
 * Passing `0x78 | (addr >> 8)` to LPI2C_MasterStart() as its "7-bit address"
 * produces exactly that byte once the driver shifts it.
 */
#define I2C_ADDR10_HEADER(a)  ((uint8_t)(0x78u | (((a) >> 8) & 0x03u)))

static status_t xfer10_write(uint16_t addr10, const void *data, size_t len)
{
    LPI2C_Type *base = sel()->base;
    uint8_t     low  = (uint8_t)(addr10 & 0xFFu);
    status_t    st;

    st = LPI2C_MasterStart(base, I2C_ADDR10_HEADER(addr10), kLPI2C_Write);
    if (st == kStatus_Success) {
        st = LPI2C_MasterSend(base, &low, 1u);
    }
    if ((st == kStatus_Success) && (len > 0u)) {
        st = LPI2C_MasterSend(base, (void *)data, len);
    }
    /* Always released, even on a failed address phase - a controller that
     * leaves START asserted after a NAK holds the bus against everyone. */
    {
        status_t sp = LPI2C_MasterStop(base);

        if (st == kStatus_Success) {
            st = sp;
        }
    }
    return st;
}

static status_t xfer10_read(uint16_t addr10, void *data, size_t len)
{
    LPI2C_Type *base = sel()->base;
    uint8_t     low  = (uint8_t)(addr10 & 0xFFu);
    status_t    st;

    st = LPI2C_MasterStart(base, I2C_ADDR10_HEADER(addr10), kLPI2C_Write);
    if (st == kStatus_Success) {
        st = LPI2C_MasterSend(base, &low, 1u);
    }
    if (st == kStatus_Success) {
        st = LPI2C_MasterRepeatedStart(base, I2C_ADDR10_HEADER(addr10),
                                       kLPI2C_Read);
    }
    if ((st == kStatus_Success) && (len > 0u)) {
        st = LPI2C_MasterReceive(base, data, len);
    }
    {
        status_t sp = LPI2C_MasterStop(base);

        if (st == kStatus_Success) {
            st = sp;
        }
    }
    return st;
}

/** 10-bit address. The full range is legal; 0 is not a device. */
static bool parse_addr10(const char *tok, uint16_t *out)
{
    uint32_t v;

    if (!bb_parse_u32_max(tok, 0x3FFu, &v) || (v == 0u)) {
        bb_reply_err(BB_ERR_RANGE, "addr10-0x001-0x3FF");
        return false;
    }
    *out = (uint16_t)v;
    return true;
}

/* ---- commands ---------------------------------------------------------- */

/** 7-bit address, refused above 0x7F rather than silently masked. */
static bool parse_addr(const char *tok, uint8_t *out)
{
    uint32_t v;

    if (!bb_parse_u32_max(tok, 0x7Fu, &v)) {
        bb_reply_err(BB_ERR_RANGE, "addr7-0x00-0x7F");
        return false;
    }
    *out = (uint8_t)v;
    return true;
}

static void cmd_bus(int argc, char **argv)
{
    uint32_t n;

    (void)argc;
    if (!bb_parse_u32_max(argv[1], BB_I2C_BUS_COUNT, &n) || n < 1u) {
        bb_reply_errf(BB_ERR_RANGE, "bus-1..%u", (unsigned)BB_I2C_BUS_COUNT);
        return;
    }
    s_selected = (unsigned)(n - 1u);
    if (!s_bus[s_selected].available) {
        /* Selecting it is allowed - a host may want to ask about it - but say
         * plainly that transfers will not work. */
        bb_reply_errf(BB_ERR_UNSUPPORTED, "i2c%u-unavailable %s",
                      (unsigned)n, s_bus[s_selected].unavailable_why);
        return;
    }
    (void)i2c_apply(sel());
    bb_reply_ok();
}

static void cmd_bus_q(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    /* Named field: bb_i2c_bus_get() reads it with bb_field_int(..., "bus").
     * A bare number is a malformed reply however right the number is. */
    bb_reply_okf("bus=%u", s_selected + 1u);
}

static void cmd_ports_q(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    /* Streamed rather than formatted into bb_reply_okf's 256-byte buffer: three
     * buses with pins and reasons is comfortably past it, and the STM32 build
     * learned that the hard way with the pin map. */
    /*
     * The wire format the host library parses, not a readable one.
     *
     *   count=<n> active=<i> <idx>:<pins>,<freq>,<up>;<idx>:...
     *
     * bb_i2c_ports() skips two space-separated tokens, splits the tail on
     * ';', then takes each entry's fields from the *right* - because the pin
     * text itself contains slashes. So a readable variant that put the
     * unavailability reason inside the entry made every port unparseable, and
     * the host reported "no usable I2C bus on this board" while the firmware
     * was correctly describing three of them.
     *
     * up carries the state instead: 1 configured, 0 idle, -1 not fitted. The
     * reason a bus is unavailable is not lost - bus_ready() names it in the
     * error when a host actually tries to use one, which is when it matters.
     * I2C:PORTS? is a machine query; SYS:PINS? is the readable pin view.
     */
    char line[BB_PAYLOAD_MAX];
    int  len = snprintf(line, sizeof line, "count=%u active=%u",
                        (unsigned)BB_I2C_BUS_COUNT, s_selected + 1u);

    for (unsigned i = 0; i < BB_I2C_BUS_COUNT && len > 0 &&
                         (size_t)len < sizeof line; i++) {
        int up = !s_bus[i].available ? -1 : (s_bus[i].configured ? 1 : 0);

        len += snprintf(line + len, sizeof line - (size_t)len,
                        "%s%u:%s,%lu,%d",
                        (i == 0u) ? " " : ";", i + 1u, s_bus[i].pins,
                        (unsigned long)s_bus[i].freq_hz, up);
    }
    bb_reply_okf("%s", line);
}

/*
 * `hz=` is a named field, not a bare number.
 *
 * The host library reads this reply with bb_field_int(payload, "hz", ...) and
 * reports a protocol error when the field is absent, so a bare "OK 100000" is
 * unparseable however correct the number is. Set and query share this so they
 * cannot drift apart.
 *
 * mccr0 comes along because it is the register I2C:TIMING writes, and reading
 * back what a raw write landed is the only way to tell an accepted value from
 * a rejected one.
 */
static void freq_reply(void)
{
    bb_reply_okf("hz=%lu mccr0=0x%08lX",
                 (unsigned long)sel()->freq_hz,
                 (unsigned long)sel()->base->MCCR0);
}

static void cmd_freq(int argc, char **argv)
{
    uint32_t hz;

    (void)argc;
    /* 10 kHz to 1 MHz: below that LPI2C's dividers run out, above it is
     * Fast-mode-plus and needs pull-ups this board does not fit. Refused rather
     * than clamped - a clamped rate that reports success is how you end up
     * measuring the wrong bus speed for an afternoon. */
    if (!bb_parse_u32(argv[1], &hz) || hz < 10000u || hz > 1000000u) {
        bb_reply_err(BB_ERR_RANGE, "freq-10000-1000000");
        return;
    }
    if (!bus_ready()) {
        return;
    }
    sel()->freq_hz = hz;
    if (!i2c_apply(sel())) {
        bb_reply_err(BB_ERR_HAL, "freq-apply-failed");
        return;
    }
    freq_reply();
}

static void cmd_freq_q(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    freq_reply();
}

static void cmd_timing(int argc, char **argv)
{
    uint32_t raw;

    (void)argc;
    if (!bb_parse_u32(argv[1], &raw)) {
        bb_reply_err(BB_ERR_NUMBER, "raw-mccr0");
        return;
    }
    if (!bus_ready()) {
        return;
    }
    /* The STM32's TIMINGR has no counterpart here. MCCR0 is the nearest thing -
     * the clock configuration register holding the four SCL phase counts - so
     * that is what this writes, and the usage string says MCCR0 rather than
     * TIMINGR so nobody feeds it an STM32 value and wonders why. */
    /*
     * Saying "MCCR0, not TIMINGR" in the usage string is not enough, because
     * the failure is unrecoverable. CLKLO and CLKHI are the low and high halves
     * of the SCL period; either at zero means SCL never completes a cycle, so
     * every later transfer reports the bus busy forever and even I2C:RECOVER
     * hangs - it deinitialises a controller stuck mid-transaction. An STM32
     * TIMINGR word fed in here lands zeros in those fields much of the time,
     * which is how this was found: a host test suite passed a TIMINGR value,
     * every I2C command after it failed, and then the device stopped answering
     * at all.
     *
     * A raw register command should still write anything the hardware can hold,
     * so this refuses only the two values that cannot work.
     */
    if (((raw & 0x3Fu) == 0u) || (((raw >> 8) & 0x3Fu) == 0u)) {
        bb_reply_err(BB_ERR_RANGE, "mccr0-clklo-and-clkhi-must-be-nonzero");
        return;
    }
    /* MCCR0 is a configuration register: LPI2C leaves the effect of writing it
     * while the master is enabled undefined. */
    LPI2C_MasterEnable(sel()->base, false);
    sel()->base->MCCR0 = raw;
    LPI2C_MasterEnable(sel()->base, true);
    bb_reply_ok();
}

static void cmd_write(int argc, char **argv)
{
    uint8_t addr;
    uint8_t buf[BB_PAYLOAD_MAX];
    size_t  len;

    if (!parse_addr(argv[1], &addr) || !bus_ready()) {
        return;
    }
    if (!bb_parse_bytes(argc, argv, 2, buf, sizeof buf, &len)) {
        bb_reply_errf(BB_ERR_TOO_LONG, "payload-max-%u", (unsigned)BB_PAYLOAD_MAX);
        return;
    }
    if (i2c_reply(xfer(addr, kLPI2C_Write, buf, len, kLPI2C_TransferDefaultFlag),
                  "write")) {
        bb_reply_ok();
    }
}

static void cmd_read(int argc, char **argv)
{
    uint8_t  addr;
    uint32_t count;
    uint8_t  buf[BB_PAYLOAD_MAX];

    (void)argc;
    if (!parse_addr(argv[1], &addr)) {
        return;
    }
    if (!bb_parse_u32_max(argv[2], BB_PAYLOAD_MAX, &count) || count == 0u) {
        bb_reply_errf(BB_ERR_RANGE, "count-1..%u", (unsigned)BB_PAYLOAD_MAX);
        return;
    }
    if (!bus_ready()) {
        return;
    }
    if (i2c_reply(xfer(addr, kLPI2C_Read, buf, count, kLPI2C_TransferDefaultFlag),
                  "read")) {
        bb_reply_ok_bytes(buf, count);
    }
}

static void cmd_write_read(int argc, char **argv)
{
    uint8_t  addr;
    uint32_t rdcount;
    uint8_t  buf[BB_PAYLOAD_MAX];
    size_t   wlen;

    if (!parse_addr(argv[1], &addr)) {
        return;
    }
    if (!bb_parse_u32_max(argv[2], BB_PAYLOAD_MAX, &rdcount) || rdcount == 0u) {
        bb_reply_errf(BB_ERR_RANGE, "rdcount-1..%u", (unsigned)BB_PAYLOAD_MAX);
        return;
    }
    if (!bb_parse_bytes(argc, argv, 3, buf, sizeof buf, &wlen) || wlen == 0u) {
        bb_reply_err(BB_ERR_ARGC, "need-write-bytes");
        return;
    }
    if (!bus_ready()) {
        return;
    }

    /* Write with no STOP, then read - a repeated START, which is what every
     * register-addressed device expects. Releasing the bus between the two
     * would let another master in and lose the address that was just set. */
    status_t st = xfer(addr, kLPI2C_Write, buf, wlen, kLPI2C_TransferNoStopFlag);
    if (!i2c_reply(st, "write")) {
        /*
         * Only terminate a transaction that actually started.
         *
         * kStatus_LPI2C_Busy comes from the bus-busy check that
         * LPI2C_MasterTransferBlocking() makes *before* sending anything, so
         * there is no transfer to end - and LPI2C_MasterStop() then waits
         * forever for a STOP condition that cannot appear on a bus which is
         * already stuck. Unless I2C_RETRY_TIMES is set its wait loop is a bare
         * `while (result == kStatus_Success)`.
         *
         * That is what killed the device: the error reply went out, the STOP
         * spun, and the firmware never returned to the command loop. From the
         * host it looked like the *next* command hung, which sent the search to
         * entirely the wrong place - I2C:RECOVER, which was innocent.
         */
        /*
         * i2c_reply() has already cleared the fault, so the controller will
         * accept a command again - without that this STOP could not execute at
         * all, and waiting for it to complete was what latched the error.
         *
         * And the STOP is only sent if the bus is actually still held. The
         * write asked for no STOP so that the read could use a repeated start,
         * so on failure something has to release the bus - but a NACK ends the
         * transaction on its own, and issuing a STOP when none is owed queues a
         * command that will never complete.
         */
        if (st != kStatus_LPI2C_Busy &&
            (LPI2C_MasterGetStatusFlags(sel()->base)
             & (uint32_t)kLPI2C_MasterBusBusyFlag) != 0u) {
            (void)LPI2C_MasterStop(sel()->base);
            i2c_clear_fault(sel());
        }
        return;
    }
    if (i2c_reply(xfer(addr, kLPI2C_Read, buf, rdcount,
                       kLPI2C_TransferDefaultFlag), "read")) {
        bb_reply_ok_bytes(buf, rdcount);
    }
}

/** One zero-length write. A NACK here means nothing is at that address. */
static status_t probe_one(uint8_t addr)
{
    return xfer(addr, kLPI2C_Write, NULL, 0u, kLPI2C_TransferDefaultFlag);
}

static void cmd_probe(int argc, char **argv)
{
    uint8_t addr;

    (void)argc;
    if (!parse_addr(argv[1], &addr) || !bus_ready()) {
        return;
    }
    status_t st = probe_one(addr);
    if (st == kStatus_Success) {
        bb_reply_okf("1");
    } else if (st == kStatus_LPI2C_Nak) {
        /* Not an error: "nothing there" is a successful answer to "is anything
         * there". The STM32 build made the same distinction and the host
         * library depends on it. */
        bb_reply_okf("0");
    } else {
        (void)i2c_reply(st, "probe");
    }
}

/** 0x08..0x77: below and above are reserved and probing them is not useful. */
/* The 7-bit addresses a scan sweeps: 0x08..0x77, what is left after the
 * reserved blocks at each end. 112 of them. */
#define SCAN_ADDR_MAX  112u

static size_t scan_collect(uint8_t *out)
{
    size_t n = 0;

    for (uint8_t a = 0x08u; a <= 0x77u; a++) {
        if (probe_one(a) == kStatus_Success) {
            out[n++] = a;
        }
    }
    return n;
}

static void scan_range(unsigned bus)
{
    unsigned found = 0;

    uint8_t addrs[SCAN_ADDR_MAX];
    size_t  count = scan_collect(addrs);

    bb_io_printf("OK i2c%u", bus + 1u);
    for (size_t i = 0; i < count; i++) {
        bb_io_printf(" %02X", (unsigned)addrs[i]);
        found++;
    }
    if (found == 0u) {
        bb_io_puts(" none");
    }
}

static void cmd_scan(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    if (!bus_ready()) {
        return;
    }
    /*
     * A bare list of hex address bytes, not the human-readable form.
     *
     * The host library parses this reply with bb_parse_hex_bytes(), so the
     * "i2c3" prefix and the word "none" that the console form prints make it
     * unparseable - the host reports a malformed reply and the GUI shows no
     * devices with no indication why. I2C:SCANALL keeps the readable form,
     * which is what a person at a terminal wants; both go through
     * scan_collect() so they cannot disagree about what is on the bus.
     */
    uint8_t found[SCAN_ADDR_MAX];
    size_t  count = scan_collect(found);

    bb_reply_ok_bytes(found, count);
}

static void cmd_scan_all(int argc, char **argv)
{
    unsigned keep = s_selected;

    (void)argc;
    (void)argv;
    for (unsigned i = 0; i < BB_I2C_BUS_COUNT; i++) {
        if (!s_bus[i].available) {
            continue;               /* silently skipped: cmd_ports_q says why */
        }
        s_selected = i;
        if (!s_bus[i].configured && !i2c_apply(&s_bus[i])) {
            continue;
        }
        scan_range(i);
        bb_io_puts(" ");
    }
    s_selected = keep;
    bb_io_puts("\r\n");
}

/** memaddr width is 1 or 2 bytes, MSB first, which covers every EEPROM. */
static bool parse_mem(char **argv, uint8_t *addr, uint32_t *memaddr, uint32_t *width)
{
    if (!parse_addr(argv[1], addr)) {
        return false;
    }
    if (!bb_parse_u32(argv[2], memaddr)) {
        bb_reply_err(BB_ERR_NUMBER, "memaddr");
        return false;
    }
    if (!bb_parse_u32_max(argv[3], 2u, width) || *width < 1u) {
        bb_reply_err(BB_ERR_RANGE, "width-1-or-2");
        return false;
    }
    if (*width == 1u && *memaddr > 0xFFu) {
        bb_reply_err(BB_ERR_RANGE, "memaddr-too-big-for-width-1");
        return false;
    }
    return true;
}

static void cmd_memread(int argc, char **argv)
{
    uint8_t  addr;
    uint32_t memaddr, width, count;
    uint8_t  buf[BB_PAYLOAD_MAX];

    (void)argc;
    if (!parse_mem(argv, &addr, &memaddr, &width)) {
        return;
    }
    if (!bb_parse_u32_max(argv[4], BB_PAYLOAD_MAX, &count) || count == 0u) {
        bb_reply_errf(BB_ERR_RANGE, "count-1..%u", (unsigned)BB_PAYLOAD_MAX);
        return;
    }
    if (!bus_ready()) {
        return;
    }

    /* LPI2C carries the sub-address itself, so a register read is one transfer
     * with a hardware repeated START rather than the two the STM32 needed. */
    lpi2c_master_transfer_t t;
    memset(&t, 0, sizeof t);
    t.flags          = kLPI2C_TransferDefaultFlag;
    t.slaveAddress   = addr;
    t.direction      = kLPI2C_Read;
    t.subaddress     = memaddr;
    t.subaddressSize = width;
    t.data           = buf;
    t.dataSize       = count;

    if (i2c_reply(LPI2C_MasterTransferBlocking(sel()->base, &t), "memread")) {
        bb_reply_ok_bytes(buf, count);
    }
}

static void cmd_memwrite(int argc, char **argv)
{
    uint8_t  addr;
    uint32_t memaddr, width;
    uint8_t  buf[BB_PAYLOAD_MAX];
    size_t   len;

    if (!parse_mem(argv, &addr, &memaddr, &width)) {
        return;
    }
    if (!bb_parse_bytes(argc, argv, 4, buf, sizeof buf, &len) || len == 0u) {
        bb_reply_err(BB_ERR_ARGC, "need-data-bytes");
        return;
    }
    if (!bus_ready()) {
        return;
    }

    lpi2c_master_transfer_t t;
    memset(&t, 0, sizeof t);
    t.flags          = kLPI2C_TransferDefaultFlag;
    t.slaveAddress   = addr;
    t.direction      = kLPI2C_Write;
    t.subaddress     = memaddr;
    t.subaddressSize = width;
    t.data           = buf;
    t.dataSize       = len;

    if (i2c_reply(LPI2C_MasterTransferBlocking(sel()->base, &t), "memwrite")) {
        bb_reply_ok();
    }
}

static void cmd_recover(int argc, char **argv)
{
    i2c_bus_t *b = sel();

    (void)argc;
    (void)argv;
    if (!b->available) {
        bb_reply_errf(BB_ERR_UNSUPPORTED, "i2c%u-unavailable %s",
                      s_selected + 1u, b->unavailable_why);
        return;
    }

    /*
     * Nine clock pulses as GPIO, then a STOP.
     *
     * A peripheral re-init is not enough and it is worth being precise about
     * why: a target that was interrupted mid-byte is still holding SDA low and
     * waiting for the clock edges that would finish the byte. Nothing the
     * controller's registers can do produces those edges - only clocking SCL
     * does. So the pins become GPIOs, get pulsed, and are handed back.
     *
     * The board may also have something permanently wired to a bus; where it
     * does, resetting that device is tried first because a target whose state
     * machine is lost cannot be clocked out of it either.
     */
    (void)bb_board_reset_bus_devices(s_selected + 1u);

    /* Same clock-gating trap as in i2c_apply(): only release a peripheral that
     * is actually clocked. Recovering a bus that was never configured is a
     * legitimate thing for a host to ask for. */
    if (b->configured) {
        LPI2C_MasterDeinit(b->base);
    }

    bb_pin_gpio(&b->scl, true, true, BB_PAD_I2C);
    bb_pin_gpio(&b->sda, false, false, BB_PAD_I2C);

    for (unsigned i = 0; i < 9u; i++) {
        bb_pin_write(&b->scl, false);
        for (volatile unsigned d = 0; d < 500u; d++) { }
        bb_pin_write(&b->scl, true);
        for (volatile unsigned d = 0; d < 500u; d++) { }
    }

    /* A STOP: SDA released while SCL is high. */
    bb_pin_gpio(&b->sda, true, false, BB_PAD_I2C);
    bb_pin_write(&b->scl, true);
    for (volatile unsigned d = 0; d < 500u; d++) { }
    bb_pin_write(&b->sda, true);

    b->configured = false;
    if (!i2c_apply(b)) {
        bb_reply_err(BB_ERR_HAL, "recover-reinit-failed");
        return;
    }
    bb_reply_ok();
}

/* ---- status ------------------------------------------------------------ */

void bb_i2c_status(unsigned bus, bb_status_t *out)
{
    if (out == NULL) {
        return;
    }
    memset(out, 0, sizeof *out);

    if (bus < 1u || bus > BB_I2C_BUS_COUNT) {
        out->state = BB_STATE_ABSENT;
        return;
    }
    const i2c_bus_t *b = &s_bus[bus - 1u];

    bb_image_copy_str(out->pins, sizeof out->pins, b->pins, strlen(b->pins));
    if (!b->available) {
        out->state = BB_STATE_ABSENT;
        bb_image_copy_str(out->detail, sizeof out->detail,
                          b->unavailable_why, strlen(b->unavailable_why));
        return;
    }
    (void)snprintf(out->detail, sizeof out->detail, "%lu Hz",
                   (unsigned long)b->freq_hz);
    out->state = b->configured ? BB_STATE_IDLE : BB_STATE_ABSENT;
}

/*
 * Ten-bit addressing lives in its own commands rather than widening I2C:W.
 *
 * The two are different transfers, not different number ranges - a 10-bit
 * address below 0x80 is perfectly legal and would be indistinguishable from a
 * 7-bit one if a single command guessed from the value. An explicit command
 * means a host always knows which it issued and a NAK always means what it
 * says.
 */
static void cmd_write10(int argc, char **argv)
{
    uint16_t addr;
    uint8_t  buf[BB_PAYLOAD_MAX];
    size_t   len;

    if (!parse_addr10(argv[1], &addr) || !bus_ready()) {
        return;
    }
    if (!bb_parse_bytes(argc, argv, 2, buf, sizeof buf, &len)) {
        bb_reply_errf(BB_ERR_TOO_LONG, "payload-max-%u", (unsigned)BB_PAYLOAD_MAX);
        return;
    }
    if (i2c_reply(xfer10_write(addr, buf, len), "write10")) {
        bb_reply_ok();
    }
}

static void cmd_read10(int argc, char **argv)
{
    uint16_t addr;
    uint32_t count;
    uint8_t  buf[BB_PAYLOAD_MAX];

    (void)argc;
    if (!parse_addr10(argv[1], &addr)) {
        return;
    }
    if (!bb_parse_u32_max(argv[2], BB_PAYLOAD_MAX, &count) || (count == 0u)) {
        bb_reply_errf(BB_ERR_RANGE, "count-1..%u", (unsigned)BB_PAYLOAD_MAX);
        return;
    }
    if (!bus_ready()) {
        return;
    }
    if (i2c_reply(xfer10_read(addr, buf, count), "read10")) {
        bb_reply_ok_bytes(buf, count);
    }
}

/*
 * A zero-length write, which is how a probe works in either addressing mode:
 * the address phase is the whole question and the answer is the acknowledge.
 */
static void cmd_probe10(int argc, char **argv)
{
    uint16_t addr;

    (void)argc;
    if (!parse_addr10(argv[1], &addr) || !bus_ready()) {
        return;
    }
    if (xfer10_write(addr, NULL, 0u) == kStatus_Success) {
        bb_reply_okf("addr10=0x%03X present", (unsigned)addr);
    } else {
        bb_reply_okf("addr10=0x%03X absent", (unsigned)addr);
    }
}

/* ---- table ------------------------------------------------------------- */

const bb_cmd_t bb_cmd_i2c[] = {
    { "I2C:BUS",     1, 1,    cmd_bus,        "I2C:BUS <1..3>" },
    { "I2C:BUS?",    0, 0,    cmd_bus_q,      "I2C:BUS?" },
    { "I2C:PORTS?",  0, 0,    cmd_ports_q,    "I2C:PORTS?" },
    { "I2C:FREQ",    1, 1,    cmd_freq,       "I2C:FREQ <hz>" },
    { "I2C:FREQ?",   0, 0,    cmd_freq_q,     "I2C:FREQ?" },

    { "I2C:TIMING",  1, 1,    cmd_timing,     "I2C:TIMING <raw-mccr0>" },
    { "I2C:W",       2, 0xFF, cmd_write,      "I2C:W <addr7> <hex-bytes...>" },
    { "I2C:R",       2, 2,    cmd_read,       "I2C:R <addr7> <count>" },
    { "I2C:WR",      3, 0xFF, cmd_write_read, "I2C:WR <addr7> <rdcount> <hex-bytes...>" },
    { "I2C:PROBE",   1, 1,    cmd_probe,      "I2C:PROBE <addr7>" },
    { "I2C:W10",     2, 0xFF, cmd_write10,    "I2C:W10 <addr10> <hex-bytes...>" },
    { "I2C:R10",     2, 2,    cmd_read10,     "I2C:R10 <addr10> <count>" },
    { "I2C:PROBE10", 1, 1,    cmd_probe10,    "I2C:PROBE10 <addr10>" },
    { "I2C:SCAN",    0, 0,    cmd_scan,       "I2C:SCAN" },
    { "I2C:SCANALL", 0, 0,    cmd_scan_all,   "I2C:SCANALL" },
    { "I2C:MEMR",    4, 4,    cmd_memread,    "I2C:MEMR <addr7> <memaddr> <1|2> <count>" },
    { "I2C:MEMW",    4, 0xFF, cmd_memwrite,   "I2C:MEMW <addr7> <memaddr> <1|2> <hex...>" },
    { "I2C:RECOVER", 0, 0,    cmd_recover,    "I2C:RECOVER" },
    { NULL, 0, 0, NULL, NULL }
};
