/**
 * @file bb_spi.c
 * @brief LPSPI4 controller — all four modes, 8/16-bit, software chip select.
 *
 * Translated from `_stm32_reference/bb_spi.c`. Command surface unchanged.
 *
 * Chip select is a GPIO driven by this file rather than LPSPI's own PCS, and
 * that is deliberate rather than lazy. `SPI:CS` has to be able to hold CS
 * asserted **across several commands** - which is how you talk to a device that
 * wants a command, a pause while the host decides, and then more clocks on the
 * same selection. A hardware PCS deasserts at the end of every transfer and
 * cannot express that. Manual is the default for the same reason the STM32
 * build chose it.
 *
 * `SPI:CSMODE auto` hands CS back to a per-transfer assert/deassert, for
 * devices that want a fresh frame each time.
 */
#include "bb_board.h"
#include "bb_cmd.h"
#include "bb_image.h"
#include "bb_io.h"
#include "bb_status.h"

#include "fsl_clock.h"
#include "fsl_lpspi.h"

#include <stdio.h>
#include <string.h>

static const bb_pin_t s_sck = BB_SPI_SCK;
static const bb_pin_t s_sdo = BB_SPI_SDO;
static const bb_pin_t s_sdi = BB_SPI_SDI;
static const bb_pin_t s_cs  = BB_SPI_CS;

static struct {
    uint32_t hz;
    uint8_t  mode;        /**< 0..3, the usual CPOL/CPHA pairing */
    uint8_t  width;       /**< 8 or 16 */
    bool     lsb_first;
    bool     cs_manual;   /**< true: SPI:CS owns the line across commands */
    bool     cs_active_low;
    uint8_t  idle_byte;   /**< clocked out while reading */
    bool     configured;
    bool     cs_asserted;
} s = {
    .hz            = 1000000u,
    .mode          = 0u,
    .width         = 8u,
    .lsb_first     = false,
    .cs_manual     = true,
    .cs_active_low = true,
    .idle_byte     = 0xFFu,
};

/* ---- configuration ----------------------------------------------------- */

static void cs_drive(bool assert)
{
    bb_pin_write(&s_cs, s.cs_active_low ? !assert : assert);
    s.cs_asserted = assert;
}

static bool spi_apply(void)
{
    lpspi_master_config_t cfg;

    bb_pin_mux(&s_sck, BB_PAD_PUSHPULL, false);
    bb_pin_mux(&s_sdo, BB_PAD_PUSHPULL, false);
    /* SION on the input so the pad's receiver is on regardless of mux mode -
     * MISO is an input and reads nothing without it. */
    bb_pin_mux(&s_sdi, BB_PAD_INPUT, true);
    bb_pin_gpio(&s_cs, true, true, BB_PAD_PUSHPULL);
    cs_drive(false);

    LPSPI_MasterGetDefaultConfig(&cfg);
    cfg.baudRate     = s.hz;
    cfg.bitsPerFrame = s.width;
    cfg.direction    = s.lsb_first ? kLPSPI_LsbFirst : kLPSPI_MsbFirst;
    /* Mode 0..3 is CPOL in bit 1 and CPHA in bit 0, which is the numbering
     * every host tool and datasheet uses. */
    cfg.cpol = ((s.mode & 2u) != 0u) ? kLPSPI_ClockPolarityActiveLow
                                     : kLPSPI_ClockPolarityActiveHigh;
    cfg.cpha = ((s.mode & 1u) != 0u) ? kLPSPI_ClockPhaseSecondEdge
                                     : kLPSPI_ClockPhaseFirstEdge;
    /* PCS is configured but never used - CS is the GPIO above. Left at PCS0
     * rather than unset so the peripheral has a defined selection if a future
     * change wants hardware CS. */
    cfg.whichPcs           = kLPSPI_Pcs0;
    cfg.pcsToSckDelayInNanoSec    = 1000u;
    cfg.lastSckToPcsDelayInNanoSec = 1000u;
    cfg.betweenTransferDelayInNanoSec = 1000u;

    /* Only release a peripheral that is actually clocked - see the long note in
     * bb_i2c.c's i2c_apply(). A *_Deinit() gates the clock as its last act and
     * resets registers as its first, so deinitialising twice writes to an
     * unclocked peripheral and stalls the AHB with no fault raised. *_Init()
     * ungates and resets on its own, so skipping the deinit is safe. */
    if (s.configured) {
        LPSPI_Deinit(BB_SPI_INSTANCE);
    }
    LPSPI_MasterInit(BB_SPI_INSTANCE, &cfg,
                     CLOCK_GetRootClockFreq(BB_SPI_CLKROOT));
    s.configured = true;
    return true;
}

void bb_spi_init(void)
{
    (void)spi_apply();
}

/* ---- transfers --------------------------------------------------------- */

static bool spi_reply(status_t st)
{
    switch (st) {
    case kStatus_Success:
        return true;
    case kStatus_LPSPI_Timeout:
        bb_reply_err(BB_ERR_TIMEOUT, "spi-timeout");
        return false;
    case kStatus_LPSPI_Busy:
        bb_reply_err(BB_ERR_BUS_STATE, "spi-busy");
        return false;
    case kStatus_LPSPI_Error:
        bb_reply_err(BB_ERR_HAL, "spi-error");
        return false;
    default:
        bb_reply_errf(BB_ERR_HAL, "spi-error-%d", (int)st);
        return false;
    }
}

/**
 * @brief One transfer, with CS handled according to the current mode.
 *
 * In manual mode CS is left exactly as the host set it - asserted or not. That
 * means a transfer issued with CS deasserted still clocks, which is what you
 * want for a bus with no device selected: the reply says what came back rather
 * than refusing on the firmware's guess about intent.
 */
static status_t spi_xfer(const uint8_t *tx, uint8_t *rx, size_t len)
{
    lpspi_transfer_t t;
    bool             took_cs = false;

    if (!s.cs_manual && !s.cs_asserted) {
        cs_drive(true);
        took_cs = true;
    }

    memset(&t, 0, sizeof t);
    t.txData      = tx;
    t.rxData      = rx;
    t.dataSize    = len;
    t.configFlags = (uint32_t)kLPSPI_MasterPcs0 | (uint32_t)kLPSPI_MasterPcsContinuous;

    status_t st = LPSPI_MasterTransferBlocking(BB_SPI_INSTANCE, &t);

    if (took_cs) {
        cs_drive(false);
    }
    return st;
}

/* ---- commands ---------------------------------------------------------- */

static void cmd_cfg(int argc, char **argv)
{
    uint32_t hz, mode, width;

    /* 100 kHz to 30 MHz. The STM32 managed 250 kHz to 32 MHz; the lower bound
     * here is where LPSPI's prescaler runs out at this clock root and the upper
     * is what the pads will do without a controlled-impedance board. Refused
     * rather than clamped. */
    if (!bb_parse_u32(argv[1], &hz) || hz < 100000u || hz > 30000000u) {
        bb_reply_err(BB_ERR_RANGE, "hz-100000-30000000");
        return;
    }
    if (!bb_parse_u32_max(argv[2], 3u, &mode)) {
        bb_reply_err(BB_ERR_RANGE, "mode-0-3");
        return;
    }
    width = 8u;
    if (argc > 3) {
        if (!bb_parse_u32(argv[3], &width) || (width != 8u && width != 16u)) {
            bb_reply_err(BB_ERR_RANGE, "width-8-or-16");
            return;
        }
    }
    bool lsb = false;
    if (argc > 4) {
        if (bb_streq_ci(argv[4], "lsb")) {
            lsb = true;
        } else if (!bb_streq_ci(argv[4], "msb")) {
            bb_reply_err(BB_ERR_RANGE, "msb-or-lsb");
            return;
        }
    }

    s.hz        = hz;
    s.mode      = (uint8_t)mode;
    s.width     = (uint8_t)width;
    s.lsb_first = lsb;
    if (!spi_apply()) {
        bb_reply_err(BB_ERR_HAL, "spi-init-failed");
        return;
    }
    bb_reply_ok();
}

static void cmd_cfg_q(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    bb_reply_okf("hz=%lu mode=%u width=%u order=%s cs=%s cspol=%u idle=%02X",
                 (unsigned long)s.hz, (unsigned)s.mode, (unsigned)s.width,
                 s.lsb_first ? "lsb" : "msb",
                 s.cs_manual ? "manual" : "auto",
                 s.cs_active_low ? 1u : 0u,
                 (unsigned)s.idle_byte);
}

static void cmd_xfer(int argc, char **argv)
{
    uint8_t tx[BB_PAYLOAD_MAX];
    uint8_t rx[BB_PAYLOAD_MAX];
    size_t  len;

    if (!bb_parse_bytes(argc, argv, 1, tx, sizeof tx, &len) || len == 0u) {
        bb_reply_errf(BB_ERR_TOO_LONG, "payload-1..%u", (unsigned)BB_PAYLOAD_MAX);
        return;
    }
    if (spi_reply(spi_xfer(tx, rx, len))) {
        bb_reply_ok_bytes(rx, len);
    }
}

static void cmd_write(int argc, char **argv)
{
    uint8_t tx[BB_PAYLOAD_MAX];
    size_t  len;

    if (!bb_parse_bytes(argc, argv, 1, tx, sizeof tx, &len) || len == 0u) {
        bb_reply_errf(BB_ERR_TOO_LONG, "payload-1..%u", (unsigned)BB_PAYLOAD_MAX);
        return;
    }
    /* rx NULL: LPSPI discards what comes back rather than needing a sink. */
    if (spi_reply(spi_xfer(tx, NULL, len))) {
        bb_reply_ok();
    }
}

static void cmd_read(int argc, char **argv)
{
    uint32_t count;
    uint8_t  tx[BB_PAYLOAD_MAX];
    uint8_t  rx[BB_PAYLOAD_MAX];

    (void)argc;
    if (!bb_parse_u32_max(argv[1], BB_PAYLOAD_MAX, &count) || count == 0u) {
        bb_reply_errf(BB_ERR_RANGE, "count-1..%u", (unsigned)BB_PAYLOAD_MAX);
        return;
    }
    /* SPI has no read without a write - something has to be clocked out. The
     * idle byte is settable because 0x00 and 0xFF mean different things to
     * different devices, and a flash that sees 0x00 where it expected 0xFF has
     * been handed a command. */
    memset(tx, s.idle_byte, count);
    if (spi_reply(spi_xfer(tx, rx, count))) {
        bb_reply_ok_bytes(rx, count);
    }
}

static void cmd_cs(int argc, char **argv)
{
    uint32_t v;

    (void)argc;
    if (!bb_parse_u32_max(argv[1], 1u, &v)) {
        bb_reply_err(BB_ERR_RANGE, "0-or-1");
        return;
    }
    if (!s.cs_manual) {
        bb_reply_err(BB_ERR_BUS_STATE, "cs-is-auto-set-CSMODE-manual");
        return;
    }
    /* 1 means asserted, not "drive high" - the polarity is CSPOL's business.
     * The STM32 build made the same choice and the host library depends on it. */
    cs_drive(v != 0u);
    bb_reply_ok();
}

static void cmd_csmode(int argc, char **argv)
{
    (void)argc;
    if (bb_streq_ci(argv[1], "manual")) {
        s.cs_manual = true;
    } else if (bb_streq_ci(argv[1], "auto")) {
        s.cs_manual = false;
        cs_drive(false);
    } else {
        bb_reply_err(BB_ERR_RANGE, "auto-or-manual");
        return;
    }
    bb_reply_ok();
}

static void cmd_cspol(int argc, char **argv)
{
    uint32_t v;

    (void)argc;
    if (!bb_parse_u32_max(argv[1], 1u, &v)) {
        bb_reply_err(BB_ERR_RANGE, "0-or-1");
        return;
    }
    bool was = s.cs_asserted;
    s.cs_active_low = (v != 0u);
    /* Re-drive so the physical level follows the new polarity immediately
     * rather than at the next CS command - otherwise the line is inverted for
     * an unpredictable window. */
    cs_drive(was);
    bb_reply_ok();
}

static void cmd_idle(int argc, char **argv)
{
    uint32_t v;

    (void)argc;
    if (!bb_parse_u32_max(argv[1], 0xFFu, &v)) {
        bb_reply_err(BB_ERR_RANGE, "byte-0x00-0xFF");
        return;
    }
    s.idle_byte = (uint8_t)v;
    bb_reply_ok();
}

/* ---- status ------------------------------------------------------------ */

void bb_spi_status(bb_status_t *out)
{
    if (out == NULL) {
        return;
    }
    memset(out, 0, sizeof *out);
    bb_image_copy_str(out->pins, sizeof out->pins,
                      BB_SPI_PINS, strlen(BB_SPI_PINS));
    (void)snprintf(out->detail, sizeof out->detail, "%lu Hz m%u %ub",
                   (unsigned long)s.hz, (unsigned)s.mode, (unsigned)s.width);
    out->state = s.configured
                     ? (s.cs_asserted ? BB_STATE_ACTIVE : BB_STATE_IDLE)
                     : BB_STATE_ABSENT;
}

/* ---- table ------------------------------------------------------------- */

const bb_cmd_t bb_cmd_spi[] = {
    { "SPI:CFG",    2, 4,    cmd_cfg,    "SPI:CFG <hz> <mode0-3> [8|16] [msb|lsb]" },
    { "SPI:CFG?",   0, 0,    cmd_cfg_q,  "SPI:CFG?" },
    { "SPI:XFER",   1, 0xFF, cmd_xfer,   "SPI:XFER <hex-bytes...>" },
    { "SPI:W",      1, 0xFF, cmd_write,  "SPI:W <hex-bytes...>" },
    { "SPI:R",      1, 1,    cmd_read,   "SPI:R <count>" },
    { "SPI:CS",     1, 1,    cmd_cs,     "SPI:CS <0|1>" },
    { "SPI:CSMODE", 1, 1,    cmd_csmode, "SPI:CSMODE <auto|manual>" },
    { "SPI:CSPOL",  1, 1,    cmd_cspol,  "SPI:CSPOL <1=active-low|0=active-high>" },
    { "SPI:IDLE",   1, 1,    cmd_idle,   "SPI:IDLE <byte-sent-while-reading>" },
    { NULL, 0, 0, NULL, NULL }
};
