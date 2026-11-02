/**
 * @file bb_spi.c
 * @brief SPI1 controller: full-duplex transfers with software chip select.
 *
 * Chip select is a plain GPIO rather than hardware NSS.  That costs nothing in
 * a command-driven bridge and buys the thing hardware NSS cannot do: hold CS
 * asserted across several commands, which is how you talk to devices that
 * expect a multi-frame transaction (flash page programs, ADC sequences).
 *
 * The kernel clock is CKPER=HSI at 64 MHz (see bb_board.c), so the achievable
 * rates are 64 MHz / 2^n for n = 1..8, i.e. 32 MHz down to 250 kHz.
 */
#include "bb_spi.h"
#include "bb_board.h"
#include "bb_status.h"
#include <stdio.h>
#include "bb_cmd.h"
#include "bb_console.h"

#define SPI_KERNEL_HZ   64000000UL
#define SPI_TIMEOUT_MS       1000u

static SPI_HandleTypeDef hspi;
static uint32_t          want_hz   = 1000000UL;
static uint32_t          actual_hz = 1000000UL;
static uint8_t           mode;                  /* 0..3, CPOL<<1 | CPHA */
static uint8_t           bits      = 8;
static bool              lsb_first;
static bool              cs_active_low = true;
static bool              cs_manual;             /* true: CS held by SPI:CS */
static uint8_t           idle_byte = 0xFF;
static bool              configured;

static const uint32_t presc_table[] = {
    SPI_BAUDRATEPRESCALER_2,   SPI_BAUDRATEPRESCALER_4,
    SPI_BAUDRATEPRESCALER_8,   SPI_BAUDRATEPRESCALER_16,
    SPI_BAUDRATEPRESCALER_32,  SPI_BAUDRATEPRESCALER_64,
    SPI_BAUDRATEPRESCALER_128, SPI_BAUDRATEPRESCALER_256,
};

static void cs_write(bool asserted)
{
    GPIO_PinState level = (asserted == cs_active_low) ? GPIO_PIN_RESET : GPIO_PIN_SET;
    HAL_GPIO_WritePin(BB_SPI_CS_PORT, BB_SPI_CS_PIN, level);
}

/** Pick the fastest prescaler that does not exceed the requested rate. */
static uint32_t choose_prescaler(uint32_t hz, uint32_t *out_hz)
{
    for (unsigned i = 0; i < sizeof presc_table / sizeof presc_table[0]; i++) {
        uint32_t f = SPI_KERNEL_HZ >> (i + 1u);
        if (f <= hz) {
            *out_hz = f;
            return presc_table[i];
        }
    }
    *out_hz = SPI_KERNEL_HZ >> 8;        /* slowest we can go: 250 kHz */
    return SPI_BAUDRATEPRESCALER_256;
}

static bool spi_apply(void)
{
    if (configured) {
        (void)HAL_SPI_DeInit(&hspi);
        configured = false;
    }

    hspi.Instance               = BB_SPI_INSTANCE;
    hspi.Init.Mode              = SPI_MODE_MASTER;
    hspi.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi.Init.DataSize          = (bits == 16u) ? SPI_DATASIZE_16BIT : SPI_DATASIZE_8BIT;
    hspi.Init.CLKPolarity       = (mode & 0x2u) ? SPI_POLARITY_HIGH : SPI_POLARITY_LOW;
    hspi.Init.CLKPhase          = (mode & 0x1u) ? SPI_PHASE_2EDGE : SPI_PHASE_1EDGE;
    hspi.Init.NSS               = SPI_NSS_SOFT;
    hspi.Init.BaudRatePrescaler = choose_prescaler(want_hz, &actual_hz);
    hspi.Init.FirstBit          = lsb_first ? SPI_FIRSTBIT_LSB : SPI_FIRSTBIT_MSB;
    hspi.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    hspi.Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;
    hspi.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
    hspi.Init.FifoThreshold     = SPI_FIFO_THRESHOLD_01DATA;

    if (HAL_SPI_Init(&hspi) != HAL_OK) {
        return false;
    }
    configured = true;
    return true;
}

void bb_spi_init(void)
{
    GPIO_InitTypeDef g = {0};

    BB_SPI_CLK_ENABLE();

    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = BB_SPI_AF;

    g.Pin = BB_SPI_SCK_PIN;
    HAL_GPIO_Init(BB_SPI_SCK_PORT, &g);
    g.Pin = BB_SPI_MOSI_PIN;
    HAL_GPIO_Init(BB_SPI_MOSI_PORT, &g);
    g.Pin  = BB_SPI_MISO_PIN;
    g.Pull = GPIO_PULLUP;               /* defined level when no device drives it */
    HAL_GPIO_Init(BB_SPI_MISO_PORT, &g);

    g.Pin   = BB_SPI_CS_PIN;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(BB_SPI_CS_PORT, &g);
    cs_write(false);

    if (!spi_apply()) {
        bb_error_handler("spi:init");
    }
}

/* ---- commands ---------------------------------------------------------- */

static void cmd_cfg(int argc, char **argv)
{
    uint32_t hz;
    uint32_t m;

    if (!bb_parse_u32(argv[1], &hz) || hz < 100000u || hz > SPI_KERNEL_HZ / 2u) {
        bb_reply_errf(BB_ERR_RANGE, "hz-250k..%lu", (unsigned long)(SPI_KERNEL_HZ / 2u));
        return;
    }
    if (!bb_parse_u32_max(argv[2], 3, &m)) {
        bb_reply_err(BB_ERR_RANGE, "mode-0..3");
        return;
    }
    uint8_t new_bits = bits;
    if (argc > 3) {
        uint32_t b;
        if (!bb_parse_u32(argv[3], &b) || (b != 8u && b != 16u)) {
            bb_reply_err(BB_ERR_RANGE, "bits-8-or-16");
            return;
        }
        new_bits = (uint8_t)b;
    }
    bool new_lsb = lsb_first;
    if (argc > 4) {
        if (bb_streq_ci(argv[4], "MSB")) {
            new_lsb = false;
        } else if (bb_streq_ci(argv[4], "LSB")) {
            new_lsb = true;
        } else {
            bb_reply_err(BB_ERR_RANGE, "msb-or-lsb");
            return;
        }
    }

    want_hz   = hz;
    mode      = (uint8_t)m;
    bits      = new_bits;
    lsb_first = new_lsb;
    if (!spi_apply()) {
        bb_reply_err(BB_ERR_HAL, "spi-init-failed");
        return;
    }
    bb_reply_okf("hz=%lu mode=%u bits=%u first=%s",
                 (unsigned long)actual_hz, mode, bits, lsb_first ? "lsb" : "msb");
}

static void cmd_cfg_q(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    bb_reply_okf("hz=%lu requested=%lu mode=%u bits=%u first=%s cs=%s active=%s idle=0x%02X",
                 (unsigned long)actual_hz, (unsigned long)want_hz, mode, bits,
                 lsb_first ? "lsb" : "msb", cs_manual ? "manual" : "auto",
                 cs_active_low ? "low" : "high", idle_byte);
}

/** Full-duplex transfer: returns exactly as many bytes as were sent. */
static void cmd_xfer(int argc, char **argv)
{
    uint8_t tx[BB_PAYLOAD_MAX];
    uint8_t rx[BB_PAYLOAD_MAX];
    size_t  len;

    if (!bb_parse_bytes(argc, argv, 1, tx, sizeof tx, &len) || len == 0) {
        bb_reply_err(BB_ERR_NUMBER, "bad-hex-payload");
        return;
    }
    if (bits == 16u && (len & 1u) != 0u) {
        bb_reply_err(BB_ERR_RANGE, "16bit-needs-even-length");
        return;
    }

    uint16_t frames = (bits == 16u) ? (uint16_t)(len / 2u) : (uint16_t)len;
    if (!cs_manual) {
        cs_write(true);
    }
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(&hspi, tx, rx, frames, SPI_TIMEOUT_MS);
    if (!cs_manual) {
        cs_write(false);
    }
    if (bb_reply_hal(st, "spi-xfer")) {
        bb_reply_ok_bytes(rx, len);
    }
}

static void cmd_write(int argc, char **argv)
{
    uint8_t tx[BB_PAYLOAD_MAX];
    size_t  len;

    if (!bb_parse_bytes(argc, argv, 1, tx, sizeof tx, &len) || len == 0) {
        bb_reply_err(BB_ERR_NUMBER, "bad-hex-payload");
        return;
    }
    if (bits == 16u && (len & 1u) != 0u) {
        bb_reply_err(BB_ERR_RANGE, "16bit-needs-even-length");
        return;
    }
    uint16_t frames = (bits == 16u) ? (uint16_t)(len / 2u) : (uint16_t)len;
    if (!cs_manual) {
        cs_write(true);
    }
    HAL_StatusTypeDef st = HAL_SPI_Transmit(&hspi, tx, frames, SPI_TIMEOUT_MS);
    if (!cs_manual) {
        cs_write(false);
    }
    if (bb_reply_hal(st, "spi-write")) {
        bb_reply_okf("wrote=%u", (unsigned)len);
    }
}

static void cmd_read(int argc, char **argv)
{
    uint8_t  tx[BB_PAYLOAD_MAX];
    uint8_t  rx[BB_PAYLOAD_MAX];
    uint32_t count;

    (void)argc;
    if (!bb_parse_u32_max(argv[1], BB_PAYLOAD_MAX, &count) || count == 0u) {
        bb_reply_errf(BB_ERR_RANGE, "count-1..%u", (unsigned)BB_PAYLOAD_MAX);
        return;
    }
    if (bits == 16u && (count & 1u) != 0u) {
        bb_reply_err(BB_ERR_RANGE, "16bit-needs-even-length");
        return;
    }
    for (uint32_t i = 0; i < count; i++) {
        tx[i] = idle_byte;
    }
    uint16_t frames = (bits == 16u) ? (uint16_t)(count / 2u) : (uint16_t)count;
    if (!cs_manual) {
        cs_write(true);
    }
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(&hspi, tx, rx, frames, SPI_TIMEOUT_MS);
    if (!cs_manual) {
        cs_write(false);
    }
    if (bb_reply_hal(st, "spi-read")) {
        bb_reply_ok_bytes(rx, count);
    }
}

static void cmd_cs(int argc, char **argv)
{
    uint32_t assert_cs;

    (void)argc;
    if (!bb_parse_u32_max(argv[1], 1, &assert_cs)) {
        bb_reply_err(BB_ERR_RANGE, "0-or-1");
        return;
    }
    /* Touching CS by hand implies the caller wants to own it. */
    cs_manual = true;
    cs_write(assert_cs != 0u);
    bb_reply_ok();
}

static void cmd_csmode(int argc, char **argv)
{
    (void)argc;
    if (bb_streq_ci(argv[1], "AUTO")) {
        cs_manual = false;
        cs_write(false);
    } else if (bb_streq_ci(argv[1], "MANUAL")) {
        cs_manual = true;
    } else {
        bb_reply_err(BB_ERR_RANGE, "auto-or-manual");
        return;
    }
    bb_reply_ok();
}

static void cmd_cspol(int argc, char **argv)
{
    uint32_t active_low;

    (void)argc;
    if (!bb_parse_u32_max(argv[1], 1, &active_low)) {
        bb_reply_err(BB_ERR_RANGE, "0-or-1");
        return;
    }
    cs_active_low = (active_low != 0u);
    cs_write(false);
    bb_reply_ok();
}

static void cmd_idle(int argc, char **argv)
{
    uint32_t byte;

    (void)argc;
    if (!bb_parse_u32_max(argv[1], 0xFFu, &byte)) {
        bb_reply_err(BB_ERR_RANGE, "byte-0..255");
        return;
    }
    idle_byte = (uint8_t)byte;
    bb_reply_ok();
}

void bb_spi_status(bb_status_t *out)
{
    if (out == NULL) {
        return;
    }
    snprintf(out->pins, sizeof out->pins, "D13/D12/D11");
    snprintf(out->detail, sizeof out->detail, "%lukHz",
             (unsigned long)(actual_hz / 1000u));
    out->state = BB_STATE_IDLE;
}

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
