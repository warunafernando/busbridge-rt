/**
 * @file bb_i2c.c
 * @brief Four I2C controllers: addressing, transfers, scan and bus recovery.
 *
 * Every controller's kernel clock is HSI (64 MHz, see bb_board.c), which keeps
 * TIMINGR arithmetic honest and means one timing calculation serves all four.
 * TIMINGR is computed from the requested SCL frequency with the mode-dependent
 * low/high split from the I2C specification; the computation is a good
 * engineering approximation, not a substitute for a scope, so I2C:TIMING lets
 * you force an exact CubeMX-derived value.
 *
 * The buses are genuinely independent - separate peripherals, separate pins,
 * separate state - so the only shared thing is which one the command layer is
 * currently pointed at.  A per-command bus argument was the alternative; a
 * sticky selector won because the overwhelmingly common case is a script that
 * talks to one bus for its whole run.
 */
#include "bb_i2c.h"
#include "bb_i2c_timing.h"
#include "bb_board.h"
#include "bb_status.h"
#include "bb_cmd.h"
#include "bb_console.h"
#include <stdio.h>

#define I2C_TIMEOUT_MS         100u
#define I2C_DEFAULT_HZ      100000UL
#define I2C_ADDR_MAX          0x7Fu

typedef struct {
    const char        *name;
    I2C_TypeDef       *inst;
    bb_pin_t           scl;
    bb_pin_t           sda;
    const char        *pins;            /**< human-readable, for I2C:PORTS? */
    IRQn_Type          ev_irq;
    IRQn_Type          er_irq;

    I2C_HandleTypeDef  h;
    uint32_t           target_hz;
    uint32_t           timingr;
    bool               timingr_forced;
    bool               configured;
    /* A controller the MCU has but this board does not bring out. Reported as
     * unavailable rather than omitted, so the bus numbering is the same on
     * every board and the reason is visible. */
    bool               available;
} i2c_bus_t;

/* Pins, including the alternate function, come from the board header. The AF
 * being per-pin rather than per-family matters: on the Discovery kit I2C4 is
 * AF6 on PB6..PB9 while I2C1 is AF4 on the same pins, and a single shared
 * BB_I2C_AF would configure that bus perfectly and leave it silent.
 *
 * Designated initialisers throughout: the mutable half of the struct is meant
 * to start zeroed, and naming the fields is how you say that to the compiler
 * rather than having it warn about the ones you left out. */
static i2c_bus_t buses[BB_I2C_BUS_COUNT] = {
    { .name = "I2C1", .inst = I2C1,
      .scl = BB_I2C1_SCL, .sda = BB_I2C1_SDA, .pins = BB_I2C1_PINS,
      .ev_irq = I2C1_EV_IRQn, .er_irq = I2C1_ER_IRQn, .available = true },
    { .name = "I2C2", .inst = I2C2,
      .scl = BB_I2C2_SCL, .sda = BB_I2C2_SDA, .pins = BB_I2C2_PINS,
      .ev_irq = I2C2_EV_IRQn, .er_irq = I2C2_ER_IRQn, .available = true },
    { .name = "I2C3", .inst = I2C3,
      .scl = BB_I2C3_SCL, .sda = BB_I2C3_SDA, .pins = BB_I2C3_PINS,
      .ev_irq = I2C3_EV_IRQn, .er_irq = I2C3_ER_IRQn,
      .available = BB_I2C3_AVAILABLE != 0 },
#if BB_I2C_BUS_COUNT >= 4
    { .name = "I2C4", .inst = I2C4,
      .scl = BB_I2C4_SCL, .sda = BB_I2C4_SDA, .pins = BB_I2C4_PINS,
      .ev_irq = I2C4_EV_IRQn, .er_irq = I2C4_ER_IRQn, .available = true },
#endif
};

/** Bus the I2C:* commands act on. Zero-based internally, 1-based on the wire. */
static unsigned active;

static i2c_bus_t *bus(void)
{
    return &buses[active];
}

/** The RCC enables are macros, so the mapping has to be spelled out. */
static void clock_enable(const i2c_bus_t *b, bool on)
{
    if (b->inst == I2C1) {
        if (on) { __HAL_RCC_I2C1_CLK_ENABLE(); } else { __HAL_RCC_I2C1_CLK_DISABLE(); }
    } else if (b->inst == I2C2) {
        if (on) { __HAL_RCC_I2C2_CLK_ENABLE(); } else { __HAL_RCC_I2C2_CLK_DISABLE(); }
    } else if (b->inst == I2C3) {
        if (on) { __HAL_RCC_I2C3_CLK_ENABLE(); } else { __HAL_RCC_I2C3_CLK_DISABLE(); }
#if BB_I2C_BUS_COUNT >= 4
    } else {
        if (on) { __HAL_RCC_I2C4_CLK_ENABLE(); } else { __HAL_RCC_I2C4_CLK_DISABLE(); }
#endif
    }
}

static bool i2c_apply(i2c_bus_t *b)
{
    if (b->configured) {
        (void)HAL_I2C_DeInit(&b->h);
        b->configured = false;
    }

    bb_pins_i2c(&b->scl, &b->sda);
    clock_enable(b, true);

    b->h.Instance              = b->inst;
    b->h.Init.Timing           = b->timingr_forced ? b->timingr
                                                   : bb_i2c_timingr(b->target_hz);
    b->h.Init.OwnAddress1      = 0;
    b->h.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    b->h.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    b->h.Init.OwnAddress2      = 0;
    b->h.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    b->h.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    b->h.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;
    b->timingr                 = b->h.Init.Timing;

    if (HAL_I2C_Init(&b->h) != HAL_OK) {
        return false;
    }
    if (HAL_I2CEx_ConfigAnalogFilter(&b->h, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
        return false;
    }

    /* HAL offers no polling variant of the sequential (repeated-start) API, so
     * the event/error vectors have to be live even though the command layer
     * waits synchronously for completion. */
    HAL_NVIC_SetPriority(b->ev_irq, 6, 0);
    HAL_NVIC_EnableIRQ(b->ev_irq);
    HAL_NVIC_SetPriority(b->er_irq, 6, 0);
    HAL_NVIC_EnableIRQ(b->er_irq);

    b->configured = true;
    return true;
}

void bb_i2c_ev_irq(unsigned n)
{
    if (n < BB_I2C_BUS_COUNT) {
        HAL_I2C_EV_IRQHandler(&buses[n].h);
    }
}

void bb_i2c_er_irq(unsigned n)
{
    if (n < BB_I2C_BUS_COUNT) {
        HAL_I2C_ER_IRQHandler(&buses[n].h);
    }
}

/** Lazily bring the selected bus up; every command funnels through here. */
static i2c_bus_t *i2c_ready(void)
{
    i2c_bus_t *b = bus();

    if (!b->available) {
        bb_reply_errf(BB_ERR_UNSUPPORTED, "%s-not-on-this-board", b->name);
        return NULL;
    }
    if (!b->configured && !i2c_apply(b)) {
        bb_reply_errf(BB_ERR_HAL, "%s-init-failed", b->name);
        return NULL;
    }
    return b;
}

void bb_i2c_init(void)
{
    for (unsigned i = 0; i < BB_I2C_BUS_COUNT; i++) {
        buses[i].target_hz = I2C_DEFAULT_HZ;
        buses[i].timingr   = bb_i2c_timingr(I2C_DEFAULT_HZ);
    }
    /* Peripherals come up on first use: a bus nobody touches costs no clock
     * and leaves its pins as plain inputs, which is what you want for pins on
     * the morpho headers that may be wired to something else. */
}

/* ---- commands ---------------------------------------------------------- */

static bool parse_addr(const char *tok, uint16_t *hal_addr)
{
    uint32_t addr;

    if (!bb_parse_u32_max(tok, I2C_ADDR_MAX, &addr)) {
        bb_reply_err(BB_ERR_RANGE, "addr-must-be-7bit");
        return false;
    }
    *hal_addr = (uint16_t)(addr << 1);
    return true;
}

static void cmd_bus(int argc, char **argv)
{
    uint32_t n;

    (void)argc;
    if (!bb_parse_u32(argv[1], &n) || n < 1u || n > BB_I2C_BUS_COUNT) {
        bb_reply_errf(BB_ERR_RANGE, "bus-1..%u", (unsigned)BB_I2C_BUS_COUNT);
        return;
    }
    if (!buses[n - 1u].available) {
        bb_reply_errf(BB_ERR_UNSUPPORTED, "%s-not-on-this-board-%s",
                      buses[n - 1u].name, buses[n - 1u].pins);
        return;
    }
    active = (unsigned)(n - 1u);
    bb_reply_okf("bus=%u %s %s", (unsigned)n, bus()->name, bus()->pins);
}

/** 1 running, 0 idle, -1 not present on this board. */
static int bus_state(const i2c_bus_t *b)
{
    if (!b->available) {
        return -1;
    }
    return b->configured ? 1 : 0;
}

static void cmd_bus_q(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    bb_reply_okf("bus=%u %s %s hz=%lu up=%d",
                 active + 1u, bus()->name, bus()->pins,
                 (unsigned long)bus()->target_hz, bus()->configured ? 1 : 0);
}

/** Every bus, its pins and its state - the answer to "what can I plug where". */
static void cmd_ports_q(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    /* One line per the protocol, so the buses are separated by semicolons.
     * up is 1 when running, 0 when idle, -1 when the board does not have it.
     *
     * Assembled from the array rather than from four hard-coded entries. The
     * previous version read buses[3] on a board with three of them, and the
     * command simply stopped replying - on the same line that was already
     * reporting the correct count. */
    char line[BB_PAYLOAD_MAX];
    int  len = snprintf(line, sizeof line, "count=%u active=%u",
                        (unsigned)BB_I2C_BUS_COUNT, active + 1u);

    /* Semicolons between buses, one space before the first: the reply is
     * whitespace-tokenised, and the client splits the tail on ';'. */
    for (unsigned i = 0; i < BB_I2C_BUS_COUNT && len > 0 && (size_t)len < sizeof line; i++) {
        len += snprintf(line + len, sizeof line - (size_t)len, "%s%u:%s,%lu,%d",
                        (i == 0u) ? " " : ";", i + 1u, buses[i].pins,
                        (unsigned long)buses[i].target_hz, bus_state(&buses[i]));
    }
    bb_reply_okf("%s", line);
}

static void cmd_freq(int argc, char **argv)
{
    uint32_t   hz;
    i2c_bus_t *b = bus();

    (void)argc;
    if (!bb_parse_u32(argv[1], &hz) || hz < 1000u || hz > 1000000u) {
        bb_reply_err(BB_ERR_RANGE, "freq-1k..1M");
        return;
    }
    b->target_hz      = hz;
    b->timingr_forced = false;
    if (!i2c_apply(b)) {
        bb_reply_errf(BB_ERR_HAL, "%s-init-failed", b->name);
        return;
    }
    bb_reply_okf("hz=%lu timingr=0x%08lX", (unsigned long)b->target_hz,
                 (unsigned long)b->timingr);
}

static void cmd_freq_q(int argc, char **argv)
{
    const i2c_bus_t *b = bus();

    (void)argc;
    (void)argv;
    bb_reply_okf("hz=%lu timingr=0x%08lX forced=%d",
                 (unsigned long)b->target_hz, (unsigned long)b->timingr,
                 b->timingr_forced ? 1 : 0);
}

static void cmd_timing(int argc, char **argv)
{
    uint32_t   raw;
    i2c_bus_t *b = bus();

    (void)argc;
    if (!bb_parse_u32(argv[1], &raw)) {
        bb_reply_err(BB_ERR_NUMBER, "bad-timingr");
        return;
    }
    b->timingr        = raw;
    b->timingr_forced = true;
    if (!i2c_apply(b)) {
        bb_reply_errf(BB_ERR_HAL, "%s-init-failed", b->name);
        return;
    }
    bb_reply_okf("timingr=0x%08lX", (unsigned long)b->timingr);
}

static void cmd_write(int argc, char **argv)
{
    uint8_t    buf[BB_PAYLOAD_MAX];
    size_t     len;
    uint16_t   addr;
    i2c_bus_t *b = i2c_ready();

    if (b == NULL || !parse_addr(argv[1], &addr)) {
        return;
    }
    if (!bb_parse_bytes(argc, argv, 2, buf, sizeof buf, &len) || len == 0) {
        bb_reply_err(BB_ERR_NUMBER, "bad-hex-payload");
        return;
    }
    HAL_StatusTypeDef st = HAL_I2C_Master_Transmit(&b->h, addr, buf, (uint16_t)len,
                                                   I2C_TIMEOUT_MS);
    if (st == HAL_ERROR && (HAL_I2C_GetError(&b->h) & HAL_I2C_ERROR_AF) != 0u) {
        bb_reply_err(BB_ERR_NACK, "nack");
        return;
    }
    if (bb_reply_hal(st, "i2c-write")) {
        bb_reply_okf("wrote=%u", (unsigned)len);
    }
}

static void cmd_read(int argc, char **argv)
{
    uint8_t    buf[BB_PAYLOAD_MAX];
    uint32_t   count;
    uint16_t   addr;
    i2c_bus_t *b = i2c_ready();

    (void)argc;
    if (b == NULL || !parse_addr(argv[1], &addr)) {
        return;
    }
    if (!bb_parse_u32_max(argv[2], BB_PAYLOAD_MAX, &count) || count == 0u) {
        bb_reply_errf(BB_ERR_RANGE, "count-1..%u", (unsigned)BB_PAYLOAD_MAX);
        return;
    }
    HAL_StatusTypeDef st = HAL_I2C_Master_Receive(&b->h, addr, buf, (uint16_t)count,
                                                  I2C_TIMEOUT_MS);
    if (st == HAL_ERROR && (HAL_I2C_GetError(&b->h) & HAL_I2C_ERROR_AF) != 0u) {
        bb_reply_err(BB_ERR_NACK, "nack");
        return;
    }
    if (bb_reply_hal(st, "i2c-read")) {
        bb_reply_ok_bytes(buf, count);
    }
}

/**
 * @brief Wait for a sequential transfer to finish, and clean up if it does not.
 *
 * The interrupt-driven API leaves its state machine mid-transfer when nobody
 * completes it, and a handle that never returns to READY answers HAL_BUSY to
 * everything afterwards. So a timeout here is not just reported: the transfer
 * is aborted, and if that does not restore the handle the peripheral is marked
 * for re-initialisation so the next command starts from a known state.
 *
 * Without this one slow or absent target turns a working bus into a dead one
 * until the board is reset.
 */
static HAL_StatusTypeDef seq_wait(i2c_bus_t *b)
{
    uint32_t start = HAL_GetTick();

    while (HAL_I2C_GetState(&b->h) != HAL_I2C_STATE_READY) {
        if (HAL_GetTick() - start > I2C_TIMEOUT_MS) {
            (void)HAL_I2C_Master_Abort_IT(&b->h, 0);

            /* Give the abort a moment to land before deciding it failed. */
            uint32_t abort_start = HAL_GetTick();
            while (HAL_I2C_GetState(&b->h) != HAL_I2C_STATE_READY) {
                if (HAL_GetTick() - abort_start > I2C_TIMEOUT_MS) {
                    /* Still stuck: force a fresh init on the next command. */
                    b->configured = false;
                    break;
                }
            }
            return HAL_TIMEOUT;
        }
    }
    return HAL_OK;
}

/** Write then read under a repeated start - the usual register-access idiom. */
static void cmd_write_read(int argc, char **argv)
{
    uint8_t    tx[BB_PAYLOAD_MAX];
    uint8_t    rx[BB_PAYLOAD_MAX];
    size_t     tx_len;
    uint32_t   count;
    uint16_t   addr;
    i2c_bus_t *b = i2c_ready();

    if (b == NULL || !parse_addr(argv[1], &addr)) {
        return;
    }
    if (!bb_parse_u32_max(argv[2], BB_PAYLOAD_MAX, &count) || count == 0u) {
        bb_reply_errf(BB_ERR_RANGE, "count-1..%u", (unsigned)BB_PAYLOAD_MAX);
        return;
    }
    if (!bb_parse_bytes(argc, argv, 3, tx, sizeof tx, &tx_len) || tx_len == 0) {
        bb_reply_err(BB_ERR_NUMBER, "bad-hex-payload");
        return;
    }

    /* HAL_I2C_Master_Seq_* keeps the bus owned between the two phases, which
     * is what makes the turnaround a repeated start rather than stop/start. */
    HAL_StatusTypeDef st = HAL_I2C_Master_Seq_Transmit_IT(&b->h, addr, tx, (uint16_t)tx_len,
                                                          I2C_FIRST_FRAME);
    if (st == HAL_OK) {
        st = seq_wait(b);
    }
    if (st == HAL_OK) {
        st = HAL_I2C_Master_Seq_Receive_IT(&b->h, addr, rx, (uint16_t)count, I2C_LAST_FRAME);
        if (st == HAL_OK) {
            st = seq_wait(b);
        }
    }
    if ((HAL_I2C_GetError(&b->h) & HAL_I2C_ERROR_AF) != 0u) {
        /* The acknowledge-failure flag latches. Left set, the next transfer on
         * this bus reports a NACK it did not actually get. */
        b->h.ErrorCode = HAL_I2C_ERROR_NONE;
        bb_reply_err(BB_ERR_NACK, "nack");
        return;
    }
    if (bb_reply_hal(st, "i2c-writeread")) {
        bb_reply_ok_bytes(rx, count);
    }
}

static void cmd_probe(int argc, char **argv)
{
    uint16_t   addr;
    i2c_bus_t *b = i2c_ready();

    (void)argc;
    if (b == NULL || !parse_addr(argv[1], &addr)) {
        return;
    }
    bool present = HAL_I2C_IsDeviceReady(&b->h, addr, 2, I2C_TIMEOUT_MS) == HAL_OK;
    bb_reply_okf("%d", present ? 1 : 0);
}

static void cmd_scan(int argc, char **argv)
{
    i2c_bus_t *b = i2c_ready();

    (void)argc;
    (void)argv;
    if (b == NULL) {
        return;
    }
    /* 0x00 is the general call and 0x78-0x7F are reserved, so sweep 0x08-0x77
     * exactly as every other bus scanner does. */
    uint8_t found[112];
    size_t  n = 0;
    for (uint16_t a = 0x08u; a <= 0x77u; a++) {
        if (HAL_I2C_IsDeviceReady(&b->h, (uint16_t)(a << 1), 1, 5) == HAL_OK) {
            found[n++] = (uint8_t)a;
        }
    }
    bb_reply_ok_bytes(found, n);
}

/** Scan every bus in one command - the fastest way to find where a part is. */
static void cmd_scan_all(int argc, char **argv)
{
    char   out[256];
    size_t used = 0;

    (void)argc;
    (void)argv;
    for (unsigned i = 0; i < BB_I2C_BUS_COUNT; i++) {
        i2c_bus_t *b = &buses[i];

        if (!b->available) {
            used += (size_t)snprintf(out + used, sizeof out - used, "%s%u:absent",
                                     (i > 0u) ? " " : "", i + 1u);
            continue;
        }
        if (!b->configured && !i2c_apply(b)) {
            used += (size_t)snprintf(out + used, sizeof out - used, "%s%u:init-failed",
                                     (i > 0u) ? " " : "", i + 1u);
            continue;
        }
        used += (size_t)snprintf(out + used, sizeof out - used, "%s%u:",
                                 (i > 0u) ? " " : "", i + 1u);
        bool any = false;
        for (uint16_t a = 0x08u; a <= 0x77u; a++) {
            if (HAL_I2C_IsDeviceReady(&b->h, (uint16_t)(a << 1), 1, 2) == HAL_OK) {
                used += (size_t)snprintf(out + used, sizeof out - used, "%s%02X",
                                         any ? "," : "", (unsigned)a);
                any = true;
            }
            if (used + 8u >= sizeof out) {
                break;
            }
        }
        if (!any) {
            used += (size_t)snprintf(out + used, sizeof out - used, "-");
        }
    }
    bb_reply_okf("%s", out);
}

static void cmd_memread(int argc, char **argv)
{
    uint8_t    buf[BB_PAYLOAD_MAX];
    uint32_t   mem_addr;
    uint32_t   addr_size;
    uint32_t   count;
    uint16_t   addr;
    i2c_bus_t *b = i2c_ready();

    (void)argc;
    if (b == NULL || !parse_addr(argv[1], &addr)) {
        return;
    }
    if (!bb_parse_u32(argv[2], &mem_addr) ||
        !bb_parse_u32_max(argv[3], 2, &addr_size) || addr_size == 0u ||
        !bb_parse_u32_max(argv[4], BB_PAYLOAD_MAX, &count) || count == 0u) {
        bb_reply_err(BB_ERR_RANGE, "memaddr-size-1or2-count-1..256");
        return;
    }
    uint16_t size = (addr_size == 1u) ? I2C_MEMADD_SIZE_8BIT : I2C_MEMADD_SIZE_16BIT;
    HAL_StatusTypeDef st = HAL_I2C_Mem_Read(&b->h, addr, (uint16_t)mem_addr, size,
                                            buf, (uint16_t)count, I2C_TIMEOUT_MS);
    if ((HAL_I2C_GetError(&b->h) & HAL_I2C_ERROR_AF) != 0u) {
        bb_reply_err(BB_ERR_NACK, "nack");
        return;
    }
    if (bb_reply_hal(st, "i2c-memread")) {
        bb_reply_ok_bytes(buf, count);
    }
}

static void cmd_memwrite(int argc, char **argv)
{
    uint8_t    buf[BB_PAYLOAD_MAX];
    size_t     len;
    uint32_t   mem_addr;
    uint32_t   addr_size;
    uint16_t   addr;
    i2c_bus_t *b = i2c_ready();

    if (b == NULL || !parse_addr(argv[1], &addr)) {
        return;
    }
    if (!bb_parse_u32(argv[2], &mem_addr) ||
        !bb_parse_u32_max(argv[3], 2, &addr_size) || addr_size == 0u) {
        bb_reply_err(BB_ERR_RANGE, "memaddr-size-1or2");
        return;
    }
    if (!bb_parse_bytes(argc, argv, 4, buf, sizeof buf, &len) || len == 0) {
        bb_reply_err(BB_ERR_NUMBER, "bad-hex-payload");
        return;
    }
    uint16_t size = (addr_size == 1u) ? I2C_MEMADD_SIZE_8BIT : I2C_MEMADD_SIZE_16BIT;
    HAL_StatusTypeDef st = HAL_I2C_Mem_Write(&b->h, addr, (uint16_t)mem_addr, size,
                                             buf, (uint16_t)len, I2C_TIMEOUT_MS);
    if ((HAL_I2C_GetError(&b->h) & HAL_I2C_ERROR_AF) != 0u) {
        bb_reply_err(BB_ERR_NACK, "nack");
        return;
    }
    if (bb_reply_hal(st, "i2c-memwrite")) {
        bb_reply_okf("wrote=%u", (unsigned)len);
    }
}

/**
 * @brief Free a bus where a target is holding SDA low.
 *
 * Nine clock pulses with SDA released, then a STOP: the standard unwedging
 * ritual for a target stuck mid-byte.  Acts on the selected bus only.
 */
static void cmd_recover(int argc, char **argv)
{
    GPIO_InitTypeDef g = {0};
    i2c_bus_t       *b = bus();

    (void)argc;
    (void)argv;
    if (b->configured) {
        (void)HAL_I2C_DeInit(&b->h);
        b->configured = false;
    }

    g.Mode  = GPIO_MODE_OUTPUT_OD;
    g.Pull  = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    g.Pin   = b->scl.pin;
    HAL_GPIO_Init(b->scl.port, &g);
    g.Pin   = b->sda.pin;
    HAL_GPIO_Init(b->sda.port, &g);
    HAL_GPIO_WritePin(b->sda.port, b->sda.pin, GPIO_PIN_SET);

    for (int i = 0; i < 9; i++) {
        HAL_GPIO_WritePin(b->scl.port, b->scl.pin, GPIO_PIN_RESET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(b->scl.port, b->scl.pin, GPIO_PIN_SET);
        HAL_Delay(1);
    }
    /* STOP: SDA low then high while SCL is high. */
    HAL_GPIO_WritePin(b->sda.port, b->sda.pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(b->sda.port, b->sda.pin, GPIO_PIN_SET);

    if (!i2c_apply(b)) {
        bb_reply_errf(BB_ERR_HAL, "%s-init-failed", b->name);
        return;
    }

    /* Clocking the bus frees a target holding SDA. It does nothing for a target
     * whose state machine is lost, so where the board can reset the devices on
     * this bus, do that too - otherwise "recovered" is a claim the command
     * cannot back up. */
    bool reset_devices = bb_board_reset_bus_devices(active + 1u);

    bb_reply_okf("clocked=9 devices_reset=%d", reset_devices ? 1 : 0);
}

void bb_i2c_status(unsigned bus, bb_status_t *out)
{
    if (out == NULL) {
        return;
    }
    if (bus < 1u || bus > BB_I2C_BUS_COUNT) {
        snprintf(out->pins, sizeof out->pins, "-");
        snprintf(out->detail, sizeof out->detail, "no such bus");
        out->state = BB_STATE_ABSENT;
        return;
    }

    const i2c_bus_t *b = &buses[bus - 1u];

    if (!b->available) {
        /* The pin string doubles as the reason on a board that cannot bring a
         * controller out, which is more use on a small screen than "absent". */
        snprintf(out->pins, sizeof out->pins, "%s", b->pins);
        snprintf(out->detail, sizeof out->detail, "-");
        out->state = BB_STATE_ABSENT;
        return;
    }

    /* Pins only, without the SCL=/SDA= labels: the screen is 40 columns and
     * the labels are the least informative part. */
    const char *p = b->pins;
    char        pins[26];
    unsigned    n = 0;
    while (*p != '\0' && n + 1u < sizeof pins) {
        if (*p == '=') {
            p++;
            while (*p != '\0' && *p != ' ' && n + 1u < sizeof pins) {
                pins[n++] = *p++;
            }
            if (*p == ' ') {
                if (n + 1u < sizeof pins) {
                    pins[n++] = '/';
                }
            }
            continue;
        }
        p++;
    }
    pins[n] = '\0';
    snprintf(out->pins, sizeof out->pins, "%s", pins);

    snprintf(out->detail, sizeof out->detail, "%luk",
             (unsigned long)(b->target_hz / 1000u));
    out->state = b->configured ? BB_STATE_ACTIVE : BB_STATE_IDLE;
}

const bb_cmd_t bb_cmd_i2c[] = {
    { "I2C:BUS",    1, 1,    cmd_bus,        "I2C:BUS <1..4>" },
    { "I2C:BUS?",   0, 0,    cmd_bus_q,      "I2C:BUS?" },
    { "I2C:PORTS?", 0, 0,    cmd_ports_q,    "I2C:PORTS?" },
    { "I2C:FREQ",   1, 1,    cmd_freq,       "I2C:FREQ <hz>" },
    { "I2C:FREQ?",  0, 0,    cmd_freq_q,     "I2C:FREQ?" },
    { "I2C:TIMING", 1, 1,    cmd_timing,     "I2C:TIMING <raw-timingr>" },
    { "I2C:W",      2, 0xFF, cmd_write,      "I2C:W <addr7> <hex-bytes...>" },
    { "I2C:R",      2, 2,    cmd_read,       "I2C:R <addr7> <count>" },
    { "I2C:WR",     3, 0xFF, cmd_write_read, "I2C:WR <addr7> <rdcount> <hex-bytes...>" },
    { "I2C:PROBE",  1, 1,    cmd_probe,      "I2C:PROBE <addr7>" },
    { "I2C:SCAN",   0, 0,    cmd_scan,       "I2C:SCAN" },
    { "I2C:SCANALL",0, 0,    cmd_scan_all,   "I2C:SCANALL" },
    { "I2C:MEMR",   4, 4,    cmd_memread,    "I2C:MEMR <addr7> <memaddr> <1|2> <count>" },
    { "I2C:MEMW",   4, 0xFF, cmd_memwrite,   "I2C:MEMW <addr7> <memaddr> <1|2> <hex...>" },
    { "I2C:RECOVER",0, 0,    cmd_recover,    "I2C:RECOVER" },
    { NULL, 0, 0, NULL, NULL }
};
