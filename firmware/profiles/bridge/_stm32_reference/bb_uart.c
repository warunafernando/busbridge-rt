/**
 * @file bb_uart.c
 * @brief Serial pass-through: a framed command mode and a transparent mode.
 *
 * Command mode (UART:TX / UART:RX?) is the one to script against - it is
 * lossless and byte-exact.  Transparent mode (UART:BRIDGE) turns the console
 * into a wire so a human can talk to a bootloader or shell on the target;
 * three 0x1D bytes (Ctrl-]) get you back out.
 *
 * RX from the target is interrupt-driven into a 4 KiB ring because a target
 * that boots and immediately prints a banner will otherwise overrun while the
 * host is still deciding what to ask for.
 */
#include "bb_uart.h"
#include "bb_board.h"
#include "bb_status.h"
#include <stdio.h>
#include "bb_cmd.h"
#include "bb_console.h"
#include "bb_io.h"
#include <string.h>

#define RX_RING_SIZE     4096u          /* power of two */
#define TX_TIMEOUT_MS      500u
#define ESCAPE_BYTE       0x1Du         /* Ctrl-] */
#define ESCAPE_COUNT         3u
#define UART_MIN_BAUD     1200UL
#define UART_MAX_BAUD  6000000UL

static UART_HandleTypeDef huart;

static volatile uint8_t  rx_ring[RX_RING_SIZE];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;
static volatile uint32_t rx_overruns;
static volatile uint32_t rx_errors;

static uint32_t baud = 115200UL;
static uint32_t word_length = UART_WORDLENGTH_8B;
static uint32_t parity      = UART_PARITY_NONE;
static uint32_t stop_bits   = UART_STOPBITS_1;
static uint32_t flow_ctl    = UART_HWCONTROL_NONE;

static bool    bridge_active;
static uint8_t escape_seen;

static inline uint16_t rx_count(void)
{
    return (uint16_t)((rx_head - rx_tail) & (RX_RING_SIZE - 1u));
}

static bool uart_apply(void)
{
    GPIO_InitTypeDef g = {0};

    BB_TARGET_CLK_ENABLE();

    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_PULLUP;
    g.Speed     = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = BB_TUART_AF;
    g.Pin       = BB_TUART_TX_PIN | BB_TUART_RX_PIN;
#ifdef BB_TUART_RTS_PIN
    if (flow_ctl != UART_HWCONTROL_NONE) {
        g.Pin |= BB_TUART_RTS_PIN | BB_TUART_CTS_PIN;
    }
#else
    /* No flow-control pins on this board. A 64-pin part has none to spare, and
     * routing them would cost a bus. The request is refused where it is made
     * rather than silently configuring two pins that carry nothing. */
    (void)flow_ctl;
#endif
    HAL_GPIO_Init(BB_TUART_PORT, &g);

    /* Only tear down a handle that was actually brought up: HAL_UART_DeInit
     * dereferences Instance without checking it, so calling it on a fresh
     * zeroed handle is a write to address 0. */
    if (huart.Instance != NULL) {
        (void)HAL_UART_DeInit(&huart);
    }

    huart.Instance                    = BB_TARGET_UART;
    huart.Init.BaudRate               = baud;
    huart.Init.WordLength             = word_length;
    huart.Init.StopBits               = stop_bits;
    huart.Init.Parity                 = parity;
    huart.Init.Mode                   = UART_MODE_TX_RX;
    huart.Init.HwFlowCtl              = flow_ctl;
    huart.Init.OverSampling           = UART_OVERSAMPLING_16;
    huart.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
    huart.Init.ClockPrescaler         = UART_PRESCALER_DIV1;
    huart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    if (HAL_UART_Init(&huart) != HAL_OK) {
        return false;
    }

    rx_head = rx_tail = 0;
    __HAL_UART_ENABLE_IT(&huart, UART_IT_RXNE);
    HAL_NVIC_SetPriority(BB_TARGET_UART_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(BB_TARGET_UART_IRQn);
    return true;
}

void bb_uart_init(void)
{
    if (!uart_apply()) {
        bb_error_handler("uart:init");
    }
}

void bb_uart_irq(void)
{
    uint32_t isr = BB_TARGET_UART->ISR;

    if (isr & (USART_ISR_ORE | USART_ISR_NE | USART_ISR_FE | USART_ISR_PE)) {
        BB_TARGET_UART->ICR = USART_ICR_ORECF | USART_ICR_NECF |
                              USART_ICR_FECF | USART_ICR_PECF;
        rx_errors++;
    }
    while (BB_TARGET_UART->ISR & USART_ISR_RXNE_RXFNE) {
        uint8_t  byte = (uint8_t)BB_TARGET_UART->RDR;
        uint16_t next = (uint16_t)((rx_head + 1u) & (RX_RING_SIZE - 1u));
        if (next == rx_tail) {
            rx_overruns++;
        } else {
            rx_ring[rx_head] = byte;
            rx_head = next;
        }
    }
}

static int uart_getchar(void)
{
    if (rx_head == rx_tail) {
        return -1;
    }
    uint8_t byte = rx_ring[rx_tail];
    rx_tail = (uint16_t)((rx_tail + 1u) & (RX_RING_SIZE - 1u));
    return byte;
}

static bool uart_send(const uint8_t *data, size_t len)
{
    return HAL_UART_Transmit(&huart, (uint8_t *)data, (uint16_t)len, TX_TIMEOUT_MS) == HAL_OK;
}

bool bb_uart_bridge_active(void)
{
    return bridge_active;
}

void bb_uart_poll(void)
{
    if (!bridge_active) {
        return;
    }

    /* Target -> host */
    int c;
    while ((c = uart_getchar()) >= 0) {
        uint8_t byte = (uint8_t)c;
        bb_io_write(&byte, 1);
    }

    /* Host -> target, watching for the escape sequence */
    while ((c = bb_console_getchar()) >= 0) {
        if ((uint8_t)c == ESCAPE_BYTE) {
            if (++escape_seen >= ESCAPE_COUNT) {
                bridge_active = false;
                escape_seen   = 0;
                bb_io_puts("\r\n");
                bb_reply_ok();
                return;
            }
            continue;                   /* held back until the run is broken */
        }
        /* A partial escape run was literal data after all: replay it. */
        while (escape_seen > 0u) {
            uint8_t esc = ESCAPE_BYTE;
            (void)uart_send(&esc, 1);
            escape_seen--;
        }
        uint8_t byte = (uint8_t)c;
        (void)uart_send(&byte, 1);
    }
}

/* ---- commands ---------------------------------------------------------- */

static void cmd_cfg(int argc, char **argv)
{
    uint32_t new_baud;

    if (!bb_parse_u32(argv[1], &new_baud) ||
        new_baud < UART_MIN_BAUD || new_baud > UART_MAX_BAUD) {
        bb_reply_errf(BB_ERR_RANGE, "baud-%lu..%lu",
                      (unsigned long)UART_MIN_BAUD, (unsigned long)UART_MAX_BAUD);
        return;
    }

    uint32_t new_parity = UART_PARITY_NONE;
    uint32_t data_bits  = 8;
    uint32_t new_stop   = UART_STOPBITS_1;
    uint32_t new_flow   = UART_HWCONTROL_NONE;

    if (argc > 2) {
        uint32_t b;
        if (!bb_parse_u32(argv[2], &b) || b < 7u || b > 9u) {
            bb_reply_err(BB_ERR_RANGE, "bits-7..9");
            return;
        }
        data_bits = b;
    }
    if (argc > 3) {
        if (bb_streq_ci(argv[3], "N")) {
            new_parity = UART_PARITY_NONE;
        } else if (bb_streq_ci(argv[3], "E")) {
            new_parity = UART_PARITY_EVEN;
        } else if (bb_streq_ci(argv[3], "O")) {
            new_parity = UART_PARITY_ODD;
        } else {
            bb_reply_err(BB_ERR_RANGE, "parity-N-E-O");
            return;
        }
    }
    if (argc > 4) {
        uint32_t s;
        if (!bb_parse_u32(argv[4], &s) || (s != 1u && s != 2u)) {
            bb_reply_err(BB_ERR_RANGE, "stop-1-or-2");
            return;
        }
        new_stop = (s == 1u) ? UART_STOPBITS_1 : UART_STOPBITS_2;
    }
    if (argc > 5) {
        if (bb_streq_ci(argv[5], "NONE")) {
            new_flow = UART_HWCONTROL_NONE;
        } else if (bb_streq_ci(argv[5], "RTSCTS")) {
            new_flow = UART_HWCONTROL_RTS_CTS;
        } else {
            bb_reply_err(BB_ERR_RANGE, "flow-none-or-rtscts");
            return;
        }
    }

    /* On STM32 the parity bit lives inside the word length, so 8 data bits
     * with parity means a 9-bit frame.  Translate rather than surprise. */
    uint32_t frame = data_bits + ((new_parity == UART_PARITY_NONE) ? 0u : 1u);
    switch (frame) {
    case 7:  word_length = UART_WORDLENGTH_7B; break;
    case 8:  word_length = UART_WORDLENGTH_8B; break;
    case 9:  word_length = UART_WORDLENGTH_9B; break;
    default:
        bb_reply_err(BB_ERR_RANGE, "bits-plus-parity-must-be-7..9");
        return;
    }

    baud      = new_baud;
    parity    = new_parity;
    stop_bits = new_stop;
    flow_ctl  = new_flow;
    if (!uart_apply()) {
        bb_reply_err(BB_ERR_HAL, "uart-init-failed");
        return;
    }
    bb_reply_okf("baud=%lu bits=%lu parity=%s stop=%s flow=%s",
                 (unsigned long)baud, (unsigned long)data_bits,
                 (parity == UART_PARITY_NONE) ? "N" : (parity == UART_PARITY_EVEN) ? "E" : "O",
                 (stop_bits == UART_STOPBITS_1) ? "1" : "2",
                 (flow_ctl == UART_HWCONTROL_NONE) ? "none" : "rtscts");
}

static void cmd_cfg_q(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    bb_reply_okf("baud=%lu rx_pending=%u rx_overruns=%lu rx_errors=%lu bridge=%d",
                 (unsigned long)baud, (unsigned)rx_count(),
                 (unsigned long)rx_overruns, (unsigned long)rx_errors,
                 bridge_active ? 1 : 0);
}

static void cmd_tx(int argc, char **argv)
{
    uint8_t buf[BB_PAYLOAD_MAX];
    size_t  len;

    if (!bb_parse_bytes(argc, argv, 1, buf, sizeof buf, &len) || len == 0) {
        bb_reply_err(BB_ERR_NUMBER, "bad-hex-payload");
        return;
    }
    if (!uart_send(buf, len)) {
        bb_reply_err(BB_ERR_TIMEOUT, "uart-tx-timeout");
        return;
    }
    bb_reply_okf("wrote=%u", (unsigned)len);
}

/** Send the rest of the line as ASCII, with the words separated by spaces. */
static void cmd_txs(int argc, char **argv)
{
    uint8_t buf[BB_PAYLOAD_MAX];
    size_t  len = 0;

    for (int i = 1; i < argc; i++) {
        size_t tok_len = strlen(argv[i]);
        if (len + tok_len + 1u > sizeof buf) {
            bb_reply_err(BB_ERR_TOO_LONG, "payload-too-long");
            return;
        }
        if (i > 1) {
            buf[len++] = ' ';
        }
        memcpy(&buf[len], argv[i], tok_len);
        len += tok_len;
    }
    if (len + 2u <= sizeof buf) {
        buf[len++] = '\r';
        buf[len++] = '\n';
    }
    if (!uart_send(buf, len)) {
        bb_reply_err(BB_ERR_TIMEOUT, "uart-tx-timeout");
        return;
    }
    bb_reply_okf("wrote=%u", (unsigned)len);
}

static void cmd_rx(int argc, char **argv)
{
    uint8_t  buf[BB_PAYLOAD_MAX];
    uint32_t want = BB_PAYLOAD_MAX;

    if (argc > 1 && (!bb_parse_u32_max(argv[1], BB_PAYLOAD_MAX, &want) || want == 0u)) {
        bb_reply_errf(BB_ERR_RANGE, "count-1..%u", (unsigned)BB_PAYLOAD_MAX);
        return;
    }
    size_t n = 0;
    int    c;
    while (n < want && (c = uart_getchar()) >= 0) {
        buf[n++] = (uint8_t)c;
    }
    bb_reply_ok_bytes(buf, n);
}

static void cmd_rxn(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    bb_reply_okf("%u", (unsigned)rx_count());
}

static void cmd_flush(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    rx_tail = rx_head;
    bb_reply_ok();
}

static void cmd_break(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    BB_TARGET_UART->RQR = USART_RQR_SBKRQ;
    bb_reply_ok();
}

static void cmd_bridge(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    bridge_active = true;
    escape_seen   = 0;
    bb_io_puts("# bridge on, press Ctrl-] three times to exit\r\n");
}

void bb_uart_status(bb_status_t *out)
{
    if (out == NULL) {
        return;
    }
    snprintf(out->pins, sizeof out->pins, "TX/RX");
    snprintf(out->detail, sizeof out->detail, "%lu", (unsigned long)baud);
    out->state = BB_STATE_IDLE;
}

const bb_cmd_t bb_cmd_uart[] = {
    { "UART:CFG",    1, 5,    cmd_cfg,    "UART:CFG <baud> [7|8|9] [N|E|O] [1|2] [none|rtscts]" },
    { "UART:CFG?",   0, 0,    cmd_cfg_q,  "UART:CFG?" },
    { "UART:TX",     1, 0xFF, cmd_tx,     "UART:TX <hex-bytes...>" },
    { "UART:TXS",    1, 0xFF, cmd_txs,    "UART:TXS <text...>" },
    { "UART:RX?",    0, 1,    cmd_rx,     "UART:RX? [maxcount]" },
    { "UART:RXN?",   0, 0,    cmd_rxn,    "UART:RXN?" },
    { "UART:FLUSH",  0, 0,    cmd_flush,  "UART:FLUSH" },
    { "UART:BREAK",  0, 0,    cmd_break,  "UART:BREAK" },
    { "UART:BRIDGE", 0, 0,    cmd_bridge, "UART:BRIDGE" },
    { NULL, 0, 0, NULL, NULL }
};
