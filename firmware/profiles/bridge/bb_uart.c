/**
 * @file bb_uart.c
 * @brief Target serial port on LPUART4 — framed and transparent modes.
 *
 * Translated from `_stm32_reference/bb_uart.c`. Command surface unchanged.
 *
 * Separate from the console on purpose: driving a target's serial port must not
 * cost the transport the commands are arriving over. On this board the console
 * is LPUART1 (the MCU-Link VCOM) and this is LPUART4 on D2/D3.
 *
 * Two modes, and the second is the interesting one:
 *
 *   framed       `UART:TX`/`UART:RX?` - bytes in and out through the command
 *                protocol, one reply per command.
 *   transparent  `UART:BRIDGE` - the console becomes a wire to the target until
 *                Ctrl-] three times. For talking to a bootloader or a shell that
 *                expects a terminal rather than a request/response protocol.
 */
#include "bb_board.h"
#include "bb_cmd.h"
#include "bb_console.h"
#include "bb_image.h"
#include "bb_io.h"
#include "bb_status.h"
#include "bb_uart.h"

#include "fsl_clock.h"
#include "fsl_lpuart.h"

#include <stdio.h>
#include <string.h>

#define RX_RING       1024u
#define RX_MASK       (RX_RING - 1u)

/* Ctrl-] three times. One would be unusable - it is a legitimate byte - and
 * three is short enough to type without being something a binary stream
 * produces by accident. */
#define ESCAPE_BYTE   0x1Du
#define ESCAPE_COUNT  3u

static const bb_pin_t s_tx = BB_TUART_TX;
static const bb_pin_t s_rx = BB_TUART_RX;

static volatile uint8_t  s_rx_buf[RX_RING];
static volatile uint16_t s_head;
static volatile uint16_t s_tail;
static volatile uint32_t s_overruns;

static struct {
    uint32_t baud;
    uint8_t  bits;      /**< 7, 8 or 9 including parity, as the host states it */
    char     parity;    /**< 'N', 'E' or 'O' */
    uint8_t  stop;      /**< 1 or 2 */
    bool     rtscts;
    bool     configured;
} s = { .baud = 115200u, .bits = 8u, .parity = 'N', .stop = 1u };

static bool     bridge_active;
static unsigned escape_seen;

/* ---- interrupt --------------------------------------------------------- */

void bb_uart_irq(void)
{
    if ((LPUART_GetStatusFlags(BB_TARGET_UART) &
         (uint32_t)kLPUART_RxOverrunFlag) != 0u) {
        /* Must be cleared or LPUART stops receiving entirely - a dead port is
         * a much harder fault to read than a lossy one. */
        LPUART_ClearStatusFlags(BB_TARGET_UART, (uint32_t)kLPUART_RxOverrunFlag);
        s_overruns++;
    }
    while ((LPUART_GetStatusFlags(BB_TARGET_UART) &
            (uint32_t)kLPUART_RxDataRegFullFlag) != 0u) {
        uint8_t  c    = LPUART_ReadByte(BB_TARGET_UART);
        uint16_t next = (uint16_t)((s_head + 1u) & RX_MASK);
        if (next == s_tail) {
            s_overruns++;           /* drop newest, count it */
        } else {
            s_rx_buf[s_head] = c;
            s_head           = next;
        }
    }
}

void BB_TARGET_IRQHANDLER(void)
{
    bb_uart_irq();
    SDK_ISR_EXIT_BARRIER;
}

/* ---- configuration ----------------------------------------------------- */

static bool uart_apply(void)
{
    lpuart_config_t cfg;

    bb_pin_mux(&s_tx, BB_PAD_UART, false);
    bb_pin_mux(&s_rx, BB_PAD_UART, true);

    LPUART_GetDefaultConfig(&cfg);
    cfg.baudRate_Bps = s.baud;
    cfg.enableTx     = true;
    cfg.enableRx     = true;

    /* The host states total bits including parity, which is how every terminal
     * program presents it: "8E1" is eight data bits and a parity bit. LPUART
     * counts data bits separately, so 8E1 becomes nine-bit mode with parity. */
    switch (s.parity) {
    case 'E': cfg.parityMode = kLPUART_ParityEven; break;
    case 'O': cfg.parityMode = kLPUART_ParityOdd;  break;
    default:  cfg.parityMode = kLPUART_ParityDisabled; break;
    }
    if (s.parity == 'N') {
        cfg.dataBitsCount = (s.bits == 7u) ? kLPUART_SevenDataBits
                                           : kLPUART_EightDataBits;
    } else {
        cfg.dataBitsCount = (s.bits == 7u) ? kLPUART_SevenDataBits
                                           : kLPUART_EightDataBits;
    }
    cfg.stopBitCount = (s.stop == 2u) ? kLPUART_TwoStopBit : kLPUART_OneStopBit;
    cfg.enableRxRTS  = s.rtscts;
    cfg.enableTxCTS  = s.rtscts;

#if BB_TARGET_UART_HAS_FLOW
    /*
     * The flow-control pads are muxed only while flow control is on, because
     * they are shared with sniffer taps d4 and d5 and a pad can serve one
     * peripheral at a time. Turning rtscts off hands them back as plain inputs
     * rather than leaving LPUART4 holding two pads it is not using - which
     * would make a later SNIFF:CH on those taps fail for a reason nothing in
     * the sniffer could explain.
     *
     * CTS is an input; RTS is driven by the peripheral, so it is push-pull.
     */
    if (s.rtscts) {
        static const bb_pin_t cts = BB_TARGET_UART_CTS_PIN;
        static const bb_pin_t rts = BB_TARGET_UART_RTS_PIN;

        bb_pin_mux(&cts, BB_PAD_INPUT_PU, true);
        bb_pin_mux(&rts, BB_PAD_PUSHPULL, false);
    }
    /*
     * Nothing is done when flow control is off, and that is deliberate rather
     * than an omission.
     *
     * The first version handed the pads back as GPIO inputs here, on the
     * principle that a peripheral should not keep pads it is not using. It hung
     * the controller: every UART:CFG without `rtscts` - which is the common case
     * and the first thing a host does - took that branch and the device stopped
     * answering. Testing only the rtscts path is what let it through.
     *
     * Handing them back is not needed anyway. push_config() muxes the taps it
     * wants when the sniffer claims them, so SNIFF:CH takes d4 and d5 back
     * whether or not LPUART4 was told to let go.
     */
#endif

    /* Only release a peripheral that is actually clocked - see the long note in
     * bb_i2c.c's i2c_apply(). A *_Deinit() gates the clock as its last act and
     * resets registers as its first, so deinitialising twice writes to an
     * unclocked peripheral and stalls the AHB with no fault raised. *_Init()
     * ungates and resets on its own, so skipping the deinit is safe. */
    if (s.configured) {
        LPUART_Deinit(BB_TARGET_UART);
    }
    if (LPUART_Init(BB_TARGET_UART, &cfg,
                    CLOCK_GetRootClockFreq(BB_TARGET_CLKROOT)) != kStatus_Success) {
        s.configured = false;
        return false;
    }

    s_head = s_tail = 0u;
    LPUART_EnableInterrupts(BB_TARGET_UART,
                            (uint32_t)kLPUART_RxDataRegFullInterruptEnable |
                            (uint32_t)kLPUART_RxOverrunInterruptEnable);
    HAL_NVIC_SetPriority(BB_TARGET_UART_IRQ, 6, 0);
    HAL_NVIC_EnableIRQ(BB_TARGET_UART_IRQ);

    s.configured = true;
    return true;
}

void bb_uart_init(void)
{
    (void)uart_apply();
}

/* ---- byte access ------------------------------------------------------- */

static int uart_getchar(void)
{
    if (s_tail == s_head) {
        return -1;
    }
    uint8_t c = s_rx_buf[s_tail];
    s_tail = (uint16_t)((s_tail + 1u) & RX_MASK);
    return (int)c;
}

static unsigned uart_available(void)
{
    return (unsigned)((s_head - s_tail) & RX_MASK);
}

/*
 * Returns false on a transmit timeout, and the callers report it.
 *
 * The status is checked because it can now fail. With UART_RETRY_TIMES set the
 * wait is bounded, and the way it times out is worth naming: hardware flow
 * control with nothing wired to CTS means the peer is telling us not to send, so
 * the transmitter never drains. Before that bound this hung the command loop -
 * UART:CFG with rtscts succeeded and the next command timed out, because the
 * reply to it could not be transmitted.
 *
 * The error is raised by the caller rather than here, because a helper that
 * replies leaves the command emitting two replies for one request.
 */
static bool uart_send(const uint8_t *d, size_t n)
{
    return LPUART_WriteBlocking(BB_TARGET_UART, d, n) == kStatus_Success;
}

/** The one message for a transmit that could not drain. */
static void uart_tx_timeout(void)
{
    bb_reply_errf(BB_ERR_TIMEOUT, "tx-timeout%s",
                  s.rtscts ? " cts-not-asserted" : "");
}

/* ---- commands ---------------------------------------------------------- */

static void cmd_cfg(int argc, char **argv)
{
    uint32_t baud;

    /* Up to 6 Mbaud. The STM32 was verified to 3 Mbaud; this part's LPUART will
     * go further, and the limit is stated as what the divider can express
     * rather than as a measured figure - nothing here has been on a wire yet. */
    if (!bb_parse_u32(argv[1], &baud) || baud < 300u || baud > 6000000u) {
        bb_reply_err(BB_ERR_RANGE, "baud-300-6000000");
        return;
    }
    uint32_t bits = 8u;
    if (argc > 2) {
        if (!bb_parse_u32(argv[2], &bits) || bits < 7u || bits > 9u) {
            bb_reply_err(BB_ERR_RANGE, "bits-7-8-9");
            return;
        }
    }
    char par = 'N';
    if (argc > 3) {
        char c = argv[3][0];
        if (c >= 'a' && c <= 'z') {
            c = (char)(c - 32);
        }
        if (c != 'N' && c != 'E' && c != 'O') {
            bb_reply_err(BB_ERR_RANGE, "parity-N-E-O");
            return;
        }
        par = c;
    }
    uint32_t stop = 1u;
    if (argc > 4) {
        if (!bb_parse_u32(argv[4], &stop) || (stop != 1u && stop != 2u)) {
            bb_reply_err(BB_ERR_RANGE, "stop-1-or-2");
            return;
        }
    }
    bool flow = false;
    if (argc > 5) {
        if (bb_streq_ci(argv[5], "rtscts")) {
            flow = true;
        } else if (!bb_streq_ci(argv[5], "none")) {
            bb_reply_err(BB_ERR_RANGE, "none-or-rtscts");
            return;
        }
    }
#if !BB_TARGET_UART_HAS_FLOW
    if (flow) {
        bb_reply_err(BB_ERR_UNSUPPORTED, "rtscts-pads-not-on-this-board");
        return;
    }
#endif

    s.baud   = baud;
    s.bits   = (uint8_t)bits;
    s.parity = par;
    s.stop   = (uint8_t)stop;
    s.rtscts = flow;
    if (!uart_apply()) {
        bb_reply_err(BB_ERR_HAL, "uart-init-failed");
        return;
    }
    bb_reply_ok();
}

static void cmd_cfg_q(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    bb_reply_okf("baud=%lu bits=%u parity=%c stop=%u flow=%s rx=%u overruns=%lu",
                 (unsigned long)s.baud, (unsigned)s.bits, s.parity,
                 (unsigned)s.stop, s.rtscts ? "rtscts" : "none",
                 uart_available(), (unsigned long)s_overruns);
}

static void cmd_tx(int argc, char **argv)
{
    uint8_t buf[BB_PAYLOAD_MAX];
    size_t  len;

    if (!bb_parse_bytes(argc, argv, 1, buf, sizeof buf, &len) || len == 0u) {
        bb_reply_errf(BB_ERR_TOO_LONG, "payload-1..%u", (unsigned)BB_PAYLOAD_MAX);
        return;
    }
    if (!uart_send(buf, len)) {
        uart_tx_timeout();
        return;
    }
    bb_reply_ok();
}

static void cmd_txs(int argc, char **argv)
{
    /* Text, re-joined with the single spaces the tokeniser removed. Anything
     * needing exact whitespace or non-ASCII should use UART:TX with hex. */
    for (int i = 1; i < argc; i++) {
        if (i > 1) {
            if (!uart_send((const uint8_t *)" ", 1u)) {
                uart_tx_timeout();
                return;
            }
        }
        if (!uart_send((const uint8_t *)argv[i], strlen(argv[i]))) {
            uart_tx_timeout();
            return;
        }
    }
    bb_reply_ok();
}

static void cmd_rx(int argc, char **argv)
{
    uint32_t want = BB_PAYLOAD_MAX;
    uint8_t  buf[BB_PAYLOAD_MAX];
    size_t   n = 0;

    if (argc > 1 && (!bb_parse_u32_max(argv[1], BB_PAYLOAD_MAX, &want) || want == 0u)) {
        bb_reply_errf(BB_ERR_RANGE, "max-1..%u", (unsigned)BB_PAYLOAD_MAX);
        return;
    }
    while (n < want) {
        int c = uart_getchar();
        if (c < 0) {
            break;
        }
        buf[n++] = (uint8_t)c;
    }
    /* An empty read is OK with no bytes, not an error: "nothing has arrived
     * yet" is a successful answer to "what has arrived". */
    bb_reply_ok_bytes(buf, n);
}

static void cmd_rxn(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    bb_reply_okf("%u", uart_available());
}

static void cmd_flush(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    s_tail = s_head;
    bb_reply_ok();
}

static void cmd_break(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    /* A break is a framing violation by definition - the line held low past a
     * stop bit - so it is generated by driving the pad rather than through the
     * peripheral, which will not emit an invalid frame for you. */
    bb_pin_gpio(&s_tx, true, false, BB_PAD_PUSHPULL);
    bb_pin_write(&s_tx, false);
    /* Roughly 13 bit times at the configured baud, which is what the standard
     * asks for and what every receiver looks for. */
    uint32_t cycles = (bb_cycle_hz() / s.baud) * 13u;
    uint32_t start  = bb_cycles();
    while ((bb_cycles() - start) < cycles) {
    }
    bb_pin_write(&s_tx, true);
    bb_pin_mux(&s_tx, BB_PAD_UART, false);
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
            if (!uart_send(&esc, 1u)) {
                uart_tx_timeout();
                return;
            }
            escape_seen--;
        }
        uint8_t byte = (uint8_t)c;
        if (!uart_send(&byte, 1u)) {
            uart_tx_timeout();
            return;
        }
    }
}

/* ---- status ------------------------------------------------------------ */

void bb_uart_status(bb_status_t *out)
{
    if (out == NULL) {
        return;
    }
    memset(out, 0, sizeof *out);
    bb_image_copy_str(out->pins, sizeof out->pins,
                      BB_TUART_PINS, strlen(BB_TUART_PINS));
    (void)snprintf(out->detail, sizeof out->detail, "%lu %u%c%u",
                   (unsigned long)s.baud, (unsigned)s.bits, s.parity,
                   (unsigned)s.stop);
    out->state = !s.configured ? BB_STATE_ABSENT
                 : bridge_active ? BB_STATE_ACTIVE
                                 : BB_STATE_IDLE;
}

/* ---- table ------------------------------------------------------------- */

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
