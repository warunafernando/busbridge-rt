/**
 * @file bb_console.c
 * @brief Host-facing console on LPUART1 / the MCU-Link Virtual COM Port.
 *
 * Same contract as the STM32 build: RX is interrupt-driven into a ring buffer
 * so a slow command handler cannot lose incoming characters, TX is polled so
 * fault reporting still works with interrupts off.
 *
 * On the STM32 this was the ST-LINK VCP and it was retired as a host transport
 * once native USB worked - 692 round trips a second against 5409. The same will
 * be true here and more so, because this part has high-speed USB. The console
 * stays because it is the transport that needs no firmware USB stack: when the
 * USB device is the thing that is broken, this is how you find out.
 */
#include "bb_console.h"
#include "bb_board.h"

#include "fsl_lpuart.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* A power of two so the wrap is a mask rather than a modulo - this runs in the
 * receive interrupt. 512 is four commands' worth at the longest line the parser
 * accepts, which is enough slack for a host that writes a burst and then waits. */
#define RX_RING       512u
#define RX_MASK       (RX_RING - 1u)

static volatile uint8_t  s_rx[RX_RING];
static volatile uint16_t s_head;      /* written by the ISR  */
static volatile uint16_t s_tail;      /* written by the loop */
static volatile uint32_t s_overruns;
static bool              s_echo;

/* Assembled across calls, because bb_console_getline() is non-blocking and a
 * line usually arrives in several USB-serial chunks. */
static uint16_t s_len;
static bool     s_overflowed;

static const bb_pin_t s_tx_pin = BB_CONSOLE_TX_PIN;
static const bb_pin_t s_rx_pin = BB_CONSOLE_RX_PIN;

/* ---- interrupt --------------------------------------------------------- */

void bb_console_irq(void)
{
    uint32_t flags = LPUART_GetStatusFlags(BB_CONSOLE_UART);

    /* Hardware overrun: the FIFO filled before this handler ran. Cleared
     * explicitly, because on LPUART an uncleared overrun stops reception
     * entirely - the console would go dead rather than lossy, which is a much
     * harder fault to read. */
    if ((flags & (uint32_t)kLPUART_RxOverrunFlag) != 0u) {
        LPUART_ClearStatusFlags(BB_CONSOLE_UART, (uint32_t)kLPUART_RxOverrunFlag);
        s_overruns++;
    }

    while ((LPUART_GetStatusFlags(BB_CONSOLE_UART) &
            (uint32_t)kLPUART_RxDataRegFullFlag) != 0u) {
        uint8_t  c    = LPUART_ReadByte(BB_CONSOLE_UART);
        uint16_t next = (uint16_t)((s_head + 1u) & RX_MASK);

        if (next == s_tail) {
            /* Software overrun: drop the newest and count it. Dropping the
             * oldest would corrupt a line already half-assembled, which reads
             * as a malformed command rather than as lost input. */
            s_overruns++;
        } else {
            s_rx[s_head] = c;
            s_head       = next;
        }
    }
}

void BB_CONSOLE_IRQHANDLER(void)
{
    bb_console_irq();
    SDK_ISR_EXIT_BARRIER;
}

/* ---- init -------------------------------------------------------------- */

void bb_console_init(void)
{
    lpuart_config_t cfg;

    bb_pin_mux(&s_tx_pin, BB_PAD_UART, false);
    /* SION on RX so the input path is on regardless of what else the pad is
     * doing. Costs nothing and removes a class of "the console went quiet"
     * that is otherwise very hard to see. */
    bb_pin_mux(&s_rx_pin, BB_PAD_UART, true);

    LPUART_GetDefaultConfig(&cfg);
    cfg.baudRate_Bps = BB_CONSOLE_BAUD;
    cfg.enableTx     = true;
    cfg.enableRx     = true;

    if (LPUART_Init(BB_CONSOLE_UART, &cfg,
                    CLOCK_GetRootClockFreq(BB_CONSOLE_CLKROOT)) != kStatus_Success) {
        /* Not bb_error_handler(): that reports through this console. */
        for (;;) {
            __NOP();
        }
    }

    s_head = s_tail = 0u;
    s_len  = 0u;
    s_echo = false;
    s_overflowed = false;

    LPUART_EnableInterrupts(BB_CONSOLE_UART,
                            (uint32_t)kLPUART_RxDataRegFullInterruptEnable |
                            (uint32_t)kLPUART_RxOverrunInterruptEnable);
    HAL_NVIC_SetPriority(BB_CONSOLE_IRQ, 5, 0);
    HAL_NVIC_EnableIRQ(BB_CONSOLE_IRQ);
}

/* ---- output ------------------------------------------------------------ */

void bb_console_write(const void *data, size_t len)
{
    LPUART_WriteBlocking(BB_CONSOLE_UART, (const uint8_t *)data, len);
}

void bb_console_puts(const char *s)
{
    if (s != NULL) {
        bb_console_write(s, strlen(s));
    }
}

void bb_console_printf(const char *fmt, ...)
{
    char    buf[256];
    va_list ap;

    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);

    if (n > 0) {
        size_t len = (size_t)n < sizeof buf ? (size_t)n : sizeof buf - 1u;
        bb_console_write(buf, len);
    }
}

void bb_console_emergency_print(const char *s)
{
    /* Polled, no interrupts, no buffering: this is what a fault handler calls,
     * and by then the ring buffer and the scheduler are not to be trusted. */
    if (s == NULL) {
        return;
    }
    for (; *s != 0; s++) {
        while ((LPUART_GetStatusFlags(BB_CONSOLE_UART) &
                (uint32_t)kLPUART_TxDataRegEmptyFlag) == 0u) {
        }
        LPUART_WriteByte(BB_CONSOLE_UART, (uint8_t)*s);
    }
}

/* ---- input ------------------------------------------------------------- */

int bb_console_getchar(void)
{
    if (s_tail == s_head) {
        return -1;
    }
    uint8_t c = s_rx[s_tail];
    s_tail = (uint16_t)((s_tail + 1u) & RX_MASK);
    return (int)c;
}

int bb_console_getline(char *buf, size_t cap)
{
    for (;;) {
        int ci = bb_console_getchar();
        if (ci < 0) {
            return -1;                  /* nothing complete yet */
        }
        char c = (char)ci;

        if (s_echo) {
            bb_console_write(&c, 1u);
        }

        if (c == '\r' || c == '\n') {
            if (s_len == 0u && !s_overflowed) {
                continue;               /* bare terminator, or the LF of CRLF */
            }
            bool was_over = s_overflowed;
            buf[s_len]    = 0;
            int  n        = (int)s_len;
            s_len         = 0u;
            s_overflowed  = false;
            /* -2 rather than -1 so the caller can tell "too long" from "not
             * ready" and answer with the protocol's own error code. */
            return was_over ? -2 : n;
        }

        if (c == 0x08 || c == 0x7F) {   /* backspace */
            if (s_len > 0u) {
                s_len--;
            }
            continue;
        }

        if ((size_t)s_len + 1u >= cap) {
            /* Keep consuming to the end of the line and report it once, rather
             * than emitting an error per surplus character. */
            s_overflowed = true;
            continue;
        }
        buf[s_len++] = c;
    }
}

void bb_console_set_echo(bool enable)
{
    s_echo = enable;
}

bool bb_console_echo(void)
{
    return s_echo;
}

uint32_t bb_console_rx_overruns(void)
{
    return s_overruns;
}
