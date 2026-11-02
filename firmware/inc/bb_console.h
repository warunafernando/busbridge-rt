/**
 * @file bb_console.h
 * @brief Host-facing console on USART3 / ST-LINK Virtual COM Port.
 *
 * Phase 1 transport for the command protocol.  RX is interrupt-driven into a
 * ring buffer so that a slow command handler cannot lose incoming characters;
 * TX is polled, which is fine for request/response traffic and keeps fault
 * reporting possible with interrupts off.
 */
#ifndef BB_CONSOLE_H
#define BB_CONSOLE_H

#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

void bb_console_init(void);

void bb_console_write(const void *data, size_t len);
void bb_console_puts(const char *s);
void bb_console_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

/** Polled, interrupt-free write for fault paths. */
void bb_console_emergency_print(const char *s);

/**
 * @brief Non-blocking line reader.
 * @param buf  destination, NUL-terminated on success
 * @param cap  size of @p buf
 * @return length of the line, or -1 when no complete line is available yet.
 *         A line too long for @p cap is reported as an overflow error and
 *         discarded.
 */
int  bb_console_getline(char *buf, size_t cap);

/** Raw byte access, used by transparent UART bridge mode. -1 when empty. */
int  bb_console_getchar(void);

void bb_console_set_echo(bool enable);
bool bb_console_echo(void);

/** Characters lost to ring-buffer overrun since boot. */
uint32_t bb_console_rx_overruns(void);

/** Called from the USART3 interrupt vector. */
void bb_console_irq(void);

#endif /* BB_CONSOLE_H */
