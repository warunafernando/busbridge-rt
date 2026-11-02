/**
 * @file bb_uart.h
 * @brief Target serial port (USART2) on CN9: TX=PD5, RX=PD6, RTS/CTS=PD4/PD3.
 */
#ifndef BB_UART_H
#define BB_UART_H

#include <stdbool.h>

void bb_uart_init(void);

/** Called from the USART2 vector; drains RX into the ring buffer. */
void bb_uart_irq(void);

/**
 * @brief Service transparent bridge mode.
 *
 * No-op unless UART:BRIDGE is active, in which case it shuttles bytes both
 * ways until the escape sequence arrives.
 */
void bb_uart_poll(void);

bool bb_uart_bridge_active(void);

#endif /* BB_UART_H */
