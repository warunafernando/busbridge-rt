/**
 * @file bb_io.h
 * @brief Host output routing.
 *
 * The device answers on two transports at once: the ST-LINK Virtual COM Port
 * and the native USB bulk interface. A reply has to leave by the same door the
 * command came in, otherwise two hosts talking at once would each receive the
 * other's answers.
 *
 * The command layer therefore writes through bb_io_*, and the main loop selects
 * the transport before dispatching each line.
 */
#ifndef BB_IO_H
#define BB_IO_H

#include <stdarg.h>
#include <stddef.h>

typedef enum {
    BB_IO_CONSOLE = 0,   /**< USART3 on the ST-LINK VCP */
    BB_IO_USB            /**< native USB bulk on CN13   */
} bb_io_target_t;

/** Route subsequent output. Set by the command pump, per line. */
void           bb_io_select(bb_io_target_t target);
bb_io_target_t bb_io_current(void);

void bb_io_write(const void *data, size_t len);
void bb_io_puts(const char *s);
void bb_io_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

#endif /* BB_IO_H */
