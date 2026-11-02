#include "bb_io.h"
#include "bb_console.h"
#include "bb_usb.h"
#include <stdio.h>
#include <string.h>

static bb_io_target_t target = BB_IO_CONSOLE;

void bb_io_select(bb_io_target_t which)
{
    target = which;
}

bb_io_target_t bb_io_current(void)
{
    return target;
}

void bb_io_write(const void *data, size_t len)
{
    if (target == BB_IO_USB) {
        bb_usb_write(data, len);
    } else {
        bb_console_write(data, len);
    }
}

void bb_io_puts(const char *s)
{
    bb_io_write(s, strlen(s));
}

void bb_io_printf(const char *fmt, ...)
{
    char    buf[256];
    va_list ap;

    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);

    if (n > 0) {
        size_t len = (size_t)n < sizeof buf ? (size_t)n : sizeof buf - 1u;
        bb_io_write(buf, len);
    }
}
