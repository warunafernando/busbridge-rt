#include "bb_cmd.h"
#include "fsl_clock.h"
#include "fsl_common.h"
#include "bb_board.h"
#include "bb_image.h"
#include "bb_console.h"
#include "bb_io.h"
#include "bb_profile.h"
#include "bb_usb.h"
#include "bb_version.h"
#include <stdio.h>
#include <string.h>

/* SYS is in every image; the rest is whatever the profile compiled in. Kept
 * NULL-terminated rather than counted so neither list has to know the other's
 * length. */

/* ---- helpers ----------------------------------------------------------- */

static const bb_cmd_t *table_at(size_t i)
{
    if (i == 0u) {
        return bb_cmd_sys;
    }
    return bb_profile_tables[i - 1u];
}


static char upper(char c)
{
    return (c >= 'a' && c <= 'z') ? (char)(c - 32) : c;
}

bool bb_streq_ci(const char *a, const char *b)
{
    while (*a != 0 && *b != 0) {
        if (upper(*a++) != upper(*b++)) {
            return false;
        }
    }
    return *a == *b;
}

static bool hex_nibble(char c, uint8_t *out)
{
    if (c >= '0' && c <= '9') {
        *out = (uint8_t)(c - '0');
        return true;
    }
    c = upper(c);
    if (c >= 'A' && c <= 'F') {
        *out = (uint8_t)(c - 'A' + 10);
        return true;
    }
    return false;
}

bool bb_parse_u32(const char *s, uint32_t *out)
{
    uint32_t base  = 10;
    uint64_t value = 0;
    bool     any   = false;

    if (s == NULL || *s == 0) {
        return false;
    }
    if (s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) {
        base = 16;
        s += 2;
    } else if (s[0] == '0' && (s[1] == 'b' || s[1] == 'B')) {
        base = 2;
        s += 2;
    }
    for (; *s != 0; s++) {
        uint8_t digit;
        if (*s == '_') {
            continue;                   /* 1_000_000 reads better than 1000000 */
        }
        if (!hex_nibble(*s, &digit) || digit >= base) {
            return false;
        }
        value = value * base + digit;
        if (value > 0xFFFFFFFFull) {
            return false;
        }
        any = true;
    }
    if (!any) {
        return false;
    }
    *out = (uint32_t)value;
    return true;
}

bool bb_parse_u32_max(const char *s, uint32_t max, uint32_t *out)
{
    return bb_parse_u32(s, out) && *out <= max;
}

bool bb_parse_bytes(int argc, char **argv, int first, uint8_t *out, size_t cap, size_t *len)
{
    size_t n = 0;

    for (int i = first; i < argc; i++) {
        const char *tok = argv[i];
        if (tok[0] == '0' && (tok[1] == 'x' || tok[1] == 'X')) {
            tok += 2;
        }
        size_t digits = strlen(tok);
        if (digits == 0 || (digits & 1u) != 0u) {
            return false;
        }
        for (size_t d = 0; d < digits; d += 2) {
            uint8_t hi, lo;
            if (!hex_nibble(tok[d], &hi) || !hex_nibble(tok[d + 1], &lo)) {
                return false;
            }
            if (n >= cap) {
                return false;
            }
            out[n++] = (uint8_t)((hi << 4) | lo);
        }
    }
    *len = n;
    return true;
}

/* ---- replies ----------------------------------------------------------- */

/* Whether the reply just emitted was a success. The reply helpers are the only
 * place that knows, since a handler reports its own outcome and returns void. */
static bool reply_ok = true;

void bb_reply_ok(void)
{
    reply_ok = true;
    bb_io_puts("OK\r\n");
}

/*
 * A reply line's working buffer.
 *
 * This was 256 bytes, and vsnprintf truncates to fit without saying so, so a
 * reply that outgrew it lost its tail silently. SNIFF:STAT? did exactly that:
 * it had grown past 255 characters one counter at a time, and the newest
 * tokens - the ones a measurement was reading - were simply not in the string.
 * Nothing reported an error, because as far as every layer was concerned the
 * reply had been formatted and sent.
 *
 * 480 is the transport's own limit (BB_MSG_MAX_PAYLOAD), which is the right
 * ceiling: a line that does not fit here would not have fit on the wire either.
 * The stack is 8 KB, so the extra quarter-kilobyte is not close to a concern.
 */
#define BB_REPLY_MAX  480u

void bb_reply_okf(const char *fmt, ...)
{
    reply_ok = true;
    char    buf[BB_REPLY_MAX];
    va_list ap;

    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);

    /*
     * Say so when it did not fit, rather than returning a plausible-looking
     * short line. A truncated reply that announces itself costs one glance; a
     * silent one costs an afternoon.
     */
    if ((n > 0) && ((size_t)n >= sizeof buf)) {
        static const char cut[] = "...TRUNC";
        memcpy(&buf[sizeof buf - sizeof cut], cut, sizeof cut);
        n = (int)(sizeof buf - 1u);
    }

    /* One write for the whole line, not three.
     *
     * Each write ends by pumping the USB endpoint, so writing the prefix, the
     * payload and the terminator separately put them in separate USB packets,
     * and how they landed depended on pump timing rather than on anything
     * meaningful. That is a fragile shape for a transport whose reader is
     * hunting for a terminator: composed here, a reply leaves as full packets
     * plus one short tail, which is what a bulk endpoint is meant to produce.
     * Serial does not care either way. */
    char   line[sizeof buf + 8u];
    size_t len = 0;

    line[len++] = 'O';
    line[len++] = 'K';
    line[len++] = ' ';
    if (n > 0) {
        size_t body = (size_t)n < sizeof buf ? (size_t)n : sizeof buf - 1u;
        memcpy(&line[len], buf, body);
        len += body;
    }
    line[len++] = '\r';
    line[len++] = '\n';
    bb_io_write(line, len);
}

void bb_reply_ok_bytes(const uint8_t *data, size_t len)
{
    static const char hex[] = "0123456789ABCDEF";
    char line[3u * 64u];

    /*
     * Refuse rather than overrun the line.
     *
     * This streams, so nothing here truncates and for a long time that read as
     * "no limit". The limit is at the other end: the host library reads a reply
     * into a fixed buffer and used to keep the prefix that fit, so an oversized
     * line arrived looking like a short, well-formed one. SNIFF:RAW:READ? 96
     * lost seventeen edges to that on every call, and because the command had
     * already advanced its cursor past them they could not be asked for again.
     *
     * An error is the only honest answer: the caller asked for something that
     * cannot be delivered, and saying so names the caller. Truncating here
     * would reproduce the original bug with the sign changed.
     */
    if (len > BB_REPLY_BYTES_MAX) {
        bb_reply_errf(BB_ERR_RANGE, "reply-%u-bytes-over-%u",
                      (unsigned)len, (unsigned)BB_REPLY_BYTES_MAX);
        return;
    }

    reply_ok = true;
    bb_io_puts("OK");
    /* Emitted in chunks so a 256-byte payload needs no 768-byte stack buffer. */
    while (len > 0) {
        size_t chunk = len < 64u ? len : 64u;
        size_t pos   = 0;
        for (size_t i = 0; i < chunk; i++) {
            line[pos++] = ' ';
            line[pos++] = hex[data[i] >> 4];
            line[pos++] = hex[data[i] & 0x0Fu];
        }
        bb_io_write(line, pos);
        data += chunk;
        len -= chunk;
    }
    bb_io_puts("\r\n");
}

void bb_reply_err(int code, const char *text)
{
    reply_ok = false;
    bb_io_printf("ERR %d %s\r\n", code, text);
}

void bb_reply_errf(int code, const char *fmt, ...)
{
    reply_ok = false;
    char    buf[192];
    va_list ap;

    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);

    /* Same treatment as bb_reply_okf: an error that did not fit says so. This
     * matters more here than there, because error text is what a caller reads
     * when it already does not know what happened, and several of this
     * project's messages carry a "see I3C:RXLOG?"-style pointer at the end -
     * precisely the part a silent truncation removes. */
    if ((n > 0) && ((size_t)n >= sizeof buf)) {
        static const char cut[] = "...TRUNC";
        memcpy(&buf[sizeof buf - sizeof cut], cut, sizeof cut);
    }
    bb_io_printf("ERR %d %s\r\n", code, buf);
}

bool bb_reply_hal(int st_hal, const char *what)
{
    switch (st_hal) {
    case HAL_OK:
        return true;
    case HAL_TIMEOUT:
        bb_reply_errf(BB_ERR_TIMEOUT, "%s-timeout", what);
        return false;
    case HAL_BUSY:
        bb_reply_errf(BB_ERR_BUS_STATE, "%s-busy", what);
        return false;
    default:
        bb_reply_errf(BB_ERR_HAL, "%s-error", what);
        return false;
    }
}

/* ---- dispatch ---------------------------------------------------------- */

static const bb_cmd_t *lookup(const char *word)
{
    for (size_t t = 0; table_at(t) != NULL; t++) {
        for (const bb_cmd_t *c = table_at(t); c->name != NULL; c++) {
            if (bb_streq_ci(word, c->name)) {
                return c;
            }
        }
    }
    return NULL;
}

void bb_cmd_execute(char *line)
{
    char *argv[BB_CMD_MAX_ARGS];
    int   argc = 0;
    char *p    = line;

    for (;;) {
        while (*p == ' ' || *p == '\t') {
            p++;
        }
        if (*p == 0) {
            break;
        }
        if (argc == (int)BB_CMD_MAX_ARGS) {
            /* Refuse rather than truncate: silently dropping the tail of a
             * payload would look like a successful short write. */
            bb_reply_errf(BB_ERR_TOO_LONG, "too-many-arguments max-%u",
                          (unsigned)BB_CMD_MAX_ARGS);
            return;
        }
        argv[argc++] = p;
        while (*p != 0 && *p != ' ' && *p != '\t') {
            p++;
        }
        if (*p != 0) {
            *p++ = 0;
        }
    }
    if (argc == 0) {
        return;
    }
    if (argv[0][0] == '#') {
        return;                         /* comment line, stays silent */
    }

    const bb_cmd_t *cmd = lookup(argv[0]);
    if (cmd == NULL) {
        bb_reply_errf(BB_ERR_UNKNOWN_CMD, "unknown-command %s", argv[0]);
        return;
    }

    int nargs = argc - 1;
    if (nargs < (int)cmd->min_args ||
        (cmd->max_args != 0xFFu && nargs > (int)cmd->max_args)) {
        bb_reply_errf(BB_ERR_ARGC, "usage %s", cmd->usage != NULL ? cmd->usage : cmd->name);
        return;
    }
    reply_ok = true;
    cmd->fn(argc, argv);
    /* Told to the profile, not to a display. The bridge profile has a panel
     * that shows the last command; the peripheral profile has no screen and no
     * business linking one in. */
    bb_profile_note_command(argv[0], reply_ok);
}

/* ---- SYS namespace ----------------------------------------------------- */

static void cmd_idn(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    bb_io_printf("OK %s,%s,%s,%08lX%08lX%08lX\r\n",
                      BB_PRODUCT, BB_BOARD, BB_VERSION,
                      (unsigned long)bb_uid_word(2),
                      (unsigned long)bb_uid_word(1),
                      (unsigned long)bb_uid_word(0));
}

/** What silicon this is, and whether it is the silicon this build expected. */
static void cmd_id(int argc, char **argv)
{
    bb_silicon_t s;

    (void)argc;
    (void)argv;
    bb_board_silicon(&s);
    /* The profile is here rather than only in the boot banner: two boards on a
     * bench, one bridge and one peripheral, are indistinguishable to *IDN?, and
     * pointing a script at the wrong one fails for reasons that look nothing
     * like the real cause. */
    /* uid is reported because a serial number of zeros is a fuse read that
     * failed, and that is worth being able to see rather than infer. */
    bb_reply_okf("board=%s dev=0x%03X rev=0x%04X flash_kb=%u "
                 "package=0x%04X expect=0x%03X checked=%u match=%u "
                 "uid=%08lX%08lX%08lX%08lX profile=%s",
                 BB_BOARD_NAME, (unsigned)s.dev_id, (unsigned)s.revision,
                 (unsigned)s.flash_kb, (unsigned)s.package,
                 (unsigned)s.expect_dev_id, s.checked ? 1u : 0u,
                 s.matches ? 1u : 0u,
                 (unsigned long)bb_uid_word(3), (unsigned long)bb_uid_word(2),
                 (unsigned long)bb_uid_word(1), (unsigned long)bb_uid_word(0),
                 bb_profile_name);
}

static void cmd_help(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    for (size_t t = 0; table_at(t) != NULL; t++) {
        for (const bb_cmd_t *c = table_at(t); c->name != NULL; c++) {
            bb_io_printf("# %s\r\n", c->usage != NULL ? c->usage : c->name);
        }
    }
    bb_reply_ok();
}

static void cmd_clk(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    /* The field names are the STM32 build's, kept so existing host code keeps
     * parsing. `sysclk` is the core this image runs on and `hclk` the bus
     * clock. `pclk1` has no counterpart here - the RT1180 clocks each
     * peripheral from its own root rather than from one shared APB - so it
     * carries the cycle counter's rate instead, which is the number a host
     * actually wants when reasoning about timing. Reporting a fabricated APB
     * frequency would be worse than reporting something true under an
     * inherited name. */
    bb_reply_okf("sysclk=%lu hclk=%lu pclk1=%lu",
                 (unsigned long)CLOCK_GetFreq(kCLOCK_CpuClk),
                 (unsigned long)CLOCK_GetFreq(kCLOCK_CoreSysClk),
                 (unsigned long)bb_cycle_hz());
}

static void cmd_led(int argc, char **argv)
{
    uint32_t which;
    uint32_t state;

    (void)argc;
    /* Against what this board has, not a fixed three. Answering "ok" for a LED
     * that is not fitted is worse than refusing: it reads as working. */
    if (!bb_parse_u32_max(argv[1], BB_LED_COUNT, &which) || which < 1u) {
        bb_reply_errf(BB_ERR_RANGE, "led-1..%u", (unsigned)BB_LED_COUNT);
        return;
    }

    /* AUTO hands the LED back to the firmware, which is the only way to get
     * the heartbeat again once the host has taken it. */
    if (bb_streq_ci(argv[2], "AUTO")) {
        bb_led_release((bb_led_t)(which - 1u));
        bb_reply_ok();
        return;
    }
    if (!bb_parse_u32_max(argv[2], 1, &state)) {
        bb_reply_err(BB_ERR_RANGE, "state-0-1-or-AUTO");
        return;
    }
    bb_led_set((bb_led_t)(which - 1u), state != 0u);
    bb_reply_ok();
}

static void cmd_echo(int argc, char **argv)
{
    uint32_t on;

    (void)argc;
    if (!bb_parse_u32_max(argv[1], 1, &on)) {
        bb_reply_err(BB_ERR_RANGE, "0-or-1");
        return;
    }
    bb_console_set_echo(on != 0u);
    bb_reply_ok();
}

/** True when @p s starts with @p prefix, ignoring case. */
static bool prefix_ci(const char *s, const char *prefix)
{
    while (*prefix != 0) {
        if (*s == 0 || upper(*s) != upper(*prefix)) {
            return false;
        }
        s++;
        prefix++;
    }
    return true;
}

/**
 * @brief Publish the pin map, optionally filtered by a signal prefix.
 *
 * Streamed rather than formatted into a buffer: the whole map is around 700
 * characters and bb_reply_okf works from 256 bytes, so building it there would
 * silently truncate.  bb_reply_ok_bytes streams for the same reason.
 */
/*
 * Drive or pulse a named bus pin as a plain GPIO.
 *
 * A bring-up instrument, and it exists because of a specific failure mode: a
 * two-board link where the firmware is healthy at both ends and a wire is on
 * the wrong header pin looks exactly like a dead peripheral. Inferring which
 * pin is which from transfer behaviour is slow and ambiguous - a transfer moves
 * both lines, so it cannot tell them apart - whereas a pulse train on one named
 * pin can be found with a meter in seconds.
 *
 * `SYS:PULSE <signal> <n>` emits n low/high pairs on one pin and leaves it high,
 * which is the idle level for every bus here. `SYS:GPIO <signal> <0|1>` holds a
 * level instead, for continuity checking.
 *
 * The pins are driven push-pull, so this must not be pointed at a pin something
 * else is actively driving. On an open-drain bus at idle that is safe, which is
 * the situation it is meant for.
 */
typedef struct {
    const char *signal;
    bb_pin_t    pin;
} bb_named_pin_t;

static const bb_named_pin_t s_named_pins[] = {
#if BB_I2C1_ENABLE
    { "i2c1.scl", BB_I2C1_SCL }, { "i2c1.sda", BB_I2C1_SDA },
#endif
    { "i2c2.scl", BB_I2C2_SCL }, { "i2c2.sda", BB_I2C2_SDA },
    { "i2c3.scl", BB_I2C3_SCL }, { "i2c3.sda", BB_I2C3_SDA },
    { "i3c1.scl", BB_I3C_P1_SCL }, { "i3c1.sda", BB_I3C_P1_SDA },
    { "i3c2.scl", BB_I3C_P2_SCL }, { "i3c2.sda", BB_I3C_P2_SDA },
};

static const bb_pin_t *named_pin(const char *want)
{
    for (unsigned i = 0; i < (sizeof s_named_pins / sizeof s_named_pins[0]); i++) {
        if (bb_streq_ci(s_named_pins[i].signal, want)) {
            return &s_named_pins[i].pin;
        }
    }
    return NULL;
}

static void named_pin_list(void)
{
    for (unsigned i = 0; i < (sizeof s_named_pins / sizeof s_named_pins[0]); i++) {
        bb_io_puts(i ? "," : " ");
        bb_io_puts(s_named_pins[i].signal);
    }
}

static void cmd_gpio(int argc, char **argv)
{
    uint32_t        level;
    const bb_pin_t *p = named_pin(argv[1]);

    (void)argc;
    if (p == NULL) {
        bb_io_printf("ERR %d no-such-signal try:", BB_ERR_RANGE);
        named_pin_list();
        bb_io_puts("\r\n");
        return;
    }
    if (!bb_parse_u32_max(argv[2], 1u, &level)) {
        bb_reply_err(BB_ERR_RANGE, "level-0-or-1");
        return;
    }
    bb_pin_gpio(p, true, level != 0u, BB_PAD_PUSHPULL);
    bb_reply_ok();
}

/*
 * SYS:SYNC - a marker the two cores can both put a time on.
 *
 * This exists to answer one question nothing else here could: where does the
 * capture core's sample-tick origin sit in the controller core's own clock?
 * The two timebases are independent - the CM33 stamps with bb_cycles(),
 * SysTick-derived at 240 MHz, and the CM7 counts its own sample clock from the
 * moment it started - so `SNIFF:SRC both` has had no way to interleave the two
 * streams and refused rather than guess.
 *
 * Asking the capture core for its tick over the link cannot answer it. The
 * round trip is tens of microseconds and its own jitter is the whole figure
 * being measured; and `cap.tick` is advanced by software draining shifters, so
 * it lags real time by however deep the queue happens to be. Both errors are
 * far larger than a bit time on a 12.5 MHz bus.
 *
 * So the marker is physical. This drives an edge on a pad wired to the taps,
 * with bb_cycles() read either side of the store, and the capture core
 * timestamps the same edge in its own ticks. One edge, two clocks, and the
 * only uncertainty is the width of that bracket - reported rather than
 * averaged away, because it is the error bar on everything derived from it.
 *
 * Push-pull on a pad something else may be driving is the usual hazard (see
 * SYS:PULSE above), so this is for an idle bus. It steals the pad, and
 * i3c_claim_pads() takes it back on the next transfer.
 */
static void cmd_sync(int argc, char **argv)
{
    uint32_t        n = 8u;
    uint32_t        gap = 1000u;
    const bb_pin_t *p = named_pin(argv[1]);

    if (p == NULL) {
        bb_io_printf("ERR %d no-such-signal try:", BB_ERR_RANGE);
        named_pin_list();
        bb_io_puts("\r\n");
        return;
    }
    /* Twelve, because each edge prints two numbers and twenty-four of them
     * plus the header is about as much as a reply line holds. A longer
     * baseline comes from running this again later in the same capture, not
     * from a longer reply. */
    if (argc > 2 && !bb_parse_u32_max(argv[2], 12u, &n)) {
        bb_reply_err(BB_ERR_RANGE, "count-1..12");
        return;
    }
    if (argc > 3 && !bb_parse_u32_max(argv[3], 100000u, &gap)) {
        bb_reply_err(BB_ERR_RANGE, "gap-us-0..100000");
        return;
    }
    if (n == 0u) {
        bb_reply_err(BB_ERR_RANGE, "count-1..12");
        return;
    }

    uint32_t lo_a[12], lo_b[12], hi_a[12], hi_b[12];
    /* The divisor is empirical padding for the loop body, not a conversion:
     * the spin is a busy loop whose cost per pass is whatever the compiler
     * made of it. The gap only has to separate the edges enough to match them
     * up, so it is approximate on purpose and the reported brackets are what
     * any fit actually uses. */
    uint32_t spin = (gap * (bb_cycle_hz() / 1000000u)) / 12u;

    bb_pin_gpio(p, true, true, BB_PAD_PUSHPULL);
    /* Let the pad settle before the first bracket: the first edge after a mux
     * change is the one that would otherwise carry the mux in its timing. */
    for (volatile uint32_t d = 0; d < 1000u; d++) { }

    for (uint32_t i = 0; i < n; i++) {
        lo_a[i] = bb_cycles();
        bb_pin_write(p, false);
        lo_b[i] = bb_cycles();
        for (volatile uint32_t d = 0; d < spin; d++) { }

        hi_a[i] = bb_cycles();
        bb_pin_write(p, true);
        hi_b[i] = bb_cycles();
        for (volatile uint32_t d = 0; d < spin; d++) { }
    }

    /*
     * Falls and rises are reported separately rather than interleaved. They
     * are not equivalent markers: a fall is driven against the bus pull-up and
     * a rise is driven with it, so they do not have to arrive with the same
     * delay. Keeping them apart lets a fit use one, the other, or both - and
     * lets the two be compared, which is the only way to notice that they
     * differ.
     */
    bb_io_printf("OK n=%lu cycle_hz=%lu pin=%s", (unsigned long)n,
                 (unsigned long)bb_cycle_hz(), argv[1]);
    bb_io_puts(" fall=");
    for (uint32_t i = 0; i < n; i++) {
        bb_io_printf("%s%lu,%lu", i ? ";" : "",
                     (unsigned long)lo_a[i], (unsigned long)lo_b[i]);
    }
    bb_io_puts(" rise=");
    for (uint32_t i = 0; i < n; i++) {
        bb_io_printf("%s%lu,%lu", i ? ";" : "",
                     (unsigned long)hi_a[i], (unsigned long)hi_b[i]);
    }
    bb_io_puts("\r\n");
}

static void cmd_pulse(int argc, char **argv)
{
    uint32_t        n = 32u;
    const bb_pin_t *p = named_pin(argv[1]);

    if (p == NULL) {
        bb_io_printf("ERR %d no-such-signal try:", BB_ERR_RANGE);
        named_pin_list();
        bb_io_puts("\r\n");
        return;
    }
    if (argc > 2 && !bb_parse_u32_max(argv[2], 100000u, &n)) {
        bb_reply_err(BB_ERR_RANGE, "count-1..100000");
        return;
    }
    bb_pin_gpio(p, true, true, BB_PAD_PUSHPULL);
    for (uint32_t i = 0; i < n; i++) {
        bb_pin_write(p, false);
        for (volatile unsigned d = 0; d < 2000u; d++) { }
        bb_pin_write(p, true);
        for (volatile unsigned d = 0; d < 2000u; d++) { }
    }
    bb_reply_okf("pulsed=%lu", (unsigned long)n);
}

static const bb_pin_t *named_pin(const char *want);

static void cmd_pins(int argc, char **argv)
{
    unsigned            count = 0;
    const bb_pinmap_t  *map   = bb_pin_map(&count);
    const char         *want  = (argc > 1) ? argv[1] : NULL;
    unsigned            shown = 0;

    bb_io_puts("OK ");
    for (unsigned i = 0; i < count; i++) {
        /* Prefix match, so "i2c" gives every I2C bus and "i2c2" gives one.
         * Hand-rolled rather than strncasecmp, which is POSIX and not declared
         * by newlib-nano string.h. */
        if (want != NULL && !prefix_ci(map[i].signal, want)) {
            continue;
        }
        if (shown > 0u) {
            bb_io_puts(" ");
        }
        bb_io_puts(map[i].signal);
        bb_io_puts("=");
        bb_io_puts(map[i].pin);
        bb_io_puts("/");
        bb_io_puts(map[i].header);
        bb_io_puts("/");
        bb_io_puts(map[i].conn);
        /*
         * The live SW_PAD_CTL value, because the pad encoding is not guessable
         * from the intent. The two IOMUXC instances spell a pull-up with
         * different bits in the same position (see pad_encode() in bb_board.c),
         * so "is the pull-up actually on" was, until this was printed, only
         * answerable by reasoning about which branch a pad fell into - and that
         * reasoning was wrong for GPIO_AD for a long time.
         */
        const bb_pin_t *bp = named_pin(map[i].signal);
        if (bp != NULL && bb_pin_present(bp) && bp->cfg_reg != 0u) {
            bb_io_printf("/cfg=0x%02lX",
                         (unsigned long)(*(volatile uint32_t *)bp->cfg_reg & 0xFFu));
        }
        shown++;
    }
    if (shown == 0u) {
        bb_io_puts("none");
    }
    bb_io_puts("\r\n");
}

static void cmd_stat(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    /* twowire= is kept for the host library's bb_bus_owner(), and is now a
     * constant: D15/D14 carries I2C1 permanently since I3C1 moved to its own
     * pins.  The field survives so old hosts keep parsing; the value is simply
     * no longer interesting. */
    bb_reply_okf("uptime_ms=%lu rx_overruns=%lu twowire=i2c i2c_buses=%u",
                 (unsigned long)HAL_GetTick(),
                 (unsigned long)bb_console_rx_overruns(),
                 (unsigned)BB_I2C_BUS_COUNT);
}

static void cmd_button(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    bb_reply_okf("%d", bb_button_pressed() ? 1 : 0);
}

/** Report the native USB link, so it is obvious which transport is in use. */
static void cmd_usb_trace(int argc, char **argv)
{
    uint64_t ev = 0;
    uint32_t dev = 0, cfg = 0, state = 0;

    (void)argc;
    (void)argv;
    uint32_t vt = 0, vi = 0, vl = 0, vs = 0;

    bb_usb_trace(&ev, &dev, &cfg, &state);
    bb_usb_vendor_diag(&vt, &vi, &vl, &vs);
    bb_reply_okf("events=%08lX%08lX dev_desc=%lu cfg_desc=%lu state=%lu "
                 "vnd_type=0x%02lX vnd_index=%lu vnd_wlen=%lu vnd_sent=%lu "
                 "codes=1reset,2dev,3cfg,4qual,5str,6bos,7setcfg",
                 (unsigned long)(uint32_t)(ev >> 32),
                 (unsigned long)(uint32_t)(ev & 0xFFFFFFFFu),
                 (unsigned long)dev, (unsigned long)cfg,
                 (unsigned long)state,
                 (unsigned long)vt, (unsigned long)vi,
                 (unsigned long)vl, (unsigned long)vs);
}

static void cmd_usb(int argc, char **argv)
{
    uint32_t rx = 0;
    uint32_t tx = 0;
    uint32_t resets = 0;

    (void)argc;
    (void)argv;
    uint32_t bos = 0;
    uint32_t msos = 0;
    uint32_t stalls = 0;
    uint16_t last = 0;

    bb_usb_stats(&rx, &tx, &resets);
    uint16_t ep0_pkts = 0;
    uint16_t ep0_bytes = 0;

    bb_usb_diag(&bos, &msos, &stalls, &last);
    bb_usb_ep0_diag(&ep0_pkts, &ep0_bytes);
    uint32_t dppu = 0;
    uint32_t frame = 0;
    uint32_t istr = 0;

    bb_usb_link(&dppu, &frame, &istr);

    /* CC states, where the board has a Type-C port wired to UCPD. "-" means
     * there is nothing to measure, not that nothing is attached. */
    uint32_t    cc1 = 0;
    uint32_t    cc2 = 0;
    const char *cc_text[2] = { "-", "-" };
    static const char *const vstate[4] = { "0-open", "1", "2", "3" };

    if (bb_usb_typec(&cc1, &cc2)) {
        cc_text[0] = vstate[cc1 & 3u];
        cc_text[1] = vstate[cc2 & 3u];
    }

    /* dppu says this device is asking to be enumerated; frame says whether
     * anything is answering. Both zero with a cable plugged in means the data
     * lines are not reaching a host - which is what a charge-only cable looks
     * like, and no firmware change will fix that. */
    bb_reply_okf("state=%s transport=%s rx=%lu tx=%lu resets=%lu "
                 "bos=%lu msos=%lu stalls=%lu last_stall=0x%04X ep0=%up/%ub "
                 "dppu=%lu frame=%lu istr=0x%04lX cc1=%s cc2=%s",
                 bb_usb_ready() ? "configured" : "no-host",
                 bb_io_current() == BB_IO_USB ? "usb" : "vcp",
                 (unsigned long)rx, (unsigned long)tx, (unsigned long)resets,
                 (unsigned long)bos, (unsigned long)msos, (unsigned long)stalls,
                 last, ep0_pkts, ep0_bytes,
                 (unsigned long)dppu, (unsigned long)frame, (unsigned long)istr,
                 cc_text[0], cc_text[1]);
}

/* ---- the two resident images -------------------------------------------- */

/** One bank's line, as key=value so a host can parse what a person can read. */
static void print_bank(bb_bank_t which)
{
    bb_bank_info_t info;

    bb_image_bank(which, &info);

    if (!info.present) {
        bb_io_printf("# bank=%c running=%u address=0x%08lX state=empty\r\n",
                     which == BB_BANK_A ? 'A' : 'B',
                     info.running ? 1u : 0u,
                     (unsigned long)info.address);
        return;
    }

    bb_io_printf("# bank=%c running=%u address=0x%08lX state=%s board=%s "
                 "profile=%s version=%s built=%s size=%lu crc=0x%08lX "
                 "computed=0x%08lX crc_ok=%u dev_ok=%u\r\n",
                 which == BB_BANK_A ? 'A' : 'B',
                 info.running ? 1u : 0u,
                 (unsigned long)info.address,
                 info.image_size == 0xFFFFFFFFu ? "unstamped"
                     : (info.crc_ok ? "ok" : "bad"),
                 info.board[0] != '\0' ? info.board : "-",
                 info.profile[0] != '\0' ? info.profile : "-",
                 info.version[0] != '\0' ? info.version : "-",
                 info.built[0] != '\0' ? info.built : "-",
                 (unsigned long)info.image_size,
                 (unsigned long)info.image_crc,
                 (unsigned long)info.computed_crc,
                 info.crc_ok ? 1u : 0u,
                 info.dev_ok ? 1u : 0u);
}

/**
 * @brief SYS:BANK? - what is in both banks, and which one is running.
 *
 * Recomputes both checksums, so it is not free: a pass over each image, a few
 * milliseconds each. That is the point of it being a command someone asks for
 * rather than part of the status poll.
 */
static void cmd_bank(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    print_bank(BB_BANK_A);
    print_bank(BB_BANK_B);
    bb_reply_okf("active=%c bank_size=%lu swap=%u",
                 bb_image_running_bank() == BB_BANK_A ? 'A' : 'B',
                 (unsigned long)bb_image_bank_size(),
                 bb_image_swapped() ? 1u : 0u);
}

/** A|B, or the word the caller used if it was neither. */
static bool parse_bank(const char *word, bb_bank_t *out)
{
    if (bb_streq_ci(word, "A") || bb_streq_ci(word, "1")) {
        *out = BB_BANK_A;
        return true;
    }
    if (bb_streq_ci(word, "B") || bb_streq_ci(word, "2")) {
        *out = BB_BANK_B;
        return true;
    }
    return false;
}

/** SYS:BANK:VERIFY <A|B> - recompute one bank's checksum and say. */
static void cmd_bank_verify(int argc, char **argv)
{
    bb_bank_t      which;
    bb_bank_info_t info;

    (void)argc;
    if (!parse_bank(argv[1], &which)) {
        bb_reply_err(BB_ERR_RANGE, "bank-A-or-B");
        return;
    }

    bb_image_bank(which, &info);
    if (!info.present) {
        bb_reply_err(BB_ERR_BUS_STATE, "no-image-in-that-bank");
        return;
    }
    bb_reply_okf("bank=%c size=%lu crc=0x%08lX computed=0x%08lX ok=%u "
                 "profile=%s dev_ok=%u",
                 which == BB_BANK_A ? 'A' : 'B',
                 (unsigned long)info.image_size,
                 (unsigned long)info.image_crc,
                 (unsigned long)info.computed_crc,
                 info.crc_ok ? 1u : 0u,
                 info.profile[0] != '\0' ? info.profile : "-",
                 info.dev_ok ? 1u : 0u);
}

/**
 * @brief SYS:BANK:SWAP <A|B> - run the other image.
 *
 * The checksum is verified here and not taken from an earlier SYS:BANK:VERIFY:
 * between the two commands anything could have written that bank, and this is
 * the decision that matters.
 *
 * On success the board resets, so the reply goes out first - the same order
 * SYS:RESET uses, and for the same reason.
 */
static void cmd_bank_swap(int argc, char **argv)
{
    bb_bank_t        which;
    bb_bank_info_t   info;
    bb_swap_result_t result;

    (void)argc;
    if (!parse_bank(argv[1], &which)) {
        bb_reply_err(BB_ERR_RANGE, "bank-A-or-B");
        return;
    }

    /* Checked twice on purpose. bb_image_activate() verifies before it touches
     * an option byte, which is the check that counts; this one exists so the
     * refusal can carry the numbers that explain it. */
    bb_image_bank(which, &info);
    if (!info.present) {
        bb_reply_err(BB_ERR_BUS_STATE, "no-image-in-that-bank");
        return;
    }
    if (!info.crc_ok) {
        bb_reply_errf(BB_ERR_BUS_STATE,
                      "checksum-mismatch crc=0x%08lX computed=0x%08lX",
                      (unsigned long)info.image_crc,
                      (unsigned long)info.computed_crc);
        return;
    }
    if (!info.dev_ok) {
        bb_reply_err(BB_ERR_BUS_STATE, "image-built-for-another-part");
        return;
    }
    if (info.running) {
        bb_reply_err(BB_ERR_BUS_STATE, "already-running-that-bank");
        return;
    }

    /* Sent before the switch, because the switch resets the board and there
     * would be nothing left to answer with. */
    bb_reply_okf("switching to %c profile=%s crc=0x%08lX",
                 which == BB_BANK_A ? 'A' : 'B',
                 info.profile[0] != '\0' ? info.profile : "-",
                 (unsigned long)info.image_crc);
    HAL_Delay(50);                      /* let the reply drain out of the FIFO */

    result = bb_image_activate(which);
    /* Only reached when the option byte would not program - the reply has
     * already claimed success, so this goes to the console where a person
     * watching a bring-up will see it. */
    if (result != BB_SWAP_OK) {
        bb_console_printf("\r\n# bank switch failed: %s\r\n",
                          bb_swap_reason(result));
    }
}

static void cmd_reset(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    bb_reply_ok();
    HAL_Delay(50);                      /* let the reply drain out of the FIFO */
    NVIC_SystemReset();
}

/* ---- eDMA trigger-rate bench ------------------------------------------- */

/*
 * How fast can eDMA service one 32-bit GPIO capture per trigger?
 *
 * This is the number that decides whether a FlexIO-free sniffer is worth
 * building. The proposed design triggers a DMA channel from a GPIO edge and
 * copies RGPIO PDIR into a ring, which takes the core out of capture entirely -
 * but only if the engine can service a trigger every 80 ns at 12.5 MHz.
 *
 * Measured with minor-loop self-linking rather than a real trigger: the channel
 * links to itself, so one software start runs the whole major loop back to back
 * at whatever rate the engine manages, with no trigger source to build first.
 *
 * That makes this an UPPER BOUND and it must be read as one. A real GPIO
 * request adds pad synchronisation and request routing on top, so the true
 * figure is lower. The bound is still decisive in one direction: if the engine
 * cannot reach 12.5 M transfers/s with no trigger path at all, it certainly
 * cannot with one, and the idea is dead without wiring anything.
 *
 * The source really is RGPIO2 PDIR, not a RAM buffer, because reading a
 * peripheral across the bus is the transfer the design would actually perform.
 */
#define DMABENCH_CH      15u     /* high channels, away from anything in use */
#define DMABENCH_CH2     14u
#define DMABENCH_LOOPS   511u    /* CITER is 9 bits once ELINK is set */

/*
 * In OCRAM, through the SDK's own macro.
 *
 * Ordinary .bss on this image is CM33 TCM and the DMA cannot write there: the
 * channel did not run slowly or drop transfers, it failed outright with
 * CH_ES destination-bus-error and moved nothing. A bus error is not a
 * performance result.
 *
 * The macro rather than a hand-written section attribute, because the section
 * has to be NOBITS. Naming "NonCacheable" directly gives GCC a PROGBITS section,
 * so a zero-filled buffer became 2044 bytes of initialised data at an OCRAM
 * address, and the image then failed to program: "No flash configured at the
 * specified address". The macro carries the %nobits that avoids it.
 */
AT_NONCACHEABLE_SECTION_ALIGN(static uint32_t s_dmabench_buf[DMABENCH_LOOPS], 32);

/*
 * Arm a GPIO-edge-triggered DMA capture, then report what it caught.
 *
 * SYS:DMACAP ARM  configures RGPIO2 pin 14 - the d4 tap, which sits on the
 *                 I2C bus 2 clock line - to raise a DMA request on every edge,
 *                 and points a DMA channel at RGPIO2 PDIR.
 * SYS:DMACAP?     reports how many transfers the engine actually completed.
 *
 * This is the real mechanism the FlexIO-free sniffer would use, measured
 * against a bus whose edge count is known in advance rather than against a
 * synthetic trigger. Run an I2C transfer between the two and the completed
 * count is the answer: a six-byte write is 54 clocks, so 108 edges.
 *
 * Two synthetic attempts came first and neither measured the engine. Linking a
 * channel to itself, and then two channels linking to each other, both left the
 * channel with ACTIVE and DONE clear and no error - the TCD read back exactly
 * as written, so the writes landed and the engine simply declined to start.
 * Starting each minor loop from software did run, at 2.34 M transfers/s, but
 * with a register write and a poll per transfer that figure is the CPU's round
 * trip and not the DMA's rate.
 */
#define DMACAP_PIN   14u    /* RGPIO2 pin 14 = GPIO_EMC_B1_14 = tap d4 */

/* Which RGPIO2 pin the capture is armed on. d4 is 14 (the I2C bus 2 clock);
 * d8 is 17 (the I3C clock), which is the only tap that sees a bus fast enough
 * to matter. */
static uint8_t s_dmacap_pin = DMACAP_PIN;
#define DMACAP_MUX   2u     /* kDma4RequestMuxGPIO2Request0, low byte */

static void cmd_dmacap(int argc, char **argv)
{
    const volatile uint32_t *pdir = &RGPIO2->PDIR;

    if (argc > 1 && bb_streq_ci(argv[1], "ARM")) {
        uint32_t pin = s_dmacap_pin;

        if (argc > 2 && !bb_parse_u32_max(argv[2], 31u, &pin)) {
            bb_reply_err(BB_ERR_RANGE, "pin-0..31");
            return;
        }
        s_dmacap_pin = (uint8_t)pin;

        for (uint32_t i = 0; i < DMABENCH_LOOPS; i++) {
            s_dmabench_buf[i] = 0u;
        }

        /*
         * Silence the pin before touching the channel, and re-enable it last.
         *
         * Order matters here and it is not obvious. Re-arming while the pin
         * could still raise requests left the channel dead: a fresh arm
         * captured every edge, the next captured nothing, and the one after
         * captured again - a perfect alternation with the arm-time registers
         * byte-identical each time. Arming twice in a row failed every time,
         * which is what identified it: the damage is done by arming on top of
         * a live request source, not by anything the previous capture did.
         */
        RGPIO2->ICR[s_dmacap_pin] = 0u;                      /* requests off */
        RGPIO2->ICR[s_dmacap_pin] = RGPIO_ICR_ISF_MASK;      /* and flag clear */

        DMA4->TCD[DMABENCH_CH].CH_CSR = 0u;
        DMA4->TCD[DMABENCH_CH].CH_CSR = DMA4_CH_CSR_DONE_MASK;   /* w1c */
        DMA4->TCD[DMABENCH_CH].CH_CSR = 0u;
        DMA4->TCD[DMABENCH_CH].CH_INT = 1u;
        DMA4->TCD[DMABENCH_CH].CH_ES  = 0x80000000u;
        DMA4->TCD[DMABENCH_CH].CH_MUX = DMACAP_MUX;

        DMA4->TCD[DMABENCH_CH].SADDR          = (uint32_t)(uintptr_t)pdir;
        DMA4->TCD[DMABENCH_CH].SOFF           = 0;
        DMA4->TCD[DMABENCH_CH].ATTR           = (uint16_t)(DMA4_ATTR_SSIZE(2u) |
                                                           DMA4_ATTR_DSIZE(2u));
        DMA4->TCD[DMABENCH_CH].NBYTES_MLOFFNO = 4u;
        DMA4->TCD[DMABENCH_CH].SLAST_SDA      = 0;
        DMA4->TCD[DMABENCH_CH].DADDR          = (uint32_t)(uintptr_t)s_dmabench_buf;
        DMA4->TCD[DMABENCH_CH].DOFF           = 4;
        DMA4->TCD[DMABENCH_CH].DLAST_SGA      = 0;
        DMA4->TCD[DMABENCH_CH].CITER_ELINKNO  = (uint16_t)DMABENCH_LOOPS;
        DMA4->TCD[DMABENCH_CH].BITER_ELINKNO  = (uint16_t)DMABENCH_LOOPS;
        DMA4->TCD[DMABENCH_CH].CSR            = 0u;

        /* The pad has to be a GPIO for RGPIO to see it, and the tap is
         * otherwise parked as a GPIO input already - so only the interrupt
         * control needs setting. IRQC=0011: flag and DMA request on either
         * edge; IRQS=0 selects request 0, the one the mux above listens to. */
        /*
         * Clear the latched flag first, or the capture never starts.
         *
         * ISF is write-one-to-clear and it latches on the first qualifying
         * edge. If it is already set when the channel is armed, the pin never
         * raises another request and the DMA sits at zero transfers with no
         * error anywhere - the channel enabled, the mux right, the TCD right.
         * Armed twice in a row, the first run captured every edge and the
         * second captured none, with the only difference visible in ICR.
         */
        DMA4->TCD[DMABENCH_CH].CH_CSR = DMA4_CH_CSR_ERQ_MASK;

        /* Last, so the first request the channel ever sees belongs to the
         * capture that was just set up. */
        RGPIO2->ICR[s_dmacap_pin] = RGPIO_ICR_IRQC(3u) | RGPIO_ICR_IRQS(0u) |
                                  RGPIO_ICR_ISF_MASK;
        bb_reply_okf("armed pin=%u citer=%u icr=0x%08lX ch_csr=0x%08lX "
                     "hrs=0x%08lX",
                     (unsigned)s_dmacap_pin, (unsigned)DMABENCH_LOOPS,
                     (unsigned long)RGPIO2->ICR[s_dmacap_pin],
                     (unsigned long)DMA4->TCD[DMABENCH_CH].CH_CSR,
                     (unsigned long)DMA4->MP_HRS_LOW);
        return;
    }

    uint32_t left = DMA4->TCD[DMABENCH_CH].CITER_ELINKNO & 0x7FFFu;
    uint32_t got  = (left <= DMABENCH_LOOPS) ? (DMABENCH_LOOPS - left) : 0u;

    /*
     * Labelled, because a zero here is not a measurement of the bus.
     *
     * This command is the remains of the GPIO-plus-eDMA capture experiment,
     * which was measured at roughly a sixth of the rate FlexIO reaches and
     * rejected on that basis. It is kept because the negative result is worth
     * being able to reproduce, not because it works: as it stands the channel
     * arms - ERQ set, ICR programmed for a request per edge - and completes no
     * transfers at all, whatever order the pin and the channel are set up in.
     *
     * Without the label a reader sees `transfers=0` next to plausible register
     * values and concludes something about the pin or the bus. The honest
     * reading is that this path does not currently capture anything.
     */
    bb_reply_okf("transfers=%lu left=%lu ch_es=0x%08lX ch_csr=0x%08lX "
                 "icr=0x%08lX first=0x%08lX last=0x%08lX "
                 "note=retired-experiment-captures-nothing",
                 (unsigned long)got, (unsigned long)left,
                 (unsigned long)DMA4->TCD[DMABENCH_CH].CH_ES,
                 (unsigned long)DMA4->TCD[DMABENCH_CH].CH_CSR,
                 (unsigned long)RGPIO2->ICR[s_dmacap_pin],
                 (unsigned long)s_dmabench_buf[0],
                 (unsigned long)s_dmabench_buf[(got > 0u) ? (got - 1u) : 0u]);
}

const bb_cmd_t bb_cmd_sys[] = {
    { "*IDN?",     0, 0, cmd_idn,    "*IDN?" },
    { "HELP",      0, 0, cmd_help,   "HELP" },
    { "SYS:ID?",   0, 0, cmd_id,     "SYS:ID?" },
    { "SYS:CLK?",  0, 0, cmd_clk,    "SYS:CLK?" },
    { "SYS:LED",   2, 2, cmd_led,    "SYS:LED <n> <0|1|AUTO>" },
    { "SYS:ECHO",  1, 1, cmd_echo,   "SYS:ECHO <0|1>" },
    { "SYS:STAT?", 0, 0, cmd_stat,   "SYS:STAT?" },
    { "SYS:PINS?", 0, 1, cmd_pins,   "SYS:PINS? [signal-prefix]" },
    { "SYS:GPIO",  2, 2, cmd_gpio,   "SYS:GPIO <signal> <0|1>" },
    { "SYS:PULSE", 1, 2, cmd_pulse,  "SYS:PULSE <signal> [count]" },
    { "SYS:SYNC",  1, 3, cmd_sync,   "SYS:SYNC <signal> [count] [gap-us]" },
    { "SYS:BTN?",  0, 0, cmd_button, "SYS:BTN?" },
    { "SYS:USB?",  0, 0, cmd_usb,    "SYS:USB?" },
    { "SYS:USB:TRACE?", 0, 0, cmd_usb_trace, "SYS:USB:TRACE?" },
    { "SYS:BANK?", 0, 0, cmd_bank,   "SYS:BANK?" },
    { "SYS:BANK:VERIFY", 1, 1, cmd_bank_verify, "SYS:BANK:VERIFY <A|B>" },
    { "SYS:BANK:SWAP",   1, 1, cmd_bank_swap,   "SYS:BANK:SWAP <A|B>" },
    { "SYS:DMACAP",  0, 2, cmd_dmacap,  "SYS:DMACAP [ARM [rgpio2-pin]]" },
    { "SYS:RESET", 0, 0, cmd_reset,  "SYS:RESET" },
    { NULL, 0, 0, NULL, NULL }
};

void bb_cmd_init(void)
{
}
