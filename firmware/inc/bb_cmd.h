/**
 * @file bb_cmd.h
 * @brief ASCII command protocol: tokeniser, dispatch table and reply helpers.
 *
 * One command per line, case-insensitive, colon-separated namespace:
 *
 *     I2C:WR 0x50 2 00 10          ->  OK 4B 5A
 *     ERR 5 nack-addr
 *
 * Every reply is exactly one line beginning with "OK" or "ERR <code> <text>",
 * which makes the host side a trivial state machine.
 */
#ifndef BB_CMD_H
#define BB_CMD_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define BB_PAYLOAD_MAX   256u   /* bytes per single bus transfer */

/* A full 256-byte payload is 512 characters as one contiguous hex token, plus
 * the command word and its arguments - hence the line budget below.  The host
 * library sends large payloads in that compact form for exactly this reason.
 * Space-separated bytes are still accepted, up to BB_CMD_MAX_ARGS tokens. */
#define BB_CMD_MAX_LINE  1100u  /* console line buffer, incl. NUL */
#define BB_CMD_MAX_ARGS    80u

/* Error codes shared with the host library (see host/busbridge/client.py). */
#define BB_ERR_UNKNOWN_CMD   1
#define BB_ERR_ARGC          2
#define BB_ERR_NUMBER        3
#define BB_ERR_BUS_STATE     4
#define BB_ERR_NACK          5
#define BB_ERR_TIMEOUT       6
#define BB_ERR_HAL           7
#define BB_ERR_UNSUPPORTED   8
#define BB_ERR_TOO_LONG      9
#define BB_ERR_RANGE        10

typedef struct {
    const char *name;                       /**< full command word, upper case */
    uint8_t     min_args;
    uint8_t     max_args;                   /**< 0xFF = variadic */
    void      (*fn)(int argc, char **argv); /**< argv[0] is the command word */
    const char *usage;
} bb_cmd_t;

void bb_cmd_init(void);

/** Feed one complete line (without newline). Emits exactly one reply line. */
void bb_cmd_execute(char *line);

/* ---- reply helpers ----------------------------------------------------- */

/**
 * The most bytes one `bb_reply_ok_bytes` line may carry.
 *
 * A reply line has a hard length limit, and it is not this firmware's: the
 * host library reads a line into a 1200-byte buffer (BB_REPLY_MAX there). Each
 * byte costs three characters here - a space and two hex digits - after a
 * two-character "OK", so (1200 - 1 - 2) / 3 = 399 bytes is what fits.
 *
 * A command that offers more does not get a long line, it gets a cut one, and
 * for a command that advances a cursor - SNIFF:RAW:READ? is the one that does -
 * the bytes beyond the cut are not re-readable. They are gone. So the bound
 * belongs on what a command is willing to *promise*, not on what it prints.
 *
 * Stated as a number rather than derived from the host's constant because the
 * two live in different repositories; if that one ever grows, this is the line
 * to change and the comment that says why.
 */
#define BB_REPLY_BYTES_MAX  399u

/**
 * The longest reply line a command may build, in characters.
 *
 * A companion to BB_REPLY_BYTES_MAX above, for the commands that print tokens
 * rather than hex bytes. 1152 rather than the host's 1200 so the header, the
 * terminator and a partly-formatted token at the end all have room without
 * arithmetic at each call site.
 *
 * Note what this does NOT explain. SNIFF:TRACE? used to take the board down
 * at about fifty events - it would enumerate afterwards and refuse to open -
 * and the cause was a stack overflow in the peek that fed it, not the reply
 * length. That is written up at bb_trace_peek(). This bound was added while
 * chasing the wrong suspect and kept because it is worth having on its own
 * terms: a command that promises more than a line holds hands back a cut one,
 * and for a command that advances a cursor the cut part is gone.
 */
#define BB_REPLY_TEXT_MAX  1152u

void bb_reply_ok(void);
void bb_reply_okf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

/**
 * Reply with `len` bytes as hex.
 *
 * `len` must not exceed BB_REPLY_BYTES_MAX; more than that is reported as an
 * error instead of being sent, because a caller that overruns the line would
 * otherwise have its tail silently removed in transit.
 */
void bb_reply_ok_bytes(const uint8_t *data, size_t len);
void bb_reply_err(int code, const char *text);
void bb_reply_errf(int code, const char *fmt, ...) __attribute__((format(printf, 2, 3)));

/** Map a HAL status onto an error reply. Returns true if @p st was HAL_OK. */
bool bb_reply_hal(int st_hal, const char *what);

/* ---- argument parsing -------------------------------------------------- */
/** Accepts 0x/0X hex, 0b binary, or decimal. */
bool bb_parse_u32(const char *s, uint32_t *out);
bool bb_parse_u32_max(const char *s, uint32_t max, uint32_t *out);

/**
 * @brief Parse argv[first..argc-1] as a hex byte stream.
 *
 * Each token is an even number of hex digits with an optional 0x prefix, so
 * "DEADBEEF", "DE AD BE EF" and "0xDE 0xAD" are all four... two, four bytes
 * respectively.  Returns false on a malformed token or on overflow of @p cap.
 */
bool bb_parse_bytes(int argc, char **argv, int first, uint8_t *out, size_t cap, size_t *len);

/** Case-insensitive whole-token compare. */
bool bb_streq_ci(const char *a, const char *b);

/* Command tables contributed by the bus modules. */
extern const bb_cmd_t bb_cmd_sys[];
extern const bb_cmd_t bb_cmd_i2c[];
extern const bb_cmd_t bb_cmd_i3c[];
extern const bb_cmd_t bb_cmd_spi[];
extern const bb_cmd_t bb_cmd_mdio[];
extern const bb_cmd_t bb_cmd_uart[];
/* New on this board: the STM32 had no CAN transceiver to drive. */
extern const bb_cmd_t bb_cmd_can[];
extern const bb_cmd_t bb_cmd_ee[];
/* Phase 6's bounded slice: the on-board PHYs, which only NETC's MDIO reaches. */
extern const bb_cmd_t bb_cmd_net[];
extern const bb_cmd_t bb_cmd_sniff[];

#endif /* BB_CMD_H */
