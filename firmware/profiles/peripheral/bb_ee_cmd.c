/**
 * @file profiles/peripheral/bb_ee_cmd.c
 * @brief Host access to the EEPROM the buses share.
 *
 * The point of these commands is to be the fifth door, and the one that is
 * always trustworthy: seed the array from the host, let a controller write to it
 * over I3C, then read it back here. If the two disagree, the bus is at fault and
 * not the memory - which is a question the other four doors cannot settle
 * between themselves.
 */
#include "bb_cmd.h"
#include "bb_ee.h"
#include "bb_ee_ports.h"
#include "bb_io.h"

#include <stdio.h>
#include <string.h>

/** EE:RD <addr> [len] - read from the array. */
static void cmd_rd(int argc, char **argv)
{
    uint8_t  buf[BB_PAYLOAD_MAX];
    uint32_t addr;
    uint32_t len = 16u;

    if (!bb_parse_u32_max(argv[1], BB_EE_SIZE - 1u, &addr)) {
        bb_reply_errf(BB_ERR_RANGE, "addr-0..%u", (unsigned)(BB_EE_SIZE - 1u));
        return;
    }
    if (argc > 2 && !bb_parse_u32_max(argv[2], sizeof buf, &len)) {
        bb_reply_errf(BB_ERR_RANGE, "len-1..%u", (unsigned)sizeof buf);
        return;
    }
    if (len == 0u) {
        bb_reply_err(BB_ERR_RANGE, "len-1-or-more");
        return;
    }
    bb_ee_read_block(BB_EE_IF_HOST, (uint16_t)addr, buf, len);
    bb_reply_ok_bytes(buf, len);
}

/** EE:WR <addr> <hex...> - write to the array. */
static void cmd_wr(int argc, char **argv)
{
    uint8_t  buf[BB_PAYLOAD_MAX];
    uint32_t addr;
    size_t   len = 0;

    if (!bb_parse_u32_max(argv[1], BB_EE_SIZE - 1u, &addr)) {
        bb_reply_errf(BB_ERR_RANGE, "addr-0..%u", (unsigned)(BB_EE_SIZE - 1u));
        return;
    }
    if (!bb_parse_bytes(argc, argv, 2, buf, sizeof buf, &len) || len == 0u) {
        bb_reply_err(BB_ERR_NUMBER, "bad-hex-payload");
        return;
    }
    /* Checked here rather than only inside bb_ee_write(), which has no way to
     * answer a host. Reporting the byte count for a write the protect flag
     * refused would let a test that never reads back pass. */
    if (bb_ee_protected(BB_EE_IF_HOST)) {
        bb_reply_err(BB_ERR_BUS_STATE, "host-write-protected");
        return;
    }
    bb_ee_write_block(BB_EE_IF_HOST, (uint16_t)addr, buf, len);
    bb_reply_okf("wrote=%u addr=%lu", (unsigned)len, (unsigned long)addr);
}

/** EE:FILL <byte> | PATTERN - put the whole array in a known state. */
static void cmd_fill(int argc, char **argv)
{
    uint32_t value;

    (void)argc;
    /* Fill and pattern go straight to the array rather than through the
     * per-byte write, so the protect flag has to be honoured here too - a
     * protected host door that can still wipe all 1024 bytes protects
     * nothing. */
    if (bb_ee_protected(BB_EE_IF_HOST)) {
        bb_reply_err(BB_ERR_BUS_STATE, "host-write-protected");
        return;
    }
    if (bb_streq_ci(argv[1], "PATTERN")) {
        bb_ee_pattern();
        bb_reply_okf("pattern=ramp");
        return;
    }
    if (!bb_parse_u32_max(argv[1], 0xFFu, &value)) {
        bb_reply_err(BB_ERR_RANGE, "byte-0..255-or-PATTERN");
        return;
    }
    bb_ee_fill((uint8_t)value);
    bb_reply_okf("filled=0x%02lX", (unsigned long)value);
}

/** EE:SIZE? - how big the array is, so a host need not assume. */
static void cmd_size(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    bb_reply_okf("size=%u addr_bytes=2 endian=big",
                 (unsigned)BB_EE_SIZE);
}

/** EE:STAT? - who has touched it. */
static void cmd_stat(int argc, char **argv)
{
    char line[BB_CMD_MAX_LINE / 2];
    size_t used = 0;

    (void)argc;
    (void)argv;

    for (unsigned i = 0; i < BB_EE_IF_COUNT; i++) {
        const bb_ee_stats_t *s = bb_ee_stats((bb_ee_if_t)i);
        int n = snprintf(&line[used], sizeof line - used,
                         "%s%s=r%lu/w%lu/t%lu/x%lu@0x%03X",
                         used > 0u ? " " : "",
                         bb_ee_if_name((bb_ee_if_t)i),
                         (unsigned long)s->reads,
                         (unsigned long)s->writes,
                         (unsigned long)s->transactions,
                         (unsigned long)s->rejected,
                         s->last_addr);
        if (n <= 0 || (size_t)n >= sizeof line - used) {
            break;
        }
        used += (size_t)n;
    }
    bb_reply_okf("%s", line);
}

/** EE:CLEAR - zero the counters, not the data. */
static void cmd_clear(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    bb_ee_clear_stats();
    bb_reply_ok();
}

/**
 * @brief EE:PROTECT <iface> <0|1> - make one door read-only.
 *
 * For proving a path really is read-only, and for finding out which bus wrote a
 * byte when two of them could have.
 */
static void cmd_protect(int argc, char **argv)
{
    uint32_t on;

    (void)argc;
    for (unsigned i = 0; i < BB_EE_IF_COUNT; i++) {
        if (bb_streq_ci(argv[1], bb_ee_if_name((bb_ee_if_t)i))) {
            if (!bb_parse_u32_max(argv[2], 1u, &on)) {
                bb_reply_err(BB_ERR_RANGE, "state-0-or-1");
                return;
            }
            bb_ee_protect((bb_ee_if_t)i, on != 0u);
            bb_reply_okf("%s protected=%lu", bb_ee_if_name((bb_ee_if_t)i),
                         (unsigned long)on);
            return;
        }
    }
    bb_reply_err(BB_ERR_ARGC, "iface-host-i2c-i3c-spi-mdio");
}

/** EE:PORTS? - what each door is configured as, in one line per bus. */
static void cmd_ports(int argc, char **argv)
{
    char detail[80];

    (void)argc;
    (void)argv;

    bb_ee_i2c_describe(detail, sizeof detail);
    bb_io_printf("# i2c  %s\r\n", detail);
    bb_ee_i3c_describe(detail, sizeof detail);
    bb_io_printf("# i3c  %s\r\n", detail);
    bb_ee_spi_describe(detail, sizeof detail);
    bb_io_printf("# spi  %s\r\n", detail);
    bb_ee_mdio_describe(detail, sizeof detail);
    bb_io_printf("# mdio %s\r\n", detail);
    bb_reply_okf("size=%u", (unsigned)BB_EE_SIZE);
}

const bb_cmd_t bb_cmd_ee[] = {
    { "EE:SIZE?",   0, 0,    cmd_size,    "EE:SIZE?" },
    { "EE:RD",      1, 2,    cmd_rd,      "EE:RD <addr> [len]" },
    { "EE:WR",      2, 0xFF, cmd_wr,      "EE:WR <addr> <hex...>" },
    { "EE:FILL",    1, 1,    cmd_fill,    "EE:FILL <byte>|PATTERN" },
    { "EE:STAT?",   0, 0,    cmd_stat,    "EE:STAT?" },
    { "EE:CLEAR",   0, 0,    cmd_clear,   "EE:CLEAR" },
    { "EE:PROTECT", 2, 2,    cmd_protect, "EE:PROTECT <iface> <0|1>" },
    { "EE:PORTS?",  0, 0,    cmd_ports,   "EE:PORTS?" },
    { NULL, 0, 0, NULL, NULL },
};
