/**
 * @file bb_lcd_cmd.c
 * @brief Display commands, on every board.
 *
 * Separate from bb_lcd.c so the command table exists whether or not the board
 * has a panel: a host can ask any board about its display and get a clear
 * answer rather than "unknown command", which is indistinguishable from talking
 * to older firmware.
 */
#include "bb_cmd.h"
#include "bb_board.h"
#include "bb_lcd.h"

/* ---- display ----------------------------------------------------------- */
#if BB_HAS_DISPLAY

static void cmd_lcd_stat(int argc, char **argv)
{
    uint16_t id[3] = {0};
    unsigned n;

    (void)argc;
    (void)argv;
    n = bb_lcd_read_id(id, 3u);
    bb_reply_okf("ready=%d %ux%u id=%04X%04X%04X words=%u",
                 bb_lcd_ready() ? 1 : 0,
                 (unsigned)BB_LCD_WIDTH, (unsigned)BB_LCD_HEIGHT,
                 id[0], id[1], id[2], n);
}

static void cmd_lcd_test(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    if (!bb_lcd_ready()) {
        bb_reply_err(BB_ERR_UNSUPPORTED, "no-display");
        return;
    }
    bb_lcd_test_pattern();
    bb_reply_ok();
}

static void cmd_lcd_refresh(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    if (!bb_lcd_ready()) {
        bb_reply_err(BB_ERR_UNSUPPORTED, "no-display");
        return;
    }
    bb_lcd_tick();
    bb_reply_ok();
}

/**
 * @brief Rotate the panel, 0..3 quarter turns.
 *
 * Which one is right depends on how the glass is mounted, so it is chosen here
 * rather than compiled in. Try them until the text reads left to right.
 */
static void cmd_lcd_rot(int argc, char **argv)
{
    uint32_t quarter;

    (void)argc;
    if (!bb_parse_u32_max(argv[1], 3, &quarter)) {
        bb_reply_err(BB_ERR_RANGE, "rotation-0..3");
        return;
    }
    if (!bb_lcd_ready()) {
        bb_reply_err(BB_ERR_UNSUPPORTED, "no-display");
        return;
    }
    bb_reply_okf("rot=%lu madctl=0x%02X", (unsigned long)quarter,
                 bb_lcd_set_rotation((unsigned)quarter));
}

const bb_cmd_t bb_cmd_lcd[] = {
    { "LCD:STAT?",   0, 0, cmd_lcd_stat,    "LCD:STAT?" },
    { "LCD:ROT",     1, 1, cmd_lcd_rot,     "LCD:ROT <0-3>" },
    { "LCD:TEST",    0, 0, cmd_lcd_test,    "LCD:TEST" },
    { "LCD:REFRESH", 0, 0, cmd_lcd_refresh, "LCD:REFRESH" },
    { NULL, 0, 0, NULL, NULL }
};

#else

/* A board with no panel still answers, so a host does not have to know which
 * board it is talking to before asking. */
static void cmd_lcd_none(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    bb_reply_err(BB_ERR_UNSUPPORTED, "no-display-on-this-board");
}

const bb_cmd_t bb_cmd_lcd[] = {
    { "LCD:STAT?",   0, 0, cmd_lcd_none, "LCD:STAT? (no display)" },
    { "LCD:ROT",     1, 1, cmd_lcd_none, "LCD:ROT (no display)" },
    { "LCD:TEST",    0, 0, cmd_lcd_none, "LCD:TEST (no display)" },
    { "LCD:REFRESH", 0, 0, cmd_lcd_none, "LCD:REFRESH (no display)" },
    { NULL, 0, 0, NULL, NULL }
};

#endif
