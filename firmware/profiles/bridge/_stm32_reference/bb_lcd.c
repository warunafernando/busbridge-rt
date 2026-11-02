/**
 * @file bb_lcd.c
 * @brief Status display on the board's own panel: an ST7789V over the FMC.
 *
 * # Why the FMC
 *
 * The panel is an 8080-style parallel display: a 16-bit data bus, a write
 * strobe, a chip select, and one address line that says whether the word is a
 * command or a pixel. The FMC drives exactly that when configured as SRAM, so
 * writing a pixel is a store to a memory address and needs no driver loop at
 * all. Command and data differ only in address bit 1, because A0 of the panel
 * is wired to FMC_A0 and the bus is 16 bits wide.
 *
 * # What it shows, and why not more
 *
 * The same facts the GUI's monitor shows: transport, every bus with its pins
 * and clock, the sniffer, and the last command. Deliberately text only. A
 * 240x240 panel at 16 bits is 115 KB per full repaint, and this firmware's job
 * is to service buses with microsecond deadlines - so the display only ever
 * redraws fields whose text actually changed, and the whole thing is skipped
 * entirely while a bus transaction is in flight.
 *
 * # Timings
 *
 * Conservative on purpose. The ST7789V wants roughly 66 ns of write cycle and
 * the FMC is clocked from a 250 MHz HCLK, so the setup and hold numbers below
 * are several times the minimum. A display that works is worth more here than
 * one that refreshes fast, and nothing about this screen is latency-critical.
 */
#include "bb_lcd.h"
#include "bb_board.h"

#if BB_HAS_DISPLAY

#include "bb_lcd_font.h"
#include "bb_status.h"
#include "bb_usb.h"
#include <stdio.h>
#include <string.h>

/* ---- panel access ------------------------------------------------------- */

#define LCD_CMD  (*(volatile uint16_t *)BB_LCD_CMD_ADDR)
#define LCD_DAT  (*(volatile uint16_t *)BB_LCD_DATA_ADDR)

/* RGB565. Chosen to read well on a small panel in a lit room rather than to
 * match the GUI exactly - a dim grey that works on a monitor is invisible
 * here. */
#define C_BG       0x0000u   /* black                */
#define C_FRAME    0x2124u   /* dark grey rules      */
#define C_TITLE    0xFFFFu   /* white                */
#define C_LABEL    0x8410u   /* mid grey             */
#define C_VALUE    0xFFFFu   /* white                */
#define C_ACCENT   0x2D7Fu   /* blue, matches the GUI accent */
#define C_GOOD     0x4E68u   /* green                */
#define C_BAD      0xE1C3u   /* orange-red           */
#define C_IDLE     0x5AEBu   /* grey-blue            */

static SRAM_HandleTypeDef hsram;
static bool ready;
static bool dirty;      /* something non-text was drawn over the screen */

/* MADCTL. The panel is mounted a quarter turn from the controller's default, so
 * 0x00 renders every line bottom-to-top. MV swaps rows and columns and MX
 * mirrors, which together give text running left to right on this board.
 *
 * Settable at run time through LCD:ROT: which of the four is correct is a fact
 * about how the glass is glued down, and finding it by rebuilding and
 * reflashing four times is a poor use of anyone's afternoon.
 */
#define MADCTL_MY  0x80u
#define MADCTL_MX  0x40u
#define MADCTL_MV  0x20u

static uint8_t madctl = MADCTL_MV | MADCTL_MX;

static void repaint_frame(void);

/* ST7789V commands used here. */
#define ST_SWRESET 0x01u
#define ST_SLPOUT  0x11u
#define ST_INVON   0x21u
#define ST_DISPON  0x29u
#define ST_CASET   0x2Au
#define ST_RASET   0x2Bu
#define ST_RAMWR   0x2Cu
#define ST_MADCTL  0x36u
#define ST_COLMOD  0x3Au

static void wr_cmd(uint16_t c)
{
    LCD_CMD = c;
}

static void wr_dat(uint16_t d)
{
    LCD_DAT = d;
}

/** Address window. Inclusive coordinates, as the controller expects. */
static void window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    wr_cmd(ST_CASET);
    wr_dat((uint16_t)(x0 >> 8)); wr_dat((uint16_t)(x0 & 0xFFu));
    wr_dat((uint16_t)(x1 >> 8)); wr_dat((uint16_t)(x1 & 0xFFu));
    wr_cmd(ST_RASET);
    wr_dat((uint16_t)(y0 >> 8)); wr_dat((uint16_t)(y0 & 0xFFu));
    wr_dat((uint16_t)(y1 >> 8)); wr_dat((uint16_t)(y1 & 0xFFu));
    wr_cmd(ST_RAMWR);
}

static void fill(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t colour)
{
    if (w == 0u || h == 0u) {
        return;
    }
    window(x, y, (uint16_t)(x + w - 1u), (uint16_t)(y + h - 1u));
    for (uint32_t i = 0; i < (uint32_t)w * h; i++) {
        wr_dat(colour);
    }
}

/**
 * @brief Text scale.
 *
 * This panel is 240 pixels across 39 mm, so a 6x8 glyph is about 1.3 mm tall -
 * present on the screen and not readable by a person standing over the bench,
 * which is the only reason the screen exists. At 2x it is 2.6 mm and legible,
 * at the cost of 20 columns by 15 rows to lay out in.
 */
#define SCALE     2u
#define CELL_W    (BB_FONT_W * SCALE)
#define CELL_H    (BB_FONT_H * SCALE)
#define COLS      (BB_LCD_WIDTH / CELL_W)    /* 20 */
#define ROWS      (BB_LCD_HEIGHT / CELL_H)   /* 15 */

/**
 * @brief Draw one glyph at SCALE, background included.
 *
 * Background is painted as part of the same window rather than cleared first:
 * two passes over the same pixels doubles the transfers and shows as a visible
 * flicker on every update.
 */
static void glyph(uint16_t x, uint16_t y, char ch, uint16_t fg, uint16_t bg)
{
    unsigned index;

    if ((unsigned char)ch < BB_FONT_FIRST || (unsigned char)ch > BB_FONT_LAST) {
        ch = '?';
    }
    index = (unsigned)((unsigned char)ch - BB_FONT_FIRST);

    window(x, y, (uint16_t)(x + CELL_W - 1u), (uint16_t)(y + CELL_H - 1u));
    for (unsigned row = 0; row < BB_FONT_H; row++) {
        for (unsigned ry = 0; ry < SCALE; ry++) {
            for (unsigned col = 0; col < BB_FONT_W; col++) {
                bool on = (bb_font6x8[index][col] >> row) & 1u;
                for (unsigned rx = 0; rx < SCALE; rx++) {
                    wr_dat(on ? fg : bg);
                }
            }
        }
    }
}

/** Text at a character cell. Returns the column after the last one drawn. */
static unsigned text_at(unsigned col, unsigned row, const char *s,
                        uint16_t fg, uint16_t bg)
{
    uint16_t x = (uint16_t)(col * CELL_W);
    uint16_t y = (uint16_t)(row * CELL_H);

    while (*s != '\0' && x + CELL_W <= BB_LCD_WIDTH) {
        glyph(x, y, *s++, fg, bg);
        x = (uint16_t)(x + CELL_W);
        col++;
    }
    return col;
}

/** Text padded to @p width cells, so a shorter value erases a longer one. */
static void field(unsigned col, unsigned row, unsigned width, const char *s,
                  uint16_t fg)
{
    unsigned end = text_at(col, row, s, fg, C_BG);

    if (end < col + width) {
        fill((uint16_t)(end * CELL_W), (uint16_t)(row * CELL_H),
             (uint16_t)((col + width - end) * CELL_W), CELL_H, C_BG);
    }
}

/* ---- bring-up ----------------------------------------------------------- */

/**
 * @brief Configure the FMC for the panel and run the ST7789V init sequence.
 *
 * The pin list comes from the board: twenty pins for the 16-bit bus plus A0,
 * NE1, NWE and NOE. They are set up here rather than in the board file because
 * nothing else on the board can use them - claiming them is part of having a
 * display at all.
 */
static void fmc_pins(void)
{
    GPIO_InitTypeDef g = {0};

    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF12_FMC;

    g.Pin = GPIO_PIN_0;                                     /* A0 */
    HAL_GPIO_Init(GPIOF, &g);
    g.Pin = GPIO_PIN_7;                                     /* NE1 */
    HAL_GPIO_Init(GPIOC, &g);

    /* NWE PD5, NOE PD4, and data lines D0 PD14, D1 PD15, D2 PD0, D3 PD1,
     * D13 PD8, D14 PD9, D15 PD10. */
    g.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_0 | GPIO_PIN_1 |
            GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_14 | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOD, &g);

    /* D4..D12 are PE7..PE15. */
    g.Pin = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 |
            GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOE, &g);
}

static bool fmc_init(void)
{
    FMC_NORSRAM_TimingTypeDef timing = {0};

    __HAL_RCC_FMC_CLK_ENABLE();
    fmc_pins();

    hsram.Instance  = FMC_NORSRAM_DEVICE;
    hsram.Extended  = FMC_NORSRAM_EXTENDED_DEVICE;

    hsram.Init.NSBank             = FMC_NORSRAM_BANK1;
    hsram.Init.DataAddressMux     = FMC_DATA_ADDRESS_MUX_DISABLE;
    hsram.Init.MemoryType         = FMC_MEMORY_TYPE_SRAM;
    hsram.Init.MemoryDataWidth    = FMC_NORSRAM_MEM_BUS_WIDTH_16;
    hsram.Init.BurstAccessMode    = FMC_BURST_ACCESS_MODE_DISABLE;
    hsram.Init.WaitSignalPolarity = FMC_WAIT_SIGNAL_POLARITY_LOW;
    hsram.Init.WaitSignalActive   = FMC_WAIT_TIMING_BEFORE_WS;
    hsram.Init.WriteOperation     = FMC_WRITE_OPERATION_ENABLE;
    hsram.Init.WaitSignal         = FMC_WAIT_SIGNAL_DISABLE;
    hsram.Init.ExtendedMode       = FMC_EXTENDED_MODE_DISABLE;
    hsram.Init.AsynchronousWait   = FMC_ASYNCHRONOUS_WAIT_DISABLE;
    hsram.Init.WriteBurst         = FMC_WRITE_BURST_DISABLE;
    hsram.Init.ContinuousClock    = FMC_CONTINUOUS_CLOCK_SYNC_ONLY;
    hsram.Init.PageSize           = FMC_PAGE_SIZE_NONE;

    /* HCLK is 4 ns. These are in HCLK cycles and are several times the
     * panel's minimum: an ST7789V wants about 66 ns of write cycle, and the
     * cost of being generous is refresh rate on a status screen. */
    timing.AddressSetupTime      = 8;
    timing.AddressHoldTime       = 15;
    timing.DataSetupTime         = 24;
    timing.BusTurnAroundDuration = 4;
    timing.CLKDivision           = 16;
    timing.DataLatency           = 17;
    timing.AccessMode            = FMC_ACCESS_MODE_A;

    return HAL_SRAM_Init(&hsram, &timing, NULL) == HAL_OK;
}

static void panel_init(void)
{
    /* The board has already released reset and the power rail; this is the
     * controller's own sequence. */
    wr_cmd(ST_SWRESET);
    HAL_Delay(150);
    wr_cmd(ST_SLPOUT);
    HAL_Delay(120);

    wr_cmd(ST_COLMOD);
    wr_dat(0x55u);                  /* 16 bits per pixel, RGB565 */
    HAL_Delay(10);

    wr_cmd(ST_MADCTL);
    wr_dat(madctl);

    /* This panel is the inverted variety: without INVON every colour comes out
     * as its complement, which looks like a broken driver rather than a
     * one-command difference. */
    wr_cmd(ST_INVON);
    HAL_Delay(10);
    wr_cmd(ST_DISPON);
    HAL_Delay(120);
}

/* ---- the screen --------------------------------------------------------- */

/* Rows are character cells, 8 pixels tall: 30 of them on a 240 pixel panel. */
/* Fifteen rows at 2x, so the layout is tight and every line has to earn its
 * place. Uptime and the command counters share a row; the last command shares
 * with its result. The separate rules and the reply row that the 30-row layout
 * could afford are gone. */
#define ROW_TITLE     0
#define ROW_BOARD     1
#define ROW_LINK      2
#define ROW_STATS     3
#define ROW_BUS_FIRST 4      /* four I2C, then I3C, SPI, MDIO, UART, sniffer */
#define ROW_LAST_CMD  13

#define VALUE_COL     5
#define VALUE_W       15

/* What is currently on the panel, so only changes are redrawn. A full repaint
 * is 115 KB of transfers; a changed field is a few hundred bytes. */
/* What is currently on the panel, so only changes are redrawn. A full repaint
 * is 115 KB of transfers; a changed field is a few hundred bytes. */
static char shown[ROWS][COLS + 1];

static void put(unsigned row, unsigned col, unsigned width, const char *text,
                uint16_t colour)
{
    if (row >= ROWS) {
        return;
    }
    if (strncmp(shown[row], text, sizeof shown[0] - 1u) == 0) {
        return;                     /* unchanged: touch nothing */
    }
    snprintf(shown[row], sizeof shown[0], "%s", text);
    field(col, row, width, text, colour);
}

/** One colour per state, so the screen reads without labels. */
static uint16_t state_colour(bb_state_t state)
{
    switch (state) {
    case BB_STATE_ACTIVE:  return C_GOOD;
    case BB_STATE_PROBLEM: return C_BAD;
    case BB_STATE_ABSENT:  return C_IDLE;
    default:               return C_VALUE;
    }
}

static void rule(unsigned row)
{
    fill(0, (uint16_t)(row * CELL_H + CELL_H - 2u), BB_LCD_WIDTH, 1, C_FRAME);
}

/* ---- state the display reports ----------------------------------------- */

static uint32_t cmd_count;
static uint32_t err_count;
static char     last_cmd[40];
static char     last_ok;

void bb_lcd_note_command(const char *line, bool ok)
{
    cmd_count++;
    if (!ok) {
        err_count++;
    }
    snprintf(last_cmd, sizeof last_cmd, "%s", (line != NULL) ? line : "");
    last_ok = ok ? 1 : 0;
}

bool bb_lcd_ready(void)
{
    return ready;
}

unsigned bb_lcd_read_id(uint16_t *words, unsigned max)
{
    /* 0x04 is read-display-ID. The first word back is a dummy: the controller
     * needs one read cycle to turn the bus around. */
    if (!ready || words == NULL || max == 0u) {
        return 0;
    }
    wr_cmd(0x04u);
    (void)LCD_DAT;
    unsigned n = 0;
    while (n < max && n < 3u) {
        words[n++] = LCD_DAT;
    }
    return n;
}

void bb_lcd_test_pattern(void)
{
    if (!ready) {
        return;
    }
    /* Four quadrants plus a white border: wrong colours, a wrong rotation or a
     * dead half of the data bus all show up immediately in this. */
    static const uint16_t quad[4] = { C_BAD, C_GOOD, C_ACCENT, C_TITLE };

    for (unsigned i = 0; i < 4u; i++) {
        uint16_t x = (uint16_t)((i & 1u) ? BB_LCD_WIDTH / 2u : 0u);
        uint16_t y = (uint16_t)((i & 2u) ? BB_LCD_HEIGHT / 2u : 0u);
        fill(x, y, BB_LCD_WIDTH / 2u, BB_LCD_HEIGHT / 2u, quad[i]);
    }
    fill(0, 0, BB_LCD_WIDTH, 2, C_TITLE);
    fill(0, BB_LCD_HEIGHT - 2u, BB_LCD_WIDTH, 2, C_TITLE);
    fill(0, 0, 2, BB_LCD_HEIGHT, C_TITLE);
    fill(BB_LCD_WIDTH - 2u, 0, 2, BB_LCD_HEIGHT, C_TITLE);
    text_at(2, 14, "LCD TEST PATTERN", C_BG, C_TITLE);

    /* The status screen owns the panel. Forgetting the text is not enough -
     * the quadrants stay wherever no text lands - so the next refresh clears
     * the whole screen first. */
    memset(shown, 0, sizeof shown);
    dirty = true;
}

void bb_lcd_init(void)
{
    if (!fmc_init()) {
        return;
    }
    panel_init();

    repaint_frame();

    /* Backlight last: nothing is shown until there is something to see. */
    HAL_GPIO_WritePin(BB_LCD_BACKLIGHT_PORT, BB_LCD_BACKLIGHT_PIN, GPIO_PIN_SET);
    ready = true;
}

/** The parts that never change. Also the recovery path after a test pattern. */
static void repaint_frame(void)
{
    fill(0, 0, BB_LCD_WIDTH, BB_LCD_HEIGHT, C_BG);
    memset(shown, 0, sizeof shown);

    text_at(0, ROW_TITLE, "BUSBRIDGE", C_ACCENT, C_BG);
    /* "STM32H573I-DK" is 13 characters and the row is 20 wide, so it fits; a
     * longer board name would be clipped rather than wrapped, which is the
     * right trade on a status screen. */
    text_at(0, ROW_BOARD, BB_BOARD_NAME, C_LABEL, C_BG);
    rule(ROW_BOARD);
    rule(ROW_STATS);

    text_at(0, ROW_LINK, "link", C_LABEL, C_BG);
    dirty = false;
}

uint8_t bb_lcd_set_rotation(unsigned quarter)
{
    static const uint8_t table[4] = {
        0u,                                  /* 0: controller default      */
        MADCTL_MV | MADCTL_MX,               /* 1: quarter turn            */
        MADCTL_MY | MADCTL_MX,               /* 2: half turn               */
        MADCTL_MV | MADCTL_MY,               /* 3: three quarters          */
    };

    madctl = table[quarter & 3u];
    if (ready) {
        wr_cmd(ST_MADCTL);
        wr_dat(madctl);
        repaint_frame();
    }
    return madctl;
}

void bb_lcd_tick(void)
{
    char text[COLS + 1];

    if (!ready) {
        return;
    }
    if (dirty) {
        repaint_frame();
    }

    put(ROW_LINK, VALUE_COL, VALUE_W,
        bb_usb_ready() ? "USB+VCP" : "VCP only", C_VALUE);

    /* Uptime and the counters share a row: at 20 columns a line each would cost
     * two of the fifteen for three numbers. */
    snprintf(text, sizeof text, "%lus c%lu e%lu",
             (unsigned long)(HAL_GetTick() / 1000u),
             (unsigned long)cmd_count, (unsigned long)err_count);
    put(ROW_STATS, 0, COLS, text, (err_count > 0u) ? C_BAD : C_VALUE);

    /* The buses, as the modules describe themselves. One line each, fixed
     * width, so the layout never moves when a bus changes state - a screen
     * whose rows shuffle is hard to read at a glance. */
    bb_status_t st;

    for (unsigned i = 0; i < BB_I2C_BUS_COUNT; i++) {
        bb_i2c_status(i + 1u, &st);
        snprintf(text, sizeof text, "I2C%u %-9s %s", i + 1u, st.pins, st.detail);
        put(ROW_BUS_FIRST + i, 0, COLS, text, state_colour(st.state));
    }

    bb_i3c_status(&st);
    snprintf(text, sizeof text, "I3C  %-9s %s", st.pins, st.detail);
    put(ROW_BUS_FIRST + 4, 0, COLS, text, state_colour(st.state));

    bb_spi_status(&st);
    snprintf(text, sizeof text, "SPI  %-9s %s", st.pins, st.detail);
    put(ROW_BUS_FIRST + 5, 0, COLS, text, state_colour(st.state));

    bb_mdio_status(&st);
    snprintf(text, sizeof text, "MDIO %-9s %s", st.pins, st.detail);
    put(ROW_BUS_FIRST + 6, 0, COLS, text, state_colour(st.state));

    bb_uart_status(&st);
    snprintf(text, sizeof text, "UART %-9s %s", st.pins, st.detail);
    put(ROW_BUS_FIRST + 7, 0, COLS, text, state_colour(st.state));

    bb_sniff_status_line(&st);
    snprintf(text, sizeof text, "SNIF %-9s %s", st.pins, st.detail);
    put(ROW_BUS_FIRST + 8, 0, COLS, text, state_colour(st.state));

    /* The last command and its result on one row: "> I2C:SCAN ok". */
    snprintf(text, sizeof text, "> %-13s %s", last_cmd, last_ok ? "ok" : "ERR");
    put(ROW_LAST_CMD, 0, COLS, text, last_ok ? C_LABEL : C_BAD);
}

#else  /* no display on this board */

void bb_lcd_init(void) {}
bool bb_lcd_ready(void) { return false; }
void bb_lcd_tick(void) {}
void bb_lcd_note_command(const char *line, bool ok) { (void)line; (void)ok; }

#endif
