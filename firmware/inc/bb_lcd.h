/**
 * @file bb_lcd.h
 * @brief Status display: what the bridge is doing, on the board itself.
 *
 * Only built for boards that have a panel (BB_HAS_DISPLAY). Everything here is
 * a no-op otherwise, so callers need no conditionals.
 */
#ifndef BB_LCD_H
#define BB_LCD_H

#include <stdbool.h>
#include <stdint.h>

/** Bring the panel up and draw the frame. Safe to call when there is no panel. */
void bb_lcd_init(void);

/** True once the panel is up. */
bool bb_lcd_ready(void);

/**
 * @brief Redraw whatever has changed.
 *
 * Called from the main loop. Cheap when nothing has changed: it compares
 * against what is on screen and touches only the fields that moved, because a
 * full 240x240 repaint over the FMC costs about 115 KB of transfers and there
 * is no reason to pay that to change one number.
 */
void bb_lcd_tick(void);

/**
 * @brief Read the controller's identity over the bus.
 *
 * Writes the read-display-ID command and reads the words the panel drives
 * back. A plausible answer is the only cheap proof that a display is actually
 * attached and responding - initialising the FMC only proves the FMC accepted
 * a configuration.
 *
 * @return number of words placed in @p words, zero if there is no panel
 */
unsigned bb_lcd_read_id(uint16_t *words, unsigned max);

/**
 * @brief Set the display rotation: 0, 1, 2 or 3 quarter turns.
 *
 * Which value is correct is a fact about how the glass is mounted, not about
 * the controller, so it is settable at run time rather than compiled in -
 * finding it by rebuilding and reflashing four times is a poor use of an
 * afternoon.
 *
 * @return the MADCTL value applied
 */
uint8_t bb_lcd_set_rotation(unsigned quarter);

/** Paint a test pattern, for a human to look at. */
void bb_lcd_test_pattern(void);

/** Note a command arriving, so the display can show activity and a count. */
void bb_lcd_note_command(const char *line, bool ok);

#endif /* BB_LCD_H */
