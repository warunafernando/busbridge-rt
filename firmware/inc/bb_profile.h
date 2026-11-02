/**
 * @file bb_profile.h
 * @brief What an image does with the board's pins.
 *
 * Two images are built from this tree, from the same board layer, the same USB
 * and console transports and the same command parser:
 *
 *   bridge      the USB bus controller - drives I3C, I2C, SPI, MDIO and serial
 *   peripheral  the device under test - answers as a target on those buses,
 *               every one of them serving one shared EEPROM image
 *
 * They cannot be one image with a mode switch, and the reason is not taste: a
 * bus controller and a bus target both want I2C1_EV_IRQHandler, both want the
 * SPI instance, and on a 64-pin part they want the same physical pins. So the
 * profile chooses which modules are compiled in, and main.c is written against
 * these three hooks rather than against a list of buses.
 *
 * Adding a bus to a profile therefore touches that profile and nothing else -
 * no central list to keep in step, the same reasoning the board directories
 * already follow.
 */
#ifndef BB_PROFILE_H
#define BB_PROFILE_H

#include "bb_cmd.h"

#include <stdbool.h>

/** "bridge" or "peripheral"; reported by SYS:ID? so a host can tell them apart. */
extern const char *const bb_profile_name;

/** One line for the banner, e.g. "5 buses" or "I3C/I2C/SPI/MDIO targets". */
extern const char *const bb_profile_summary;

/** Bring up this profile's buses. Called once, after the board and USB. */
void bb_profile_init(void);

/** Called on every pass of the command pump. For anything that must be serviced
 *  faster than a command arrives - a bit-banged target, a transparent bridge. */
void bb_profile_poll(void);

/** Called on the heartbeat, for work with nothing to gain from going faster. */
void bb_profile_tick(void);

/**
 * @brief One command has just run, and whether it succeeded.
 *
 * For a profile that shows the last command somewhere - the bridge's on-board
 * panel does. The command pump reports the outcome without knowing whether
 * anything is listening.
 */
void bb_profile_note_command(const char *word, bool ok);

/** This profile's command tables, NULL-terminated. SYS is added by bb_cmd.c. */
extern const bb_cmd_t *const bb_profile_tables[];

#endif /* BB_PROFILE_H */
