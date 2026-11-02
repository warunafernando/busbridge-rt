/**
 * @file bb_board.h
 * @brief Board-independent hardware interface.
 *
 * The pin map, clock tree and peripheral choices live in
 * firmware/boards/<board>/board.h, selected by the build. This header is the
 * part every module can rely on regardless of which board it is: the pin
 * types, the LED and button API, the cycle counter, the published pin map.
 *
 * One board so far. The porting discipline from the STM32 tree is kept anyway -
 * three files under boards/<name>/ and nothing outside them - because that is
 * what made the third STM32 board cost nothing to add. Anything that reads like
 * a board fact belongs in the board header, not here.
 */
#ifndef BB_BOARD_H
#define BB_BOARD_H

#include "bb_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* bb_pin_t is defined in bb_hal.h: on this part a pad is a mux register, a
 * mux mode, an input-select daisy chain and a pad-config register, not a
 * port/bit/AF triple. */

/**
 * @brief One entry of the published pin map.
 *
 * The silkscreen label and the connector are separate fields on purpose. A
 * label alone does not say which hole to put a wire in: the Arduino numbering
 * runs across four connectors in no particular order, so "D65" and "D28" are
 * on different ones, and guessing the connector from the label is exactly the
 * mistake that made a two-board link read as an open circuit.
 */
typedef struct {
    const char *signal;    /**< "i2c1.scl"                                  */
    const char *pin;       /**< MCU pin, "PB8"                              */
    const char *header;    /**< silkscreen label, "D15", or "-"             */
    const char *conn;      /**< connector and pin, "CN7-2", or "-" if not
                                brought out to one                          */
} bb_pinmap_t;

/**
 * @brief What the running silicon reports about itself.
 *
 * Read from the factory-programmed registers, so it describes the part in the
 * socket rather than the part this firmware was compiled for. The two disagree
 * exactly when the wrong image has been flashed, which is worth catching: the
 * pin map, and therefore every wiring decision made from it, belongs to the
 * board the build assumed.
 */
typedef struct {
    uint16_t dev_id;         /**< family id from OCOTP, 0x1186 for this part */
    uint16_t revision;       /**< DBGMCU REV_ID                              */
    uint16_t flash_kb;       /**< flash size the factory recorded            */
    uint16_t package;        /**< package code from OCOTP, raw and undecoded */
    uint16_t expect_dev_id;  /**< what the build expects, 0 if it says none  */
    bool     checked;        /**< false when the board declares no expectation */
    bool     matches;        /**< true when the silicon is what was expected */
} bb_silicon_t;

void bb_board_silicon(bb_silicon_t *out);

/* ---- the board ---------------------------------------------------------- */
/* Adding a board does not mean editing a chain here - there is no chain. The
 * build puts exactly one boards/<name>/ directory on the include path, and
 * that decides which definition below resolves. Configure with
 * -DBB_BOARD=<directory name>.
 *
 * The file is bb_board_def.h rather than board.h because the MCUXpresso SDK
 * ships its own board.h for this board, and two files of that name on one
 * include path is a coin flip resolved by search order. */
#include "bb_board_def.h"

#ifndef BB_BOARD_NAME
#  error "boards/<name>/bb_board_def.h did not define BB_BOARD_NAME - wrong board on the include path?"
#endif

/* Defaults for facts most boards do not need to state. */
#ifndef BB_CONSOLE_BAUD
/* Not a board property: the ST-LINK VCP runs at whatever both ends agree on,
 * and every host tool here defaults to this. */
#  define BB_CONSOLE_BAUD 115200U
#endif
#ifndef BB_I2C3_AVAILABLE
#  define BB_I2C3_AVAILABLE 1
#endif
#ifndef BB_I3C_PORT_COUNT
#  define BB_I3C_PORT_COUNT 1u
#endif

/* ---- User LEDs --------------------------------------------------------- */
/* The first three keep their roles on every board - green is the heartbeat,
 * yellow lights while a command runs, red means a fault - so the indications
 * mean the same thing wherever you are looking. A board with a fourth LED
 * exposes it as BB_LED_EXTRA. */
typedef enum {
    BB_LED_GREEN = 0,
    BB_LED_YELLOW,
    BB_LED_RED,
    BB_LED_EXTRA,
    BB_LED_MAX
} bb_led_t;

/** LEDs this board actually has. */
#define BB_LED_COUNT  BB_LED_COUNT_HW

/* What the firmware uses LEDs for, resolved to the LEDs a board has rather
 * than to fixed indices. BB_LED_MAX means "this board has nowhere to show it".
 *
 * A fault falls back to the first LED when there is no third one: the board is
 * already in trouble, and taking the heartbeat's LED to say so is a better
 * trade than saying nothing. Activity does not fall back - a one-LED board
 * blinking on every command would just look broken. */
#define BB_LED_HEARTBEAT  BB_LED_GREEN
#define BB_LED_ACTIVITY   ((BB_LED_COUNT > 1u) ? BB_LED_YELLOW : BB_LED_MAX)
#define BB_LED_FAULT      ((BB_LED_COUNT > 2u) ? BB_LED_RED : BB_LED_GREEN)

void bb_board_init(void);

/**
 * @brief Free-running cycle counter for sub-microsecond delays.
 *
 * Backed by TIM2 (32-bit, clocked at PCLK1) rather than DWT->CYCCNT.  CYCCNT
 * only runs while DEMCR.TRCENA is set, and a debugger clears that when it
 * detaches - which silently turns any CYCCNT-based delay loop into an infinite
 * one the moment you unplug the debugger.  A timer has no such dependency.
 */
uint32_t bb_cycles(void);

/** Frequency at which bb_cycles() advances, in Hz. */
uint32_t bb_cycle_hz(void);

/**
 * @brief Drive a LED on behalf of the host, and keep it.
 *
 * The LED stops being used for the heartbeat or anything else automatic until
 * bb_led_release() gives it back. Without that, a board with one LED cannot be
 * driven by the host at all: the heartbeat overwrites it twice a second.
 */
void     bb_led_set(bb_led_t led, bool on);

/** Give a LED back to the firmware's own use. */
void     bb_led_release(bb_led_t led);

/** True when the host has claimed this LED. */
bool     bb_led_claimed(bb_led_t led);

/**
 * @brief Drive a LED for the firmware's own indication.
 *
 * Does nothing for a LED the host has claimed, or one this board does not
 * have - so callers can name a role without checking whether it landed
 * anywhere.
 */
void     bb_led_indicate(bb_led_t led, bool on);
void     bb_led_toggle(bb_led_t led);
bool     bb_button_pressed(void);
uint32_t bb_uid_word(uint8_t index);

/* ---- pin helpers ------------------------------------------------------- */

/**
 * @brief Drive @p scl and @p sda as open-drain alternate-function pins.
 *
 * The internal pull-up is enabled so a bus with no external resistors still
 * idles high and is testable; it is far too weak (roughly 40 kOhm) for real
 * signalling, so fit 2.2k to 4.7k externals for anything above a slow crawl.
 */
void bb_pins_i2c(const bb_pin_t *scl, const bb_pin_t *sda);

/**
 * @brief Put the I3C controller's pins on port @p port (1-based).
 *
 * There is one I3C peripheral on both supported parts, so this chooses which
 * pin pair the bus appears on, not which controller to use. Boards with a
 * single pair ignore anything but port 1.
 *
 * I3C drives push-pull itself.  ST's sequence is to come up with the internal
 * pull-up so the bus reaches a valid idle high, then remove it.
 */
void bb_pins_i3c(unsigned port);

/** Pins of I3C port @p port, for reporting. NULL if there is no such port. */
const char *bb_i3c_port_pins(unsigned port);

/**
 * @brief The board's pin map, as published by SYS:PINS?.
 *
 * One table, next to the macros it describes, so the host tools and the GUI can
 * show the wiring without keeping their own copy of it. A pin that moves has to
 * move here too, and then everything downstream follows.
 */
const bb_pinmap_t *bb_pin_map(unsigned *count);

/**
 * @brief Hold the I3C port's pins at fixed levels, as push-pull outputs.
 *
 * A continuity aid, not a bus operation: it exists so a second board can be
 * asked "do you see this?" about a level rather than about an edge. Push-pull
 * so it needs no pull-up resistor to prove a wire.
 *
 * The next I3C command takes the pins back.
 */
void bb_pins_i3c_drive(unsigned port, bool scl_high, bool sda_high);

/**
 * @brief Let go of the I3C port's pins entirely: inputs, no pull.
 *
 * The state a board needs when it is listening rather than talking. An I3C
 * controller drives SCL push-pull, so two initialised controllers on one wire
 * fight each other, and the loser is whichever line you were trying to read.
 * A released pin contributes nothing to the bus.
 */
void bb_pins_i3c_free(unsigned port);

/**
 * @brief Choose whether bb_pins_i3c() leaves the internal pull-ups enabled.
 *
 * Off by default. See I3C:PULL: the internal pull-up is roughly 40 kOhm, which
 * is enough to hold a short bench wire high at low speed and nowhere near what
 * a real I3C bus needs. Takes effect the next time the pins are configured.
 */
void bb_pins_i3c_set_pull(bool on);

/**
 * @brief Reset any device the board has permanently wired to an I2C bus.
 *
 * Bus recovery clocks a target that is holding SDA. It cannot help a target
 * whose state machine is lost, and at least one device on the Discovery kit can
 * end up exactly there. Where the board has a reset line for such a device,
 * this pulses it; elsewhere it does nothing.
 *
 * @param bus 1-based I2C bus number
 * @return true if something was actually reset
 */
bool bb_board_reset_bus_devices(unsigned bus);

void bb_error_handler(const char *where);

#endif /* BB_BOARD_H */
