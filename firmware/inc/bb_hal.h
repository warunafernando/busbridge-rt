/**
 * @file bb_hal.h
 * @brief Thin compatibility layer over the MCUXpresso SDK.
 *
 * The STM32 tree was written against ST's HAL. Three things in this project are
 * genuinely hardware-independent - the command parser, the reply helpers and
 * the output router - and they should not have to be rewritten to move silicon.
 * They depend on ST's HAL for almost nothing: a status enum, a millisecond
 * tick, a busy-wait. This header supplies exactly that much on top of the NXP
 * drivers, so `bb_cmd.c`, `bb_io.c` and `main.c` port unchanged.
 *
 * It is deliberately small. Anything that needs a real peripheral driver uses
 * the `fsl_*` API directly rather than growing this shim into a second HAL -
 * a translation layer that grows to cover two vendors' peripherals ends up
 * expressing neither one well, and hides the differences that matter.
 */
#ifndef BB_HAL_H
#define BB_HAL_H

#include "fsl_common.h"
#include "fsl_iomuxc.h"
#include "fsl_rgpio.h"

#include <stdbool.h>
#include <stdint.h>

/* ---- status ------------------------------------------------------------ */
/* Same four values, same names, so the ~58 HAL_OK comparisons in the ported
 * code keep their meaning. bb_reply_hal() maps these onto the wire protocol's
 * error codes and that mapping is unchanged. */
typedef enum {
    HAL_OK      = 0,
    HAL_ERROR   = 1,
    HAL_BUSY    = 2,
    HAL_TIMEOUT = 3
} HAL_StatusTypeDef;

/** Translate an `fsl_` status_t into the four-value form. */
static inline HAL_StatusTypeDef bb_hal_from_status(status_t s)
{
    switch (s) {
    case kStatus_Success: return HAL_OK;
    case kStatus_Timeout: return HAL_TIMEOUT;
    case kStatus_Busy:    return HAL_BUSY;
    default:              return HAL_ERROR;
    }
}

/* ---- time -------------------------------------------------------------- */
/* Milliseconds since boot, from the SysTick handler in bb_board.c. */
uint32_t HAL_GetTick(void);
void     HAL_Delay(uint32_t ms);

/* ---- interrupts -------------------------------------------------------- */
/* CMSIS provides NVIC_*; ST's spellings are kept so the ported call sites do
 * not change. Priority grouping is not used - every priority here is a plain
 * pre-emption level. */
#define HAL_NVIC_EnableIRQ(irq)             NVIC_EnableIRQ(irq)
#define HAL_NVIC_DisableIRQ(irq)            NVIC_DisableIRQ(irq)
#define HAL_NVIC_SetPriority(irq, pre, sub) NVIC_SetPriority((irq), (pre))

/* ---- pins -------------------------------------------------------------- */
/**
 * @brief One pin, in the shape the i.MX IOMUXC actually has.
 *
 * On the STM32 a pin was a port, a bit and an alternate-function number. Here a
 * pad has a mux register, a mux mode, an optional input-select (daisy chain)
 * register and value, and a pad-config register - which is why the SDK's
 * `IOMUXC_GPIO_AD_18_LPI2C3_SCL` expands to five comma-separated values rather
 * than one.
 *
 * The daisy chain is the part with no STM32 counterpart and it matters later:
 * a peripheral input selects which pad drives it, independently of that pad's
 * own mux mode. That is the mechanism the self-sniffing experiment turns on
 * (see docs/PINS.md section 5).
 */
typedef struct {
    uint32_t    mux_reg;      /**< IOMUXC SW_MUX_CTL_PAD register address    */
    uint32_t    mux_mode;     /**< ALT number for the wanted function        */
    uint32_t    input_reg;    /**< daisy-chain SELECT_INPUT register, or 0   */
    uint32_t    input_daisy;  /**< value for that register                   */
    uint32_t    cfg_reg;      /**< IOMUXC SW_PAD_CTL_PAD register address    */
    RGPIO_Type *gpio;         /**< port for GPIO use, NULL if never a GPIO   */
    uint8_t     bit;          /**< bit within that port                      */
} bb_pin_t;

/**
 * @brief Build a bb_pin_t from an SDK IOMUXC symbol.
 *
 * @code
 *   static const bb_pin_t scl = BB_PIN(IOMUXC_GPIO_AD_18_LPI2C3_SCL, RGPIO3, 18);
 * @endcode
 *
 * The SDK symbol supplies the first five fields because it expands to exactly
 * those five values in that order.
 */
#define BB_PIN(iomuxc_sym, gpio_port, gpio_bit) \
    { iomuxc_sym, (gpio_port), (gpio_bit) }

/** A pad this board does not bring out. Reported as absent rather than faked. */
#define BB_PIN_NONE { 0u, 0u, 0u, 0u, 0u, NULL, 0u }

static inline bool bb_pin_present(const bb_pin_t *p)
{
    return p != NULL && p->mux_reg != 0u;
}

/* ---- pad configurations ------------------------------------------------ */
/* Named so a call site says what electrical behaviour it wants rather than a
 * These are *intents*, not register values, and that is the whole point.
 *
 * This part has two different SW_PAD_CTL layouts and which one a pad uses
 * depends on where it is:
 *
 *   GPIO_EMC_B1_*, GPIO_EMC_B2_*   PDRV[1:0], PULL[3:2], ODE[4]
 *   GPIO_AD_*, GPIO_AON_*          SRE[0], DSE[1], PUE[2], PUS[3], ODE[4]
 *
 * The encodings disagree in the bits that matter most here: a weak pull-up is
 * PULL=01 (bit 2 only) on an EMC pad and PUE+PUS (bits 2 and 3) on an AD pad,
 * so bits 2 and 3 together mean "pull up" on one and "no pull at all" on the
 * other. One numeric constant therefore cannot be right for every pin, and the
 * original single set of values - taken from the RT1170 layout - silently
 * configured every EMC pad as open drain with *no* pull.
 *
 * That is not a cosmetic error. It made a released open-drain line float:
 * bit-banged MDIO read 0x0000 on an idle bus instead of 0xFFFF, and LPI2C kept
 * its bus-busy flag set forever because SCL and SDA never rose - which
 * presented as "peripheral busy" on every transfer and looked for all the
 * world like a stuck peripheral rather than a missing pull-up.
 *
 * bb_pin_mux() and bb_pin_gpio() translate these to the correct encoding for
 * the pad being configured, so callers name what they want and cannot get the
 * layout wrong.
 *
 * The internal pull-up is roughly 40 kOhm either way - enough to let a bus with
 * no external resistors idle high and be testable, nowhere near enough for real
 * signalling. Same caveat as the STM32 build, same advice: fit 2.2k to 4.7k. */
#define BB_PAD_I2C        0U   /* open drain + weak pull-up                */
#define BB_PAD_PUSHPULL   1U   /* push-pull, no pull                       */
#define BB_PAD_INPUT      2U   /* input, no pull                           */
#define BB_PAD_INPUT_PU   3U   /* input, weak pull-up                      */
#define BB_PAD_I3C        4U   /* push-pull + fast slew + weak pull-up      */
#define BB_PAD_UART       BB_PAD_PUSHPULL

/*
 * BB_PAD_I3C exists because BB_PAD_I2C is wrong for I3C and the difference is
 * not small.
 *
 * I3C asks for the internal pull-up for the same reason I2C does - the
 * open-drain address header needs something to pull the line high on a bench
 * with no resistors - and the obvious way to get it was to hand the pads
 * BB_PAD_I2C. That also sets ODE. But I3C SDR data is push-pull: after the
 * address header the controller drives both edges. With ODE set it cannot, so
 * every rising edge in the payload was being made by a ~40 kOhm resistor
 * instead of a driver.
 *
 * The cost was the whole high-rate story. Reads above about 6 MHz timed out,
 * reads between 1.5 and 4 MHz came back with more bytes in the receive FIFO
 * than the wire carried, the working point moved with the bus rate, and it
 * inverted depending on whether the internal pull-up was on - all the
 * signatures of an RC that was never meant to be in the signal path.
 *
 * Measured with this mode instead: 5/5 exact on broadcast CCC, private write
 * and private read at 1, 2, 4, 6, 8, 10 and 12.5 MHz, with zero extra FIFO
 * entries at every rate.
 *
 * Pull-up, high drive and fast slew, ODE clear. The pull-up still serves the
 * open-drain phases, where the pad's own open-drain behaviour is supplied by
 * the I3C peripheral rather than by ODE. Fit real 2.2k-4.7k resistors for
 * anything beyond a bench.
 */

/**
 * @brief Route a pad to a peripheral function and set its pad control.
 *
 * @param sion  force the input path on regardless of mux mode. Needed whenever
 *              a peripheral must read back what it drives - every open-drain
 *              bus - and it is also the first half of the internal-tap
 *              experiment.
 */
void bb_pin_mux(const bb_pin_t *p, uint32_t pad_cfg, bool sion);

/** Configure the pad as a plain GPIO, ALT5 on every i.MX RT pad. */
void bb_pin_gpio(const bb_pin_t *p, bool output, bool initial, uint32_t pad_cfg);

void bb_pin_write(const bb_pin_t *p, bool high);
bool bb_pin_read(const bb_pin_t *p);
void bb_pin_toggle(const bb_pin_t *p);

#endif /* BB_HAL_H */
