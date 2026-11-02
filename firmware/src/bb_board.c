/**
 * @file bb_board.c
 * @brief Board layer for the FRDM-IMXRT1186: clocks, pins, LEDs, time.
 *
 * The STM32 version of this file programmed the PLL by hand. Here the clock
 * tree is left to the SDK's generated `BOARD_InitHardware()`, for a reason
 * worth stating: on the RT1180 the clock roots, the ELE and the TRDC all have
 * to agree before a peripheral answers at all, and reproducing that by hand
 * would be a second source of truth that drifts from the SDK's.
 *
 * What is here is what the SDK does not do: the pin helpers in the shape this
 * project uses, LED ownership, a monotonic tick, and the published pin map.
 */
#include "bb_board.h"

#include "fsl_silicon_id.h"
#include "bb_cmd.h"
#include "bb_io.h"

#include "fsl_clock.h"

#include <string.h>

/* Provided by the SDK board support for this board. */
void BOARD_InitHardware(void);

/* The board's own map, from firmware/boards/<board>/bb_pinmap.inc. */
#include "bb_pinmap.inc"

/* ---- time -------------------------------------------------------------- */
/*
 * A millisecond tick from SysTick, and a free-running cycle counter for
 * sub-microsecond work.
 *
 * The STM32 build used TIM2 for the cycle counter rather than DWT->CYCCNT,
 * because CYCCNT only runs while DEMCR.TRCENA is set and a debugger clears it
 * on detach - which turns any CYCCNT delay loop into an infinite one the moment
 * you unplug. The same trap exists here, so the same choice is made: the
 * counter below is derived from SysTick, which nothing external switches off.
 */
static volatile uint32_t s_tick_ms;
static uint32_t          s_cycle_hz;

void SysTick_Handler(void)
{
    s_tick_ms++;
}

uint32_t HAL_GetTick(void)
{
    return s_tick_ms;
}

void HAL_Delay(uint32_t ms)
{
    uint32_t start = s_tick_ms;
    /* +1 so a 1 ms delay is never zero: the tick may be about to advance. */
    while ((s_tick_ms - start) < (ms + 1u)) {
        __NOP();
    }
}

uint32_t bb_cycles(void)
{
    /* SysTick counts down and reloads every millisecond, so a monotonic cycle
     * count is elapsed milliseconds scaled, plus how far into the current one
     * we are. The tick is read either side of VAL: if it moved, the reload fell
     * between them and that pairing was inconsistent. */
    uint32_t ms1, val, ms2;
    do {
        ms1 = s_tick_ms;
        val = SysTick->VAL;
        ms2 = s_tick_ms;
    } while (ms1 != ms2);

    uint32_t load = SysTick->LOAD + 1u;
    return (ms1 * load) + (load - val);
}

uint32_t bb_cycle_hz(void)
{
    return s_cycle_hz;
}

/* ---- pins -------------------------------------------------------------- */

/*
 * Translate a BB_PAD_* intent into this pad's SW_PAD_CTL encoding.
 *
 * The two layouts and why this exists are documented at the BB_PAD_* defines in
 * bb_hal.h. The pad group is identified by its config register address, taken
 * from fsl_iomuxc.h: the EMC_B1 and EMC_B2 blocks are contiguous, and
 * everything else this project uses (AD, AON) is on the other layout.
 *
 * Deriving it from the address rather than carrying a flag in bb_pin_t keeps
 * bb_pinmap.inc free of a field that describes the silicon rather than the
 * board, and means a pin added later gets the right encoding without anyone
 * remembering this exists.
 */
/*
 * The two pad layouts are per IOMUXC INSTANCE, not per pad group.
 *
 * There are two IOMUXC blocks and their SW_PAD_CTL registers are not the same
 * register. The AON block (0x443C_xxxx) has PUE[2] "pull enable" and PUS[3]
 * "1 = up", so a pull-up is both bits set. The main block (0x42A1_xxxx) has a
 * single PULL[3:2] field instead - 01 = up, 10 = down, 11 = none, 00 forbidden
 * - so a pull-up is bit 2 set and bit 3 CLEAR.
 *
 * Those two encodings collide on the same two bits, and "both bits set" is a
 * pull-up under one and No Pull under the other. The header does not protect
 * anyone here: PERI_IOMUXC.h defines PUE and PUS macros alongside PULL, aliasing
 * the same bits with the opposite meaning, so code written against the wrong
 * pair compiles and runs and quietly configures the opposite thing. Bit 1 is
 * aliased the same way, as DSE (1 = high driver) and PDRV (0 = high driver).
 *
 * This previously split on the EMC address range, which put the EMC pads on the
 * PULL encoding and everything else - including GPIO_AD - on PUE/PUS. AD pads
 * are in the main block, so BB_PAD_I2C emitted 0x1E there: PULL = 11, No Pull,
 * on the two pads carrying I3C2. The cost was a long hunt: I3C's ENTDAA failed
 * with no-ack-for-7E and recovered only while the sniffer was running, because
 * claiming the tap pads put a correctly configured pull-up on the same net. It
 * read as a wiring fault, then as a target fault, and was neither.
 *
 * Splitting on the instance is also what makes it stay right: a pad added later
 * gets the encoding its own register actually uses.
 */
#define PAD_AON_CFG_BASE   0x443C0000u

/*
 * What bit 1 of the main block actually is, measured rather than read.
 *
 * The header aliases it as DSE (1 = high driver) and PDRV (1 = *normal*
 * driver), which are opposite, and nothing in the header says which applies to
 * a given pad. Settled on the bench at 12.5 MHz - the rate with the least
 * margin - six write/read pairs per value, against I3C2 on AD_18/AD_19:
 *
 *   0x0F   PULL=11 No Pull, bit1=1, bit0=1   6/6 exact
 *   0x07   PULL=01 pull-up, bit1=1, bit0=1   6/6 exact
 *   0x0C   PULL=11 No Pull, bit1=0, bit0=0   0/6
 *   0x04   PULL=01 pull-up, bit1=0, bit0=0   0/6
 *
 * Bits 0 and 1 decide it; the pull does not, in either direction. A bus that
 * works only with bit 1 set is one that needs the stronger driver, so on these
 * pads bit 1 is DSE and PDRV is the wrong alias. Worth writing down, because
 * the two readings disagree about whether the shipping value asks for a high
 * driver or a normal one and only one of them explains the bench.
 *
 * That the pull makes no difference also says this bench has external pull-ups
 * on the I3C2 net. Anything relying on an *internal* one - I3C1 on the AON
 * pads, where the same constant really does enable it - cannot infer from I3C2
 * working that it will.
 */
static uint32_t pad_encode(uint32_t cfg_reg, uint32_t mode)
{
    if (cfg_reg < PAD_AON_CFG_BASE) {
        /* Main IOMUXC: SRE[0], DSE[1] 1 = high driver, PULL[3:2], ODE[4]. */
        switch (mode) {
        case BB_PAD_I2C:      return 0x14u;   /* pull-up + open drain */
        case BB_PAD_INPUT_PU: return 0x04u;   /* pull-up              */
        /* Drive and slew, and PULL = 11, which on this block is No Pull - not
         * the pull-up this line used to claim. Left as it is: it is what the
         * bench runs at every rate to 12.5 MHz, and the measurement above shows
         * the pull bits change nothing here either way. */
        case BB_PAD_I3C:      return 0x0Fu;   /* drive + slew, no pull, no ODE */
        default:              return 0x0Cu;   /* no pull              */
        }
    }
    /* AON IOMUXC: SRE[0] 0 = fast, DSE[1] 1 = high driver, PUE[2] pull enable,
     * PUS[3] 1 = up, ODE[4]. Bit 7 would force the input buffer off and is
     * never set - a pin that cannot be read back breaks every bit-banged
     * protocol that releases a line and then samples it. */
    switch (mode) {
    case BB_PAD_I2C:      return 0x1Eu;   /* pull-up + open drain */
    case BB_PAD_INPUT_PU: return 0x0Eu;   /* pull-up              */
    case BB_PAD_I3C:      return 0x0Fu;   /* PUE+PUS+DSE+SRE, ODE clear */
    default:              return 0x02u;   /* no pull              */
    }
}

void bb_pin_mux(const bb_pin_t *p, uint32_t pad_cfg, bool sion)
{
    if (!bb_pin_present(p)) {
        return;
    }
    IOMUXC_SetPinMux(p->mux_reg, p->mux_mode, p->input_reg, p->input_daisy,
                     p->cfg_reg, sion ? 1U : 0U);
    IOMUXC_SetPinConfig(p->mux_reg, p->mux_mode, p->input_reg, p->input_daisy,
                        p->cfg_reg, pad_encode(p->cfg_reg, pad_cfg));
}

/* ALT5 is GPIO on every i.MX RT pad, which is why a pin needs only one IOMUXC
 * symbol even when it is used both ways. */
#define BB_ALT_GPIO 5u

void bb_pin_gpio(const bb_pin_t *p, bool output, bool initial, uint32_t pad_cfg)
{
    if (!bb_pin_present(p) || p->gpio == NULL) {
        return;
    }
    /*
     * SION on, always - the last argument.
     *
     * Software Input On forces the pad's input path active regardless of
     * direction. Without it a pin configured as an output cannot be read back,
     * and every bit-banged open-drain protocol depends on exactly that: it
     * drives the line high to release it and then reads what the other end is
     * doing. With SION off the read returns 0 whatever the pad is at, so a
     * floating MDIO bus read as 0x0000 instead of 0xFFFF - which looks like a
     * dead PHY rather than a pad configuration.
     *
     * It costs nothing for output-only or input-only use, so it is unconditional
     * here rather than another parameter for callers to get wrong. i2c_apply()
     * already passes SION for the same reason when it hands the pads to LPI2C.
     */
    IOMUXC_SetPinMux(p->mux_reg, BB_ALT_GPIO, 0U, 0U, p->cfg_reg, 1U);
    IOMUXC_SetPinConfig(p->mux_reg, BB_ALT_GPIO, 0U, 0U, p->cfg_reg,
                        pad_encode(p->cfg_reg, pad_cfg));

    rgpio_pin_config_t cfg;
    cfg.pinDirection = output ? kRGPIO_DigitalOutput : kRGPIO_DigitalInput;
    cfg.outputLogic  = initial ? 1U : 0U;
    RGPIO_PinInit(p->gpio, p->bit, &cfg);
}

void bb_pin_write(const bb_pin_t *p, bool high)
{
    if (bb_pin_present(p) && p->gpio != NULL) {
        RGPIO_PinWrite(p->gpio, p->bit, high ? 1U : 0U);
    }
}

bool bb_pin_read(const bb_pin_t *p)
{
    if (!bb_pin_present(p) || p->gpio == NULL) {
        return false;
    }
    return RGPIO_PinRead(p->gpio, p->bit) != 0U;
}

void bb_pin_toggle(const bb_pin_t *p)
{
    if (bb_pin_present(p) && p->gpio != NULL) {
        RGPIO_PortToggle(p->gpio, 1UL << p->bit);
    }
}

/* ---- LEDs -------------------------------------------------------------- */
/*
 * Ownership, not just state. A host that sets a LED keeps it until it says
 * AUTO: without that the heartbeat overwrites the host twice a second, and the
 * LED command looks broken rather than overridden.
 */
static const bb_pin_t s_leds[] = BB_LED_PINS;
static bool           s_claimed[BB_LED_MAX];

#if BB_LED_ACTIVE_LOW
#  define LED_LEVEL(on) (!(on))
#else
#  define LED_LEVEL(on) (on)
#endif

static bool led_exists(bb_led_t led)
{
    return (unsigned)led < (unsigned)BB_LED_COUNT;
}

void bb_led_set(bb_led_t led, bool on)
{
    if (!led_exists(led)) {
        return;
    }
    s_claimed[led] = true;
    bb_pin_write(&s_leds[led], LED_LEVEL(on));
}

void bb_led_release(bb_led_t led)
{
    if (led_exists(led)) {
        s_claimed[led] = false;
    }
}

bool bb_led_claimed(bb_led_t led)
{
    return led_exists(led) && s_claimed[led];
}

void bb_led_indicate(bb_led_t led, bool on)
{
    /* Silently does nothing for a LED this board lacks or the host owns, so a
     * caller can name a role - fault, activity - without checking whether it
     * lands anywhere. */
    if (led_exists(led) && !s_claimed[led]) {
        bb_pin_write(&s_leds[led], LED_LEVEL(on));
    }
}

void bb_led_toggle(bb_led_t led)
{
    if (led_exists(led) && !s_claimed[led]) {
        bb_pin_toggle(&s_leds[led]);
    }
}

/* ---- button ------------------------------------------------------------ */

static const bb_pin_t s_button = BB_BUTTON_PIN;

bool bb_button_pressed(void)
{
    bool level = bb_pin_read(&s_button);
#if BB_BUTTON_ACTIVE_LOW
    return !level;
#else
    return level;
#endif
}

/* ---- identity ---------------------------------------------------------- */
/*
 * There is no DBGMCU IDCODE on this part; identity comes from OCOTP.
 *
 * The point of reporting it is unchanged from the STM32 build: the pin map, and
 * therefore every wiring decision made from it, belongs to the board this image
 * was built for. Silicon that disagrees with the build means the wrong image is
 * running, which is worth catching before someone follows its advice.
 */
uint32_t bb_uid_word(uint8_t index)
{
    /*
     * The device UID, from the OCOTP shadow registers.
     *
     * Read through SILICONID_ReadUniqueID() rather than a literal address, and
     * that is the fix rather than a tidy-up: this used to read 0x47510C00 and
     * returned zeros on every word, because the OCOTP_FSB block is at
     * 0x47518000 and the UID sits in OTP_SHADOW_PARTA[12..15]. Nothing
     * complained - a UID of all zeros looks exactly like a part with no fuses
     * programmed - and it quietly cost the USB serial number, which is derived
     * from it.
     *
     * Cached because it cannot change and the read is a handful of register
     * accesses that several commands make.
     */
    static uint8_t  uid[SILICONID_MAX_LENGTH];
    static bool     have;
    static uint32_t words;

    if (!have) {
        uint32_t len = sizeof uid;

        if (SILICONID_GetID(uid, &len) == kStatus_Success) {
            words = len / 4u;
        } else {
            words = 0u;
        }
        have = true;
    }
    if (index >= words) {
        return 0u;
    }
    /* Assembled byte-wise rather than cast: the buffer is a uint8_t array with
     * no alignment guarantee, and a 32-bit load through a misaligned pointer is
     * a fault waiting for a different compiler. */
    return ((uint32_t)uid[index * 4u + 0u])
         | ((uint32_t)uid[index * 4u + 1u] << 8)
         | ((uint32_t)uid[index * 4u + 2u] << 16)
         | ((uint32_t)uid[index * 4u + 3u] << 24);
}

void bb_board_silicon(bb_silicon_t *out)
{
    if (out == NULL) {
        return;
    }
    memset(out, 0, sizeof *out);

    /*
     * Still not read, and now for a specific reason rather than a TODO.
     *
     * The register that would answer this is ANADIG_MISC's MISC_DIFPROG, which
     * carries a CHIPID field. PERI_ANADIG_MISC.h exists in the SDK and defines
     * those fields - but MIMXRT1189 never instantiates the block: there is no
     * ANADIG_MISC base address and no pointer, so the peripheral is not
     * reachable from this part's headers at all.
     *
     * So `checked` stays false. The device UID *is* read (see bb_uid_word), and
     * SYS:ID? reports it - which is the useful half of identifying a board, and
     * the half that was silently broken. Reporting the build's own expectation
     * as if it were a measurement would defeat the point of the flag.
     */
    out->dev_id        = 0u;
    out->revision      = 0u;
    out->flash_kb      = 0u;   /* external QSPI NOR; not a silicon property */
    out->package       = 0u;
    out->expect_dev_id = BB_EXPECT_SILICON_ID;
    out->checked       = false;
    out->matches       = false;
}

/* ---- pin map ----------------------------------------------------------- */

const bb_pinmap_t *bb_pin_map(unsigned *count)
{
    if (count != NULL) {
        *count = (unsigned)(sizeof pin_map / sizeof pin_map[0]);
    }
    return pin_map;
}

/* ---- board-wired devices ----------------------------------------------- */

bool bb_board_reset_bus_devices(unsigned bus)
{
    /* Nothing on this board is permanently wired to an I2C bus that the
     * firmware can reset - unlike the STM32H573I-DK, whose FT3267 touch
     * controller could get its state machine lost in a way no amount of clock
     * pulsing would fix.
     *
     * Returning false rather than true matters: I2C:RECOVER reports whether
     * anything was actually reset, and claiming a reset that did not happen
     * would send someone looking for a hardware fault that is really a stuck
     * target needing clocks. */
    (void)bus;
    return false;
}

/* ---- faults ------------------------------------------------------------ */

void bb_error_handler(const char *where)
{
    /* Say where, then stop. A silent hang during bring-up is the most
     * expensive failure mode there is. */
    bb_led_indicate(BB_LED_FAULT, true);
    bb_io_printf("# FATAL %s\r\n", where != NULL ? where : "unknown");
    for (;;) {
        __NOP();
    }
}

/* ---- init -------------------------------------------------------------- */

void bb_board_init(void)
{
    /* Clock tree, TRDC and the pads the SDK owns. Everything below assumes it
     * has already run. */
    BOARD_InitHardware();

    s_cycle_hz = CLOCK_GetFreq(kCLOCK_CpuClk);
    if (s_cycle_hz == 0u) {
        s_cycle_hz = BB_CM33_HZ;
    }
    (void)SysTick_Config(s_cycle_hz / 1000u);

    for (unsigned i = 0; i < BB_LED_COUNT; i++) {
        bb_pin_gpio(&s_leds[i], true, LED_LEVEL(false) != 0, BB_PAD_PUSHPULL);
        s_claimed[i] = false;
    }
    bb_pin_gpio(&s_button, false, false, BB_PAD_INPUT_PU);
}
