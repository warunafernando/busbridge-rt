/**
 * @file profiles/peripheral/bb_ee_mdio.c
 * @brief An MDIO slave - a PHY, as far as a controller is concerned - over the
 *        same kilobyte.
 *
 * MDIO registers are sixteen bits and there are thirty-two of them, so the
 * whole array cannot be mapped directly: 1 KB is 512 words. Real PHYs solve
 * this the same way, with an indirect window, and so does this:
 *
 *   reg 0x00  control        writes ignored, reads 0x1000
 *   reg 0x02  PHY ID high    0xBBBB
 *   reg 0x03  PHY ID low     0x1001
 *   reg 0x10  address        byte offset into the array, 0..1023
 *   reg 0x11  data           two bytes at that offset, big-endian; each access
 *                            advances the address by two
 *   reg 0x12  size           1024
 *
 * So reading 64 bytes is a write of the address to reg 0x10 and 32 reads of
 * reg 0x11 - which is exactly what a PHY's MMD or EEPROM window looks like, and
 * the bridge's MDIO:DUMP walks registers 0..31 without disturbing anything.
 *
 * Why this is bit-banged and blocking
 * -----------------------------------
 * There is no MDIO peripheral on this part - the bridge profile bit-bangs its
 * controller too. As the slave, the hard part is the read: the specification
 * gives the responder the second turnaround bit to get onto the line, and then
 * a bit every MDC period, changing while MDC is low and stable on its rising
 * edge. At the bridge's default 1 MHz that is 500 ns per half period, or 125
 * cycles at 250 MHz.
 *
 * One interrupt per MDC edge does not fit that: the entry and exit alone are a
 * good fraction of the budget. So the first rising edge arms this module through
 * EXTI, and the rest of the frame is then sampled in a tight spin loop with
 * interrupts masked, which is the only way to hold the bit timing. The cost is
 * that everything else on the board stops for the length of one frame - 64 bit
 * times, 64 us at 1 MHz - and the benefit is a slave that answers correctly
 * rather than one that answers late.
 *
 * The spin is bounded by a cycle-count deadline, not by a bit count, because a
 * controller that stops clocking mid-frame must not take the board with it.
 */
#include "bb_board.h"
#include "bb_cmd.h"
#include "bb_ee.h"
#include "bb_ee_ports.h"

#include <stdio.h>

/* Register map. */
#define REG_CONTROL     0x00u
#define REG_STATUS      0x01u
#define REG_PHYID_HI    0x02u
#define REG_PHYID_LO    0x03u
#define REG_EE_ADDR     0x10u
#define REG_EE_DATA     0x11u
#define REG_EE_SIZE     0x12u

#define PHYID_HI        0xBBBBu
#define PHYID_LO        0x1001u

#define EE_MDIO_DEFAULT_PHY  0x01u

/* Clause 22 frame: ST=01, then OP, PHYAD, REGAD, TA, 16 data bits. */
#define OP_WRITE        0x01u
#define OP_READ         0x02u

/**
 * @brief How long to wait for the rest of a frame once it has started.
 *
 * Generous: a whole clause 22 frame is 64 bit times, and at the slowest rate the
 * bridge will drive - 100 kHz - that is 640 us. The deadline exists so a
 * controller that dies mid-frame cannot wedge this board, not to enforce a rate.
 */
#define FRAME_TIMEOUT_US     2000u

static uint8_t  phy_addr = EE_MDIO_DEFAULT_PHY;
static bool     enabled;
static uint16_t ee_addr;                   /* the window's byte offset */

static volatile uint32_t frames_read;
static volatile uint32_t frames_written;
static volatile uint32_t frames_foreign;  /* another PHY's address */
static volatile uint32_t frames_timeout;
static volatile uint32_t frames_malformed;

/* ---- line access -------------------------------------------------------- */

static inline bool mdc_high(void)
{
    return (BB_MDC_PORT->IDR & BB_MDC_PIN) != 0u;
}

static inline bool mdio_high(void)
{
    return (BB_MDIO_PORT->IDR & BB_MDIO_PIN) != 0u;
}

/** Drive the data line. Open drain plus pull-up, as the bus requires: driving
 *  a one means releasing, so the controller can drive it low against us. */
static inline void mdio_drive(bool high)
{
    BB_MDIO_PORT->BSRR = high ? (uint32_t)BB_MDIO_PIN
                              : ((uint32_t)BB_MDIO_PIN << 16);
}

static inline void mdio_release(void)
{
    BB_MDIO_PORT->BSRR = (uint32_t)BB_MDIO_PIN;
}

/** The EXTI line MDC sits on, derived from the pin the board declares. */
static unsigned mdc_line(void)
{
    unsigned line = 0;

    while (line < 15u && (BB_MDC_PIN & (1u << line)) == 0u) {
        line++;
    }
    return line;
}

unsigned bb_ee_mdio_mdc_line(void)
{
    return mdc_line();
}

/* ---- the register file -------------------------------------------------- */

static uint16_t reg_read(uint8_t reg)
{
    switch (reg) {
    case REG_CONTROL:
        return 0x1000u;                    /* auto-negotiation enabled, idle */
    case REG_STATUS:
        return 0x0004u;                    /* link up, so a driver proceeds */
    case REG_PHYID_HI:
        return PHYID_HI;
    case REG_PHYID_LO:
        return PHYID_LO;
    case REG_EE_ADDR:
        return ee_addr;
    case REG_EE_DATA: {
        /* Big-endian, to match the address format on every other bus here. */
        uint16_t v = (uint16_t)((uint16_t)bb_ee_read(BB_EE_IF_MDIO, ee_addr) << 8);
        v |= bb_ee_read(BB_EE_IF_MDIO, (uint16_t)(ee_addr + 1u));
        ee_addr = (uint16_t)((ee_addr + 2u) & BB_EE_MASK);
        return v;
    }
    case REG_EE_SIZE:
        return (uint16_t)BB_EE_SIZE;
    default:
        return 0x0000u;
    }
}

static void reg_write(uint8_t reg, uint16_t value)
{
    switch (reg) {
    case REG_EE_ADDR:
        ee_addr = (uint16_t)(value & BB_EE_MASK);
        break;
    case REG_EE_DATA:
        bb_ee_write(BB_EE_IF_MDIO, ee_addr, (uint8_t)(value >> 8));
        bb_ee_write(BB_EE_IF_MDIO, (uint16_t)(ee_addr + 1u), (uint8_t)value);
        ee_addr = (uint16_t)((ee_addr + 2u) & BB_EE_MASK);
        break;
    default:
        /* Control and the rest are accepted and dropped, as an unimplemented
         * PHY register is. */
        break;
    }
}

/* ---- the frame ---------------------------------------------------------- */

/* Cycle-counter deadline. bb_cycles() is the same counter the bridge's MDIO
 * controller times its half periods with. */
#define DEADLINE_CYCLES ((bb_cycle_hz() / 1000000u) * FRAME_TIMEOUT_US)

/** Wait for MDC to reach @p want. False if the deadline passed first. */
static inline bool wait_mdc(bool want, uint32_t start, uint32_t budget)
{
    while (mdc_high() != want) {
        if ((bb_cycles() - start) > budget) {
            return false;
        }
    }
    return true;
}

/**
 * @brief Sample @p nbits from the bus, MSB first, on MDC rising edges.
 *
 * Assumes MDC is currently high and the caller has already consumed that edge.
 */
static bool sample_bits(unsigned nbits, uint32_t *out,
                        uint32_t start, uint32_t budget)
{
    uint32_t value = 0;

    for (unsigned i = 0; i < nbits; i++) {
        if (!wait_mdc(false, start, budget) || !wait_mdc(true, start, budget)) {
            return false;
        }
        value = (value << 1) | (mdio_high() ? 1u : 0u);
    }
    *out = value;
    return true;
}

/**
 * @brief Emit @p nbits of @p value, MSB first, changing while MDC is low.
 *
 * The controller samples on the rising edge, so each bit is presented during the
 * low half and held across the following high half.
 */
static bool drive_bits(unsigned nbits, uint32_t value,
                       uint32_t start, uint32_t budget)
{
    for (unsigned i = 0; i < nbits; i++) {
        if (!wait_mdc(false, start, budget)) {
            return false;
        }
        mdio_drive(((value >> (nbits - 1u - i)) & 1u) != 0u);
        if (!wait_mdc(true, start, budget)) {
            return false;
        }
    }
    return true;
}

/**
 * @brief Handle one frame, from the first rising edge of its preamble.
 *
 * Called from the MDC EXTI handler with interrupts masked. Runs for at most one
 * frame time; see the file comment for why this blocks rather than returning to
 * be re-entered per edge.
 */
static void run_frame(void)
{
    const uint32_t start  = bb_cycles();
    const uint32_t budget = DEADLINE_CYCLES;
    uint32_t       bits;

    /* The preamble is 32 ones. We were woken by one of its rising edges, and
     * possibly by the very first, so consume ones until the pattern breaks -
     * that first zero is the ST field's leading bit. Bounded by the deadline
     * rather than by a count, because an idle line reads as ones for ever. */
    for (;;) {
        if (!wait_mdc(false, start, budget) || !wait_mdc(true, start, budget)) {
            frames_timeout++;
            return;
        }
        if (!mdio_high()) {
            break;                  /* the 0 of ST=01 */
        }
    }

    /* The rest of ST, then OP, PHYAD and REGAD: 1 + 2 + 5 + 5 bits. */
    if (!sample_bits(13, &bits, start, budget)) {
        frames_timeout++;
        return;
    }

    unsigned st_low = (bits >> 12) & 0x1u;
    unsigned op     = (bits >> 10) & 0x3u;
    unsigned pa     = (bits >> 5)  & 0x1Fu;
    unsigned ra     = bits         & 0x1Fu;

    if (st_low != 1u) {
        frames_malformed++;
        return;
    }
    if (pa != phy_addr) {
        /* Somebody else's PHY. Stay off the line - the addressed device drives
         * the turnaround, and two responders is a bus fight. */
        frames_foreign++;
        return;
    }

    if (op == OP_WRITE) {
        /* Two turnaround bits driven by the controller, then the data. */
        if (!sample_bits(2, &bits, start, budget)) {
            frames_timeout++;
            return;
        }
        if (!sample_bits(16, &bits, start, budget)) {
            frames_timeout++;
            return;
        }
        reg_write((uint8_t)ra, (uint16_t)bits);
        frames_written++;
        bb_ee_note_transaction(BB_EE_IF_MDIO);
        return;
    }

    if (op != OP_READ) {
        frames_malformed++;
        return;
    }

    /* A read. The controller releases the line for the first turnaround bit and
     * we drive the second one low, then sixteen data bits. Fetch the value
     * before touching the line: a register read can walk the array, and doing it
     * inside the bit loop would spend a bit time on arithmetic. */
    uint16_t value = reg_read((uint8_t)ra);

    /* First TA bit: the controller's, and it is high-impedance. Skip it. */
    if (!wait_mdc(false, start, budget) || !wait_mdc(true, start, budget)) {
        frames_timeout++;
        mdio_release();
        return;
    }
    /* Second TA bit: ours, and it must be zero. */
    if (!drive_bits(1, 0u, start, budget)) {
        frames_timeout++;
        mdio_release();
        return;
    }
    if (!drive_bits(16, value, start, budget)) {
        frames_timeout++;
        mdio_release();
        return;
    }
    /* Release on the falling edge after the last bit, so the line is free
     * before the next frame's preamble. */
    (void)wait_mdc(false, start, budget);
    mdio_release();

    frames_read++;
    bb_ee_note_transaction(BB_EE_IF_MDIO);
}

void bb_ee_mdio_mdc_edge(void)
{
    if (!enabled) {
        return;
    }
    /* Interrupts off for the duration: the bit timing cannot survive being
     * interrupted, and everything else on this board can survive one frame of
     * being ignored. */
    __disable_irq();
    run_frame();
    __enable_irq();
}

/* ---- bring-up ---------------------------------------------------------- */

void bb_ee_mdio_init(void)
{
    GPIO_InitTypeDef g = {0};

    enabled = true;
    ee_addr = 0;

    /* MDC: input with an EXTI on the rising edge. */
    g.Mode  = GPIO_MODE_IT_RISING;
    g.Pull  = GPIO_PULLDOWN;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Pin   = BB_MDC_PIN;
    HAL_GPIO_Init(BB_MDC_PORT, &g);

    /* MDIO: open drain with a pull-up, the same way the bridge's controller
     * drives it. Driving a one releases the line. */
    g.Mode = GPIO_MODE_OUTPUT_OD;
    g.Pull = GPIO_PULLUP;
    g.Pin  = BB_MDIO_PIN;
    HAL_GPIO_Init(BB_MDIO_PORT, &g);
    mdio_release();

    /* Highest priority in this profile. A late entry here is a bit lost at the
     * start of the frame, which is a wrong answer rather than a slow one. */
    HAL_NVIC_SetPriority((IRQn_Type)((int)EXTI0_IRQn + (int)mdc_line()), 2, 0);
    HAL_NVIC_EnableIRQ((IRQn_Type)((int)EXTI0_IRQn + (int)mdc_line()));
}

void bb_ee_mdio_describe(char *out, size_t cap)
{
    if (!enabled) {
        snprintf(out, cap, "off");
    } else {
        snprintf(out, cap, "phy=0x%02X id=%04X%04X addr=0x%03X",
                 phy_addr, PHYID_HI, PHYID_LO, ee_addr);
    }
}

void bb_ee_mdio_status(bb_status_t *out)
{
    snprintf(out->pins, sizeof out->pins, "phy 0x%02X", phy_addr);
    if (!enabled) {
        snprintf(out->detail, sizeof out->detail, "off");
        out->state = BB_STATE_IDLE;
        return;
    }
    snprintf(out->detail, sizeof out->detail, "r%lu w%lu",
             (unsigned long)frames_read, (unsigned long)frames_written);
    out->state = (frames_timeout + frames_malformed) > 0u ? BB_STATE_PROBLEM
               : ((frames_read + frames_written) > 0u ? BB_STATE_ACTIVE
                                                      : BB_STATE_IDLE);
}

/* ---- commands ---------------------------------------------------------- */

static void cmd_mdio(int argc, char **argv)
{
    uint32_t pa;

    if (argc == 1) {
        bb_reply_okf("phy=0x%02X addr=0x%03X reads=%lu writes=%lu "
                     "foreign=%lu timeout=%lu bad=%lu %s",
                     phy_addr, ee_addr,
                     (unsigned long)frames_read,
                     (unsigned long)frames_written,
                     (unsigned long)frames_foreign,
                     (unsigned long)frames_timeout,
                     (unsigned long)frames_malformed,
                     enabled ? "on" : "off");
        return;
    }
    if (bb_streq_ci(argv[1], "OFF")) {
        enabled = false;
        mdio_release();
        bb_reply_ok();
        return;
    }
    if (bb_streq_ci(argv[1], "ON")) {
        bb_ee_mdio_init();
        bb_reply_okf("phy=0x%02X", phy_addr);
        return;
    }
    if (!bb_parse_u32_max(argv[1], 0x1Fu, &pa)) {
        bb_reply_err(BB_ERR_RANGE, "phy-0..31-or-ON-OFF");
        return;
    }
    phy_addr = (uint8_t)pa;
    enabled  = true;
    bb_reply_okf("phy=0x%02X", phy_addr);
}

const bb_cmd_t bb_cmd_ee_mdio[] = {
    { "EE:MDIO", 0, 1, cmd_mdio, "EE:MDIO [<phy 0..31> | ON | OFF]" },
    { NULL, 0, 0, NULL, NULL },
};
