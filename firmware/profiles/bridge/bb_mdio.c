/**
 * @file bb_mdio.c
 * @brief MDIO management interface, bit-banged on MDC=D8, MDIO=D9.
 *
 * Still bit-banged on this part, and not for lack of hardware: the RT1186 has
 * a real MDIO in ENETC, but EMDIO reaches none of the 22 Arduino pads - it is
 * wired to the board's own PHYs and can talk to nothing else. So the hardware
 * MDIO is useless for an external PHY and this stays a GPIO exercise.
 * See docs/PINS.md section 3.
 *
 * MDIO stays configured as an open-drain output for the whole exchange: a
 * written 1 releases the line, so the PHY can drive it during the read phase
 * and the same pin reads back through IDR.  That removes the mode switch at
 * the turnaround, which is where bit-banged MDIO usually goes wrong.
 *
 * Frame layout (clause 22): 32x preamble, ST=01, OP, PHYAD[4:0], REGAD[4:0],
 * TA, DATA[15:0].  Clause 45 uses ST=00 with a separate address cycle so it
 * can reach the 16-bit register space of a PMA/PMD or PCS device.
 */
#include "bb_mdio.h"
#include "bb_board.h"
#include "bb_status.h"
#include <stdio.h>
#include "bb_cmd.h"
#include "bb_console.h"

#define MDIO_DEFAULT_HZ    1000000UL
#define MDIO_MIN_HZ          10000UL
#define MDIO_MAX_HZ        2500000UL

/* Clause 22 opcodes */
#define OP22_WRITE  0x1u
#define OP22_READ   0x2u
/* Clause 45 opcodes */
#define OP45_ADDR       0x0u
#define OP45_WRITE      0x1u
#define OP45_READ_INC   0x2u
#define OP45_READ       0x3u

static uint32_t mdc_hz = MDIO_DEFAULT_HZ;
static uint32_t half_cycles;             /* timer ticks per half MDC period */
static bool     suppress_preamble;

/* ---- timing ------------------------------------------------------------ */

static void recompute_timing(void)
{
    half_cycles = bb_cycle_hz() / (mdc_hz * 2u);
    if (half_cycles < 4u) {
        half_cycles = 4u;
    }
}

static inline void half_period(void)
{
    uint32_t start = bb_cycles();
    while ((bb_cycles() - start) < half_cycles) {
    }
}

/* ---- line primitives --------------------------------------------------- */

static const bb_pin_t s_mdc  = BB_MDC;
static const bb_pin_t s_mdio = BB_MDIO;

static inline void mdc_set(bool high)
{
    bb_pin_write(&s_mdc, high);
}

static inline void mdio_drive(bool high)
{
    bb_pin_write(&s_mdio, high);
}

static inline bool mdio_read(void)
{
    /* Reads back through the same pad it drives, which is what open drain plus
     * SION is for. That removes the direction switch at the turnaround, and the
     * turnaround is where bit-banged MDIO usually goes wrong. */
    return bb_pin_read(&s_mdio);
}

/** Emit @p nbits of @p value, MSB first. Data changes while MDC is low. */
static void write_bits(uint32_t value, unsigned nbits)
{
    for (unsigned i = 0; i < nbits; i++) {
        bool bit = (value >> (nbits - 1u - i)) & 1u;
        mdc_set(false);
        mdio_drive(bit);
        half_period();
        mdc_set(true);
        half_period();
    }
}

/** Clock in @p nbits, sampling just after each rising edge. */
static uint32_t read_bits(unsigned nbits)
{
    uint32_t value = 0;

    mdio_drive(true);                   /* release the line to the PHY */
    for (unsigned i = 0; i < nbits; i++) {
        mdc_set(false);
        half_period();
        mdc_set(true);
        value = (value << 1) | (mdio_read() ? 1u : 0u);
        half_period();
    }
    return value;
}

/*
 * MDC and MDIO are re-claimed, before every frame.
 *
 * Muxing at configure time alone is not enough on a board where pads are
 * shared. The sniffer takes a pad by muxing it to FlexIO, which is exactly what
 * SNIFF:CH is for and is allowed by design - but nothing handed it back, so a
 * bus whose pads had been sniffed stayed broken until the board was reset. It
 * failed quietly, too: after its pads were sniffed, a floating-bus read returned 0x0000 instead of 0xFFFF - a perfectly plausible register value, and indistinguishable from a PHY that answered.
 *
 * Re-muxing here is the same rule the rest of the board already follows - the
 * capture engine muxes the taps it wants when it claims them, and the UART does
 * the same for flow control - applied in the other direction. Whoever is about
 * to use a pad claims it. The cost is a couple of register writes against a
 * transaction that takes microseconds at least.
 */
static void mdio_claim_pads(void)
{
    /* MDC push-pull: the clock is ours alone and wants clean fast edges. MDIO
     * open drain plus pull-up, so driving 1 releases the line for the PHY and
     * the same pad reads back. Both match bb_mdio_init(), and both are applied
     * at a frame boundary where re-asserting the idle level is harmless. */
    bb_pin_gpio(&s_mdc, true, true, BB_PAD_PUSHPULL);
    bb_pin_gpio(&s_mdio, true, true, BB_PAD_I2C);
}

static void preamble(void)
{
    mdio_claim_pads();
    if (!suppress_preamble) {
        write_bits(0xFFFFFFFFu, 32);
    }
}

/** Two idle bit times with the line released, between frames. */
static void idle(void)
{
    mdio_drive(true);
    mdc_set(false);
    half_period();
    mdc_set(true);
    half_period();
}

/* ---- clause 22 --------------------------------------------------------- */

static void c22_write(uint8_t phy, uint8_t reg, uint16_t data)
{
    preamble();
    write_bits(0x1u, 2);                /* ST = 01 */
    write_bits(OP22_WRITE, 2);
    write_bits(phy, 5);
    write_bits(reg, 5);
    write_bits(0x2u, 2);                /* TA = 10, driven by the station */
    write_bits(data, 16);
    idle();
}

static uint16_t c22_read(uint8_t phy, uint8_t reg)
{
    preamble();
    write_bits(0x1u, 2);                /* ST = 01 */
    write_bits(OP22_READ, 2);
    write_bits(phy, 5);
    write_bits(reg, 5);
    /* TA: one released bit, then the PHY drives a 0.  read_bits releases the
     * line, so clocking two bits and ignoring them covers the turnaround. */
    (void)read_bits(2);
    uint16_t data = (uint16_t)read_bits(16);
    idle();
    return data;
}

/* ---- clause 45 --------------------------------------------------------- */

static void c45_address(uint8_t port, uint8_t dev, uint16_t reg)
{
    preamble();
    write_bits(0x0u, 2);                /* ST = 00 */
    write_bits(OP45_ADDR, 2);
    write_bits(port, 5);
    write_bits(dev, 5);
    write_bits(0x2u, 2);
    write_bits(reg, 16);
    idle();
}

static void c45_write(uint8_t port, uint8_t dev, uint16_t reg, uint16_t data)
{
    c45_address(port, dev, reg);
    preamble();
    write_bits(0x0u, 2);
    write_bits(OP45_WRITE, 2);
    write_bits(port, 5);
    write_bits(dev, 5);
    write_bits(0x2u, 2);
    write_bits(data, 16);
    idle();
}

static uint16_t c45_read(uint8_t port, uint8_t dev, uint16_t reg)
{
    c45_address(port, dev, reg);
    preamble();
    write_bits(0x0u, 2);
    write_bits(OP45_READ, 2);
    write_bits(port, 5);
    write_bits(dev, 5);
    (void)read_bits(2);
    uint16_t data = (uint16_t)read_bits(16);
    idle();
    return data;
}

void bb_mdio_init(void)
{
    recompute_timing();

    /* MDC push-pull: the clock is ours alone and wants clean fast edges. */
    bb_pin_gpio(&s_mdc, true, true, BB_PAD_PUSHPULL);

    /* MDIO open drain plus pull-up, so driving 1 releases the line for the PHY
     * to drive during the read phase, and the same pad reads back. */
    bb_pin_gpio(&s_mdio, true, true, BB_PAD_I2C);

    mdc_set(true);
    mdio_drive(true);
}

/* ---- commands ---------------------------------------------------------- */

static void cmd_clk(int argc, char **argv)
{
    uint32_t hz;

    (void)argc;
    if (!bb_parse_u32(argv[1], &hz) || hz < MDIO_MIN_HZ || hz > MDIO_MAX_HZ) {
        bb_reply_errf(BB_ERR_RANGE, "hz-%lu..%lu",
                      (unsigned long)MDIO_MIN_HZ, (unsigned long)MDIO_MAX_HZ);
        return;
    }
    mdc_hz = hz;
    recompute_timing();
    bb_reply_okf("hz=%lu", (unsigned long)mdc_hz);
}

static void cmd_clk_q(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    bb_reply_okf("hz=%lu tick_hz=%lu preamble=%s", (unsigned long)mdc_hz,
                 (unsigned long)bb_cycle_hz(), suppress_preamble ? "off" : "on");
}

static void cmd_preamble(int argc, char **argv)
{
    uint32_t on;

    (void)argc;
    if (!bb_parse_u32_max(argv[1], 1, &on)) {
        bb_reply_err(BB_ERR_RANGE, "0-or-1");
        return;
    }
    suppress_preamble = (on == 0u);
    bb_reply_ok();
}

static void cmd_c22_read(int argc, char **argv)
{
    uint32_t phy;
    uint32_t reg;

    (void)argc;
    if (!bb_parse_u32_max(argv[1], 31, &phy) || !bb_parse_u32_max(argv[2], 31, &reg)) {
        bb_reply_err(BB_ERR_RANGE, "phy-0..31-reg-0..31");
        return;
    }
    uint16_t v = c22_read((uint8_t)phy, (uint8_t)reg);
    bb_reply_okf("0x%04X", v);
}

static void cmd_c22_write(int argc, char **argv)
{
    uint32_t phy;
    uint32_t reg;
    uint32_t val;

    (void)argc;
    if (!bb_parse_u32_max(argv[1], 31, &phy) || !bb_parse_u32_max(argv[2], 31, &reg) ||
        !bb_parse_u32_max(argv[3], 0xFFFFu, &val)) {
        bb_reply_err(BB_ERR_RANGE, "phy-0..31-reg-0..31-val-16bit");
        return;
    }
    c22_write((uint8_t)phy, (uint8_t)reg, (uint16_t)val);
    bb_reply_ok();
}

static void cmd_c45_read(int argc, char **argv)
{
    uint32_t port;
    uint32_t dev;
    uint32_t reg;

    (void)argc;
    if (!bb_parse_u32_max(argv[1], 31, &port) || !bb_parse_u32_max(argv[2], 31, &dev) ||
        !bb_parse_u32_max(argv[3], 0xFFFFu, &reg)) {
        bb_reply_err(BB_ERR_RANGE, "port-0..31-dev-0..31-reg-16bit");
        return;
    }
    uint16_t v = c45_read((uint8_t)port, (uint8_t)dev, (uint16_t)reg);
    bb_reply_okf("0x%04X", v);
}

static void cmd_c45_write(int argc, char **argv)
{
    uint32_t port;
    uint32_t dev;
    uint32_t reg;
    uint32_t val;

    (void)argc;
    if (!bb_parse_u32_max(argv[1], 31, &port) || !bb_parse_u32_max(argv[2], 31, &dev) ||
        !bb_parse_u32_max(argv[3], 0xFFFFu, &reg) ||
        !bb_parse_u32_max(argv[4], 0xFFFFu, &val)) {
        bb_reply_err(BB_ERR_RANGE, "port-dev-0..31-reg-val-16bit");
        return;
    }
    c45_write((uint8_t)port, (uint8_t)dev, (uint16_t)reg, (uint16_t)val);
    bb_reply_ok();
}

/**
 * @brief Report which PHY addresses answer.
 *
 * A floating bus reads back all ones, and an absent address usually reads
 * 0xFFFF, so an ID register pair that is neither all-ones nor all-zeros is the
 * practical test for "something is there".
 */
static void cmd_scan(int argc, char **argv)
{
    uint8_t found[32];
    size_t  n = 0;

    (void)argc;
    (void)argv;
    for (uint8_t phy = 0; phy < 32u; phy++) {
        uint16_t id1 = c22_read(phy, 2);
        uint16_t id2 = c22_read(phy, 3);
        if (!(id1 == 0xFFFFu && id2 == 0xFFFFu) && !(id1 == 0u && id2 == 0u)) {
            found[n++] = phy;
        }
    }
    bb_reply_ok_bytes(found, n);
}

/** Dump the 32 clause-22 registers of one PHY in one round trip. */
static void cmd_dump(int argc, char **argv)
{
    uint32_t phy;

    (void)argc;
    if (!bb_parse_u32_max(argv[1], 31, &phy)) {
        bb_reply_err(BB_ERR_RANGE, "phy-0..31");
        return;
    }
    uint8_t regs[64];
    for (unsigned r = 0; r < 32u; r++) {
        uint16_t v = c22_read((uint8_t)phy, (uint8_t)r);
        regs[2u * r]      = (uint8_t)(v >> 8);
        regs[2u * r + 1u] = (uint8_t)v;
    }
    bb_reply_ok_bytes(regs, sizeof regs);
}

void bb_mdio_status(bb_status_t *out)
{
    if (out == NULL) {
        return;
    }
    snprintf(out->pins, sizeof out->pins, "MDC/MDIO");
    snprintf(out->detail, sizeof out->detail, "%lukHz",
             (unsigned long)(mdc_hz / 1000u));
    out->state = BB_STATE_IDLE;
}

const bb_cmd_t bb_cmd_mdio[] = {
    { "MDIO:CLK",      1, 1, cmd_clk,       "MDIO:CLK <hz>" },
    { "MDIO:CLK?",     0, 0, cmd_clk_q,     "MDIO:CLK?" },
    { "MDIO:PREAMBLE", 1, 1, cmd_preamble,  "MDIO:PREAMBLE <0|1>" },
    { "MDIO:C22:R",    2, 2, cmd_c22_read,  "MDIO:C22:R <phy> <reg>" },
    { "MDIO:C22:W",    3, 3, cmd_c22_write, "MDIO:C22:W <phy> <reg> <val16>" },
    { "MDIO:C45:R",    3, 3, cmd_c45_read,  "MDIO:C45:R <port> <dev> <reg16>" },
    { "MDIO:C45:W",    4, 4, cmd_c45_write, "MDIO:C45:W <port> <dev> <reg16> <val16>" },
    { "MDIO:SCAN",     0, 0, cmd_scan,      "MDIO:SCAN" },
    { "MDIO:DUMP",     1, 1, cmd_dump,      "MDIO:DUMP <phy>" },
    { NULL, 0, 0, NULL, NULL }
};
