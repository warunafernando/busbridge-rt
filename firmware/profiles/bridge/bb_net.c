/**
 * @file bb_net.c
 * @brief NET:* — the on-board Ethernet PHYs, over hardware MDIO.
 *
 * Phase 6 is Ethernet, TSN and EtherCAT, and the spec is explicit that the
 * whole of it deserves its own document once the rest is real. This file is not
 * that. It is the one slice of Phase 6 that is genuinely bounded, immediately
 * useful, and impossible to do any other way on this board.
 *
 * **Why this slice.** `docs/PINS.md` established that `EMDIO` reaches none of
 * the 22 Arduino pads - the hardware MDIO in NETC is wired to the board's own
 * PHYs and can talk to nothing else. That is why `MDIO:*` in `bb_mdio.c` is
 * bit-banged on D8/D9. But the corollary is the useful half: the hardware MDIO
 * *can* reach the two Motorcomm YT8531 gigabit PHYs (U21 and U29), and nothing
 * else can. Without this, those PHYs are unreachable from the command protocol.
 *
 * So `NET:*` is where the board's own network hardware is inspected, and
 * `MDIO:*` stays where an external PHY is driven. Two namespaces because they
 * are two different pieces of hardware reaching two different places - not
 * because one is a better version of the other.
 *
 * **What is deliberately not here.** No frame transmission, no TSN scheduling,
 * no EtherCAT. Those need the switch, the port descriptors and the ENETC
 * datapath brought up, and each is a project. Reading a PHY's identity and link
 * state is the foundation you want before any of that - if MDIO to the PHY does
 * not work, nothing above it will, and you will spend the debugging time in the
 * wrong layer.
 */
#include "bb_board.h"
#include "bb_cmd.h"
#include "bb_io.h"
#include "bb_net.h"

#include "fsl_clock.h"
#include "fsl_netc_mdio.h"

#include <stdio.h>
#include <string.h>

/* Clause-22 registers every PHY has, whatever the vendor. */
#define PHY_REG_BMCR    0x00u   /* basic mode control    */
#define PHY_REG_BMSR    0x01u   /* basic mode status     */
#define PHY_REG_ID1     0x02u   /* OUI high              */
#define PHY_REG_ID2     0x03u   /* OUI low, model, rev   */

#define BMSR_LINK_UP    0x0004u
#define BMSR_AN_DONE    0x0020u

static netc_mdio_handle_t s_mdio;
static bool               s_ready;

/*
 * The two PHYs this board fits, from the schematic: U21 is PHY2 on ETH0 and
 * U29 is PHY3 on ETH2, both Motorcomm YT8531SH-CA behind RJ45s J56A/J56B.
 *
 * Their MDIO addresses are a strapping choice and the schematic netlist does
 * not spell them out in a form worth trusting here, so `NET:SCAN` finds them
 * rather than this file asserting them. That is the same discipline the pin map
 * follows: publish what was found, do not hardcode what was assumed.
 */
#define NET_PHY_MAX     32u

static void net_init_once(void)
{
    netc_mdio_config_t cfg;

    if (s_ready) {
        return;
    }
    memset(&cfg, 0, sizeof cfg);
    /* EMDIO rather than the internal or per-port MDIO: this is the instance
     * wired to the board's PHYs. `netc_mdio_t` is a struct rather than a plain
     * enum because the port-MDIO case needs a port index alongside the type;
     * for EMDIO the port field is not meaningful and stays zero. */
    cfg.mdio.type         = kNETC_EMdio;
    cfg.srcClockHz        = CLOCK_GetRootClockFreq(kCLOCK_Root_Netc);
    cfg.isNegativeDriven  = false;
    cfg.isPreambleDisable = false;

    s_ready = (NETC_MDIOInit(&s_mdio, &cfg) == kStatus_Success);
}

static bool net_ready(void)
{
    net_init_once();
    if (!s_ready) {
        bb_reply_err(BB_ERR_HAL, "netc-mdio-init-failed");
        return false;
    }
    return true;
}

/* ---- commands ---------------------------------------------------------- */

static void cmd_phy_r(int argc, char **argv)
{
    uint32_t phy, reg;
    uint16_t val = 0u;

    (void)argc;
    if (!bb_parse_u32_max(argv[1], 31u, &phy) ||
        !bb_parse_u32_max(argv[2], 31u, &reg)) {
        bb_reply_err(BB_ERR_RANGE, "phy-0..31-reg-0..31");
        return;
    }
    if (!net_ready()) {
        return;
    }
    if (NETC_MDIORead(&s_mdio, (uint8_t)phy, (uint8_t)reg, &val) != kStatus_Success) {
        /* Distinguished from a register that reads 0xFFFF. A read that did not
         * complete is "nothing answered"; 0xFFFF from a floating bus is a
         * successful read of a bus with nobody on it. The bit-banged MDIO makes
         * the same distinction and for the same reason - conflating them sends
         * you looking for a dead PHY when the MDC clock is simply not running. */
        bb_reply_err(BB_ERR_TIMEOUT, "mdio-read-no-response");
        return;
    }
    bb_reply_okf("%04X", (unsigned)val);
}

static void cmd_phy_w(int argc, char **argv)
{
    uint32_t phy, reg, val;

    (void)argc;
    if (!bb_parse_u32_max(argv[1], 31u, &phy) ||
        !bb_parse_u32_max(argv[2], 31u, &reg) ||
        !bb_parse_u32_max(argv[3], 0xFFFFu, &val)) {
        bb_reply_err(BB_ERR_RANGE, "phy-reg-val16");
        return;
    }
    if (!net_ready()) {
        return;
    }
    if (NETC_MDIOWrite(&s_mdio, (uint8_t)phy, (uint8_t)reg,
                       (uint16_t)val) != kStatus_Success) {
        bb_reply_err(BB_ERR_TIMEOUT, "mdio-write-no-response");
        return;
    }
    bb_reply_ok();
}

static void cmd_scan(int argc, char **argv)
{
    unsigned found = 0;

    (void)argc;
    (void)argv;
    if (!net_ready()) {
        return;
    }
    bb_io_puts("OK");
    for (uint8_t a = 0; a < NET_PHY_MAX; a++) {
        uint16_t id1 = 0u, id2 = 0u;

        if (NETC_MDIORead(&s_mdio, a, PHY_REG_ID1, &id1) != kStatus_Success) {
            continue;
        }
        /* 0x0000 and 0xFFFF are both "nothing there": an unstrapped address
         * floats high and an unpowered one reads low. Neither is an OUI. */
        if (id1 == 0x0000u || id1 == 0xFFFFu) {
            continue;
        }
        (void)NETC_MDIORead(&s_mdio, a, PHY_REG_ID2, &id2);
        bb_io_printf(" %u:id=%04X%04X", (unsigned)a, (unsigned)id1, (unsigned)id2);
        found++;
    }
    if (found == 0u) {
        bb_io_puts(" none");
    }
    bb_io_puts("\r\n");
}

static void cmd_link_q(int argc, char **argv)
{
    uint32_t phy;
    uint16_t bmsr = 0u;

    (void)argc;
    if (!bb_parse_u32_max(argv[1], 31u, &phy)) {
        bb_reply_err(BB_ERR_RANGE, "phy-0..31");
        return;
    }
    if (!net_ready()) {
        return;
    }
    /*
     * BMSR's link bit is latching-low on most PHYs: it reports "link has been
     * down since you last read this", not "link is down now". So it is read
     * twice and the second value reported, which is the standard way to get the
     * current state rather than the sticky one. Reporting the first read makes a
     * link that came up look down exactly once, which is maddening to chase.
     */
    if (NETC_MDIORead(&s_mdio, (uint8_t)phy, PHY_REG_BMSR, &bmsr) != kStatus_Success ||
        NETC_MDIORead(&s_mdio, (uint8_t)phy, PHY_REG_BMSR, &bmsr) != kStatus_Success) {
        bb_reply_err(BB_ERR_TIMEOUT, "mdio-read-no-response");
        return;
    }
    bb_reply_okf("link=%u autoneg=%u bmsr=%04X",
                 (bmsr & BMSR_LINK_UP) ? 1u : 0u,
                 (bmsr & BMSR_AN_DONE) ? 1u : 0u,
                 (unsigned)bmsr);
}

static void cmd_ports_q(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    /*
     * What the board has, from the schematic, so a host knows what it is looking
     * at before it scans. The PHY addresses are deliberately absent - NET:SCAN
     * reports what actually answered, and putting a guess here would be a second
     * source of truth that could disagree with the hardware.
     */
    bb_reply_okf("eth0=U21/YT8531/J56A eth2=U29/YT8531/J56B "
                 "ecat0=J57A ecat1=J57B mdio=netc-emdio "
                 "note=frames-tsn-ecat-not-implemented");
}

/* ---- table ------------------------------------------------------------- */

const bb_cmd_t bb_cmd_net[] = {
    { "NET:PORTS?",  0, 0, cmd_ports_q, "NET:PORTS?" },
    { "NET:SCAN",    0, 0, cmd_scan,    "NET:SCAN" },
    { "NET:PHY:R",   2, 2, cmd_phy_r,   "NET:PHY:R <phy> <reg>" },
    { "NET:PHY:W",   3, 3, cmd_phy_w,   "NET:PHY:W <phy> <reg> <val16>" },
    { "NET:LINK?",   1, 1, cmd_link_q,  "NET:LINK? <phy>" },
    { NULL, 0, 0, NULL, NULL }
};
