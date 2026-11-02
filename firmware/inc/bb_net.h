/**
 * @file bb_net.h
 * @brief NET:* — the board's own Ethernet PHYs, over hardware MDIO.
 *
 * Separate from MDIO:* because it is separate hardware reaching a separate
 * place. NETC's EMDIO is wired to the two on-board YT8531 PHYs and to nothing
 * else; the Arduino headers cannot reach it. So NET:* inspects this board's
 * network hardware and MDIO:* drives an external PHY, and neither is a better
 * version of the other.
 *
 * This is the bounded first slice of Phase 6. Frames, TSN scheduling and
 * EtherCAT are not here - each needs the switch and the ENETC datapath brought
 * up, and reading a PHY's identity and link state is the thing you want working
 * before any of them.
 */
#ifndef BB_NET_H
#define BB_NET_H

#endif /* BB_NET_H */
