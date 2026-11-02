/**
 * @file bb_mdio.h
 * @brief Bit-banged MDIO (IEEE 802.3 clause 22 and clause 45) on D8/D9.
 *
 * The H5 has no MDIO controller outside the Ethernet MAC, and the MAC's SMI is
 * tied to the MAC pins rather than the Arduino header, so this is a GPIO
 * implementation.  MDC tops out around 2.5 MHz, which bit-banging reaches
 * comfortably at 250 MHz core clock.
 */
#ifndef BB_MDIO_H
#define BB_MDIO_H

void bb_mdio_init(void);

#endif /* BB_MDIO_H */
