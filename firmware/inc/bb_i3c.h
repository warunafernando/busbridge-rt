/**
 * @file bb_i3c.h
 * @brief I3C1 in either role, controller or target, on the board's own I3C
 *        pins. Not shared with I2C1 any more; see bb_board.h for the pins,
 *        which differ per board.
 */
#ifndef BB_I3C_H
#define BB_I3C_H

void bb_i3c_init(void);

/*
 * Service anything the bus raised on its own - an in-band interrupt or a
 * hot-join. Called from the profile poll, because a target does not wait for a
 * host command before asking for attention.
 */
void bb_i3c_poll(void);

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/** Release the peripheral so I2C can take the pins. Called by the arbiter. */
void bb_i3c_release(void);

/** GPDMA channel interrupt for the target's transmit path. */
void bb_i3c_dma_irq(void);

/**
 * @brief Whether the I3C peripheral currently owns its pins.
 *
 * Asked by the sniffer, not by anything doing I3C. When the taps are jumpered
 * across to this board's own I3C pins - the usual way to watch your own
 * traffic - a configured controller drives the very wires the self test wants
 * to drive, and the test fails for a reason that has nothing to do with the
 * decoder. Knowing that lets the failure say so.
 */
bool bb_i3c_configured(void);

void bb_i3c_ev_irq(void);
void bb_i3c_er_irq(void);

/* ---- driving the target from firmware rather than from a host ------------ */
/* The bridge profile arms target replies from I3C:TGT:TX and drains received
 * bytes with I3C:TGT:RX?, one command at a time. The peripheral profile has no
 * host in the loop - an EEPROM answers a read nobody prepared it for - so the
 * same machinery is available directly.
 *
 * Everything below is target-role only and returns false or zero otherwise. */

/** Put the peripheral into the target role, rebuilt from scratch. */
bool bb_i3c_tgt_begin(void);

/** Load @p len bytes for the controller's next private read. */
bool bb_i3c_tgt_arm(const uint8_t *data, size_t len);

/** True while a loaded reply has not been read. */
bool bb_i3c_tgt_armed(void);

/**
 * @brief How many of the armed bytes the controller actually clocked out.
 *
 * From the DMA channel's remaining count. An auto-incrementing address pointer
 * must advance by this and not by the length that was armed: a controller that
 * reads four bytes of a thirty-two byte arm has moved the pointer four.
 */
uint16_t bb_i3c_tgt_tx_consumed(void);

/** Completed private writes seen. A change means a write frame ended. */
uint32_t bb_i3c_tgt_write_frames(void);

/** Take received bytes out of the ring. Returns how many. */
size_t bb_i3c_tgt_take(uint8_t *out, size_t max);

/** The dynamic address ENTDAA gave this target, or 0 if it has none yet. */
uint8_t bb_i3c_tgt_address(void);

#endif /* BB_I3C_H */
