/**
 * @file bb_can.h
 * @brief FlexCAN1 on the on-board transceiver.
 *
 * New to this project: the NUCLEO had no CAN transceiver, so the sniffer's CAN
 * decoder has never seen real traffic. Two board conditions have to hold before
 * frames appear - jumpers J44/J46 in position 1-2, and a header fitted to the
 * DNP connector J22. `CAN:CFG` reports the second in its reply, because a
 * missing connector looks exactly like a firmware fault.
 *
 * Listen-only is a hardware mode here, not a firmware convention: the receiver
 * acknowledges nothing and transmits nothing, so attaching to a live bus cannot
 * disturb it.
 */
#ifndef BB_CAN_H
#define BB_CAN_H

#include "bb_status.h"

void bb_can_init(void);

/** Drain the receive message buffer into the ring. Called from the poll hook. */
void bb_can_poll(void);

void bb_can_status(bb_status_t *out);

#endif /* BB_CAN_H */
