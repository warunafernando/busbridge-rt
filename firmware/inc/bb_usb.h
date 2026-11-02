/**
 * @file bb_usb.h
 * @brief Native USB device on the user USB-C connector (CN13, USB_DRD_FS).
 *
 * A vendor-specific interface with one bulk OUT and one bulk IN endpoint,
 * carrying the same line protocol as the serial console. Full-speed bulk is
 * 12 Mbit/s against the VCP's 115200 baud, and a command costs one USB
 * transaction instead of a UART round trip.
 *
 * The device advertises itself as WinUSB-compatible through Microsoft OS 2.0
 * descriptors, so Windows loads its own signed winusb.sys automatically on
 * first plug-in. No INF file, no driver signing, nothing to install.
 *
 * PA11/PA12 reach CN13 by default on this board (UM3115 table 14: SB21/SB22
 * OFF, SB27/SB28 ON), so no rework is needed.
 */
#ifndef BB_USB_H
#define BB_USB_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/** Bring up the USB device. Safe to call when no host is attached. */
void bb_usb_init(void);

/** True once the host has configured the device and bulk transfers can flow. */
bool bb_usb_ready(void);

/** Queue data for the host. Blocks briefly if the IN path is still busy. */
void bb_usb_write(const void *data, size_t len);

/** Non-blocking byte read from the host. -1 when nothing is buffered. */
int bb_usb_getchar(void);

/**
 * @brief Non-blocking line reader, mirroring bb_console_getline().
 * @return line length, -1 when no complete line is ready, -2 on overflow
 */
int bb_usb_getline(char *buf, size_t cap);

/** Bytes transferred since boot, for the throughput counters. */
void bb_usb_stats(uint32_t *rx_bytes, uint32_t *tx_bytes, uint32_t *resets);

/**
 * @brief Control-path counters: BOS reads, MS OS 2.0 reads, stalls.
 *
 * These answer the question a failed driver bind cannot: did the host ever
 * ask for the descriptors that select WinUSB?
 */
void bb_usb_diag(uint32_t *bos, uint32_t *msos, uint32_t *stalls, uint16_t *last);

/**
 * @brief The link's physical state, straight out of the peripheral.
 *
 * Distinguishes "no host has spoken to us yet" from "there is no host on the
 * other end of this cable", which the protocol counters cannot: both are zero.
 *
 * @param dppu  non-zero when this device is pulling D+ up, asking to be seen
 * @param frame the peripheral's frame counter; it advances once a millisecond
 *              while a host is present, and never moves if the bus is dead
 * @param istr  latched interrupt flags
 */
void bb_usb_link(uint32_t *dppu, uint32_t *frame, uint32_t *istr);

/**
 * @brief Measure the Type-C CC lines.
 *
 * Brings UCPD up as a sink just far enough to read the CC comparators, which is
 * the only way from software to tell "no host has spoken to us" apart from
 * "there is nothing plugged into this connector".
 *
 * Each vstate is the raw two-bit TYPEC_VSTATE field: 0 is the lowest level,
 * meaning nothing is pulling that line up. A host or a USB-A adapter cable
 * shows a non-zero level on exactly one of the two, whichever way round the
 * plug went in.
 *
 * @return false when this board has no Type-C port wired to UCPD
 */
bool bb_usb_typec(uint32_t *cc1, uint32_t *cc2);

/** Packets and bytes of the most recent MS OS 2.0 control reply. */
void bb_usb_ep0_diag(uint16_t *packets, uint16_t *bytes);

/**
 * @brief Enumeration event trace, newest event in the low nibble of @p events.
 *
 * Diagnostic. Reports the order in which the host asked for things rather than
 * how many times, which is what distinguishes "a descriptor is malformed" from
 * "SET_ADDRESS never landed" - two failures that look identical in the counts.
 * See the TRC_* codes in bb_usb.c for the nibble values.
 */
void bb_usb_trace(uint64_t *events, uint32_t *dev_desc, uint32_t *cfg_desc,
                  uint32_t *state);

/** @brief The last vendor setup packet as received, and the length answered. */
void bb_usb_vendor_diag(uint32_t *reqtype, uint32_t *windex, uint32_t *wlength,
                        uint32_t *sent);

/** Called from the USB_DRD_FS vector. */
void bb_usb_irq(void);

#endif /* BB_USB_H */
