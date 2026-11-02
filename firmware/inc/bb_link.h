/**
 * @file bb_link.h
 * @brief The CM33 <-> CM7 link, as seen by the controller core.
 *
 * The capture engine lives on the CM7 and the command protocol lives on the
 * CM33, so every `SNIFF:*` command is a request across this link. Wrapped in
 * one file for two reasons:
 *
 *   - the sniffer command layer should read as request/response, not as
 *     RPMsg plumbing; and
 *   - the link can be **absent**, and that has to be a first-class state rather
 *     than a hang. The CM7 image may not have been loaded, may have crashed, or
 *     may never have been built. In all three cases `bb_link_up()` is false and
 *     every command answers "capture core not running" immediately.
 *
 * That last point is the one worth insisting on. A sniffer whose commands block
 * forever when the other core is missing is indistinguishable from a sniffer
 * that is quietly wrong, and the STM32 build's hardest bug - a decoder that
 * reported `stops=0` while decoding every byte correctly - was found precisely
 * because the failure was loud and specific rather than a silence.
 */
#ifndef BB_LINK_H
#define BB_LINK_H

#include "bb_capture.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/**
 * @brief Bring up the link. Safe to call when the CM7 is not running.
 *
 * Does not wait for the far side. `bb_link_up()` becomes true when the CM7
 * answers a ping, which it cannot do before it has booted - and the CM33 boots
 * first by design, since it is the core that releases the CM7.
 */
void bb_link_init(void);

/** True when the capture core has answered at least one ping and not gone away. */
bool bb_link_up(void);

/** One-line description of the far side, or why there isn't one. */
const char *bb_link_status(void);

/**
 * @brief Raw shared-memory handshake words, for diagnosing a dead link.
 *
 * Reports what is in the mailbox rather than the conclusion drawn from it -
 * which is the difference between "the capture core is not running" and knowing
 * why it is not.
 */
void bb_link_raw(uint32_t *magic, uint32_t *alive, uint32_t *ticks);

/**
 * @brief The capture core's beacon, read from its ITCM rather than the mailbox.
 *
 * Answers "is that core executing" without depending on the shared memory that
 * is the most likely thing to be broken.
 */
void bb_link_beacon(uint32_t *magic, uint32_t *ticks, uint32_t *saw_magic);

/** @brief How far the capture core's last long operation got. */
uint32_t bb_link_beacon_phase(void);

/**
 * @brief Whether the capture core has been given a clock.
 *
 * Everything that reads that core's memory - the beacon and its phase - returns
 * zeros until this is true, because the read would stall rather than fail. A
 * caller reporting on the core needs to tell "absent" apart from "silent".
 */
bool bb_link_cm7_clocked(void);

/** @brief M7 control registers as the last release attempt found and left them. */
void bb_link_m7_state(uint32_t *cfg_before, uint32_t *cfg_after,
                      uint32_t *src_scr, uint32_t *itcm_w0, uint32_t *itcm_w1);

/**
 * @brief The M7's root clock as the controller left it, and the enclave's answer.
 *
 * root_hz of 0 means the core has no clock and cannot run whatever else is
 * configured; apc_status is the ELE_BaseAPI_EnableAPC() return, -1 if it was
 * never attempted.
 */
void bb_link_m7_clock(uint32_t *root_hz, int *apc_status);

/** Outcome of trying to bring the capture core up. */
typedef enum {
    BB_CM7_STARTED = 0,     /**< loaded, released, and it answered */
    BB_CM7_ALREADY_UP,      /**< it was already running */
    BB_CM7_STARTED_NO_ELE,  /**< it runs, but the enclave never acknowledged */
    BB_CM7_NO_IMAGE,        /**< this build carries no capture image */
    BB_CM7_COPY_FAILED,     /**< the ITCM window did not read back what was written */
    BB_CM7_ELE_TIMEOUT,     /**< the EdgeLock Enclave did not respond */
    BB_CM7_NO_RESPONSE      /**< released, but nothing came back */
} bb_cm7_start_t;

/**
 * @brief Copy the embedded capture image into the CM7's ITCM and start it.
 *
 * Safe to call unconditionally: this core supplies the image, so it knows what
 * it is starting, and the copy is read back before the core is released. That is
 * the whole reason the image is embedded rather than left to a debugger - it
 * turns "start whatever happens to be in ITCM" into a defined operation.
 */
bb_cm7_start_t bb_link_start_cm7(void);

/** @brief S3MU status registers as the last release attempt found them. */
void bb_link_ele_state(uint32_t *tsr, uint32_t *rsr);

/**
 * @brief Send a request and wait for its reply.
 *
 * @param op        one of bb_msg_op_t
 * @param req       request payload, may be NULL
 * @param req_len   its length
 * @param reply     buffer for the reply payload, may be NULL
 * @param reply_cap its capacity
 * @param reply_len receives the reply length
 * @return BB_CAP_OK, or the far side's error, or BB_CAP_ERR_UNSUPPORTED when
 *         the link is down. Never blocks indefinitely: a request that goes
 *         unanswered times out and is reported as such.
 */
bb_cap_err_t bb_link_request(bb_msg_op_t op,
                             const void *req, size_t req_len,
                             void *reply, size_t reply_cap, size_t *reply_len);

/** Called from the poll hook: services the link and refreshes liveness. */
void bb_link_poll(void);

#endif /* BB_LINK_H */
