/*
 * bb_winusb_hs.c — WinUSB transport, tuned for a high-speed device.
 *
 * A drop-in replacement for the STM32 tree's windows/src/bb_winusb.c. Same six
 * entry points, same handle fields, so busbridge.c and the public API are
 * unchanged. The STM32 tree itself is left untouched.
 *
 * ============================================================================
 * WHAT ACTUALLY LIMITS THROUGHPUT, AND WHY THE INF IS NOT IT
 * ============================================================================
 *
 * `driver/busbridge.inf` sets PortSubClass and a `serenum` upper filter. That
 * is the *CDC serial* device - the ST-LINK VCP path that was retired at 692
 * round trips a second. WinUSB needs no INF at all: the device's Microsoft
 * OS 2.0 descriptors bind it. So there is nothing in the driver package to tune
 * and no registry policy that raises the ceiling.
 *
 * The old transport's real limits were these, in order of size:
 *
 *   1. ONE TRANSFER IN FLIGHT. It issued a 512-byte overlapped read, waited for
 *      it, then issued the next. USB is scheduled per frame, so with one
 *      outstanding transfer the device gets at most one packet per frame served
 *      - the link rate is irrelevant. On full speed (1 ms frames) that is the
 *      whole story behind 110 kB/s. On high speed (125 us microframes) the same
 *      code would reach roughly 8x that and stop, because the bottleneck moved
 *      but did not go away. **This is the single biggest lever and it is here,
 *      on the host, not in the firmware.**
 *
 *   2. A 512-BYTE BUFFER. On full speed that is 8 packets of 64. On high speed
 *      it is *one* packet of 512 - so the same buffer that batched eight
 *      transfers' worth of work now batches one. Read size has to grow with the
 *      packet size or the transfer count silently multiplies.
 *
 *   3. NO RAW_IO. Without it WinUSB buffers and re-splits transfers in the
 *      kernel. With it, submissions go straight to the host controller.
 *
 * Fixing 1 and 2 is what this file is for. Fixing 3 is two lines.
 *
 * ============================================================================
 * WHAT WILL NOT IMPROVE 40x, AND SAYING SO UP FRONT
 * ============================================================================
 *
 * High speed is 480 Mbit/s against full speed's 12 - a 40x link. **Command
 * round trips will not improve 40x.** A round trip is one OUT transfer, the
 * firmware's response time, and one IN transfer, and its floor is set by frame
 * timing: 1 ms frames become 125 us microframes, which is 8x. Expect round
 * trips to improve by something in that order and bulk payload by much more.
 *
 * This is worth stating because the natural benchmark - "commands per second" -
 * measures the thing that improves least, and reporting it as though it
 * measured the link is how a successful change gets recorded as a failure.
 *
 * The STM32 numbers to beat, measured, on full speed:
 *
 *     USB round trips   5864 /s        USB payload   113.1 kB/s
 *
 * No prediction is offered for what this reaches. Measure it.
 */
#include "bb_internal.h"

#include <stdio.h>
#include <string.h>

/* ---- tuning ------------------------------------------------------------ */

/*
 * Transfers kept in flight on the IN pipe.
 *
 * Four is chosen so the host controller always has a submission queued for the
 * next microframe even if the application is slow to come back for the last
 * one. More than a handful stops helping: the queue only has to cover the gap
 * between completion and resubmission, and every extra one is a buffer.
 */
#define HS_IN_FLIGHT     4

/*
 * 64 KB per read. Two reasons for this size specifically:
 *
 *   - It is a multiple of 512, which RAW_IO requires.
 *   - It is the largest transfer WinUSB accepts by default on most systems.
 *     MAXIMUM_TRANSFER_SIZE is queried at open and this is clamped to it,
 *     because exceeding it makes WinUsb_ReadPipe fail with a parameter error
 *     that reads like a bug in the caller.
 */
#define HS_READ_SIZE     (64u * 1024u)

/* Reassembly buffer. A reply can be one line or a burst of them; this is drained
 * by read_line() and refilled from whichever transfer completed next. */
#define HS_RX_CAPACITY   (256u * 1024u)

typedef struct {
    OVERLAPPED     ov;
    HANDLE         evt;
    unsigned char *buf;
    ULONG          len;
    BOOL           pending;
} hs_xfer_t;

/*
 * Kept in a side table keyed by handle rather than added to bb_handle_s, so
 * this file is a drop-in against the existing header. If it graduates into the
 * tree proper these belong in the struct.
 */
typedef struct {
    bb_handle      owner;
    hs_xfer_t      in[HS_IN_FLIGHT];
    unsigned char *rx;
    size_t         rx_len;
    size_t         rx_pos;
    unsigned       next_reap;      /* transfers complete in submission order */
    ULONG          read_size;
    BOOL           raw_io;
} hs_state_t;

#define HS_MAX_HANDLES 8
static hs_state_t g_hs[HS_MAX_HANDLES];

static hs_state_t *hs_for(bb_handle h)
{
    for (int i = 0; i < HS_MAX_HANDLES; i++) {
        if (g_hs[i].owner == h) {
            return &g_hs[i];
        }
    }
    return NULL;
}

static hs_state_t *hs_alloc(bb_handle h)
{
    for (int i = 0; i < HS_MAX_HANDLES; i++) {
        if (g_hs[i].owner == NULL) {
            memset(&g_hs[i], 0, sizeof g_hs[i]);
            g_hs[i].owner = h;
            return &g_hs[i];
        }
    }
    return NULL;
}

/* ---- pipe setup -------------------------------------------------------- */

static void hs_configure_pipes(bb_handle h, hs_state_t *s)
{
    UCHAR enable  = TRUE;
    UCHAR disable = FALSE;
    ULONG value   = 0;
    ULONG size    = sizeof value;

    /* A stalled pipe should recover itself rather than needing the application
     * to notice. Unchanged from the old transport. */
    WinUsb_SetPipePolicy(h->winusb, h->pipe_in,  AUTO_CLEAR_STALL,
                         sizeof enable, &enable);
    WinUsb_SetPipePolicy(h->winusb, h->pipe_out, AUTO_CLEAR_STALL,
                         sizeof enable, &enable);

    /*
     * IGNORE_SHORT_PACKETS off. A short packet must complete the transfer
     * immediately: this protocol's replies are single lines terminated by CRLF,
     * and waiting for a full 64 KB buffer before returning one would turn every
     * command into a timeout. The old transport set this and it was correct.
     */
    WinUsb_SetPipePolicy(h->winusb, h->pipe_in, IGNORE_SHORT_PACKETS,
                         sizeof disable, &disable);

    /*
     * How large a transfer WinUSB will accept. Queried rather than assumed -
     * exceeding it fails with ERROR_INVALID_PARAMETER, which reads like a bug
     * in the caller rather than a policy limit.
     */
    s->read_size = HS_READ_SIZE;
    if (WinUsb_GetPipePolicy(h->winusb, h->pipe_in, MAXIMUM_TRANSFER_SIZE,
                             &size, &value) && value > 0 && value < s->read_size) {
        s->read_size = value;
    }

    /*
     * RAW_IO: submissions go straight to the host controller instead of being
     * buffered and re-split in the kernel.
     *
     * It comes with two hard requirements, and both happen to suit us:
     *   - every read must be a multiple of the endpoint's max packet size
     *   - every read must be <= MAXIMUM_TRANSFER_SIZE
     *
     * 512-byte high-speed bulk packets divide 64 KB exactly. If the device
     * enumerated at full speed the packet size is 64, which also divides it -
     * so this is safe at either speed, which matters because the host does not
     * get to choose.
     *
     * If the policy is refused, RAW_IO stays off and everything still works,
     * just with the kernel doing more copying. It is an optimisation, not a
     * requirement, and it is recorded so a throughput number can be attributed.
     */
    s->raw_io = WinUsb_SetPipePolicy(h->winusb, h->pipe_in, RAW_IO,
                                     sizeof enable, &enable);
    if (!s->raw_io) {
        /* Without RAW_IO, partial reads must be allowed or a short reply would
         * sit in the kernel until the buffer filled. */
        WinUsb_SetPipePolicy(h->winusb, h->pipe_in, ALLOW_PARTIAL_READS,
                             sizeof enable, &enable);
    }
    /* Round down to a packet multiple regardless, since RAW_IO may be set later
     * or by a future change. */
    s->read_size &= ~(ULONG)511u;
    if (s->read_size == 0) {
        s->read_size = 512u;
    }
}

/* ---- the in-flight queue ----------------------------------------------- */

static BOOL hs_submit(bb_handle h, hs_state_t *s, unsigned slot)
{
    hs_xfer_t *x = &s->in[slot];
    ULONG      got = 0;

    memset(&x->ov, 0, sizeof x->ov);
    ResetEvent(x->evt);
    x->ov.hEvent = x->evt;
    x->len       = 0;

    BOOL ok = WinUsb_ReadPipe(h->winusb, h->pipe_in, x->buf, s->read_size,
                              &got, &x->ov);
    if (ok) {
        /* Completed synchronously - already-buffered data. */
        x->len     = got;
        x->pending = FALSE;
        return TRUE;
    }
    if (GetLastError() == ERROR_IO_PENDING) {
        x->pending = TRUE;
        return TRUE;
    }
    x->pending = FALSE;
    return FALSE;
}

static void hs_prime(bb_handle h, hs_state_t *s)
{
    /*
     * Fill the queue. This is the change that matters: after this, the host
     * controller has HS_IN_FLIGHT reads it can service back-to-back, so the
     * device is no longer waiting a whole frame between packets.
     */
    for (unsigned i = 0; i < HS_IN_FLIGHT; i++) {
        if (!s->in[i].pending && s->in[i].len == 0) {
            (void)hs_submit(h, s, i);
        }
    }
}

/**
 * Wait for the oldest outstanding read and move its bytes into the reassembly
 * buffer, then resubmit that slot.
 *
 * Reaped in submission order rather than by whichever event fires first.
 * Bulk transfers on one pipe complete in order, and taking them out of order
 * would reorder the byte stream - which for a line-oriented protocol shows up
 * as garbled replies under load only, the worst kind of bug to chase.
 */
static int hs_reap(bb_handle h, hs_state_t *s, int timeout_ms)
{
    hs_xfer_t *x = &s->in[s->next_reap];

    if (!x->pending && x->len == 0) {
        if (!hs_submit(h, s, s->next_reap)) {
            return BB_ERR_IO;
        }
        x = &s->in[s->next_reap];
    }

    if (x->pending) {
        DWORD w = WaitForSingleObject(x->evt, (DWORD)(timeout_ms < 0 ? 0 : timeout_ms));
        if (w == WAIT_TIMEOUT) {
            return BB_ERR_TIMEOUT;
        }
        if (w != WAIT_OBJECT_0) {
            return BB_ERR_IO;
        }
        ULONG got = 0;
        if (!WinUsb_GetOverlappedResult(h->winusb, &x->ov, &got, FALSE)) {
            x->pending = FALSE;
            return BB_ERR_IO;
        }
        x->len     = got;
        x->pending = FALSE;
    }

    if (x->len > 0) {
        size_t room = HS_RX_CAPACITY - s->rx_len;
        size_t n    = (x->len < room) ? x->len : room;
        memcpy(s->rx + s->rx_len, x->buf, n);
        s->rx_len += n;
    }
    x->len = 0;

    unsigned slot = s->next_reap;
    s->next_reap = (s->next_reap + 1u) % HS_IN_FLIGHT;
    (void)hs_submit(h, s, slot);
    return BB_OK;
}

/* ---- public transport -------------------------------------------------- */

int bb_winusb_open_hs(bb_handle h, const char *path)
{
    /* Device open, interface claim and pipe discovery are unchanged from the
     * original transport - call it first, then layer the queue on top. */
    int rc = bb_winusb_open(h, path);
    if (rc != BB_OK) {
        return rc;
    }

    hs_state_t *s = hs_alloc(h);
    if (s == NULL) {
        bb_winusb_close(h);
        return BB_ERR_OPEN;
    }

    hs_configure_pipes(h, s);

    s->rx = (unsigned char *)malloc(HS_RX_CAPACITY);
    if (s->rx == NULL) {
        s->owner = NULL;
        bb_winusb_close(h);
        return BB_ERR_OPEN;
    }
    for (unsigned i = 0; i < HS_IN_FLIGHT; i++) {
        s->in[i].evt = CreateEvent(NULL, TRUE, FALSE, NULL);
        s->in[i].buf = (unsigned char *)malloc(s->read_size);
        if (s->in[i].evt == NULL || s->in[i].buf == NULL) {
            bb_winusb_close_hs(h);
            return BB_ERR_OPEN;
        }
    }
    hs_prime(h, s);
    return BB_OK;
}

void bb_winusb_close_hs(bb_handle h)
{
    hs_state_t *s = hs_for(h);

    if (s != NULL) {
        /* Cancel before freeing: a pending overlapped read still owns its
         * buffer, and freeing it under the kernel is a use-after-free that
         * manifests as heap corruption much later. */
        WinUsb_AbortPipe(h->winusb, h->pipe_in);
        for (unsigned i = 0; i < HS_IN_FLIGHT; i++) {
            if (s->in[i].pending) {
                ULONG got = 0;
                (void)WinUsb_GetOverlappedResult(h->winusb, &s->in[i].ov, &got, TRUE);
                s->in[i].pending = FALSE;
            }
            if (s->in[i].evt != NULL) { CloseHandle(s->in[i].evt); }
            free(s->in[i].buf);
        }
        free(s->rx);
        s->owner = NULL;
    }
    bb_winusb_close(h);
}

/**
 * Read one line.
 *
 * Drains the reassembly buffer first and only waits on the wire when it is
 * empty, which is what lets a burst of replies come out of a single transfer
 * instead of one transfer per line.
 */
int bb_winusb_read_line_hs(bb_handle h, char *buf, int cap, DWORD deadline)
{
    hs_state_t *s = hs_for(h);
    int         len = 0;

    if (s == NULL) {
        return bb_winusb_read_line(h, buf, cap, deadline);
    }

    for (;;) {
        while (s->rx_pos < s->rx_len) {
            char c = (char)s->rx[s->rx_pos++];
            if (c == '\n') {
                buf[len] = 0;
                if (len > 0 && buf[len - 1] == '\r') {
                    buf[--len] = 0;
                }
                if (s->rx_pos >= s->rx_len) {
                    s->rx_pos = s->rx_len = 0;
                }
                return len;
            }
            if (c != '\r' && len < cap - 1) {
                buf[len++] = c;
            }
        }
        s->rx_pos = s->rx_len = 0;

        DWORD now = GetTickCount();
        if (now > deadline) {
            return BB_ERR_TIMEOUT;
        }
        int slice = (int)(deadline - now);
        if (slice > 500) {
            slice = 500;            /* stay responsive to the deadline */
        }
        int rc = hs_reap(h, s, slice);
        if (rc == BB_ERR_TIMEOUT) {
            continue;               /* nothing yet; the deadline governs */
        }
        if (rc != BB_OK) {
            return rc;
        }
    }
}

void bb_winusb_flush_hs(bb_handle h)
{
    hs_state_t *s = hs_for(h);

    if (s == NULL) {
        bb_winusb_flush(h);
        return;
    }
    /*
     * Abort, drain, resubmit. Just clearing the reassembly buffer would leave
     * queued transfers holding stale bytes that surface as the first line of the
     * *next* command's reply - which is exactly the kind of one-command-behind
     * bug that queueing introduces if flush is not written for it.
     */
    WinUsb_AbortPipe(h->winusb, h->pipe_in);
    for (unsigned i = 0; i < HS_IN_FLIGHT; i++) {
        if (s->in[i].pending) {
            ULONG got = 0;
            (void)WinUsb_GetOverlappedResult(h->winusb, &s->in[i].ov, &got, TRUE);
            s->in[i].pending = FALSE;
        }
        s->in[i].len = 0;
    }
    s->rx_len = s->rx_pos = 0;
    s->next_reap = 0;
    hs_prime(h, s);
}

/**
 * Report how the transport is configured, so a throughput figure can be
 * attributed to something. Without this, "110 kB/s" and "40 MB/s" are both just
 * numbers - the STM32 project's habit was to publish the method alongside the
 * measurement, and this is that method.
 */
int bb_winusb_transport_info_hs(bb_handle h, char *out, int cap)
{
    hs_state_t *s = hs_for(h);

    if (s == NULL) {
        return snprintf(out, (size_t)cap, "legacy single-transfer 512B");
    }
    return snprintf(out, (size_t)cap,
                    "in_flight=%d read_size=%lu raw_io=%d",
                    HS_IN_FLIGHT, (unsigned long)s->read_size, s->raw_io ? 1 : 0);
}
