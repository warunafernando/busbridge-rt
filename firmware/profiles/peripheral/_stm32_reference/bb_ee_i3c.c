/**
 * @file profiles/peripheral/bb_ee_i3c.c
 * @brief The EEPROM as an I3C target, over the machinery in bb_i3c.c.
 *
 * The address format is the I2C EEPROM's, carried in I3C private transfers,
 * because that is what the rest of this board already answers and one format
 * across four buses is worth more than an exotic one here:
 *
 *   private write  ah al [data...]     set the pointer, then write from it
 *   private read   [data...]           read from the pointer
 *
 * Two ways this deliberately differs from the I2C, SPI and MDIO doors, both
 * forced by what an I3C target physically is:
 *
 * 1. A read is answered from a window loaded in advance. An I3C target cannot
 *    stall a read the way an I2C target stretches the clock - the controller
 *    starts clocking the data phase immediately - so the bytes must already be
 *    in the peripheral's FIFO. A window from the current pointer is therefore
 *    kept loaded at all times: at startup, after every write, and the moment a
 *    read consumes it. Write-then-read, which is what every EEPROM driver does,
 *    is served exactly.
 *
 * 2. A read does not advance the pointer. It cannot, honestly: the DMA that
 *    feeds the window reports how many bytes it handed to the peripheral, and
 *    the peripheral's FIFO holds several of those beyond what the controller
 *    actually clocked. Advancing by that number would put the pointer a few
 *    bytes past the truth and call it auto-increment. So an I3C read always
 *    returns bytes from the address the last write set, repeatably, and a
 *    driver that wants the next chunk sets the address again. The byte counts
 *    in EE:STAT? come from the DMA and are therefore an upper bound - they are
 *    diagnostics, not data.
 *
 * Writes auto-increment normally: those bytes arrive one at a time and are
 * counted as they land, with nothing prefetched and nothing estimated.
 *
 * Driven from the command pump rather than from the interrupt. Reloading the
 * window is a DMA abort, two FIFO flushes and a transfer start, which has no
 * business running in the path of a byte arriving every 640 ns at 12.5 MHz. The
 * pump comes round in microseconds; a controller's write-then-read pair is
 * milliseconds apart.
 */
#include "bb_board.h"
#include "bb_cmd.h"
#include "bb_ee.h"
#include "bb_ee_ports.h"
#include "bb_i3c.h"

#include <stdio.h>

/**
 * @brief How much of the array to keep loaded for a read that gives no notice.
 *
 * Large enough that a driver reading a record gets it from one window, small
 * enough that reloading after every read is cheap. A controller reading past
 * the end of the window gets the window and then nothing, which is what a real
 * part does when its output buffer runs dry.
 */
#define EE_I3C_WINDOW    64u

/* One private write: the two address bytes plus data. */
#define EE_I3C_FRAME_MAX BB_PAYLOAD_MAX

static bool     enabled;
static bool     started;
static uint16_t pointer;
static bool     need_reload;
static uint32_t last_write_frames;
static uint32_t writes_applied;
static uint32_t reads_served;
static uint32_t short_frames;      /* writes too short to carry an address */

static uint8_t  frame[EE_I3C_FRAME_MAX];
static size_t   frame_len;

/** Load EE_I3C_WINDOW bytes from the pointer into the target's reply buffer. */
static void load_window(void)
{
    uint8_t win[EE_I3C_WINDOW];

    for (unsigned i = 0; i < EE_I3C_WINDOW; i++) {
        /* peek, not read: loading the window is this module's bookkeeping and
         * counting it would report reads the controller never performed. */
        win[i] = bb_ee_peek((uint16_t)(pointer + i));
    }
    if (bb_i3c_tgt_arm(win, sizeof win)) {
        need_reload = false;
    }
}

void bb_ee_i3c_init(void)
{
    enabled     = true;
    pointer     = 0;
    frame_len   = 0;
    need_reload = true;

    if (!bb_i3c_tgt_begin()) {
        started = false;
        return;
    }
    started           = true;
    last_write_frames = bb_i3c_tgt_write_frames();
    /* No window yet: arming needs a dynamic address and this target has not
     * been through ENTDAA. The pump loads it as soon as the bus assigns one. */
}

bool bb_ee_i3c_enabled(void)
{
    return enabled;
}

/* ---- the pump ---------------------------------------------------------- */

/** Interpret one completed private write. */
static void apply_write(const uint8_t *data, size_t len)
{
    if (len < 2u) {
        /* One byte is not an address. Taking it as data at the current pointer
         * is what a real part would do, but that makes a truncated frame look
         * like a successful write, so it is counted and dropped. */
        short_frames++;
        return;
    }

    pointer = (uint16_t)((((uint16_t)data[0] << 8) | data[1]) & BB_EE_MASK);
    for (size_t i = 2; i < len; i++) {
        bb_ee_write(BB_EE_IF_I3C, (uint16_t)(pointer + (i - 2u)), data[i]);
    }
    if (len > 2u) {
        pointer = (uint16_t)((pointer + (len - 2u)) & BB_EE_MASK);
    }
    writes_applied++;
    bb_ee_note_transaction(BB_EE_IF_I3C);
}

void bb_ee_i3c_poll(void)
{
    if (!enabled || !started) {
        return;
    }

    /* Collect whatever the controller has written since the last pass. */
    if (frame_len < sizeof frame) {
        frame_len += bb_i3c_tgt_take(&frame[frame_len], sizeof frame - frame_len);
    }

    /* The frame counter is read after draining, not before: the event handler
     * empties the FIFO and only then counts the frame, so a counter that has
     * moved means every byte of that frame is already in hand. */
    uint32_t frames_now = bb_i3c_tgt_write_frames();

    if (frames_now != last_write_frames) {
        last_write_frames = frames_now;
        if (frame_len > 0u) {
            apply_write(frame, frame_len);
            frame_len = 0;
        }
        need_reload = true;         /* the pointer moved; the window is stale */
    }

    if (bb_i3c_tgt_address() == 0u) {
        return;                     /* unaddressed: nothing can read us */
    }

    if (need_reload) {
        load_window();
        return;
    }

    /* A read consumed some or all of the window. Either way the window has to
     * be rebuilt, because part of it has already gone out of the FIFO. */
    uint16_t took = bb_i3c_tgt_tx_consumed();

    if (took > 0u || !bb_i3c_tgt_armed()) {
        if (took > 0u) {
            reads_served++;
            bb_ee_note_transaction(BB_EE_IF_I3C);
            bb_ee_note_bytes_read(BB_EE_IF_I3C, took);
        }
        load_window();
    }
}

void bb_ee_i3c_ev_irq(void)
{
    bb_i3c_ev_irq();
}

void bb_ee_i3c_er_irq(void)
{
    bb_i3c_er_irq();
}

void bb_ee_i3c_describe(char *out, size_t cap)
{
    if (!enabled) {
        snprintf(out, cap, "off");
    } else if (!started) {
        snprintf(out, cap, "failed");
    } else {
        snprintf(out, cap, "addr=%s ptr=0x%03X window=%u",
                 bb_i3c_tgt_address() != 0u ? "assigned" : "none-yet",
                 pointer, (unsigned)EE_I3C_WINDOW);
    }
}

void bb_ee_i3c_status(bb_status_t *out)
{
    snprintf(out->pins, sizeof out->pins, "tgt 0x%02X", bb_i3c_tgt_address());
    if (!enabled) {
        snprintf(out->detail, sizeof out->detail, "off");
        out->state = BB_STATE_IDLE;
        return;
    }
    snprintf(out->detail, sizeof out->detail, "r%lu w%lu",
             (unsigned long)reads_served, (unsigned long)writes_applied);
    out->state = !started ? BB_STATE_PROBLEM
               : ((reads_served + writes_applied) > 0u ? BB_STATE_ACTIVE
                                                       : BB_STATE_IDLE);
}

/* ---- commands ---------------------------------------------------------- */

static void cmd_i3c(int argc, char **argv)
{
    char detail[64];

    if (argc == 1) {
        bb_ee_i3c_describe(detail, sizeof detail);
        bb_reply_okf("%s reads=%lu writes=%lu short=%lu", detail,
                     (unsigned long)reads_served,
                     (unsigned long)writes_applied,
                     (unsigned long)short_frames);
        return;
    }
    if (bb_streq_ci(argv[1], "OFF")) {
        enabled = false;
        started = false;
        bb_i3c_release();
        bb_reply_ok();
        return;
    }
    if (bb_streq_ci(argv[1], "ON")) {
        bb_ee_i3c_init();
        if (!started) {
            bb_reply_err(BB_ERR_HAL, "i3c-target-init");
            return;
        }
        bb_ee_i3c_describe(detail, sizeof detail);
        bb_reply_okf("%s", detail);
        return;
    }
    bb_reply_err(BB_ERR_ARGC, "expected-ON-or-OFF");
}

const bb_cmd_t bb_cmd_ee_i3c[] = {
    { "EE:I3C", 0, 1, cmd_i3c, "EE:I3C [ON | OFF]" },
    { NULL, 0, 0, NULL, NULL },
};
