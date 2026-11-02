/**
 * @file bb_ee_i2c.c
 * @brief LPI2C target — answers as an I2C EEPROM at 0x50.
 *
 * This is the image that makes one board something to test the other against.
 * The controller image drives a bus; this one answers on it, and both serve the
 * same 1 KB image through `bb_ee_*` so a write over I2C is readable over SPI.
 *
 * **Which bus it sits on.** Bus 3 (LPI2C3 on D14/D15, the Arduino I2C
 * position), because that is the one bus on this board that is neither shared
 * with the console (bus 1) nor stranded on the motor connector (bus 2). A
 * target that answers on a bus nobody can conveniently reach is a target nobody
 * uses.
 *
 * **Event-driven, not polled.** The transfer callback runs from the LPI2C
 * interrupt and has to be quick: while it is running, SCL is stretched and the
 * controller is waiting. So it does exactly one thing per event - hand over the
 * next byte, or take one - and all the bookkeeping happens through `bb_ee_*`
 * which is a plain array access.
 */
#include "bb_ee.h"
#include "bb_ee_ports.h"
#include "bb_board.h"
#include "bb_image.h"

#include "fsl_clock.h"
#include "fsl_lpi2c.h"

#include <stdio.h>
#include <string.h>

/* 0x50 is where every I2C EEPROM lives, so a host's existing tooling finds it
 * without being told. */
#define EE_I2C_ADDR   0x50u

#define EE_I2C_BASE   BB_I2C3_INSTANCE
#define EE_I2C_IRQ    LPI2C3_IRQn
#define EE_I2C_ROOT   kCLOCK_Root_Lpi2c0304

static const bb_pin_t s_scl = BB_I2C3_SCL;
static const bb_pin_t s_sda = BB_I2C3_SDA;

static lpi2c_slave_handle_t s_handle;
static bool                 s_ready;

/*
 * Where the controller's register pointer is.
 *
 * An I2C EEPROM's protocol is: a write of one or two address bytes sets the
 * pointer, and everything after that is data. So the first byte(s) of any write
 * are the address and the rest is payload - which means the target has to count
 * how far into the current transaction it is. `sub_seen` is that count, and it
 * resets on every START, not on every byte.
 */
static uint16_t s_ptr;
static uint8_t  s_sub_seen;
static uint8_t  s_scratch;

static void ee_i2c_callback(LPI2C_Type *base, lpi2c_slave_transfer_t *xfer,
                            void *userData)
{
    (void)base;
    (void)userData;

    switch (xfer->event) {
    case kLPI2C_SlaveAddressMatchEvent:
        /* A fresh transaction. The pointer survives from the last one - that is
         * how a read-after-write works without repeating the address - but the
         * sub-address counter must not, or the second write in a row would be
         * treated as payload. */
        s_sub_seen = 0u;
        break;

    case kLPI2C_SlaveReceiveEvent:
        xfer->data     = &s_scratch;
        xfer->dataSize = 1u;
        break;

    case kLPI2C_SlaveTransmitEvent:
        /* Reads wrap within the 1 KB image rather than running off the end.
         * A target that returns garbage past its last byte is indistinguishable
         * from one that has stopped answering. */
        s_scratch      = bb_ee_read(BB_EE_IF_I2C, s_ptr);
        s_ptr          = (uint16_t)((s_ptr + 1u) % BB_EE_SIZE);
        xfer->data     = &s_scratch;
        xfer->dataSize = 1u;
        break;

    case kLPI2C_SlaveCompletionEvent:
        bb_ee_note_transaction(BB_EE_IF_I2C);
        break;

    default:
        break;
    }
}

/*
 * The received byte is consumed here rather than in the callback because the
 * callback is told "a byte is coming" before it has arrived. LPI2C reports
 * kLPI2C_SlaveReceiveEvent to ask for a buffer; the data is in it by the time
 * the next event or the completion arrives.
 */
static void ee_i2c_take(uint8_t byte)
{
    if (s_sub_seen < 2u) {
        /* Two-byte big-endian address, which is what a 1 KB device with a
         * 10-bit space uses. A one-byte-address part would be a different
         * device; supporting both by guessing from the transaction length is
         * how you get a target that works until someone does a one-byte write. */
        s_ptr = (uint16_t)((s_ptr << 8) | byte);
        s_sub_seen++;
        if (s_sub_seen == 2u) {
            s_ptr = (uint16_t)(s_ptr % BB_EE_SIZE);
        }
        return;
    }
    bb_ee_write(BB_EE_IF_I2C, s_ptr, byte);
    s_ptr = (uint16_t)((s_ptr + 1u) % BB_EE_SIZE);
}

void LPI2C3_IRQHandler(void)
{
    /* Snapshot before the driver runs: it consumes the receive flag, so a byte
     * has to be taken from the scratch slot on the transition. */
    static uint8_t last;

    LPI2C_SlaveTransferHandleIRQ(EE_I2C_BASE, &s_handle);
    if (s_scratch != last) {
        ee_i2c_take(s_scratch);
        last = s_scratch;
    }
    SDK_ISR_EXIT_BARRIER;
}

void bb_ee_i2c_init(void)
{
    lpi2c_slave_config_t cfg;

    bb_pin_mux(&s_scl, BB_PAD_I2C, true);
    bb_pin_mux(&s_sda, BB_PAD_I2C, true);

    LPI2C_SlaveGetDefaultConfig(&cfg);
    cfg.enableSlave = true;
    cfg.address0    = EE_I2C_ADDR;
    /* Clock stretching on every phase. A software target cannot always answer
     * within a bit time, and stretching is the protocol's own answer to that -
     * a controller that cannot tolerate it is not spec-compliant. The STM32
     * sniffer notes make the same point from the other side: clock stretching
     * changes nothing for an edge-driven decoder. */
    cfg.sclStall.enableAck     = true;
    cfg.sclStall.enableTx      = true;
    cfg.sclStall.enableRx      = true;
    cfg.sclStall.enableAddress = true;

    LPI2C_SlaveDeinit(EE_I2C_BASE);
    LPI2C_SlaveInit(EE_I2C_BASE, &cfg, CLOCK_GetRootClockFreq(EE_I2C_ROOT));

    LPI2C_SlaveTransferCreateHandle(EE_I2C_BASE, &s_handle, ee_i2c_callback, NULL);
    if (LPI2C_SlaveTransferNonBlocking(EE_I2C_BASE, &s_handle,
                                       (uint32_t)kLPI2C_SlaveAllEvents)
        != kStatus_Success) {
        s_ready = false;
        return;
    }
    HAL_NVIC_SetPriority(EE_I2C_IRQ, 4, 0);
    HAL_NVIC_EnableIRQ(EE_I2C_IRQ);

    s_ptr      = 0u;
    s_sub_seen = 0u;
    s_ready    = true;
}

void bb_ee_i2c_ev_irq(unsigned bus)
{
    (void)bus;   /* one target bus on this board; the vector is direct */
}

void bb_ee_i2c_er_irq(unsigned bus)
{
    (void)bus;
}

void bb_ee_i2c_describe(char *out, size_t cap)
{
    (void)snprintf(out, cap, "i2c3@0x%02X %s", EE_I2C_ADDR,
                   s_ready ? "ready" : "failed");
}

void bb_ee_i2c_status(bb_status_t *out)
{
    if (out == NULL) {
        return;
    }
    memset(out, 0, sizeof *out);
    bb_image_copy_str(out->pins, sizeof out->pins,
                      BB_I2C3_PINS, strlen(BB_I2C3_PINS));
    const bb_ee_stats_t *st = bb_ee_stats(BB_EE_IF_I2C);
    (void)snprintf(out->detail, sizeof out->detail, "0x%02X t=%lu",
                   EE_I2C_ADDR,
                   (unsigned long)(st != NULL ? st->transactions : 0u));
    out->state = !s_ready ? BB_STATE_PROBLEM
                 : (st != NULL && st->transactions > 0u) ? BB_STATE_ACTIVE
                                                         : BB_STATE_IDLE;
}
