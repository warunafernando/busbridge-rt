/**
 * @file bb_ee_spi.c
 * @brief LPSPI target — the second door into the same 1 KB.
 *
 * The value of having two doors is being able to write through one and read
 * through the other: a byte set over SPI and read back over I2C proves the two
 * back-ends agree about the array, which is a test neither can do alone.
 *
 * **Protocol.** Deliberately the simplest thing that is still a real SPI
 * EEPROM, and the same one the STM32 peripheral used so a host's scripts do not
 * change:
 *
 *     0x03 <ahi> <alo> -> read, then bytes stream out until CS rises
 *     0x02 <ahi> <alo> <data...> -> write, until CS rises
 *     0x05 -> status byte
 *
 * **CS is the frame, and that is the whole difficulty.** A SPI target has no
 * START or STOP - the only thing that says where a transaction begins and ends
 * is chip select. LPSPI's own PCS handling deasserts per transfer, which is
 * wrong here: a read of 64 bytes is one CS assertion with 67 clocked bytes in
 * it. So CS is watched as a GPIO edge and the state machine is reset there,
 * exactly as the STM32 version did with an EXTI on the same line.
 */
#include "bb_ee.h"
#include "bb_ee_ports.h"
#include "bb_board.h"
#include "bb_image.h"

#include "fsl_clock.h"
#include "fsl_lpspi.h"

#include <stdio.h>
#include <string.h>

#define EE_SPI_BASE   BB_SPI_INSTANCE
#define EE_SPI_IRQ    BB_SPI_IRQ

static const bb_pin_t s_sck = BB_SPI_SCK;
static const bb_pin_t s_sdo = BB_SPI_SDO;
static const bb_pin_t s_sdi = BB_SPI_SDI;
static const bb_pin_t s_cs  = BB_SPI_CS;

#define EE_CMD_READ    0x03u
#define EE_CMD_WRITE   0x02u
#define EE_CMD_STATUS  0x05u

typedef enum {
    ST_CMD = 0,
    ST_ADDR_HI,
    ST_ADDR_LO,
    ST_DATA
} ee_spi_state_t;

static struct {
    ee_spi_state_t state;
    uint8_t        cmd;
    uint16_t       ptr;
    bool           ready;
} s;

static uint8_t s_rx;
static uint8_t s_tx;
static lpspi_slave_handle_t s_handle;

/**
 * @brief One byte in, one byte out.
 *
 * A SPI transfer is always simultaneous, so the byte returned for the current
 * clock was decided by the *previous* one. That off-by-one is the thing SPI
 * targets get wrong: the data byte for a read command cannot be presented until
 * the address has been fully received, so the byte clocked out during the
 * address phase is a dummy and must be, rather than being the first data byte
 * one position early.
 */
static uint8_t ee_spi_step(uint8_t in)
{
    switch (s.state) {
    case ST_CMD:
        s.cmd = in;
        if (in == EE_CMD_STATUS) {
            s.state = ST_DATA;
            /* Bit 1 is the write-enable latch on a real part. Reported as
             * "always writable" because this target has no latch - and saying
             * 0x02 rather than 0x00 is what a host's existing EEPROM driver
             * expects before it will issue a write. */
            return bb_ee_protected(BB_EE_IF_SPI) ? 0x00u : 0x02u;
        }
        s.state = ST_ADDR_HI;
        return 0xFFu;

    case ST_ADDR_HI:
        s.ptr   = (uint16_t)((uint16_t)in << 8);
        s.state = ST_ADDR_LO;
        return 0xFFu;

    case ST_ADDR_LO:
        s.ptr   = (uint16_t)((s.ptr | in) % BB_EE_SIZE);
        s.state = ST_DATA;
        /* For a read, this is the last dummy: the first real data byte goes out
         * on the next clock. */
        return 0xFFu;

    case ST_DATA:
    default:
        if (s.cmd == EE_CMD_READ) {
            uint8_t out = bb_ee_read(BB_EE_IF_SPI, s.ptr);
            s.ptr = (uint16_t)((s.ptr + 1u) % BB_EE_SIZE);
            return out;
        }
        if (s.cmd == EE_CMD_WRITE) {
            bb_ee_write(BB_EE_IF_SPI, s.ptr, in);
            s.ptr = (uint16_t)((s.ptr + 1u) % BB_EE_SIZE);
            return 0xFFu;
        }
        return 0xFFu;
    }
}

static void ee_spi_callback(LPSPI_Type *base, lpspi_slave_handle_t *handle,
                            status_t status, void *userData)
{
    (void)base;
    (void)handle;
    (void)userData;

    if (status == kStatus_Success) {
        s_tx = ee_spi_step(s_rx);
    }
    /* Re-arm immediately. A target that stops listening between bytes drops the
     * rest of the frame, and the controller has no way to know. */
    lpspi_transfer_t t;
    memset(&t, 0, sizeof t);
    t.txData   = &s_tx;
    t.rxData   = &s_rx;
    t.dataSize = 1u;
    (void)LPSPI_SlaveTransferNonBlocking(EE_SPI_BASE, &s_handle, &t);
}

void BB_SPI_IRQHANDLER(void)
{
    LPSPI_SlaveTransferHandleIRQ(EE_SPI_BASE, &s_handle);
    SDK_ISR_EXIT_BARRIER;
}

void bb_ee_spi_cs_edge(void)
{
    /*
     * CS has moved. Whichever way, the frame is over: reset the state machine.
     *
     * Called from the CS pin's interrupt on a real build. On this board it is
     * also called from the poll hook, because RGPIO interrupt routing for this
     * pad has not been set up yet - polling a level is a poor substitute for an
     * edge and is honest about being one: a frame shorter than the poll interval
     * will not be seen, and `bb_ee_spi_describe()` says so.
     */
    s.state = ST_CMD;
    bb_ee_note_transaction(BB_EE_IF_SPI);
}

void bb_ee_spi_poll(void)
{
    static bool was_asserted;
    bool        now = !bb_pin_read(&s_cs);   /* active low */

    if (now != was_asserted) {
        was_asserted = now;
        if (!now) {
            bb_ee_spi_cs_edge();
        }
    }
}

unsigned bb_ee_spi_cs_line(void)
{
    return s_cs.bit;
}

void bb_ee_spi_init(void)
{
    lpspi_slave_config_t cfg;

    bb_pin_mux(&s_sck, BB_PAD_INPUT, true);
    /* SDO is our output in target mode, SDI our input - the opposite of the
     * controller image, and a place it is easy to mirror by mistake. */
    bb_pin_mux(&s_sdo, BB_PAD_PUSHPULL, true);
    bb_pin_mux(&s_sdi, BB_PAD_INPUT, true);
    bb_pin_gpio(&s_cs, false, false, BB_PAD_INPUT_PU);

    LPSPI_SlaveGetDefaultConfig(&cfg);
    cfg.bitsPerFrame = 8u;
    cfg.cpol         = kLPSPI_ClockPolarityActiveHigh;
    cfg.cpha         = kLPSPI_ClockPhaseFirstEdge;
    cfg.direction    = kLPSPI_MsbFirst;
    cfg.whichPcs     = kLPSPI_Pcs0;

    LPSPI_Deinit(EE_SPI_BASE);
    LPSPI_SlaveInit(EE_SPI_BASE, &cfg);
    LPSPI_SlaveTransferCreateHandle(EE_SPI_BASE, &s_handle, ee_spi_callback, NULL);

    s.state = ST_CMD;
    s.ptr   = 0u;

    lpspi_transfer_t t;
    memset(&t, 0, sizeof t);
    s_tx       = 0xFFu;
    t.txData   = &s_tx;
    t.rxData   = &s_rx;
    t.dataSize = 1u;
    s.ready = (LPSPI_SlaveTransferNonBlocking(EE_SPI_BASE, &s_handle, &t)
               == kStatus_Success);

    if (s.ready) {
        HAL_NVIC_SetPriority(EE_SPI_IRQ, 4, 0);
        HAL_NVIC_EnableIRQ(EE_SPI_IRQ);
    }
}

void bb_ee_spi_irq(void)
{
}

void bb_ee_spi_describe(char *out, size_t cap)
{
    /* The CS caveat is in the description rather than only in a comment,
     * because it changes what results mean: a short frame can be missed. */
    (void)snprintf(out, cap, "spi4 %s cs=polled",
                   s.ready ? "ready" : "failed");
}

void bb_ee_spi_status(bb_status_t *out)
{
    if (out == NULL) {
        return;
    }
    memset(out, 0, sizeof *out);
    bb_image_copy_str(out->pins, sizeof out->pins,
                      BB_SPI_PINS, strlen(BB_SPI_PINS));
    const bb_ee_stats_t *st = bb_ee_stats(BB_EE_IF_SPI);
    (void)snprintf(out->detail, sizeof out->detail, "t=%lu cs-polled",
                   (unsigned long)(st != NULL ? st->transactions : 0u));
    out->state = !s.ready ? BB_STATE_PROBLEM
                 : (st != NULL && st->transactions > 0u) ? BB_STATE_ACTIVE
                                                         : BB_STATE_IDLE;
}
