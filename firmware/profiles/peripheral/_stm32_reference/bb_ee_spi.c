/**
 * @file profiles/peripheral/bb_ee_spi.c
 * @brief A 25-series SPI EEPROM, over the same kilobyte as the other three buses.
 *
 * The opcodes are the ones every SPI EEPROM and serial flash uses, so the host's
 * existing spi_flash_id example and any 25xx driver work against it unchanged:
 *
 *   0x03 READ   addr16, then data out, auto-incrementing
 *   0x02 WRITE  addr16, then data in
 *   0x05 RDSR   status: bit1 = write enable latch, bit0 = write in progress (0)
 *   0x06 WREN / 0x04 WRDI   set and clear that latch
 *   0x9F RDID   three bytes, 0xBB 0x10 0x01
 *
 * WREN is enforced, because a driver that forgets it should fail here exactly as
 * it would against a real part.
 *
 * Chip select is an EXTI input rather than the peripheral's hardware NSS: the
 * boards bring out a plain GPIO for CS - it is the bridge profile's software
 * chip select - and that pin is not the SPI's NSS alternate function on any of
 * them. Framing the transaction on the edge costs one interrupt per transfer and
 * keeps the same pin working in both images.
 *
 * The timing that matters: this is full duplex, so the byte the controller
 * clocks out of us during transfer N is the byte we loaded before transfer N
 * began. After the address low byte arrives we have one bit time to load the
 * first data byte, which is what sets the usable clock ceiling. Registers are
 * driven directly here rather than through the HAL's transfer API for that
 * reason - the HAL's slave path is written for a known length, and there is no
 * known length when the opcode has not arrived yet.
 */
#include "bb_board.h"
#include "bb_cmd.h"
#include "bb_ee.h"
#include "bb_ee_ports.h"

#include <stdio.h>

/* 25-series opcodes. */
#define OP_WRSR   0x01u
#define OP_WRITE  0x02u
#define OP_READ   0x03u
#define OP_WRDI   0x04u
#define OP_RDSR   0x05u
#define OP_WREN   0x06u
#define OP_RDID   0x9Fu

#define SR_WEL    0x02u          /* write enable latch */

/* What this part answers to RDID. Not a real manufacturer code: 0xBB for
 * busbridge, then a two-byte type, so a scope trace or a log line says at a
 * glance that it reached this firmware and not a genuine EEPROM. */
static const uint8_t device_id[3] = { 0xBB, 0x10, 0x01 };

typedef enum {
    ST_OPCODE = 0,
    ST_ADDR_HI,
    ST_ADDR_LO,
    ST_READ,
    ST_WRITE,
    ST_RDSR,
    ST_RDID,
    ST_IGNORE
} state_t;

static SPI_HandleTypeDef hspi;
static bool     enabled;
static bool     configured;
static uint8_t  spi_mode;                 /* 0..3, clock polarity and phase */

static volatile state_t  state;
static volatile uint16_t pointer;
static volatile uint8_t  status_reg;
/* Which of READ and WRITE is in progress. Both opcodes take the same two
 * address bytes, so the state machine cannot tell them apart on its own - and
 * keeping it in a spare status-register bit would have made RDSR report a bit
 * no real part has. */
static volatile bool     writing;
static volatile uint8_t  id_index;
static volatile uint32_t frames;          /* chip selects seen */

static inline void tx_push(uint8_t byte)
{
    *(volatile uint8_t *)&hspi.Instance->TXDR = byte;
}

static inline uint8_t rx_pop(void)
{
    return *(volatile uint8_t *)&hspi.Instance->RXDR;
}

/**
 * @brief The EXTI vector for the chip select pin.
 *
 * Derived from the pin rather than named in every board header: EXTI lines 0 to
 * 15 have consecutive IRQ numbers on this family, and one fact in the board file
 * (which pin CS is on) is better than two that can disagree.
 */
static IRQn_Type cs_exti_irqn(void)
{
    unsigned line = 0;

    while (line < 15u && (BB_SPI_CS_PIN & (1u << line)) == 0u) {
        line++;
    }
    return (IRQn_Type)((int)EXTI0_IRQn + (int)line);
}

/** The EXTI line the chip select is on, for the dispatcher in irq.c. */
unsigned bb_ee_spi_cs_line(void)
{
    unsigned line = 0;

    while (line < 15u && (BB_SPI_CS_PIN & (1u << line)) == 0u) {
        line++;
    }
    return line;
}

static inline bool cs_asserted(void)
{
    /* Active low, as every SPI device's chip select is. */
    return HAL_GPIO_ReadPin(BB_SPI_CS_PORT, BB_SPI_CS_PIN) == GPIO_PIN_RESET;
}

/* ---- the transaction --------------------------------------------------- */

/**
 * @brief One byte arrived; decide what the controller gets during the next one.
 *
 * Every branch ends in exactly one tx_push(), because a full-duplex slave that
 * skips a load sends whatever was left in the register - which reads as
 * plausible data and is the hardest kind of bug to see on a scope.
 */
static void byte_arrived(uint8_t in)
{
    switch (state) {
    case ST_OPCODE:
        switch (in) {
        case OP_READ:
        case OP_WRITE:
            state   = ST_ADDR_HI;
            writing = (in == OP_WRITE);
            tx_push(0xFF);
            break;
        case OP_RDSR:
            state = ST_RDSR;
            tx_push(status_reg);
            break;
        case OP_RDID:
            state    = ST_RDID;
            id_index = 0;
            tx_push(device_id[0]);
            break;
        case OP_WREN:
            status_reg |= SR_WEL;
            state = ST_IGNORE;
            tx_push(0xFF);
            break;
        case OP_WRDI:
            status_reg &= (uint8_t)~SR_WEL;
            state = ST_IGNORE;
            tx_push(0xFF);
            break;
        case OP_WRSR:
        default:
            /* Unknown opcodes are swallowed rather than guessed at. */
            state = ST_IGNORE;
            tx_push(0xFF);
            break;
        }
        break;

    case ST_ADDR_HI:
        pointer = (uint16_t)((uint16_t)in << 8);
        state   = ST_ADDR_LO;
        tx_push(0xFF);
        break;

    case ST_ADDR_LO:
        pointer = (uint16_t)((pointer | in) & BB_EE_MASK);
        if (writing) {
            state = ST_WRITE;
            tx_push(0xFF);
        } else {
            state = ST_READ;
            /* The controller's next clocked byte is the first data byte, so it
             * has to be in the register now. This load is the tight one. */
            tx_push(bb_ee_read(BB_EE_IF_SPI, pointer));
            pointer = (uint16_t)((pointer + 1u) & BB_EE_MASK);
        }
        break;

    case ST_READ:
        tx_push(bb_ee_read(BB_EE_IF_SPI, pointer));
        pointer = (uint16_t)((pointer + 1u) & BB_EE_MASK);
        break;

    case ST_WRITE:
        if ((status_reg & SR_WEL) != 0u) {
            bb_ee_write(BB_EE_IF_SPI, pointer, in);
        }
        pointer = (uint16_t)((pointer + 1u) & BB_EE_MASK);
        tx_push(0xFF);
        break;

    case ST_RDSR:
        tx_push(status_reg);
        break;

    case ST_RDID:
        id_index = (uint8_t)((id_index + 1u) % (uint8_t)sizeof device_id);
        tx_push(device_id[id_index]);
        break;

    case ST_IGNORE:
    default:
        tx_push(0xFF);
        break;
    }
}

void bb_ee_spi_irq(void)
{
    if (!configured) {
        return;
    }
    /* Drain: at a high clock rate more than one byte can land per interrupt, and
     * leaving one in the FIFO stalls the response by a whole byte. */
    while ((hspi.Instance->SR & SPI_FLAG_RXP) != 0u) {
        byte_arrived(rx_pop());
    }
    /* Overrun means a byte was lost - the controller outran the interrupt. Say
     * so in the counter rather than pretending the data was right. */
    if ((hspi.Instance->SR & SPI_FLAG_OVR) != 0u) {
        hspi.Instance->IFCR = SPI_IFCR_OVRC;
        status_reg |= 0x40u;                    /* sticky overrun flag */
    }
}

void bb_ee_spi_cs_edge(void)
{
    if (!configured) {
        return;
    }
    if (cs_asserted()) {
        /* Start of a transaction. Reset the state machine and prime the first
         * response, which for an unknown opcode is 0xFF. */
        state    = ST_OPCODE;
        id_index = 0;
        frames++;
        bb_ee_note_transaction(BB_EE_IF_SPI);
        while ((hspi.Instance->SR & SPI_FLAG_RXP) != 0u) {
            (void)rx_pop();                     /* debris from the last frame */
        }
        tx_push(0xFF);
    } else {
        /* End of a transaction. A real EEPROM clears the write-enable latch on
         * the rising edge that completes a write, and drivers rely on having to
         * send WREN again. */
        if (state == ST_WRITE) {
            status_reg &= (uint8_t)~SR_WEL;
        }
        state = ST_OPCODE;
    }
}

/* ---- bring-up ---------------------------------------------------------- */

static bool apply(void)
{
    GPIO_InitTypeDef g = {0};

    if (configured) {
        hspi.Instance->CR1 &= ~SPI_CR1_SPE;
        (void)HAL_SPI_DeInit(&hspi);
        configured = false;
    }
    if (!enabled) {
        return true;
    }

    BB_SPI_CLK_ENABLE();

    /* SCK and MOSI are inputs now, MISO the only output. The alternate function
     * handles direction, but the pull matters: an unclocked SCK and an
     * undriven MOSI must not float. */
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = BB_SPI_AF;
    g.Pin       = BB_SPI_SCK_PIN;
    HAL_GPIO_Init(BB_SPI_SCK_PORT, &g);
    g.Pin = BB_SPI_MISO_PIN;
    HAL_GPIO_Init(BB_SPI_MISO_PORT, &g);
    g.Pin  = BB_SPI_MOSI_PIN;
    g.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(BB_SPI_MOSI_PORT, &g);

    /* Chip select: input, pulled up so an unconnected board is simply not
     * selected, with both edges to EXTI to frame the transaction. */
    g.Pin  = BB_SPI_CS_PIN;
    g.Mode = GPIO_MODE_IT_RISING_FALLING;
    g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(BB_SPI_CS_PORT, &g);

    hspi.Instance               = BB_SPI_INSTANCE;
    hspi.Init.Mode              = SPI_MODE_SLAVE;
    hspi.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi.Init.CLKPolarity       = (spi_mode & 0x2u) ? SPI_POLARITY_HIGH
                                                     : SPI_POLARITY_LOW;
    hspi.Init.CLKPhase          = (spi_mode & 0x1u) ? SPI_PHASE_2EDGE
                                                     : SPI_PHASE_1EDGE;
    hspi.Init.NSS               = SPI_NSS_SOFT;
    hspi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;  /* unused by a slave */
    hspi.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    hspi.Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;
    hspi.Init.FifoThreshold     = SPI_FIFO_THRESHOLD_01DATA;

    if (HAL_SPI_Init(&hspi) != HAL_OK) {
        return false;
    }

    /* From here the peripheral is driven by hand. TSIZE stays 0: a slave that
     * declares a length stops after it, and this one has no idea how many bytes
     * the controller intends to clock. */
    hspi.Instance->CR2 = 0;
    hspi.Instance->IER = SPI_IER_RXPIE;
    hspi.Instance->CR1 |= SPI_CR1_SPE;

    state      = ST_OPCODE;
    status_reg = 0;
    tx_push(0xFF);

    /* Top priority among this profile's vectors. A late byte here is a wrong
     * byte on the wire, where a late I2C event only stretches the clock. */
    HAL_NVIC_SetPriority(BB_SPI_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(BB_SPI_IRQn);

    HAL_NVIC_SetPriority(cs_exti_irqn(), 4, 0);
    HAL_NVIC_EnableIRQ(cs_exti_irqn());

    configured = true;
    return true;
}

void bb_ee_spi_init(void)
{
    enabled  = true;
    spi_mode = 0;
    if (!apply()) {
        bb_error_handler("ee:spi:init");
    }
}

void bb_ee_spi_describe(char *out, size_t cap)
{
    if (!enabled) {
        snprintf(out, cap, "off");
    } else {
        snprintf(out, cap, "mode=%u id=%02X%02X%02X cs-exti=%u",
                 spi_mode, device_id[0], device_id[1], device_id[2],
                 bb_ee_spi_cs_line());
    }
}

void bb_ee_spi_status(bb_status_t *out)
{
    const bb_ee_stats_t *s = bb_ee_stats(BB_EE_IF_SPI);

    snprintf(out->pins, sizeof out->pins, "slave m%u", spi_mode);
    if (!enabled) {
        snprintf(out->detail, sizeof out->detail, "off");
        out->state = BB_STATE_IDLE;
        return;
    }
    snprintf(out->detail, sizeof out->detail, "cs%lu r%lu w%lu",
             (unsigned long)frames, (unsigned long)s->reads,
             (unsigned long)s->writes);
    out->state = ((status_reg & 0x40u) != 0u) ? BB_STATE_PROBLEM
               : (frames > 0u ? BB_STATE_ACTIVE : BB_STATE_IDLE);
}

/* ---- commands ---------------------------------------------------------- */

static void cmd_spi(int argc, char **argv)
{
    uint32_t m;

    if (argc == 1) {
        bb_reply_okf("mode=%u frames=%lu ovr=%u %s", spi_mode,
                     (unsigned long)frames,
                     (status_reg & 0x40u) ? 1u : 0u,
                     enabled ? "on" : "off");
        return;
    }
    if (bb_streq_ci(argv[1], "OFF")) {
        enabled = false;
        (void)apply();
        bb_reply_ok();
        return;
    }
    if (bb_streq_ci(argv[1], "ON")) {
        enabled = true;
        if (!apply()) {
            bb_reply_err(BB_ERR_HAL, "ee-spi-init");
            return;
        }
        bb_reply_ok();
        return;
    }
    if (!bb_parse_u32_max(argv[1], 3, &m)) {
        bb_reply_err(BB_ERR_RANGE, "mode-0..3-or-ON-OFF");
        return;
    }
    spi_mode = (uint8_t)m;
    enabled  = true;
    if (!apply()) {
        bb_reply_err(BB_ERR_HAL, "ee-spi-init");
        return;
    }
    bb_reply_okf("mode=%u", spi_mode);
}

const bb_cmd_t bb_cmd_ee_spi[] = {
    { "EE:SPI", 0, 1, cmd_spi, "EE:SPI [<mode 0..3> | ON | OFF]" },
    { NULL, 0, 0, NULL, NULL },
};
