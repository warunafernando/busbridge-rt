/**
 * @file profiles/peripheral/bb_ee_i2c.c
 * @brief An I2C EEPROM, in the shape of a 24C08 with a two-byte address.
 *
 * The protocol is the one every I2C EEPROM uses, so an existing host driver
 * needs no changes to talk to it:
 *
 *   write:          S ADDR+W  ah al  d0 d1 ...  P     pointer, then data
 *   random read:    S ADDR+W  ah al  Sr ADDR+R  d0 ...  P
 *   current read:   S ADDR+R  d0 d1 ...  P           continues where it stopped
 *
 * Two address bytes rather than one, big-endian, because one byte cannot reach
 * 1 KB and the block-select-bits-in-the-device-address trick that the real
 * 24C08 uses would spend four I2C addresses on a part this small.
 *
 * The pointer auto-increments and wraps at the end of the array, as the real
 * part does within its page. Nothing here knows what the bytes mean; bb_ee.c
 * owns them, and the other three buses see every write this one makes.
 *
 * Implementation note: the transaction is driven a byte at a time from the
 * event interrupt, not with a length known up front. A controller may stop a
 * write after two bytes (a pointer set), after three (one byte of data), or
 * after two hundred, and it never says which in advance - so the only correct
 * length for a slave receive is one, repeated.
 */
#include "bb_board.h"
#include "bb_cmd.h"
#include "bb_ee.h"
#include "bb_ee_ports.h"
#include "bb_i2c_timing.h"

#include <stdio.h>

/* Fast mode. A target must be configured for the fastest bus it may sit on:
 * SDADEL and SCLDEL come from this and a target set up for 100 kHz holds SDA
 * too long for a 400 kHz controller. It costs nothing to be ready. */
#define EE_I2C_HZ          400000UL
#define EE_I2C_DEFAULT_ADDR  0x50u      /* where every EEPROM driver looks first */

/* Where the transaction is: how many of the two address bytes have arrived. */
typedef enum {
    PHASE_ADDR_HI = 0,
    PHASE_ADDR_LO,
    PHASE_DATA
} phase_t;

static I2C_HandleTypeDef hi2c;
static uint8_t   own_addr = EE_I2C_DEFAULT_ADDR;
static unsigned  bus_index;                  /* 0-based, into the board's list */
static bool      enabled;
static bool      configured;

static volatile uint16_t pointer;            /* the EEPROM's address pointer */
static volatile phase_t  phase;
static volatile uint8_t  rx_byte;
static volatile uint8_t  tx_byte;
static volatile uint32_t bus_errors;

/* ---- the board's I2C buses ---------------------------------------------- */
/* Same macros the bridge's controller uses, so the target lands on the pins the
 * board says are that bus - including the alternate function, which differs per
 * pin and not per peripheral. */
typedef struct {
    const char  *name;
    I2C_TypeDef *inst;
    bb_pin_t     scl;
    bb_pin_t     sda;
    const char  *pins;
    IRQn_Type    ev_irq;
    IRQn_Type    er_irq;
    bool         shares_i3c;
} ee_bus_t;

static const ee_bus_t buses[BB_I2C_BUS_COUNT] = {
    { "I2C1", I2C1, BB_I2C1_SCL, BB_I2C1_SDA, BB_I2C1_PINS,
      I2C1_EV_IRQn, I2C1_ER_IRQn,
#if defined(BB_I3C_P1_SHARES_I2C) && BB_I3C_P1_SHARES_I2C
      true
#else
      false
#endif
    },
    { "I2C2", I2C2, BB_I2C2_SCL, BB_I2C2_SDA, BB_I2C2_PINS,
      I2C2_EV_IRQn, I2C2_ER_IRQn, false },
    { "I2C3", I2C3, BB_I2C3_SCL, BB_I2C3_SDA, BB_I2C3_PINS,
      I2C3_EV_IRQn, I2C3_ER_IRQn, false },
#if BB_I2C_BUS_COUNT >= 4
    { "I2C4", I2C4, BB_I2C4_SCL, BB_I2C4_SDA, BB_I2C4_PINS,
      I2C4_EV_IRQn, I2C4_ER_IRQn, false },
#endif
};

/**
 * @brief The bus to sit on unless told otherwise.
 *
 * The first one whose pins the board does not share with I3C1. On a 64-pin part
 * I2C1 and I3C1 are the same two pins, and a peripheral board that answered as
 * an I2C EEPROM there would have no I3C target at all - the more interesting of
 * the two, and the harder to test elsewhere. Boards with pins to spare get
 * I2C1, which is where a controller looks first.
 */
static unsigned default_bus(void)
{
    for (unsigned i = 0; i < BB_I2C_BUS_COUNT; i++) {
        if (!buses[i].shares_i3c) {
            return i;
        }
    }
    return 0;
}

static void clock_enable(const ee_bus_t *b)
{
    if (b->inst == I2C1) {
        __HAL_RCC_I2C1_CLK_ENABLE();
    } else if (b->inst == I2C2) {
        __HAL_RCC_I2C2_CLK_ENABLE();
    } else if (b->inst == I2C3) {
        __HAL_RCC_I2C3_CLK_ENABLE();
#if BB_I2C_BUS_COUNT >= 4
    } else if (b->inst == I2C4) {
        __HAL_RCC_I2C4_CLK_ENABLE();
#endif
    }
}

/* ---- the transaction --------------------------------------------------- */

/** Arm for one more received byte. A slave never knows how many are coming. */
static void arm_receive(void)
{
    (void)HAL_I2C_Slave_Seq_Receive_IT(&hi2c, (uint8_t *)&rx_byte, 1,
                                       I2C_NEXT_FRAME);
}

static void arm_transmit(void)
{
    tx_byte = bb_ee_read(BB_EE_IF_I2C, pointer);
    pointer = (uint16_t)((pointer + 1u) & BB_EE_MASK);
    (void)HAL_I2C_Slave_Seq_Transmit_IT(&hi2c, (uint8_t *)&tx_byte, 1,
                                        I2C_NEXT_FRAME);
}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *h, uint8_t direction,
                          uint16_t addr_match)
{
    (void)addr_match;
    if (h != &hi2c) {
        return;
    }

    bb_ee_note_transaction(BB_EE_IF_I2C);

    if (direction == I2C_DIRECTION_TRANSMIT) {
        /* Controller is writing: the next two bytes are the address. */
        phase = PHASE_ADDR_HI;
        arm_receive();
    } else {
        /* Controller is reading. Either it set the pointer with a write and a
         * repeated start, in which case pointer is already right, or this is a
         * current-address read that carries on from the last byte. */
        arm_transmit();
    }
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *h)
{
    if (h != &hi2c) {
        return;
    }

    switch (phase) {
    case PHASE_ADDR_HI:
        pointer = (uint16_t)((uint16_t)rx_byte << 8);
        phase   = PHASE_ADDR_LO;
        break;
    case PHASE_ADDR_LO:
        pointer = (uint16_t)((pointer | rx_byte) & BB_EE_MASK);
        phase   = PHASE_DATA;
        break;
    case PHASE_DATA:
    default:
        bb_ee_write(BB_EE_IF_I2C, pointer, rx_byte);
        pointer = (uint16_t)((pointer + 1u) & BB_EE_MASK);
        break;
    }
    arm_receive();
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *h)
{
    if (h == &hi2c) {
        arm_transmit();
    }
}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *h)
{
    if (h == &hi2c) {
        (void)HAL_I2C_EnableListen_IT(h);
    }
}

/**
 * @brief End of transaction, usually.
 *
 * A read ends with the controller not acknowledging the last byte, which the
 * peripheral reports as an acknowledge failure. That is the normal and only way
 * an I2C read can end, so it is not counted as an error - counting it would
 * make every successful read look like a fault.
 */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *h)
{
    if (h != &hi2c) {
        return;
    }
    if ((HAL_I2C_GetError(h) & ~(uint32_t)HAL_I2C_ERROR_AF) != 0u) {
        bus_errors++;
    }
    (void)HAL_I2C_EnableListen_IT(h);
}

/* ---- bring-up ---------------------------------------------------------- */

static bool apply(void)
{
    const ee_bus_t *b = &buses[bus_index];

    if (configured) {
        (void)HAL_I2C_DisableListen_IT(&hi2c);
        (void)HAL_I2C_DeInit(&hi2c);
        HAL_NVIC_DisableIRQ(b->ev_irq);
        HAL_NVIC_DisableIRQ(b->er_irq);
        configured = false;
    }
    if (!enabled) {
        return true;
    }

    bb_pins_i2c(&b->scl, &b->sda);
    clock_enable(b);

    hi2c.Instance              = b->inst;
    hi2c.Init.Timing           = bb_i2c_timingr(EE_I2C_HZ);
    /* The peripheral wants the address in bits 7:1. */
    hi2c.Init.OwnAddress1      = (uint32_t)own_addr << 1;
    hi2c.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    hi2c.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    hi2c.Init.OwnAddress2      = 0;
    hi2c.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    /* Stretching stays on: this target reads and writes RAM from an interrupt,
     * and a controller that clocks faster than the interrupt can answer must be
     * held off rather than given a stale byte. */
    hi2c.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&hi2c) != HAL_OK) {
        return false;
    }
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
        return false;
    }

    /* Above the console and USB, below the sniffer's taps in the other profile:
     * a missed slave event stretches the clock at worst, but a missed one during
     * a transfer wedges the controller waiting for a byte. */
    HAL_NVIC_SetPriority(b->ev_irq, 5, 0);
    HAL_NVIC_EnableIRQ(b->ev_irq);
    HAL_NVIC_SetPriority(b->er_irq, 5, 0);
    HAL_NVIC_EnableIRQ(b->er_irq);

    if (HAL_I2C_EnableListen_IT(&hi2c) != HAL_OK) {
        return false;
    }
    configured = true;
    return true;
}

void bb_ee_i2c_init(void)
{
    bus_index = default_bus();
    enabled   = true;
    (void)apply();
}

void bb_ee_i2c_ev_irq(unsigned n)
{
    if (configured && n == bus_index) {
        HAL_I2C_EV_IRQHandler(&hi2c);
    }
}

void bb_ee_i2c_er_irq(unsigned n)
{
    if (configured && n == bus_index) {
        HAL_I2C_ER_IRQHandler(&hi2c);
    }
}

void bb_ee_i2c_describe(char *out, size_t cap)
{
    if (!enabled) {
        snprintf(out, cap, "off");
    } else if (!configured) {
        snprintf(out, cap, "failed");
    } else {
        snprintf(out, cap, "bus=%u addr=0x%02X %s",
                 bus_index + 1u, own_addr, buses[bus_index].pins);
    }
}

void bb_ee_i2c_status(bb_status_t *out)
{
    const bb_ee_stats_t *s = bb_ee_stats(BB_EE_IF_I2C);

    snprintf(out->pins, sizeof out->pins, "%s", buses[bus_index].pins);
    if (!enabled) {
        snprintf(out->detail, sizeof out->detail, "off");
        out->state = BB_STATE_IDLE;
        return;
    }
    snprintf(out->detail, sizeof out->detail, "0x%02X t%lu", own_addr,
             (unsigned long)s->transactions);
    out->state = bus_errors > 0u ? BB_STATE_PROBLEM
               : (s->transactions > 0u ? BB_STATE_ACTIVE : BB_STATE_IDLE);
}

/* ---- commands ---------------------------------------------------------- */

static void cmd_i2c(int argc, char **argv)
{
    uint32_t bus;
    uint32_t addr;

    if (argc == 1) {
        char detail[64];
        bb_ee_i2c_describe(detail, sizeof detail);
        bb_reply_okf("%s errors=%lu", detail, (unsigned long)bus_errors);
        return;
    }

    if (bb_streq_ci(argv[1], "OFF")) {
        enabled = false;
        (void)apply();
        bb_reply_ok();
        return;
    }

    if (!bb_parse_u32_max(argv[1], BB_I2C_BUS_COUNT, &bus) || bus < 1u) {
        bb_reply_errf(BB_ERR_RANGE, "bus-1..%u-or-OFF", (unsigned)BB_I2C_BUS_COUNT);
        return;
    }
    addr = own_addr;
    if (argc >= 3 && (!bb_parse_u32_max(argv[2], 0x7Fu, &addr) || addr < 0x08u)) {
        bb_reply_err(BB_ERR_RANGE, "addr-0x08..0x7F");
        return;
    }

    /* Sitting on the pins I3C1 is using would make both silent, and the failure
     * looks like a wiring fault rather than a configuration one. */
    if (buses[bus - 1u].shares_i3c && bb_ee_i3c_enabled()) {
        bb_reply_errf(BB_ERR_BUS_STATE, "bus%lu-shares-pins-with-i3c-target",
                      (unsigned long)bus);
        return;
    }

    bus_index = (unsigned)(bus - 1u);
    own_addr  = (uint8_t)addr;
    enabled   = true;
    if (!apply()) {
        bb_reply_err(BB_ERR_HAL, "i2c-target-init");
        return;
    }
    bb_reply_okf("bus=%lu addr=0x%02X %s", (unsigned long)bus,
                 own_addr, buses[bus_index].pins);
}

const bb_cmd_t bb_cmd_ee_i2c[] = {
    { "EE:I2C", 0, 2, cmd_i2c, "EE:I2C [<bus> [addr] | OFF]" },
    { NULL, 0, 0, NULL, NULL },
};
