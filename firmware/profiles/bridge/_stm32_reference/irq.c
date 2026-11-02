/**
 * @file profiles/bridge/irq.c
 * @brief Bus interrupt vectors for the bridge profile.
 *
 * Split out of bb_it.c when the peripheral profile arrived: both images want
 * I2C1_EV_IRQHandler and I3C1_EV_IRQHandler, one to run a controller and one to
 * answer as a target, and a vector can only be defined once per image.
 */
#include "bb_board.h"
#include "bb_i2c.h"
#include "bb_i3c.h"
#include "bb_sniff.h"
#include "bb_uart.h"

void BB_TARGET_IRQHANDLER(void)
{
    bb_uart_irq();
}

void I2C1_EV_IRQHandler(void) { bb_i2c_ev_irq(0); }
void I2C1_ER_IRQHandler(void) { bb_i2c_er_irq(0); }
void I2C2_EV_IRQHandler(void) { bb_i2c_ev_irq(1); }
void I2C2_ER_IRQHandler(void) { bb_i2c_er_irq(1); }
void I2C3_EV_IRQHandler(void) { bb_i2c_ev_irq(2); }
void I2C3_ER_IRQHandler(void) { bb_i2c_er_irq(2); }
void I2C4_EV_IRQHandler(void) { bb_i2c_ev_irq(3); }
void I2C4_ER_IRQHandler(void) { bb_i2c_er_irq(3); }

/* The sniffer's taps.  These two are the highest-priority interrupts in the
 * system; see bb_sniff.c for why. */
void BB_SNIFF_SCL_HANDLER(void)
{
    bb_sniff_scl_irq();
}

void BB_SNIFF_SDA_HANDLER(void)
{
    bb_sniff_sda_irq();
}

void I3C1_EV_IRQHandler(void)
{
    bb_i3c_ev_irq();
}

/* Feeds an I3C target's replies. Only the transmit direction uses DMA; see
 * bb_i3c.c for why the receive side is hand-fed. */
void GPDMA1_Channel0_IRQHandler(void)
{
    bb_i3c_dma_irq();
}

void I3C1_ER_IRQHandler(void)
{
    bb_i3c_er_irq();
}
