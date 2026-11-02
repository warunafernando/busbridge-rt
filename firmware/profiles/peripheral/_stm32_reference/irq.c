/**
 * @file profiles/peripheral/irq.c
 * @brief Bus interrupt vectors for the peripheral profile.
 *
 * The same vectors the bridge profile claims, pointed at targets instead of
 * controllers. A vector can be defined once per image, which is the reason the
 * two profiles are two images and not one with a mode switch.
 *
 * All sixteen EXTI lines are defined here and dispatched by line number, rather
 * than each board naming the two vectors this profile happens to need. Which pin
 * the SPI chip select and MDC sit on is already a board fact in board.h; a
 * vector name beside it would be a second place to keep in step, and the wrong
 * one would fail as a silently dead interrupt.
 */
#include "bb_board.h"
#include "bb_ee_ports.h"
#include "bb_i3c.h"

/* ---- I2C: one target, but the board decides which bus it sits on --------- */
void I2C1_EV_IRQHandler(void) { bb_ee_i2c_ev_irq(0); }
void I2C1_ER_IRQHandler(void) { bb_ee_i2c_er_irq(0); }
void I2C2_EV_IRQHandler(void) { bb_ee_i2c_ev_irq(1); }
void I2C2_ER_IRQHandler(void) { bb_ee_i2c_er_irq(1); }
void I2C3_EV_IRQHandler(void) { bb_ee_i2c_ev_irq(2); }
void I2C3_ER_IRQHandler(void) { bb_ee_i2c_er_irq(2); }
#if BB_I2C_BUS_COUNT >= 4
void I2C4_EV_IRQHandler(void) { bb_ee_i2c_ev_irq(3); }
void I2C4_ER_IRQHandler(void) { bb_ee_i2c_er_irq(3); }
#endif

/* ---- I3C ---------------------------------------------------------------- */
void I3C1_EV_IRQHandler(void) { bb_ee_i3c_ev_irq(); }
void I3C1_ER_IRQHandler(void) { bb_ee_i3c_er_irq(); }

/* Feeds the target's reply window; see bb_ee_i3c.c. */
void GPDMA1_Channel0_IRQHandler(void) { bb_i3c_dma_irq(); }

/* ---- SPI ---------------------------------------------------------------- */
void BB_SPI_IRQHANDLER(void) { bb_ee_spi_irq(); }

/* ---- EXTI: the SPI chip select and MDC ---------------------------------- */
/* HAL_GPIO_EXTI_IRQHandler clears the pending flag and calls the rising or
 * falling callback; going through it rather than clearing by hand keeps the
 * two edges of the chip select distinguishable. */
void bb_ee_exti(unsigned line)
{
    if (line == bb_ee_spi_cs_line()) {
        bb_ee_spi_cs_edge();
    } else if (line == bb_ee_mdio_mdc_line()) {
        bb_ee_mdio_mdc_edge();
    }
}

#define EXTI_VECTOR(n)                              \
    void EXTI##n##_IRQHandler(void)                 \
    {                                               \
        __HAL_GPIO_EXTI_CLEAR_FALLING_IT(1u << (n)); \
        __HAL_GPIO_EXTI_CLEAR_RISING_IT(1u << (n));  \
        bb_ee_exti(n);                              \
    }

EXTI_VECTOR(0)
EXTI_VECTOR(1)
EXTI_VECTOR(2)
EXTI_VECTOR(3)
EXTI_VECTOR(4)
EXTI_VECTOR(5)
EXTI_VECTOR(6)
EXTI_VECTOR(7)
EXTI_VECTOR(8)
EXTI_VECTOR(9)
EXTI_VECTOR(10)
EXTI_VECTOR(11)
EXTI_VECTOR(12)
EXTI_VECTOR(13)
EXTI_VECTOR(14)
EXTI_VECTOR(15)
