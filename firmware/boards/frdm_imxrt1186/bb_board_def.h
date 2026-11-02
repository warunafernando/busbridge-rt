/**
 * @file board.h
 * @brief FRDM-IMXRT1186 — pin map, clock choices and peripheral selection.
 *
 * Everything here is a property of this board, checked against the schematic
 * netlist (SCH-95302 Rev A4) and UM12450 Rev 3.0. The reasoning behind each
 * allocation is in docs/PINS.md; the short version of the constraints:
 *
 *   - The Arduino headers carry 22 signals. J52 is power only.
 *   - LPI2C6 reaches no connector on this package. It cannot be used.
 *   - LPI2C2 reaches only the motor connector J3, not the Arduino headers.
 *   - So three I2C masters is the ceiling, not four as on the NUCLEO.
 *   - Most pads reach the header through a 3-pin selection jumper. The board
 *     is already a pin-sharing matrix; several choices below are really
 *     "which way is the jumper".
 */
#ifndef BB_BOARD_FRDM_IMXRT1186_H
#define BB_BOARD_FRDM_IMXRT1186_H

#define BB_BOARD_NAME "FRDM-IMXRT1186"

/* ---- silicon expectation ----------------------------------------------- */
/* Checked at boot so a build for one part cannot quietly run on another and
 * report the wrong pin map. There is no DBGMCU IDCODE here as there is on the
 * STM32; the RT1180 carries its identity in OCOTP fuses, read by bb_board.c. */
#define BB_EXPECT_SILICON_ID  0x1186u

/* ---- clocks ------------------------------------------------------------ */
/* 24 MHz crystal Y1. The CM33 runs at 240 MHz and the CM7 at 800 MHz; both are
 * set up by the SDK clock config, which this board layer calls rather than
 * reimplementing. */
#define BB_XTAL_HZ            24000000UL
#define BB_CM33_HZ            240000000UL
#define BB_CM7_HZ             800000000UL

/* ---- console ----------------------------------------------------------- */
/* LPUART1 on GPIO_AON_08/09, which is where the MCU-Link VCOM lands - the port
 * that shows up on the host as a COM port with no extra cable.
 *
 * These two pads are also Arduino D1/D0, selected by jumpers J43/J41. Keeping
 * the console here is why LPI2C1 is not enabled by default: see below. */
#define BB_CONSOLE_UART       LPUART1
#define BB_CONSOLE_IRQ        LPUART1_IRQn
#define BB_CONSOLE_IRQHANDLER LPUART1_IRQHandler
#define BB_CONSOLE_CLKROOT    kCLOCK_Root_Lpuart0102
#define BB_CONSOLE_TX_PIN     BB_PIN(IOMUXC_GPIO_AON_08_LPUART1_TX, RGPIO1, 8)
#define BB_CONSOLE_RX_PIN     BB_PIN(IOMUXC_GPIO_AON_09_LPUART1_RX, RGPIO1, 9)
#define BB_CONSOLE_PINS       "AON_08/AON_09"
#define BB_CONSOLE_BAUD       115200U

/* ---- LEDs and button --------------------------------------------------- */
/* RGB LED D4. Red and green are on GPIO2, blue on GPIO3. All active high on
 * this board, unlike the NUCLEO's active-low user LEDs. */
#define BB_LED_COUNT_HW       3u
#define BB_LED_ACTIVE_LOW     0
#define BB_LED_NAMES          { "green", "yellow", "red" }
/* Index order follows bb_led_t: GREEN is the heartbeat, YELLOW is activity,
 * RED is fault. This board has no yellow, so blue takes that role - the
 * indication means the same thing, which is what matters. */
#define BB_LED_PINS { \
    BB_PIN(IOMUXC_GPIO_EMC_B1_11_GPIO2_IO11, RGPIO2, 11), /* green  */ \
    BB_PIN(IOMUXC_GPIO_EMC_B1_39_GPIO3_IO07, RGPIO3,  7), /* blue   */ \
    BB_PIN(IOMUXC_GPIO_EMC_B1_09_GPIO2_IO09, RGPIO2,  9)  /* red    */ \
}

/* SW4, via jumper J30. J30 in position 1-2 gives the button; position 2-3
 * gives Arduino D6 instead. Default is the button. */
#define BB_BUTTON_PIN         BB_PIN(IOMUXC_GPIO_AD_12_GPIO4_IO12, RGPIO4, 12)
#define BB_BUTTON_ACTIVE_LOW  1

/* ---- I2C --------------------------------------------------------------- */
/*
 * Three masters, and the third costs something. Published as three so the host
 * sees the truth; bus 1 reports itself absent until the jumpers are moved.
 *
 *   bus 1  LPI2C1  D0/D1        needs J41 and J43 moved to 2-3, which takes
 *                               the pads away from LPUART1 and so removes the
 *                               MCU-Link console. Build with -DBB_I2C1_ENABLE=1
 *                               and move the console to LPUART3 (D7/A3).
 *   bus 2  LPI2C2  J3.12/J3.10  motor connector. Shares its pads with I3C1 -
 *                               one or the other, arbitrated at runtime.
 *   bus 3  LPI2C3  D14/D15      the Arduino I2C position.
 *
 * WARNING, unresolved: UM12450 Table 22 labels J2.18 "D14/I2C_SDA" and J2.20
 * "D15/I2C_SCL", but the IOMUX has GPIO_EMC_B2_19 (D14) as LPI2C3_SCL and
 * GPIO_EMC_B2_20 (D15) as LPI2C3_SDA - crossed relative to the silkscreen.
 * The map below follows the silicon, not the silkscreen. Verify on the bench
 * before trusting either. See docs/PINS.md section 4(a).
 */
#define BB_I2C_BUS_COUNT      3u

#ifndef BB_I2C1_ENABLE
#  define BB_I2C1_ENABLE      0     /* off: would cost the console */
#endif

#define BB_I2C1_INSTANCE      LPI2C1
#define BB_I2C1_SCL           BB_PIN(IOMUXC_GPIO_AON_09_LPI2C1_SCL, RGPIO1, 9)
#define BB_I2C1_SDA           BB_PIN(IOMUXC_GPIO_AON_08_LPI2C1_SDA, RGPIO1, 8)
#define BB_I2C1_PINS          "AON_09/AON_08"

#define BB_I2C2_INSTANCE      LPI2C2
#define BB_I2C2_SCL           BB_PIN(IOMUXC_GPIO_AON_16_LPI2C2_SCL, RGPIO1, 16)
#define BB_I2C2_SDA           BB_PIN(IOMUXC_GPIO_AON_15_LPI2C2_SDA, RGPIO1, 15)
#define BB_I2C2_PINS          "AON_16/AON_15"
#define BB_I2C2_SHARES_I3C1   1

#define BB_I2C3_INSTANCE      LPI2C3
#define BB_I2C3_SCL           BB_PIN(IOMUXC_GPIO_EMC_B2_19_LPI2C3_SCL, RGPIO3, 29)
#define BB_I2C3_SDA           BB_PIN(IOMUXC_GPIO_EMC_B2_20_LPI2C3_SDA, RGPIO3, 30)
#define BB_I2C3_PINS          "EMC_B2_19/EMC_B2_20"

/* ---- I3C --------------------------------------------------------------- */
/*
 * Two controllers, which the STM32 build never had. Port numbering here is per
 * controller, not per pin pair - that is the one place this board header
 * deliberately differs from the NUCLEO's, where BB_I3C_PORT_COUNT counted pin
 * pairs of a single I3C1.
 *
 *   port 1  I3C1  J3.12/J3.10   shares pads with LPI2C2
 *   port 2  I3C2  A5/A4         sole option for I3C2 on this package
 */
#define BB_I3C_PORT_COUNT     2u

#define BB_I3C_P1_INSTANCE    I3C1
#define BB_I3C_P1_SCL         BB_PIN(IOMUXC_GPIO_AON_16_I3C1_SCL, RGPIO1, 16)
#define BB_I3C_P1_SDA         BB_PIN(IOMUXC_GPIO_AON_15_I3C1_SDA, RGPIO1, 15)
#define BB_I3C_P1_PINS        "AON_16/AON_15"
#define BB_I3C_P1_SHARES_I2C  2     /* bus number it takes the pads from */

#define BB_I3C_P2_INSTANCE    I3C2
#define BB_I3C_P2_SCL         BB_PIN(IOMUXC_GPIO_AD_18_I3C2_SCL, RGPIO4, 18)
#define BB_I3C_P2_SDA         BB_PIN(IOMUXC_GPIO_AD_19_I3C2_SDA, RGPIO4, 19)
#define BB_I3C_P2_PINS        "AD_18/AD_19"
#define BB_I3C_P2_SHARES_I2C  0     /* nothing: LPI2C3 lives on D14/D15 */

/* ---- SPI --------------------------------------------------------------- */
/* LPSPI4 in the standard Arduino SPI position, D10-D13. Chip select is driven
 * in software, as on the STM32, so a transfer can hold CS across several
 * commands. */
#define BB_SPI_INSTANCE       LPSPI4
#define BB_SPI_IRQ            LPSPI4_IRQn
#define BB_SPI_IRQHANDLER     LPSPI4_IRQHandler
#define BB_SPI_CLKROOT        kCLOCK_Root_Lpspi0304
#define BB_SPI_SCK            BB_PIN(IOMUXC_GPIO_EMC_B1_22_LPSPI4_SCK, RGPIO2, 22)
#define BB_SPI_SDO            BB_PIN(IOMUXC_GPIO_EMC_B1_24_LPSPI4_SDO, RGPIO2, 24)
#define BB_SPI_SDI            BB_PIN(IOMUXC_GPIO_EMC_B1_23_LPSPI4_SDI, RGPIO2, 23)
#define BB_SPI_CS             BB_PIN(IOMUXC_GPIO_EMC_B1_25_GPIO2_IO25, RGPIO2, 25)
#define BB_SPI_PINS           "EMC_B1_22/24/23/25"

/* ---- target UART ------------------------------------------------------- */
/* LPUART4 on D2/D3. Separate from the console so a host can drive a target's
 * serial port without losing the transport it is issuing commands over. */
/*
 * LPUART4's flow control, on D4 and D5.
 *
 * These exist, contrary to what this file used to say. The earlier comment
 * claimed LPUART4's RTS/CTS pads were not brought out to the headers and
 * UART:CFG refused `rtscts` on that basis - which was simply wrong:
 * GPIO_EMC_B1_14 carries LPUART4_CTS_B and GPIO_EMC_B1_15 carries
 * LPUART4_RTS_B, and both are header pins (D4 on J1-10, D5 on J1-12).
 *
 * They are shared with sniffer taps d4 and d5, and that sharing is declared
 * rather than hidden: asking for flow control takes those two pads away from
 * the capture engine, because a pad can only be muxed to one peripheral. The
 * spec allows exactly this - "if needed we can use the same pins for different
 * ports" - and the alternative pads are worse: LPUART4_RTS_B is otherwise only
 * on GPIO_EMC_B1_22, which is D13, which is spi.sck. Sharing with one
 * subsystem beats splitting the conflict across two.
 */
#define BB_TARGET_UART_HAS_FLOW  1
#define BB_TARGET_UART_CTS_PIN   BB_PIN(IOMUXC_GPIO_EMC_B1_14_LPUART4_CTS_B, RGPIO2, 14)
#define BB_TARGET_UART_RTS_PIN   BB_PIN(IOMUXC_GPIO_EMC_B1_15_LPUART4_RTS_B, RGPIO2, 15)

#define BB_TARGET_UART        LPUART4
#define BB_TARGET_UART_IRQ    LPUART4_IRQn
#define BB_TARGET_IRQHANDLER  LPUART4_IRQHandler
#define BB_TARGET_CLKROOT     kCLOCK_Root_Lpuart0304
#define BB_TUART_TX           BB_PIN(IOMUXC_GPIO_EMC_B1_12_LPUART4_TX, RGPIO2, 12)
#define BB_TUART_RX           BB_PIN(IOMUXC_GPIO_EMC_B1_13_LPUART4_RX, RGPIO2, 13)
#define BB_TUART_PINS         "EMC_B1_12/EMC_B1_13"

/* ---- MDIO -------------------------------------------------------------- */
/* Bit-banged, as on the STM32, and for a reason worth recording: this part has
 * a hardware MDIO in ENETC, but EMDIO reaches none of the 22 Arduino pads - it
 * is wired to the board's own PHYs. Hardware MDIO can therefore talk to the
 * on-board PHYs and to nothing else, so external MDIO stays bit-banged.
 *
 * D8/D9. Kept on separate GPIO bits from the SPI chip select, as the STM32
 * lint required, though the reason no longer applies: RGPIO has per-pin
 * interrupts rather than one EXTI line shared across ports. */
#define BB_MDC                BB_PIN(IOMUXC_GPIO_EMC_B1_17_GPIO2_IO17, RGPIO2, 17)
#define BB_MDIO               BB_PIN(IOMUXC_GPIO_EMC_B1_16_GPIO2_IO16, RGPIO2, 16)
#define BB_MDIO_PINS          "EMC_B1_17/EMC_B1_16"

/* ---- CAN --------------------------------------------------------------- */
/* FlexCAN1 to the on-board transceiver U13, through jumpers J44 and J46 in
 * position 1-2.
 *
 * The CAN H/L connector J22 is DNP on this board - the transceiver is fitted
 * and wired, but there is no header to plug a bus into. Fit one before
 * expecting traffic. */
#define BB_CAN_INSTANCE       CAN1
#define BB_CAN_CLKROOT        kCLOCK_Root_Can1
#define BB_CAN_TX             BB_PIN(IOMUXC_GPIO_AON_06_CAN1_TX, RGPIO1, 6)
#define BB_CAN_RX             BB_PIN(IOMUXC_GPIO_AON_07_CAN1_RX, RGPIO1, 7)
#define BB_CAN_PINS           "AON_06/AON_07"
#define BB_CAN_NEEDS_J22      1

/* ---- sniffer taps ------------------------------------------------------ */
/*
 * FlexIO sits at ALT 8 on nearly every Arduino pad, which is what makes a
 * hardware-clocked capture possible at all. The pads below are the ones left
 * over after the allocations above - seven of them, enough to tap one two-wire
 * bus and one four-wire SPI at the same time, or three two-wire buses.
 *
 * Which of them a capture channel uses is chosen at runtime by SNIFF:CH, not
 * fixed here, because no fixed choice fits every combination.
 *
 * FLEXIO1 and FLEXIO2 are separate peripherals and a channel cannot straddle
 * them, so the split is recorded.
 */
#define BB_SNIFF_FLEXIO1      FLEXIO1
#define BB_SNIFF_FLEXIO2      FLEXIO2

#define BB_SNIFF_TAPS { \
    { "d4", BB_PIN(IOMUXC_GPIO_EMC_B1_14_FLEXIO1_FLEXIO14, RGPIO2, 14), 1, 14 }, \
    { "d5", BB_PIN(IOMUXC_GPIO_EMC_B1_15_FLEXIO1_FLEXIO15, RGPIO2, 15), 1, 15 }, \
    { "d8", BB_PIN(IOMUXC_GPIO_EMC_B1_17_FLEXIO1_FLEXIO17, RGPIO2, 17), 1, 17 }, \
    { "d9", BB_PIN(IOMUXC_GPIO_EMC_B1_16_FLEXIO1_FLEXIO16, RGPIO2, 16), 1, 16 }, \
    { "a0", BB_PIN(IOMUXC_GPIO_AD_15_FLEXIO2_FLEXIO15,     RGPIO4, 15), 2, 15 }, \
    { "a1", BB_PIN(IOMUXC_GPIO_AD_16_FLEXIO2_FLEXIO16,     RGPIO4, 16), 2, 16 }, \
    { "a2", BB_PIN(IOMUXC_GPIO_AD_17_FLEXIO2_FLEXIO17,     RGPIO4, 17), 2, 17 }  \
}
#define BB_SNIFF_TAP_COUNT    7u

/* ---- what this board does not have ------------------------------------- */
#define BB_HAS_DISPLAY        0     /* no LCD, unlike the STM32H573I-DK */

#endif /* BB_BOARD_FRDM_IMXRT1186_H */
