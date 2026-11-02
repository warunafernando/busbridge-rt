/*
 * The published pin map for the frdm_imxrt1186.
 *
 * Included by bb_board.c through BB_BOARD_PINMAP. This table is what SYS:PINS?
 * returns, and it is the only place the GUI, both host libraries and anyone
 * deciding where a wire goes are supposed to read. Kept beside board.h because
 * it is board data - on the STM32 tree a new board once silently inherited
 * another's map, built cleanly, ran, and answered every command with wiring
 * advice for the wrong hardware.
 *
 * Columns: signal, MCU pad, silkscreen label, connector and pin.
 *
 * The connector column comes from UM12450 Rev 3.0 tables 20-23, not from the
 * label. On this board the Arduino numbering is at least contiguous per header,
 * but two things still make inference unsafe:
 *
 *   - J52 carries no signals at all, only power, so "the fourth header" is not
 *     a place a wire can go.
 *   - most pads reach the header through a 3-pin selection jumper, so a pad
 *     being listed here does not mean it is connected there right now. Where a
 *     jumper decides, the label says so.
 */
static const bb_pinmap_t pin_map[] = {

    /* Console - the MCU-Link VCOM, so this needs no wire at all. Jumpers J41
     * and J43 in position 1-2 (the default) put these pads on LPUART1; move
     * them to 2-3 and the pads become Arduino D0/D1 and the console is gone. */
    { "console.tx",  "GPIO_AON_08", "D1-or-vcom",  "J1-4"  },
    { "console.rx",  "GPIO_AON_09", "D0-or-vcom",  "J1-2"  },

    /* I2C bus 1 - the same two pads as the console. Off unless the firmware
     * was built with BB_I2C1_ENABLE and the jumpers moved. Listed so the cost
     * is visible rather than discovered. */
    { "i2c1.scl",    "GPIO_AON_09", "D0-or-vcom",  "J1-2"  },
    { "i2c1.sda",    "GPIO_AON_08", "D1-or-vcom",  "J1-4"  },

    /* I2C bus 2 - motor connector, not the Arduino headers. Shares its pads
     * with I3C port 1; whichever was configured last owns them. */
    { "i2c2.scl",    "GPIO_AON_16", "J3-or-i3c1",  "J3-12" },
    { "i2c2.sda",    "GPIO_AON_15", "J3-or-i3c1",  "J3-10" },

    /* I2C bus 3 - the Arduino I2C position.
     *
     * NOTE the labels. UM12450 table 22 calls J2.18 "D14/I2C_SDA" and J2.20
     * "D15/I2C_SCL", but the IOMUX puts LPI2C3_SCL on GPIO_EMC_B2_19 (D14) and
     * LPI2C3_SDA on GPIO_EMC_B2_20 (D15) - crossed relative to the silkscreen.
     * This map follows the silicon. If the bench disagrees, the silkscreen was
     * right and this is the line to change. */
    { "i2c3.scl",    "GPIO_EMC_B2_19", "D14",      "J2-18" },
    { "i2c3.sda",    "GPIO_EMC_B2_20", "D15",      "J2-20" },

    /* I3C port 1 - shares pads with I2C bus 2. */
    { "i3c1.scl",    "GPIO_AON_16", "J3-or-i2c2",  "J3-12" },
    { "i3c1.sda",    "GPIO_AON_15", "J3-or-i2c2",  "J3-10" },

    /* I3C port 2 - the only pads I3C2 can use on this package. Free of any
     * sharing, because I2C bus 3 was put on D14/D15 rather than here. */
    { "i3c2.scl",    "GPIO_AD_18",  "A5",          "J51-12" },
    { "i3c2.sda",    "GPIO_AD_19",  "A4",          "J51-10" },

    /* SPI - the standard Arduino position. CS is a plain GPIO, driven in
     * software so it can be held across several commands. */
    { "spi.sck",     "GPIO_EMC_B1_22", "D13",      "J2-12" },
    { "spi.mosi",    "GPIO_EMC_B1_24", "D11",      "J2-8"  },
    { "spi.miso",    "GPIO_EMC_B1_23", "D12",      "J2-10" },
    { "spi.cs",      "GPIO_EMC_B1_25", "D10",      "J2-6"  },

    /* Target UART - separate from the console, so driving a target's serial
     * port does not cost the transport the commands arrive on. */
    { "uart.tx",     "GPIO_EMC_B1_12", "D2",       "J1-6"  },
    { "uart.rx",     "GPIO_EMC_B1_13", "D3",       "J1-8"  },

    /* MDIO - bit-banged. The hardware MDIO in ENETC reaches none of these
     * headers; it is wired to the board's own PHYs and can talk to nothing
     * else. */
    { "mdio.mdc",    "GPIO_EMC_B1_17", "D8",       "J2-2"  },
    { "mdio.mdio",   "GPIO_EMC_B1_16", "D9",       "J2-4"  },

    /* CAN - to the on-board transceiver U13 via jumpers J44/J46 in position
     * 1-2. The bus connector J22 is DNP: the transceiver is fitted and wired,
     * but there is no header to plug a bus into until one is soldered on. */
    { "can.tx",      "GPIO_AON_06", "u13-or-J51-9",  "J22-1" },
    { "can.rx",      "GPIO_AON_07", "u13-or-J51-11", "J22-3" },

    /* Sniffer taps. Every one of these is a FlexIO pad at ALT 8, which is what
     * allows a hardware-clocked capture rather than an edge interrupt. Which
     * ones a channel uses is chosen at runtime by SNIFF:CH - no fixed
     * assignment fits every combination of buses. */
    { "sniff.f1.14", "GPIO_EMC_B1_14", "D4",       "J1-10" },
    { "sniff.f1.15", "GPIO_EMC_B1_15", "D5",       "J1-12" },
    { "sniff.f1.17", "GPIO_EMC_B1_17", "D8",       "J2-2"  },
    { "sniff.f1.16", "GPIO_EMC_B1_16", "D9",       "J2-4"  },
    { "sniff.f2.15", "GPIO_AD_15",     "A0",       "J51-2" },
    { "sniff.f2.16", "GPIO_AD_16",     "A1",       "J51-4" },
    { "sniff.f2.17", "GPIO_AD_17",     "A2",       "J51-6" },

    /* Board furniture, so a host can tell what is spoken for. */
    { "led.green",   "GPIO_EMC_B1_11", "D4-rgb",   "-"     },
    { "led.blue",    "GPIO_EMC_B1_39", "D4-rgb",   "-"     },
    { "led.red",     "GPIO_EMC_B1_09", "D4-rgb",   "-"     },
    { "button",      "GPIO_AD_12",     "SW4-or-D6","J1-14" },
};
