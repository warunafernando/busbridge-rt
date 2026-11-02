#ifndef BB_VERSION_H
#define BB_VERSION_H

#include "bb_board.h"

#define BB_PRODUCT  "BUSBRIDGE"
#define BB_VERSION  "0.1.0"

/* The board names itself, so *IDN? cannot claim to be hardware it is not.
 * This was hardcoded to the NUCLEO, which meant the first Discovery kit build
 * booted correctly and then introduced itself as the wrong board. */
#define BB_BOARD    BB_BOARD_NAME

#endif /* BB_VERSION_H */
