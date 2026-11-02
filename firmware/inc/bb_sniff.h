/**
 * @file bb_sniff.h
 * @brief The sniffer's command surface, in front of the CM7 capture engine.
 *
 * Rewritten rather than ported. The STM32 decoded in a GPIO edge interrupt,
 * which is where all three of its limits came from - ~450 ns per bus clock, a
 * ~1 MHz ceiling, and native I3C bits read one position late because the
 * handler sampled SDA after SCL had already fallen. Capture is now hardware,
 * on the other core, clocked by the bus's own clock line.
 */
#ifndef BB_SNIFF_H
#define BB_SNIFF_H

#include "bb_status.h"

void bb_sniff_init(void);
void bb_sniff_status_line(bb_status_t *out);

#endif /* BB_SNIFF_H */
