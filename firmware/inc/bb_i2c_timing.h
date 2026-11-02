/**
 * @file bb_i2c_timing.h
 * @brief The I2C peripheral's TIMINGR register, computed rather than guessed.
 */
#ifndef BB_I2C_TIMING_H
#define BB_I2C_TIMING_H

#include <stdbool.h>
#include <stdint.h>

/** Every I2C kernel clock on this board is HSI; see bb_board.c. */
#define BB_I2C_KERNEL_HZ   64000000UL

/**
 * @brief TIMINGR for a bus running at @p hz.
 *
 * A target uses the same value as a controller would for the fastest bus it
 * expects to sit on: it ignores SCLL and SCLH, which it does not drive, and
 * takes SDADEL and SCLDEL, which it does.
 */
uint32_t bb_i2c_timingr(uint32_t hz);

#endif /* BB_I2C_TIMING_H */
