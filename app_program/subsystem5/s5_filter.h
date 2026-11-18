/** @file s5_filter.h @brief Median of a few ultrasonic pings. */
#ifndef S5_FILTER_H
#define S5_FILTER_H

#include <stdint.h>

/* The most readings s5_filter_median() will look at. */
#define S5_FILTER_MAX       (5u)

/*
 * Median of count readings in millimetres.  A "no obstacle" reading is the
 * largest value (0xFFFF), so it sorts last.  With an even count the NEARER
 * of the two middle values is taken, so the median is "no obstacle" only
 * when more than half of the pings saw nothing, and with 2 readings the
 * nearer one wins (the safe side: no fake edge).  With 3 pings one bad
 * ping, whether a false near spike or a missing echo, never decides it.
 * Returns 0xFFFF (no obstacle) if count is 0.  count is limited to
 * S5_FILTER_MAX.
 */
uint16_t s5_filter_median (const uint16_t * const p_values,
                           const uint8_t count);

#endif
