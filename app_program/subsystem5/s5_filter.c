/** @file s5_filter.c @brief Median of a few ultrasonic pings. */
#include "s5_config.h"
#include "s5_filter.h"

uint16_t s5_filter_median (const uint16_t * const p_values, const uint8_t count)
{
    uint16_t sorted[S5_FILTER_MAX];
    uint8_t  used = (count > S5_FILTER_MAX) ? (uint8_t)S5_FILTER_MAX : count;

    if (0u == used)
    {
        return NO_OBSTACLE;   /* nothing to take a median of */
    }

    for (uint8_t i = 0u; i < used; i++)
    {
        uint8_t j = i;

        /* Insertion sort: put p_values[i] in its place among the earlier
         * ones. */
        while ((j > 0u) && (sorted[j - 1u] > p_values[i]))
        {
            sorted[j] = sorted[j - 1u];
            j--;
        }
        sorted[j] = p_values[i];
    }

    /* Even count: take the NEARER of the two middle values, so with one good
     * echo and one missing one the obstacle is kept, not a fake edge made. */
    return sorted[(used - 1u) / 2u];
}
