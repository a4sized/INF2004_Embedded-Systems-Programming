/** @file ir_barcode_segments.c @brief Normalize captured edge intervals. */
#include "ir_barcode_segments.h"

bool ir_barcode_segments_make (ir_barcode_window_t const * p_window,
                                ir_barcode_segments_t * p_segments)
{
    p_segments->count = 0u;
    p_segments->total_us = 0u;

    if (p_window->b_invalid || (p_window->count < 4u) ||
        (p_window->count > IR_BARCODE_EDGE_CAPACITY) ||
        (0u != (p_window->count & 1u)) ||
        !p_window->edges[0].b_high ||
        p_window->edges[p_window->count - 1u].b_high)
    {
        return false;
    }

    uint32_t const total =
        p_window->edges[p_window->count - 1u].time_us -
        p_window->edges[0].time_us;

    if ((0u == total) || (total > IR_BARCODE_WINDOW_US))
    {
        return false;
    }

    for (uint32_t index = 0u; index + 1u < p_window->count; ++index)
    {
        ir_barcode_edge_t const current = p_window->edges[index];
        ir_barcode_edge_t const next = p_window->edges[index + 1u];
        uint32_t const duration = next.time_us - current.time_us;

        if ((0u == duration) || (duration > total) ||
            current.b_ambiguous || next.b_ambiguous ||
            (current.b_high == next.b_high))
        {
            return false;
        }

        uint64_t const scaled =
            ((uint64_t)duration * IR_BARCODE_NORMAL_SCALE) +
            ((uint64_t)total / 2u);
        p_segments->fractions[index] = (uint32_t)(scaled / total);
    }

    p_segments->count = p_window->count - 1u;
    p_segments->total_us = total;
    return true;
}
