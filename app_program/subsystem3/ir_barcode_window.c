/** @file ir_barcode_window.c @brief Task-side edge collection without decoding. */
#include "ir_barcode_window.h"

void ir_barcode_window_reset (ir_barcode_window_t * p_window)
{
    p_window->count = 0u;
    p_window->start_us = 0u;
    p_window->b_invalid = false;
}

void ir_barcode_window_invalidate (ir_barcode_window_t * p_window)
{
    p_window->b_invalid = true;
}

bool ir_barcode_window_expired (ir_barcode_window_t const * p_window,
                                uint32_t now_us)
{
    return (p_window->count > 0u) &&
           ((now_us - p_window->start_us) >= IR_BARCODE_WINDOW_US);
}

void ir_barcode_window_feed (ir_barcode_window_t * p_window,
                             ir_barcode_edge_t edge)
{
    if (0u == p_window->count)
    {
        if (!edge.b_high)
        {
            return;
        }
        p_window->start_us = edge.time_us;
    }
    else if (edge.b_high == p_window->edges[p_window->count - 1u].b_high)
    {
        p_window->b_invalid = true;
    }

    if (edge.b_ambiguous)
    {
        p_window->b_invalid = true;
    }

    if (p_window->count < IR_BARCODE_EDGE_CAPACITY)
    {
        p_window->edges[p_window->count] = edge;
        ++p_window->count;
    }
    else
    {
        p_window->b_invalid = true;
    }
}
