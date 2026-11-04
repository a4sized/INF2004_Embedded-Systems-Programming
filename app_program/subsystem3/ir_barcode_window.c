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
    if ((0u == p_window->count) ||
        ((now_us - p_window->start_us) < IR_BARCODE_WINDOW_US))
    {
        return false;
    }

    /* The configured limit applies to capturing bars. Once all 30 edges arrived
     * within it, allow a separately bounded white quiet zone to complete.
     * Manual scans can finish near the limit and still need ~10 narrow widths of
     * white before the decoder may accept the stop character.
     */
    if ((IR_BARCODE_COMPLETE_EDGES == p_window->count) &&
        !p_window->b_invalid &&
        ((p_window->edges[p_window->count - 1u].time_us -
          p_window->start_us) < IR_BARCODE_WINDOW_US) &&
        ((now_us - p_window->edges[p_window->count - 1u].time_us) <
         IR_BARCODE_QUIET_GRACE_US))
    {
        return false;
    }
    return true;
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
    else
    {
        ir_barcode_edge_t const previous =
            p_window->edges[p_window->count - 1u];
        uint32_t const elapsed_us = edge.time_us - previous.time_us;

        if ((edge.b_high == previous.b_high) || (0u == elapsed_us) ||
            ((edge.time_us - p_window->start_us) > IR_BARCODE_WINDOW_US))
        {
            p_window->b_invalid = true;
        }
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
