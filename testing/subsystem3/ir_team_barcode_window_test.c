/* A progressing, very slow scan must not hit a total-duration limit. */
#include "../../app_program/subsystem3/ir_barcode_window.h"

#include <assert.h>

int main (void)
{
    ir_barcode_window_t window = {0};
    ir_barcode_window_reset(&window);

    for (uint32_t index = 0u; index < IR_BARCODE_COMPLETE_EDGES; ++index)
    {
        ir_barcode_edge_t const edge =
        {
            .time_us = 1000u + (index * 500000u),
            .b_high = (0u == (index & 1u)),
            .b_ambiguous = false
        };
        ir_barcode_window_feed(&window, edge);
    }

    assert(IR_BARCODE_COMPLETE_EDGES == window.count);
    assert(!window.b_invalid);
    assert(8000000u < (window.edges[29].time_us - window.start_us));
    assert(!ir_barcode_window_expired(&window,
                                     window.edges[29].time_us +
                                     IR_BARCODE_EDGE_IDLE_US));
    assert(ir_barcode_window_expired(
        &window, window.edges[29].time_us +
        IR_BARCODE_FINAL_QUIET_MAX_US));
    return 0;
}
