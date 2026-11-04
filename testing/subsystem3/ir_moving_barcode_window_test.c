/* The moving USB test profile must retain slow 30-edge scans. */
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
            .time_us = 1000u + (index * 200000u),
            .b_high = (0u == (index & 1u)),
            .b_ambiguous = false
        };
        ir_barcode_window_feed(&window, edge);
    }
    assert(IR_BARCODE_COMPLETE_EDGES == window.count);
    assert(!window.b_invalid);
    assert(!ir_barcode_window_expired(&window,
                                    1000u + IR_BARCODE_WINDOW_US));
    assert(ir_barcode_window_expired(
        &window, window.edges[IR_BARCODE_COMPLETE_EDGES - 1u].time_us +
        IR_BARCODE_QUIET_GRACE_US));

    ir_barcode_window_reset(&window);
    ir_barcode_edge_t const first = {1000u, true, false};
    ir_barcode_window_feed(&window, first);
    assert(ir_barcode_window_expired(&window,
                                    1000u + IR_BARCODE_WINDOW_US));
    return 0;
}
