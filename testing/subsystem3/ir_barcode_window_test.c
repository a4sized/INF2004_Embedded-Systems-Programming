#include "ir_barcode_window.h"

#include <assert.h>
#include <stdint.h>

static ir_barcode_edge_t edge (uint32_t time_us, bool b_high,
                               bool b_ambiguous)
{
    ir_barcode_edge_t const value = {time_us, b_high, b_ambiguous};
    return value;
}

int main (void)
{
    ir_barcode_window_t window = {0};

    ir_barcode_window_reset(&window);
    ir_barcode_window_feed(&window, edge(100u, false, false));
    assert(0u == window.count);
    ir_barcode_window_feed(&window, edge(200u, true, false));
    ir_barcode_window_feed(&window, edge(260u, false, false));
    ir_barcode_window_feed(&window, edge(300u, true, false));
    assert(3u == window.count);
    assert(!window.b_invalid);
    assert(60u == (window.edges[1].time_us - window.edges[0].time_us));
    assert(!ir_barcode_window_expired(&window, 200u + IR_BARCODE_WINDOW_US - 1u));
    assert(ir_barcode_window_expired(&window, 200u + IR_BARCODE_WINDOW_US));

    ir_barcode_window_reset(&window);
    ir_barcode_window_feed(&window, edge(UINT32_MAX - 100u, true, false));
    ir_barcode_window_feed(&window, edge(50u, false, false));
    assert(151u ==
           (window.edges[1].time_us - window.edges[0].time_us));
    assert(!ir_barcode_window_expired(&window, 20u));
    assert(ir_barcode_window_expired(&window,
                                     (UINT32_MAX - 100u) +
                                     IR_BARCODE_WINDOW_US));

    ir_barcode_window_reset(&window);
    ir_barcode_window_feed(&window, edge(10u, true, false));
    ir_barcode_window_feed(&window, edge(20u, true, false));
    assert(window.b_invalid);
    ir_barcode_window_reset(&window);
    ir_barcode_window_feed(&window, edge(10u, true, true));
    assert(window.b_invalid);
    ir_barcode_window_reset(&window);
    ir_barcode_window_invalidate(&window);
    ir_barcode_window_feed(&window, edge(10u, true, false));
    assert(window.b_invalid);

    ir_barcode_window_reset(&window);
    for (uint32_t index = 0u; index <= IR_BARCODE_EDGE_CAPACITY; ++index)
    {
        ir_barcode_window_feed(&window,
                               edge(index * 10u, (0u == (index & 1u)), false));
    }
    assert(IR_BARCODE_EDGE_CAPACITY == window.count);
    assert(window.b_invalid);
    return 0;
}
