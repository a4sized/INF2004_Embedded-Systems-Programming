#include "ir_barcode_segments.h"

#include <assert.h>
#include <stdint.h>

static void add (ir_barcode_window_t * p_window, uint32_t time_us,
                 bool b_high)
{
    ir_barcode_edge_t const edge = {time_us, b_high, false};
    ir_barcode_window_feed(p_window, edge);
}

static void check_scaled (uint32_t start_us, uint32_t scale)
{
    ir_barcode_window_t window = {0};
    ir_barcode_segments_t segments = {0};

    ir_barcode_window_reset(&window);
    add(&window, start_us, true);
    add(&window, start_us + (200u * scale), false);
    add(&window, start_us + (500u * scale), true);
    add(&window, start_us + (1000u * scale), false);
    assert(ir_barcode_segments_make(&window, &segments));
    assert(3u == segments.count);
    assert(1000u * scale == segments.total_us);
    assert(200000u == segments.fractions[0]);
    assert(300000u == segments.fractions[1]);
    assert(500000u == segments.fractions[2]);
}

int main (void)
{
    ir_barcode_window_t window = {0};
    ir_barcode_segments_t segments = {0};

    check_scaled(100u, 1u);
    check_scaled(100u, 10u);
    check_scaled(UINT32_MAX - 300u, 1u);

    ir_barcode_window_reset(&window);
    add(&window, 100u, true);
    add(&window, 200u, false);
    add(&window, 300u, true);
    assert(!ir_barcode_segments_make(&window, &segments));

    add(&window, 400u, false);
    ir_barcode_window_invalidate(&window);
    assert(!ir_barcode_segments_make(&window, &segments));

    ir_barcode_window_reset(&window);
    add(&window, 100u, true);
    add(&window, 100u, false);
    add(&window, 200u, true);
    add(&window, 300u, false);
    assert(!ir_barcode_segments_make(&window, &segments));

    ir_barcode_window_reset(&window);
    add(&window, 100u, true);
    add(&window, 2100000u, false);
    add(&window, 2200000u, true);
    add(&window, 2300000u, false);
    assert(!ir_barcode_segments_make(&window, &segments));
    return 0;
}
