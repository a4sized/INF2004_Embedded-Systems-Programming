/* Host check: 5 ms GPIO samples must reach the real Code 39 edge decoder. */
#include "../../app_program/subsystem3/ir_barcode_gpio.h"
#include "../../app_program/subsystem3/ir_barcode_window.h"
#include "../../app_program/subsystem3/ir_code39.h"

#include <assert.h>

#define SAMPLE_US 5000u
#define PIXEL_US 500u

static uint32_t const a_widths[29] =
{
    28, 85, 28, 29, 85, 28, 85, 28, 29, 28,
    85, 28, 29, 28, 28, 85, 29, 28, 85, 28,
    29, 85, 28, 29, 85, 28, 85, 28, 29
};

static uint32_t const z_widths[29] =
{
    28, 85, 28, 29, 85, 28, 85, 28, 29, 28,
    28, 85, 85, 29, 85, 28, 28, 29, 28, 28,
    29, 85, 28, 29, 85, 28, 85, 28, 29
};

static void check_scan (uint32_t const widths[29], char expected,
                        bool b_reverse)
{
    uint32_t transitions[30] = {100000u};
    ir_barcode_window_t window = {0};
    uint32_t seen = 0u;

    for (uint32_t i = 1u; i < 30u; ++i)
    {
        transitions[i] = transitions[i - 1u] +
                         (widths[b_reverse ? (29u - i) : (i - 1u)] *
                          PIXEL_US);
    }

    ir_barcode_gpio_start();
    ir_barcode_window_reset(&window);
    for (uint32_t now_us = 0u; now_us < (transitions[29] + 200000u);
         now_us += SAMPLE_US)
    {
        while ((seen < 30u) && (now_us >= transitions[seen]))
        {
            ++seen;
        }
        ir_barcode_gpio_poll(0u != (seen & 1u), now_us);
        ir_barcode_edge_t edge = {0};
        while (ir_barcode_gpio_pop(&edge))
        {
            ir_barcode_window_feed(&window, edge);
        }
    }

    assert(30u == window.count);
    assert(0u == ir_barcode_gpio_overflows());
    char decoded = '\0';
    ir_code39_direction_t direction = IR_CODE39_FORWARD;
    assert(ir_code39_decode_single_with_direction(
        &window, transitions[29] + 200000u, &decoded, &direction));
    assert(expected == decoded);
    assert((b_reverse ? IR_CODE39_REVERSE : IR_CODE39_FORWARD) == direction);
}

int main (void)
{
    ir_barcode_edge_t edge = {0};
    ir_barcode_gpio_start();
    ir_barcode_gpio_poll(true, 0u);
    ir_barcode_gpio_poll(true, SAMPLE_US);
    assert(!ir_barcode_gpio_pop(&edge));
    ir_barcode_gpio_poll(false, SAMPLE_US * 2u);
    assert(ir_barcode_gpio_pop(&edge));
    assert(!edge.b_high);
    assert((SAMPLE_US * 2u) == edge.time_us);

    check_scan(a_widths, 'A', false);
    check_scan(z_widths, 'Z', false);
    check_scan(a_widths, 'A', true);
    check_scan(z_widths, 'Z', true);
    return 0;
}
