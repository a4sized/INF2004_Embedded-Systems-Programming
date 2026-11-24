/** Separate-colour classification: measured small prints and alphabet. */
#include "ir_code39.h"
#include <assert.h>
#include <string.h>

static uint16_t const letters[26] =
{
    0x109u,0x049u,0x148u,0x019u,0x118u,0x058u,0x00du,
    0x10cu,0x04cu,0x01cu,0x103u,0x043u,0x142u,0x013u,
    0x112u,0x052u,0x007u,0x106u,0x046u,0x016u,0x181u,
    0x0c1u,0x1c0u,0x091u,0x190u,0x0d0u
};

/* Exact adjacent GP6 durations (ms), stationary manual small-print passes,
 * attachment 1c7e74c4-881f-4710-bbd5-efaac799b3a7, 18 November 2026.
 * Firmware formerly reset at every ten edges; regrouped here as 30 edges.
 */
static uint32_t const small_a_reverse[29] =
{
    25,70,85,66,84,66,19,110,15,50,
    65,55,16,129,15,55,15,50,75,55,
    15,65,85,65,95,75,16,129,25
};
static uint32_t const small_z_forward[29] =
{
    15,130,20,70,90,70,95,60,20,60,
    20,100,80,61,89,60,16,64,20,60,
    15,120,15,65,90,65,105,80,20
};

static ir_barcode_window_t window_for (uint32_t const widths[29],
                                       bool reverse, uint32_t scale)
{
    ir_barcode_window_t window = {0};
    uint32_t time = 1000u;
    window.count = 30u;
    for (uint32_t i = 0u; i < 30u; ++i)
    {
        window.edges[i].time_us = time;
        window.edges[i].b_high = (0u == (i & 1u));
        if (i < 29u) time += widths[reverse ? 28u - i : i] * scale;
    }
    return window;
}

static void expect_letter (ir_barcode_window_t window, char expected,
                            ir_code39_direction_t order)
{
    window.count = 10u;
    assert(ir_code39_start_marker_seen(&window));
    window.count = 30u;
    char decoded = '\0';
    ir_code39_direction_t direction = IR_CODE39_FORWARD;
    assert(ir_code39_frame_single_with_direction(&window, &decoded, &direction));
    assert(expected == decoded);
    assert(order == direction);
    assert(0 == strcmp("VALID", ir_code39_frame_reason(&window,
                           IR_CODE39_REVERSE == order)));
}

static void generate_widths (uint32_t widths[29], uint16_t data,
                              bool white_larger)
{
    uint16_t const patterns[3] = {0x094u, data, 0x094u};
    for (uint32_t c = 0u; c < 3u; ++c)
    {
        for (uint32_t i = 0u; i < 9u; ++i)
        {
            bool const wide = (0u != (patterns[c] & (1u << (8u - i))));
            bool const bar = (0u == (i & 1u));
            /* Overlap colours in either direction while preserving the
             * distinction within each. A pooled 1.4 ratio rejects these.
             */
            widths[c * 10u + i] = (bar == white_larger) ?
                (wide ? 80u : 20u) : (wide ? 140u : 70u);
        }
        if (c < 2u) widths[c * 10u + 9u] = white_larger ? 70u : 20u;
    }
}

int main (void)
{
    expect_letter(window_for(small_a_reverse, false, 1000u), 'A', IR_CODE39_REVERSE);
    expect_letter(window_for(small_z_forward, false, 1000u), 'Z', IR_CODE39_FORWARD);
    /* Reverse both physical captures, too: no print-specific direction. */
    expect_letter(window_for(small_a_reverse, true, 1000u), 'A', IR_CODE39_FORWARD);
    expect_letter(window_for(small_z_forward, true, 1000u), 'Z', IR_CODE39_REVERSE);

    uint32_t widths[29] = {0};
    for (uint32_t letter = 0u; letter < 26u; ++letter)
    {
        for (uint32_t polarity = 0u; polarity < 2u; ++polarity)
        {
            generate_widths(widths, letters[letter], 0u != polarity);
            for (uint32_t direction = 0u; direction < 2u; ++direction)
            {
                for (uint32_t scale = 1u; scale <= 1000u; scale *= 10u)
                {
                    expect_letter(window_for(widths, direction != 0u, scale),
                        (char)('A' + (char)letter),
                        direction ? IR_CODE39_REVERSE : IR_CODE39_FORWARD);
                }
            }
        }
    }

    char decoded = '\0';
    ir_code39_direction_t direction = IR_CODE39_FORWARD;
    generate_widths(widths, letters[25], true);
    widths[1] = 97u; /* <1.4 separation from narrow white 70. */
    ir_barcode_window_t bad = window_for(widths, false, 1000u);
    assert(!ir_code39_start_marker_seen(&(ir_barcode_window_t){0}));
    assert(!ir_code39_frame_single_with_direction(&bad, &decoded, &direction));
    generate_widths(widths, letters[25], true);
    widths[1] = 500u; /* Extreme wide white, no unbounded outlier. */
    bad = window_for(widths, false, 1000u);
    assert(!ir_code39_frame_single_with_direction(&bad, &decoded, &direction));
    generate_widths(widths, letters[25], true);
    widths[4] = 401u; /* Wide black >4x its 80-wide peer. */
    bad = window_for(widths, false, 1000u);
    assert(!ir_code39_frame_single_with_direction(&bad, &decoded, &direction));
    generate_widths(widths, letters[25], true);
    widths[9] = 500u; /* Wrong intercharacter white gap. */
    bad = window_for(widths, false, 1000u);
    assert(!ir_code39_frame_single_with_direction(&bad, &decoded, &direction));
    generate_widths(widths, 0x034u, true); /* Digit, outside A-Z contract. */
    bad = window_for(widths, false, 1000u);
    assert(!ir_code39_frame_single_with_direction(&bad, &decoded, &direction));
    generate_widths(widths, 0x0a8u, true); /* Three wide spaces, not a letter. */
    bad = window_for(widths, false, 1000u);
    assert(!ir_code39_frame_single_with_direction(&bad, &decoded, &direction));
    generate_widths(widths, letters[25], true);
    for (uint32_t i = 0u; i < 9u; i += 2u) widths[i] = 20u;
    bad = window_for(widths, false, 1000u); /* No distinguishable wide bars. */
    assert(!ir_code39_frame_single_with_direction(&bad, &decoded, &direction));
    generate_widths(widths, letters[25], true);
    uint32_t other[29] = {0};
    generate_widths(other, letters[0], true);
    for (uint32_t i = 0u; i < 9u; ++i) widths[20u + i] = other[10u + i];
    bad = window_for(widths, false, 1000u); /* Letter A instead of closing *. */
    assert(!ir_code39_frame_single_with_direction(&bad, &decoded, &direction));
    assert(0 == strcmp("STOP_PATTERN", ir_code39_frame_reason(&bad, false)));

    bad = window_for(small_z_forward, false, 1000u);
    bad.count = 28u;
    assert(!ir_code39_frame_single_with_direction(&bad, &decoded, &direction));
    bad.count = 30u;
    bad.edges[29].time_us = bad.edges[28].time_us;
    assert(!ir_code39_frame_single_with_direction(&bad, &decoded, &direction));
    return 0;
}
