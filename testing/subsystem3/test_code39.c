/* Host check using measured mid-row runs from the supplied A and Z images.
 * Pixel widths stand in for proportional time at constant scan speed.
 */
#include "../../app_program/subsystem3/ir_code39.h"

#include <assert.h>
#include <stddef.h>

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

/* GP6 moving-car trace: complete Z at edge 30, then junction black after
 * 675 ms. The old 10-narrow-width quiet rule required about 850 ms.
 */
static uint32_t const moving_z_widths_ms[29] =
{
    30, 190, 30, 75, 145, 70, 141, 84, 20, 90,
    25, 195, 140, 80, 145, 85, 31, 79, 26, 89,
    25, 195, 25, 85, 140, 85, 140, 80, 30
};

/* Second moving-car Z: brief correction stretched one Z wide interval to
 * 275 ms; all 30 edges and both * markers were still present.
 */
static uint32_t const corrected_z_widths_ms[29] =
{
    10, 130, 11, 54, 85, 55, 85, 55, 11, 59,
    15, 275, 105, 55, 90, 55, 10, 60, 10, 60,
    10, 125, 10, 60, 85, 55, 85, 55, 11
};

/* Physical GP6 pass that decoded W in reverse order on 9 November. */
static uint32_t const moving_w_reverse_widths_ms[29] =
{
    52, 38, 160, 71, 159, 65, 30, 196, 24, 90,
    20, 90, 26, 80, 29, 85, 140, 200, 140, 80,
    30, 80, 145, 80, 145, 80, 25, 195, 31
};

static ir_barcode_window_t make_window (uint32_t const widths[29],
                                        bool b_reverse)
{
    ir_barcode_window_t window = {0};
    uint32_t time_us = 1000u;
    window.count = 30u;
    for (uint32_t i = 0u; i < 30u; ++i)
    {
        window.edges[i].time_us = time_us;
        window.edges[i].b_high = (0u == (i & 1u));
        if (i < 29u)
        {
            time_us += widths[b_reverse ? (28u - i) : i];
        }
    }
    return window;
}

int main (void)
{
    ir_barcode_window_t a = make_window(a_widths, false);
    ir_barcode_window_t z = make_window(z_widths, false);
    ir_barcode_window_t a_reverse = make_window(a_widths, true);
    ir_barcode_window_t z_reverse = make_window(z_widths, true);
    char letter = '\0';
    ir_code39_direction_t direction = IR_CODE39_FORWARD;

    a.count = 10u;
    z.count = 10u;
    a_reverse.count = 10u;
    z_reverse.count = 10u;
    assert(ir_code39_start_marker_seen(&a));
    assert(ir_code39_start_marker_seen(&z));
    assert(ir_code39_start_marker_seen(&a_reverse));
    assert(ir_code39_start_marker_seen(&z_reverse));
    a.count = 30u;
    z.count = 30u;
    a_reverse.count = 30u;
    z_reverse.count = 30u;
    assert(ir_code39_frame_single_with_direction(&a, &letter, &direction));
    assert('A' == letter);
    assert(ir_code39_frame_single_with_direction(&z_reverse,
                                                  &letter, &direction));
    assert('Z' == letter);

    ir_barcode_window_t moving_z = {0};
    moving_z.count = IR_BARCODE_COMPLETE_EDGES;
    uint32_t moving_time_us = 16685000u;
    for (uint32_t i = 0u; i < IR_BARCODE_COMPLETE_EDGES; ++i)
    {
        moving_z.edges[i].time_us = moving_time_us;
        moving_z.edges[i].b_high = (0u == (i & 1u));
        if (i < (IR_BARCODE_COMPLETE_EDGES - 1u))
        {
            moving_time_us += moving_z_widths_ms[i] * 1000u;
        }
    }
    moving_z.count = 10u;
    assert(ir_code39_start_marker_seen(&moving_z));
    moving_z.count = IR_BARCODE_COMPLETE_EDGES;
    assert(ir_code39_frame_single_with_direction(
        &moving_z, &letter, &direction));
    assert('Z' == letter);
    assert(IR_CODE39_FORWARD == direction);
    assert(!ir_code39_decode_single_with_direction(
        &moving_z, moving_z.edges[29].time_us + 675000u,
        &letter, &direction));

    ir_barcode_window_t corrected_z =
        make_window(corrected_z_widths_ms, false);
    assert(ir_code39_frame_single_with_direction(
        &corrected_z, &letter, &direction));
    assert('Z' == letter);
    assert(IR_CODE39_FORWARD == direction);
    uint32_t rejected_z_widths_ms[29];
    for (uint32_t i = 0u; i < 29u; ++i)
    {
        rejected_z_widths_ms[i] = corrected_z_widths_ms[i];
    }
    rejected_z_widths_ms[11] = 500u;
    ir_barcode_window_t rejected_z =
        make_window(rejected_z_widths_ms, false);
    assert(!ir_code39_frame_single_with_direction(
        &rejected_z, &letter, &direction));

    assert(ir_code39_decode_single_with_direction(
        &a, a.edges[29].time_us + 300u, &letter, &direction));
    assert('A' == letter);
    assert(IR_CODE39_FORWARD == direction);
    assert(IR_BARCODE_NO_ACTION == ir_barcode_action_for(letter));

    ir_barcode_window_t moving_w =
        make_window(moving_w_reverse_widths_ms, false);
    assert(ir_code39_frame_single_with_direction(
        &moving_w, &letter, &direction));
    assert('W' == letter);
    assert(IR_CODE39_REVERSE == direction);
    assert(IR_BARCODE_TURN_LEFT == ir_barcode_action_for(letter));
    assert(ir_code39_decode_single_with_direction(
        &z, z.edges[29].time_us + 300u, &letter, &direction));
    assert('Z' == letter);
    assert(IR_CODE39_FORWARD == direction);
    assert(IR_BARCODE_TURN_RIGHT == ir_barcode_action_for(letter));
    assert(IR_BARCODE_NO_ACTION == ir_barcode_action_for('B'));

    assert(ir_code39_decode_single_with_direction(
        &a_reverse, a_reverse.edges[29].time_us + 300u,
        &letter, &direction));
    assert('A' == letter);
    assert(IR_CODE39_REVERSE == direction);
    assert(ir_code39_decode_single_with_direction(
        &z_reverse, z_reverse.edges[29].time_us + 300u,
        &letter, &direction));
    assert('Z' == letter);
    assert(IR_CODE39_REVERSE == direction);

    /* A slow but continuously advancing *Z* takes over eight seconds.
     * Start and stop markers, rather than total travel time, frame it.
     */
    ir_barcode_window_t slow = {0};
    ir_barcode_window_reset(&slow);
    for (uint32_t i = 0u; i < IR_BARCODE_COMPLETE_EDGES; ++i)
    {
        ir_barcode_edge_t slow_edge = z.edges[i];
        slow_edge.time_us = 1000u +
            ((z.edges[i].time_us - 1000u) * 10000u);
        assert(!ir_barcode_window_expired(&slow, slow_edge.time_us));
        ir_barcode_window_feed(&slow, slow_edge);
        assert(!slow.b_invalid);
        if (10u == slow.count)
        {
            assert(ir_code39_start_marker_seen(&slow));
        }
    }
    assert((slow.edges[29].time_us - slow.edges[0].time_us) > 8000000u);
    assert(ir_code39_frame_single_with_direction(&slow, &letter,
                                                  &direction));
    assert('Z' == letter);
    assert(ir_code39_decode_single_with_direction(
        &slow, slow.edges[29].time_us + 3000000u,
        &letter, &direction));
    assert('Z' == letter);

    letter = '\0';
    assert(!ir_code39_decode_single(&a, a.edges[29].time_us + 200u, &letter));
    a.b_invalid = true;
    assert(!ir_code39_decode_single(&a, a.edges[29].time_us + 300u, &letter));
    a.b_invalid = false;
    a.count = 31u;
    assert(!ir_code39_decode_single(&a, a.edges[29].time_us + 300u, &letter));
    return 0;
}
