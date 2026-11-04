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

    assert(ir_code39_decode_single_with_direction(
        &a, a.edges[29].time_us + 300u, &letter, &direction));
    assert('A' == letter);
    assert(IR_CODE39_FORWARD == direction);
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

    letter = '\0';
    assert(!ir_code39_decode_single(&a, a.edges[29].time_us + 200u, &letter));
    a.b_invalid = true;
    assert(!ir_code39_decode_single(&a, a.edges[29].time_us + 300u, &letter));
    a.b_invalid = false;
    a.count = 31u;
    assert(!ir_code39_decode_single(&a, a.edges[29].time_us + 300u, &letter));
    return 0;
}
