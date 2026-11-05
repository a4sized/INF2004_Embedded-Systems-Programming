/**
 * @file ir_line_filter_test.c
 * @brief Host checks for transient rejection and independent line channels.
 */
#include "ir_line_filter.h"

#include <assert.h>
#include <stdint.h>

int main (void)
{
    ir_line_filter_t filter = {0};
    drv_ir_levels_t raw = {0};
    drv_ir_levels_t stable = {0};

    ir_line_filter_reset(&filter);
    stable = ir_line_filter_update(&filter, raw, 0u, 10u);
    assert(!stable.b_left_high);
    assert(!stable.b_right_high);

    /* A short left pulse must not become a steering input. */
    raw.b_left_high = true;
    stable = ir_line_filter_update(&filter, raw, 5u, 10u);
    assert(!stable.b_left_high);
    raw.b_left_high = false;
    stable = ir_line_filter_update(&filter, raw, 9u, 10u);
    assert(!stable.b_left_high);

    /* A sustained left change is accepted while right remains separate. */
    raw.b_left_high = true;
    raw.b_barcode_high = true;
    stable = ir_line_filter_update(&filter, raw, 20u, 10u);
    assert(!stable.b_left_high);
    assert(stable.b_barcode_high);
    stable = ir_line_filter_update(&filter, raw, 30u, 10u);
    assert(stable.b_left_high);
    assert(!stable.b_right_high);

    /* Right can change later without resetting left. */
    raw.b_right_high = true;
    raw.b_barcode_high = false;
    stable = ir_line_filter_update(&filter, raw, 35u, 10u);
    assert(!stable.b_right_high);
    assert(!stable.b_barcode_high);
    stable = ir_line_filter_update(&filter, raw, 45u, 10u);
    assert(stable.b_right_high);
    assert(stable.b_left_high);

    /* A 32-bit clock wrap does not prevent a sustained change. */
    ir_line_filter_reset(&filter);
    raw.b_left_high = false;
    stable = ir_line_filter_update(&filter, raw, UINT32_MAX - 5u, 10u);
    raw.b_left_high = true;
    stable = ir_line_filter_update(&filter, raw, UINT32_MAX - 2u, 10u);
    assert(!stable.b_left_high);
    stable = ir_line_filter_update(&filter, raw, 8u, 10u);
    assert(stable.b_left_high);

    return 0;
}

/* End of file. */
