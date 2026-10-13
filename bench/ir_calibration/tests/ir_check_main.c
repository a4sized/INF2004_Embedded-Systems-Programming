/** @file ir_check_main.c
 * @brief Synthetic checks; these do not establish sensor performance.
 */
#include "ir_check_main.h"
#include "ir_calib.h"

#include <assert.h>

static void fill_window (ir_calib_window_t * p_window, uint32_t high_count);

int main (void)
{
    ir_calib_window_t black  = {0};
    ir_calib_window_t white  = {0};
    ir_calib_result_t result = {0};

    /* Test every accepted majority, in both electrical polarities. */
    for (uint32_t high_count = IR_CALIB_MIN_AGREEMENT;
         high_count <= IR_CALIB_SAMPLES; ++high_count)
    {
        fill_window(&black, high_count);
        fill_window(&white, IR_CALIB_SAMPLES - high_count);
        assert(IR_CALIB_OK == ir_calib_evaluate(&black, &white, &result));
        assert((result.b_valid) && (result.b_black_high));

        assert(IR_CALIB_OK == ir_calib_evaluate(&white, &black, &result));
        assert((result.b_valid) && (!result.b_black_high));
    }

    /* Reject every noise count between the two acceptance boundaries. */
    for (uint32_t high_count =
             (IR_CALIB_SAMPLES - IR_CALIB_MIN_AGREEMENT) + 1u;
         high_count < IR_CALIB_MIN_AGREEMENT;
         ++high_count)
    {
        fill_window(&black, high_count);
        fill_window(&white, 0u);
        assert(IR_CALIB_UNSTABLE ==
               ir_calib_evaluate(&black, &white, &result));
        assert(!result.b_valid);
        assert(IR_CALIB_UNSTABLE ==
               ir_calib_evaluate(&white, &black, &result));
    }

    fill_window(&black, IR_CALIB_SAMPLES);
    fill_window(&white, IR_CALIB_SAMPLES);
    assert(IR_CALIB_NO_CONTRAST ==
           ir_calib_evaluate(&black, &white, &result));

    fill_window(&black, 0u);
    fill_window(&white, 0u);
    assert(IR_CALIB_NO_CONTRAST ==
           ir_calib_evaluate(&black, &white, &result));

    /* A failed recalibration must invalidate a previously valid result. */
    fill_window(&black, IR_CALIB_SAMPLES);
    assert(IR_CALIB_OK == ir_calib_evaluate(&black, &white, &result));
    ir_calib_reset(&white);
    assert(IR_CALIB_INCOMPLETE ==
           ir_calib_evaluate(&black, &white, &result));
    assert(!result.b_valid);
    assert(!result.b_black_high);

    /* Transition counting and saturation must remain bounded. */
    ir_calib_reset(&black);

    for (uint32_t sample = 0u; sample < IR_CALIB_SAMPLES; ++sample)
    {
        assert(ir_calib_add(&black, (0u == (sample % 2u))));
    }

    assert(IR_CALIB_SAMPLES == black.count);
    assert((IR_CALIB_SAMPLES - 1u) == black.transitions);
    assert((IR_CALIB_SAMPLES / 2u) == black.high_count);
    assert(!ir_calib_add(&black, true));
    assert(IR_CALIB_SAMPLES == black.count);
    assert((IR_CALIB_SAMPLES - 1u) == black.transitions);

    return 0;
}

/** @brief Create exactly 100 samples with a chosen high-level count. */
static void fill_window (ir_calib_window_t * p_window, uint32_t high_count)
{
    ir_calib_reset(p_window);

    for (uint32_t sample = 0u; sample < IR_CALIB_SAMPLES; ++sample)
    {
        assert(ir_calib_add(p_window, (sample < high_count)));
    }
}

/* End of file. */

