/**
 * @file ir_calib.c
 * @brief Bounded digital calibration without hardware or RTOS dependencies.
 * @details See PROJECT_CONTEXT.md section 10 for polarity requirements.
 */
#include "ir_calib.h"

static bool window_is_stable (ir_calib_window_t const * p_window);

void ir_calib_reset (ir_calib_window_t * p_window)
{
    p_window->count           = 0u;
    p_window->high_count      = 0u;
    p_window->transitions     = 0u;
    p_window->b_previous_high = false;
}

bool ir_calib_add (ir_calib_window_t * p_window, bool b_high)
{
    bool b_added = false;

    if (p_window->count < IR_CALIB_SAMPLES)
    {
        if ((p_window->count > 0u) &&
            (b_high != p_window->b_previous_high))
        {
            ++p_window->transitions;
        }

        if (b_high)
        {
            ++p_window->high_count;
        }

        p_window->b_previous_high = b_high;
        ++p_window->count;
        b_added = true;
    }

    return b_added;
}

ir_calib_status_t ir_calib_evaluate (
    ir_calib_window_t const * p_black,
    ir_calib_window_t const * p_white,
    ir_calib_result_t * p_result)
{
    ir_calib_status_t status = IR_CALIB_INCOMPLETE;

    p_result->b_valid      = false;
    p_result->b_black_high = false;

    if ((IR_CALIB_SAMPLES != p_black->count) ||
        (IR_CALIB_SAMPLES != p_white->count))
    {
        return status;
    }

    bool const b_black_stable = window_is_stable(p_black);
    bool const b_white_stable = window_is_stable(p_white);

    if ((!b_black_stable) || (!b_white_stable))
    {
        return IR_CALIB_UNSTABLE;
    }

    bool const b_black_high =
        (p_black->high_count >= IR_CALIB_MIN_AGREEMENT);
    bool const b_white_high =
        (p_white->high_count >= IR_CALIB_MIN_AGREEMENT);

    status = IR_CALIB_NO_CONTRAST;

    if (b_black_high != b_white_high)
    {
        p_result->b_black_high = b_black_high;
        p_result->b_valid      = true;
        status                = IR_CALIB_OK;
    }

    return status;
}

/** @brief Require at least 95 of 100 samples at one electrical level. */
static bool window_is_stable (ir_calib_window_t const * p_window)
{
    bool b_stable = false;

    if (p_window->high_count <= p_window->count)
    {
        uint32_t const low_count = p_window->count - p_window->high_count;

        b_stable = (p_window->high_count >= IR_CALIB_MIN_AGREEMENT) ||
                   (low_count >= IR_CALIB_MIN_AGREEMENT);
    }

    return b_stable;
}

/* End of file. */

