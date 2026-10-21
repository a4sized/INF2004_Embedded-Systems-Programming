/**
 * @file ir_line_filter.c
 * @brief Time-qualified digital levels for the two line sensors.
 */
#include "ir_line_filter.h"

static bool update_channel (
    ir_line_channel_t * p_channel,
    bool b_raw_high,
    uint32_t now_ms,
    uint32_t stable_ms);

void ir_line_filter_reset (ir_line_filter_t * p_filter)
{
    *p_filter = (ir_line_filter_t){0};
}

drv_ir_levels_t ir_line_filter_update (
    ir_line_filter_t * p_filter,
    drv_ir_levels_t raw,
    uint32_t now_ms,
    uint32_t stable_ms)
{
    drv_ir_levels_t stable = raw;

    stable.b_left_high = update_channel(
        &p_filter->left, raw.b_left_high, now_ms, stable_ms);
    stable.b_right_high = update_channel(
        &p_filter->right, raw.b_right_high, now_ms, stable_ms);

    return stable;
}

static bool update_channel (
    ir_line_channel_t * p_channel,
    bool b_raw_high,
    uint32_t now_ms,
    uint32_t stable_ms)
{
    if (!p_channel->b_initialized)
    {
        p_channel->b_initialized = true;
        p_channel->b_stable_high = b_raw_high;
        p_channel->b_candidate_high = b_raw_high;
        p_channel->candidate_since_ms = now_ms;
    }
    else if (b_raw_high == p_channel->b_stable_high)
    {
        p_channel->b_candidate_high = b_raw_high;
        p_channel->candidate_since_ms = now_ms;
    }
    else if (b_raw_high != p_channel->b_candidate_high)
    {
        p_channel->b_candidate_high = b_raw_high;
        p_channel->candidate_since_ms = now_ms;
    }
    else if ((uint32_t)(now_ms - p_channel->candidate_since_ms) >= stable_ms)
    {
        p_channel->b_stable_high = b_raw_high;
    }

    return p_channel->b_stable_high;
}

/* End of file. */
