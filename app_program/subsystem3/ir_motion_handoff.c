/** @file ir_motion_handoff.c @brief Bounded steering targets from fresh IR. */
#include "ir_motion_handoff.h"
#include <stddef.h>

bool ir_motion_snapshot_is_new (
    ir_line_snapshot_t const * p_snapshot,
    uint32_t last_sequence,
    bool b_seen_sequence)
{
    return ((void *)0 != p_snapshot) &&
           (!b_seen_sequence || (p_snapshot->sequence != last_sequence));
}

ir_motion_target_t ir_motion_target_for_snapshot (
    ir_line_snapshot_t const * p_snapshot,
    uint32_t now_ms,
    uint32_t max_age_ms,
    bool b_follow_allowed,
    uint8_t base_speed,
    uint8_t turn_speed)
{
    ir_motion_target_t target = {0};
    target.status = ir_line_snapshot_steering_status(
        p_snapshot, now_ms, max_age_ms, b_follow_allowed);
    if (IR_STEERING_AVAILABLE != target.status)
    {
        return target;
    }

    (void)turn_speed;
    target.left = (int16_t)base_speed;
    target.right = (int16_t)base_speed;
    return target;
}

void ir_follow_controller_reset (ir_follow_controller_t * p_controller)
{
    *p_controller = (ir_follow_controller_t){0};
}

bool ir_barcode_capture_target (
    ir_line_snapshot_t const * p_snapshot,
    uint32_t now_ms,
    uint32_t max_age_ms,
    bool b_follow_allowed,
    bool b_line_seen,
    uint8_t base_speed,
    ir_motion_target_t * p_target)
{
    if ((NULL == p_snapshot) || (NULL == p_target) ||
        !p_snapshot->b_barcode_capturing || p_snapshot->b_barcode_decoded ||
        !b_follow_allowed ||
        !b_line_seen || !p_snapshot->b_gpio_ready ||
        !p_snapshot->b_ready ||
        ((uint32_t)(now_ms - p_snapshot->sampled_at_ms) > max_age_ms))
    {
        return false;
    }

    *p_target = (ir_motion_target_t)
    {
        .left = (int16_t)base_speed,
        .right = (int16_t)base_speed,
        .status = IR_STEERING_AVAILABLE
    };
    return true;
}

ir_motion_target_t ir_follow_controller_update (
    ir_follow_controller_t * p_controller,
    ir_line_snapshot_t const * p_snapshot,
    uint32_t now_ms,
    uint32_t max_age_ms,
    bool b_follow_allowed,
    uint8_t base_speed,
    uint8_t turn_speed)
{
    ir_motion_target_t target = ir_motion_target_for_snapshot(
        p_snapshot, now_ms, max_age_ms, b_follow_allowed,
        base_speed, turn_speed);

    if ((IR_STEERING_PAUSED == target.status) ||
        (IR_STEERING_UNREADY == target.status) ||
        (IR_STEERING_STALE == target.status))
    {
        ir_follow_controller_reset(p_controller);
        return target;
    }

    if (IR_STEERING_AVAILABLE == target.status)
    {
        p_controller->mode = IR_FOLLOW_TRACK;
        p_controller->both_white_samples = 0u;
        if (IR_LINE_LEFT_BLACK == p_snapshot->position)
        {
            p_controller->last_side = IR_SEARCH_LEFT;
            p_controller->b_last_side_valid = true;
        }
        else if (IR_LINE_RIGHT_BLACK == p_snapshot->position)
        {
            p_controller->last_side = IR_SEARCH_RIGHT;
            p_controller->b_last_side_valid = true;
        }
        return target;
    }

    /* Only simultaneous white readings can start or sustain search. */
    if ((NULL == p_snapshot) ||
        (IR_LINE_BOTH_WHITE != p_snapshot->position))
    {
        ir_follow_controller_reset(p_controller);
        return target;
    }

    if (IR_FOLLOW_TRACK == p_controller->mode)
    {
        if (p_controller->both_white_samples < IR_SEARCH_WHITE_SAMPLES)
        {
            ++p_controller->both_white_samples;
        }
        if (p_controller->both_white_samples >= IR_SEARCH_WHITE_SAMPLES)
        {
            if (p_controller->b_last_side_valid)
            {
                p_controller->mode = IR_FOLLOW_CORRECT;
                p_controller->correction_started_ms = now_ms;
            }
            else
            {
                p_controller->mode = IR_FOLLOW_STOPPING;
                p_controller->stop_started_ms = now_ms;
            }
        }
    }
    else if ((IR_FOLLOW_CORRECT == p_controller->mode) &&
             ((uint32_t)(now_ms - p_controller->correction_started_ms) >=
              IR_CORRECT_MS))
    {
        p_controller->mode = IR_FOLLOW_STOPPING;
        p_controller->stop_started_ms = now_ms;
    }
    else if ((IR_FOLLOW_STOPPING == p_controller->mode) &&
             ((uint32_t)(now_ms - p_controller->stop_started_ms) >=
              IR_SEARCH_STOP_MS))
    {
        p_controller->mode = IR_FOLLOW_SEARCH;
        return target; /* Still zero on the transition cycle. */
    }

    if (IR_FOLLOW_CORRECT == p_controller->mode)
    {
        target.status = IR_STEERING_AVAILABLE;
        if (IR_SEARCH_LEFT == p_controller->last_side)
        {
            target.right = (int16_t)IR_CORRECT_SPEED;
        }
        else
        {
            target.left = (int16_t)IR_CORRECT_SPEED;
        }
    }
    else if (IR_FOLLOW_SEARCH == p_controller->mode)
    {
        target.status = IR_STEERING_AVAILABLE;
        if (IR_SEARCH_LEFT == p_controller->last_side)
        {
            target.right = (int16_t)turn_speed;
        }
        else
        {
            target.left = (int16_t)turn_speed;
        }
    }
    return target;
}
