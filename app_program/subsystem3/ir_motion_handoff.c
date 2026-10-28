/** @file ir_motion_handoff.c @brief Bounded steering targets from fresh IR. */
#include "ir_motion_handoff.h"

ir_motion_target_t ir_motion_target_for_snapshot (
    ir_line_snapshot_t const * p_snapshot,
    uint32_t now_ms,
    uint32_t max_age_ms,
    bool b_follow_allowed,
    uint8_t base_speed,
    uint8_t correction)
{
    ir_motion_target_t target = {0};
    target.status = ir_line_snapshot_steering_status(
        p_snapshot, now_ms, max_age_ms, b_follow_allowed);
    if (IR_STEERING_AVAILABLE != target.status)
    {
        return target;
    }

    /* Keep each wheel in the forward 0..255 range. */
    uint16_t const fast = (uint16_t)base_speed + correction;
    int16_t const slow = (base_speed > correction) ?
        (int16_t)(base_speed - correction) : 0;
    int16_t const bounded_fast = (fast > 255u) ? 255 : (int16_t)fast;
    target.left = (int16_t)base_speed;
    target.right = (int16_t)base_speed;
    if (IR_LINE_STEER_LEFT == p_snapshot->steer)
    {
        target.left = slow;
        target.right = bounded_fast;
    }
    else if (IR_LINE_STEER_RIGHT == p_snapshot->steer)
    {
        target.left = bounded_fast;
        target.right = slow;
    }
    return target;
}
