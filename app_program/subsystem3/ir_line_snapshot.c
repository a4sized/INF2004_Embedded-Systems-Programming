/** @file ir_line_snapshot.c
 *  @brief Steering eligibility from one IR snapshot.
 */
#include "ir_line_snapshot.h"

#include <stddef.h>

ir_steering_status_t ir_line_snapshot_steering_status (
    ir_line_snapshot_t const * p_snapshot,
    uint32_t now_ms,
    uint32_t max_age_ms,
    bool b_follow_allowed)
{
    if ((NULL == p_snapshot) || !p_snapshot->b_gpio_ready ||
        !p_snapshot->b_ready)
    {
        return IR_STEERING_UNREADY;
    }

    if (!b_follow_allowed)
    {
        return IR_STEERING_PAUSED;
    }

    if ((now_ms - p_snapshot->sampled_at_ms) > max_age_ms)
    {
        return IR_STEERING_STALE;
    }

    if (p_snapshot->b_lost)
    {
        return IR_STEERING_NO_LINE;
    }

    switch (p_snapshot->position)
    {
        case IR_LINE_LEFT_BLACK:
            return (IR_LINE_STEER_LEFT == p_snapshot->steer) ?
                IR_STEERING_AVAILABLE : IR_STEERING_NO_LINE;
        case IR_LINE_RIGHT_BLACK:
            return (IR_LINE_STEER_RIGHT == p_snapshot->steer) ?
                IR_STEERING_AVAILABLE : IR_STEERING_NO_LINE;
        case IR_LINE_CENTERED:
            return (IR_LINE_STEER_NONE == p_snapshot->steer) ?
                IR_STEERING_AVAILABLE : IR_STEERING_NO_LINE;
        case IR_LINE_BOTH_WHITE:
        default:
            return IR_STEERING_NO_LINE;
    }
}
