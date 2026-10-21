/**
 * @file ir_line_state.c
 * @brief Four line positions and continuous loss/reacquisition timing.
 */
#include "ir_line_state.h"

#define IR_LOSS_MS       30u
#define IR_REACQUIRE_MS  20u

void ir_line_state_reset (ir_line_state_t * p_state)
{
    *p_state = (ir_line_state_t){0};
}

void ir_line_state_update (
    ir_line_state_t * p_state,
    drv_ir_levels_t stable,
    uint32_t now_ms)
{
    bool const b_left_black = stable.b_left_high;
    bool const b_right_black = stable.b_right_high;

    p_state->sampled_at_ms = now_ms;
    p_state->latest_steer = IR_LINE_STEER_NONE;

    if (b_left_black && b_right_black)
    {
        p_state->position = IR_LINE_CENTERED;
        p_state->b_white_run = false;
        if (!p_state->b_black_run)
        {
            p_state->black_since_ms = now_ms;
            p_state->b_black_run = true;
        }
        if ((uint32_t)(now_ms - p_state->black_since_ms) >= IR_REACQUIRE_MS)
        {
            p_state->b_lost = false;
        }
    }
    else if (b_left_black)
    {
        p_state->position = IR_LINE_LEFT_BLACK;
        p_state->latest_steer = IR_LINE_STEER_LEFT;
        p_state->b_white_run = false;
        p_state->b_black_run = false;
    }
    else if (b_right_black)
    {
        p_state->position = IR_LINE_RIGHT_BLACK;
        p_state->latest_steer = IR_LINE_STEER_RIGHT;
        p_state->b_white_run = false;
        p_state->b_black_run = false;
    }
    else
    {
        p_state->position = IR_LINE_BOTH_WHITE;
        p_state->b_black_run = false;
        if (!p_state->b_white_run)
        {
            p_state->white_since_ms = now_ms;
            p_state->b_white_run = true;
        }
        if ((uint32_t)(now_ms - p_state->white_since_ms) >= IR_LOSS_MS)
        {
            p_state->b_lost = true;
        }
    }
}

/* End of file. */
