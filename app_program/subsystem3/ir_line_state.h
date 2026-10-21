/**
 * @file ir_line_state.h
 * @brief Latest line position and timed loss status from filtered DO levels.
 */
#ifndef IR_LINE_STATE_H
#define IR_LINE_STATE_H

#include "drv_ir.h"

#include <stdint.h>

typedef enum
{
    IR_LINE_BOTH_WHITE = 0,
    IR_LINE_LEFT_BLACK,
    IR_LINE_RIGHT_BLACK,
    IR_LINE_CENTERED
} ir_line_position_t;

typedef enum
{
    IR_LINE_STEER_NONE = 0,
    IR_LINE_STEER_LEFT,
    IR_LINE_STEER_RIGHT
} ir_line_steer_t;

typedef struct
{
    ir_line_position_t position;
    ir_line_steer_t latest_steer;
    uint32_t sampled_at_ms;
    uint32_t white_since_ms;
    uint32_t black_since_ms;
    bool b_white_run;
    bool b_black_run;
    bool b_lost;
} ir_line_state_t;

/** @brief Reset the latest decision and consecutive-state timers. */
void ir_line_state_reset (ir_line_state_t * p_state);

/**
 * @brief Replace the latest decision from one filtered 5 ms sample.
 * @details Both white for 30 ms confirms loss. After loss, both black for
 * 20 ms clears it. Time subtraction tolerates a wrapping 32-bit ms clock.
 * This stores a decision only; it does not send motion requests.
 */
void ir_line_state_update (
    ir_line_state_t * p_state,
    drv_ir_levels_t stable,
    uint32_t now_ms);

#endif

/* End of file. */
