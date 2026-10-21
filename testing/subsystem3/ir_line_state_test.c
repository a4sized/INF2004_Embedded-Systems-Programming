/**
 * @file ir_line_state_test.c
 * @brief Host checks for latest steering and continuous line timing.
 */
#include "ir_line_state.h"

#include <assert.h>
#include <stdint.h>

int main (void)
{
    ir_line_state_t state = {0};
    drv_ir_levels_t levels = {0};

    ir_line_state_reset(&state);
    levels.b_left_high = true;
    ir_line_state_update(&state, levels, 0u);
    assert(state.position == IR_LINE_LEFT_BLACK);
    assert(state.latest_steer == IR_LINE_STEER_LEFT);

    levels.b_left_high = false;
    levels.b_right_high = true;
    ir_line_state_update(&state, levels, 5u);
    assert(state.position == IR_LINE_RIGHT_BLACK);
    assert(state.latest_steer == IR_LINE_STEER_RIGHT);
    assert(state.sampled_at_ms == 5u);

    levels.b_left_high = true;
    ir_line_state_update(&state, levels, 10u);
    assert(state.position == IR_LINE_CENTERED);
    assert(state.latest_steer == IR_LINE_STEER_NONE);

    levels.b_left_high = false;
    levels.b_right_high = false;
    ir_line_state_update(&state, levels, 15u);
    assert(!state.b_lost);
    ir_line_state_update(&state, levels, 40u);
    assert(!state.b_lost);
    ir_line_state_update(&state, levels, 45u);
    assert(state.b_lost);

    levels.b_left_high = true;
    ir_line_state_update(&state, levels, 50u);
    assert(state.b_lost);
    assert(state.latest_steer == IR_LINE_STEER_LEFT);

    levels.b_right_high = true;
    ir_line_state_update(&state, levels, 55u);
    assert(state.b_lost);
    ir_line_state_update(&state, levels, 70u);
    assert(state.b_lost);
    ir_line_state_update(&state, levels, 75u);
    assert(!state.b_lost);

    /* A white run spanning 32-bit clock wrap must still confirm loss. */
    ir_line_state_reset(&state);
    levels.b_left_high = false;
    levels.b_right_high = false;
    ir_line_state_update(&state, levels, UINT32_MAX - 9u);
    ir_line_state_update(&state, levels, 20u);
    assert(state.b_lost);

    return 0;
}

/* End of file. */
