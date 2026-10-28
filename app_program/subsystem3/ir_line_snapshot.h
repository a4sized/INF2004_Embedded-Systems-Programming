/** @file ir_line_snapshot.h
 *  @brief One timestamped IR reading for task readers.
 */
#ifndef IR_LINE_SNAPSHOT_H
#define IR_LINE_SNAPSHOT_H

#include "ir_line_state.h"
#include "ir_sensing.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint32_t sequence;
    uint32_t sampled_at_ms;
    drv_ir_levels_t raw;
    drv_ir_levels_t stable;
    ir_surface_t left_surface;
    ir_surface_t right_surface;
    ir_surface_t barcode_surface;
    ir_line_position_t position;
    ir_line_steer_t steer;
    bool b_lost;
    bool b_gpio_ready;
    bool b_ready;
} ir_line_snapshot_t;

typedef enum
{
    IR_STEERING_UNREADY = 0,
    IR_STEERING_PAUSED,
    IR_STEERING_STALE,
    IR_STEERING_NO_LINE,
    IR_STEERING_AVAILABLE
} ir_steering_status_t;

/**
 * @brief Check whether a snapshot may be used for ordinary steering.
 * @param b_follow_allowed Permission supplied by the Vehicle Controller.
 * @param max_age_ms Caller-agreed freshness limit; not a global policy.
 * @details Both-white and confirmed loss are never movement permission.
 * A centered reading is available with IR_LINE_STEER_NONE.
 */
ir_steering_status_t ir_line_snapshot_steering_status (
    ir_line_snapshot_t const * p_snapshot,
    uint32_t now_ms,
    uint32_t max_age_ms,
    bool b_follow_allowed);

#endif
