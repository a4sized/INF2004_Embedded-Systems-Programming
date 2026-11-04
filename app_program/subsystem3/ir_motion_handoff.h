/** @file ir_motion_handoff.h @brief Pure IR-to-motion request policy. */
#ifndef IR_MOTION_HANDOFF_H
#define IR_MOTION_HANDOFF_H

#include "ir_line_snapshot.h"

typedef struct
{
    int16_t left;
    int16_t right;
    ir_steering_status_t status;
} ir_motion_target_t;

typedef enum
{
    IR_FOLLOW_WAIT = 0,
    IR_FOLLOW_TRACK,
    IR_FOLLOW_CORRECT,
    IR_FOLLOW_STOPPING,
    IR_FOLLOW_SEARCH
} ir_follow_mode_t;

typedef enum
{
    IR_SEARCH_LEFT = 0,
    IR_SEARCH_RIGHT
} ir_search_side_t;

typedef struct
{
    ir_follow_mode_t mode;
    ir_search_side_t last_side;
    bool b_last_side_valid;
    uint8_t both_white_samples;
    uint32_t correction_started_ms;
    uint32_t stop_started_ms;
} ir_follow_controller_t;

#define IR_SEARCH_WHITE_SAMPLES 2u
#define IR_CORRECT_MS 250u
#define IR_CORRECT_SPEED 30u
#define IR_SEARCH_STOP_MS 250u

void ir_follow_controller_reset (ir_follow_controller_t * p_controller);

ir_motion_target_t ir_follow_controller_update (
    ir_follow_controller_t * p_controller,
    ir_line_snapshot_t const * p_snapshot,
    uint32_t now_ms,
    uint32_t max_age_ms,
    bool b_follow_allowed,
    uint8_t base_speed,
    uint8_t turn_speed);

/** Require a new publication for each Motion State cycle. */
bool ir_motion_snapshot_is_new (
    ir_line_snapshot_t const * p_snapshot,
    uint32_t last_sequence,
    bool b_seen_sequence);

/** Ordinary tracking drives straight while either line sensor sees black. */
ir_motion_target_t ir_motion_target_for_snapshot (
    ir_line_snapshot_t const * p_snapshot,
    uint32_t now_ms,
    uint32_t max_age_ms,
    bool b_follow_allowed,
    uint8_t base_speed,
    uint8_t turn_speed);

#endif
