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
    int8_t gentle_bias;
    uint32_t correction_started_ms;
    uint32_t stop_started_ms;
} ir_follow_controller_t;

#define IR_SEARCH_WHITE_SAMPLES 2u
#define IR_CORRECT_MS 250u
#define IR_CORRECT_SPEED 51u
#define IR_SEARCH_STOP_MS 250u
#define IR_GENTLE_BIAS_MAX 13
#define IR_GENTLE_BIAS_STEP 4

void ir_follow_controller_reset (ir_follow_controller_t * p_controller);

ir_motion_target_t ir_follow_controller_update (
    ir_follow_controller_t * p_controller,
    ir_line_snapshot_t const * p_snapshot,
    uint32_t now_ms,
    uint32_t max_age_ms,
    bool b_follow_allowed,
    uint8_t base_speed,
    uint8_t turn_speed);

/** USB moving-test straight request while GP6 capture is active. */
bool ir_barcode_capture_target (
    ir_line_snapshot_t const * p_snapshot,
    uint32_t now_ms,
    uint32_t max_age_ms,
    bool b_follow_allowed,
    bool b_line_seen,
    uint8_t base_speed,
    ir_motion_target_t * p_target);

/** Require a new publication for each Motion State cycle. */
bool ir_motion_snapshot_is_new (
    ir_line_snapshot_t const * p_snapshot,
    uint32_t last_sequence,
    bool b_seen_sequence);

/** Return the basic equal-speed target before tracking bias is applied. */
ir_motion_target_t ir_motion_target_for_snapshot (
    ir_line_snapshot_t const * p_snapshot,
    uint32_t now_ms,
    uint32_t max_age_ms,
    bool b_follow_allowed,
    uint8_t base_speed,
    uint8_t turn_speed);

#endif
