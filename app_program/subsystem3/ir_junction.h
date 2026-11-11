/** @file ir_junction.h @brief Decoded-letter junction turn policy. */
#ifndef IR_JUNCTION_H
#define IR_JUNCTION_H

#include "ir_code39.h"
#include "ir_motion_handoff.h"

typedef enum
{
    IR_JUNCTION_IDLE = 0,
    IR_JUNCTION_WAIT_CLEAR,
    IR_JUNCTION_WAIT_BLACK,
    IR_JUNCTION_WAIT_EXIT,
    IR_JUNCTION_TURNING,
    IR_JUNCTION_FAILED
} ir_junction_mode_t;

typedef enum
{
    IR_JUNCTION_NO_EVENT = 0,
    IR_JUNCTION_ARMED,
    IR_JUNCTION_CROSSING,
    IR_JUNCTION_TURN_STARTED,
    IR_JUNCTION_TURN_DONE,
    IR_JUNCTION_TURN_TIMEOUT,
    IR_JUNCTION_CROSSING_TIMEOUT
} ir_junction_event_t;

typedef struct
{
    ir_junction_mode_t mode;
    ir_barcode_action_t action;
    uint32_t seen_decode_sequence;
    uint32_t crossing_started_ms;
    uint32_t turn_started_ms;
    uint8_t level_samples;
    uint8_t white_seen_mask;
    uint8_t barcode_phase;
    bool b_seen_decode_sequence;
    bool b_line_confirmed;
} ir_junction_t;

#define IR_JUNCTION_LEVEL_SAMPLES 2u
#define IR_JUNCTION_TIMEOUT_MS 3000u
#define IR_JUNCTION_CROSSING_TIMEOUT_MS 3000u

/** Returns true while the junction policy owns the motion request. */
bool ir_junction_update (ir_junction_t * p_junction,
                         ir_line_snapshot_t const * p_snapshot,
                         uint32_t now_ms, uint32_t max_age_ms,
                         bool b_follow_allowed, uint8_t base_speed,
                         uint8_t turn_speed,
                         ir_motion_target_t * p_target,
                         ir_junction_event_t * p_event);

#endif
