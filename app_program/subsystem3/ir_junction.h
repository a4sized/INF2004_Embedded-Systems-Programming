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
    IR_JUNCTION_FAILED,
    IR_JUNCTION_UTURNING,
    IR_JUNCTION_UTURN_SETTLING
} ir_junction_mode_t;

typedef enum
{
    IR_JUNCTION_NO_EVENT = 0,
    IR_JUNCTION_ARMED,
    IR_JUNCTION_CROSSING,
    IR_JUNCTION_TURN_STARTED,
    IR_JUNCTION_TURN_DONE,
    IR_JUNCTION_TURN_TIMEOUT,
    IR_JUNCTION_CROSSING_TIMEOUT,
    IR_JUNCTION_UTURN_DONE,
    IR_JUNCTION_STRAIGHT_DONE
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
    bool b_odometry_ready;
    int32_t left_ticks;
    int32_t right_ticks;
    int32_t start_left_ticks;
    int32_t start_right_ticks;
    uint32_t uturn_stop_ms;
} ir_junction_t;

#define IR_JUNCTION_LEVEL_SAMPLES 2u
#define IR_JUNCTION_TIMEOUT_MS 3000u
#define IR_JUNCTION_CROSSING_TIMEOUT_MS 3000u

/* User-accepted 180-degree pivot calibration after 147/136/132 mm trials.
 * Effective track models slip/drag; it is not the physical wheel spacing.
 */
#ifndef IR_UTURN_EFFECTIVE_TRACK_MM
#define IR_UTURN_EFFECTIVE_TRACK_MM 129u
#endif
#define IR_UTURN_TICKS_PER_REV 2050u
#define IR_UTURN_WHEEL_CIRCUM_MM 207u
#define IR_UTURN_WHEEL_TICKS ((IR_UTURN_EFFECTIVE_TRACK_MM * 355u * \
    IR_UTURN_TICKS_PER_REV + (2u * 113u * IR_UTURN_WHEEL_CIRCUM_MM - 1u)) / \
    (2u * 113u * IR_UTURN_WHEEL_CIRCUM_MM))
#define IR_UTURN_SETTLE_MS 200u
#define IR_UTURN_TIMEOUT_MS 8000u

/** Supply S2 signed totals before update. Does not reset encoders. */
void ir_junction_note_odometry (ir_junction_t * p_junction,
                                int32_t left_ticks, int32_t right_ticks);

/** Start the encoder-measured 180-degree pivot now, without waiting for a
 * junction (the isolated U-turn calibration trial). Call after
 * ir_junction_note_odometry. */
void ir_junction_start_uturn (ir_junction_t * p_junction, uint32_t now_ms);

/** Returns true while the junction policy owns the motion request.
 * Every mapped letter (left, right, U-turn, straight) arms on decode and
 * waits for the GP6 junction cue: white, then black (CROSSING), then the
 * exit white edge. There W/Z start their turn, F starts the pivot, and the
 * straight letter finishes (STRAIGHT_DONE) and hands back to the follower. */
bool ir_junction_update (ir_junction_t * p_junction,
                         ir_line_snapshot_t const * p_snapshot,
                         uint32_t now_ms, uint32_t max_age_ms,
                         bool b_follow_allowed, uint8_t base_speed,
                         uint8_t turn_speed,
                         ir_motion_target_t * p_target,
                         ir_junction_event_t * p_event);

#endif
