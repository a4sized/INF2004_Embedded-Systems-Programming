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

/** Speed values and freshness are supplied by the controller integration. */
ir_motion_target_t ir_motion_target_for_snapshot (
    ir_line_snapshot_t const * p_snapshot,
    uint32_t now_ms,
    uint32_t max_age_ms,
    bool b_follow_allowed,
    uint8_t base_speed,
    uint8_t correction);

#endif
