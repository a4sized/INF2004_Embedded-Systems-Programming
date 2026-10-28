/** @file ir_motion_consumer.c @brief 20 ms latest-snapshot motion consumer. */
#include "ir_motion_consumer.h"

#if defined(BUDDY3_TEAM_IR)
#include "ir_line_snapshot_shared.h"
#include "ir_motion_handoff.h"
#include "../subsystem2/subsystem2.h"

#define IR_MOTION_PERIOD_MS 20u
/* Pending controller ownership, mounted calibration and speed agreement. */
#define IR_FOLLOW_ALLOWED false
#define IR_BASE_SPEED 0u
#define IR_CORRECTION 0u
#define IR_MAX_AGE_MS 20u

void ir_motion_consumer_task (INT stacd, void * exinf)
{
    (void)stacd;
    (void)exinf;
    for (;;)
    {
        ir_line_snapshot_t snapshot = {0};
        SYSTIM now = {0};
        ir_line_snapshot_t const * p_snapshot = NULL;
        if (ir_line_snapshot_shared_read(&snapshot) &&
            (E_OK == tk_get_tim(&now)))
        {
            p_snapshot = &snapshot;
        }
        ir_motion_target_t const target = ir_motion_target_for_snapshot(
            p_snapshot, now.lo, IR_MAX_AGE_MS, IR_FOLLOW_ALLOWED,
            IR_BASE_SPEED, IR_CORRECTION);
        s2_set_ir_velocity(target.left, target.right);
        (void)tk_dly_tsk(IR_MOTION_PERIOD_MS);
    }
}
#endif
