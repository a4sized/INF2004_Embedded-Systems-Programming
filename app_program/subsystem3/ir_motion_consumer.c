/** @file ir_motion_consumer.c @brief 10 ms latest-snapshot motion consumer. */
#include "ir_motion_consumer.h"

#if defined(BUDDY3_TEAM_IR)
#include "ir_line_snapshot_shared.h"
#include "ir_motion_handoff.h"
#include "../subsystem2/subsystem2.h"

#define IR_MOTION_PERIOD_MS 10u

static ID settings_mutex;
/* Movement stays disabled until the Vehicle Controller explicitly enables it. */
static ir_motion_settings_t settings =
{
    .b_follow_allowed = false,
    .base_speed = IR_START_BASE_SPEED,
    .turn_speed = IR_START_TURN_SPEED,
    .max_age_ms = IR_START_MAX_AGE_MS
};

ER ir_motion_consumer_init (void)
{
    T_CMTX config = {0};
    config.mtxatr = TA_INHERIT;
    settings = (ir_motion_settings_t)
    {
        .b_follow_allowed = false,
        .base_speed = IR_START_BASE_SPEED,
        .turn_speed = IR_START_TURN_SPEED,
        .max_age_ms = IR_START_MAX_AGE_MS
    };
    settings_mutex = tk_cre_mtx(&config);
    return (settings_mutex < E_OK) ? settings_mutex : E_OK;
}

ER ir_motion_consumer_set_following (ir_motion_settings_t const * p_settings)
{
    if (((void *)0 == p_settings) ||
        (p_settings->b_follow_allowed &&
         ((0u == p_settings->base_speed) ||
          (p_settings->base_speed > IR_START_BASE_SPEED) ||
          (p_settings->turn_speed > IR_START_TURN_SPEED) ||
          (0u == p_settings->max_age_ms) ||
          (p_settings->max_age_ms > IR_START_MAX_AGE_MS))))
    {
        return E_PAR;
    }
    ER const status = tk_loc_mtx(settings_mutex, TMO_FEVR);
    if (E_OK != status)
    {
        return status;
    }
    settings = *p_settings;
    if (!settings.b_follow_allowed)
    {
        /* Serialize revocation with the consumer's own Buddy 2 write. */
        s2_set_ir_velocity(0, 0);
    }
    return tk_unl_mtx(settings_mutex);
}

void ir_motion_consumer_task (INT stacd, void * exinf)
{
    uint32_t last_sequence = 0u;
    bool b_seen_sequence = false;
    ir_follow_controller_t follow = {0};
    (void)stacd;
    (void)exinf;
    /* Permission is supplied only after whole-vehicle startup is complete. */
    s2_set_ir_velocity(0, 0);
    for (;;)
    {
        ir_line_snapshot_t snapshot = {0};
        SYSTIM now = {0};
        ir_line_snapshot_t const * p_snapshot = (void *)0;
        if (E_OK == tk_loc_mtx(settings_mutex, TMO_FEVR))
        {
            if (ir_line_snapshot_shared_read(&snapshot) &&
                (E_OK == tk_get_tim(&now)) &&
                ir_motion_snapshot_is_new(&snapshot, last_sequence,
                                          b_seen_sequence))
            {
                p_snapshot = &snapshot;
                last_sequence = snapshot.sequence;
                b_seen_sequence = true;
            }
            ir_motion_target_t const target = ir_follow_controller_update(
                &follow, p_snapshot, now.lo, settings.max_age_ms,
                settings.b_follow_allowed, settings.base_speed,
                settings.turn_speed);
            s2_set_ir_velocity(target.left, target.right);
            (void)tk_unl_mtx(settings_mutex);
        }
        else
        {
            s2_set_ir_velocity(0, 0);
        }
        (void)tk_dly_tsk(IR_MOTION_PERIOD_MS);
    }
}
#endif
