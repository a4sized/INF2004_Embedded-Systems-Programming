/** @file ir_motion_consumer.c @brief 10 ms latest-snapshot motion consumer. */
#include "ir_motion_consumer.h"

#if defined(BUDDY3_TEAM_IR)
#include "ir_line_snapshot_shared.h"
#include "ir_motion_handoff.h"
#include "../subsystem2/subsystem2.h"
#if defined(BUDDY3_IR_MOVING_TEST)
#include "../subsystem1/subsystem1.h"
#endif

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

#if defined(BUDDY3_IR_MOVING_TEST)
static char * moving_append_text (char * p_out, char const * p_text)
{
    while ('\0' != *p_text)
    {
        *p_out++ = *p_text++;
    }
    return p_out;
}

static char * moving_append_u16 (char * p_out, uint16_t value)
{
    char digits[5];
    uint32_t count = 0u;
    do
    {
        digits[count++] = (char)('0' + (char)(value % 10u));
        value /= 10u;
    } while (0u != value);
    while (0u != count)
    {
        *p_out++ = digits[--count];
    }
    return p_out;
}

static void moving_log_motion (ir_line_snapshot_t const * p_snapshot,
                               bool b_have_snapshot, bool b_allowed,
                               ir_follow_mode_t mode,
                               ir_motion_target_t target)
{
    char details[96];
    char * p_out = moving_append_text(details, "L=");
    *p_out++ = b_have_snapshot && p_snapshot->raw.b_left_high ? '1' : '0';
    *p_out++ = '>';
    *p_out++ = b_have_snapshot && p_snapshot->stable.b_left_high ? '1' : '0';
    p_out = moving_append_text(p_out, ";R=");
    *p_out++ = b_have_snapshot && p_snapshot->raw.b_right_high ? '1' : '0';
    *p_out++ = '>';
    *p_out++ = b_have_snapshot && p_snapshot->stable.b_right_high ? '1' : '0';
    p_out = moving_append_text(p_out, ";B=");
    *p_out++ = b_have_snapshot && p_snapshot->raw.b_barcode_high ? '1' : '0';
    p_out = moving_append_text(p_out, ";capture=");
    *p_out++ = b_have_snapshot && p_snapshot->b_barcode_capturing ? '1' : '0';
    p_out = moving_append_text(p_out, ";pos=");
    p_out = moving_append_u16(p_out,
                              b_have_snapshot ?
                                  (uint16_t)p_snapshot->position : 0u);
    p_out = moving_append_text(p_out, ";mode=");
    p_out = moving_append_u16(p_out, (uint16_t)mode);
    p_out = moving_append_text(p_out, ";allowed=");
    *p_out++ = b_allowed ? '1' : '0';
    p_out = moving_append_text(p_out, ";status=");
    p_out = moving_append_u16(p_out, (uint16_t)target.status);
    p_out = moving_append_text(p_out, ";req=");
    p_out = moving_append_u16(p_out, (uint16_t)target.left);
    *p_out++ = '/';
    p_out = moving_append_u16(p_out, (uint16_t)target.right);
    *p_out = '\0';
    s1_csv_log_event("IR_MOTION", details);
}
#endif

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
#if defined(BUDDY3_IR_MOVING_TEST)
    uint32_t diagnostic_cycles = 0u;
    bool b_line_seen = false;
#endif
    (void)stacd;
    (void)exinf;
    /* Permission is supplied only after whole-vehicle startup is complete. */
    s2_set_ir_velocity(0, 0);
    for (;;)
    {
        ir_line_snapshot_t snapshot = {0};
        SYSTIM now = {0};
        ir_line_snapshot_t const * p_snapshot = (void *)0;
        ir_motion_target_t target = {0};
        bool b_have_snapshot = false;
#if defined(BUDDY3_IR_MOVING_TEST)
        bool b_allowed = false;
#endif
        if (E_OK == tk_loc_mtx(settings_mutex, TMO_FEVR))
        {
            b_have_snapshot = ir_line_snapshot_shared_read(&snapshot);
            if (b_have_snapshot &&
                (E_OK == tk_get_tim(&now)) &&
                ir_motion_snapshot_is_new(&snapshot, last_sequence,
                                          b_seen_sequence))
            {
                p_snapshot = &snapshot;
                last_sequence = snapshot.sequence;
                b_seen_sequence = true;
            }
#if defined(BUDDY3_IR_MOVING_TEST)
            b_allowed = settings.b_follow_allowed;
            if (!settings.b_follow_allowed)
            {
                b_line_seen = false;
            }
            else if (((void *)0 != p_snapshot) &&
                     (IR_STEERING_AVAILABLE ==
                      ir_line_snapshot_steering_status(
                          p_snapshot, now.lo, settings.max_age_ms, true)))
            {
                b_line_seen = true;
            }
            if (ir_barcode_capture_target(
                    p_snapshot, now.lo, settings.max_age_ms,
                    settings.b_follow_allowed, b_line_seen,
                    settings.base_speed, &target))
            {
                /* Keep the last tracked side across the barcode. When the
                 * GP6 first edge arrives with a valid line reading, seed
                 * that side before freezing ordinary search timers.
                 */
                if ((IR_FOLLOW_WAIT == follow.mode) &&
                    (IR_STEERING_AVAILABLE ==
                     ir_line_snapshot_steering_status(
                         p_snapshot, now.lo, settings.max_age_ms, true)))
                {
                    (void)ir_follow_controller_update(
                        &follow, p_snapshot, now.lo, settings.max_age_ms,
                        true, settings.base_speed, settings.turn_speed);
                }
            }
            else
            {
                target = ir_follow_controller_update(
                    &follow, p_snapshot, now.lo, settings.max_age_ms,
                    settings.b_follow_allowed, settings.base_speed,
                    settings.turn_speed);
            }
#else
            target = ir_follow_controller_update(
                &follow, p_snapshot, now.lo, settings.max_age_ms,
                settings.b_follow_allowed, settings.base_speed,
                settings.turn_speed);
#endif
            s2_set_ir_velocity(target.left, target.right);
            (void)tk_unl_mtx(settings_mutex);
        }
        else
        {
            s2_set_ir_velocity(0, 0);
        }
#if defined(BUDDY3_IR_MOVING_TEST)
        if (++diagnostic_cycles >= 50u)
        {
            moving_log_motion(&snapshot, b_have_snapshot, b_allowed,
                              follow.mode, target);
            diagnostic_cycles = 0u;
        }
#endif
        (void)tk_dly_tsk(IR_MOTION_PERIOD_MS);
    }
}
#endif
