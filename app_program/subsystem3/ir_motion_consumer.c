/** @file ir_motion_consumer.c @brief 10 ms latest-snapshot motion consumer. */
#include "ir_motion_consumer.h"

#if defined(BUDDY3_TEAM_IR)
#include "ir_line_snapshot_shared.h"
#include "ir_motion_handoff.h"
#include "ir_junction.h"
#include "../subsystem2/subsystem2.h"
#include "../subsystem1/subsystem1.h"
#include "../systemFaults.h"

#define IR_MOTION_PERIOD_MS 10u
/* Keep the wheels even through short line-sensor gaps while reading bars.
 * A sustained both-white reading still returns control to line recovery.
 */
#define IR_BARCODE_WHITE_HOLD_MS 350u
#if defined(BUDDY3_IR_MOVING_TEST)
#define IR_TEST_STRAIGHT_MAX_MS 8000u
#endif

static ID settings_mutex;
/* True while Subsystem 5 drives an obstacle bypass: no Buddy 2 writes. */
static bool b_output_paused = false;
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
    b_output_paused = false;
    settings_mutex = tk_cre_mtx(&config);
    return (settings_mutex < E_OK) ? settings_mutex : E_OK;
}

ER ir_motion_consumer_set_paused (bool b_paused)
{
    ER const status = tk_loc_mtx(settings_mutex, TMO_FEVR);
    if (E_OK != status)
    {
        return status;
    }
    b_output_paused = b_paused;
    return tk_unl_mtx(settings_mutex);
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
    ir_follow_controller_t follow = {0};
#if defined(BUDDY3_BARCODE_DEBUG)
    ir_follow_mode_t logged_follow_mode = IR_FOLLOW_WAIT;
#endif
#if !defined(BUDDY3_IR_MOVING_TEST)
    ir_junction_t junction = {0};
    uint32_t barcode_white_started_ms = 0u;
    bool b_barcode_white_hold = false;
#endif
#if defined(BUDDY3_IR_MOVING_TEST)
    uint32_t diagnostic_cycles = 0u;
    uint32_t capture_started_ms = 0u;
    bool b_line_seen = false;
    bool b_capture_started = false;
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
        bool b_time_valid = false;
#if !defined(BUDDY3_IR_MOVING_TEST)
        ir_junction_event_t junction_event = IR_JUNCTION_NO_EVENT;
#endif
#if defined(BUDDY3_IR_MOVING_TEST)
        bool b_allowed = false;
#endif
        if (E_OK == tk_loc_mtx(settings_mutex, TMO_FEVR))
        {
            b_have_snapshot = ir_line_snapshot_shared_read(&snapshot);
            b_time_valid = b_have_snapshot && (E_OK == tk_get_tim(&now));
            if (b_time_valid)
            {
                /* A still-fresh publication remains valid for this cycle.
                 * Treating an unchanged sequence as NULL reset the ordinary
                 * loss/search state whenever sensing missed one consumer
                 * cycle under load. Freshness is checked by the controller.
                 */
                p_snapshot = &snapshot;
            }
#if defined(BUDDY3_IR_MOVING_TEST)
            b_allowed = settings.b_follow_allowed;
            if (!settings.b_follow_allowed)
            {
                b_line_seen = false;
                b_capture_started = false;
            }
            else if (((void *)0 != p_snapshot) &&
                     (IR_STEERING_AVAILABLE ==
                      ir_line_snapshot_steering_status(
                          p_snapshot, now.lo, settings.max_age_ms, true)))
            {
                b_line_seen = true;
            }
            if ((void *)0 != p_snapshot)
            {
                if (!p_snapshot->b_barcode_capturing)
                {
                    b_capture_started = false;
                }
                else if (!b_capture_started)
                {
                    capture_started_ms = now.lo;
                    b_capture_started = true;
                }
            }
            if (ir_barcode_capture_target(
                    p_snapshot, now.lo, settings.max_age_ms,
                    settings.b_follow_allowed,
                    b_line_seen && b_capture_started &&
                    ((uint32_t)(now.lo - capture_started_ms) <=
                     IR_TEST_STRAIGHT_MAX_MS),
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
            if ((NULL != p_snapshot) &&
                settings.b_follow_allowed &&
                p_snapshot->b_barcode_capturing &&
                (IR_LINE_BOTH_WHITE == p_snapshot->position) &&
                (IR_FOLLOW_TRACK == follow.mode))
            {
                if (!b_barcode_white_hold)
                {
                    barcode_white_started_ms = now.lo;
                    b_barcode_white_hold = true;
                }
            }
            else
            {
                b_barcode_white_hold = false;
            }
            if (ir_junction_update(&junction,
                                   b_time_valid ? &snapshot : (void *)0,
                                   now.lo,
                                   settings.max_age_ms,
                                   settings.b_follow_allowed,
                                   settings.base_speed,
                                   settings.turn_speed, &target,
                                   &junction_event))
            {
                /* Turn and failed states own the request. */
            }
            else
            {
                if (IR_JUNCTION_TURN_DONE == junction_event)
                {
                    ir_follow_controller_reset(&follow);
                }
                if (!b_barcode_white_hold ||
                    ((uint32_t)(now.lo - barcode_white_started_ms) >=
                     IR_BARCODE_WHITE_HOLD_MS) ||
                    !ir_barcode_capture_target(
                        p_snapshot, now.lo, settings.max_age_ms,
                        settings.b_follow_allowed,
                        (IR_FOLLOW_TRACK == follow.mode),
                        settings.base_speed, &target))
                {
                    target = ir_follow_controller_update(
                        &follow, p_snapshot, now.lo, settings.max_age_ms,
                        settings.b_follow_allowed, settings.base_speed,
                        settings.turn_speed);
                }
            }
#endif
            if (!b_output_paused)
            {
                s2_set_ir_velocity(target.left, target.right);
            }
            (void)tk_unl_mtx(settings_mutex);
        }
        else
        {
            s2_set_ir_velocity(0, 0);
        }
#if !defined(BUDDY3_IR_MOVING_TEST)
        switch (junction_event)
        {
            case IR_JUNCTION_ARMED:
                s1_csv_log_event("JUNCTION_ARMED",
                                 (IR_BARCODE_TURN_LEFT == junction.action) ?
                                     "LEFT" : "RIGHT");
                break;
            case IR_JUNCTION_TURN_STARTED:
                /* The letter has become a turn request. Clear the visible
                 * telemetry value so the next decode is a new instruction.
                 * The junction sequence number prevents stale rearming.
                 */
                s1_set_barcode_id(0u);
                s1_csv_log_event("JUNCTION_TURN",
                                 (IR_BARCODE_TURN_LEFT == junction.action) ?
                                     "LEFT" : "RIGHT");
                break;
            case IR_JUNCTION_CROSSING:
                s1_csv_log_event("JUNCTION_CROSSING", "FORWARD");
                break;
            case IR_JUNCTION_TURN_DONE:
                s1_csv_log_event("JUNCTION_DONE", "LINE_FOUND");
                break;
            case IR_JUNCTION_TURN_TIMEOUT:
                s1_csv_log_event("JUNCTION_TIMEOUT", "LINE_NOT_FOUND");
                s1_report_fault(FAULT_IR_NAV);
                break;
            case IR_JUNCTION_CROSSING_TIMEOUT:
                s1_csv_log_event("JUNCTION_TIMEOUT", "GP6_NOT_CLEARED");
                s1_report_fault(FAULT_IR_NAV);
                break;
            case IR_JUNCTION_NO_EVENT:
            default:
                break;
        }
#endif
#if defined(BUDDY3_BARCODE_DEBUG)
        if (follow.mode != logged_follow_mode)
        {
            char const * p_mode = "WAIT";
            switch (follow.mode)
            {
                case IR_FOLLOW_TRACK: p_mode = "TRACK"; break;
                case IR_FOLLOW_CORRECT: p_mode = "CORRECT"; break;
                case IR_FOLLOW_STOPPING: p_mode = "STOPPING"; break;
                case IR_FOLLOW_SEARCH: p_mode = "SEARCH"; break;
                case IR_FOLLOW_WAIT:
                default: break;
            }
            s1_csv_log_event("IR_FOLLOW_MODE", p_mode);
            logged_follow_mode = follow.mode;
        }
#endif
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
