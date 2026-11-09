/**
 * @file    subsystem5.c
 * @brief   Subsystem 5: servo-mounted ultrasonic obstacle scanner.
 *
 * Start-up (only once, when the car powers up)
 *   The car is held still (speed cap 0) while the servo turns to face the
 *   front of the car (90 degrees). As soon as it does, Subsystem 5 is ready
 *   (subsystem5_startup_scan_ready) and the hold is released; from then on
 *   the readings straight ahead limit the car. There is no start-up sweep.
 *   90 degrees is straight ahead; below 90 is one side, above 90 the other.
 *
 * Normal running
 *   The servo stays at 90 degrees and only the straight-ahead distance is
 *   used. A reading is printed every S5_FORWARD_PRINT_MS, e.g.
 *       "S5 scan 90 deg: 342 mm"   or   "S5 scan 90 deg: no obstacle"
 *
 * Stages 2 to 4, fine scan, obstacle profile and bypass decision (each
 * time the car stops for an obstacle)
 *   The car is held still while the servo steps outward from 90 degrees,
 *   S5_FINE_STEP_DEG at a time, first to the low side, then to the high
 *   side, until the reading jumps off the obstacle (its edge) or the scan
 *   limit is reached. The readings go to s5_profile.c, which
 *   works out the obstacle's closest point, location, width and the free
 *   gap beside it on the left and right. The servo turns back to 90
 *   degrees, and s5_avoid.c decides what to do: continue, go round left,
 *   go round right, reverse and scan again, or stop. The scan runs once
 *   per stop: it re-arms when the obstacle clears.
 *
 * Driving the bypass (S5_DRIVE_BYPASS)
 *   While it handles an obstacle Subsystem 5 reports "avoiding"
 *   (subsystem5_is_avoiding). The vehicle controller then pauses the line
 *   follower and gives Subsystem 5 permission to drive
 *   (subsystem5_set_drive_permission). Only with that permission does it
 *   carry the decision out through s2_set_ir_velocity(), measuring each
 *   move with the wheel encoders (s5_drive.c): reverse slowly and scan
 *   again, or go round the obstacle. Then it stops the wheels and hands
 *   the car back to the line follower. Losing the permission mid-move
 *   stops the wheels at once.
 *
 * Clearance test (S5_CLEARANCE_TEST = 1)
 *   The car stops S5_STOP_DISTANCE_CM in front of the first obstacle and
 *   fine scans it. Only the clearance (left and right edge from the car's
 *   centre line) and the bypass decision are printed. The car is then held
 *   still (speed cap 0) and nothing more is driven or handled until reset.
 *   All other Subsystem 5 console output is off (s5_log.h).
 *
 * Detection range
 *   Straight-ahead readings further than S5_DETECT_RANGE_CM
 *   count as "no obstacle". The fine scan uses the sensor's full
 *   S5_MAX_RANGE_CM.
 *
 * The sensor is pinged every S5_PING_INTERVAL_MS the whole time. Every
 * good reading counts toward sensor health (s5_report.c: repeated sensor or
 * servo failures raise FAULT_US_SENS). Coarse-scan readings also go to
 * telemetry, and only straight-ahead readings go to the obstacle guard
 * (s5_guard.c), which caps Subsystem 2's speed: slow near an obstacle, stop
 * S5_STOP_DISTANCE_CM away from it. So a side reading can neither stop the
 * car nor release a stop.
 * A failed reading is never treated as "no obstacle", so it cannot release
 * a stop.
 *
 * Casts in this file: (UB *) only retypes a string literal for the T-Kernel
 * tm_printf() API; (unsigned int) and (int) widen values that always fit;
 * (void) discards a return value that is not needed.
 */
#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>
#include "../app.h"
#include "subsystem5.h"
#include "s5_config.h"
#include "s5_log.h"
#include "drv_servo.h"
#include "drv_ultrasonic.h"
#include "s5_report.h"
#include "s5_guard.h"
#include "s5_profile.h"
#include "s5_avoid.h"
#include "s5_drive.h"
#include "s5_edge.h"
#include "s5_filter.h"

_Static_assert((S5_FINE_SAMPLES >= 1u) && (S5_FINE_SAMPLES <= S5_FILTER_MAX),
               "S5_FINE_SAMPLES must be between 1 and S5_FILTER_MAX");

/* Telemetry slot (s5_report.c) for readings taken straight ahead. */
#define FORWARD_POSITION    (2u)

/* Not a coarse-scan position: the reading is from the fine scan, so it only
 * counts toward sensor health (not stored, not sent to the guard). */
#define FINE_SCAN_POSITION  (S5_SCAN_POSITIONS)

/* Servo steps between two pings while the servo is turning (rounded up). */
#define STEPS_PER_PING      (((S5_PING_INTERVAL_MS + SERVO_STEP_DELAY_MS) - \
                              1u) / SERVO_STEP_DELAY_MS)

/* Fine-scan steps on each side of 90 degrees. */
#define FINE_LOW_STEPS      ((S5_FORWARD_DEG - S5_FINE_MIN_ANGLE) / \
                             S5_FINE_STEP_DEG)
#define FINE_HIGH_STEPS     ((S5_FINE_MAX_ANGLE - S5_FORWARD_DEG) / \
                             S5_FINE_STEP_DEG)

/* Driving a bypass (S5_DRIVE_BYPASS): check the encoders every
 * DRIVE_POLL_MS (Subsystem 2's control period), ping every
 * S5_PING_INTERVAL_MS. */
#define DRIVE_POLL_MS       (S5_DRIVE_POLL_MS)
#define BYPASS_LEGS         (8u)
#define TURN_DEG            (90u)
#define HALF_CAR_CM         ((S5_CAR_WIDTH_CM + 1u) / 2u)   /* rounded up */

_Static_assert(0u == (S5_PING_INTERVAL_MS % DRIVE_POLL_MS),
               "ping interval must be a multiple of the drive poll");
_Static_assert((S5_STOP_TRIGGER_CM < S5_DETECT_RANGE_CM) &&
               (S5_SLOW_DISTANCE_CM <= S5_DETECT_RANGE_CM) &&
               (S5_DETECT_RANGE_CM <= S5_MAX_RANGE_CM),
               "slow distance <= detect range <= sensor range");

_Static_assert(((S5_STOP_DISTANCE_CM * S5_MM_PER_CM) + S5_SCAN_BACKUP_MM) <
               (S5_STILL_BLOCKED_CM * S5_MM_PER_CM),
               "after backing off to scan, the obstacle must still count as "
               "in the way");

_Static_assert((S5_FINE_MIN_ANGLE < S5_FORWARD_DEG) &&
               (S5_FINE_MAX_ANGLE > S5_FORWARD_DEG) &&
               (S5_FINE_MAX_ANGLE <= S5_SERVO_MAX_DEG),
               "fine scan must cover both sides of 90 degrees");

/* Minimum-bit scanner state: 3 bits. b_hardware_ready is set once in init
 * (before the scanning task exists); b_profile_done and b_test_done are
 * used only by the scanning task, so they never race. */
typedef struct
{
    uint8_t b_hardware_ready : 1;   /* 1 = servo and sensor both started  */
    uint8_t b_profile_done   : 1;   /* 1 = current obstacle already profiled */
    uint8_t b_test_done      : 1;   /* 1 = clearance test over: car held  */
} scanner_state_t;

/* One move of a bypass. */
typedef struct
{
    s5_move_t move;     /* which way                          */
    uint16_t  amount;   /* mm (forward / back) or degrees     */
} drive_leg_t;

static scanner_state_t g_scanner = { 0u, 0u, 0u };

/* Start-up readiness, read by other tasks (subsystem5_startup_scan_ready):
 * true once the servo faces the front of the car. Written only by the
 * scanning task (and cleared once in init, before it exists); accessed with
 * atomic loads and stores so a reader in another task always sees a whole,
 * up-to-date value. */
static bool gb_facing_front = false;

/* Obstacle hand-over with the vehicle controller, same access rules:
 * gb_avoiding is written by the scanning task, gb_drive_permitted by the
 * vehicle controller task. */
static bool gb_avoiding        = false;
static bool gb_drive_permitted = false;

/* True while the vehicle controller lets Subsystem 5 drive. */
static bool s5_drive_permitted (void)
{
    return __atomic_load_n(&gb_drive_permitted, __ATOMIC_ACQUIRE);
}

/* Print one good reading taken at the servo's current angle. */
static void s5_print_reading (const uint16_t distance_mm)
{
    if (NO_OBSTACLE == distance_mm)
    {
        S5_LOG((UB *)"S5 scan %u deg: no obstacle\n",
               (unsigned int)drv_servo_get_angle());
    }
    else
    {
        S5_LOG((UB *)"S5 scan %u deg: %u mm\n",
               (unsigned int)drv_servo_get_angle(),
               (unsigned int)distance_mm);
    }
}

/*
 * Take one reading and report it. A coarse-scan 'position' is also stored
 * for telemetry. Only a reading taken with the servo exactly at
 * S5_FORWARD_DEG goes to the obstacle guard: readings taken while the servo
 * is still turning toward 90 look sideways and must not stop the car.
 * FINE_SCAN_POSITION only counts toward sensor health.
 * b_print = true also prints it. If p_distance_mm is not NULL it receives
 * the distance. Returns true if a good reading (a distance or "no
 * obstacle") was taken.
 */
static bool s5_take_reading (const uint8_t position, const bool b_print,
                             uint16_t * const p_distance_mm)
{
    uint16_t distance_mm    = NO_OBSTACLE;
    bool     b_good_reading = false;
    ER       result         = drv_ultrasonic_measure(&distance_mm);

    if (E_BUSY == result)
    {
        /* Sensor still finishing the last ping: no reading, not a fault. */
    }
    else if (result < E_OK)
    {
        /* Keep the last good distances, so a failure never releases a stop. */
        s5_report_sensor_error();
        if (b_print)
        {
            S5_LOG((UB *)"S5 scan %u deg: sensor error %d\n",
                   (unsigned int)drv_servo_get_angle(), (int)result);
        }
    }
    else
    {
        /* Coarse-scan readings beyond S5_DETECT_RANGE_CM are ignored. */
        uint16_t seen_mm = distance_mm;

        if ((position < S5_SCAN_POSITIONS) && (NO_OBSTACLE != seen_mm) &&
            (seen_mm > (S5_DETECT_RANGE_CM * S5_MM_PER_CM)))
        {
            seen_mm = NO_OBSTACLE;
        }

        b_good_reading = true;
        s5_report_sensor_ok();
        if (position < S5_SCAN_POSITIONS)
        {
            s5_report_distance(position, seen_mm);
        }
        if ((FORWARD_POSITION == position) &&
            (S5_FORWARD_DEG == drv_servo_get_angle()))
        {
            s5_guard_update(s5_report_mm_to_cm(seen_mm));
        }
        if (b_print)
        {
            s5_print_reading(seen_mm);
        }
    }

    if (NULL != p_distance_mm)
    {
        *p_distance_mm = distance_mm;
    }

    return b_good_reading;
}

/* Wait duration_ms, pinging every S5_PING_INTERVAL_MS meanwhile. */
static void s5_watch_for (const uint8_t position, const uint32_t duration_ms)
{
    uint32_t waited_ms = 0u;

    while (waited_ms < duration_ms)
    {
        (void)tk_dly_tsk(S5_PING_INTERVAL_MS);
        waited_ms += S5_PING_INTERVAL_MS;
        (void)s5_take_reading(position, false, NULL);
    }
}

/* One SERVO_STEP_DEGREES step from angle_now toward target_degrees,
 * without overshooting it. */
static uint16_t s5_step_toward (const uint16_t angle_now,
                                const uint16_t target_degrees)
{
    uint16_t next_angle = target_degrees;

    /* Each cast below is safe: the left side of the && guarantees the
     * difference is positive, and the result lies between the two angles
     * (0..180), which fits in 16 bits. */
    if ((angle_now < target_degrees) &&
        ((uint16_t)(target_degrees - angle_now) > SERVO_STEP_DEGREES))
    {
        next_angle = (uint16_t)(angle_now + SERVO_STEP_DEGREES);
    }
    else if ((angle_now > target_degrees) &&
             ((uint16_t)(angle_now - target_degrees) > SERVO_STEP_DEGREES))
    {
        next_angle = (uint16_t)(angle_now - SERVO_STEP_DEGREES);
    }
    else
    {
        /* Within one step of the target (or on it): go straight there. */
    }

    return next_angle;
}

/*
 * Turn the servo to target_degrees gently, one step at a time, instead of
 * jumping there at full speed (which jerks the sensor around). Keeps
 * pinging on the way (readings handled as for 'position'), then lets the
 * servo settle. Returns E_OK, or the servo driver's error.
 */
static ER s5_servo_move_slowly (const uint16_t target_degrees,
                                const uint8_t position)
{
    uint16_t angle_now        = drv_servo_get_angle();
    uint8_t  steps_since_ping = 0u;
    ER       result           = E_OK;

    while ((result >= E_OK) && (angle_now != target_degrees))
    {
        angle_now = s5_step_toward(angle_now, target_degrees);
        result    = drv_servo_set_angle(angle_now);
        (void)tk_dly_tsk(SERVO_STEP_DELAY_MS);

        steps_since_ping++;
        if (steps_since_ping >= STEPS_PER_PING)
        {
            steps_since_ping = 0u;
            (void)s5_take_reading(position, false, NULL);
        }
    }

    if (result >= E_OK)
    {
        s5_watch_for(position, SERVO_SETTLE_MS);   /* stop wobbling */
    }

    return result;
}

/* Move to target_degrees, retrying until the servo gets there. Keeps
 * pinging, and reports the failure, while it retries. */
static void s5_servo_move_until_done (const uint16_t target_degrees,
                                      const uint8_t position)
{
    ER result = s5_servo_move_slowly(target_degrees, position);

    while (result < E_OK)
    {
        S5_LOG((UB *)"S5 servo move failed: %d\n", (int)result);
        s5_report_sensor_error();
        s5_watch_for(position, PAUSE_AT_EACH_ANGLE_MS);
        result = s5_servo_move_slowly(target_degrees, position);
    }
}

/*
 * One fine-scan reading at the servo's current angle, retried up to
 * S5_FINE_READ_TRIES times. On success *p_distance_cm gets the distance in
 * cm (S5_OBSTACLE_NONE_CM if nothing echoed back) and true is returned.
 */
static bool s5_fine_reading (uint16_t * const p_distance_cm)
{
    uint16_t samples[S5_FINE_SAMPLES];
    uint8_t  count    = 0u;
    uint8_t  attempts = 0u;
    bool     b_good;

    /* Up to S5_FINE_SAMPLES good pings, S5_PING_INTERVAL_MS apart; a few
     * extra attempts are allowed for pings that fail. */
    while ((count < S5_FINE_SAMPLES) &&
           (attempts < (S5_FINE_SAMPLES + S5_FINE_READ_TRIES)))
    {
        uint16_t distance_mm = NO_OBSTACLE;

        if (attempts > 0u)
        {
            (void)tk_dly_tsk(S5_PING_INTERVAL_MS);
        }
        attempts++;

        if (s5_take_reading(FINE_SCAN_POSITION, false, &distance_mm))
        {
            samples[count] = distance_mm;
            count++;
        }
    }

    b_good = (count > 0u);
    if (b_good)
    {
        uint16_t median_mm = s5_filter_median(samples, count);

        s5_print_reading(median_mm);     /* one line per angle: the median */
        *p_distance_cm = s5_report_mm_to_cm(median_mm);
    }

    return b_good;
}

/*
 * Diagnostic (S5_SCAN_DEBUG): print one fine-scan reading with the side of
 * the car the code takes it to be. b_edge = true marks the reading where the
 * edge was found (the obstacle ended).
 */
static void s5_debug_reading (const uint16_t angle_deg, const bool b_good,
                              const uint16_t distance_cm, const bool b_edge)
{
    if (0u != S5_SCAN_DEBUG)
    {
        const bool         b_high = (angle_deg > S5_FORWARD_DEG);
        const bool         b_left = (b_high == (0u != S5_HIGH_ANGLE_IS_LEFT));
        const char * const p_side = (S5_FORWARD_DEG == angle_deg) ? "ahead" :
                                    (b_left ? "left" : "right");
        const char * const p_mark = b_edge ? "  <- edge" : "";

        if (!b_good)
        {
            tm_printf((UB *)"S5 scan %u deg (%s): reading failed\n",
                      (unsigned int)angle_deg, p_side);
        }
        else if (S5_OBSTACLE_NONE_CM == distance_cm)
        {
            tm_printf((UB *)"S5 scan %u deg (%s): no echo%s\n",
                      (unsigned int)angle_deg, p_side, p_mark);
        }
        else
        {
            tm_printf((UB *)"S5 scan %u deg (%s): %u cm%s\n",
                      (unsigned int)angle_deg, p_side,
                      (unsigned int)distance_cm, p_mark);
        }
    }
}

/*
 * Fine-scan one side, stepping outward from 90 degrees until the reading
 * jumps off the obstacle (its edge: s5_profile_edge_found()) or the scan
 * limit is reached. Past the edge the obstacle is not there any more, so
 * the rest of that side is not scanned.
 * Returns false if a reading failed (the profile would be incomplete).
 */
static bool s5_fine_scan_side (const bool b_high_side)
{
    const uint8_t max_steps  = b_high_side ? FINE_HIGH_STEPS : FINE_LOW_STEPS;
    bool          b_all_read = true;

    for (uint8_t step = 1u;
         (step <= max_steps) && (!s5_profile_edge_found(b_high_side));
         step++)
    {
        /* Casts are safe: step <= max_steps keeps the angle inside
         * S5_FINE_MIN_ANGLE..S5_FINE_MAX_ANGLE (0..180). */
        uint16_t offset_deg  = (uint16_t)(step * S5_FINE_STEP_DEG);
        uint16_t angle_deg   = b_high_side ?
                               (uint16_t)(S5_FORWARD_DEG + offset_deg) :
                               (uint16_t)(S5_FORWARD_DEG - offset_deg);
        uint16_t distance_cm = S5_OBSTACLE_NONE_CM;

        s5_servo_move_until_done(angle_deg, FINE_SCAN_POSITION);

        if (s5_fine_reading(&distance_cm))
        {
            s5_profile_add(angle_deg, distance_cm);
            s5_debug_reading(angle_deg, true, distance_cm,
                             s5_profile_edge_found(b_high_side));
        }
        else
        {
            b_all_read = false;
            s5_debug_reading(angle_deg, false, distance_cm, false);
        }
    }

    return b_all_read;
}

/*
 * Stages 2 to 4: fine scan the obstacle ahead, publish its profile into
 * *p_profile and decide how to get past it. The car is held still the
 * whole time, because the sensor is not looking at the path while it scans
 * sideways. The path counts as blocked while something is still within
 * S5_STILL_BLOCKED_CM straight ahead (also after reversing a little).
 */
static s5_bypass_t s5_fine_scan (s5_profile_t * const p_profile)
{
    uint16_t center_cm = S5_OBSTACLE_NONE_CM;
    uint16_t ahead_cm  = S5_OBSTACLE_NONE_CM;
    bool     b_scan_ok = false;

    s5_guard_set_hold(true);
    S5_LOG((UB *)"S5 fine scan: profiling the obstacle\n");
    s5_log_event("S5_FINE_SCAN", "START");

    /* Let the car come to rest (Subsystem 2 ramps down), still pinging. */
    s5_watch_for(FORWARD_POSITION, S5_FINE_START_DELAY_MS);

    if (s5_fine_reading(&center_cm))
    {
        s5_debug_reading(S5_FORWARD_DEG, true, center_cm, false);
        s5_profile_start(center_cm);

        /* Both sides always run, so the servo always sweeps both ways. */
        bool b_low_ok  = s5_fine_scan_side(false);
        bool b_high_ok = s5_fine_scan_side(true);

        b_scan_ok = (b_low_ok && b_high_ok);
    }

    s5_servo_move_until_done(S5_FORWARD_DEG, FORWARD_POSITION);
    s5_profile_finish(b_scan_ok);
    s5_log_event("S5_FINE_SCAN", b_scan_ok ? "DONE" : "FAILED");

    /* Is the obstacle still in the way? A failed reading counts as yes, and
     * so does no echo at all: the car has just stopped for something within
     * S5_STOP_TRIGGER_CM, and from that close the HC-SR04 often gets no
     * echo, so "nothing seen" means "too close to measure", not "gone".
     * Only a real echo further away than S5_STILL_BLOCKED_CM means the
     * obstacle has moved. */
    bool b_blocked = ((!s5_fine_reading(&ahead_cm)) ||
                      (S5_OBSTACLE_NONE_CM == ahead_cm) ||
                      (ahead_cm <= S5_STILL_BLOCKED_CM));

    (void)s5_profile_get(p_profile);
    s5_bypass_t decision = s5_avoid_plan(p_profile, b_blocked);

    s5_guard_set_hold(false);
    return decision;
}

/*
 * Drive one move and wait for it to finish, checking the encoders every
 * DRIVE_POLL_MS and the path ahead every S5_PING_INTERVAL_MS. A forward
 * move stops early if something comes within the stop distance.
 * Every move gives up after S5_DRIVE_MOVE_TIMEOUT_MS, and stops at once if
 * the vehicle controller withdraws the drive permission. Returns true if
 * the move was completed.
 */
static bool s5_drive_move (const s5_move_t move, const uint16_t amount)
{
    /* During a bypass the servo may be held on the obstacle: its readings
     * then only count toward sensor health, not as "straight ahead". */
    const uint8_t position   = (S5_FORWARD_DEG == drv_servo_get_angle()) ?
                               FORWARD_POSITION : FINE_SCAN_POSITION;
    uint32_t      elapsed_ms = 0u;
    bool          b_blocked  = false;

    s5_drive_start(move, amount);

    while ((!s5_drive_is_done()) && (!b_blocked) &&
           (elapsed_ms < S5_DRIVE_MOVE_TIMEOUT_MS) && s5_drive_permitted())
    {
        (void)tk_dly_tsk(DRIVE_POLL_MS);
        elapsed_ms += DRIVE_POLL_MS;
        s5_drive_nudge(elapsed_ms);   /* raise a turn that has not started */

        if (0u == (elapsed_ms % S5_PING_INTERVAL_MS))
        {
            (void)s5_take_reading(position, false, NULL);
            b_blocked = ((S5_MOVE_FORWARD == move) && s5_guard_is_stopped());
        }
    }

    bool b_done = s5_drive_is_done();

    s5_drive_print_result(move, amount, b_done, elapsed_ms);   /* DIAGNOSTIC */
    s5_drive_stop();
    s5_watch_for(position, S5_DRIVE_SETTLE_MS);

    if (!s5_drive_permitted())
    {
        b_done = false;
        S5_LOG((UB *)"S5 drive: permission withdrawn, move stopped\n");
    }
    else if (b_blocked)
    {
        S5_LOG((UB *)"S5 drive: something ahead, move stopped\n");
    }
    else if (!b_done)
    {
        S5_LOG((UB *)"S5 drive: move timed out (wheels or encoders?)\n");
    }
    else
    {
        /* Move completed. */
    }

    return (b_done && (!b_blocked));
}

/*
 * Drive straight along the obstacle, the sensor turned to look at it
 * (look_deg), until it has ended, then drive extra_mm further.  The sensor
 * keeps watching the whole time: if the obstacle shows up beside the car
 * again during the extra distance, it has not really ended, so the car
 * keeps driving and the extra distance starts again from its new end.  If
 * the obstacle is still ahead of the car (b_wait_for_obstacle), the car
 * first drives until it comes into view beside it.  Returns true
 * if the end was found and the extra distance driven; p_driven_mm receives
 * how far the car went in total.  The servo goes back to look ahead after
 * only if b_face_ahead_after; otherwise it stays on the obstacle.
 */
static bool s5_drive_along (const uint16_t look_deg, const uint16_t extra_mm,
                            const bool b_wait_for_obstacle,
                            const bool b_face_ahead_after,
                            uint16_t * const p_driven_mm)
{
    uint32_t elapsed_ms   = 0u;
    bool     b_edge_found = false;

    s5_servo_move_until_done(look_deg, FINE_SCAN_POSITION);
    s5_edge_start(b_wait_for_obstacle,
                  s5_drive_ticks_for_distance(S5_EDGE_WAIT_MM));
    s5_drive_start(S5_MOVE_FORWARD, S5_EDGE_MAX_MM);

    while ((!s5_drive_is_done()) && (elapsed_ms < S5_DRIVE_MOVE_TIMEOUT_MS) &&
           s5_drive_permitted())
    {
        (void)tk_dly_tsk(DRIVE_POLL_MS);
        elapsed_ms += DRIVE_POLL_MS;

        if (0u == (elapsed_ms % S5_PING_INTERVAL_MS))
        {
            uint16_t distance_mm = NO_OBSTACLE;
            bool     b_good      = s5_take_reading(FINE_SCAN_POSITION, false,
                                                   &distance_mm);
            bool     b_ended     = s5_edge_feed(b_good,
                                                s5_report_mm_to_cm(distance_mm),
                                                s5_drive_progress_ticks());

            if (b_ended && (!b_edge_found))
            {
                b_edge_found = true;
                if (!s5_edge_was_seen())
                {
                    S5_LOG((UB *)"S5 along: obstacle NOT seen beside the "
                           "car, going by distance only (servo looking "
                           "the wrong way, or obstacle out of range?)\n");
                    s5_log_event("S5_ALONG", "NOT_SEEN");
                }
                s5_drive_retarget(s5_edge_last_seen_ticks() +
                                  s5_drive_ticks_for_distance(extra_mm));
                S5_LOG((UB *)"S5 along: obstacle ends after %u mm, "
                       "driving %u mm more\n",
                       (unsigned int)s5_drive_mm_for_ticks(
                           s5_edge_last_seen_ticks()),
                       (unsigned int)extra_mm);
            }
            else if ((!b_ended) && b_edge_found)
            {
                /* Seen beside the car again: not past it yet. Keep going
                 * (up to S5_EDGE_MAX_MM) until it ends again. */
                b_edge_found = false;
                s5_drive_retarget(s5_drive_ticks_for_distance(S5_EDGE_MAX_MM));
                S5_LOG((UB *)"S5 along: obstacle still beside the car "
                       "at %u mm, carrying on\n",
                       (unsigned int)s5_drive_mm_for_ticks(
                           s5_drive_progress_ticks()));
            }
            else
            {
                /* No change: still beside it, or still past it. */
            }
        }
    }

    bool b_done = s5_drive_is_done();

    s5_drive_stop();
    s5_watch_for(FINE_SCAN_POSITION, S5_DRIVE_SETTLE_MS);
    *p_driven_mm = s5_drive_mm_for_ticks(s5_drive_progress_ticks());
    if (b_face_ahead_after)
    {
        s5_servo_move_until_done(S5_FORWARD_DEG, FORWARD_POSITION);
    }

    if (!s5_drive_permitted())
    {
        b_done = false;
        S5_LOG((UB *)"S5 drive: permission withdrawn, move stopped\n");
    }
    else if (!b_edge_found)
    {
        b_done = false;
        S5_LOG((UB *)"S5 along: obstacle did not end within %u mm\n",
               (unsigned int)S5_EDGE_MAX_MM);
    }
    else if (!b_done)
    {
        S5_LOG((UB *)"S5 drive: move timed out (wheels or encoders?)\n");
    }
    else
    {
        S5_LOG((UB *)"S5 along: drove %u mm\n", (unsigned int)*p_driven_mm);
    }

    return (b_done && b_edge_found);
}

/*
 * Go round the obstacle on the chosen side.  The width is NOT taken from the
 * fine scan (see S5_EDGE_*): the car feels its way along the obstacle with
 * the sensor turned to face it (look_deg) the whole time it is beside it.
 *   1. back off, turn out;
 *   2. drive sideways along the obstacle until the sensor no longer sees
 *      it, then just enough more to clear the car's width (clear_mm);
 *   3. turn back to the original heading;
 *   4. drive along the obstacle until the sensor no longer sees it, then
 *      clear_mm more, so the car's side clears the obstacle's back corner;
 *   5. turn in, drive back the sideways distance of step 2 onto the line,
 *      turn to the original heading.
 * clear_mm = sensor-to-axle + half the car's width + margin: the sensor is
 * ahead of the axle the car turns about, and after turning the car's half
 * width must be past the obstacle's edge.
 */
static void s5_drive_bypass (const s5_bypass_t side,
                             const s5_profile_t * const p_profile)
{
    const bool      b_left   = (S5_BYPASS_LEFT == side);
    const s5_move_t turn_out = b_left ? S5_MOVE_TURN_LEFT :
                                        S5_MOVE_TURN_RIGHT;
    const s5_move_t turn_in  = b_left ? S5_MOVE_TURN_RIGHT :
                                        S5_MOVE_TURN_LEFT;
    /* The obstacle is on the car's right after a left turn out, and the
     * other way round; the servo's high angles look to the right. */
    const bool      b_high_is_right = (0u == S5_HIGH_ANGLE_IS_LEFT);
    const uint16_t  look_deg = (b_left == b_high_is_right) ?
                               S5_EDGE_LOOK_HIGH_DEG : S5_EDGE_LOOK_LOW_DEG;
    /* Cast is safe: at most (255 + 255 + 255) x 10 = 7650 mm. */
    const uint16_t  clear_mm = (uint16_t)((S5_SENSOR_TO_AXLE_CM +
                                           HALF_CAR_CM +
                                           S5_BYPASS_CLEAR_MARGIN_CM) *
                                          S5_MM_PER_CM);
    uint16_t        out_mm  = 0u;
    uint16_t        pass_mm = 0u;
    bool            b_ok;

    (void)p_profile;     /* the front scan no longer sets the distances */
    S5_LOG((UB *)"S5 drive: going round on the %s, feeling along the "
           "obstacle\n", b_left ? "left" : "right");
    s5_guard_set_drive(true);

    /* The servo faces ahead while backing off, then stays on the obstacle
     * from the turn out until the car has passed it (no swinging back to
     * straight ahead in between), then faces ahead again for the way back
     * to the line. */
    b_ok = s5_drive_move(S5_MOVE_BACK, S5_BYPASS_BACKUP_MM);
    if (b_ok)
    {
        s5_servo_move_until_done(look_deg, FINE_SCAN_POSITION);
        b_ok = s5_drive_move(turn_out, TURN_DEG);
    }
    if (b_ok)
    {
        b_ok = s5_drive_along(look_deg, clear_mm, false, false, &out_mm);
    }
    if (b_ok)
    {
        b_ok = s5_drive_move(turn_in, TURN_DEG);
    }
    if (b_ok)
    {
        b_ok = s5_drive_along(look_deg, clear_mm, true, true, &pass_mm);
    }
    if (b_ok)
    {
        b_ok = s5_drive_move(turn_in, TURN_DEG);
    }
    if (b_ok)
    {
        b_ok = s5_drive_move(S5_MOVE_FORWARD, out_mm);
    }
    if (b_ok)
    {
        b_ok = s5_drive_move(turn_out, TURN_DEG);
    }

    /* Always face ahead again (also after an aborted bypass): the obstacle
     * guard only uses readings taken straight ahead. */
    s5_servo_move_until_done(S5_FORWARD_DEG, FORWARD_POSITION);
    s5_guard_set_drive(false);

    if (b_ok)
    {
        S5_LOG((UB *)"S5 drive: bypass done, back on the original path\n");
        s5_log_event("S5_DRIVE", "DONE");
        s5_drive_resume();
    }
    else
    {
        S5_LOG((UB *)"S5 drive: bypass stopped part-way, car stays\n");
        s5_log_event("S5_DRIVE", "ABORTED");
    }
}

/*
 * The car has stopped for an obstacle: fine scan it and decide. With
 * S5_DRIVE_BYPASS, also carry the decision out: reverse and scan again
 * while the decision is REVERSE, then go round on the chosen side.
 */
static void s5_handle_obstacle (void)
{
    s5_profile_t profile    = { 0u };
    s5_bypass_t  decision   = S5_BYPASS_NONE;
    bool         b_reversed = false;

    /* Tell the vehicle controller first, so it pauses the line follower
     * and grants the drive permission while the fine scan runs. */
    __atomic_store_n(&gb_avoiding, true, __ATOMIC_RELEASE);
    S5_LOG((UB *)"S5 avoiding: obstacle ahead, taking over\n");

    /* Back off before scanning, so the obstacle's edges can be told apart
     * (S5_SCAN_BACKUP_MM). Wait first: the car comes to rest and the
     * vehicle controller grants the drive permission. */
    if ((0u != S5_DRIVE_BYPASS) && (0u != S5_SCAN_BACKUP_MM))
    {
        s5_watch_for(FORWARD_POSITION, S5_FINE_START_DELAY_MS);
        if (s5_drive_permitted())
        {
            S5_LOG((UB *)"S5 scan: backing %u mm to scan from further "
                   "away\n", (unsigned int)S5_SCAN_BACKUP_MM);
            s5_guard_set_drive(true);
            (void)s5_drive_move(S5_MOVE_BACK, S5_SCAN_BACKUP_MM);
            s5_guard_set_drive(false);
        }
    }

    decision = s5_fine_scan(&profile);

    if (0u != S5_CLEARANCE_TEST)
    {
        /* Clearance and decision have been printed: hold the car still
         * for good (nothing else sets the speed cap after this). */
        s5_guard_set_hold(true);
        g_scanner.b_test_done = 1u;
    }
    else if (0u != S5_DRIVE_BYPASS)
    {
        while ((S5_BYPASS_REVERSE == decision) && s5_drive_permitted())
        {
            S5_LOG((UB *)"S5 reverse: backing %u mm to scan again\n",
                   (unsigned int)S5_REVERSE_MM);
            s5_guard_set_drive(true);
            (void)s5_drive_move(S5_MOVE_BACK, S5_REVERSE_MM);
            s5_guard_set_drive(false);
            b_reversed = true;
            decision   = s5_fine_scan(&profile);
        }

        if (!s5_drive_permitted())
        {
            /* Not following the line (or stopped by the operator): decide
             * only, never move the car on our own. */
            S5_LOG((UB *)"S5 avoiding: no drive permission "
                   "(car not following), decision only\n");
        }
        else if ((S5_BYPASS_LEFT == decision) ||
                 (S5_BYPASS_RIGHT == decision))
        {
            s5_drive_bypass(decision, &profile);
        }
        else if ((S5_BYPASS_CONTINUE == decision) && b_reversed)
        {
            S5_LOG((UB *)"S5 drive: path clear after reversing\n");
            s5_drive_resume();
        }
        else
        {
            /* STOP, or CONTINUE without reversing: nothing to drive. */
        }
    }

    /* Face the front of the car (90 degrees) before handing back, whatever
     * happened above, so the line follower always starts with the sensor
     * watching the path ahead. */
    s5_servo_move_until_done(S5_FORWARD_DEG, FORWARD_POSITION);

    /* Done: the vehicle controller hands the car back to the line follower
     * (while the car is still stopped for the obstacle, it stays avoiding,
     * see subsystem5_is_avoiding). After the clearance test the car is not
     * handed back: it stays avoiding, so the line follower stays paused. */
    if (0u == g_scanner.b_test_done)
    {
        __atomic_store_n(&gb_avoiding, false, __ATOMIC_RELEASE);
        S5_LOG((UB *)"S5 avoiding: done, handing back\n");
    }
}

/*
 * One pass of normal running, every S5_PING_INTERVAL_MS: ping straight
 * ahead, print a reading now and then, and handle each obstacle the car
 * stops for. *p_since_print_ms counts the time since the last print.
 */
static void s5_watch_ahead (uint32_t * const p_since_print_ms)
{
    *p_since_print_ms += S5_PING_INTERVAL_MS;

    bool b_print = (*p_since_print_ms >= S5_FORWARD_PRINT_MS);

    if (b_print)
    {
        *p_since_print_ms = 0u;
    }

    /* Normal running: the sensor must face the front of the car. Put it
     * back if anything left it turned (the guard ignores readings that
     * are not taken at exactly 90 degrees). */
    if (S5_FORWARD_DEG != drv_servo_get_angle())
    {
        S5_LOG((UB *)"S5 servo at %u deg, turning back to %u\n",
               (unsigned int)drv_servo_get_angle(),
               (unsigned int)S5_FORWARD_DEG);
        s5_servo_move_until_done(S5_FORWARD_DEG, FORWARD_POSITION);
    }
    (void)s5_take_reading(FORWARD_POSITION, b_print, NULL);

    if (!s5_guard_is_stopped())
    {
        if (0u != g_scanner.b_profile_done)
        {
            s5_avoid_new_obstacle();   /* fresh reverse count */
        }
        g_scanner.b_profile_done = 0u;   /* re-arm for the next one */
    }
    else if (0u == g_scanner.b_profile_done)
    {
        s5_handle_obstacle();
        g_scanner.b_profile_done = 1u;
    }
    else
    {
        /* Still stopped for an obstacle that is already profiled. */
    }
}

/*
 * The scanning task.
 *   1. Turn the servo to face the front of the car, with the car held still
 *      (subsystem5_start() has already set the hold).
 *   2. Ready: release the hold. From here only the path ahead limits the
 *      car.
 *   3. Forever: ping straight ahead, print a reading now and then, and
 *      handle each obstacle the car stops for.
 */
static void s5_sensor_task (INT start_code, void * p_extra_info)
{
    uint32_t since_print_ms = 0u;

    (void)start_code;     /* not used */
    (void)p_extra_info;   /* not used */

    /* Face the front of the car; then Subsystem 5 is ready. */
    s5_servo_move_until_done(S5_FORWARD_DEG, FORWARD_POSITION);
    s5_guard_set_hold(false);
    __atomic_store_n(&gb_facing_front, true, __ATOMIC_RELEASE);
    S5_LOG((UB *)"S5 start-up: sensor facing front, ready\n");
    s5_log_event("S5_STARTUP", "READY");

    for (;;)
    {
        (void)tk_dly_tsk(S5_PING_INTERVAL_MS);

        /* After the clearance test the task only idles: the car stays held
         * and no further obstacle is handled. */
        if (0u == g_scanner.b_test_done)
        {
            s5_watch_ahead(&since_print_ms);
        }
    }
}

EXPORT bool subsystem5_startup_scan_ready (void)
{
    return __atomic_load_n(&gb_facing_front, __ATOMIC_ACQUIRE);
}

EXPORT bool subsystem5_is_avoiding (void)
{
    return (__atomic_load_n(&gb_avoiding, __ATOMIC_ACQUIRE) ||
            (__atomic_load_n(&gb_facing_front, __ATOMIC_ACQUIRE) &&
             s5_guard_is_stopped()));
}

EXPORT void subsystem5_set_drive_permission (const bool b_allowed)
{
    __atomic_store_n(&gb_drive_permitted, b_allowed, __ATOMIC_RELEASE);
}

EXPORT ER subsystem5_init (void)
{
    ER result = drv_servo_init();   /* also turns the servo to 90 degrees */

    __atomic_store_n(&gb_facing_front, false, __ATOMIC_RELAXED);
    __atomic_store_n(&gb_avoiding, false, __ATOMIC_RELAXED);
    __atomic_store_n(&gb_drive_permitted, false, __ATOMIC_RELAXED);

    if (result < E_OK)
    {
        S5_LOG((UB *)"S5 servo init failed: %d\n", (int)result);
    }
    else
    {
        result = drv_ultrasonic_init();
        if (result < E_OK)
        {
            S5_LOG((UB *)"S5 ultrasonic init failed: %d\n", (int)result);
        }
        else
        {
            g_scanner.b_hardware_ready = 1u;
        }
    }

    return result;
}

EXPORT ER subsystem5_start (void)
{
    ER result = E_OBJ;

    if (0u == g_scanner.b_hardware_ready)
    {
        /* Without the scanner the car cannot see obstacles, so tell
         * Subsystem 1 (the car then stays paused on FAULT_US_SENS). */
        s5_report_scanner_failed();
    }
    else
    {
        /* Hold the car still until the servo faces the front of the car
         * (Subsystem 2 otherwise starts with no speed limit). */
        s5_guard_set_hold(true);

        /* (FP) is the generic task-entry type app_start_task() expects;
         * the task really has the T-Kernel (INT, void *) signature. */
        ID task_id = app_start_task((FP)s5_sensor_task,
                                    SCANNER_TASK_PRIORITY, 0u, "s5_sensor");

        /* ID and ER are both INT, so the cast cannot change the value. */
        result = (task_id < E_OK) ? (ER)task_id : E_OK;
    }

    return result;
}
