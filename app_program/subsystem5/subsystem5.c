/**
 * @file    subsystem5.c
 * @brief   Subsystem 5: servo-mounted ultrasonic obstacle scanner.
 *
 * Stage 1, coarse start-up scan (only once, when the car powers up)
 *   The car is held still (speed cap 0) while the servo sweeps 5 angles:
 *       30 -> 60 -> 90 -> 120 -> 150
 *   90 degrees is straight ahead; below 90 is one side, above 90 the other.
 *   The servo then returns to 90. Only an obstacle straight ahead within
 *   the stop distance (S5_STOP_TRIGGER_CM) holds the car. Anything further
 *   away does not: the car moves off, slows down near it, stops
 *   S5_STOP_DISTANCE_CM in front of it, and the fine scan below profiles
 *   it. Side readings go to telemetry but never stop the car (a wall
 *   beside the track must not block it). The sweep is never repeated.
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
 *   side, out to the scan limits. The readings go to s5_profile.c, which
 *   works out the obstacle's closest point, location, width and the free
 *   gap beside it on the left and right. The servo turns back to 90
 *   degrees, and s5_avoid.c decides what to do: continue, say which
 *   side has the most clearance to go round on, or stop. The scan runs once
 *   per stop: it re-arms when the obstacle clears.
 *
 * Detection range
 *   Straight-ahead and start-up readings further than S5_DETECT_RANGE_CM
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
#include "../subsystem1/subsystem1.h"
#include "subsystem5.h"
#include "s5_config.h"
#include "drv_servo.h"
#include "drv_ultrasonic.h"
#include "s5_report.h"
#include "s5_guard.h"
#include "s5_profile.h"
#include "s5_avoid.h"

/* Index of S5_FORWARD_DEG (straight ahead) in g_scan_angles[]. */
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

_Static_assert(S5_STILL_BLOCKED_CM <= S5_MAX_RANGE_CM,
               "still-blocked distance must be within the sensor's range");
_Static_assert((S5_STOP_TRIGGER_CM < S5_DETECT_RANGE_CM) &&
               (S5_SLOW_DISTANCE_CM <= S5_DETECT_RANGE_CM) &&
               (S5_DETECT_RANGE_CM <= S5_MAX_RANGE_CM),
               "slow distance <= detect range <= sensor range");

_Static_assert((S5_FINE_MIN_ANGLE < S5_FORWARD_DEG) &&
               (S5_FINE_MAX_ANGLE > S5_FORWARD_DEG) &&
               (S5_FINE_MAX_ANGLE <= S5_SERVO_MAX_DEG),
               "fine scan must cover both sides of 90 degrees");

/* Minimum-bit scanner state: 2 bits. b_hardware_ready is set once in init
 * (before the scanning task exists); b_profile_done is used only by the
 * scanning task, so the two never race. */
typedef struct
{
    uint8_t b_hardware_ready : 1;   /* 1 = servo and sensor both started  */
    uint8_t b_profile_done   : 1;   /* 1 = current obstacle already profiled */
} scanner_state_t;

/* Start-up sweep angles, in order. 30..150 fits in 8 bits. */
static const uint8_t g_scan_angles[S5_SCAN_POSITIONS] =
{
    30u, 60u, 90u, 120u, 150u
};

static scanner_state_t g_scanner = { 0u, 0u };

/* Start-up readiness, read by other tasks (subsystem5_startup_scan_ready).
 * Written only by the scanning task (and cleared once in init, before it
 * exists); accessed with atomic loads and stores so a reader in another
 * task always sees a whole, up-to-date value. */
static bool gb_startup_sweep_done   = false;
static bool gb_forward_reading_seen = false;

/* Obstacle hand-over with the vehicle controller, same access rules:
 * gb_avoiding is written by the scanning task. */
static bool gb_avoiding        = false;

/* Set when the decision was STOP (no way round): the car stays stopped
 * (still "avoiding") until nothing is within S5_STILL_BLOCKED_CM straight
 * ahead, so the line follower does not drive it into the obstacle again.
 * Scanning task only. */
static bool gb_stay_stopped    = false;

/* Record a good straight-ahead reading for subsystem5_startup_scan_ready. */
static void s5_note_forward_reading (const bool b_good_reading)
{
    if (b_good_reading)
    {
        __atomic_store_n(&gb_forward_reading_seen, true, __ATOMIC_RELEASE);
    }
}

/* Print one good reading taken at the servo's current angle. */
static void s5_print_reading (const uint16_t distance_mm)
{
    if (NO_OBSTACLE == distance_mm)
    {
        tm_printf((UB *)"S5 scan %u deg: no obstacle\n",
                  (unsigned int)drv_servo_get_angle());
    }
    else
    {
        tm_printf((UB *)"S5 scan %u deg: %u mm\n",
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
            tm_printf((UB *)"S5 scan %u deg: sensor error %d\n",
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
        tm_printf((UB *)"S5 servo move failed: %d\n", (int)result);
        s5_report_sensor_error();
        s5_watch_for(position, PAUSE_AT_EACH_ANGLE_MS);
        result = s5_servo_move_slowly(target_degrees, position);
    }
}

/* The start-up sweep: one printed reading at each of the
 * S5_SCAN_POSITIONS angles, 30 -> 150. */
static void s5_startup_sweep (void)
{
    for (uint8_t position = 0u; position < S5_SCAN_POSITIONS; position++)
    {
        s5_servo_move_until_done(g_scan_angles[position], position);
        (void)s5_take_reading(position, true, NULL);
        s5_watch_for(position, PAUSE_AT_EACH_ANGLE_MS);
    }
}

/*
 * One fine-scan reading at the servo's current angle. The sensor may still
 * be busy from a ping that found nothing (echo held HIGH up to ~200 ms), so
 * keep trying every S5_FINE_RETRY_MS for up to S5_FINE_READ_TIMEOUT_MS. On
 * success *p_distance_cm gets the distance in cm (S5_OBSTACLE_NONE_CM if
 * nothing echoed back) and true is returned.
 */
static bool s5_fine_reading (uint16_t * const p_distance_cm)
{
    uint16_t distance_mm = NO_OBSTACLE;
    uint32_t waited_ms   = 0u;
    bool     b_good      = s5_take_reading(FINE_SCAN_POSITION, true,
                                           &distance_mm);

    while ((!b_good) && (waited_ms < S5_FINE_READ_TIMEOUT_MS))
    {
        (void)tk_dly_tsk(S5_FINE_RETRY_MS);
        waited_ms += S5_FINE_RETRY_MS;
        b_good = s5_take_reading(FINE_SCAN_POSITION, true, &distance_mm);
    }

    if (b_good)
    {
        *p_distance_cm = s5_report_mm_to_cm(distance_mm);
    }

    return b_good;
}

/*
 * Fine-scan one side, stepping outward from 90 degrees to the scan limit.
 * Angles with no reading are skipped. Returns how many were missed.
 */
static uint8_t s5_fine_scan_side (const bool b_high_side)
{
    const uint8_t max_steps = b_high_side ? FINE_HIGH_STEPS : FINE_LOW_STEPS;
    uint8_t       missed    = 0u;

    for (uint8_t step = 1u; step <= max_steps; step++)
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
        }
        else
        {
            missed++;   /* at most FINE_LOW/HIGH_STEPS: fits 8 bits */
        }
    }

    return missed;
}

/*
 * Stages 2 to 4: fine scan the obstacle ahead, publish its profile into
 * *p_profile and decide how to get past it. The car is held still the
 * whole time, because the sensor is not looking at the path while it scans
 * sideways. The path counts as blocked while something is still within
 * S5_STILL_BLOCKED_CM straight ahead.
 */
static s5_bypass_t s5_fine_scan (s5_profile_t * const p_profile)
{
    uint16_t center_cm = S5_OBSTACLE_NONE_CM;
    uint16_t ahead_cm  = S5_OBSTACLE_NONE_CM;
    bool     b_scan_ok = false;

    s5_guard_set_hold(true);
    tm_printf((UB *)"S5 fine scan: profiling the obstacle\n");
    s1_csv_log_event("S5_FINE_SCAN", "START");

    /* Let the car come to rest (Subsystem 2 ramps down), still pinging. */
    s5_watch_for(FORWARD_POSITION, S5_FINE_START_DELAY_MS);

    if (s5_fine_reading(&center_cm))
    {
        s5_profile_start(center_cm);

        /* Both sides always run, so the servo always sweeps both ways.
         * Cast is safe: at most 12 + 12 missed angles. */
        uint8_t missed = (uint8_t)(s5_fine_scan_side(false) +
                                   s5_fine_scan_side(true));

        b_scan_ok = (missed <= S5_FINE_MAX_MISSED);
        if (0u != missed)
        {
            tm_printf((UB *)"S5 fine scan: %u angles had no reading%s\n",
                      (unsigned int)missed,
                      b_scan_ok ? " (skipped)" : ", too many: no profile");
        }
    }
    else
    {
        tm_printf((UB *)"S5 fine scan: no reading straight ahead\n");
    }

    s5_servo_move_until_done(S5_FORWARD_DEG, FORWARD_POSITION);
    s5_profile_finish(b_scan_ok);
    s1_csv_log_event("S5_FINE_SCAN", b_scan_ok ? "DONE" : "FAILED");

    /* Is the obstacle still in the way? A failed reading counts as yes. */
    bool b_blocked = ((!s5_fine_reading(&ahead_cm)) ||
                      (ahead_cm <= S5_STILL_BLOCKED_CM));

    (void)s5_profile_get(p_profile);
    s5_bypass_t decision = s5_avoid_plan(p_profile, b_blocked);

    s5_guard_set_hold(false);
    return decision;
}

/*
 * The car has stopped for an obstacle: fine scan it, work out its width
 * and the clearance on each side, and decide which side to bypass on. The
 * decision is only published (s5_avoid_get()); Subsystem 5 never moves the
 * car to carry it out.
 */
static void s5_handle_obstacle (void)
{
    s5_profile_t profile  = { 0u };
    s5_bypass_t  decision = S5_BYPASS_NONE;

    /* Tell the vehicle controller first, so it pauses the line follower
     * while the fine scan runs. */
    __atomic_store_n(&gb_avoiding, true, __ATOMIC_RELEASE);
    tm_printf((UB *)"S5 avoiding: obstacle ahead, taking over\n");
    decision = s5_fine_scan(&profile);

    if (S5_BYPASS_STOP == decision)
    {
        /* Scan failed: stay stopped until the obstacle is gone. */
        __atomic_store_n(&gb_stay_stopped, true, __ATOMIC_RELEASE);
        tm_printf((UB *)"S5 avoiding: scan failed, staying stopped "
                  "until the path ahead is clear\n");
    }

    /* Done: the vehicle controller hands the car back to the line follower
     * (while the car is still stopped for the obstacle, it stays avoiding,
     * see subsystem5_is_avoiding). */
    __atomic_store_n(&gb_avoiding, false, __ATOMIC_RELEASE);
    tm_printf((UB *)"S5 avoiding: done, handing back\n");
}

/*
 * The scanning task.
 *   1. Start-up sweep, once, with the car held still (subsystem5_start()
 *      has already set the hold).
 *   2. Face straight ahead, drop the side readings from telemetry and
 *      release the hold. From here only the path ahead limits the car.
 *   3. Forever: ping straight ahead, print a reading now and then, and
 *      handle each obstacle the car stops for.
 */
static void s5_sensor_task (INT start_code, void * p_extra_info)
{
    uint32_t since_print_ms = 0u;

    (void)start_code;     /* not used */
    (void)p_extra_info;   /* not used */

    tm_printf((UB *)"S5 start-up scan: checking the path\n");
    s5_startup_sweep();

    s5_servo_move_until_done(S5_FORWARD_DEG, FORWARD_POSITION);
    s5_note_forward_reading(s5_take_reading(FORWARD_POSITION, false, NULL));
    s5_report_keep_only(FORWARD_POSITION);
    s5_guard_set_hold(false);
    __atomic_store_n(&gb_startup_sweep_done, true, __ATOMIC_RELEASE);

    if (s5_guard_is_stopped())
    {
        tm_printf((UB *)"S5 start-up scan: obstacle ahead, car waits\n");
        s1_csv_log_event("S5_STARTUP", "BLOCKED");
    }
    else
    {
        tm_printf((UB *)"S5 start-up scan: path clear, car may move off\n");
        s1_csv_log_event("S5_STARTUP", "CLEAR");
    }

    for (;;)
    {
        (void)tk_dly_tsk(S5_PING_INTERVAL_MS);
        since_print_ms += S5_PING_INTERVAL_MS;

        bool b_print = (since_print_ms >= S5_FORWARD_PRINT_MS);

        if (b_print)
        {
            since_print_ms = 0u;
        }
        uint16_t ahead_mm = NO_OBSTACLE;
        bool     b_read   = s5_take_reading(FORWARD_POSITION, b_print,
                                            &ahead_mm);

        s5_note_forward_reading(b_read);

        /* Gave up on an obstacle (STOP)? Hand back only once it is gone. */
        if (__atomic_load_n(&gb_stay_stopped, __ATOMIC_ACQUIRE) && b_read &&
            ((NO_OBSTACLE == ahead_mm) ||
             (ahead_mm > (S5_STILL_BLOCKED_CM * S5_MM_PER_CM))))
        {
            __atomic_store_n(&gb_stay_stopped, false, __ATOMIC_RELEASE);
            tm_printf((UB *)"S5 avoiding: path ahead clear, handing back\n");
        }

        if (__atomic_load_n(&gb_stay_stopped, __ATOMIC_ACQUIRE))
        {
            /* Waiting for the obstacle to go. */
        }
        else if (!s5_guard_is_stopped())
        {
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
}

EXPORT bool subsystem5_startup_scan_ready (void)
{
    return (__atomic_load_n(&gb_startup_sweep_done, __ATOMIC_ACQUIRE) &&
            __atomic_load_n(&gb_forward_reading_seen, __ATOMIC_ACQUIRE) &&
            (!s5_report_is_faulted()) && (!s5_guard_is_stopped()));
}

EXPORT bool subsystem5_is_avoiding (void)
{
    return (__atomic_load_n(&gb_avoiding, __ATOMIC_ACQUIRE) ||
            __atomic_load_n(&gb_stay_stopped, __ATOMIC_ACQUIRE) ||
            (__atomic_load_n(&gb_startup_sweep_done, __ATOMIC_ACQUIRE) &&
             s5_guard_is_stopped()));
}

EXPORT void subsystem5_set_drive_permission (const bool b_allowed)
{
    /* Subsystem 5 no longer drives the car. Kept only so the vehicle
     * controller still links; the permission is ignored. */
    (void)b_allowed;   /* not used */
}

EXPORT ER subsystem5_init (void)
{
    ER result = drv_servo_init();   /* also turns the servo to 90 degrees */

    __atomic_store_n(&gb_startup_sweep_done, false, __ATOMIC_RELAXED);
    __atomic_store_n(&gb_forward_reading_seen, false, __ATOMIC_RELAXED);
    __atomic_store_n(&gb_avoiding, false, __ATOMIC_RELAXED);
    __atomic_store_n(&gb_stay_stopped, false, __ATOMIC_RELAXED);

    if (result < E_OK)
    {
        tm_printf((UB *)"S5 servo init failed: %d\n", (int)result);
    }
    else
    {
        result = drv_ultrasonic_init();
        if (result < E_OK)
        {
            tm_printf((UB *)"S5 ultrasonic init failed: %d\n", (int)result);
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
        /* Hold the car still until the start-up scan says the path is
         * clear (Subsystem 2 otherwise starts with no speed limit). */
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
