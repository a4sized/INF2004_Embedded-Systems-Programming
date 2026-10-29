/**
 * @file    subsystem5.c
 * @brief   Subsystem 5: servo-mounted ultrasonic obstacle scanner.
 *
 * Stage 1, coarse start-up scan (once, when the car powers up)
 *   The car is held still (speed cap 0) while the servo sweeps 5 angles:
 *       30 -> 60 -> 90 -> 120 -> 150
 *   90 degrees is straight ahead; below 90 is one side, above 90 the other.
 *   If every angle gave a good reading and nothing is within the stop
 *   distance, the path is clear: the servo returns to 90 and the car may
 *   move off. Otherwise the sweep is repeated until the path is clear.
 *
 * Normal running
 *   The servo stays at 90 degrees and only the straight-ahead distance is
 *   used. A reading is printed every S5_FORWARD_PRINT_MS, e.g.
 *       "S5 scan 90 deg: 342 mm"   or   "S5 scan 90 deg: no obstacle"
 *
 * Stages 2 and 3, fine scan and obstacle profile (each time the car stops
 * for an obstacle)
 *   The car is held still while the servo steps outward from 90 degrees,
 *   S5_FINE_STEP_DEG at a time, first to the low side, then to the high
 *   side, until it passes the obstacle's edge (plus S5_CLEAR_STEPS more
 *   readings to measure the clearance) or reaches the scan limit. The
 *   readings go to s5_profile.c, which works out the obstacle's closest
 *   point, location, width and left/right clearance. Then the servo turns
 *   back to 90 degrees. The scan runs once per stop: it re-arms when the
 *   obstacle clears.
 *
 * The sensor is pinged every S5_PING_INTERVAL_MS the whole time. While it
 * faces ahead, every good reading:
 *   - goes to Subsystem 1 (s5_report.c): nearest obstacle -> telemetry, and
 *     repeated sensor/servo failures raise FAULT_US_SENS;
 *   - goes to the obstacle guard (s5_guard.c), which caps Subsystem 2's
 *     speed: slow near an obstacle, stop S5_STOP_DISTANCE_CM away from it.
 * While it looks sideways during the fine scan, readings only count toward
 * sensor health, so a clear side can never release the stop.
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
 * Take one reading and report it. For a coarse-scan 'position' the reading
 * is also stored under that position and sent to the obstacle guard; for
 * FINE_SCAN_POSITION it only counts toward sensor health.
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
        b_good_reading = true;
        s5_report_sensor_ok();
        if (position < S5_SCAN_POSITIONS)
        {
            s5_guard_update(s5_report_distance(position, distance_mm));
        }
        if (b_print)
        {
            s5_print_reading(distance_mm);
        }
    }

    if (NULL != p_distance_mm)
    {
        *p_distance_mm = distance_mm;
    }

    return b_good_reading;
}

/*
 * Wait duration_ms, pinging every S5_PING_INTERVAL_MS meanwhile.
 * Returns true if at least one of those readings was good.
 */
static bool s5_watch_for (const uint8_t position, const uint32_t duration_ms)
{
    uint32_t waited_ms  = 0u;
    bool     b_any_good = false;

    while (waited_ms < duration_ms)
    {
        (void)tk_dly_tsk(S5_PING_INTERVAL_MS);
        waited_ms += S5_PING_INTERVAL_MS;

        if (s5_take_reading(position, false, NULL))
        {
            b_any_good = true;
        }
    }

    return b_any_good;
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
        (void)s5_watch_for(position, SERVO_SETTLE_MS);   /* stop wobbling */
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
        (void)s5_watch_for(position, PAUSE_AT_EACH_ANGLE_MS);
        result = s5_servo_move_slowly(target_degrees, position);
    }
}

/*
 * One start-up sweep over all S5_SCAN_POSITIONS angles, 30 -> 150.
 * Returns true if every angle gave a good reading AND the guard is not
 * holding the car for an obstacle, i.e. the path is clear to move off.
 */
static bool s5_startup_sweep (void)
{
    bool b_all_read = true;

    for (uint8_t position = 0u; position < S5_SCAN_POSITIONS; position++)
    {
        s5_servo_move_until_done(g_scan_angles[position], position);

        /* The printed reading or any reading during the pause counts, so
         * one glitch does not throw the whole sweep away. Both calls always
         * run, so the pause always happens. */
        bool b_printed = s5_take_reading(position, true, NULL);
        bool b_watched = s5_watch_for(position, PAUSE_AT_EACH_ANGLE_MS);

        if ((!b_printed) && (!b_watched))
        {
            b_all_read = false;   /* cannot call this angle clear */
        }
    }

    return (b_all_read && (!s5_guard_is_stopped()));
}

/*
 * One fine-scan reading at the servo's current angle, retried up to
 * S5_FINE_READ_TRIES times. On success *p_distance_cm gets the distance in
 * cm (S5_OBSTACLE_NONE_CM if nothing echoed back) and true is returned.
 */
static bool s5_fine_reading (uint16_t * const p_distance_cm)
{
    uint16_t distance_mm = NO_OBSTACLE;
    bool     b_good      = s5_take_reading(FINE_SCAN_POSITION, true,
                                           &distance_mm);

    for (uint8_t tries = 1u; (!b_good) && (tries < S5_FINE_READ_TRIES);
         tries++)
    {
        (void)tk_dly_tsk(S5_PING_INTERVAL_MS);
        b_good = s5_take_reading(FINE_SCAN_POSITION, true, &distance_mm);
    }

    if (b_good)
    {
        *p_distance_cm = s5_report_mm_to_cm(distance_mm);
    }

    return b_good;
}

/*
 * Fine-scan one side, stepping outward from 90 degrees until the profile
 * has what it needs for that side or the scan limit is reached.
 * Returns false if a reading failed (the profile would be incomplete).
 */
static bool s5_fine_scan_side (const bool b_high_side)
{
    const uint8_t max_steps  = b_high_side ? FINE_HIGH_STEPS : FINE_LOW_STEPS;
    bool          b_all_read = true;

    for (uint8_t step = 1u;
         (step <= max_steps) && s5_profile_wants_more(b_high_side);
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
        }
        else
        {
            b_all_read = false;
        }
    }

    return b_all_read;
}

/*
 * Stages 2 and 3: fine scan the obstacle ahead and publish its profile.
 * The car is held still the whole time, because the sensor is not looking
 * at the path while it scans sideways.
 */
static void s5_fine_scan (void)
{
    uint16_t center_cm = S5_OBSTACLE_NONE_CM;
    bool     b_scan_ok = false;

    s5_guard_set_hold(true);
    tm_printf((UB *)"S5 fine scan: profiling the obstacle\n");
    s1_csv_log_event("S5_FINE_SCAN", "START");

    /* Let the car come to rest (Subsystem 2 ramps down), still pinging. */
    (void)s5_watch_for(FORWARD_POSITION, S5_FINE_START_DELAY_MS);

    if (s5_fine_reading(&center_cm))
    {
        s5_profile_start(center_cm);

        /* Both sides always run, so the servo always sweeps both ways. */
        bool b_low_ok  = s5_fine_scan_side(false);
        bool b_high_ok = s5_fine_scan_side(true);

        b_scan_ok = (b_low_ok && b_high_ok);
    }

    s5_servo_move_until_done(S5_FORWARD_DEG, FORWARD_POSITION);
    s5_profile_finish(b_scan_ok);
    s1_csv_log_event("S5_FINE_SCAN", b_scan_ok ? "DONE" : "FAILED");
    s5_guard_set_hold(false);
}

/*
 * The scanning task.
 *   1. Start-up scan with the car held still, repeated until the path is
 *      clear (subsystem5_start() has already set the hold).
 *   2. Face straight ahead, forget the side readings (the sensor no longer
 *      looks sideways, so an old side reading must not hold the car), and
 *      release the hold. The next reading sends the real speed cap.
 *   3. Forever: ping straight ahead, print a reading now and then, and
 *      fine scan each obstacle the car stops for.
 */
static void s5_sensor_task (INT start_code, void * p_extra_info)
{
    uint32_t since_print_ms = 0u;

    (void)start_code;     /* not used */
    (void)p_extra_info;   /* not used */

    tm_printf((UB *)"S5 start-up scan: checking the path\n");
    while (!s5_startup_sweep())
    {
        tm_printf((UB *)"S5 start-up scan: path blocked, scanning again\n");
    }

    s5_servo_move_until_done(S5_FORWARD_DEG, FORWARD_POSITION);
    s5_report_keep_only(FORWARD_POSITION);
    s5_guard_set_hold(false);
    tm_printf((UB *)"S5 start-up scan: path clear, car may move off\n");
    s1_csv_log_event("S5_STARTUP", "CLEAR");

    for (;;)
    {
        (void)tk_dly_tsk(S5_PING_INTERVAL_MS);
        since_print_ms += S5_PING_INTERVAL_MS;

        bool b_print = (since_print_ms >= S5_FORWARD_PRINT_MS);

        if (b_print)
        {
            since_print_ms = 0u;
        }
        (void)s5_take_reading(FORWARD_POSITION, b_print, NULL);

        if (!s5_guard_is_stopped())
        {
            g_scanner.b_profile_done = 0u;   /* re-arm for the next one */
        }
        else if (0u == g_scanner.b_profile_done)
        {
            s5_fine_scan();
            g_scanner.b_profile_done = 1u;
        }
        else
        {
            /* Still stopped for an obstacle that is already profiled. */
        }
    }
}

EXPORT ER subsystem5_init (void)
{
    ER result = drv_servo_init();   /* also turns the servo to 90 degrees */

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
