/**
 * @file    subsystem5.c
 * @brief   Subsystem 5: servo-mounted ultrasonic obstacle scanner.
 *
 * Start-up scan (once, when the car powers up)
 *   The car is held still (speed cap 0) while the servo sweeps 5 angles:
 *       30 -> 60 -> 90 -> 120 -> 150
 *   90 degrees is straight ahead; below 90 is one side, above 90 the other.
 *   If every angle gave a good reading and nothing is within the stop
 *   distance, the path is clear: the servo returns to 90 and the car may
 *   move off. Otherwise the sweep is repeated until the path is clear.
 *
 * Normal running (after the start-up scan)
 *   The servo stays at 90 degrees and only the straight-ahead distance is
 *   used. A reading is printed every S5_FORWARD_PRINT_MS, e.g.
 *       "S5 scan 90 deg: 342 mm"   or   "S5 scan 90 deg: no obstacle"
 *
 * The sensor is pinged every S5_PING_INTERVAL_MS the whole time, also while
 * the servo turns and settles. Every good reading:
 *   - goes to Subsystem 1 (s5_report.c): nearest obstacle -> telemetry, and
 *     repeated sensor/servo failures raise FAULT_US_SENS;
 *   - goes to the obstacle guard (s5_guard.c), which caps Subsystem 2's
 *     speed: slow near an obstacle, stop S5_STOP_DISTANCE_CM away from it.
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

/* Index of 90 degrees (straight ahead) in g_scan_angles[]. */
#define FORWARD_POSITION    (2u)

/* Servo steps between two pings while the servo is turning (rounded up). */
#define STEPS_PER_PING      (((S5_PING_INTERVAL_MS + SERVO_STEP_DELAY_MS) - \
                              1u) / SERVO_STEP_DELAY_MS)

/* Minimum-bit scanner state: 1 bit. Set once in init, read in start. */
typedef struct
{
    uint8_t b_hardware_ready : 1;   /* 1 = servo and sensor both started */
} scanner_state_t;

/* Start-up sweep angles, in order. 30..150 fits in 8 bits. */
static const uint8_t g_scan_angles[S5_SCAN_POSITIONS] =
{
    30u, 60u, 90u, 120u, 150u
};

static scanner_state_t g_scanner = { 0u };

/* Print one good reading taken at scan position 'position'. */
static void s5_print_reading (const uint8_t position,
                              const uint16_t distance_mm)
{
    if (NO_OBSTACLE == distance_mm)
    {
        tm_printf((UB *)"S5 scan %u deg: no obstacle\n",
                  (unsigned int)g_scan_angles[position]);
    }
    else
    {
        tm_printf((UB *)"S5 scan %u deg: %u mm\n",
                  (unsigned int)g_scan_angles[position],
                  (unsigned int)distance_mm);
    }
}

/*
 * Take one reading, store it under scan position 'position', report it to
 * Subsystem 1 and update the obstacle guard. b_print = true also prints it.
 * Returns true if a good reading (a distance or "no obstacle") was taken.
 */
static bool s5_take_reading (const uint8_t position, const bool b_print)
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
                      (unsigned int)g_scan_angles[position], (int)result);
        }
    }
    else
    {
        b_good_reading = true;
        s5_report_sensor_ok();
        s5_guard_update(s5_report_distance(position, distance_mm));
        if (b_print)
        {
            s5_print_reading(position, distance_mm);
        }
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

        if (s5_take_reading(position, false))
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
 * Turn the servo to scan position 'position' gently, one step at a time,
 * instead of jumping there at full speed (which jerks the sensor around).
 * Keeps pinging on the way so the obstacle guard never goes blind, then
 * lets the servo settle. Returns E_OK, or the servo driver's error.
 */
static ER s5_servo_move_slowly (const uint8_t position)
{
    const uint16_t target_degrees   = g_scan_angles[position];
    uint16_t       angle_now        = drv_servo_get_angle();
    uint8_t        steps_since_ping = 0u;
    ER             result           = E_OK;

    while ((result >= E_OK) && (angle_now != target_degrees))
    {
        angle_now = s5_step_toward(angle_now, target_degrees);
        result    = drv_servo_set_angle(angle_now);
        (void)tk_dly_tsk(SERVO_STEP_DELAY_MS);

        steps_since_ping++;
        if (steps_since_ping >= STEPS_PER_PING)
        {
            steps_since_ping = 0u;
            (void)s5_take_reading(position, false);
        }
    }

    if (result >= E_OK)
    {
        (void)s5_watch_for(position, SERVO_SETTLE_MS);   /* stop wobbling */
    }

    return result;
}

/* Move to scan position 'position', retrying until the servo gets there.
 * Keeps guarding, and reports the failure, while it retries. */
static void s5_servo_move_until_done (const uint8_t position)
{
    ER result = s5_servo_move_slowly(position);

    while (result < E_OK)
    {
        tm_printf((UB *)"S5 servo move failed: %d\n", (int)result);
        s5_report_sensor_error();
        (void)s5_watch_for(position, PAUSE_AT_EACH_ANGLE_MS);
        result = s5_servo_move_slowly(position);
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
        s5_servo_move_until_done(position);

        /* The printed reading or any reading during the pause counts, so
         * one glitch does not throw the whole sweep away. Both calls always
         * run, so the pause always happens. */
        bool b_printed = s5_take_reading(position, true);
        bool b_watched = s5_watch_for(position, PAUSE_AT_EACH_ANGLE_MS);

        if ((!b_printed) && (!b_watched))
        {
            b_all_read = false;   /* cannot call this angle clear */
        }
    }

    return (b_all_read && (!s5_guard_is_stopped()));
}

/*
 * The scanning task.
 *   1. Start-up scan with the car held still, repeated until the path is
 *      clear (subsystem5_start() has already set the hold).
 *   2. Face straight ahead, forget the side readings (the sensor no longer
 *      looks sideways, so an old side reading must not hold the car), and
 *      release the hold. The next reading sends the real speed cap.
 *   3. Forever: watch straight ahead, printing a reading now and then.
 */
static void s5_sensor_task (INT start_code, void * p_extra_info)
{
    (void)start_code;     /* not used */
    (void)p_extra_info;   /* not used */

    tm_printf((UB *)"S5 start-up scan: checking the path\n");
    while (!s5_startup_sweep())
    {
        tm_printf((UB *)"S5 start-up scan: path blocked, scanning again\n");
    }

    s5_servo_move_until_done(FORWARD_POSITION);
    s5_report_keep_only(FORWARD_POSITION);
    s5_guard_set_startup_hold(false);
    tm_printf((UB *)"S5 start-up scan: path clear, car may move off\n");
    s1_csv_log_event("S5_STARTUP", "CLEAR");

    for (;;)
    {
        (void)s5_watch_for(FORWARD_POSITION, S5_FORWARD_PRINT_MS);
        (void)s5_take_reading(FORWARD_POSITION, true);
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
        s5_guard_set_startup_hold(true);

        /* (FP) is the generic task-entry type app_start_task() expects;
         * the task really has the T-Kernel (INT, void *) signature. */
        ID task_id = app_start_task((FP)s5_sensor_task,
                                    SCANNER_TASK_PRIORITY, 0u, "s5_sensor");

        /* ID and ER are both INT, so the cast cannot change the value. */
        result = (task_id < E_OK) ? (ER)task_id : E_OK;
    }

    return result;
}
