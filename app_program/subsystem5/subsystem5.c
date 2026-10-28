/**
 * @file    subsystem5.c
 * @brief   Subsystem 5: servo-mounted ultrasonic obstacle scanner.
 *
 * A task turns the servo through 5 angles, back and forth forever:
 *     30 -> 60 -> 90 -> 120 -> 150 -> 120 -> 90 -> 60 -> 30 -> ...
 * 90 degrees is straight ahead; below 90 is one side, above 90 the other.
 *
 * The sensor is pinged every S5_PING_INTERVAL_MS the whole time: while the
 * servo turns, while it settles, and while it pauses at each angle. One
 * reading per angle is printed, e.g.
 *     "S5 scan 90 deg: 342 mm"   or   "S5 scan 90 deg: no obstacle"
 *
 * Every good reading:
 *   - goes to Subsystem 1 (s5_report.c): nearest obstacle -> telemetry, and
 *     repeated sensor/servo failures raise FAULT_US_SENS;
 *   - goes to the obstacle guard (s5_guard.c), which caps Subsystem 2's
 *     speed: slow near an obstacle, stop S5_STOP_DISTANCE_CM away from it.
 * A failed reading is never treated as "no obstacle", so it cannot release
 * a stop.
 *
 * Subsystem 5 only limits speed (s2_set_us_clearance). Remote commands from
 * Subsystem 1 are for the vehicle controller, not this subsystem.
 */
#include <tk/tkernel.h>
#include <stdbool.h>
#include "../app.h"
#include "subsystem5.h"
#include "drv_servo.h"
#include "drv_ultrasonic.h"
#include "s5_config.h"
#include "s5_report.h"
#include "s5_guard.h"

/* The angles the scanner stops at, in order. */
/* Angles 30..150 fit in 8 bits. */
static const uint8_t g_scan_angles[S5_SCAN_POSITIONS] =
{
    30u, 60u, 90u, 120u, 150u
};

#define START_POSITION     (2u)   /* index of 90 degrees: start facing ahead */

/* Servo steps between two pings while the servo is turning (rounded up). */
#define STEPS_PER_PING     (((S5_PING_INTERVAL_MS + SERVO_STEP_DELAY_MS) - 1u) / \
                            SERVO_STEP_DELAY_MS)

/* Minimum-bit scanner state: 1 bit. Set once in init, read in start. */
typedef struct
{
    uint8_t b_hardware_ready : 1;   /* 1 = servo and sensor both started */
} scanner_state_t;

static scanner_state_t g_scanner = { 0u };

/* Index of the scan position closest to angle_degrees. */
static uint8_t s5_nearest_position(const uint16_t angle_degrees)
{
    uint8_t  best_index = 0u;
    uint16_t best_gap = 0xFFFFu;
    uint16_t gap;
    uint8_t  index;

    for (index = 0u; index < S5_SCAN_POSITIONS; index++)
    {
        if (angle_degrees >= g_scan_angles[index])
        {
            gap = (uint16_t)(angle_degrees - g_scan_angles[index]);
        }
        else
        {
            gap = (uint16_t)(g_scan_angles[index] - angle_degrees);
        }

        if (gap < best_gap)
        {
            best_gap   = gap;
            best_index = index;
        }
    }
    return best_index;
}

/*
 * Take one reading at the servo's current angle, report it, and update the
 * obstacle guard. b_print = true prints the result on the console.
 */
static void s5_take_reading(const uint16_t angle_degrees, const bool b_print)
{
    uint16_t distance_mm = NO_OBSTACLE;
    uint16_t nearest_cm;
    ER       result;

    result = drv_ultrasonic_measure(&distance_mm);

    if (E_BUSY == result)
    {
        return;   /* sensor still finishing the last ping: no reading */
    }

    if (result < E_OK)
    {
        if (b_print)
        {
            tm_printf((UB*)"S5 scan %u deg: sensor error %d\n",
                      (unsigned int)angle_degrees, (int)result);
        }
        s5_report_sensor_error();
        return;   /* keep the last good distances; never release a stop */
    }

    s5_report_sensor_ok();
    nearest_cm = s5_report_distance(s5_nearest_position(angle_degrees),
                                    distance_mm);
    s5_guard_update(nearest_cm);

    if (b_print)
    {
        if (NO_OBSTACLE == distance_mm)
        {
            tm_printf((UB*)"S5 scan %u deg: no obstacle\n",
                      (unsigned int)angle_degrees);
        }
        else
        {
            tm_printf((UB*)"S5 scan %u deg: %u mm\n",
                      (unsigned int)angle_degrees,
                      (unsigned int)distance_mm);
        }
    }
}

/* Wait duration_ms, pinging every S5_PING_INTERVAL_MS meanwhile. */
static void s5_watch_for(const uint32_t duration_ms)
{
    uint32_t waited_ms = 0u;

    while (waited_ms < duration_ms)
    {
        (void)tk_dly_tsk(S5_PING_INTERVAL_MS);
        waited_ms += S5_PING_INTERVAL_MS;
        s5_take_reading(drv_servo_get_angle(), false);
    }
}

/*
 * Turn the servo to target_degrees gently, SERVO_STEP_DEGREES at a time,
 * instead of jumping there at full speed (which jerks the sensor around).
 * Keeps pinging on the way so the obstacle guard never goes blind.
 */
static ER s5_servo_move_slowly(const uint16_t target_degrees)
{
    uint16_t angle_now = drv_servo_get_angle();
    uint8_t  steps_since_ping = 0u;
    ER       result;

    while (angle_now != target_degrees)
    {
        /* Take one step toward the target, without overshooting it. */
        if (angle_now < target_degrees)
        {
            if ((uint16_t)(target_degrees - angle_now) > SERVO_STEP_DEGREES)
            {
                angle_now = (uint16_t)(angle_now + SERVO_STEP_DEGREES);
            }
            else
            {
                angle_now = target_degrees;
            }
        }
        else
        {
            if ((uint16_t)(angle_now - target_degrees) > SERVO_STEP_DEGREES)
            {
                angle_now = (uint16_t)(angle_now - SERVO_STEP_DEGREES);
            }
            else
            {
                angle_now = target_degrees;
            }
        }

        result = drv_servo_set_angle(angle_now);
        if (result < E_OK)
        {
            return result;
        }
        (void)tk_dly_tsk(SERVO_STEP_DELAY_MS);

        steps_since_ping++;
        if (steps_since_ping >= STEPS_PER_PING)
        {
            steps_since_ping = 0u;
            s5_take_reading(angle_now, false);
        }
    }

    /* Let the servo stop wobbling before the printed reading. */
    s5_watch_for(SERVO_SETTLE_MS);
    return E_OK;
}

/*
 * The scanning task. Runs forever: move to an angle, measure, print,
 * keep watching for a while, then go to the next angle, reversing
 * direction at either end.
 */
static void s5_sensor_task(INT start_code, void *p_extra_info)
{
    uint8_t  position = START_POSITION;   /* index into g_scan_angles[] */
    int8_t   direction = 1;               /* +1 = towards 150, -1 = towards 30 */
    uint16_t angle_degrees;
    ER       result;

    (void)start_code;     /* not used */
    (void)p_extra_info;   /* not used */

    for (;;)
    {
        /* 1. Turn the sensor to the next angle. */
        angle_degrees = (uint16_t)g_scan_angles[position];
        result = s5_servo_move_slowly(angle_degrees);
        if (result < E_OK)
        {
            tm_printf((UB*)"S5 servo move failed: %d\n", (int)result);
            s5_report_sensor_error();
            s5_watch_for(PAUSE_AT_EACH_ANGLE_MS);   /* keep guarding */
            continue;   /* try the same angle again */
        }

        /* 2. Printed reading at this angle, then keep watching. */
        s5_take_reading(angle_degrees, true);
        s5_watch_for(PAUSE_AT_EACH_ANGLE_MS);

        /* 3. Pick the next angle. Turn around at the last or first angle. */
        if ((direction > 0) && ((S5_SCAN_POSITIONS - 1u) == position))
        {
            direction = -1;
        }
        else if ((direction < 0) && (0u == position))
        {
            direction = 1;
        }
        else
        {
            /* keep going the same way */
        }
        position = (uint8_t)((int8_t)position + direction);
    }
}

EXPORT ER subsystem5_init(void)
{
    ER result;

    /* Servo first: it also moves to the 90 degree (ahead) position. */
    result = drv_servo_init();
    if (result < E_OK)
    {
        tm_printf((UB*)"S5 servo init failed: %d\n", (int)result);
        return result;
    }

    result = drv_ultrasonic_init();
    if (result < E_OK)
    {
        tm_printf((UB*)"S5 ultrasonic init failed: %d\n", (int)result);
        return result;
    }

    g_scanner.b_hardware_ready = 1u;
    return E_OK;
}

EXPORT ER subsystem5_start(void)
{
    ID task_id;

    /* Don't start scanning if the hardware failed to set up. Without the
     * scanner the car cannot see obstacles, so tell Subsystem 1 (the car
     * then stays paused on FAULT_US_SENS). */
    if (0u == g_scanner.b_hardware_ready)
    {
        s5_report_scanner_failed();
        return E_OBJ;
    }

    task_id = app_start_task((FP)s5_sensor_task, SCANNER_TASK_PRIORITY, 0u,
                             "s5_sensor");
    return (task_id < E_OK) ? (ER)task_id : E_OK;
}
