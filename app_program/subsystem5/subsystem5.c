/**
 * @file    subsystem5.c
 * @brief   Subsystem 5: servo-mounted ultrasonic obstacle scanner.
 *
 * A task turns the servo through 5 angles, back and forth forever:
 *     30 -> 60 -> 90 -> 120 -> 150 -> 120 -> 90 -> 60 -> 30 -> ...
 * At each angle it measures the distance and prints it, e.g.
 *     "S5 scan 90 deg: 342 mm"   or   "S5 scan 90 deg: no obstacle"
 * 90 degrees is straight ahead; below 90 is one side, above 90 the other.
 */
#include <tk/tkernel.h>
#include <stdbool.h>
#include "../app.h"
#include "subsystem5.h"
#include "drv_servo.h"
#include "drv_ultrasonic.h"

/* The angles the scanner stops at, in order. */
static const uint16_t scan_angles[] = { 30u, 60u, 90u, 120u, 150u };

#define NUMBER_OF_ANGLES   (sizeof(scan_angles) / sizeof(scan_angles[0]))
#define START_POSITION     (2u)   /* index of 90 degrees: start facing ahead */

/* Set to true only when both the servo and the sensor started correctly. */
static bool hardware_ready = false;

/*
 * Turn the servo to target_degrees gently, SERVO_STEP_DEGREES at a time,
 * instead of jumping there at full speed (which jerks the sensor around).
 */
static ER s5_servo_move_slowly(const uint16_t target_degrees)
{
    uint16_t angle_now = drv_servo_get_angle();
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
    }

    /* Let the servo stop wobbling before anyone takes a reading. */
    (void)tk_dly_tsk(SERVO_SETTLE_MS);
    return E_OK;
}

/*
 * The scanning task. Runs forever: move to an angle, measure, print,
 * then go to the next angle, reversing direction at either end.
 */
static void s5_sensor_task(INT start_code, void *extra_info)
{
    uint8_t  position = START_POSITION;   /* index into scan_angles[] */
    int8_t   direction = 1;               /* +1 = towards 150, -1 = towards 30 */
    uint16_t angle_degrees;
    uint16_t distance_mm;
    ER       result;

    (void)start_code;   /* not used */
    (void)extra_info;   /* not used */

    for (;;)
    {
        /* 1. Turn the sensor to the next angle. */
        angle_degrees = scan_angles[position];
        result = s5_servo_move_slowly(angle_degrees);
        if (result < E_OK)
        {
            tm_printf((UB*)"S5 servo move failed: %d\n", (int)result);
            (void)tk_dly_tsk(PAUSE_AT_EACH_ANGLE_MS);
            continue;   /* try the same angle again */
        }

        /* 2. Measure and print the distance. */
        distance_mm = drv_ultrasonic_read_distance_mm();
        if (distance_mm == NO_OBSTACLE)
        {
            tm_printf((UB*)"S5 scan %u deg: no obstacle\n",
                      (unsigned int)angle_degrees);
        }
        else
        {
            tm_printf((UB*)"S5 scan %u deg: %u mm\n",
                      (unsigned int)angle_degrees, (unsigned int)distance_mm);
        }
        (void)tk_dly_tsk(PAUSE_AT_EACH_ANGLE_MS);

        /* 3. Pick the next angle. Turn around at the last or first angle. */
        if ((direction > 0) && (position == (NUMBER_OF_ANGLES - 1u)))
        {
            direction = -1;
        }
        else if ((direction < 0) && (position == 0u))
        {
            direction = 1;
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

    hardware_ready = true;
    return E_OK;
}

EXPORT ER subsystem5_start(void)
{
    ID task_id;

    /* Don't start scanning if the hardware failed to set up. */
    if (!hardware_ready)
    {
        return E_OBJ;
    }

    task_id = app_start_task((FP)s5_sensor_task, SCANNER_TASK_PRIORITY, 0u,
                             "s5_sensor");
    return (task_id < E_OK) ? (ER)task_id : E_OK;
}
