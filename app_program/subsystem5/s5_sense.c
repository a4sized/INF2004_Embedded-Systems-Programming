/**
 * @file    s5_sense.c
 * @brief   Subsystem 5: taking readings and turning the servo.
 *
 * See s5_sense.h. Every good reading counts toward sensor health
 * (s5_report.c: repeated sensor or servo failures raise FAULT_US_SENS).
 * Straight-ahead readings also go to telemetry, and only those taken with
 * the servo exactly at S5_FORWARD_DEG go to the obstacle guard
 * (s5_guard.c), so a side reading can neither stop the car nor release a
 * stop.
 *
 * Casts in this file: (UB *) only retypes a string literal for the T-Kernel
 * tm_printf() API; (unsigned int) and (int) widen values that always fit;
 * (void) discards a return value that is not needed.
 */
#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>
#include "../app.h"
#include "s5_config.h"
#include "s5_log.h"
#include "drv_servo.h"
#include "drv_ultrasonic.h"
#include "s5_report.h"
#include "s5_guard.h"
#include "s5_filter.h"
#include "s5_sense.h"

_Static_assert((S5_FINE_SAMPLES >= 1u) && (S5_FINE_SAMPLES <= S5_FILTER_MAX),
               "S5_FINE_SAMPLES must be between 1 and S5_FILTER_MAX");

/* Servo steps between two pings while the servo is turning (rounded up). */
#define STEPS_PER_PING      (((S5_PING_INTERVAL_MS + SERVO_STEP_DELAY_MS) - \
                              1u) / SERVO_STEP_DELAY_MS)

/* Print one good reading taken at the servo's current angle: the distance
 * line, one of the few always printed (s5_log.h), except in the clearance
 * test. */
static void s5_sense_print (const uint16_t distance_mm)
{
    if (0u != S5_CLEARANCE_TEST)
    {
        /* Clearance test: only the result and the decision. */
    }
    else if (NO_OBSTACLE == distance_mm)
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

bool s5_sense_reading (const s5_look_t look, const bool b_print,
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
        uint16_t seen_mm = distance_mm;

        b_good_reading = true;
        s5_report_sensor_ok();
        if (S5_LOOK_AHEAD == look)
        {
            /* Ahead, anything beyond S5_DETECT_RANGE_CM is ignored. */
            if ((NO_OBSTACLE != seen_mm) &&
                (seen_mm > (S5_DETECT_RANGE_CM * S5_MM_PER_CM)))
            {
                seen_mm = NO_OBSTACLE;
            }
            s5_report_distance(seen_mm);
            if (S5_FORWARD_DEG == drv_servo_get_angle())
            {
                s5_guard_update(s5_report_mm_to_cm(seen_mm));
            }
        }
        if (b_print)
        {
            s5_sense_print(seen_mm);
        }
    }

    if (NULL != p_distance_mm)
    {
        *p_distance_mm = distance_mm;
    }

    return b_good_reading;
}

void s5_sense_watch_for (const s5_look_t look, const uint32_t duration_ms)
{
    uint32_t waited_ms = 0u;

    while (waited_ms < duration_ms)
    {
        (void)tk_dly_tsk(S5_PING_INTERVAL_MS);
        waited_ms += S5_PING_INTERVAL_MS;
        (void)s5_sense_reading(look, false, NULL);
    }
}

s5_look_t s5_sense_look (void)
{
    return (S5_FORWARD_DEG == drv_servo_get_angle()) ? S5_LOOK_AHEAD :
                                                       S5_LOOK_SIDE;
}

/* One SERVO_STEP_DEGREES step from angle_now toward target_degrees,
 * without overshooting it. Each cast is safe: the left side of the &&
 * keeps the difference positive, and the result lies between the two
 * angles (0..180). */
static uint16_t s5_sense_step_toward (const uint16_t angle_now,
                                      const uint16_t target_degrees)
{
    uint16_t next_angle = target_degrees;   /* within one step: go there */

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
        /* Within one step of the target (or on it). */
    }

    return next_angle;
}

/* Turn the servo to target_degrees one step at a time (jumping there at
 * full speed jerks the sensor), pinging on the way, then let it settle.
 * Returns E_OK, or the servo driver's error. */
static ER s5_sense_servo_move (const uint16_t target_degrees,
                               const s5_look_t look)
{
    uint16_t angle_now        = drv_servo_get_angle();
    uint8_t  steps_since_ping = 0u;
    ER       result           = E_OK;

    while ((result >= E_OK) && (angle_now != target_degrees))
    {
        angle_now = s5_sense_step_toward(angle_now, target_degrees);
        result    = drv_servo_set_angle(angle_now);
        (void)tk_dly_tsk(SERVO_STEP_DELAY_MS);

        steps_since_ping++;
        if (steps_since_ping >= STEPS_PER_PING)
        {
            steps_since_ping = 0u;
            (void)s5_sense_reading(look, false, NULL);
        }
    }

    if (result >= E_OK)
    {
        s5_sense_watch_for(look, SERVO_SETTLE_MS);   /* stop wobbling */
    }

    return result;
}

void s5_sense_servo_to (const uint16_t target_degrees, const s5_look_t look)
{
    /* Never turn further than the fine scan does (S5_FINE_MIN_ANGLE ..
     * S5_FINE_MAX_ANGLE), whatever asks for it. */
    const uint16_t target = (target_degrees < S5_FINE_MIN_ANGLE) ?
                            S5_FINE_MIN_ANGLE :
                            ((target_degrees > S5_FINE_MAX_ANGLE) ?
                             S5_FINE_MAX_ANGLE : target_degrees);
    ER             result = s5_sense_servo_move(target, look);

    while (result < E_OK)
    {
        S5_LOG((UB *)"S5 servo move failed: %d\n", (int)result);
        s5_report_sensor_error();
        s5_sense_watch_for(look, SERVO_RETRY_PAUSE_MS);
        result = s5_sense_servo_move(target, look);
    }
}

bool s5_sense_fine_reading (uint16_t * const p_distance_cm)
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

        if (s5_sense_reading(S5_LOOK_SIDE, false, &distance_mm))
        {
            samples[count] = distance_mm;
            count++;
        }
    }

    b_good = (count > 0u);
    if (b_good)
    {
        /* Printed per angle by the caller (s5_scan.c), with its side. */
        *p_distance_cm = s5_report_mm_to_cm(s5_filter_median(samples, count));
    }

    return b_good;
}
