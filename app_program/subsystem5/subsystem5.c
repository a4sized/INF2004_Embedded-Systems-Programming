/**
 * @file    subsystem5.c
 * @brief   Subsystem 5 ultrasonic sensor initialization.
 */
#include <tk/tkernel.h>
#include <stdbool.h>
#include "../app.h"
#include "subsystem5.h"
#include "drv_servo.h"

static bool s_initialized;

/* Scan angles, swept back and forth: 30..150, then 120..60, and repeat. */
static const uint16_t k_scan_deg[S5_SCAN_ANGLE_COUNT] =
{
    30u, 60u, 90u, 120u, 150u
};

/* Turn the servo to target_deg slowly, S5_SERVO_STEP_DEG at a time. */
static ER s5_servo_move_slowly(const uint16_t target_deg)
{
    uint16_t current = drv_servo_get_angle();
    ER       er = E_OK;

    while ((current != target_deg) && (er >= E_OK))
    {
        if (current < target_deg)
        {
            current = (uint16_t)(current + S5_SERVO_STEP_DEG);
            if (current > target_deg)
            {
                current = target_deg;
            }
        }
        else if ((current - target_deg) > S5_SERVO_STEP_DEG)
        {
            current = (uint16_t)(current - S5_SERVO_STEP_DEG);
        }
        else
        {
            current = target_deg;
        }

        er = drv_servo_set_angle(current);
        (void)tk_dly_tsk(S5_SERVO_STEP_MS);
    }

    (void)tk_dly_tsk(S5_SERVO_FINAL_SETTLE_MS);
    return er;
}

#if S5_WIGGLE_DEMO
/* Offsets from centre in steps: left (+) is the higher angle. */
static const int8_t k_wiggle_steps[8] =
{
    1, 1, -1, -1, -1, -1, 1, 1
};

static void s5_wiggle_task(void)
{
    const uint16_t centre = S5_SERVO_HOME_DEG;
    int32_t  target;
    uint8_t  i;
    ER       er;

    (void)s5_servo_move_slowly(centre);

    for (;;)
    {
        target = (int32_t)centre;
        for (i = 0u; i < 8u; i++)
        {
            target += ((int32_t)k_wiggle_steps[i] *
                       (int32_t)S5_WIGGLE_STEP_DEG);
            er = s5_servo_move_slowly((uint16_t)target);
            tm_printf((UB*)"S5 wiggle %u deg\n", (unsigned int)target);
            if (er < E_OK)
            {
                tm_printf((UB*)"S5 servo move failed: %d\n", (int)er);
            }
            (void)tk_dly_tsk(S5_WIGGLE_PAUSE_MS);
        }
    }
}
#endif

static void s5_sensor_task(INT stacd, void *p_exinf)
{
    uint16_t distance_mm;
    uint16_t target_deg;
    uint8_t  idx = 2u;
    int8_t   dir = 1;
    ER       er;

    (void)stacd;
    (void)p_exinf;

#if S5_WIGGLE_DEMO
    s5_wiggle_task();
#endif

    for (;;)
    {
#if S5_CALIBRATE_HOLD
        /* Calibration: stay at the snap angle and just report distance. */
        distance_mm = drv_ultrasonic_ping_mm();
        tm_printf((UB*)"S5 hold %u deg: %u mm\n",
                  (unsigned int)drv_servo_get_angle(),
                  (unsigned int)distance_mm);
        (void)tk_dly_tsk(S5_SAMPLE_PERIOD_MS);
        continue;
#endif
        target_deg = k_scan_deg[idx];
        er = s5_servo_move_slowly(target_deg);
        if (er < E_OK)
        {
            tm_printf((UB*)"S5 servo move failed: %d\n", (int)er);
            (void)tk_dly_tsk(S5_SAMPLE_PERIOD_MS);
            continue;
        }

        distance_mm = drv_ultrasonic_ping_mm();
        if (distance_mm == S5_DIST_NONE)
        {
            tm_printf((UB*)"S5 scan %u deg: no echo\n",
                      (unsigned int)target_deg);
        }
        else
        {
            tm_printf((UB*)"S5 scan %u deg: %u mm\n",
                      (unsigned int)target_deg, (unsigned int)distance_mm);
        }
        (void)tk_dly_tsk(S5_SAMPLE_PERIOD_MS);

        /* Move to the next angle, reversing at either end. */
        if ((dir > 0) && (idx >= (S5_SCAN_ANGLE_COUNT - 1u)))
        {
            dir = -1;
        }
        else if ((dir < 0) && (idx == 0u))
        {
            dir = 1;
        }
        idx = (uint8_t)((int8_t)idx + dir);
    }
}

EXPORT ER subsystem5_init(void)
{
    ER er = drv_servo_init();

    if (er < E_OK)
    {
        tm_printf((UB*)"S5 servo init failed: %d\n", (int)er);
        return er;
    }

    er = drv_ultrasonic_init();
    if (er < E_OK)
    {
        tm_printf((UB*)"S5 ultrasonic init failed: %d\n", (int)er);
    }

    s_initialized = (er == E_OK);
    return er;
}

EXPORT ER subsystem5_start(void)
{
    ID tid;

    if (!s_initialized)
    {
        return E_OBJ;
    }

    tid = app_start_task((FP)s5_sensor_task, S5_TASK_PRI, 0u, "s5_sensor");
    return (tid < E_OK) ? (ER)tid : E_OK;
}