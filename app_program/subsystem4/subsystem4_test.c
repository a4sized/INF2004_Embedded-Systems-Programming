/*
 *  subsystem4_test.c  --  on-target tests, built with "make TEST=1".
 *
 *  Keep the robot STILL and LEVEL and do not touch it for about 8 seconds.
 *  Hump heights and collisions can not be tested like this: they need the
 *  car driven (see testing/subsystem4/TEST_LOG.md).
 */

#include "subsystem4.h"
#include "../test_report.h"
#include "../systemFaults.h"
#include "../subsystem1/subsystem1.h"

#if APP_TEST

void subsystem4_test(void)
{
    UW      samples_before, samples_after;
    float   pitch, roll;

    TEST_INIT();

    /* Give imu_task time to start the sensor and calibrate (about 1.5 s). */
    tk_dly_tsk(3000);

    /* S4-U01: the IMU answers. */
    TEST_CHECK("S4-U01", (s4_get_faults() & S4_FAULT_COMM) == 0U,
               "IMU answers on I2C1 (GP26 SDA, GP27 SCL)");

    /* S4-U02: calibration worked (the car was still and level). */
    TEST_CHECK("S4-U02", s4_calibrated, "accelerometer calibrated while still");

    /* S4-U03: about 50 readings a second. */
    samples_before = s4_samples;
    tk_dly_tsk(1000);
    samples_after = s4_samples;
    TEST_CHECK("S4-U03",
               (samples_after - samples_before) >= 35U &&
               (samples_after - samples_before) <= 60U,
               "sensor read about 50 times a second");

    /* S4-U04: a level robot reads level. */
    s4_get_attitude(&pitch, &roll);
    TEST_CHECK("S4-U04", pitch > -5.0f && pitch < 5.0f && roll > -5.0f && roll < 5.0f,
               "pitch and roll within +/-5 degrees when level");

    /* S4-U05: a still robot is STATIONARY. */
    TEST_CHECK("S4-U05", s4_get_motion() == S4_STATIONARY, "still robot is STATIONARY");

    /* S4-U06: no hump and no collision while sitting still. */
    TEST_CHECK("S4-U06",
               s4_get_hump_count() == 0U &&
               (s4_get_faults() & S4_FAULT_COLLISION) == 0U,
               "no false hump and no false collision");

    /* S4-U07: the pitch reaches Subsystem 1 (IF-01) and the other fields
     * that Subsystem 2 and 5 set are left alone. */
    {
        S1_TELEMETRY t;
        INT speed, obstacle;

        s1_get_telemetry(&t);
        speed = t.speed_mm_s + 7;
        obstacle = t.obstacle_cm + 3;
        s1_update_telemetry(speed, obstacle, 99, t.fault_code, NULL);
        tk_dly_tsk(400);                        /* a few status messages */
        s1_get_telemetry(&t);
        TEST_CHECK("S4-U07",
                   t.pitch_deg > -5 && t.pitch_deg < 5 &&
                   t.speed_mm_s == speed && t.obstacle_cm == obstacle,
                   "pitch reaches subsystem 1; speed and obstacle left unchanged");
    }

    /* S4-U08: an IMU fault reaches the shared mask without disturbing the
     * bits of other subsystems, and is removed again. */
    {
        UW   other = (UW)FAULT_CMD_REJECTED;
        BOOL set_ok, clear_ok, other_ok;

        SET_FAULT(active_fault_mask, other);
        s4_fault_set(S4_FAULT_COLLISION);
        set_ok   = ((active_fault_mask & S4_FAULT_COLLISION) != 0U);
        s4_fault_clear(S4_FAULT_COLLISION);
        clear_ok = ((active_fault_mask & S4_FAULT_COLLISION) == 0U);
        other_ok = ((active_fault_mask & other) != 0U);
        CLEAR_FAULT(active_fault_mask, other);
        TEST_CHECK("S4-U08", set_ok && clear_ok && other_ok,
                   "IMU fault bit set/cleared in the shared mask, others kept");
    }

    TEST_SUMMARY("subsystem4");
}

#endif /* APP_TEST */
