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
    UH              samples_before, samples_after;
    INT             pitch, roll;
    S1_TELEMETRY_PKT pkt;

    TEST_INIT();

    /* Give imu_task time to start the sensor and calibrate (about 1.5 s). */
    tk_dly_tsk(3000);

    /* S4-U01: the IMU answers. */
    TEST_CHECK("S4-U01", (s4_get_faults() & S4_FAULT_COMM) == 0U,
               "IMU answers on I2C1 (GP26 SDA, GP27 SCL)");

    /* S4-U02: calibration worked (the car was still and level). */
    TEST_CHECK("S4-U02", s4_is_calibrated(), "accelerometer calibrated while still");

    /* S4-U03: about 50 readings a second.  s4_samples is a 16-bit counter that
     * wraps, so only the DIFFERENCE means anything. */
    samples_before = s4_samples;
    tk_dly_tsk(1000);
    samples_after = s4_samples;
    TEST_CHECK("S4-U03",
               (UH)(samples_after - samples_before) >= 35U &&
               (UH)(samples_after - samples_before) <= 60U,
               "sensor read about 50 times a second");

    /* S4-U04: a level robot reads level. */
    s4_get_attitude(&pitch, &roll);
    TEST_CHECK("S4-U04", pitch > -5 && pitch < 5 && roll > -5 && roll < 5,
               "pitch and roll within +/-5 degrees when level");

    /* S4-U05: a still robot is STATIONARY. */
    TEST_CHECK("S4-U05", s4_get_motion() == S4_STATIONARY, "still robot is STATIONARY");

    /* S4-U06: no hump and no collision while sitting still. */
    TEST_CHECK("S4-U06",
               s4_get_hump_count() == 0 &&
               (s4_get_faults() & S4_FAULT_COLLISION) == 0U,
               "no false hump and no false collision");

    /* S4-U07: the pitch reaches Subsystem 1 (IF-01) and the fields that
     * Subsystem 2 and 5 set are left alone. */
    s1_set_speed(57);
    s1_set_obstacle(33);
    s1_set_pitch(99);                           /* a value that is not ours */
    tk_dly_tsk(400);                            /* a few status messages    */
    s1_get_telemetry(&pkt);
    TEST_CHECK("S4-U07",
               pkt.pitch_deg > -5 && pkt.pitch_deg < 5 &&
               pkt.speed_mm_s == 57 && pkt.obstacle_cm == 33,
               "pitch reaches subsystem 1; speed and obstacle left unchanged");

    /* S4-U08: an IMU fault reaches Subsystem 1's fault mask (FAULT_IMU_SENS)
     * without disturbing another subsystem's bit, and is removed again. */
    {
        BOOL set_ok, clear_ok, other_ok;

        s1_report_fault(FAULT_WIFI);            /* somebody else's fault */
        s4_fault_set(S4_FAULT_COLLISION);
        s1_get_telemetry(&pkt);
        set_ok   = ((pkt.fault_mask & FAULT_IMU_SENS) != 0U);
        s4_fault_clear(S4_FAULT_COLLISION);
        s1_get_telemetry(&pkt);
        clear_ok = ((pkt.fault_mask & FAULT_IMU_SENS) == 0U);
        other_ok = ((pkt.fault_mask & FAULT_WIFI) != 0U);
        s1_clear_fault(FAULT_WIFI);
        TEST_CHECK("S4-U08", set_ok && clear_ok && other_ok,
                   "FAULT_IMU_SENS set/cleared through subsystem 1, other bits kept");
    }

    /* S4-U17: our two conditions share the ONE IMU bit.  It stays on until
     * both are over. */
    {
        BOOL on_both, on_after_one, off_after_both;

        s4_fault_set(S4_FAULT_COMM);
        s4_fault_set(S4_FAULT_COLLISION);
        s1_get_telemetry(&pkt);
        on_both = ((pkt.fault_mask & FAULT_IMU_SENS) != 0U);
        s4_fault_clear(S4_FAULT_COLLISION);
        s1_get_telemetry(&pkt);
        on_after_one = ((pkt.fault_mask & FAULT_IMU_SENS) != 0U);
        s4_fault_clear(S4_FAULT_COMM);
        s1_get_telemetry(&pkt);
        off_after_both = ((pkt.fault_mask & FAULT_IMU_SENS) == 0U);
        TEST_CHECK("S4-U17", on_both && on_after_one && off_after_both,
                   "one IMU bit stays set until both conditions are over");
    }

    /* S4-U18: the message is 8 bytes, and values outside the range are CLIPPED,
     * not wrapped (a pitch of 200 degrees must not become -56). */
    {
        UW   snap = s4_snap_pack(-128, 127, -5);

        TEST_CHECK("S4-U18",
                   sizeof(S4_MSG) == 8U &&
                   s4_clip_s(200, 8) == 127 && s4_clip_s(-200, 8) == -128 &&
                   s4_clip_s(127, 8) == 127 && s4_clip_s(-128, 8) == -128 &&
                   s4_clip_u(300, 8) == 255U && s4_clip_u(-5, 8) == 0U &&
                   s4_clip_u(255, 8) == 255U && s4_clip_u(127, 7) == 127U &&
                   s4_clip_u(128, 7) == 127U &&
                   s4_snap_field(snap, 0) == -128 && s4_snap_field(snap, 1) == 127 &&
                   s4_snap_field(snap, 2) == -5,
                   "8-byte message; values are clipped at the limits, never wrapped");
    }

    /* S4-U19: Subsystem 1's car_state and hump_state fields carry OUR motion
     * state and hump state (if Subsystem 1 decides car_state means something
     * else, drop this test and change s4_report_status()). */
    s1_get_telemetry(&pkt);
    TEST_CHECK("S4-U19",
               pkt.car_state == (UINT)S4_STATIONARY && pkt.hump_state == 0U,
               "telemetry car_state/hump_state show STATIONARY and flat");

    TEST_SUMMARY("subsystem4");
}

#endif /* APP_TEST */
