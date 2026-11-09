/**
 * @file    subsystem5_test.c
 * @brief   On-target tests for subsystem 5. Built only with: make TEST=1
 *
 * Record results in testing/subsystem5/TEST_LOG.md. IDs: S5-U01, ...
 *
 *   S5-U02     S5 -> S1 fault reporting, with the real sensor fitted (the
 *              scanner has been running for a few seconds by then).
 *   S5-U03..08 Obstacle guard stop / slow-down decisions (pure function,
 *              no hardware or motion needed).
 *   S5-U09..11 Obstacle profile geometry: sideways distance of a point
 *              (pure function).
 *   S5-U12..17 Bypass decision from a made-up profile (pure function).
 *   S5-U18..19 Bypass driving maths: encoder ticks and turns (pure).
 *   S5-U20     Bypass decision: least protrusion wins over a wider gap.
 *
 * Lateral checks assume S5_EDGE_TRIM_DEG = 0 (sin 30 deg = 0.5 exactly).
 */
#include <stdint.h>
#include <stdbool.h>
#include "../app.h"
#include "../systemFaults.h"
#include "../subsystem1/subsystem1.h"
#include "../test_report.h"
#include "subsystem5.h"
#include "s5_config.h"
#include "s5_report.h"
#include "s5_guard.h"
#include "s5_profile.h"
#include "s5_avoid.h"
#include "s5_drive.h"

#if APP_TEST

EXPORT void subsystem5_test (void)
{
    S1_TELEMETRY_PKT telemetry;
    bool             b_stopped = false;
    uint8_t          speed_cap;
    s5_profile_t     profile   = { 0u };

    TEST_INIT();

    TEST_CHECK("S5-U01", tk_get_tid() > 0, "running in a task context");

    /* Reporting to Subsystem 1 (sensor connected) */
    s1_get_telemetry(&telemetry);
    TEST_CHECK("S5-U02", (!s5_report_is_faulted()) &&
                         (0u == (telemetry.fault_mask & FAULT_US_SENS)),
               "healthy ultrasonic: FAULT_US_SENS not reported");

    /* Obstacle guard decisions */
    speed_cap = s5_guard_speed_cap(S5_OBSTACLE_NONE_CM, false, &b_stopped);
    TEST_CHECK("S5-U03", (S5_FULL_SPEED_CAP == speed_cap) && (!b_stopped),
               "no obstacle: no speed limit");

    speed_cap = s5_guard_speed_cap(S5_SENSOR_TO_FRONT_CM + S5_SLOW_DISTANCE_CM,
                                   false, &b_stopped);
    TEST_CHECK("S5-U04", (S5_SLOW_SPEED_CAP == speed_cap) && (!b_stopped),
               "obstacle within slow distance: car slows down");

    speed_cap = s5_guard_speed_cap(S5_SENSOR_TO_FRONT_CM + S5_STOP_DISTANCE_CM,
                                   false, &b_stopped);
    TEST_CHECK("S5-U05", (0u == speed_cap) && b_stopped,
               "obstacle at the stop distance: car stops");

    speed_cap = s5_guard_speed_cap(S5_SENSOR_TO_FRONT_CM + S5_STOP_TRIGGER_CM,
                                   false, &b_stopped);
    TEST_CHECK("S5-U06", (0u == speed_cap) && b_stopped,
               "stop starts S5_BRAKE_MARGIN_CM early");

    speed_cap = s5_guard_speed_cap(S5_SENSOR_TO_FRONT_CM + S5_RESUME_GAP_CM,
                                   true, &b_stopped);
    TEST_CHECK("S5-U07", (0u == speed_cap) && b_stopped,
               "stopped: stays stopped inside the resume gap");

    speed_cap = s5_guard_speed_cap((S5_SENSOR_TO_FRONT_CM +
                                    S5_RESUME_GAP_CM) + 1u,
                                   true, &b_stopped);
    TEST_CHECK("S5-U08", (0u != speed_cap) && (!b_stopped),
               "stopped: resumes once the obstacle is beyond the resume gap");

    /* Obstacle profile geometry */
    TEST_CHECK("S5-U09", 0u == s5_profile_lateral_cm(20u, 90u),
               "point straight ahead is 0 cm to the side");
    TEST_CHECK("S5-U10", 10u == s5_profile_lateral_cm(20u, 60u),
               "20 cm away, 30 deg off axis (low side): 10 cm to the side");
    TEST_CHECK("S5-U11", 10u == s5_profile_lateral_cm(20u, 120u),
               "20 cm away, 30 deg off axis (high side): 10 cm to the side");

    /* Bypass decision. Start from a valid profile with no room either
     * side, then open up one side at a time. */
    profile.b_valid         = 1u;
    profile.left_extent_cm  = 8u;
    profile.right_extent_cm = 4u;

    TEST_CHECK("S5-U12", S5_BYPASS_CONTINUE ==
                         s5_avoid_choose(&profile, false, 0u),
               "obstacle gone after the scan: continue");
    TEST_CHECK("S5-U13", S5_BYPASS_STOP ==
                         s5_avoid_choose(NULL, true, 0u),
               "scan failed: stop");
    TEST_CHECK("S5-U14", S5_BYPASS_REVERSE ==
                         s5_avoid_choose(&profile, true, 0u),
               "no room either side: reverse and scan again");
    TEST_CHECK("S5-U15", S5_BYPASS_STOP ==
                         s5_avoid_choose(&profile, true,
                                         S5_BYPASS_MAX_REVERSES),
               "no room either side, reverses used up: stop");

    profile.left_clear_cm = S5_BYPASS_NEED_CM;
    TEST_CHECK("S5-U16", S5_BYPASS_LEFT ==
                         s5_avoid_choose(&profile, true, 0u),
               "room only on the left: go left");

    profile.right_clear_cm = S5_BYPASS_NEED_CM;
    TEST_CHECK("S5-U17", S5_BYPASS_RIGHT ==
                         s5_avoid_choose(&profile, true, 0u),
               "room both sides: shorter detour (right) wins");

    profile.left_clear_cm = (uint16_t)(S5_BYPASS_NEED_CM + 10u);
    TEST_CHECK("S5-U20", S5_BYPASS_RIGHT ==
                         s5_avoid_choose(&profile, true, 0u),
               "room both sides: the side the obstacle sticks out least "
               "(right) wins over a wider gap (left)");

    /* Bypass driving maths */
    TEST_CHECK("S5-U18", S5_ENCODER_TICKS_PER_REV ==
                         s5_drive_ticks_for_mm(S5_WHEEL_CIRCUM_MM),
               "one wheel circumference is one encoder revolution");
    TEST_CHECK("S5-U19", (s5_drive_turn_mm(360u) -
                          (4u * s5_drive_turn_mm(90u))) <= 3u,
               "four 90 deg turns make one full turn (within rounding)");

    TEST_SUMMARY("subsystem5");
}

#endif /* APP_TEST */
