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

#if APP_TEST

EXPORT void subsystem5_test (void)
{
    S1_TELEMETRY_PKT telemetry;
    bool             b_stopped = false;
    uint8_t          speed_cap;

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
               "obstacle 5 cm from the front: car stops");

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

    TEST_SUMMARY("subsystem5");
}

#endif /* APP_TEST */
