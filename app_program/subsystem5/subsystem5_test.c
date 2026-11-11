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
 *              (pure function). S5-U28: 1 degree resolution.
 *   S5-U12..16 Bypass decision from a made-up profile (pure function):
 *              the edge closer to the centre line wins.
 *   S5-U18..19 Bypass driving maths: encoder ticks and turns (pure).
 *   S5-U20..22 Bypass decision when an edge was not found within the scan
 *              limits: the side whose edge was found; none: reverse, then
 *              stop.
 *   S5-U23..27 Too big to get round (pure): a side too wide is skipped;
 *              a side without room from here is gone round after
 *              reversing just far enough; more than
 *              S5_BYPASS_MAX_BACKUP_MM needed: stop (wait).
 *   S5-U34     How far to reverse first (pure).
 *   S5-U29     A measured edge is preferred to a lower-bound one.
 *   S5-U30     No edge found at all: reverse.
 *   S5-U31..33 Planned edge from the 10..20 cm width range (pure): a lower
 *              bound is planned at the widest the range allows, a measured
 *              edge at least as far as the narrowest allows.
 *
 * Lateral checks assume S5_EDGE_TRIM_DEG = 0 and S5_SENSOR_PIVOT_CM = 0
 * (sin 30 deg = 0.5 exactly).
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
    TEST_CHECK("S5-U28", (s5_profile_lateral_cm(100u, 69u) == 36u) &&
                         (s5_profile_lateral_cm(100u, 68u) == 37u),
               "1 degree apart gives a different sideways distance "
               "(100 cm at 21 and 22 deg: 36 and 37 cm)");

    /* Bypass decision. Start from a valid profile with both edges found:
     * the left edge 8 cm and the right edge 4 cm from the centre line, the
     * obstacle 25 cm ahead. */
    profile.b_valid         = 1u;
    profile.closest_cm      = 25u;
    profile.b_left_edge     = 1u;
    profile.b_right_edge    = 1u;
    profile.left_extent_cm  = 8u;
    profile.right_extent_cm = 4u;

    TEST_CHECK("S5-U12", S5_BYPASS_CONTINUE ==
                         s5_avoid_choose(&profile, false, 0u),
               "obstacle gone after the scan: continue");
    TEST_CHECK("S5-U13", S5_BYPASS_STOP ==
                         s5_avoid_choose(NULL, true, 0u),
               "scan failed: stop");
    TEST_CHECK("S5-U14", S5_BYPASS_RIGHT ==
                         s5_avoid_choose(&profile, true, 0u),
               "both edges found: the closer edge (right, 4 cm) wins");

    profile.left_extent_cm = 3u;
    TEST_CHECK("S5-U15", S5_BYPASS_LEFT ==
                         s5_avoid_choose(&profile, true, 0u),
               "both edges found: the closer edge (left, 3 cm) wins");

    profile.left_extent_cm = 4u;
    TEST_CHECK("S5-U16", S5_BYPASS_LEFT ==
                         s5_avoid_choose(&profile, true, 0u),
               "edges equally far: left");

    profile.b_right_edge = 0u;
    TEST_CHECK("S5-U20", S5_BYPASS_LEFT ==
                         s5_avoid_choose(&profile, true, 0u),
               "right edge not found (width unknown), nothing else beside "
               "it: go round the left at once");
    TEST_CHECK("S5-U21", S5_BYPASS_LEFT ==
                         s5_avoid_choose(&profile, true,
                                         S5_BYPASS_MAX_REVERSES),
               "right edge still not found, reverses used up: go round the "
               "side whose edge was found (left)");

    profile.b_left_edge = 0u;
    TEST_CHECK("S5-U30", S5_BYPASS_REVERSE ==
                         s5_avoid_choose(&profile, true, 0u),
               "no edge found at all: reverse and scan again");
    TEST_CHECK("S5-U22", S5_BYPASS_STOP ==
                         s5_avoid_choose(&profile, true,
                                         S5_BYPASS_MAX_REVERSES),
               "no edge found at all, reverses used up: stop");

    /* Too big to get round (S5_BYPASS_TURN_DEG = 45, S5_OUT_LEG_MIN_CM =
     * 12, sensor 9 cm ahead of the axle: from 25 cm away, edges up to
     * about 21 cm to the side fit). */
    TEST_CHECK("S5-U23", s5_avoid_side_fits(15u, 25u) &&
                         (!s5_avoid_side_fits(30u, 25u)),
               "from 25 cm: a 15 cm edge fits, a 30 cm edge does not");
    TEST_CHECK("S5-U24", s5_avoid_side_fits(30u, 45u),
               "further back (45 cm) the 30 cm edge fits");

    profile.b_left_edge     = 1u;
    profile.b_right_edge    = 1u;
    profile.left_extent_cm  = 30u;
    profile.right_extent_cm = 10u;
    TEST_CHECK("S5-U25", S5_BYPASS_RIGHT ==
                         s5_avoid_choose(&profile, true, 0u),
               "left too wide: go round the right");

    profile.right_extent_cm = 35u;
    TEST_CHECK("S5-U26", (S5_BYPASS_LEFT ==
                          s5_avoid_choose(&profile, true, 0u)) &&
                         (82u == s5_avoid_plan_backup_mm(&profile, true)),
               "too wide on both sides from here: go round the narrower "
               "(left), reversing 82 mm first, no new scan");

    profile.left_extent_cm  = 40u;
    profile.right_extent_cm = 45u;
    TEST_CHECK("S5-U27", S5_BYPASS_STOP ==
                         s5_avoid_choose(&profile, true, 0u),
               "both sides need more than S5_BYPASS_MAX_BACKUP_MM of "
               "reversing: stop and wait");
    TEST_CHECK("S5-U34", (0u == s5_avoid_backup_mm(5u, 10u)) &&
                         (132u == s5_avoid_backup_mm(20u, 10u)),
               "from 10 cm a 5 cm edge needs no reversing, a 20 cm edge "
               "132 mm");

    profile.left_extent_cm  = 10u;
    profile.right_extent_cm = 6u;
    profile.b_left_exact    = 1u;
    profile.b_right_exact   = 0u;
    TEST_CHECK("S5-U29", S5_BYPASS_LEFT ==
                         s5_avoid_choose(&profile, true, 0u),
               "left measured at 10 cm beats right only known to be at "
               "least 6 cm (planned at 20 - 10 = 10 cm: a tie, measured "
               "wins)");

    /* Planned edges (S5_OBSTACLE_MIN/MAX_WIDTH_CM = 10 / 20). */
    profile.left_extent_cm  = 5u;
    profile.b_left_exact    = 0u;
    profile.right_extent_cm = 8u;
    profile.b_right_exact   = 1u;
    TEST_CHECK("S5-U31", (12u == s5_avoid_plan_extent(&profile, true)) &&
                         (8u == s5_avoid_plan_extent(&profile, false)),
               "left lower bound 5 cm is planned at 20 - 8 = 12 cm; right "
               "measured 8 cm stays 8 cm");

    profile.left_extent_cm  = 4u;
    profile.b_left_exact    = 1u;
    profile.right_extent_cm = 3u;
    TEST_CHECK("S5-U32", 6u == s5_avoid_plan_extent(&profile, false),
               "measured 4 + 3 cm is under the 10 cm minimum: right "
               "planned at 10 - 4 = 6 cm");
    TEST_CHECK("S5-U33", S5_BYPASS_RIGHT ==
                         s5_avoid_choose(&profile, true, 0u),
               "right planned 6 cm vs left planned 7 cm: right");

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
