/**
 * @file    subsystem5_test.c
 * @brief   On-target tests for subsystem 5. Built only with: make TEST=1
 *
 * Record results in testing/subsystem5/TEST_LOG.md. All checks are pure
 * Subsystem 5 functions: no other subsystem, hardware or motion is needed.
 *
 *   S5-U03, U07  Obstacle guard: speed-cap zones; stop/resume hysteresis.
 *   S5-U09, U28  Obstacle profile geometry: sideways distance; 1 degree
 *                resolution.
 *   S5-U12..U14  Bypass decision: continue, stop on a failed scan, the
 *                closer edge wins.
 *   S5-U20, U22  Bypass decision when an edge was not found: the found
 *                side; none: reverse, then stop.
 *   S5-U23..U27  Too big to get round: room to swing out, a side too wide
 *                is skipped, reverse first, or stop (wait).
 *   S5-U34       How far to reverse first.
 *   S5-U31, U29  Planned edges from the 10..20 cm width range, and the
 *                decision made from them.
 *   S5-U18       Bypass driving maths: encoder ticks and turns.
 *
 * Retired IDs (merged or removed; never reused): S5-U01, U02, U04..U06,
 * U08, U10, U11, U15, U16, U19, U21, U24, U30, U32, U33, U35.
 *
 * Lateral checks assume S5_SENSOR_PIVOT_CM = 0 (sin 30 deg = 0.5 exactly).
 */
#include <stdint.h>
#include <stdbool.h>
#include "../app.h"
#include "../test_report.h"
#include "subsystem5.h"
#include "s5_config.h"
#include "s5_guard.h"
#include "s5_profile.h"
#include "s5_avoid.h"
#include "s5_drive.h"

#if APP_TEST

EXPORT void subsystem5_test (void)
{
    s5_profile_t     profile   = { 0u };

    TEST_INIT();

    /* Obstacle guard: speed-cap zones (S5-U03) and hysteresis (S5-U07) */
    {
        bool    b_none, b_slow, b_stop, b_early;
        uint8_t cap_none, cap_slow, cap_stop, cap_early;

        cap_none  = s5_guard_speed_cap(S5_OBSTACLE_NONE_CM, false, &b_none);
        cap_slow  = s5_guard_speed_cap(S5_SENSOR_TO_FRONT_CM +
                                       S5_SLOW_DISTANCE_CM, false, &b_slow);
        cap_stop  = s5_guard_speed_cap(S5_SENSOR_TO_FRONT_CM +
                                       S5_STOP_DISTANCE_CM, false, &b_stop);
        cap_early = s5_guard_speed_cap(S5_SENSOR_TO_FRONT_CM +
                                       S5_STOP_TRIGGER_CM, false, &b_early);
        TEST_CHECK("S5-U03", (S5_FULL_SPEED_CAP == cap_none) && (!b_none) &&
                             (S5_SLOW_SPEED_CAP == cap_slow) && (!b_slow) &&
                             (0u == cap_stop) && b_stop &&
                             (0u == cap_early) && b_early,
                   "guard zones: none = full speed, slow distance = slow, "
                   "stop distance and early trigger = stop");
    }
    {
        bool    b_inside, b_beyond;
        uint8_t cap_inside, cap_beyond;

        cap_inside = s5_guard_speed_cap(S5_SENSOR_TO_FRONT_CM +
                                        S5_RESUME_GAP_CM, true, &b_inside);
        cap_beyond = s5_guard_speed_cap((S5_SENSOR_TO_FRONT_CM +
                                         S5_RESUME_GAP_CM) + 1u,
                                        true, &b_beyond);
        TEST_CHECK("S5-U07", (0u == cap_inside) && b_inside &&
                             (0u != cap_beyond) && (!b_beyond),
                   "stopped: stays stopped inside the resume gap, "
                   "resumes beyond it");
    }

    /* Obstacle profile geometry */
    TEST_CHECK("S5-U09", (0u == s5_profile_lateral_cm(20u, 90u)) &&
                         (10u == s5_profile_lateral_cm(20u, 60u)) &&
                         (10u == s5_profile_lateral_cm(20u, 120u)),
               "sideways distance: straight ahead 0 cm; 20 cm away, "
               "30 deg off axis on either side 10 cm");
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
    {
        const s5_bypass_t right_closer = s5_avoid_choose(&profile, true, 0u);
        s5_bypass_t       left_closer;
        s5_bypass_t       equal;

        profile.left_extent_cm = 3u;
        left_closer = s5_avoid_choose(&profile, true, 0u);
        profile.left_extent_cm = 4u;
        equal = s5_avoid_choose(&profile, true, 0u);
        TEST_CHECK("S5-U14", (S5_BYPASS_RIGHT == right_closer) &&
                             (S5_BYPASS_LEFT == left_closer) &&
                             (S5_BYPASS_LEFT == equal),
                   "both edges found: the closer edge wins (right 4 cm, "
                   "left 3 cm); equally far: left");
    }

    profile.b_right_edge = 0u;
    TEST_CHECK("S5-U20", (S5_BYPASS_LEFT ==
                          s5_avoid_choose(&profile, true, 0u)) &&
                         (S5_BYPASS_LEFT ==
                          s5_avoid_choose(&profile, true,
                                          S5_BYPASS_MAX_REVERSES)),
               "only the left edge found: go round the left, before and "
               "after the reverses are used up");

    profile.b_left_edge = 0u;
    TEST_CHECK("S5-U22", (S5_BYPASS_REVERSE ==
                          s5_avoid_choose(&profile, true, 0u)) &&
                         (S5_BYPASS_STOP ==
                          s5_avoid_choose(&profile, true,
                                          S5_BYPASS_MAX_REVERSES)),
               "no edge found: reverse and scan again; reverses used up: "
               "stop");

    /* Too big to get round (S5_BYPASS_TURN_DEG = 35, S5_OUT_LEG_MIN_CM =
     * 12, sensor 9 cm ahead of the axle: from 25 cm away, edges up to
     * 15 cm to the side fit; a 30 cm edge from 46 cm). */
    TEST_CHECK("S5-U23", s5_avoid_side_fits(15u, 25u) &&
                         (!s5_avoid_side_fits(30u, 25u)) &&
                         s5_avoid_side_fits(30u, 55u),
               "room to swing out: from 25 cm a 15 cm edge fits, a 30 cm "
               "edge does not; from 55 cm it does");

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
                         (210u == s5_avoid_plan_backup_mm(&profile, true)),
               "too wide on both sides from here: go round the narrower "
               "(left), reversing 210 mm first, no new scan");

    profile.left_extent_cm  = 50u;   /* both far over the 250 mm max */
    profile.right_extent_cm = 55u;
    TEST_CHECK("S5-U27", S5_BYPASS_STOP ==
                         s5_avoid_choose(&profile, true, 0u),
               "both sides need more than S5_BYPASS_MAX_BACKUP_MM of "
               "reversing: stop and wait");
    TEST_CHECK("S5-U34", (0u == s5_avoid_backup_mm(5u, 10u)) &&
                         (217u == s5_avoid_backup_mm(20u, 10u)),
               "from 10 cm a 5 cm edge needs no reversing, a 20 cm edge "
               "217 mm");

    /* Planned edges (S5_OBSTACLE_MIN/MAX_WIDTH_CM = 10 / 20) */
    {
        uint16_t lower_left, measured_right, spread_left, short_right;

        /* Left lower bound 5 cm, right measured 8 cm. */
        profile.left_extent_cm  = 5u;
        profile.b_left_exact    = 0u;
        profile.right_extent_cm = 8u;
        profile.b_right_exact   = 1u;
        lower_left     = s5_avoid_plan_extent(&profile, true);
        measured_right = s5_avoid_plan_extent(&profile, false);

        /* A 20 cm box, 12 cm left / 8 cm right: the right reads 2 cm wide
         * (beam spread), the left only a lower bound at 9 cm. */
        profile.left_extent_cm  = 9u;
        profile.right_extent_cm = 10u;
        spread_left = s5_avoid_plan_extent(&profile, true);

        /* Measured 4 + 3 cm, under the 10 cm minimum. */
        profile.left_extent_cm  = 4u;
        profile.b_left_exact    = 1u;
        profile.right_extent_cm = 3u;
        short_right = s5_avoid_plan_extent(&profile, false);

        TEST_CHECK("S5-U31", (15u == lower_left) && (8u == measured_right) &&
                             (spread_left >= 12u) && (6u == short_right),
                   "planned edges: lower bound 5 cm planned at 15 cm, "
                   "measured 8 cm stays 8 cm, not short of the real edge "
                   "when the other side reads wide, 4 + 3 cm right planned "
                   "at 6 cm");
    }

    /* Decision from planned edges */
    {
        const s5_bypass_t planned_6_7 = s5_avoid_choose(&profile, true, 0u);
        s5_bypass_t       measured_vs_lower;

        profile.left_extent_cm  = 10u;
        profile.b_left_exact    = 1u;
        profile.right_extent_cm = 6u;
        profile.b_right_exact   = 0u;
        measured_vs_lower = s5_avoid_choose(&profile, true, 0u);
        TEST_CHECK("S5-U29", (S5_BYPASS_LEFT == measured_vs_lower) &&
                             (S5_BYPASS_RIGHT == planned_6_7),
                   "decision uses planned edges: left measured 10 cm beats "
                   "right lower bound (planned 13 cm); right planned 6 cm "
                   "beats left planned 7 cm");
    }

    /* Bypass driving maths */
    TEST_CHECK("S5-U18", (S5_ENCODER_TICKS_PER_REV ==
                          s5_drive_ticks_for_mm(S5_WHEEL_CIRCUM_MM)) &&
                         ((s5_drive_turn_mm(360u) -
                           (4u * s5_drive_turn_mm(90u))) <= 3u),
               "drive maths: one wheel circumference is one encoder "
               "revolution; four 90 deg turns make one full turn");

    TEST_SUMMARY("subsystem5");
}

#endif /* APP_TEST */
