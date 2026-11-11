/**
 * @file    s5_scan.c
 * @brief   Subsystem 5, stages 2 to 4: fine scan an obstacle and decide.
 *
 * Each side (low angles first, then high) is scanned in two passes:
 *   1. coarse: the servo steps outward from 90 degrees, S5_FINE_STEP_DEG
 *      at a time, all the way to the scan limit (also past the edge, so
 *      both sides always cover the same angles). The edge is where the
 *      readings leave the obstacle for good: a jump off it that comes back
 *      close to the readings before it within S5_EDGE_BRIDGE_MAX readings
 *      was a missed echo, so the scan carries on to the real edge;
 *   2. refine: between the last coarse angle on the obstacle and the first
 *      one off it, the servo steps outward S5_FINE_REFINE_DEG at a time
 *      until a reading leaves the obstacle, which pins the edge down to
 *      that step.
 * The readings go to s5_profile.c, which works out the edges; the servo
 * turns back to 90 degrees, and s5_avoid.c decides: continue, go round left
 * or right, reverse and scan again, or stop.
 *
 * Casts in this file: (UB *) only retypes a string literal for the T-Kernel
 * tm_printf() API; (unsigned int) widens values that always fit. The other
 * casts are commented where they are made.
 */
#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>
#include "../app.h"
#include "s5_config.h"
#include "s5_log.h"
#include "s5_guard.h"
#include "s5_profile.h"
#include "s5_avoid.h"
#include "s5_sense.h"
#include "s5_scan.h"

/* Coarse steps on each side of 90 degrees. */
#define FINE_LOW_STEPS      ((S5_FORWARD_DEG - S5_FINE_MIN_ANGLE) / \
                             S5_FINE_STEP_DEG)
#define FINE_HIGH_STEPS     ((S5_FINE_MAX_ANGLE - S5_FORWARD_DEG) / \
                             S5_FINE_STEP_DEG)

_Static_assert((S5_FINE_MIN_ANGLE < S5_FORWARD_DEG) &&
               (S5_FINE_MAX_ANGLE > S5_FORWARD_DEG) &&
               (S5_FINE_MAX_ANGLE <= S5_SERVO_MAX_DEG),
               "fine scan must cover both sides of 90 degrees");
_Static_assert((S5_FINE_REFINE_DEG >= 1u) &&
               (S5_FINE_REFINE_DEG < S5_FINE_STEP_DEG),
               "refine step must be finer than the coarse step");

/* The sensor is at most S5_SENSOR_TO_FRONT_CM + S5_STOP_TRIGGER_CM from the
 * obstacle when the car stops. After every reverse the obstacle must still
 * count as in the way (s5_scan_obstacle), or the car would take it for gone
 * and drive into it. */
_Static_assert((((S5_SENSOR_TO_FRONT_CM + S5_STOP_TRIGGER_CM) *
                 S5_MM_PER_CM) +
                (S5_BYPASS_MAX_REVERSES * S5_REVERSE_MM)) <=
               (S5_STILL_BLOCKED_CM * S5_MM_PER_CM),
               "after reversing, the obstacle must still count as in the "
               "way");

/* Name of a side as the car sees it (S5_HIGH_ANGLE_IS_LEFT). */
static const char * s5_scan_side_name (const bool b_high_side)
{
    return (b_high_side == (0u != S5_HIGH_ANGLE_IS_LEFT)) ? "left" : "right";
}

/* Marks printed after a fine-scan reading. */
#define MARK_NONE           ""
#define MARK_EDGE           "  <- edge (first off the obstacle)"
#define MARK_PAST           "  (past the edge)"
#define MARK_BRIDGED        "  (back on the obstacle: that was a missed echo)"
#define MARK_BACK           "  (past the edge, looks like the obstacle)"

/*
 * Print one fine-scan reading (always, s5_log.h): the servo angle, the side
 * of the car it looks at (S5_HIGH_ANGLE_IS_LEFT), the distance (the median
 * of the pings there), and p_mark (one of the MARK_ strings).
 */
static void s5_scan_print_reading (const uint16_t angle_deg, const bool b_good,
                                   const uint16_t distance_cm,
                                   const char * const p_mark)
{
    const char * const p_side = (S5_FORWARD_DEG == angle_deg) ? "ahead" :
                                s5_scan_side_name(angle_deg > S5_FORWARD_DEG);

    if (!b_good)
    {
        tm_printf((UB *)"S5 scan %u deg (%s): reading failed\n",
                  (unsigned int)angle_deg, p_side);
    }
    else if (S5_OBSTACLE_NONE_CM == distance_cm)
    {
        tm_printf((UB *)"S5 scan %u deg (%s): no echo%s\n",
                  (unsigned int)angle_deg, p_side, p_mark);
    }
    else
    {
        tm_printf((UB *)"S5 scan %u deg (%s): %u cm%s\n",
                  (unsigned int)angle_deg, p_side,
                  (unsigned int)distance_cm, p_mark);
    }
}

/* What one side's scan covered, for the summary (scanning task only). */
typedef struct
{
    uint8_t angles;       /* angles the servo was turned to               */
    uint8_t missed;       /* of those, how many got no reading            */
    uint8_t furthest;     /* furthest it looked, degrees off straight ahead */
    uint8_t bridged;      /* missed echoes bridged (the scan carried on)  */
    uint8_t came_back;    /* readings after the edge was final that looked
                           * like the obstacle again (edge in doubt)      */
} side_stats_t;

static side_stats_t g_stats[2];   /* [0] = low side, [1] = high side */

/* The summary of one side (b_high_side = angles above 90). */
static side_stats_t * s5_scan_stats (const bool b_high_side)
{
    return &g_stats[b_high_side ? 1u : 0u];
}

/* Turn the servo to angle_deg, take a fine-scan reading there (cm) and
 * count it for that side's summary. Returns false if the reading failed.
 * The casts are safe: a side scans at most a few dozen angles, and an
 * angle is at most 90 degrees off straight ahead. */
static bool s5_scan_read (const uint16_t angle_deg, uint16_t * const p_cm)
{
    side_stats_t * const p_stats = s5_scan_stats(angle_deg > S5_FORWARD_DEG);
    const uint8_t        off_deg = (uint8_t)s5_profile_off_axis_deg(
                                       angle_deg);
    bool                 b_read;

    s5_sense_servo_to(angle_deg, S5_LOOK_SIDE);
    b_read = s5_sense_fine_reading(p_cm);

    p_stats->angles = (uint8_t)(p_stats->angles + 1u);
    if (!b_read)
    {
        p_stats->missed = (uint8_t)(p_stats->missed + 1u);
    }
    if (off_deg > p_stats->furthest)
    {
        p_stats->furthest = off_deg;
    }

    return b_read;
}

/*
 * Print what one side's scan covered, so both sides can be compared: how
 * many angles, how far out, and where the edge was (between the last angle
 * on the obstacle and the first off it, in degrees off straight ahead).
 * Printed with S5_VERBOSE: an obstacle that "looks" narrower on one side
 * every time shows up here first.
 */
static void s5_scan_print_side (const bool b_high_side)
{
    const side_stats_t * const p_stats = s5_scan_stats(b_high_side);

    if (s5_profile_edge_found(b_high_side))
    {
        S5_LOG((UB *)"S5 scan %s: %u angles (%u failed), out to %u deg "
               "off ahead; edge between %u and %u deg off ahead\n",
               s5_scan_side_name(b_high_side),
               (unsigned int)p_stats->angles, (unsigned int)p_stats->missed,
               (unsigned int)p_stats->furthest,
               (unsigned int)s5_profile_off_axis_deg(
                   s5_profile_on_angle(b_high_side)),
               (unsigned int)s5_profile_off_axis_deg(
                   s5_profile_off_angle(b_high_side)));
        if (0u != p_stats->bridged)
        {
            S5_LOG((UB *)"S5 scan %s: %u missed echo(es) bridged: the "
                   "readings came back onto the obstacle, so the scan "
                   "carried on to the real edge\n",
                   s5_scan_side_name(b_high_side),
                   (unsigned int)p_stats->bridged);
        }
        if (0u != p_stats->came_back)
        {
            S5_LOG((UB *)"S5 scan %s: WARNING %u reading(s) after a gap too "
                   "long to bridge looked like the obstacle again (missed "
                   "echoes or the floor), so the edge only counts as a "
                   "lower bound\n", s5_scan_side_name(b_high_side),
                   (unsigned int)p_stats->came_back);
        }
    }
    else
    {
        S5_LOG((UB *)"S5 scan %s: %u angles (%u failed), out to %u deg "
               "off ahead; no edge up to the %u deg limit\n",
               s5_scan_side_name(b_high_side),
               (unsigned int)p_stats->angles, (unsigned int)p_stats->missed,
               (unsigned int)p_stats->furthest,
               (unsigned int)(b_high_side ?
                              (S5_FINE_MAX_ANGLE - S5_FORWARD_DEG) :
                              (S5_FORWARD_DEG - S5_FINE_MIN_ANGLE)));
    }
}

/*
 * Centring check, for setting SERVO_TRIM_DEGREES. Only when both edges
 * were found. If the obstacle is centred on the car, its two edges are
 * equally far off straight ahead, so any difference means the servo's 90
 * degrees does not point straight ahead: it points half the difference
 * toward the side whose edge came sooner. Each edge is taken halfway
 * between its last-on and first-off angle; the sum of those two angles is
 * kept (x 2) so no half degree is lost before the final rounding. A
 * misaligned servo pushes one edge out past S5_FLAT_ECHO_MAX_DEG, so an
 * edge that is only a lower bound for that reason is allowed, with a
 * warning: a narrower box (both edges measured) gives a reading that can
 * be trusted. An edge the readings came back past (missed echoes) says
 * nothing about the servo, so then there is no check at all.
 */
static void s5_scan_print_centring (const s5_profile_t * const p_profile)
{
    if ((0u != s5_scan_stats(false)->came_back) ||
        (0u != s5_scan_stats(true)->came_back))
    {
        S5_LOG((UB *)"S5 centring check: skipped, an edge is in doubt "
               "(missed echoes)\n");
    }
    else if ((0u != p_profile->b_valid) && (0u != p_profile->b_left_edge) &&
             (0u != p_profile->b_right_edge))
    {
        const char * const p_note = ((0u != p_profile->b_left_exact) &&
                                     (0u != p_profile->b_right_exact)) ? "" :
                                    " (one edge is only a lower bound: "
                                    "check with a narrower box)";
        /* Casts are safe: each angle is at most 90. */
        const int32_t high_x2 = (int32_t)s5_profile_off_axis_deg(
                                    s5_profile_on_angle(true)) +
                                (int32_t)s5_profile_off_axis_deg(
                                    s5_profile_off_angle(true));
        const int32_t low_x2  = (int32_t)s5_profile_off_axis_deg(
                                    s5_profile_on_angle(false)) +
                                (int32_t)s5_profile_off_axis_deg(
                                    s5_profile_off_angle(false));
        /* Trim (commanded degrees) that would move 90 onto the middle of
         * the two edges: (high - low) / 2 = (high_x2 - low_x2) / 4,
         * rounded to the nearest degree. */
        const int32_t diff_x4 = high_x2 - low_x2;
        const int32_t trim    = (diff_x4 >= 0) ? ((diff_x4 + 2) / 4) :
                                                 -((2 - diff_x4) / 4);

        if (0 == trim)
        {
            S5_LOG((UB *)"S5 centring check: both edges equally far off "
                   "ahead (servo centred, if this obstacle is centred)%s\n",
                   p_note);
        }
        else
        {
            /* The servo's 90 points toward the side with the SMALLER edge
             * angle, which is the high side when trim < 0. */
            S5_LOG((UB *)"S5 centring check: if this obstacle is centred "
                   "on the car, the servo's 90 deg points %d deg to the "
                   "%s: set SERVO_TRIM_DEGREES to %d (now %d)%s\n",
                   (int)((trim < 0) ? -trim : trim),
                   s5_scan_side_name(trim < 0),
                   (int)(trim + SERVO_TRIM_DEGREES),
                   (int)SERVO_TRIM_DEGREES, p_note);
        }
    }
}

/*
 * Coarse pass of one side: step outward S5_FINE_STEP_DEG at a time all the
 * way to the scan limit, also after the edge is found, so both sides always
 * cover the same angles. A short gap that comes back onto the obstacle is a
 * missed echo and is bridged (the edge moves on out, s5_profile_add); after
 * a longer gap the edge is final, and a reading that looks like the
 * obstacle again is only counted (came_back). A failed angle is skipped
 * (and counted in g_stats).
 */
static void s5_scan_coarse (const bool b_high_side)
{
    const uint8_t        max_steps = b_high_side ? FINE_HIGH_STEPS :
                                                   FINE_LOW_STEPS;
    side_stats_t * const p_stats   = s5_scan_stats(b_high_side);

    for (uint8_t step = 1u; step <= max_steps; step++)
    {
        /* Casts are safe: step stays within FINE_LOW_STEPS or
         * FINE_HIGH_STEPS, so the angle stays inside
         * S5_FINE_MIN_ANGLE..S5_FINE_MAX_ANGLE (0..180). */
        const uint16_t offset_deg  = (uint16_t)(step * S5_FINE_STEP_DEG);
        const uint16_t angle_deg   = b_high_side ?
                                     (uint16_t)(S5_FORWARD_DEG + offset_deg) :
                                     (uint16_t)(S5_FORWARD_DEG - offset_deg);
        uint16_t       distance_cm = S5_OBSTACLE_NONE_CM;
        const bool     b_read      = s5_scan_read(angle_deg, &distance_cm);
        const char *   p_mark      = MARK_NONE;

        if (b_read)
        {
            switch (s5_profile_add(angle_deg, distance_cm))
            {
                case S5_READING_EDGE:
                    p_mark = MARK_EDGE;
                break;
                case S5_READING_PAST:
                    p_mark = MARK_PAST;
                break;
                case S5_READING_BRIDGED:
                    p_stats->bridged = (uint8_t)(p_stats->bridged + 1u);
                    p_mark = MARK_BRIDGED;
                break;
                case S5_READING_BACK_IGNORED:
                    p_stats->came_back = (uint8_t)(p_stats->came_back + 1u);
                    p_mark = MARK_BACK;
                break;
                case S5_READING_ON:
                default:
                    /* Still on the obstacle. */
                break;
            }
        }
        s5_scan_print_reading(angle_deg, b_read, distance_cm, p_mark);
    }
}

/*
 * Refine pass of one side (only once its edge is found): step outward
 * S5_FINE_REFINE_DEG at a time from the last coarse angle on the obstacle,
 * strictly before the first one off it, until a reading leaves the
 * obstacle. A failed angle is skipped (and counted in g_stats).
 */
static void s5_scan_refine (const bool b_high_side)
{
    const uint16_t off_deg = s5_profile_off_angle(b_high_side);
    uint16_t       angle   = s5_profile_on_angle(b_high_side);
    bool           b_on    = true;

    while (b_on)
    {
        /* Casts are safe: angle stays between the two scanned angles,
         * inside 0..180. */
        angle = b_high_side ? (uint16_t)(angle + S5_FINE_REFINE_DEG) :
                              (uint16_t)(angle - S5_FINE_REFINE_DEG);

        if (b_high_side ? (angle >= off_deg) : (angle <= off_deg))
        {
            b_on = false;   /* reached the coarse off angle: edge pinned */
        }
        else
        {
            uint16_t   distance_cm = S5_OBSTACLE_NONE_CM;
            const bool b_read      = s5_scan_read(angle, &distance_cm);

            if (b_read)
            {
                b_on = s5_profile_refine(angle, distance_cm);
            }
            s5_scan_print_reading(angle, b_read, distance_cm,
                                  (b_read && (!b_on)) ? MARK_EDGE :
                                                        MARK_NONE);
        }
    }
}

/* Scan one side: coarse, then (if its edge was found) refine, and print
 * what it covered. Returns how many angles got no reading. */
static uint8_t s5_scan_side (const bool b_high_side)
{
    side_stats_t * const p_stats = s5_scan_stats(b_high_side);

    p_stats->angles    = 0u;
    p_stats->missed    = 0u;
    p_stats->furthest  = 0u;
    p_stats->bridged   = 0u;
    p_stats->came_back = 0u;

    s5_scan_coarse(b_high_side);
    if (s5_profile_edge_found(b_high_side))
    {
        s5_scan_refine(b_high_side);
    }
    s5_scan_print_side(b_high_side);

    return p_stats->missed;
}

s5_bypass_t s5_scan_obstacle (s5_profile_t * const p_profile)
{
    uint16_t center_cm = S5_OBSTACLE_NONE_CM;
    uint16_t ahead_cm  = S5_OBSTACLE_NONE_CM;
    bool     b_scan_ok = false;

    s5_guard_set_hold(true);
    S5_LOG((UB *)"S5 fine scan: profiling the obstacle\n");
    s5_log_event("S5_FINE_SCAN", "START");

    /* Let the car come to rest (Subsystem 2 ramps down), still pinging. */
    s5_sense_watch_for(S5_LOOK_AHEAD, S5_FINE_START_DELAY_MS);

    if (s5_sense_fine_reading(&center_cm))
    {
        s5_scan_print_reading(S5_FORWARD_DEG, true, center_cm, MARK_NONE);
        s5_profile_start(center_cm);

        /* One side completely, then the other. A few missed angles are
         * skipped; too many and the profile cannot be trusted. The cast is
         * safe: at most a few dozen angles are scanned. */
        uint8_t missed = s5_scan_side(false);

        missed    = (uint8_t)(missed + s5_scan_side(true));
        b_scan_ok = (missed <= S5_FINE_MAX_MISSED);
        if (!b_scan_ok)
        {
            S5_LOG((UB *)"S5 fine scan: %u angles got no reading (max %u)\n",
                   (unsigned int)missed, (unsigned int)S5_FINE_MAX_MISSED);
        }
    }

    s5_sense_servo_to(S5_FORWARD_DEG, S5_LOOK_AHEAD);
    s5_profile_finish(b_scan_ok, p_profile);
    s5_scan_print_centring(p_profile);
    s5_log_event("S5_FINE_SCAN", b_scan_ok ? "DONE" : "FAILED");

    /* Is the obstacle still in the way? A failed reading counts as yes, and
     * so does no echo at all: the car has just stopped for something close,
     * and from close up the HC-SR04 often gets no echo, so "nothing seen"
     * means "too close to measure", not "gone". Only a real echo further
     * away than S5_STILL_BLOCKED_CM means the obstacle has moved. */
    bool b_blocked = ((!s5_sense_fine_reading(&ahead_cm)) ||
                      (S5_OBSTACLE_NONE_CM == ahead_cm) ||
                      (ahead_cm <= S5_STILL_BLOCKED_CM));

    s5_bypass_t decision = s5_avoid_plan(p_profile, b_blocked);

    s5_guard_set_hold(false);
    return decision;
}
