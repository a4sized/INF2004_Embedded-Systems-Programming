/**
 * @file    s5_profile.c
 * @brief   Subsystem 5: obstacle profile from the fine scan.
 *
 * See s5_profile.h for what is measured. Geometry: the sensor's face is
 * S5_SENSOR_PIVOT_CM out from the servo's axis, which is on the car's
 * centre line, so a point seen d cm away at servo angle a is
 * (d + pivot) x sin(|90 - a|) cm to the side. sin() comes from a Q14 table
 * in whole degrees (no floating point; the RP2040 has no FPU).
 *
 * Sides: angles below 90 are the "low" side, above 90 the "high" side.
 * S5_HIGH_ANGLE_IS_LEFT maps them to left and right.
 *
 * Every distance is clipped to 0..511 on the way in, so it always fits the
 * 9-bit fields.
 *
 * Casts in this file: (UB *) only retypes a string literal for the T-Kernel
 * tm_printf() API; (unsigned int) widens a value that always fits. The
 * other casts are commented where they are made.
 */
#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>
#include "../app.h"
#include "s5_config.h"
#include "s5_log.h"
#include "s5_profile.h"

#define SIDE_LOW            (0u)    /* angles below 90 */
#define SIDE_HIGH           (1u)    /* angles above 90 */
#define SIDE_COUNT          (2u)

#if (0u != S5_HIGH_ANGLE_IS_LEFT)
#define SIDE_LEFT           (SIDE_HIGH)
#define SIDE_RIGHT          (SIDE_LOW)
#else
#define SIDE_LEFT           (SIDE_LOW)
#define SIDE_RIGHT          (SIDE_HIGH)
#endif

#define SIN_TABLE_SIZE      (91u)     /* 0, 1, ..., 90 degrees */
#define Q14_SHIFT           (14u)
#define Q14_HALF            (8192u)   /* 0.5 in Q14, for rounding */

/* Store into a 9-bit cm field / an 8-bit angle field. Every distance is
 * clipped to 0..511 on the way in and every angle is 0..180, so these never
 * change a value; they show the compiler (and the reader) that it fits. */
#define CM9(cm)             ((cm) & 0x1FFu)
#define DEG8(deg)           ((uint8_t)(deg))
_Static_assert(0x1FFu == S5_OBSTACLE_NONE_CM, "cm fields are 9 bits");

_Static_assert(sizeof(s5_profile_t) == sizeof(uint64_t),
               "s5_profile_t must pack into one 64-bit word");

/* Working state for one side: 9 + 8 + 8 + 1 + 1 + 1 + 3 = 31 bits. */
typedef struct
{
    uint32_t last_on_cm   : 9;   /* last reading that was on the obstacle  */
    uint32_t on_angle     : 8;   /* angle of that reading                  */
    uint32_t off_angle    : 8;   /* first angle off the obstacle (if found) */
    uint32_t b_edge_found : 1;   /* 1 = a reading left the obstacle        */
    uint32_t b_locked     : 1;   /* 1 = more than S5_EDGE_BRIDGE_MAX off in
                                  *     a row: the edge is final           */
    uint32_t b_came_back  : 1;   /* 1 = a reading after the edge was final
                                  *     looked like the obstacle: in doubt */
    uint32_t gap          : 3;   /* readings off the obstacle in a row     */
} side_state_t;

_Static_assert(S5_EDGE_BRIDGE_MAX <= 6u, "gap count needs > 3 bits");

/* sin(0, 1, ..., 90 degrees) x 16384, rounded. */
static const uint16_t g_sin_q14[SIN_TABLE_SIZE] =
{
        0u,   286u,   572u,   857u,  1143u,  1428u,  1713u,  1997u,  2280u,
     2563u,  2845u,  3126u,  3406u,  3686u,  3964u,  4240u,  4516u,  4790u,
     5063u,  5334u,  5604u,  5872u,  6138u,  6402u,  6664u,  6924u,  7182u,
     7438u,  7692u,  7943u,  8192u,  8438u,  8682u,  8923u,  9162u,  9397u,
     9630u,  9860u, 10087u, 10311u, 10531u, 10749u, 10963u, 11174u, 11381u,
    11585u, 11786u, 11982u, 12176u, 12365u, 12551u, 12733u, 12911u, 13085u,
    13255u, 13421u, 13583u, 13741u, 13894u, 14044u, 14189u, 14330u, 14466u,
    14598u, 14726u, 14849u, 14968u, 15082u, 15191u, 15296u, 15396u, 15491u,
    15582u, 15668u, 15749u, 15826u, 15897u, 15964u, 16026u, 16083u, 16135u,
    16182u, 16225u, 16262u, 16294u, 16322u, 16344u, 16362u, 16374u, 16382u,
    16384u
};

/* Scanning task only: no lock needed. */
static side_state_t g_side[SIDE_COUNT];
static s5_profile_t g_work;

/* Clip a distance to the 9-bit field range (511 = nothing seen). */
static uint16_t s5_profile_clip_cm (const uint16_t distance_cm)
{
    return (distance_cm > S5_OBSTACLE_NONE_CM) ? S5_OBSTACLE_NONE_CM :
                                                 distance_cm;
}

uint16_t s5_profile_off_axis_deg (const uint16_t angle_deg)
{
    /* Casts are safe: each subtraction is guarded to be positive. */
    return (angle_deg < S5_FORWARD_DEG) ?
           (uint16_t)(S5_FORWARD_DEG - angle_deg) :
           (uint16_t)(angle_deg - S5_FORWARD_DEG);
}

/* SIDE_HIGH for angles above 90, else SIDE_LOW. */
static uint8_t s5_profile_side (const uint16_t angle_deg)
{
    return (angle_deg > S5_FORWARD_DEG) ? SIDE_HIGH : SIDE_LOW;
}

/* The working state of one side (b_high_side = angles above 90). */
static const side_state_t * s5_profile_state (const bool b_high_side)
{
    return &g_side[b_high_side ? SIDE_HIGH : SIDE_LOW];
}

uint16_t s5_profile_sin_q14 (const uint16_t angle_deg)
{
    return g_sin_q14[(angle_deg < SIN_TABLE_SIZE) ? angle_deg :
                     (SIN_TABLE_SIZE - 1u)];
}

uint16_t s5_profile_lateral_cm (const uint16_t distance_cm,
                                const uint16_t angle_deg)
{
    uint16_t off_axis_deg = s5_profile_off_axis_deg(angle_deg);

    /* Move the point inward to undo the sensor's beam spread (tuned). The
     * cast is safe: the subtraction is guarded to be positive. */
    off_axis_deg = (off_axis_deg > S5_EDGE_TRIM_DEG) ?
                   (uint16_t)(off_axis_deg - S5_EDGE_TRIM_DEG) : 0u;

    /* Widened to 32 bits: (511 + 255) x 16384 + 8192 fits. After the
     * rounding shift the result is at most distance + pivot (below 800), so
     * the cast back to 16 bits is safe. */
    return (uint16_t)(((((uint32_t)distance_cm + S5_SENSOR_PIVOT_CM) *
                        s5_profile_sin_q14(off_axis_deg)) + Q14_HALF) >>
                      Q14_SHIFT);
}

void s5_profile_start (const uint16_t center_cm)
{
    uint16_t start_cm = s5_profile_clip_cm(center_cm);

    g_work.b_valid       = 0u;
    g_work.closest_angle = S5_FORWARD_DEG;
    g_work.closest_cm    = CM9(start_cm);

    for (uint8_t side = 0u; side < SIDE_COUNT; side++)
    {
        g_side[side].last_on_cm   = CM9(start_cm);
        g_side[side].on_angle     = S5_FORWARD_DEG;
        g_side[side].off_angle    = S5_FORWARD_DEG;
        g_side[side].b_edge_found = 0u;
        g_side[side].b_locked     = 0u;
        g_side[side].b_came_back  = 0u;
        g_side[side].gap          = 0u;
    }
}

/* True if a reading (clipped cm) is still on the obstacle: not more than
 * S5_EDGE_JUMP_CM further than the last point on it. "No echo" (511) is
 * always off it. */
static bool s5_profile_is_on (const uint8_t side, const uint16_t clip_cm)
{
    return ((clip_cm < S5_OBSTACLE_NONE_CM) &&
            (clip_cm <= (g_side[side].last_on_cm + S5_EDGE_JUMP_CM)));
}

/* A reading on the obstacle: move the edge out to it, keep the closest. */
static void s5_profile_on_obstacle (const uint8_t side,
                                    const uint16_t angle_deg,
                                    const uint16_t clip_cm)
{
    g_side[side].last_on_cm = CM9(clip_cm);
    g_side[side].on_angle   = DEG8(angle_deg);

    if (clip_cm < g_work.closest_cm)
    {
        g_work.closest_cm    = CM9(clip_cm);
        g_work.closest_angle = DEG8(angle_deg);
    }
}

s5_reading_t s5_profile_add (const uint16_t angle_deg,
                              const uint16_t distance_cm)
{
    const uint8_t        side    = s5_profile_side(angle_deg);
    side_state_t * const p_side  = &g_side[side];
    const uint16_t       clip_cm = s5_profile_clip_cm(distance_cm);
    const bool           b_on    = s5_profile_is_on(side, clip_cm);
    s5_reading_t         result  = S5_READING_ON;

    if (0u != p_side->b_locked)
    {
        /* The edge is final (more than S5_EDGE_BRIDGE_MAX readings off in a
         * row: beyond that a flat face may give no echo and the floor may
         * echo, so neither can be trusted to move it). One that looks like
         * the obstacle again puts the edge in doubt. */
        if (b_on)
        {
            p_side->b_came_back = 1u;
            result              = S5_READING_BACK_IGNORED;
        }
        else
        {
            result = S5_READING_PAST;
        }
    }
    else if (b_on)
    {
        /* On the obstacle. If the readings had just left it, they have come
         * back close to where they were before the jump, so that jump was a
         * missed echo, not the edge: the edge has not been reached yet. */
        if (0u != p_side->b_edge_found)
        {
            p_side->b_edge_found = 0u;
            result               = S5_READING_BRIDGED;
        }
        p_side->gap = 0u;
        s5_profile_on_obstacle(side, angle_deg, clip_cm);
    }
    else
    {
        /* Off the obstacle: the first one is the edge (for now). */
        if (0u == p_side->b_edge_found)
        {
            p_side->b_edge_found = 1u;
            p_side->off_angle    = DEG8(angle_deg);
            result               = S5_READING_EDGE;
        }
        else
        {
            result = S5_READING_PAST;
        }
        /* Once gap passes S5_EDGE_BRIDGE_MAX the edge locks and this branch
         * is not reached again, so gap is at most S5_EDGE_BRIDGE_MAX + 1
         * (static assert: fits 3 bits); the mask only shows it fits. */
        p_side->gap = (p_side->gap + 1u) & 0x7u;
        if (p_side->gap > S5_EDGE_BRIDGE_MAX)
        {
            p_side->b_locked = 1u;
        }
    }

    return result;
}

bool s5_profile_edge_found (const bool b_high_side)
{
    return (0u != s5_profile_state(b_high_side)->b_edge_found);
}

uint16_t s5_profile_on_angle (const bool b_high_side)
{
    return (uint16_t)s5_profile_state(b_high_side)->on_angle;
}

uint16_t s5_profile_off_angle (const bool b_high_side)
{
    return (uint16_t)s5_profile_state(b_high_side)->off_angle;
}

bool s5_profile_refine (const uint16_t angle_deg, const uint16_t distance_cm)
{
    const uint8_t  side    = s5_profile_side(angle_deg);
    const uint16_t clip_cm = s5_profile_clip_cm(distance_cm);
    const bool     b_on    = s5_profile_is_on(side, clip_cm);

    if (b_on)
    {
        s5_profile_on_obstacle(side, angle_deg, clip_cm);
    }
    else
    {
        g_side[side].off_angle = DEG8(angle_deg);   /* the edge is closer */
    }

    return b_on;
}

/* Sideways distance from the centre line to one side's edge: halfway
 * between the last angle on the obstacle and the first angle off it (both
 * at the last distance on it). If no edge was found before the scan limit,
 * the obstacle reaches at least to the last angle. The cast is safe: the
 * mean of two 16-bit values fits. */
static uint16_t s5_profile_extent (const uint8_t side)
{
    const uint16_t on_cm  = g_side[side].last_on_cm;
    uint16_t       extent = s5_profile_lateral_cm(on_cm,
                                                  g_side[side].on_angle);

    if (0u != g_side[side].b_edge_found)
    {
        extent = (uint16_t)((extent + s5_profile_lateral_cm(
                                 on_cm, g_side[side].off_angle) + 1u) / 2u);
    }

    return extent;
}

/* True if one side's edge was MEASURED: the reading left the obstacle at an
 * angle where a flat face would still have echoed, and no reading past it
 * looked like the obstacle again, so it really ended. The beam's spread
 * lets the sensor see the face S5_EDGE_TRIM_DEG further out than its axis
 * points, so the limit applies to the trimmed angle (the same one the
 * extent is worked out from). */
static bool s5_profile_exact (const uint8_t side)
{
    return ((0u != g_side[side].b_edge_found) &&
            (0u == g_side[side].b_came_back) &&
            (s5_profile_off_axis_deg(g_side[side].off_angle) <=
             (S5_FLAT_ECHO_MAX_DEG + S5_EDGE_TRIM_DEG)));
}

/* The console note for one side's edge. */
static const char * s5_profile_note (const uint64_t b_edge,
                                     const uint64_t b_exact)
{
    return (0u == b_edge)  ? " (edge not found: at least)" :
           (0u == b_exact) ? " (lower bound: may be wider)" : "";
}

/* Print a finished profile: the width and how far each edge is from the
 * car's centre line (always, also in the clearance test), and the closest
 * point (S5_VERBOSE). */
static void s5_profile_print (const s5_profile_t * const p_profile)
{
    if (0u == p_profile->b_valid)
    {
        tm_printf((UB *)"S5 obstacle width: not measured (scan failed)\n");
    }
    else
    {
        /* The cast is safe: two 9-bit fields add up to less than 1023. */
        const uint16_t width_cm = (uint16_t)(p_profile->left_extent_cm +
                                             p_profile->right_extent_cm);
        const bool     b_odd    = ((width_cm < S5_OBSTACLE_MIN_WIDTH_CM) ||
                                   (width_cm > S5_OBSTACLE_MAX_WIDTH_CM));

        tm_printf((UB *)"S5 obstacle width: %u cm%s (left %u cm%s, right %u "
                  "cm%s from the car's centre line)\n",
                  (unsigned int)width_cm,
                  b_odd ? " (outside the expected range)" : "",
                  (unsigned int)p_profile->left_extent_cm,
                  s5_profile_note(p_profile->b_left_edge,
                                  p_profile->b_left_exact),
                  (unsigned int)p_profile->right_extent_cm,
                  s5_profile_note(p_profile->b_right_edge,
                                  p_profile->b_right_exact));
        S5_LOG((UB *)"S5 profile: closest %u cm at %u deg\n",
               (unsigned int)p_profile->closest_cm,
               (unsigned int)p_profile->closest_angle);
    }
}

void s5_profile_finish (const bool b_scan_ok, s5_profile_t * const p_profile)
{
    g_work.b_valid = b_scan_ok ? 1u : 0u;

    if (b_scan_ok)
    {
        g_work.b_left_edge     = g_side[SIDE_LEFT].b_edge_found;
        g_work.b_right_edge    = g_side[SIDE_RIGHT].b_edge_found;
        g_work.b_left_exact    = s5_profile_exact(SIDE_LEFT) ? 1u : 0u;
        g_work.b_right_exact   = s5_profile_exact(SIDE_RIGHT) ? 1u : 0u;
        g_work.left_extent_cm  = CM9(s5_profile_extent(SIDE_LEFT));
        g_work.right_extent_cm = CM9(s5_profile_extent(SIDE_RIGHT));
    }

    s5_profile_print(&g_work);
    *p_profile = g_work;
}
