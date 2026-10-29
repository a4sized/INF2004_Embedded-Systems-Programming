/**
 * @file    s5_profile.c
 * @brief   Subsystem 5: obstacle profile from the fine scan.
 *
 * See s5_profile.h for what is measured. Geometry: a point seen d cm away
 * at servo angle a is |90 - a| degrees off the centre line, so it lies
 * d x sin(|90 - a|) cm to the side. sin() comes from a Q14 table (no
 * floating point; the RP2040 has no FPU).
 *
 * Sides: angles below 90 are the "low" side, above 90 the "high" side.
 * S5_HIGH_ANGLE_IS_LEFT maps them to left and right.
 *
 * Every distance is clipped to 0..511 on the way in, so it always fits the
 * 9-bit fields.
 *
 * Casts in this file: (UB *) only retypes a string literal for the T-Kernel
 * tm_printf() API; (unsigned int) widens a bit-field that always fits. The
 * other casts are commented where they are made.
 */
#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>
#include "../app.h"
#include "s5_config.h"
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

#define SIN_TABLE_STEP_DEG  (5u)
#define SIN_TABLE_SIZE      (19u)     /* 0, 5, ..., 90 degrees */
#define Q14_SHIFT           (14u)
#define Q14_HALF            (8192u)   /* 0.5 in Q14, for rounding */
#define STEPS_PAST_MAX      (3u)      /* largest value a 2-bit field holds */

_Static_assert(sizeof(s5_profile_t) == sizeof(uint64_t),
               "s5_profile_t must pack into one 64-bit word");
_Static_assert(0u == (S5_FINE_STEP_DEG % SIN_TABLE_STEP_DEG),
               "fine step must be a multiple of the sin table step");
_Static_assert(0u == (S5_EDGE_TRIM_DEG % SIN_TABLE_STEP_DEG),
               "edge trim must be a multiple of the sin table step");
_Static_assert(S5_CLEAR_STEPS <= STEPS_PAST_MAX,
               "steps past the edge need > 2 bits");

/* Minimum-bit working state for one side: 9 + 8 + 9 + 2 + 1 = 29 bits. */
typedef struct
{
    uint32_t last_on_cm   : 9;   /* last reading that was on the obstacle  */
    uint32_t edge_angle   : 8;   /* angle of that reading                  */
    uint32_t clear_cm     : 9;   /* nearest reading past the edge          */
    uint32_t steps_past   : 2;   /* readings taken past the edge           */
    uint32_t b_edge_found : 1;   /* 1 = a reading jumped off the obstacle  */
} side_state_t;

/* sin(0, 5, ..., 90 degrees) x 16384. */
static const uint16_t g_sin_q14[SIN_TABLE_SIZE] =
{
        0u,  1428u,  2845u,  4240u,  5604u,  6924u,  8192u,  9397u, 10531u,
    11585u, 12551u, 13421u, 14189u, 14849u, 15396u, 15826u, 16135u, 16322u,
    16384u
};

/* Scanning task only: no lock needed. */
static side_state_t g_side[SIDE_COUNT];
static s5_profile_t g_work;

/* Read by other tasks: copied with interrupts disabled (a 64-bit copy is
 * not atomic on the Cortex-M0+). */
static s5_profile_t g_published;

/* Clip a distance to the 9-bit field range (511 = nothing seen). */
static uint16_t s5_profile_clip_cm (const uint16_t distance_cm)
{
    uint16_t clipped_cm = distance_cm;

    if (clipped_cm > S5_OBSTACLE_NONE_CM)
    {
        clipped_cm = S5_OBSTACLE_NONE_CM;
    }

    return clipped_cm;
}

uint16_t s5_profile_lateral_cm (const uint16_t distance_cm,
                                const uint16_t angle_deg)
{
    uint16_t off_axis_deg = 0u;

    /* Casts are safe: each subtraction is guarded to be positive. */
    if (angle_deg < S5_FORWARD_DEG)
    {
        off_axis_deg = (uint16_t)(S5_FORWARD_DEG - angle_deg);
    }
    else
    {
        off_axis_deg = (uint16_t)(angle_deg - S5_FORWARD_DEG);
    }

    /* Move the edge inward to undo the sensor's beam spread (tuned). */
    if (off_axis_deg > S5_EDGE_TRIM_DEG)
    {
        off_axis_deg = (uint16_t)(off_axis_deg - S5_EDGE_TRIM_DEG);
    }
    else
    {
        off_axis_deg = 0u;
    }

    uint16_t index = (uint16_t)(off_axis_deg / SIN_TABLE_STEP_DEG);

    if (index >= SIN_TABLE_SIZE)
    {
        index = (uint16_t)(SIN_TABLE_SIZE - 1u);   /* 90 deg off the axis */
    }

    /* Widened to 32 bits: 511 x 16384 + 8192 fits. After the rounding
     * shift the result is at most distance_cm, so the cast back to 16 bits
     * is safe. */
    return (uint16_t)((((uint32_t)distance_cm * g_sin_q14[index]) +
                       Q14_HALF) >> Q14_SHIFT);
}

void s5_profile_start (const uint16_t center_cm)
{
    uint16_t start_cm = s5_profile_clip_cm(center_cm);

    g_work.b_valid       = 0u;
    g_work.closest_angle = S5_FORWARD_DEG;
    g_work.closest_cm    = start_cm;

    for (uint8_t side = 0u; side < SIDE_COUNT; side++)
    {
        g_side[side].last_on_cm   = start_cm;
        g_side[side].edge_angle   = S5_FORWARD_DEG;
        g_side[side].clear_cm     = S5_OBSTACLE_NONE_CM;
        g_side[side].steps_past   = 0u;
        g_side[side].b_edge_found = 0u;
    }
}

/* A reading still on the obstacle: move the edge out to it, and keep the
 * closest point. */
static void s5_profile_on_obstacle (const uint8_t side,
                                    const uint16_t angle_deg,
                                    const uint16_t distance_cm)
{
    g_side[side].last_on_cm = distance_cm;
    g_side[side].edge_angle = angle_deg;

    if (distance_cm < g_work.closest_cm)
    {
        g_work.closest_cm    = distance_cm;
        g_work.closest_angle = angle_deg;
    }
}

/* A reading past the edge: count it and keep the nearest one. */
static void s5_profile_past_edge (const uint8_t side,
                                  const uint16_t distance_cm)
{
    g_side[side].b_edge_found = 1u;

    if (g_side[side].steps_past < STEPS_PAST_MAX)
    {
        g_side[side].steps_past++;
    }

    if (distance_cm < g_side[side].clear_cm)
    {
        g_side[side].clear_cm = distance_cm;
    }
}

void s5_profile_add (const uint16_t angle_deg, const uint16_t distance_cm)
{
    uint8_t  side    = (angle_deg > S5_FORWARD_DEG) ? SIDE_HIGH : SIDE_LOW;
    uint16_t clip_cm = s5_profile_clip_cm(distance_cm);

    if ((0u != g_side[side].b_edge_found) ||
        (clip_cm > (g_side[side].last_on_cm + S5_EDGE_JUMP_CM)))
    {
        s5_profile_past_edge(side, clip_cm);   /* at or past the edge */
    }
    else
    {
        s5_profile_on_obstacle(side, angle_deg, clip_cm);
    }
}

bool s5_profile_wants_more (const bool b_high_side)
{
    uint8_t side = b_high_side ? SIDE_HIGH : SIDE_LOW;

    return ((0u == g_side[side].b_edge_found) ||
            (g_side[side].steps_past < S5_CLEAR_STEPS));
}

/* Sideways distance from the centre line to one side's edge. The real
 * edge lies between the last angle on the obstacle and the next angle
 * (off it), so take the halfway point. If no edge was found before the
 * scan limit, the obstacle reaches at least to the last angle. */
static uint16_t s5_profile_extent (const uint8_t side)
{
    uint16_t on_cm    = g_side[side].last_on_cm;
    uint16_t on_angle = g_side[side].edge_angle;
    uint16_t extent   = s5_profile_lateral_cm(on_cm, on_angle);

    if (0u != g_side[side].b_edge_found)
    {
        /* Casts are safe: the next angle stays inside 0..180 because a
         * reading was taken there. */
        uint16_t off_angle = (SIDE_HIGH == side) ?
                             (uint16_t)(on_angle + S5_FINE_STEP_DEG) :
                             (uint16_t)(on_angle - S5_FINE_STEP_DEG);
        uint16_t off_cm    = s5_profile_lateral_cm(on_cm, off_angle);

        extent = (uint16_t)(((extent + off_cm) + 1u) / 2u);   /* rounded */
    }

    return extent;
}

/* Clearance past one side's edge: 0 if no edge was found before the scan
 * limit (the obstacle may carry on), otherwise the nearest reading. */
static uint16_t s5_profile_clearance (const uint8_t side)
{
    uint16_t clear_cm = 0u;

    if (0u != g_side[side].b_edge_found)
    {
        clear_cm = g_side[side].clear_cm;
    }

    return clear_cm;
}

/* Print a finished profile on the console. */
static void s5_profile_print (const s5_profile_t * const p_profile)
{
    if (0u == p_profile->b_valid)
    {
        tm_printf((UB *)"S5 profile: scan failed, no profile\n");
    }
    else
    {
        tm_printf((UB *)"S5 profile: closest %u cm at %u deg, "
                  "width %u cm (left %u, right %u)\n",
                  (unsigned int)p_profile->closest_cm,
                  (unsigned int)p_profile->closest_angle,
                  (unsigned int)p_profile->width_cm,
                  (unsigned int)p_profile->left_extent_cm,
                  (unsigned int)p_profile->right_extent_cm);
        tm_printf((UB *)"S5 profile: clearance left %u cm, right %u cm "
                  "(0 = blocked, %u = open)\n",
                  (unsigned int)p_profile->left_clear_cm,
                  (unsigned int)p_profile->right_clear_cm,
                  (unsigned int)S5_OBSTACLE_NONE_CM);
    }
}

void s5_profile_finish (const bool b_scan_ok)
{
    UINT interrupt_status = 0u;

    g_work.b_valid = b_scan_ok ? 1u : 0u;

    if (b_scan_ok)
    {
        uint16_t left_cm  = s5_profile_extent(SIDE_LEFT);
        uint16_t right_cm = s5_profile_extent(SIDE_RIGHT);
        uint16_t width_cm = (uint16_t)(left_cm + right_cm);   /* <= 1022 */

        if (width_cm > S5_OBSTACLE_MAX_CM)
        {
            width_cm = S5_OBSTACLE_MAX_CM;
        }

        g_work.width_cm        = width_cm;
        g_work.left_extent_cm  = left_cm;
        g_work.right_extent_cm = right_cm;
        g_work.left_clear_cm   = s5_profile_clearance(SIDE_LEFT);
        g_work.right_clear_cm  = s5_profile_clearance(SIDE_RIGHT);
    }

    DI(interrupt_status);
    g_published = g_work;
    EI(interrupt_status);

    s5_profile_print(&g_work);
}

bool s5_profile_get (s5_profile_t * const p_profile)
{
    bool b_valid          = false;
    UINT interrupt_status = 0u;

    if (NULL != p_profile)
    {
        DI(interrupt_status);
        *p_profile = g_published;
        EI(interrupt_status);
        b_valid = (0u != p_profile->b_valid);
    }

    return b_valid;
}
