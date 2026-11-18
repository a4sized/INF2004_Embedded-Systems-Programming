/**
 * @file    s5_avoid.c
 * @brief   Subsystem 5, stage 4: bypass decision from the obstacle profile.
 *
 * See s5_avoid.h for the rules.
 *
 * Casts in this file: (UB *) only retypes a string literal for the T-Kernel
 * tm_printf() API; (unsigned int) widens a value that always fits; (int32_t)
 * widens settings and Q14 values (all far below 2^31) before signed maths.
 */
#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>
#include "../app.h"
#include "s5_config.h"
#include "s5_log.h"
#include "s5_profile.h"
#include "s5_avoid.h"

_Static_assert((S5_OBSTACLE_MIN_WIDTH_CM <= S5_OBSTACLE_MAX_WIDTH_CM) &&
               (S5_OBSTACLE_MAX_WIDTH_CM < S5_OBSTACLE_NONE_CM),
               "obstacle width range must be 0 < min <= max < 511 cm");

/* Decision names for the console and the CSV log, in s5_bypass_t order. */
static const char * const g_bypass_names[S5_BYPASS_COUNT] =
{
    "CONTINUE", "LEFT", "RIGHT", "REVERSE", "STOP"
};

/* REVERSE decisions in a row so far. Scanning task only. */
static uint8_t g_reverse_count = 0u;

uint16_t s5_avoid_backup_mm (const uint16_t extent_cm,
                             const uint16_t ahead_cm)
{
    /* Worked in mm with sin / cos in Q14 (no floating point). Turning out
     * on the spot about the rear axle swings the sensor axle_mm x sin(turn)
     * sideways and axle_mm x (1 - cos(turn)) back. Then, to clear the edge,
     * it still needs lat_mm sideways, which on the slant costs
     * lat_mm x cos / sin of forward travel. When it clears the edge the
     * obstacle's front must still be more than S5_OUT_LEG_MIN_CM ahead of
     * it; any shortfall is how far the car must first reverse. Largest
     * products: 5110 x 16384 (about 84 million), so int32 is enough. */
    const int32_t sin_q14 = (int32_t)s5_profile_sin_q14(S5_BYPASS_TURN_DEG);
    const int32_t cos_q14 = (int32_t)s5_profile_sin_q14(
                                90u - S5_BYPASS_TURN_DEG);
    const int32_t axle_mm = (int32_t)(S5_SENSOR_TO_AXLE_CM * S5_MM_PER_CM);
    const int32_t lat_mm  = ((int32_t)extent_cm * (int32_t)S5_MM_PER_CM) -
                            ((axle_mm * sin_q14) >> 14);
    int32_t       short_mm = 0;   /* sensor already clear after turning */

    if (lat_mm > 0)
    {
        const int32_t front_mm = ((int32_t)ahead_cm * (int32_t)S5_MM_PER_CM) +
                                 ((axle_mm * (16384 - cos_q14)) >> 14) -
                                 ((lat_mm * cos_q14) / sin_q14);

        /* +1: the front must be MORE than the minimum away. */
        short_mm = ((int32_t)(S5_OUT_LEG_MIN_CM * S5_MM_PER_CM) + 1) -
                   front_mm;
    }

    /* Casts are safe: clipped to 0..65535 first. */
    return (short_mm <= 0) ? 0u :
           ((short_mm > 65535) ? (uint16_t)65535u : (uint16_t)short_mm);
}

bool s5_avoid_side_fits (const uint16_t extent_cm, const uint16_t ahead_cm)
{
    return (0u == s5_avoid_backup_mm(extent_cm, ahead_cm));
}

uint16_t s5_avoid_plan_backup_mm (const s5_profile_t * const p_profile,
                                  const bool b_left)
{
    /* The cast is safe: a 9-bit field fits 16 bits. */
    return s5_avoid_backup_mm(s5_avoid_plan_extent(p_profile, b_left),
                              (uint16_t)p_profile->closest_cm);
}

uint16_t s5_avoid_plan_extent (const s5_profile_t * const p_profile,
                               const bool b_left)
{
    /* Casts are safe: 9-bit fields fit 16 bits. */
    const uint16_t own_cm   = b_left ? (uint16_t)p_profile->left_extent_cm :
                                       (uint16_t)p_profile->right_extent_cm;
    const uint16_t other_cm = b_left ? (uint16_t)p_profile->right_extent_cm :
                                       (uint16_t)p_profile->left_extent_cm;
    const bool     b_exact  = b_left ? (0u != p_profile->b_left_exact) :
                                       (0u != p_profile->b_right_exact);
    /* The width is MIN..MAX, so this side's edge is at most MAX - other
     * and at least MIN - other from the centre line. A measured edge is
     * kept, but never closer than MIN allows; a lower bound (or no edge)
     * is taken at the furthest MAX allows. The other side's extent is
     * itself at least the real one, so both limits err on the wide side.
     * The casts are safe: each subtraction is guarded to be positive. */
    const uint16_t limit_cm = b_exact ?
        ((other_cm < S5_OBSTACLE_MIN_WIDTH_CM) ?
         (uint16_t)(S5_OBSTACLE_MIN_WIDTH_CM - other_cm) : 0u) :
        ((other_cm < S5_OBSTACLE_MAX_WIDTH_CM) ?
         (uint16_t)(S5_OBSTACLE_MAX_WIDTH_CM - other_cm) : 0u);

    return (own_cm > limit_cm) ? own_cm : limit_cm;
}

/* True if at least one edge was found, so the width range pins both
 * planned edges down. */
static bool s5_avoid_any_edge (const s5_profile_t * const p_profile)
{
    return ((0u != p_profile->b_left_edge) ||
            (0u != p_profile->b_right_edge));
}

/* True if the car can go round the obstacle on the left (b_left) or right:
 * an edge was found, and it fits round the planned edge after reversing no
 * more than S5_BYPASS_MAX_BACKUP_MM (0 if it fits from where it is). */
static bool s5_avoid_side_ok (const s5_profile_t * const p_profile,
                              const bool b_left)
{
    return (s5_avoid_any_edge(p_profile) &&
            (s5_avoid_plan_backup_mm(p_profile, b_left) <=
             S5_BYPASS_MAX_BACKUP_MM));
}

/* The car fits round both sides: the side whose planned edge is closer to
 * the centre line (shorter detour); on a tie a MEASURED edge before a lower
 * bound, then the left. */
static s5_bypass_t s5_avoid_better_side (const s5_profile_t * const p_profile)
{
    const uint16_t left_cm  = s5_avoid_plan_extent(p_profile, true);
    const uint16_t right_cm = s5_avoid_plan_extent(p_profile, false);
    s5_bypass_t    choice   = S5_BYPASS_LEFT;

    if (right_cm < left_cm)
    {
        choice = S5_BYPASS_RIGHT;
    }
    else if ((right_cm == left_cm) && (0u == p_profile->b_left_exact) &&
             (0u != p_profile->b_right_exact))
    {
        choice = S5_BYPASS_RIGHT;
    }
    else
    {
        /* Left is closer, or a tie the right does not win. */
    }

    return choice;
}

/* The obstacle is still in the way and the profile is valid. No edge found
 * at all: its width is not known, so reverse and scan again while reverses
 * are left, then stay stopped. Otherwise there is never anything else
 * beside it, so the car goes round any side it fits round (reversing first
 * if it must, s5_avoid_plan_backup_mm), the better one if both; if neither
 * fits even after reversing S5_BYPASS_MAX_BACKUP_MM, stay stopped (scanning
 * again would not change its width). */
static s5_bypass_t s5_avoid_from_edges (const s5_profile_t * const p_profile,
                                        const uint8_t reverse_count)
{
    const bool  b_left_ok  = s5_avoid_side_ok(p_profile, true);
    const bool  b_right_ok = s5_avoid_side_ok(p_profile, false);
    s5_bypass_t decision   = S5_BYPASS_STOP;

    if (!s5_avoid_any_edge(p_profile))
    {
        if (reverse_count < S5_BYPASS_MAX_REVERSES)
        {
            decision = S5_BYPASS_REVERSE;
        }
    }
    else if (b_left_ok && b_right_ok)
    {
        decision = s5_avoid_better_side(p_profile);
    }
    else if (b_left_ok)
    {
        decision = S5_BYPASS_LEFT;
    }
    else if (b_right_ok)
    {
        decision = S5_BYPASS_RIGHT;
    }
    else
    {
        /* Too big to get round even after reversing: stay stopped and wait
         * for the obstacle to be removed. */
    }

    return decision;
}

s5_bypass_t s5_avoid_choose (const s5_profile_t * const p_profile,
                             const bool b_path_blocked,
                             const uint8_t reverse_count)
{
    s5_bypass_t decision = S5_BYPASS_STOP;

    if (!b_path_blocked)
    {
        decision = S5_BYPASS_CONTINUE;
    }
    else if ((NULL == p_profile) || (0u == p_profile->b_valid))
    {
        /* Scan failed: nothing safe to go on, stay stopped. */
    }
    else
    {
        decision = s5_avoid_from_edges(p_profile, reverse_count);
    }

    return decision;
}

s5_bypass_t s5_avoid_plan (const s5_profile_t * const p_profile,
                           const bool b_path_blocked)
{
    s5_bypass_t decision = s5_avoid_choose(p_profile, b_path_blocked,
                                           g_reverse_count);

    if (S5_BYPASS_REVERSE == decision)
    {
        g_reverse_count++;   /* at most S5_BYPASS_MAX_REVERSES */
    }
    else if (S5_BYPASS_STOP != decision)
    {
        g_reverse_count = 0u;   /* got past (or no longer blocked) */
    }
    else
    {
        /* STOP: keep the count, so the car does not reverse forever. */
    }

    /* The decision is always printed, also in the clearance test. */
    tm_printf((UB *)"S5 bypass decision: %s\n", g_bypass_names[decision]);
    if ((NULL != p_profile) && (0u != p_profile->b_valid))
    {
        S5_RESULT_LOG((UB *)"S5   planned edges: left %u cm (reverse %u "
                      "mm first), right %u cm (reverse %u mm first)\n",
                      (unsigned int)s5_avoid_plan_extent(p_profile, true),
                      (unsigned int)s5_avoid_plan_backup_mm(p_profile, true),
                      (unsigned int)s5_avoid_plan_extent(p_profile, false),
                      (unsigned int)s5_avoid_plan_backup_mm(p_profile,
                                                            false));
    }
    s5_log_event("S5_BYPASS", g_bypass_names[decision]);

    return decision;
}

void s5_avoid_new_obstacle (void)
{
    g_reverse_count = 0u;
}
