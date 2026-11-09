/**
 * @file    s5_avoid.c
 * @brief   Subsystem 5, stage 4: bypass decision from the obstacle profile.
 *
 * See s5_avoid.h for the rules.
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
#include "s5_log.h"
#include "s5_profile.h"
#include "s5_avoid.h"

#define REVERSE_COUNT_MAX   (3u)   /* largest value a 2-bit field holds */
#define DECISION_MAX        (7u)   /* largest value a 3-bit field holds */

_Static_assert(S5_BYPASS_MAX_REVERSES <= REVERSE_COUNT_MAX,
               "reverse count needs > 2 bits");
_Static_assert(S5_BYPASS_COUNT <= (DECISION_MAX + 1u),
               "bypass decision needs > 3 bits");

/* Minimum-bit decision state: 3 + 2 = 5 bits in one byte. Written only by
 * the scanning task. s5_avoid_get() may be called from another task: a
 * byte load is atomic on the RP2040 and there is one writer, so a reader
 * always sees a whole value. */
typedef struct
{
    uint8_t decision      : 3;   /* s5_bypass_t                      */
    uint8_t reverse_count : 2;   /* REVERSE decisions in a row so far */
} avoid_state_t;

/* Decision names for the console and the CSV log, in s5_bypass_t order. */
static const char * const g_bypass_names[S5_BYPASS_COUNT] =
{
    "NONE", "CONTINUE", "LEFT", "RIGHT", "REVERSE", "STOP"
};

static volatile avoid_state_t g_avoid = { S5_BYPASS_NONE, 0u };

/* Print the free gap beside the obstacle on one side, against the room the
 * car needs (S5_BYPASS_NEED_CM). */
static void s5_avoid_print_gap (const char * const p_side,
                                const uint16_t clear_cm)
{
    const char * const p_fits = (clear_cm >= S5_BYPASS_NEED_CM) ?
                                "fits" : "too narrow";

    if (S5_OBSTACLE_NONE_CM == clear_cm)
    {
        tm_printf((UB *)"S5   gap %s: open, nothing beside it (fits)\n",
                  p_side);
    }
    else
    {
        tm_printf((UB *)"S5   gap %s: %u cm, car needs %u cm (%s)\n",
                  p_side, (unsigned int)clear_cm,
                  (unsigned int)S5_BYPASS_NEED_CM, p_fits);
    }
}

/* Both sides have room: take the side where the obstacle sticks out
 * least past the car's centre line (the smaller extent, i.e. the shorter
 * detour back to the line). On a tie take the wider free gap, then the
 * left. */
static s5_bypass_t s5_avoid_pick_side (const s5_profile_t * const p_profile)
{
    s5_bypass_t choice = S5_BYPASS_LEFT;

    if (p_profile->right_extent_cm < p_profile->left_extent_cm)
    {
        choice = S5_BYPASS_RIGHT;
    }
    else if ((p_profile->right_extent_cm == p_profile->left_extent_cm) &&
             (p_profile->right_clear_cm > p_profile->left_clear_cm))
    {
        choice = S5_BYPASS_RIGHT;
    }
    else
    {
        /* Left sticks out less (or the same, with no less room). */
    }

    return choice;
}

/* The obstacle is still in the way and the profile is valid: go round on a
 * side with room, or reverse, or give up. */
static s5_bypass_t s5_avoid_from_gaps (const s5_profile_t * const p_profile,
                                       const uint8_t reverse_count)
{
    bool        b_left_fits  = (p_profile->left_clear_cm >= S5_BYPASS_NEED_CM);
    bool        b_right_fits = (p_profile->right_clear_cm >=
                                S5_BYPASS_NEED_CM);
    s5_bypass_t decision     = S5_BYPASS_STOP;

    if (b_left_fits && b_right_fits)
    {
        decision = s5_avoid_pick_side(p_profile);
    }
    else if (b_left_fits)
    {
        decision = S5_BYPASS_LEFT;
    }
    else if (b_right_fits)
    {
        decision = S5_BYPASS_RIGHT;
    }
    else if (reverse_count < S5_BYPASS_MAX_REVERSES)
    {
        decision = S5_BYPASS_REVERSE;
    }
    else
    {
        /* No room either side and no reverses left: stay stopped. */
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
        decision = s5_avoid_from_gaps(p_profile, reverse_count);
    }

    return decision;
}

s5_bypass_t s5_avoid_plan (const s5_profile_t * const p_profile,
                           const bool b_path_blocked)
{
    s5_bypass_t decision = s5_avoid_choose(p_profile, b_path_blocked,
                                           g_avoid.reverse_count);

    if (S5_BYPASS_REVERSE == decision)
    {
        g_avoid.reverse_count++;   /* < S5_BYPASS_MAX_REVERSES <= 3 */
    }
    else if (S5_BYPASS_STOP != decision)
    {
        g_avoid.reverse_count = 0u;   /* got past (or no longer blocked) */
    }
    else
    {
        /* STOP: keep the count, so the car does not reverse forever. */
    }

    /* Cast is safe: every s5_bypass_t value is below S5_BYPASS_COUNT,
     * which the static assert above keeps within 3 bits. */
    g_avoid.decision = (uint8_t)decision;

    /* Always printed, also in the clearance test, with the free gap beside
     * the obstacle on each side (the room to drive round it) that the
     * decision was based on. */
    tm_printf((UB *)"S5 bypass decision: %s\n", g_bypass_names[decision]);
    if ((NULL != p_profile) && (0u != p_profile->b_valid))
    {
        s5_avoid_print_gap("left", p_profile->left_clear_cm);
        s5_avoid_print_gap("right", p_profile->right_clear_cm);
    }
    s5_log_event("S5_BYPASS", g_bypass_names[decision]);

    return decision;
}

void s5_avoid_new_obstacle (void)
{
    g_avoid.reverse_count = 0u;
}

s5_bypass_t s5_avoid_get (void)
{
    /* Cast is safe: only s5_avoid_plan() writes the field, and it only
     * stores s5_bypass_t values. */
    return (s5_bypass_t)g_avoid.decision;
}
