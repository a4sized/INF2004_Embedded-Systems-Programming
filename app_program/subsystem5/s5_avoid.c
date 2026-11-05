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
#include "../subsystem1/subsystem1.h"
#include "s5_config.h"
#include "s5_profile.h"
#include "s5_avoid.h"

#define DECISION_MAX        (7u)   /* largest value a 3-bit field holds */

_Static_assert(S5_BYPASS_COUNT <= (DECISION_MAX + 1u),
               "bypass decision needs > 3 bits");

/* Minimum-bit decision state: 3 bits in one byte. Written only by the
 * scanning task. s5_avoid_get() may be called from another task: a byte
 * load is atomic on the RP2040 and there is one writer, so a reader always
 * sees a whole value. */
typedef struct
{
    uint8_t decision : 3;   /* s5_bypass_t */
} avoid_state_t;

/* Decision names for the console and the CSV log, in s5_bypass_t order. */
static const char * const g_bypass_names[S5_BYPASS_COUNT] =
{
    "NONE", "CONTINUE", "LEFT", "RIGHT", "STOP"
};

static volatile avoid_state_t g_avoid = { S5_BYPASS_NONE };

/* The obstacle is still in the way and the profile is valid: always go
 * round, on the side with the most clearance. On a tie take the shorter
 * detour (smaller extent from the centre line), then the left. */
static s5_bypass_t s5_avoid_pick_side (const s5_profile_t * const p_profile)
{
    s5_bypass_t choice = S5_BYPASS_LEFT;

    if (p_profile->right_clear_cm > p_profile->left_clear_cm)
    {
        choice = S5_BYPASS_RIGHT;
    }
    else if ((p_profile->right_clear_cm == p_profile->left_clear_cm) &&
             (p_profile->right_extent_cm < p_profile->left_extent_cm))
    {
        choice = S5_BYPASS_RIGHT;
    }
    else
    {
        /* Left has more (or equal) clearance and no longer a detour. */
    }

    return choice;
}

s5_bypass_t s5_avoid_choose (const s5_profile_t * const p_profile,
                             const bool b_path_blocked)
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
        decision = s5_avoid_pick_side(p_profile);
    }

    return decision;
}

s5_bypass_t s5_avoid_plan (const s5_profile_t * const p_profile,
                           const bool b_path_blocked)
{
    s5_bypass_t decision = s5_avoid_choose(p_profile, b_path_blocked);
    uint16_t    left_cm  = 0u;
    uint16_t    right_cm = 0u;
    uint16_t    gap_cm   = 0u;

    if (NULL != p_profile)
    {
        left_cm  = p_profile->left_clear_cm;
        right_cm = p_profile->right_clear_cm;
    }

    gap_cm = (S5_BYPASS_RIGHT == decision) ? right_cm : left_cm;

    /* Cast is safe: every s5_bypass_t value is below S5_BYPASS_COUNT,
     * which the static assert above keeps within 3 bits. */
    g_avoid.decision = (uint8_t)decision;

    tm_printf((UB *)"S5 bypass: %s (clearance left %u cm, right %u cm)\n",
              g_bypass_names[decision], (unsigned int)left_cm,
              (unsigned int)right_cm);

    if (((S5_BYPASS_LEFT == decision) || (S5_BYPASS_RIGHT == decision)) &&
        (gap_cm < S5_BYPASS_NEED_CM))
    {
        tm_printf((UB *)"S5 bypass: warning, widest gap %u cm is below "
                  "the %u cm the car needs\n", (unsigned int)gap_cm,
                  (unsigned int)S5_BYPASS_NEED_CM);
    }

    s1_csv_log_event("S5_BYPASS", g_bypass_names[decision]);

    return decision;
}

s5_bypass_t s5_avoid_get (void)
{
    /* Cast is safe: only s5_avoid_plan() writes the field, and it only
     * stores s5_bypass_t values. */
    return (s5_bypass_t)g_avoid.decision;
}
