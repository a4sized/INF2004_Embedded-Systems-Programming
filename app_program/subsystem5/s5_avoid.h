/**
 * @file    s5_avoid.h
 * @brief   Subsystem 5, stage 4: bypass decision from the obstacle profile.
 *
 * After each fine scan the scanning task calls s5_avoid_plan(), which
 * decides, in this order:
 *   CONTINUE  the path ahead is clear again (the obstacle moved);
 *   STOP      the scan failed, so there is nothing safe to go on;
 *   LEFT /    a side whose free gap is at least S5_BYPASS_NEED_CM (car
 *   RIGHT     width + margin). If both sides fit, the side with the shorter
 *             detour (smaller extent from the centre line) wins, so the car
 *             leaves the line as little as possible;
 *   REVERSE   neither side fits: back off and scan again, up to
 *             S5_BYPASS_MAX_REVERSES times in a row;
 *   STOP      neither side fits and the reverses are used up.
 * The reverse count restarts after any CONTINUE, LEFT or RIGHT, and for
 * every new obstacle (s5_avoid_new_obstacle()).
 *
 * The decision stays published until the next fine scan. With
 * S5_DRIVE_BYPASS = 1 Subsystem 5 also carries it out (s5_drive.c);
 * otherwise whoever drives the car does.
 */
#ifndef S5_AVOID_H
#define S5_AVOID_H

#include <stdint.h>
#include <stdbool.h>
#include "s5_profile.h"

typedef enum
{
    S5_BYPASS_NONE = 0,   /* no fine scan done yet                   */
    S5_BYPASS_CONTINUE,   /* path ahead clear: carry on along line   */
    S5_BYPASS_LEFT,       /* go round the obstacle on its left       */
    S5_BYPASS_RIGHT,      /* go round the obstacle on its right      */
    S5_BYPASS_REVERSE,    /* both sides blocked: back off, scan again */
    S5_BYPASS_STOP,       /* give up: stay stopped                   */
    S5_BYPASS_COUNT       /* number of values (not a decision)       */
} s5_bypass_t;

/* Pure decision, no side effects (used by s5_avoid_plan and the tests).
 * p_profile      latest profile (NULL or invalid = scan failed)
 * b_path_blocked true if the car is still stopped for the obstacle
 * reverse_count  REVERSE decisions in a row so far */
s5_bypass_t s5_avoid_choose (const s5_profile_t * const p_profile,
                             const bool b_path_blocked,
                             const uint8_t reverse_count);

/* Decide, publish, print and log the bypass for a finished fine scan.
 * Call only from the scanning task. Returns the decision. */
s5_bypass_t s5_avoid_plan (const s5_profile_t * const p_profile,
                           const bool b_path_blocked);

/* The car got going again after an obstacle: the next obstacle starts
 * with a fresh reverse count. Call only from the scanning task. */
void s5_avoid_new_obstacle (void);

/* Latest published decision. May be called from any task. */
s5_bypass_t s5_avoid_get (void);

#endif /* S5_AVOID_H */
