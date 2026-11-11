/**
 * @file    s5_avoid.h
 * @brief   Subsystem 5, stage 4: bypass decision from the obstacle profile.
 *
 * After each fine scan the scanning task calls s5_avoid_plan(), which
 * decides, in this order:
 *   CONTINUE  the path ahead is clear again (the obstacle moved);
 *   STOP      the scan failed, so there is nothing safe to go on;
 *   REVERSE   no edge was found at all, so the width is not known: back
 *             off S5_REVERSE_MM and scan again, up to
 *             S5_BYPASS_MAX_REVERSES times in a row (then STOP);
 *   LEFT /    a side the car FITS round, judged on that side's PLANNED edge
 *   RIGHT     (s5_avoid_plan_extent: the measured edge, or for a lower
 *             bound the furthest the S5_OBSTACLE_MAX_WIDTH_CM width allows),
 *             reversing first if it must, by no more than
 *             S5_BYPASS_MAX_BACKUP_MM (s5_avoid_plan_backup_mm). There is
 *             never anything else beside an obstacle, so that is all it
 *             takes. If both: the planned edge closer to the car's centre
 *             line (shorter detour, less reversing); on a tie a measured
 *             edge, then the left;
 *   STOP      too big to get round even after reversing that far. The car
 *             waits for the obstacle to be removed.
 * The reverse count restarts after any CONTINUE, LEFT or RIGHT, and for
 * every new obstacle (s5_avoid_new_obstacle()).
 *
 * With S5_DRIVE_BYPASS = 1 Subsystem 5 carries the decision out
 * (s5_bypass.c); otherwise it is only printed and logged.
 */
#ifndef S5_AVOID_H
#define S5_AVOID_H

#include <stdint.h>
#include <stdbool.h>
#include "s5_profile.h"

typedef enum
{
    S5_BYPASS_CONTINUE = 0,  /* path ahead clear: carry on along line  */
    S5_BYPASS_LEFT,          /* go round the obstacle on its left      */
    S5_BYPASS_RIGHT,         /* go round the obstacle on its right     */
    S5_BYPASS_REVERSE,       /* edges unknown / no room: back, rescan  */
    S5_BYPASS_STOP,          /* cannot get round: wait for it to go    */
    S5_BYPASS_COUNT          /* number of values (not a decision)      */
} s5_bypass_t;

/* Pure decision, no side effects (used by s5_avoid_plan and the tests).
 * p_profile      latest profile (NULL or invalid = scan failed)
 * b_path_blocked true if the car is still stopped for the obstacle
 * reverse_count  REVERSE decisions in a row so far */
s5_bypass_t s5_avoid_choose (const s5_profile_t * const p_profile,
                             const bool b_path_blocked,
                             const uint8_t reverse_count);

/* True if the car can swing out round an edge extent_cm to the side of
 * its centre line, starting with the obstacle's front ahead_cm in front of
 * the sensor: on the S5_BYPASS_TURN_DEG out leg the sensor must clear the
 * edge while the front is still more than S5_OUT_LEG_MIN_CM away (closer
 * and the car's front corner would hit it). Pure function (tests). The
 * obstacle may still be wider than planned, so the out leg checks this
 * again while driving (s5_bypass.c). */
bool s5_avoid_side_fits (const uint16_t extent_cm, const uint16_t ahead_cm);

/* How far (mm) the car must first reverse for s5_avoid_side_fits to be
 * true; 0 if it already is. Pure function (tests). */
uint16_t s5_avoid_backup_mm (const uint16_t extent_cm,
                             const uint16_t ahead_cm);

/* s5_avoid_backup_mm for one side's planned edge (b_left, else right),
 * from where the car is now (the profile's closest point). Pure. */
uint16_t s5_avoid_plan_backup_mm (const s5_profile_t * const p_profile,
                                  const bool b_left);

/* Where to plan the edge on one side (b_left, else right), in cm from the
 * car's centre line, using the width range S5_OBSTACLE_MIN/MAX_WIDTH_CM:
 * a measured edge as measured (but at least MIN minus the other side); a
 * lower-bound or missing edge as far out as the width allows (MAX minus
 * the other side). The bypass drives out to clear this. Pure function. */
uint16_t s5_avoid_plan_extent (const s5_profile_t * const p_profile,
                               const bool b_left);

/* Decide, print and log the bypass for a finished fine scan.
 * Call only from the scanning task. Returns the decision. */
s5_bypass_t s5_avoid_plan (const s5_profile_t * const p_profile,
                           const bool b_path_blocked);

/* The car got going again after an obstacle: the next obstacle starts
 * with a fresh reverse count. Call only from the scanning task. */
void s5_avoid_new_obstacle (void);

#endif /* S5_AVOID_H */
