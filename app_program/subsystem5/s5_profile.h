/**
 * @file    s5_profile.h
 * @brief   Subsystem 5: obstacle profile from the fine scan.
 *
 * When the car stops for an obstacle, the scanning task fine-scans outward
 * from straight ahead (90 degrees), one side at a time, and feeds every
 * reading in here. The profile records:
 *   - closest point:   distance and servo angle of the nearest reading
 *   - location:        lateral distance from the car's centre line to the
 *                      obstacle's left and right edges
 *   - width:           left extent + right extent
 *   - left / right clearance: sideways free gap between each edge and the
 *                      next thing beside the obstacle (a wall, another
 *                      obstacle), i.e. the room the car has to pass. Only
 *                      things up to S5_BYPASS_DEPTH_CM beyond the
 *                      obstacle's front count: anything further ahead does
 *                      not block the way round.
 *
 * An edge is where the reading jumps by more than S5_EDGE_JUMP_CM compared
 * with the last point on the obstacle (or nothing echoes back). The scan of
 * a side stops at its edge (s5_profile_edge_found()), so only the first
 * reading past the edge can show something beside the obstacle; if it shows
 * nothing, that side's clearance is "open" (511).
 *
 * Call s5_profile_start/add/finish only from the scanning task.
 * s5_profile_get() and s5_profile_lateral_cm() may be called from any task.
 */
#ifndef S5_PROFILE_H
#define S5_PROFILE_H

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>

/* Minimum-bit obstacle profile: 1 + 8 + 7 x 9 = 63 bits, one 64-bit word.
 * All distances are in cm on the telemetry scale (0..510 cm). The other
 * fields are only meaningful while b_valid is 1. */
typedef struct
{
    uint64_t b_valid         : 1;   /* 1 = profile from a completed scan   */
    uint64_t closest_angle   : 8;   /* 0..180 servo degrees, 90 = ahead     */
    uint64_t closest_cm      : 9;   /* nearest point on the obstacle        */
    uint64_t width_cm        : 9;   /* left extent + right extent           */
    uint64_t left_extent_cm  : 9;   /* centre line -> left edge, sideways   */
    uint64_t right_extent_cm : 9;   /* centre line -> right edge, sideways  */
    uint64_t left_clear_cm   : 9;   /* free gap; 511 = nothing seen beside, */
    uint64_t right_clear_cm  : 9;   /*   0 = no edge (blocked to the limit) */
} s5_profile_t;

/* Begin a new profile. center_cm is the reading straight ahead (90 deg). */
void s5_profile_start (const uint16_t center_cm);

/* Add one fine-scan reading (angle_deg != 90, distance in cm,
 * S5_OBSTACLE_NONE_CM if nothing echoed back). */
void s5_profile_add (const uint16_t angle_deg, const uint16_t distance_cm);

/* True once the fine scan of one side (b_high_side = angles above 90) has
 * passed the obstacle's edge: a reading jumped more than S5_EDGE_JUMP_CM
 * further than the last point on the obstacle, or nothing echoed back.
 * The obstacle is not there any more, so that side need not be scanned
 * further. Call only from the scanning task. */
bool s5_profile_edge_found (const bool b_high_side);

/* Work out width, extents and clearances and publish the profile.
 * b_scan_ok = false publishes an invalid profile (scan failed). */
void s5_profile_finish (const bool b_scan_ok);

/* Copy the latest published profile. Returns its b_valid flag. */
bool s5_profile_get (s5_profile_t * const p_profile);

/* Sideways distance (cm) from the car's centre line to a point seen
 * distance_cm away at servo angle angle_deg. Pure function (tests). */
uint16_t s5_profile_lateral_cm (const uint16_t distance_cm,
                                const uint16_t angle_deg);

#endif /* S5_PROFILE_H */
