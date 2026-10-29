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
 *   - left / right clearance: nearest thing seen beyond each edge
 *
 * An edge is where the reading jumps by more than S5_EDGE_JUMP_CM compared
 * with the last point on the obstacle (or nothing echoes back).
 *
 * Call s5_profile_start/add/wants_more/finish only from the scanning task.
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
    uint64_t left_clear_cm   : 9;   /* 511 = nothing seen past the edge,    */
    uint64_t right_clear_cm  : 9;   /*   0 = no edge (blocked to the limit) */
} s5_profile_t;

/* Begin a new profile. center_cm is the reading straight ahead (90 deg). */
void s5_profile_start (const uint16_t center_cm);

/* Add one fine-scan reading (angle_deg != 90, distance in cm,
 * S5_OBSTACLE_NONE_CM if nothing echoed back). */
void s5_profile_add (const uint16_t angle_deg, const uint16_t distance_cm);

/* True while the given side still needs readings: its edge has not been
 * found yet, or fewer than S5_CLEAR_STEPS readings were taken past it.
 * b_high_side = true for angles above 90, false for angles below 90. */
bool s5_profile_wants_more (const bool b_high_side);

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
