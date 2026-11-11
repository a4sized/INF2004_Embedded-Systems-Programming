/**
 * @file    s5_profile.h
 * @brief   Subsystem 5: obstacle profile from the fine scan.
 *
 * When the car stops for an obstacle, the scanning task fine-scans outward
 * from straight ahead (90 degrees), one side at a time, and feeds every
 * reading in here. There is never anything else beside the obstacle, so
 * only the obstacle itself is measured:
 *   - closest point:  distance and servo angle of the nearest reading;
 *   - extents:        sideways distance from the car's centre line to the
 *                     obstacle's left and right edges (width = their sum);
 *   - per side, how good the extent is:
 *       edge measured     the reading left the obstacle at an angle where
 *                         its face would still have echoed: the real edge;
 *       edge lower bound  it left the obstacle further than
 *                         S5_FLAT_ECHO_MAX_DEG off straight ahead, where a
 *                         flat face stops echoing anyway, or a reading
 *                         looked like the obstacle again after a gap too
 *                         long to bridge, so the obstacle may carry on: it
 *                         is AT LEAST this wide;
 *       no edge           still on the obstacle at the scan limit.
 *
 * Finding an edge (s5_scan.c): coarse steps of S5_FINE_STEP_DEG out to the
 * scan limit. The edge is where a reading jumps more than S5_EDGE_JUMP_CM
 * further than the last point on the obstacle (or nothing echoes back) and
 * the readings do not come back onto it within S5_EDGE_BRIDGE_MAX readings
 * (if they do, the jump was a missed echo). Then S5_FINE_REFINE_DEG steps
 * back through that gap (s5_profile_refine) to pin the edge down. The edge is
 * taken halfway between the last angle on the obstacle and the first angle
 * off it, corrected for the beam's spread (S5_EDGE_TRIM_DEG).
 *
 * Call s5_profile_start/add/refine/finish and the edge queries only from
 * the scanning task. The pure functions (s5_profile_lateral_cm,
 * s5_profile_off_axis_deg, s5_profile_sin_q14) may be called from any
 * task.
 */
#ifndef S5_PROFILE_H
#define S5_PROFILE_H

#include <stdint.h>
#include <stdbool.h>

/* Minimum-bit obstacle profile: 1 + 8 + 9 + 2 + 2 + 2 x 9 = 40 bits, one
 * 64-bit word. All distances are in cm on the telemetry scale (0..510 cm).
 * The other fields are only meaningful while b_valid is 1. */
typedef struct
{
    uint64_t b_valid         : 1;   /* 1 = from a completed scan          */
    uint64_t closest_angle   : 8;   /* 0..180 servo degrees, 90 = ahead    */
    uint64_t closest_cm      : 9;   /* nearest point on the obstacle       */
    uint64_t b_left_edge     : 1;   /* 1 = left edge found within limits   */
    uint64_t b_right_edge    : 1;   /* 1 = right edge found within limits  */
    uint64_t b_left_exact    : 1;   /* 1 = left edge measured, 0 = bound   */
    uint64_t b_right_exact   : 1;   /* 1 = right edge measured, 0 = bound  */
    uint64_t left_extent_cm  : 9;   /* centre line -> left edge, sideways  */
    uint64_t right_extent_cm : 9;   /* centre line -> right edge, sideways */
} s5_profile_t;

/* Begin a new profile. center_cm is the reading straight ahead (90 deg). */
void s5_profile_start (const uint16_t center_cm);

/* What one coarse reading turned out to be (s5_profile_add). "On the
 * obstacle" = no more than S5_EDGE_JUMP_CM further than the last point on
 * it; "no echo" is always off it. */
typedef enum
{
    S5_READING_ON = 0,        /* on the obstacle                            */
    S5_READING_EDGE,          /* first one off it: the edge (for now)       */
    S5_READING_PAST,          /* off it again, past the edge                */
    S5_READING_BRIDGED,       /* back on it within S5_EDGE_BRIDGE_MAX: the
                               * "edge" was a missed echo, scan on          */
    S5_READING_BACK_IGNORED   /* back on it after the edge was final: only
                               * reported, the edge becomes a lower bound   */
} s5_reading_t;

/* Add one coarse fine-scan reading (angle_deg != 90, distance in cm,
 * S5_OBSTACLE_NONE_CM if nothing echoed back) and say what it was. The
 * edge is where the readings leave the obstacle and stay off it for more
 * than S5_EDGE_BRIDGE_MAX readings in a row; a shorter gap that comes back
 * close to the readings before it is a missed echo and is bridged. */
s5_reading_t s5_profile_add (const uint16_t angle_deg,
                             const uint16_t distance_cm);

/* True once the coarse scan of one side (b_high_side = angles above 90) has
 * passed the obstacle's edge. */
bool s5_profile_edge_found (const bool b_high_side);

/* After the edge of a side is found: the last servo angle still on the
 * obstacle and the first one off it. Refinement scans strictly between. */
uint16_t s5_profile_on_angle (const bool b_high_side);
uint16_t s5_profile_off_angle (const bool b_high_side);

/* One refinement reading between those two angles. Returns true if it is
 * still on the obstacle (the edge moves out to it: carry on stepping
 * outward) or false if it is off (the edge is before it: stop). */
bool s5_profile_refine (const uint16_t angle_deg, const uint16_t distance_cm);

/* Work out the extents, print the profile and copy it into *p_profile.
 * b_scan_ok = false gives an invalid profile (scan failed). */
void s5_profile_finish (const bool b_scan_ok, s5_profile_t * const p_profile);

/* Sideways distance (cm) from the car's centre line to a point seen
 * distance_cm away (from the sensor's face) at servo angle angle_deg,
 * allowing for S5_SENSOR_PIVOT_CM and S5_EDGE_TRIM_DEG. Pure function. */
uint16_t s5_profile_lateral_cm (const uint16_t distance_cm,
                                const uint16_t angle_deg);

/* Degrees between servo angle angle_deg and straight ahead (0..90 for
 * angles 0..180). Pure function, any task. */
uint16_t s5_profile_off_axis_deg (const uint16_t angle_deg);

/* sin(angle_deg) x 16384 for 0..90 whole degrees (above 90 counts as 90).
 * Pure function, any task. */
uint16_t s5_profile_sin_q14 (const uint16_t angle_deg);

#endif /* S5_PROFILE_H */
