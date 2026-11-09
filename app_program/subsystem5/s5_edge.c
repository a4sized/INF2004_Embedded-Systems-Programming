/**
 * @file    s5_edge.c
 * @brief   Finds where an obstacle ENDS while the car drives along it.
 *
 * See s5_edge.h.  The end is not final: if the obstacle is seen beside the
 * car again, s5_edge_feed() reports "not ended" once more, so the caller
 * keeps driving instead of turning in next to it.
 *
 * Casts in this file: (uint16_t) widens a sum that always fits (both terms
 * are at most 511).
 */
#include <stdbool.h>
#include <stdint.h>
#include "s5_edge.h"
#include "s5_config.h"

#define EDGE_COUNT_MAX      (255u)   /* largest value g_clear_count holds */

static bool     g_b_have_face = false; /* the first reading has been taken  */
static uint16_t g_face_cm     = 0u;    /* how far away the face is (0=none) */
static uint8_t  g_clear_count = 0u;    /* readings in a row without it      */
static uint32_t g_seen_ticks  = 0u;    /* move progress when last seen      */
static bool     g_b_found     = false; /* true = the obstacle has ended     */
static bool     g_b_wait      = false; /* obstacle not yet beside the car   */
static uint32_t g_wait_limit  = 0u;    /* give up waiting after this        */

void s5_edge_start (const bool b_wait_for_obstacle,
                    const uint32_t wait_limit_ticks)
{
    g_b_wait      = b_wait_for_obstacle;
    g_wait_limit  = wait_limit_ticks;
    g_b_have_face = false;
    g_face_cm     = 0u;
    g_clear_count = 0u;
    g_seen_ticks  = 0u;
    g_b_found     = false;
}

/* True if an echo distance_cm away is the obstacle beside the car: close to
 * the face first seen, or (no face seen yet) within S5_EDGE_NEAR_CM. */
static bool s5_edge_is_obstacle (const uint16_t distance_cm)
{
    bool b_obstacle = false;

    if (distance_cm >= S5_OBSTACLE_NONE_CM)
    {
        /* Nothing echoed back. */
    }
    else if (0u != g_face_cm)
    {
        b_obstacle = (distance_cm <=
                      (uint16_t)(g_face_cm + S5_EDGE_TOL_CM));
    }
    else
    {
        b_obstacle = (distance_cm <= S5_EDGE_NEAR_CM);
    }

    return b_obstacle;
}

bool s5_edge_feed (const bool b_good, const uint16_t distance_cm,
                   const uint32_t moved_ticks)
{
    const bool b_echo = (distance_cm < S5_OBSTACLE_NONE_CM);

    if (!b_good)
    {
        /* Failed reading: says nothing, ignore it. */
    }
    else if ((!g_b_have_face) && g_b_wait && (!b_echo))
    {
        /* Still waiting for the obstacle to come into view beside the car;
         * give up once it has not appeared within the limit. */
        if ((!g_b_found) && (moved_ticks >= g_wait_limit))
        {
            g_b_found    = true;
            g_seen_ticks = moved_ticks;
        }
    }
    else if (!g_b_have_face)
    {
        g_b_have_face = true;
        if (b_echo)
        {
            g_face_cm     = distance_cm;
            g_clear_count = 0u;
            g_seen_ticks  = moved_ticks;
            g_b_found     = false;
        }
        else
        {
            /* Nothing at the side at all: the car looks clear already. */
            g_face_cm     = 0u;
            g_clear_count = 1u;
            g_seen_ticks  = 0u;
        }
    }
    else if (s5_edge_is_obstacle(distance_cm))
    {
        /* Obstacle beside the car (again): it has not ended.  Follow its
         * face: if the car's heading is a little off after a turn, the
         * distance to the face drifts as the car drives, and comparing with
         * the first reading only would end the obstacle too early. */
        g_face_cm     = distance_cm;
        g_clear_count = 0u;
        g_seen_ticks  = moved_ticks;
        g_b_found     = false;
    }
    else if (g_clear_count < EDGE_COUNT_MAX)
    {
        g_clear_count++;
    }
    else
    {
        /* Saturated: nothing more to count. */
    }

    if (g_clear_count >= S5_EDGE_CONFIRM)
    {
        g_b_found = true;
    }

    return g_b_found;
}

uint32_t s5_edge_last_seen_ticks (void)
{
    return g_seen_ticks;
}

bool s5_edge_was_seen (void)
{
    return (0u != g_face_cm);
}
