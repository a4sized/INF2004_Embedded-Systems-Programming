/** @file s5_edge.c @brief Finds where an obstacle ENDS while the car drives along it. */
#include "s5_edge.h"
#include "s5_config.h"

static bool g_b_have_face = false; /* the first reading has been taken     */
static uint16_t g_face_cm = 0u;    /* how far away the face is (0 = none)  */
static uint8_t g_clear_count = 0u; /* readings in a row without it         */
static uint32_t g_seen_ticks = 0u;
static bool g_b_found = false;
static bool g_b_wait = false; /* obstacle not yet beside the car      */
static uint32_t g_wait_limit = 0u;

void s5_edge_start(const bool b_wait_for_obstacle,
                   const uint32_t wait_limit_ticks)
{
    g_b_wait = b_wait_for_obstacle;
    g_wait_limit = wait_limit_ticks;
    g_b_have_face = false;
    g_face_cm = 0u;
    g_clear_count = 0u;
    g_seen_ticks = 0u;
    g_b_found = false;
}

bool s5_edge_feed(const bool b_good, const uint16_t distance_cm,
                  const uint32_t moved_ticks)
{
    const bool b_echo = (distance_cm < S5_OBSTACLE_NONE_CM);

    if (b_good && (!g_b_found))
    {
        if ((!g_b_have_face) && g_b_wait && (!b_echo))
        {
            /* Still waiting for the obstacle to come into view beside the
             * car; give up once it has not appeared within the limit. */
            if (moved_ticks >= g_wait_limit)
            {
                g_b_found = true;
                g_seen_ticks = moved_ticks;
            }
        }
        else if (!g_b_have_face)
        {
            g_b_have_face = true;
            if (b_echo)
            {
                g_face_cm = distance_cm;
                g_clear_count = 0u;
                g_seen_ticks = moved_ticks;
            }
            else
            {
                /* Nothing at the side at all: the car is already clear. */
                g_face_cm = 0u;
                g_clear_count = 1u;
                g_seen_ticks = 0u;
            }
        }
        else if (b_echo && (0u != g_face_cm) &&
                 (distance_cm <= (uint16_t)(g_face_cm + S5_EDGE_TOL_CM)))
        {
            g_clear_count = 0u;
            g_seen_ticks = moved_ticks;
        }
        else if (g_clear_count < 255u)
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
    }

    return g_b_found;
}

uint32_t s5_edge_last_seen_ticks(void)
{
    return g_seen_ticks;
}
