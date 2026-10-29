/**
 * @file    s5_guard.c
 * @brief   Subsystem 5 -> Subsystem 2: obstacle stop / slow-down guard.
 *
 * See s5_guard.h for the rules. Subsystem 5 is the only writer of
 * Subsystem 2's ultrasonic speed cap, and only this file writes it.
 * The cap is sent only when it changes, to keep Subsystem 2's mutex free.
 *
 * Casts in this file: (UB *) only retypes a string literal for the T-Kernel
 * tm_printf() API; (unsigned int) widens a value that always fits.
 */
#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>
#include "../app.h"
#include "../subsystem1/subsystem1.h"
#include "../subsystem2/subsystem2.h"
#include "s5_config.h"
#include "s5_guard.h"

_Static_assert(S5_FULL_SPEED_CAP <= 255u, "speed cap needs > 8 bits");

/* Minimum-bit guard state: 8 + 1 + 1 + 1 = 11 bits in one 16-bit word.
 * Written only by the scanning task (and once by subsystem5_start() before
 * that task exists). s5_guard_is_stopped() may be called from another task:
 * a 16-bit aligned load is atomic on the RP2040 and there is one writer, so
 * a reader always sees a whole value. */
typedef struct
{
    uint16_t last_cap   : 8;   /* 0..255, last cap sent to Subsystem 2     */
    uint16_t b_stopped  : 1;   /* 1 = holding the car for an obstacle      */
    uint16_t b_cap_sent : 1;   /* 1 = a cap has been sent at least once    */
    uint16_t b_hold     : 1;   /* 1 = start-up scan running: cap forced 0  */
} guard_state_t;

static volatile guard_state_t g_guard = { S5_FULL_SPEED_CAP, 0u, 0u, 0u };

uint8_t s5_guard_speed_cap (const uint16_t nearest_cm,
                            const bool b_was_stopped,
                            bool * const p_b_stopped)
{
    uint16_t gap_cm    = 0u;
    bool     b_stopped = b_was_stopped;
    uint8_t  speed_cap = S5_FULL_SPEED_CAP;

    /* Distance from the front of the car, not from the sensor. The cast is
     * safe: the if guarantees the difference is positive. */
    if (nearest_cm > S5_SENSOR_TO_FRONT_CM)
    {
        gap_cm = (uint16_t)(nearest_cm - S5_SENSOR_TO_FRONT_CM);
    }

    if (b_stopped)
    {
        if (gap_cm > S5_RESUME_GAP_CM)
        {
            b_stopped = false;   /* obstacle moved away or was removed */
        }
    }
    else if (gap_cm <= S5_STOP_TRIGGER_CM)
    {
        b_stopped = true;
    }
    else
    {
        /* Still clear. */
    }

    if (b_stopped)
    {
        speed_cap = 0u;
    }
    else if (gap_cm <= S5_SLOW_DISTANCE_CM)
    {
        speed_cap = S5_SLOW_SPEED_CAP;
    }
    else
    {
        /* No obstacle near: keep S5_FULL_SPEED_CAP. */
    }

    if (NULL != p_b_stopped)
    {
        *p_b_stopped = b_stopped;
    }

    return speed_cap;
}

void s5_guard_update (const uint16_t nearest_cm)
{
    bool    b_was_stopped = (0u != g_guard.b_stopped);
    bool    b_now_stopped = false;
    uint8_t speed_cap     = s5_guard_speed_cap(nearest_cm, b_was_stopped,
                                               &b_now_stopped);

    if (0u != g_guard.b_hold)
    {
        speed_cap = 0u;   /* start-up scan not finished: do not move off */
    }

    if (b_now_stopped != b_was_stopped)
    {
        if (b_now_stopped)
        {
            tm_printf((UB *)"S5 obstacle %u cm: STOP\n",
                      (unsigned int)nearest_cm);
            s1_csv_log_event("S5_OBSTACLE", "STOP");
        }
        else
        {
            tm_printf((UB *)"S5 obstacle clear (%u cm): resume\n",
                      (unsigned int)nearest_cm);
            s1_csv_log_event("S5_OBSTACLE", "CLEAR");
        }
        g_guard.b_stopped = b_now_stopped ? 1u : 0u;
    }

    if ((0u == g_guard.b_cap_sent) || (speed_cap != g_guard.last_cap))
    {
        s2_set_us_clearance(speed_cap);
        g_guard.last_cap   = speed_cap;
        g_guard.b_cap_sent = 1u;
    }
}

bool s5_guard_is_stopped (void)
{
    return (0u != g_guard.b_stopped);
}

void s5_guard_set_startup_hold (const bool b_hold)
{
    if (b_hold)
    {
        s2_set_us_clearance(0u);
        g_guard.last_cap   = 0u;
        g_guard.b_cap_sent = 1u;
        g_guard.b_hold     = 1u;
    }
    else
    {
        g_guard.b_hold = 0u;   /* next s5_guard_update() sends the real cap */
    }
}
