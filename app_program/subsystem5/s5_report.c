/**
 * @file    s5_report.c
 * @brief   Subsystem 5 -> Subsystem 1: fault and obstacle telemetry.
 *
 * Faults
 *   Subsystem 5 owns FAULT_US_SENS (Tier 2: Subsystem 2 pauses the car while
 *   it is set and resumes when it clears). It is reported after
 *   S5_US_FAIL_LIMIT failed readings in a row and cleared after
 *   S5_US_RECOVER_LIMIT good readings in a row. While the problem lasts the
 *   report is repeated on every failed reading, so the bit comes back even if
 *   somebody else clears the fault list too early (same rule as Subsystem 4).
 *
 *   "Nothing in range" is a good reading, not a fault. Only a sensor that
 *   does not answer, a stuck echo pin, or a failed servo move counts.
 *
 * Obstacle telemetry
 *   Every good straight-ahead reading is sent with s1_set_obstacle() (cm,
 *   S5_OBSTACLE_NONE_CM = nothing in range).
 *
 * Casts in this file: (UB *) only retypes a string literal for the T-Kernel
 * tm_printf() API. The other casts are commented where they are made.
 */
#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>
#include "../app.h"
#include "../systemFaults.h"
#include "../subsystem1/subsystem1.h"
#include "s5_config.h"
#include "s5_log.h"
#include "s5_report.h"

/* ---- Minimum-bit storage ------------------------------------------------
 * Sensor health: 2 + 2 + 1 = 5 bits in one byte. Streaks never exceed 3.
 * Written only by the scanning task. s5_report_is_faulted() may be called
 * from another task: a byte load is atomic on the RP2040, and there is only
 * one writer, so a reader always sees a whole old or a whole new value. */
#define STREAK_MAX              (3u)    /* largest value a 2-bit field holds */

_Static_assert(S5_US_FAIL_LIMIT <= STREAK_MAX,
               "fail streak needs > 2 bits");
_Static_assert(S5_US_RECOVER_LIMIT <= STREAK_MAX,
               "good streak needs > 2 bits");

typedef struct
{
    uint8_t fail_streak : 2;   /* 0..S5_US_FAIL_LIMIT        */
    uint8_t good_streak : 2;   /* 0..S5_US_RECOVER_LIMIT     */
    uint8_t b_faulted   : 1;   /* 1 = FAULT_US_SENS reported */
} us_health_t;

static volatile us_health_t g_health = { 0u, 0u, 0u };

void s5_report_sensor_ok (void)
{
    g_health.fail_streak = 0u;

    if (0u != g_health.b_faulted)
    {
        if (g_health.good_streak < S5_US_RECOVER_LIMIT)
        {
            g_health.good_streak++;
        }

        if (g_health.good_streak >= S5_US_RECOVER_LIMIT)
        {
            s1_clear_fault(FAULT_US_SENS);
            g_health.b_faulted   = 0u;
            g_health.good_streak = 0u;
            S5_LOG((UB *)"S5 ultrasonic recovered: "
                   "FAULT_US_SENS cleared\n");
        }
    }
}

void s5_report_sensor_error (void)
{
    g_health.good_streak = 0u;

    if (g_health.fail_streak < S5_US_FAIL_LIMIT)
    {
        g_health.fail_streak++;
    }

    if (g_health.fail_streak >= S5_US_FAIL_LIMIT)
    {
        /* Repeat while the problem lasts (cheap, and idempotent in S1). */
        s1_report_fault(FAULT_US_SENS);

        if (0u == g_health.b_faulted)
        {
            g_health.b_faulted = 1u;
            S5_LOG((UB *)"S5 ultrasonic fault: FAULT_US_SENS reported\n");
            s5_log_event("S5_FAULT", "US_SENS");
        }
    }
}

void s5_report_scanner_failed (void)
{
    g_health.fail_streak = S5_US_FAIL_LIMIT;
    g_health.good_streak = 0u;
    g_health.b_faulted   = 1u;
    s1_report_fault(FAULT_US_SENS);
    S5_LOG((UB *)"S5 scanner hardware failed: FAULT_US_SENS reported\n");
    s5_log_event("S5_FAULT", "SCANNER_INIT");
}

bool s5_report_is_faulted (void)
{
    return (0u != g_health.b_faulted);
}

uint16_t s5_report_mm_to_cm (const uint16_t distance_mm)
{
    uint16_t distance_cm = S5_OBSTACLE_NONE_CM;

    if (NO_OBSTACLE != distance_mm)
    {
        /* Cast is safe: dividing a 16-bit value cannot make it bigger. */
        distance_cm = (uint16_t)(distance_mm / S5_MM_PER_CM);
        if (distance_cm > S5_OBSTACLE_MAX_CM)
        {
            distance_cm = S5_OBSTACLE_MAX_CM;   /* clip, keep 511 for "none" */
        }
    }

    return distance_cm;
}

void s5_report_distance (const uint16_t distance_mm)
{
    /* 0..511 always fits in INT. */
    s1_set_obstacle((INT)s5_report_mm_to_cm(distance_mm));
}
