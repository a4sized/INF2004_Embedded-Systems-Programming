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
 *   The latest distance (cm, 9 bits) at each scan position is kept, and the
 *   nearest one (in cm) is sent with s1_set_obstacle() after every reading,
 *   so telemetry always shows the closest thing the scanner currently sees.
 *   After the start-up scan only the straight-ahead position is kept
 *   (s5_report_keep_only), since the sensor then stays facing forward.
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

/* Latest distance per scan position, in cm, 9 bits each (0..510 cm,
 * 511 = nothing seen, same scale as telemetry obstacle_cm).
 * 5 x 9 = 45 bits packed into one 64-bit word with masks and shifts.
 * Only the scanning task touches it, so no lock. */
#define CM_FIELD_BITS           (9u)
#define CM_FIELD_MASK           (0x1FFu)
#define CM_MAX_REAL             (S5_OBSTACLE_NONE_CM - 1u)   /* 510 cm */
#define CM_ALL_NONE             ((1ull << (S5_SCAN_POSITIONS * \
                                           CM_FIELD_BITS)) - 1ull)

_Static_assert((S5_SCAN_POSITIONS * CM_FIELD_BITS) <= 64u,
               "scan distances do not fit in 64 bits");
_Static_assert(CM_FIELD_MASK == S5_OBSTACLE_NONE_CM,
               "9-bit field must match the telemetry 'none' value");

static volatile us_health_t g_health    = { 0u, 0u, 0u };
static uint64_t             g_latest_cm = CM_ALL_NONE;

/* Bit position of one scan position's 9-bit field. */
static uint32_t s5_field_shift (const uint8_t position)
{
    return ((uint32_t)position * CM_FIELD_BITS);   /* widening: always fits */
}

/* Read one 9-bit distance field. The cast is safe: the mask leaves at most
 * 9 bits. */
static uint16_t s5_latest_cm_get (const uint8_t position)
{
    return (uint16_t)((g_latest_cm >> s5_field_shift(position)) &
                      CM_FIELD_MASK);
}

/* Write one 9-bit distance field, leaving the others untouched. The casts
 * only widen to 64 bits before shifting, so nothing is lost. */
static void s5_latest_cm_set (const uint8_t position, const uint16_t cm)
{
    uint32_t shift = s5_field_shift(position);

    g_latest_cm = (g_latest_cm & ~((uint64_t)CM_FIELD_MASK << shift)) |
                  ((uint64_t)(cm & CM_FIELD_MASK) << shift);
}

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
            tm_printf((UB *)"S5 ultrasonic recovered: "
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
            tm_printf((UB *)"S5 ultrasonic fault: FAULT_US_SENS reported\n");
            s1_csv_log_event("S5_FAULT", "US_SENS");
        }
    }
}

void s5_report_scanner_failed (void)
{
    g_health.fail_streak = S5_US_FAIL_LIMIT;
    g_health.good_streak = 0u;
    g_health.b_faulted   = 1u;
    s1_report_fault(FAULT_US_SENS);
    tm_printf((UB *)"S5 scanner hardware failed: FAULT_US_SENS reported\n");
    s1_csv_log_event("S5_FAULT", "SCANNER_INIT");
}

void s5_report_keep_only (const uint8_t keep_position)
{
    uint64_t keep_mask = 0u;   /* invalid position: forget everything */

    if (keep_position < S5_SCAN_POSITIONS)
    {
        /* Cast only widens to 64 bits before shifting. */
        keep_mask = (uint64_t)CM_FIELD_MASK << s5_field_shift(keep_position);
    }

    g_latest_cm = (g_latest_cm & keep_mask) | (CM_ALL_NONE & ~keep_mask);
}

bool s5_report_is_faulted (void)
{
    return (0u != g_health.b_faulted);
}

uint16_t s5_report_distance (const uint8_t position,
                             const uint16_t distance_mm)
{
    uint16_t cm_now     = S5_OBSTACLE_NONE_CM;
    uint16_t nearest_cm = S5_OBSTACLE_NONE_CM;

    /* Convert this reading to the 9-bit cm scale. */
    if (NO_OBSTACLE != distance_mm)
    {
        /* Cast is safe: dividing a 16-bit value cannot make it bigger. */
        cm_now = (uint16_t)(distance_mm / S5_MM_PER_CM);
        if (cm_now > CM_MAX_REAL)
        {
            cm_now = CM_MAX_REAL;   /* clip, keep 511 for "none" */
        }
    }

    if (position < S5_SCAN_POSITIONS)
    {
        s5_latest_cm_set(position, cm_now);
    }

    for (uint8_t index = 0u; index < S5_SCAN_POSITIONS; index++)
    {
        uint16_t field_cm = s5_latest_cm_get(index);

        if (field_cm < nearest_cm)
        {
            nearest_cm = field_cm;
        }
    }

    s1_set_obstacle((INT)nearest_cm);   /* 0..511 always fits in INT */
    return nearest_cm;
}
