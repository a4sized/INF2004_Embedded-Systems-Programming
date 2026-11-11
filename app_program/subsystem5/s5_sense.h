/**
 * @file    s5_sense.h
 * @brief   Subsystem 5: taking readings and turning the servo.
 *
 * Shared by the scanning task's modules (subsystem5.c, s5_scan.c,
 * s5_bypass.c). Call only from the scanning task.
 */
#ifndef S5_SENSE_H
#define S5_SENSE_H

#include <stdbool.h>
#include <stdint.h>
#include "s5_config.h"

/* Where a reading is taken, which decides what it is used for. */
typedef enum
{
    S5_LOOK_AHEAD = 0,  /* straight ahead: telemetry and, with the servo
                         * exactly at S5_FORWARD_DEG, the obstacle guard   */
    S5_LOOK_SIDE        /* anywhere else (fine scan, looking at the obstacle
                         * while driving past it): sensor health only      */
} s5_look_t;

/* Take one reading looking 'look' (see above). b_print = true also prints it.
 * If p_distance_mm is not NULL it receives the distance in mm (NO_OBSTACLE
 * if nothing echoed). Returns true if a good reading was taken. A failed
 * reading is never treated as "no obstacle", so it cannot release a stop. */
bool s5_sense_reading (const s5_look_t look, const bool b_print,
                       uint16_t * const p_distance_mm);

/* Wait duration_ms, pinging every S5_PING_INTERVAL_MS meanwhile. */
void s5_sense_watch_for (const s5_look_t look, const uint32_t duration_ms);

/* Turn the servo gently to target_degrees (pinging on the way), let it
 * settle, and retry until it gets there. The angle is limited to the fine
 * scan's S5_FINE_MIN_ANGLE..S5_FINE_MAX_ANGLE: the servo never turns
 * further, also while bypassing. */
void s5_sense_servo_to (const uint16_t target_degrees, const s5_look_t look);

/* S5_LOOK_AHEAD if the servo faces straight ahead, else S5_LOOK_SIDE. */
s5_look_t s5_sense_look (void);

/* S5_FINE_SAMPLES pings at the current angle (with a few retries for failed
 * ones); *p_distance_cm gets their median in cm (S5_OBSTACLE_NONE_CM if
 * nothing echoed). Returns false if no ping succeeded. */
bool s5_sense_fine_reading (uint16_t * const p_distance_cm);

#endif /* S5_SENSE_H */
