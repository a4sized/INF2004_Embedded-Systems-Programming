/**
 * @file    s5_guard.h
 * @brief   Subsystem 5 -> Subsystem 2: obstacle stop / slow-down guard.
 *
 * Turns the nearest obstacle distance into Subsystem 2's ultrasonic speed
 * cap (s2_set_us_clearance), which overrides every other speed request:
 *     gap <= S5_STOP_TRIGGER_CM   cap 0                  (stop)
 *     gap <= S5_SLOW_DISTANCE_CM  cap S5_SLOW_SPEED_CAP  (slow down)
 *     otherwise                   cap S5_FULL_SPEED_CAP  (no limit)
 * where gap = nearest distance - S5_SENSOR_TO_FRONT_CM.
 * Once stopped, the car stays stopped until the gap is wider than
 * S5_RESUME_GAP_CM, so it does not creep forward on noisy readings.
 *
 * Hold: while the sensor is not watching the path (start-up scan, fine
 * scan), the cap is forced to 0 so the car cannot move.
 */
#ifndef S5_GUARD_H
#define S5_GUARD_H

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>

/* Pure decision, no side effects (used by the guard and by the tests).
 * nearest_cm    nearest obstacle seen by the sensor, S5_OBSTACLE_NONE_CM
 *               if none
 * b_was_stopped guard state before this reading
 * p_b_stopped   guard state after this reading (may be NULL)
 * Returns the speed cap for Subsystem 2 (0..255). */
uint8_t s5_guard_speed_cap (const uint16_t nearest_cm,
                            const bool b_was_stopped,
                            bool * const p_b_stopped);

/* Apply a new nearest distance: update the state and, if the cap changed,
 * send it to Subsystem 2. Call only from the scanning task. */
void s5_guard_update (const uint16_t nearest_cm);

/* True while the guard is holding the car stopped for an obstacle. */
bool s5_guard_is_stopped (void);

/* b_hold = true: send cap 0 now and keep it there (sensor busy scanning).
 * b_hold = false: stop forcing; the next s5_guard_update() sends the normal
 * cap. Call only from subsystem5_start() (before the scanning task exists)
 * or from the scanning task. */
void s5_guard_set_hold (const bool b_hold);

#endif /* S5_GUARD_H */
