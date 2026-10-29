/**
 * @file    s5_report.h
 * @brief   Subsystem 5 -> Subsystem 1: fault and obstacle telemetry.
 *
 * Subsystem 1 owns the one shared fault list and the telemetry packet.
 * Subsystem 5 never writes them itself; it only calls Subsystem 1's API:
 *     s1_report_fault(FAULT_US_SENS) / s1_clear_fault(FAULT_US_SENS)
 *     s1_set_obstacle(cm)
 *
 * Call these only from the scanning task (they are not re-entrant), except
 * s5_report_scanner_failed(), which is for start-up.
 */
#ifndef S5_REPORT_H
#define S5_REPORT_H

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>

/* One reading worked: counts toward clearing FAULT_US_SENS. */
void s5_report_sensor_ok (void);

/* One reading (or servo move) failed: counts toward FAULT_US_SENS. */
void s5_report_sensor_error (void);

/* Scanner hardware did not start: report FAULT_US_SENS at once. It stays
 * set because the scanning task (which would clear it) never runs. */
void s5_report_scanner_failed (void);

/* Record the latest GOOD distance at one scan position, send the nearest
 * obstacle over all positions to telemetry, and return it in cm
 * (S5_OBSTACLE_NONE_CM if nothing is seen). NO_OBSTACLE = nothing seen.
 * Never call it for a failed reading. */
uint16_t s5_report_distance (const uint8_t position,
                             const uint16_t distance_mm);

/* Forget the distances at every scan position except keep_position (set
 * them to "nothing seen"). Used after the start-up scan, when the sensor
 * stays facing ahead and old side readings must not hold the car. */
void s5_report_keep_only (const uint8_t keep_position);

/* True while Subsystem 5 has FAULT_US_SENS reported. */
bool s5_report_is_faulted (void);

#endif /* S5_REPORT_H */
