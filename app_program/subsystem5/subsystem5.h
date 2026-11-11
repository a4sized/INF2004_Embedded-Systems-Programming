/**
 * @file    subsystem5.h
 * @brief   Subsystem 5: servo-mounted ultrasonic obstacle scanner.
 */
#ifndef SUBSYSTEM5_H
#define SUBSYSTEM5_H

#include <tk/tkernel.h>
#include <stdbool.h>

/* subsystem5_init() (set up the servo and sensor) and subsystem5_start()
 * (hold the car still and start the scanning task) are declared in app.h
 * with the other subsystems' start-up calls. */

/* True once the servo faces the front of the car (there is no start-up
 * sweep), a good straight-ahead reading has been taken, there is no sensor
 * fault (FAULT_US_SENS) and the path ahead is clear (not stopped for an
 * obstacle: that is subsystem5_is_avoiding). */
bool subsystem5_startup_scan_ready (void);

/* True from the moment Subsystem 5 stops the car for an obstacle until it
 * has got round it and back to the line (or the obstacle has gone). Also
 * true while the car is held because it could not get past (until the drive
 * permission is withdrawn, or, on the line, the obstacle is removed).
 * The vehicle controller then pauses the line follower (VC_AVOIDING).
 * Any task. */
bool subsystem5_is_avoiding (void);

/* Once after each bypass that got round the obstacle: true, and
 * *p_b_line_on_left = whether the line is to the car's left (else its
 * right), so the line follower can be handed the car searching toward it
 * (ir_motion_consumer_request_search). Clears it: false until the next
 * bypass. Call when taking the car back from AVOIDING. Any task. */
bool subsystem5_take_line_side (bool * const p_b_line_on_left);

/* Set by the vehicle controller: true while it is in VC_AVOIDING and lets
 * Subsystem 5 drive the bypass through Subsystem 2. Withdrawing it stops
 * any move at once and releases a hold. Any task. */
void subsystem5_set_drive_permission (const bool b_allowed);

#endif /* SUBSYSTEM5_H */
