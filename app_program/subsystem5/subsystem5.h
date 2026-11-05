/**
 * @file    subsystem5.h
 * @brief   Subsystem 5: servo-mounted ultrasonic obstacle scanner.
 */
#ifndef SUBSYSTEM5_H
#define SUBSYSTEM5_H

#include <tk/tkernel.h>
#include <stdbool.h>

/* Set up the servo and ultrasonic sensor (called once at startup). */
ER subsystem5_init (void);

/* Hold the car still and start the scanning task, which runs the start-up
 * scan (called after every subsystem's init). */
ER subsystem5_start (void);

/* True only after the first sweep, a good forward reading and a clear path. */
bool subsystem5_startup_scan_ready (void);

/* True from the moment Subsystem 5 stops the car for an obstacle until it
 * has profiled it and decided the bypass side (or the obstacle has gone). The vehicle controller then
 * pauses the line follower (VC_AVOIDING). Any task. */
bool subsystem5_is_avoiding (void);

/* Obsolete: Subsystem 5 no longer drives the wheels, so the value is
 * ignored. Kept so the vehicle controller still links. Any task. */
void subsystem5_set_drive_permission (const bool b_allowed);

#endif /* SUBSYSTEM5_H */
