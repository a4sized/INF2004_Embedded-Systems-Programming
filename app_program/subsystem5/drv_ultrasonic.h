/**
 * @file    drv_ultrasonic.h
 * @brief   HC-SR04 ultrasonic distance sensor driver.
 *
 * Only call these from one task at a time.
 */
#ifndef DRV_ULTRASONIC_H
#define DRV_ULTRASONIC_H

#include <stdint.h>
#include <tk/tkernel.h>
#include "s5_config.h"

/* Set up the trigger and echo pins. Call once before reading. */
ER       drv_ultrasonic_init(void);

/* Take one measurement and say whether the sensor itself worked.
 *   E_OK     *p_distance_mm = distance, or NO_OBSTACLE if nothing in range
 *   E_BUSY   sensor still finishing the previous ping: no reading taken,
 *            not a fault, just try again later
 *   E_IO     echo pin stuck HIGH long after the last ping (stuck / wiring)
 *   E_TMOUT  sensor never answered the trigger (unplugged / no power)
 *   E_PAR    p_distance_mm is NULL
 * On any error *p_distance_mm is set to NO_OBSTACLE. */
ER       drv_ultrasonic_measure(uint16_t * const p_distance_mm);

/* Take one measurement. Returns the distance in millimetres,
 * or NO_OBSTACLE if nothing was detected or the sensor failed. */
uint16_t drv_ultrasonic_read_distance_mm(void);

#endif /* DRV_ULTRASONIC_H */
