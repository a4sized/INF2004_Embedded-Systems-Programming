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

/* Take one measurement. Returns the distance in millimetres,
 * or NO_OBSTACLE if nothing was detected within range. */
uint16_t drv_ultrasonic_read_distance_mm(void);

#endif /* DRV_ULTRASONIC_H */
