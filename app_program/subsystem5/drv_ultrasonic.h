/**
 * @file    drv_ultrasonic.h
 * @brief   HC-SR04 style ultrasonic ranger (trigger + echo pulse width).
 *
 * Call from a single task at a time. The echo pin must be 3.3 V safe
 * (Robo Pico header or a divider on a bare HC-SR04).
 */
#ifndef DRV_ULTRASONIC_H
#define DRV_ULTRASONIC_H

#include <stdint.h>
#include <tk/tkernel.h>
#include "s5_config.h"

ER       drv_ultrasonic_init(void);

/** One measurement in mm, or S5_DIST_NONE if no echo within range. */
uint16_t drv_ultrasonic_ping_mm(void);

#endif /* DRV_ULTRASONIC_H */
