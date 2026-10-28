/**
 * @file    drv_servo.h
 * @brief   Hobby servo driver (50 Hz PWM, 0-180 degrees).
 */
#ifndef DRV_SERVO_H
#define DRV_SERVO_H

#include <stdint.h>
#include <tk/tkernel.h>

/* Set up PWM on the servo pin and move the servo to 90 degrees (centre). */
ER       drv_servo_init(void);

/* Move the servo straight to angle_degrees (0-180). Returns E_OK on success. */
ER       drv_servo_set_angle(uint16_t angle_degrees);

/* The angle the servo was last told to go to. */
uint16_t drv_servo_get_angle(void);

#endif /* DRV_SERVO_H */
