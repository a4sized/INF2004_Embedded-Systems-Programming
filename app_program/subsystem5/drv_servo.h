/**
 * @file    drv_servo.h
 * @brief   Robo Pico hobby-servo driver (50 Hz PWM on GP12).
 *
 * Uses the project BSP PWM interface and the RP2040 PWM divider register.
 */
#ifndef DRV_SERVO_H
#define DRV_SERVO_H

#include <stdint.h>
#include <tk/tkernel.h>

ER       drv_servo_init(void);
ER       drv_servo_set_angle(uint16_t angle_deg);
uint16_t drv_servo_get_angle(void);

/** Time the horn needs to reach target_deg from where it is now. */
uint16_t drv_servo_settle_ms(uint16_t target_deg);

#endif /* DRV_SERVO_H */
