/**
 * @file    s5_config.h
 * @brief   Settings for subsystem 5: servo + ultrasonic obstacle scanner.
 *
 * Change the numbers here to tune the scanner; the .c files read them.
 */
#ifndef S5_CONFIG_H
#define S5_CONFIG_H

/* ---------------- Pins (Pico / Robo Pico "GP" numbers) ---------------- */
#define S5_PIN_SERVO                (12u)   /* servo signal wire          */
#define S5_PIN_TRIG                 (1u)    /* HC-SR04 "Trig"             */
#define S5_PIN_ECHO                 (0u)    /* HC-SR04 "Echo" (3.3 V safe) */

/* ---------------- Ultrasonic sensor ---------------- */
/* Longest echo we wait for. 8750 us is about 1.5 m of range.
 * Raise to 23500 for the sensor's full ~4 m range. */
#define ECHO_TIMEOUT_US             (8750u)

/* Value returned by drv_ultrasonic_read_distance_mm() when nothing echoes back. */
#define NO_OBSTACLE                 (0xFFFFu)

/* ---------------- Servo movement ---------------- */
/* The servo moves SERVO_STEP_DEGREES every SERVO_STEP_DELAY_MS.
 * 1 degree every 30 ms is about 33 degrees per second.
 * Bigger delay = slower, smaller delay = faster. */
#define SERVO_STEP_DEGREES          (1u)
#define SERVO_STEP_DELAY_MS         (30u)

/* Extra wait after the servo arrives, so it has stopped before we measure. */
#define SERVO_SETTLE_MS             (150u)

/* Fine-tune where "straight ahead" is. If the 90 degree position does not
 * face forward, change this (e.g. 10 or -10) until it does. */
#define SERVO_TRIM_DEGREES          (0)

/* ---------------- Scanning ---------------- */
/* Pause at each angle after taking a reading. */
#define PAUSE_AT_EACH_ANGLE_MS      (500u)

/* Priority of the scanning task (lower number = more important). */
#define SCANNER_TASK_PRIORITY       (20)

#endif /* S5_CONFIG_H */
