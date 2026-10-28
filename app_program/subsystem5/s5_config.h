/**
 * @file    s5_config.h
 * @brief   Ultrasonic sensor pins and timing.
 */
#ifndef S5_CONFIG_H
#define S5_CONFIG_H

/* GPIO numbers refer to Pico/Robo Pico GP labels; servo signal is on GP12. */
#define S5_PIN_TRIG                 (1u)
#define S5_PIN_ECHO                 (0u)
#define S5_PIN_SERVO                (12u)

#define S5_TRIG_PULSE_US            (10u)
#define S5_ECHO_START_TIMEOUT_US    (3000u)
#define S5_ECHO_TIMEOUT_US          (8750u)
#define S5_US_SOUND_NUM             (343u)
#define S5_US_SOUND_DEN             (2000u)
#define S5_DIST_NONE                (0xFFFFu)
#define S5_SAMPLE_PERIOD_MS         (500u)
#define S5_TASK_PRI                 (20)
#define S5_SCAN_ANGLE_COUNT         (5u)   /* 30, 60, 90, 120, 150 deg */

/* Servo speed = S5_SERVO_STEP_DEG per S5_SERVO_STEP_MS (1 deg / 30 ms is
 * about 33 deg/s). Raise STEP_MS to go slower, lower it to go faster. */
#define S5_SERVO_STEP_DEG           (1u)
#define S5_SERVO_STEP_MS            (30u)
#define S5_SERVO_FINAL_SETTLE_MS    (150u) /* let the horn stop before a ping */

/* Servo calibration. HOME is the angle the horn snaps to at power-up.
 * 90 is the middle of travel and points right on this mount, so the front is
 * probably near 0 or 180. Try one, reflash, and see where it points; adjust
 * until it faces straight ahead, then use that value as the scan centre.
 * HOLD=1 keeps the servo at HOME and just pings (no scan). Set 0 to scan. */
#define S5_SERVO_HOME_DEG           (90u)
#define S5_CALIBRATE_HOLD           (1)

/* Wiggle demo (takes priority over HOLD and the scan). From the centre
 * (S5_SERVO_HOME_DEG, treated as 90) it goes left, left, right, right back to
 * centre, then right, right, left, left back to centre, and repeats. Left is
 * the higher angle. Each move is S5_WIGGLE_STEP_DEG. Set 0 to disable. */
#define S5_WIGGLE_DEMO              (1)
#define S5_WIGGLE_STEP_DEG          (30u)
#define S5_WIGGLE_PAUSE_MS          (500u)

#endif /* S5_CONFIG_H */