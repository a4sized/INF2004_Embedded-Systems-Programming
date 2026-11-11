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

/* Distance drv_ultrasonic_measure() gives when nothing echoes back. */
#define NO_OBSTACLE                 (0xFFFFu)

/* ---------------- Servo movement ---------------- */
/* The servo moves SERVO_STEP_DEGREES every SERVO_STEP_DELAY_MS.
 * 2 degrees every 10 ms is about 200 degrees per second: fast enough for a
 * short start-up scan, still gentle enough not to jerk the sensor.
 * Bigger delay = slower, smaller delay = faster. */
#define SERVO_STEP_DEGREES          (2u)
#define SERVO_STEP_DELAY_MS         (10u)

/* Extra wait after the servo arrives, so it has stopped before we measure.
 * Raise it if the reading right after a move looks wrong. */
#define SERVO_SETTLE_MS             (100u)

/* Fine-tune where "straight ahead" is. If the 90 degree position does not
 * face forward, change this (e.g. 10 or -10) until it does. */
#define SERVO_TRIM_DEGREES          (0)

/* Servo angle that faces straight ahead, and the servo's largest angle. */
#define S5_FORWARD_DEG              (90u)
#define S5_SERVO_MAX_DEG            (180u)

/* ---------------- Scanning ---------------- */
/* Number of angles in the start-up sweep (g_scan_angles[] in subsystem5.c).
 * The sweep runs only at start-up, with the car held still, to check the
 * path is clear before moving off. After that the sensor faces straight
 * ahead, except during a fine scan (see below). */
#define S5_SCAN_POSITIONS           (5u)

/* Pause at each angle of the start-up sweep after taking a reading.
 * Two ping intervals: enough to retry once if the first reading fails. */
#define PAUSE_AT_EACH_ANGLE_MS      (120u)

/* After the start-up scan: print the straight-ahead distance this often.
 * Printing is only for people watching the console. The sensor is still
 * pinged every S5_PING_INTERVAL_MS in between, and every one of those
 * readings goes to the obstacle guard and telemetry. Printing all of them
 * (about 16 lines a second) would flood the console and slow the task. */
#define S5_FORWARD_PRINT_MS         (500u)

/* Priority of the scanning task (lower number = more important).
 * S5's band is 20-21 (TEAM.md). */
#define SCANNER_TASK_PRIORITY       (20)

/* ---------------- Reporting to Subsystem 1 (s5_report.c) ---------------- */
/* FAULT_US_SENS is reported after this many failed readings in a row, and
 * cleared after S5_US_RECOVER_LIMIT good ones, so a single glitch does not
 * pause the car (FAULT_US_SENS is a Tier-2 "pause and resume" fault). */
#define S5_US_FAIL_LIMIT            (3u)
#define S5_US_RECOVER_LIMIT         (3u)

/* Telemetry obstacle_cm is 9 bits (0..511 cm). "Nothing in range" is sent
 * as the top value. */
#define S5_OBSTACLE_NONE_CM         (511u)
#define S5_OBSTACLE_MAX_CM          (S5_OBSTACLE_NONE_CM - 1u)   /* 510 cm */
#define S5_MM_PER_CM                (10u)

/* ---------------- Obstacle stop / slow-down (s5_guard.c) ---------------- */
/* Ping the sensor this often while scanning (HC-SR04 needs >= 60 ms
 * between pings so old echoes die out). */
#define S5_PING_INTERVAL_MS         (60u)

/* The car should come to rest this far from the obstacle. */
#define S5_STOP_DISTANCE_CM         (5u)

/* Start stopping this much earlier, because S2 ramps down instead of
 * stopping instantly and a reading is up to S5_PING_INTERVAL_MS old.
 * Tune on the track: raise it if the car stops closer than 5 cm. */
#define S5_BRAKE_MARGIN_CM          (2u)

/* Distance from the sensor's face to the front of the car. Measure this:
 * all distances above are from the front of the car. */
#define S5_SENSOR_TO_FRONT_CM       (0u)

/* Once stopped, stay stopped until the gap is this much wider than the
 * stop point, so noisy readings do not make the car creep. */
#define S5_RESUME_HYSTERESIS_CM     (5u)

/* Slow down when an obstacle is within this distance... */
#define S5_SLOW_DISTANCE_CM         (30u)
/* ...to at most this speed (S2's 0..255 PWM scale). */
#define S5_SLOW_SPEED_CAP           (80u)
/* No obstacle near: no limit. */
#define S5_FULL_SPEED_CAP           (255u)

/* ---------------- Fine scan and obstacle profile (s5_profile.c) -------- */
/* When the car stops for an obstacle, the servo steps outward from 90
 * degrees by S5_FINE_STEP_DEG (a multiple of 5) up to these limits. */
#define S5_FINE_STEP_DEG            (5u)
#define S5_FINE_MIN_ANGLE           (30u)
#define S5_FINE_MAX_ANGLE           (150u)

/* Wait this long after the stop before scanning, so the car is at rest. */
#define S5_FINE_START_DELAY_MS      (300u)

/* Tries per fine-scan angle before giving up on that reading. */
#define S5_FINE_READ_TRIES          (3u)

/* A reading this much further than the last point on the obstacle (or no
 * echo at all) means the beam has passed the obstacle's edge. */
#define S5_EDGE_JUMP_CM             (15u)

/* Readings taken past each edge to measure the clearance (max 3). */
#define S5_CLEAR_STEPS              (3u)

/* The sensor's beam is a cone, so it still "sees" the obstacle a little
 * past its real edge and the width comes out too big. Tune on the track
 * with a box of known width: raise this (multiple of 5) until the printed
 * width matches. 0 = no correction. */
#define S5_EDGE_TRIM_DEG            (0u)

/* 1 = servo angles above 90 point to the car's left, 0 = to its right.
 * Checked on the car: 30 degrees faces left, so 150 degrees faces right. */
#define S5_HIGH_ANGLE_IS_LEFT       (0u)

/* Derived: do not edit. */
#define S5_STOP_TRIGGER_CM          (S5_STOP_DISTANCE_CM + S5_BRAKE_MARGIN_CM)
#define S5_RESUME_GAP_CM            (S5_STOP_TRIGGER_CM + \
                                     S5_RESUME_HYSTERESIS_CM)

#endif /* S5_CONFIG_H */
