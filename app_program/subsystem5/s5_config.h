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
/* How far the sensor looks. Anything further away counts as "no
 * obstacle", so far-off walls and furniture are ignored. Raise it to see
 * further (the sensor itself reaches about 400 cm). */
#define S5_MAX_RANGE_CM             (50u)

/* While driving, ignore anything further than this straight ahead (or in
 * the start-up sweep): it does not slow or stop the car and is reported as
 * "no obstacle". The fine scan still looks out to S5_MAX_RANGE_CM, so it
 * can see the obstacle's edges and the gaps beside it. */
#define S5_DETECT_RANGE_CM          (15u)

/* Longest echo to wait for: sound goes there and back at 343 m/s, i.e.
 * 20000 / 343 = about 58 us per cm of range. Derived: do not edit. */
#define ECHO_TIMEOUT_US             ((S5_MAX_RANGE_CM * 20000u) / 343u)

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
#define S5_STOP_DISTANCE_CM         (10u)

/* Start stopping this much earlier, because S2 ramps down instead of
 * stopping instantly and a reading is up to S5_PING_INTERVAL_MS old.
 * Tune on the track: raise it if the car stops closer than 10 cm. */
#define S5_BRAKE_MARGIN_CM          (2u)

/* Distance from the sensor's face to the front of the car. Measure this:
 * all distances above are from the front of the car. */
#define S5_SENSOR_TO_FRONT_CM       (0u)

/* Once stopped, stay stopped until the gap is this much wider than the
 * stop point, so noisy readings do not make the car creep. */
#define S5_RESUME_HYSTERESIS_CM     (5u)

/* Slow down when an obstacle is within this distance... */
#define S5_SLOW_DISTANCE_CM         (15u)
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

/* When a ping finds nothing at all, the HC-SR04 keeps its echo pin HIGH
 * for 38 ms (cheap clones up to ~200 ms) and is busy until then. So each
 * fine-scan angle keeps trying, every S5_FINE_RETRY_MS, for up to
 * S5_FINE_READ_TIMEOUT_MS before it counts as missed. */
#define S5_FINE_RETRY_MS            (10u)
#define S5_FINE_READ_TIMEOUT_MS     (300u)

/* The profile is still used if no more than this many fine-scan angles
 * (out of 24) got no reading; missed angles are simply skipped. */
#define S5_FINE_MAX_MISSED          (4u)

/* A reading this much further than the last point on the obstacle (or no
 * echo at all) means the beam has passed the obstacle's edge. */
#define S5_EDGE_JUMP_CM             (15u)

/* The sensor's beam is a cone, so it still "sees" the obstacle a little
 * past its real edge and the width comes out too big. Tune on the track
 * with a box of known width: raise this (multiple of 5) until the printed
 * width matches. 0 = no correction. */
#define S5_EDGE_TRIM_DEG            (0u)

/* 1 = servo angles above 90 point to the car's left, 0 = to its right.
 * Checked on the car: 30 degrees faces left, so 150 degrees faces right. */
#define S5_HIGH_ANGLE_IS_LEFT       (0u)

/* ---------------- Bypass decision (s5_avoid.c) ---------------- */
/* Overall width of the car (widest point, wheels included): measured
 * 127.5 mm, rounded up to whole cm. The car only goes round an obstacle on
 * a side whose free gap is at least S5_CAR_WIDTH_CM + S5_BYPASS_MARGIN_CM.
 * The ultrasonic sensor is on the car's centre line, so sideways distances
 * need no offset. */
#define S5_CAR_WIDTH_CM             (13u)
#define S5_BYPASS_MARGIN_CM         (5u)

/* Only things up to this far beyond the obstacle's front count as being
 * "beside" it (they block the way round). Anything further ahead does not.
 * Roughly the obstacle's depth plus the car's length. */
#define S5_BYPASS_DEPTH_CM          (30u)

/* ---------------- Driving the bypass (s5_drive.c) ---------------- */
/* 1 = Subsystem 5 carries the bypass decision out itself through
 * Subsystem 2's s2_set_ir_velocity(): turn towards the side with the most
 * clearance and go round the obstacle. 0 = only decide (s5_avoid_get()). */
#define S5_DRIVE_BYPASS             (1u)

/* Wheel speeds sent to Subsystem 2 (1..255, PWM duty): for driving and
 * turning during a bypass, and a slower one for the short back-up. Too low and
 * the motors will not turn. */
#define S5_DRIVE_SPEED              (120)
#define S5_REVERSE_SPEED            (70)

/* Reversing is never boosted (see stall boost below) past this speed. */
#define S5_REVERSE_MAX_SPEED        (110u)

/* After a successful bypass, keep driving straight ahead at
 * S5_DRIVE_SPEED (1), or stop and hand the car back (0). 0: the vehicle
 * controller hands it back to the line follower (AVOIDING -> FOLLOWING). */
#define S5_RESUME_AFTER_BYPASS      (0u)

/* Wheels, from Subsystem 2's encoder.h: 2050 ticks per wheel turn, 66 mm
 * wheel, so pi x 66 = 207 mm per turn. */
#define S5_ENCODER_TICKS_PER_REV    (2050u)
#define S5_WHEEL_CIRCUM_MM          (207u)

/* TO MEASURE: distance between the middles of the two drive wheels. Used
 * to turn on the spot. If a 90 degree turn comes out short, raise it; if
 * it overshoots, lower it. */
#define S5_WHEEL_TRACK_MM           (110u)

/* Back off this far before turning, so the car's front corner clears the
 * obstacle. */
#define S5_BYPASS_BACKUP_MM         (30u)

/* After a fine scan, the obstacle counts as still in the way while
 * something is within this distance straight ahead (the sensor's whole
 * range), so the car goes round it rather than driving on. */
#define S5_STILL_BLOCKED_CM         (S5_MAX_RANGE_CM)

/* Stall boost: if the encoders move less than S5_DRIVE_STALL_TICKS in
 * S5_DRIVE_STALL_MS during a move (turning on the spot needs more power
 * than driving straight), raise that move's speed by S5_DRIVE_BOOST_STEP,
 * up to 255 (S5_REVERSE_MAX_SPEED when reversing). Not checked during the
 * first S5_DRIVE_STALL_GRACE_MS of a move: Subsystem 2 pauses 100 ms when
 * a wheel changes direction and then ramps the speed up gradually. */
#define S5_DRIVE_STALL_GRACE_MS     (600u)
#define S5_DRIVE_STALL_MS           (300u)
#define S5_DRIVE_STALL_TICKS        (20u)
#define S5_DRIVE_BOOST_STEP         (30u)

/* Subsystem 2 ramps a wheel's speed by this much every 20 ms, also when
 * stopping, so the car coasts on after the wheels are told to stop. Each
 * move stops early by that coasting distance (worked out from how fast
 * the wheels turn), so turns and distances come out right. */
#define S5_S2_RAMP_STEP             (10u)

/* After a move has stopped, if it came up more than
 * S5_DRIVE_TOLERANCE_TICKS short (motors that stop dead instead of
 * coasting), drive the rest, up to S5_DRIVE_PASSES passes in all. 20
 * ticks is about 2 mm of wheel travel. */
#define S5_DRIVE_TOLERANCE_TICKS    (20u)
#define S5_DRIVE_PASSES             (3u)

/* Give up on one move if it has not finished in this time (stuck wheel,
 * encoder not counting). */
#define S5_DRIVE_MOVE_TIMEOUT_MS    (5000u)

/* After each move, wait at least S5_DRIVE_SETTLE_MS and until the
 * encoders show the wheels at rest (at most S5_DRIVE_REST_MAX_MS):
 * Subsystem 2 ramps them down gradually, and starting the next move while
 * a wheel still turns would swing the car off course. A multiple of
 * S5_PING_INTERVAL_MS. */
#define S5_DRIVE_REST_MAX_MS        (1200u)
#define S5_DRIVE_SETTLE_MS          (240u)

/* Derived: do not edit. */
#define S5_BYPASS_NEED_CM           (S5_CAR_WIDTH_CM + S5_BYPASS_MARGIN_CM)
#define S5_STOP_TRIGGER_CM          (S5_STOP_DISTANCE_CM + S5_BRAKE_MARGIN_CM)
#define S5_RESUME_GAP_CM            (S5_STOP_TRIGGER_CM + \
                                     S5_RESUME_HYSTERESIS_CM)

#endif /* S5_CONFIG_H */
