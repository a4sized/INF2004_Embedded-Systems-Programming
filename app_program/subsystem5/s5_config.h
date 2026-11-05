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

/* Pings taken at each fine-scan angle (1..S5_FILTER_MAX); the MEDIAN is used,
 * so one spike or one missing echo does not change the profile.  Each extra
 * ping adds about 60 ms per angle (about 1.5 s over the whole scan).
 * Simulated, with 13% bad pings, the shorter side was chosen about 36-47% of
 * the time from 1 ping per angle and 54-81% from 3 (5 pings: 55-86%, but
 * about 5 s longer).  A finer angle step did NOT help (it gives bad pings
 * more chances to fake an edge), so the step stays at S5_FINE_STEP_DEG. */
#define S5_FINE_SAMPLES             (3u)

/* A reading this much further than the last point on the obstacle (or no
 * echo at all) means the beam has passed the obstacle's edge. */
#define S5_EDGE_JUMP_CM             (15u)

/* The sensor's beam is a cone, so it still "sees" the obstacle a little
 * past its real edge and the width comes out too big. Tune on the track
 * with a box of known width: raise this (multiple of 5) until the printed
 * width matches. 0 = no correction. */
#define S5_EDGE_TRIM_DEG            (0u)

/* 1 = servo angles above 90 point to the car's left, 0 = to its right.
 * Checked on the car (from its own scan log): the scan that starts at 85 and
 * goes down to 30 degrees runs over the car's RIGHT side, so angles above 90
 * are the LEFT.  If the "left" and "right" in the log ever look swapped
 * against the car, this is the setting to flip. */
#define S5_HIGH_ANGLE_IS_LEFT       (1u)

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

/* GOING ROUND BY FEEL.  From the front, the sensor cannot tell how wide a flat
 * obstacle is: a flat face stops echoing once the beam hits it about 20
 * degrees off straight-on, so a 40 cm box looks as narrow as a 10 cm one and
 * the sideways distance comes out far too small (the car scrapes the box).
 * Instead the car drives ALONG the obstacle with the sensor turned to look
 * straight at it, where the echo is reliable, until the obstacle ends, and
 * then drives a little further. */
#define S5_EDGE_LOOK_HIGH_DEG       (170u)  /* servo angle: look at the car's right */
#define S5_EDGE_LOOK_LOW_DEG        (10u)   /* servo angle: look at the car's left  */
#define S5_EDGE_TOL_CM              (8u)    /* still "the obstacle" up to this much
                                             * farther than the first reading     */
#define S5_EDGE_CONFIRM             (3u)    /* readings in a row without it = end */
#define S5_EDGE_MAX_MM              (500u)  /* give up if it has not ended by now */
#define S5_EDGE_WAIT_MM             (300u)  /* when the obstacle is still AHEAD of
                                             * the car (passing it), how far the
                                             * car may drive for it to come into
                                             * view beside it                     */
#define S5_CAR_LENGTH_CM            (20u)   /* sensor to the REAR of the car.
                                             * MEASURE this.                      */
#define S5_SENSOR_TO_AXLE_CM        (10u)   /* sensor back to the middle of the
                                             * two drive wheels (the point the
                                             * car turns about).  MEASURE this.
                                             * Driving sideways, the sensor is
                                             * this far ahead of that point, so
                                             * it loses the obstacle this much
                                             * before the car's middle is clear.  */

/* If neither side has room, reverse S5_REVERSE_MM and scan again, up to
 * this many times in a row (max 3), before giving up and staying stopped.
 * Each step back widens the sensor's view of the obstacle's edges. */
#define S5_BYPASS_MAX_REVERSES      (3u)

/* ---------------- Driving the bypass (s5_drive.c) ---------------- */
/* 1 = Subsystem 5 carries the bypass decision out itself through
 * Subsystem 2's s2_set_ir_velocity(): go round the obstacle, or reverse
 * and scan again. 0 = only decide (s5_avoid_get()). */
#define S5_DRIVE_BYPASS             (1u)

/* Wheel speeds sent to Subsystem 2 (1..255, PWM duty): for driving and
 * turning during a bypass, and a slower one for reversing. Too low and
 * the motors will not turn. */
#define S5_DRIVE_SPEED              (120)
#define S5_REVERSE_SPEED            (90)

/* The loop that watches the encoders runs this often. */
#define S5_DRIVE_POLL_MS            (20u)

/* TURNING ON THE SPOT is done slower than the other moves.  At S5_DRIVE_SPEED
 * the car spins at roughly 360 degrees a second, so a 90 degree turn is over
 * in a quarter of a second, and Subsystem 2 then needs another quarter of a
 * second to ramp the wheels down while the car is still turning.  A fast spin
 * cannot be stopped on the mark. */
#ifndef S5_TURN_SPEED
#define S5_TURN_SPEED               (70)
#endif

/* Stop the turn EARLY by the angle the car will still turn after the stop
 * command.  Measured live: this many milliseconds of the car's CURRENT turning
 * speed is subtracted from the turn.  Raise it if the turn still overshoots,
 * lower it if it falls short.  0 = no early stop. */
#ifndef S5_TURN_COAST_MS
#define S5_TURN_COAST_MS            (140u)
#endif

/* The same early stop for the STRAIGHT moves.  After the stop command the car
 * keeps rolling while Subsystem 2 ramps the wheels down (about 5 cm going
 * forward at S5_DRIVE_SPEED).  The move is stopped early by this many
 * milliseconds of the car's current speed.  Raise it if a leg still runs past
 * its distance, lower it if it stops short. */
#ifndef S5_FORWARD_COAST_MS
#define S5_FORWARD_COAST_MS         (180u)
#endif
#ifndef S5_BACK_COAST_MS
#define S5_BACK_COAST_MS            (140u)
#endif

/* A turn should be finished within S5_TURN_MAX_MS.  Every S5_TURN_START_MS
 * the progress is checked against a straight line to "finished in that
 * time"; a turn that is behind it (the motors need more than S5_TURN_SPEED
 * to start or to turn at a useful rate) is raised by S5_TURN_SPEED_STEP,
 * never above S5_DRIVE_SPEED. */
#define S5_TURN_START_MS            (300u)
#define S5_TURN_SPEED_STEP          (15)
#ifndef S5_TURN_MAX_MS
#define S5_TURN_MAX_MS              (1500u)
#endif

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

/* REVERSE decision: back off this far, then scan again. */
#define S5_REVERSE_MM               (50u)

/* After a fine scan, the obstacle counts as still in the way while
 * something is within this distance straight ahead (also after reversing
 * S5_REVERSE_MM). */
#define S5_STILL_BLOCKED_CM         (30u)

/* Give up on one move if it has not finished in this time (stuck wheel,
 * encoder not counting). */
#define S5_DRIVE_MOVE_TIMEOUT_MS    (5000u)

/* Pause after each move while Subsystem 2 ramps the wheels down. A
 * multiple of S5_PING_INTERVAL_MS. */
#define S5_DRIVE_SETTLE_MS          (240u)

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

/* After a fine scan, the obstacle counts as still in the way while
 * something is within this distance straight ahead (the sensor's whole
 * range), so a bypass side is chosen rather than continuing. */
#define S5_STILL_BLOCKED_CM         (S5_MAX_RANGE_CM)

/* Derived: do not edit. */
#define S5_BYPASS_NEED_CM           (S5_CAR_WIDTH_CM + S5_BYPASS_MARGIN_CM)
#define S5_STOP_TRIGGER_CM          (S5_STOP_DISTANCE_CM + S5_BRAKE_MARGIN_CM)
#define S5_RESUME_GAP_CM            (S5_STOP_TRIGGER_CM + \
                                     S5_RESUME_HYSTERESIS_CM)

#endif /* S5_CONFIG_H */
