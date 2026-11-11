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

/* While driving, ignore anything further than this straight ahead: it does
 * not slow or stop the car and is reported as "no obstacle". The fine scan
 * still looks out to S5_MAX_RANGE_CM, so it can see the obstacle's edges
 * (which read further away than its front). */
#define S5_DETECT_RANGE_CM          (35u)

/* Longest echo to wait for: sound goes there and back at 343 m/s, i.e.
 * 20000 / 343 = about 58 us per cm of range. Derived: do not edit. */
#define ECHO_TIMEOUT_US             ((S5_MAX_RANGE_CM * 20000u) / 343u)

/* Distance drv_ultrasonic_measure() gives when nothing echoes back. */
#define NO_OBSTACLE                 (0xFFFFu)

/* ---------------- Servo movement ---------------- */
/* The servo moves SERVO_STEP_DEGREES every SERVO_STEP_DELAY_MS.
 * 2 degrees every 10 ms is about 200 degrees per second: fast enough for a
 * quick fine scan, still gentle enough not to jerk the sensor.
 * Bigger delay = slower, smaller delay = faster. */
#define SERVO_STEP_DEGREES          (2u)
#define SERVO_STEP_DELAY_MS         (10u)

/* Extra wait after the servo arrives, so it has stopped before we measure.
 * Raise it if the reading right after a move looks wrong. */
#define SERVO_SETTLE_MS             (100u)

/* Fine-tune where "straight ahead" is. If the 90 degree position does not
 * face forward, the fine scan sees one side of a centred obstacle sooner
 * than the other, so that side always looks narrower and always wins the
 * bypass decision.  To set it: with S5_VERBOSE = 1, put a box about 15 cm
 * wide centred on the line in front of the car, let it stop and scan, and
 * use the value the "S5 centring check" line prints (negative = turn 90
 * toward the low angles). */
#define SERVO_TRIM_DEGREES          (0)

/* Servo angle that faces straight ahead, and the servo's largest angle. */
#define S5_FORWARD_DEG              (90u)
#define S5_SERVO_MAX_DEG            (180u)

/* Pause before retrying a servo move that failed (still pinging, so the
 * readings carry on).  Two ping intervals. */
#define SERVO_RETRY_PAUSE_MS        (120u)

/* ---------------- Scanning ---------------- */
/* Print the straight-ahead distance this often.
 * Printing is only for people watching the console. The sensor is still
 * pinged every S5_PING_INTERVAL_MS in between, and every one of those
 * readings goes to the obstacle guard and telemetry. Printing all of them
 * (about 16 lines a second) would flood the console and slow the task. */
#define S5_FORWARD_PRINT_MS         (500u)

/* Priority of the scanning task (lower number = more important).
 * S5's band is 20-21 (TEAM.md). */
#define SCANNER_TASK_PRIORITY       (20)

/* ---------------- Clearance test (subsystem5.c, s5_log.h) ---------------- */
/* 1 = clearance test: the car stops S5_STOP_DISTANCE_CM in front of the
 * first obstacle, fine scans it, prints ONLY the fine scan (each angle's
 * distance, the obstacle's width and its left and right edges from the
 * car's centre line) and the bypass decision,
 * then stays stopped (speed cap held at 0; the bypass is not driven and no
 * further obstacle is handled until reset).  Every other Subsystem 5
 * console line and CSV event is switched off.
 * 0 = normal running, bypass driven. */
#define S5_CLEARANCE_TEST           (0u)

/* Console output (s5_log.h).  0 = Subsystem 5 prints only the distance
 * straight ahead (every S5_FORWARD_PRINT_MS), the fine scan (each angle's
 * distance and side, marked where the edge is, then the obstacle's width
 * and its left and right edges from the car's centre line) and each bypass
 * decision.  1 = also every diagnostic line: room to go round, what each
 * side's scan covered, the centring check (for SERVO_TRIM_DEGREES), the
 * width check (for S5_EDGE_TRIM_DEG), stops, holds, each move of a bypass
 * and why it ended.  Switch it on to tune. */
#define S5_VERBOSE                  (0u)

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

/* The car should come to rest this far (cm) from the obstacle (from the
 * front of the car), and fine scans it from there: it does not back off
 * first.  5 (15 and 10 stopped too far away to scan accurately on the
 * track): the sensor is then about 10-14 cm from the obstacle.  From there
 * it can only swing out round a planned edge of about 6-10 cm, so for a
 * wider one the bypass first reverses just as far as it needs to
 * (s5_avoid_plan_backup_mm, at most S5_BYPASS_MAX_BACKUP_MM), without
 * scanning again. */
#define S5_STOP_DISTANCE_CM         (5u)

/* Start stopping this much earlier, because S2 ramps down instead of
 * stopping instantly and a reading is up to S5_PING_INTERVAL_MS old.
 * Tune on the track: raise it if the car stops closer than
 * S5_STOP_DISTANCE_CM.  5 (was 2): with 2 the car braked at 7 cm and rolled
 * on until it was too close for the HC-SR04 to get an echo at all. */
#define S5_BRAKE_MARGIN_CM          (5u)

/* Distance from the sensor's face to the front of the car: all distances
 * above are from the front of the car.  Measured: the IR line sensor board
 * sticks out about 5 cm in front of the ultrasonic sensor. */
#define S5_SENSOR_TO_FRONT_CM       (5u)

/* Once stopped, stay stopped until the gap is this much wider than the
 * stop point, so noisy readings do not make the car creep. */
#define S5_RESUME_HYSTERESIS_CM     (5u)

/* Slow down when an obstacle is within this distance... */
#define S5_SLOW_DISTANCE_CM         (30u)
/* ...to at most this speed (S2's 0..255 PWM scale). */
#define S5_SLOW_SPEED_CAP           (80u)
/* No obstacle near: no limit. */
#define S5_FULL_SPEED_CAP           (255u)

/* ---------------- Fine scan and profile (s5_scan.c, s5_profile.c) ------ */
/* When the car stops for an obstacle, each side (low angles first, then
 * high) is scanned in two passes (s5_scan.c).  These limits are also the
 * furthest the servo ever turns, while bypassing too (s5_sense_servo_to):
 *   coarse  the servo steps outward from 90 degrees S5_FINE_STEP_DEG at a
 *           time all the way to these limits (45..135, i.e. 45 degrees off
 *           straight ahead), also past the edge, so both sides always
 *           cover the same angles (9 each);
 *   refine  it then steps back through that last coarse gap
 *           S5_FINE_REFINE_DEG at a time, so the edge is found to within
 *           S5_FINE_REFINE_DEG (about 0.4 cm sideways per degree from
 *           25 cm) instead of S5_FINE_STEP_DEG.
 * Scanning everything at 1 degree would be no more accurate but about four
 * times slower (each angle takes about 0.3 s, so the full coarse pass is
 * about 2.7 s per side, plus up to 4 refine angles).  If the edge is not
 * found on a side by the limit, see S5_BYPASS_MAX_REVERSES. */
#define S5_FINE_STEP_DEG            (5u)
#define S5_FINE_REFINE_DEG          (1u)
#define S5_FINE_MIN_ANGLE           (45u)
#define S5_FINE_MAX_ANGLE           (135u)

/* A FLAT face reflects the sound away once the beam meets it more than about
 * this many degrees off square-on (seen in the fine scans: 20-30), so from
 * that angle on it gives no echo even though it carries on.  An edge found
 * up to this far off straight ahead (plus S5_EDGE_TRIM_DEG, for the beam's
 * spread) is MEASURED; one found further out, or one the readings came back
 * past after a gap too long to bridge (S5_EDGE_BRIDGE_MAX), is only a LOWER
 * BOUND (the obstacle is at least that wide).  The car still goes round a
 * lower-bound side if it fits, and checks the real width while driving
 * out, square-on to the obstacle's front (s5_bypass.c). */
#define S5_FLAT_ECHO_MAX_DEG        (25u)

/* Distance (cm) from the servo's turning axis forward to the sensor's face:
 * the face swings round on this radius, so every sideways distance is
 * (reading + this) x sin(angle).  MEASURE it on the car. */
#define S5_SENSOR_PIVOT_CM          (0u)

/* Wait this long after the stop before scanning, so the car is at rest. */
#define S5_FINE_START_DELAY_MS      (300u)

/* Extra ping attempts allowed at each fine-scan angle, on top of the
 * S5_FINE_SAMPLES wanted, for pings that fail (see s5_sense_fine_reading()).
 * When a ping finds nothing at all, the HC-SR04 keeps its echo pin HIGH for
 * 38 ms (cheap clones up to ~200 ms) and is busy until then; the attempts
 * are S5_PING_INTERVAL_MS apart, so a busy sensor gets time to recover. */
#define S5_FINE_READ_TRIES          (3u)

/* The profile is still used if no more than this many fine-scan angles
 * got no reading at all; missed angles are simply skipped.  The
 * straight-ahead reading must always succeed. */
#define S5_FINE_MAX_MISSED          (4u)

/* Pings taken at each fine-scan angle (1..S5_FILTER_MAX); the MEDIAN is used,
 * so one spike or one missing echo does not change the profile.  Each extra
 * ping adds about 60 ms per angle.  Simulated (with the earlier 5-degree
 * scan), with 13% bad pings, the shorter side was chosen about 36-47% of the
 * time from 1 ping per angle and 54-81% from 3 (5 pings: 55-86%, but much
 * slower).  Stepping EVERY angle finely gave bad pings more chances to fake
 * an edge; the refine pass only steps finely next to the edge already
 * found, so it does not. */
#define S5_FINE_SAMPLES             (3u)

/* A reading this much further than the last point on the obstacle (or no
 * echo at all) means the beam has passed the obstacle's edge.  6 (was 15):
 * from the scan distance (10-30 cm) the obstacle's own corner reads at
 * most a few cm further than its face, so a bigger step is something behind
 * it (e.g. the floor, which echoed at 35-36 cm) and must not count as part
 * of the obstacle. */
#define S5_EDGE_JUMP_CM             (6u)

/* MISSED ECHOES.  If a reading jumps off the obstacle (far, or no echo) but
 * the next reading(s) come back within S5_EDGE_JUMP_CM of the readings
 * before the jump, the jump was a missed echo: the edge has not been
 * reached, and the scan carries on looking for it.  E.g. 55 deg: 27 cm,
 * 50 deg: no echo, 45 deg: 28 cm -> the obstacle is still there at 45.
 * This is how many readings in a row off the obstacle can be bridged like
 * that; a longer gap is the real edge, and anything that comes back after
 * it is only warned about (the edge then counts as a lower bound), because
 * past a real edge the floor can echo at about the same distance.
 * 0 = never bridge (the first jump is always the edge).  Max 6. */
#define S5_EDGE_BRIDGE_MAX          (1u)

/* The sensor's beam is a cone, so it still "sees" the obstacle a little
 * past its real edge and the width comes out too big.  Tune on the track
 * with a box of known width (narrow enough that both edges are MEASURED):
 * raise this, in whole degrees, until the printed "S5 obstacle width"
 * matches.  With S5_VERBOSE = 1 the "S5 width check" line printed while
 * driving out (square-on to the front) is a second measurement to compare
 * with.  0 = no correction. */
#define S5_EDGE_TRIM_DEG            (0u)

/* 1 = servo angles above 90 point to the car's left, 0 = to its right.
 * Checked on the car: with 1 the car goes round on the side where the
 * obstacle sticks out least; with 0 it chose the other.  This one setting
 * decides both which side the fine scan calls left / right (so the bypass
 * side) and which side the servo looks at while driving past the obstacle,
 * so those two are always right together or wrong together. */
#define S5_HIGH_ANGLE_IS_LEFT       (1u)

/* ---------------- Bypass decision (s5_avoid.c) ---------------- */
/* Every obstacle is between these widths (cm).  The fine scan cannot
 * always measure both edges (beyond S5_FLAT_ECHO_MAX_DEG a flat face stops
 * echoing), so the bypass is planned with these: an edge that is only a
 * lower bound (or was not found) is taken as far out as the width allows,
 * S5_OBSTACLE_MAX_WIDTH_CM minus the other side; a measured edge is never
 * taken closer than S5_OBSTACLE_MIN_WIDTH_CM minus the other side
 * (s5_avoid_plan_extent). */
#define S5_OBSTACLE_MIN_WIDTH_CM    (10u)
#define S5_OBSTACLE_MAX_WIDTH_CM    (20u)

/* Overall width of the car (widest point, wheels included): measured
 * 127.5 mm, rounded up to whole cm. Used to drive far enough out to clear
 * the obstacle (half the car's width). There is never anything else beside
 * an obstacle, so the bypass side is decided from the obstacle's edges
 * alone. The servo is on the car's centre line. */
#define S5_CAR_WIDTH_CM             (13u)

/* PASSING IT BY FEEL.  The fine scan sees only the obstacle's front, not how
 * long it is, so after driving out (planned from the width) the car drives
 * ALONG the obstacle with the sensor turned to look straight at its side,
 * where the echo is reliable, until it ends, and then a little further. */
/* Where the sensor looks while passing the obstacle: this many degrees off
 * straight ahead, toward the obstacle (the servo turns to 90 +/- this; which
 * side follows S5_HIGH_ANGLE_IS_LEFT).  The servo never turns further than
 * the fine scan (S5_FINE_MIN_ANGLE..S5_FINE_MAX_ANGLE, i.e. 45 degrees off
 * ahead), so this is at most 45: the sensor looks 45 degrees forward at the
 * side.  A flat side only echoes up to about S5_FLAT_ECHO_MAX_DEG off
 * square-on, so at 45 it may not be seen at all: it is then taken as a
 * short obstacle, and the car drives just S5_PASS_CLEAR_MM before it turns
 * in.  When it is seen and then ends, the car drives S5_PASS_CLEAR_MM more
 * from there.
 * Square-on (90) would see the side best, but needs the servo to turn to
 * 0 / 180 degrees. */
#define S5_PASS_LOOK_OFF_DEG        (45u)
/* Still "the obstacle" up to this much (cm) farther than the first
 * reading. */
#define S5_EDGE_TOL_CM              (8u)

/* Readings in a row without the obstacle = it has ended. */
#define S5_EDGE_CONFIRM             (3u)

/* Passing it: if the obstacle has not ended after this far (mm) it is too
 * long to get round, and the car goes back to the line (s5_bypass.c). */
#define S5_EDGE_MAX_MM              (500u)

/* Sensor back to the middle of the two drive wheels (the point the car
 * turns about; the drive wheels are at the rear).  Driving sideways, the
 * sensor is this far ahead of that point, so it loses the obstacle this
 * much before the car's middle is clear. */
#define S5_SENSOR_TO_AXLE_CM        (9u)    /* measured 8.5 cm (sensor to the
                                             * middle of the REAR drive
                                             * wheels), rounded up            */

/* Passing it: once the sensor stops seeing the obstacle's side, the car
 * drives this much further (mm) before it turns back in toward the line:
 * about one car's length (22 cm), since the sensor is at the front of the
 * car and the rest of it is still beside the obstacle.  Raise it if the
 * back of the car clips the corner when it turns in. */
#define S5_PASS_CLEAR_MM            (220u)

/* Extra room left between the car and the obstacle when driving round it,
 * on top of the car's own size.  Driving out, on the S5_BYPASS_TURN_DEG
 * slant only part of each cm driven is sideways, so the car drives
 * (planned edge + half its width + this margin) / sin(turn): then, once it
 * turns back, its side is this far clear of the edge (passing it, see
 * S5_PASS_CLEAR_MM).  Raise it (in cm) if the car still clips the obstacle
 * on the way out; lower it if the detour is far wider than needed. */
#define S5_BYPASS_CLEAR_MARGIN_CM   (3u)    /* 3 (was 10): a smaller detour;
                                             * raise it if the car clips the
                                             * obstacle                         */

/* While driving along the obstacle, an echo this close (cm) after the
 * obstacle seemed to end, or when it was not seen at first, counts as the
 * obstacle still being beside the car, so the car keeps going instead of
 * turning in. */
#define S5_EDGE_NEAR_CM             (35u)

/* Driving OUT on the slant, the sensor looks along the original heading at
 * the obstacle's front, so its reading is how far the car still is from that
 * face.  The car's front corner nearest the obstacle is about 8 cm further
 * forward than the sensor (front overhang + half the width, turned by the
 * slant).  If the face reads this close (cm) or closer the obstacle is
 * wider than planned: the move is stopped before that corner hits it, the
 * car retraces its moves back to the line and waits there for the obstacle
 * to be removed.  The bypass decision uses the same limit to work out how
 * far to reverse first (s5_avoid_plan_backup_mm). */
#define S5_OUT_LEG_MIN_CM           (12u)

/* When the obstacle is too big to get round (or the scan failed), the car
 * waits on the line, held still (Subsystem 5 keeps "avoiding", speed cap
 * 0), instead of being handed back to the line follower.  It goes on once
 * this many good straight-ahead readings in a row show nothing within
 * S5_STILL_BLOCKED_CM (the obstacle was removed), or when the vehicle
 * controller takes the drive permission away (operator STOP / a fault).
 * A bypass stopped part-way (off the line) only ends on the latter.
 * 10 readings = 0.6 s. */
#define S5_HOLD_CLEAR_READINGS      (10u)

/* Going round, the car first reverses (in one straight move, without
 * scanning again: that would not change the obstacle's width) just as far
 * as it needs to for its out leg to clear the planned edge while the
 * obstacle's front is still more than S5_OUT_LEG_MIN_CM away
 * (s5_avoid_plan_backup_mm); 0 if it can go round from where it stopped.
 * No further than this (mm): a side that needs more is too wide to get
 * round (the car waits).  150 is enough for a 20 cm edge from the 5 cm stop
 * (about 100 mm). */
#define S5_BYPASS_MAX_BACKUP_MM     (150u)

/* If no edge was found at all (the width is not known), reverse
 * S5_REVERSE_MM and scan again, up to this many times in a row: further
 * back the edges come within the scan limits.  After that the car stays
 * stopped and waits.  Limited by S5_STILL_BLOCKED_CM: after all the
 * reverses the obstacle must still count as in the way (checked when
 * building, s5_scan.c). */
#define S5_BYPASS_MAX_REVERSES      (3u)

/* ---------------- Driving the bypass (s5_bypass.c, s5_drive.c) -------- */
/* 1 = Subsystem 5 carries the bypass decision out itself through
 * Subsystem 2's s2_set_ir_velocity(): go round the obstacle, or reverse
 * and scan again. 0 = only decide (printed and logged). */
#define S5_DRIVE_BYPASS             (1u)

/* Wheel speeds sent to Subsystem 2 (1..255, PWM duty): for driving during
 * a bypass, and a slower one for reversing.  Halved (were 120 and 90) for
 * a slower, more careful bypass; the line follower drives at 50-55, so
 * these still move the car.  Too low and the motors will not turn. */
#define S5_DRIVE_SPEED              (60)
#define S5_REVERSE_SPEED            (45)

/* The loop that watches the encoders runs this often. */
#define S5_DRIVE_POLL_MS            (20u)

/* TURNING ON THE SPOT is done slower than the other moves.  At 120 the car
 * spun at roughly 360 degrees a second, so a 90 degree turn was over in a
 * quarter of a second, and Subsystem 2 then needs time to ramp the wheels
 * down while the car is still turning: a fast spin cannot be stopped on the
 * mark.  Halved (was 70) with the other bypass speeds.  A turn that does
 * not get going is raised (S5_TURN_SPEED_STEP), up to S5_TURN_SPEED_MAX. */
#ifndef S5_TURN_SPEED
#define S5_TURN_SPEED               (35)
#endif

/* The most a turn is raised to if it does not get going.  Turning on the
 * spot drags the front of the car sideways, so it needs more than driving
 * straight: 70 (the turning speed before the bypass was slowed down) is
 * known to turn the car. */
#define S5_TURN_SPEED_MAX           (70)

/* Stop the turn EARLY by the angle the car will still turn after the stop
 * command: this many milliseconds of the car's CURRENT turning speed is
 * subtracted from the turn.  Subsystem 2 ramps the wheels down in fixed
 * steps, so from half the speed the ramp takes half as long: halved with
 * the speeds (measured live as 140 at the old speed; recheck).  Raise it if
 * the turn still overshoots, lower it if it falls short.  0 = no early
 * stop. */
#ifndef S5_TURN_COAST_MS
#define S5_TURN_COAST_MS            (70u)
#endif

/* The same early stop for the STRAIGHT moves.  After the stop command the car
 * keeps rolling while Subsystem 2 ramps the wheels down (about 5 cm going
 * forward at the old speed of 120, about a quarter of that at 60).  The
 * move is stopped early by this many milliseconds of the car's current
 * speed: halved with the speeds (measured as 180 / 140 at the old speeds;
 * recheck).  Raise it if a leg still runs past its distance, lower it if
 * it stops short. */
#ifndef S5_FORWARD_COAST_MS
#define S5_FORWARD_COAST_MS         (90u)
#endif
#ifndef S5_BACK_COAST_MS
#define S5_BACK_COAST_MS            (70u)
#endif

/* A turn should be finished within S5_TURN_MAX_MS.  Every S5_TURN_START_MS
 * the progress is checked against a straight line to "finished in that
 * time"; a turn that is behind it (the motors need more than S5_TURN_SPEED
 * to start or to turn at a useful rate) is raised by S5_TURN_SPEED_STEP,
 * never above S5_TURN_SPEED_MAX.  3000 (was 1500 at twice the speed), so a
 * turn at half speed is not pushed straight back up. */
#define S5_TURN_START_MS            (300u)
#define S5_TURN_SPEED_STEP          (10)
#ifndef S5_TURN_MAX_MS
#define S5_TURN_MAX_MS              (3000u)
#endif

/* After a successful bypass, keep driving straight ahead at
 * S5_DRIVE_SPEED (1), or stop and hand the car back (0). 0: the vehicle
 * controller hands it back to the line follower (AVOIDING -> FOLLOWING). */
#define S5_RESUME_AFTER_BYPASS      (0u)

/* Wheels, from Subsystem 2's encoder.h: 2050 ticks per wheel turn, 66 mm
 * wheel, so pi x 66 = 207 mm per turn. */
#define S5_ENCODER_TICKS_PER_REV    (2050u)
#define S5_WHEEL_CIRCUM_MM          (207u)

/* Distance between the middles of the two drive wheels, as the turn on the
 * spot sees it. Used to turn on the spot. If a turn comes out short, raise
 * it; if it overshoots, lower it.  220 (the real track is 110): the drive
 * wheels are at the back, so turning on the spot drags the front of the car
 * sideways and the wheels slip; measured, a turn asked for at 110 came out
 * at about half the angle, so the wheels must roll about twice as far. */
#define S5_WHEEL_TRACK_MM           (220u)

/* The turns of the bypass going out (out, and back to the heading), in
 * degrees.  45: quicker than 90 and enough to get round. */
#define S5_BYPASS_TURN_DEG          (45u)

/* Coming back to the line after passing the obstacle, the car turns in
 * (and at the line back to the heading) by this percentage of
 * S5_BYPASS_TURN_DEG: 70, i.e. 30% less (45 -> 32 degrees), a gentler
 * slant back onto the line.  The return leg is then longer, to cover the
 * same sideways distance as the out leg (s5_bypass_go_round). */
#define S5_RETURN_TURN_PERCENT      (70u)

/* Derived: do not edit.  Rounded to the nearest degree. */
#define S5_RETURN_TURN_DEG          (((S5_BYPASS_TURN_DEG * \
                                       S5_RETURN_TURN_PERCENT) + 50u) / 100u)

/* ---------------- Line recovery (Stage 5, s5_bypass.c) ---------------- */
/* Driving back to the line after a bypass, the car watches Subsystem 3's IR
 * line sensors (ir_line_snapshot_shared_read()).  It expects the line about
 * the distance it drove away from it, and looks for it from
 * S5_LINE_SEARCH_MM before that to S5_LINE_SEARCH_MM past it.  When either
 * sensor sees the line, the car drives S5_IR_TO_AXLE_MM more, so the middle
 * of the drive wheels (the point it turns about) is on the line, then turns
 * back to the original heading: the IR sensors are then over the line.  If
 * the line is not seen on the way, the car goes back to the distance
 * measured by the encoders (within S5_RETURN_TOL_MM), turns back to the
 * heading and hands the car back: the line follower then starts in its
 * search state, turning toward the side the line is on. */
#define S5_LINE_SEARCH_MM           (100u)
#define S5_RETURN_TOL_MM            (10u)

/* Distance (mm) from the middle of the two (rear) drive wheels forward to
 * the IR line sensors: measured 125.  Too small and the car turns before its
 * wheels reach the line (it ends up short of the line); too big and it
 * overshoots. */
#define S5_IR_TO_AXLE_MM            (125u)

/* An IR reading older than this (ms) is not trusted. */
#define S5_LINE_MAX_AGE_MS          (50u)

/* REVERSE decision (no edge found at all): back off this far, then scan
 * again. */
#define S5_REVERSE_MM               (50u)

/* After a fine scan, the obstacle counts as still in the way while
 * something is within this distance straight ahead (also after reversing
 * S5_REVERSE_MM).  45 (was 35): the car stops with the sensor up to about
 * 15 cm away, plus up to 3 reverses of S5_REVERSE_MM is up to about 30 cm
 * (checked when building, s5_scan.c). */
#define S5_STILL_BLOCKED_CM         (45u)

/* Give up on one move if it has not finished in this time (stuck wheel,
 * encoder not counting).  10000 (was 5000 at twice the speed): the longest
 * move, passing an obstacle, can be most of a metre. */
#define S5_DRIVE_MOVE_TIMEOUT_MS    (10000u)

/* Pause after each move while Subsystem 2 ramps the wheels down. A
 * multiple of S5_PING_INTERVAL_MS. */
#define S5_DRIVE_SETTLE_MS          (240u)

/* Derived: do not edit. */
#define S5_STOP_TRIGGER_CM          (S5_STOP_DISTANCE_CM + S5_BRAKE_MARGIN_CM)
#define S5_RESUME_GAP_CM            (S5_STOP_TRIGGER_CM + \
                                     S5_RESUME_HYSTERESIS_CM)

#endif /* S5_CONFIG_H */
