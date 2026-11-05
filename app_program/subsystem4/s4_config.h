/*
 *  s4_config.h  --  every number you may want to tune, in ONE place.
 *
 *  The values marked (tune) are starting points: change them using real
 *  data from the car, not by guessing.
 */

#ifndef S4_CONFIG_H
#define S4_CONFIG_H

/* ------------------------------------------------------------------ *
 *  Sensor wiring
 * ------------------------------------------------------------------ */
/* The IMU is on I2C unit 1 (pins GP26 = SDA, GP27 = SCL).  The kernel names
 * unit 0 "iica" and unit 1 "iicb". */
#define S4_I2C_UNIT         1
#define S4_I2C_NAME         "iicb"

/* ------------------------------------------------------------------ *
 *  Axis mapping.  Car axis = sign * sensor axis.
 *  Car frame: forward, left, up.  The board is mounted so that its X axis
 *  points BACKWARD and its Z axis points DOWN, hence -1 for forward and up.
 *  The sensor axes are right-handed, so left = up x forward = +Y.
 *  Check: nose up must give positive pitch, left side up positive roll.
 * ------------------------------------------------------------------ */
#define S4_SIGN_FWD         (-1.0f)
#define S4_SIGN_LEFT        (1.0f)
#define S4_SIGN_UP          (-1.0f)

/* Magnetometer offset in gauss, caused by the motors and battery (tune:
 * turn the car slowly on the spot with everything mounted and centre the
 * readings).  Leave at 0 until measured. */
#define S4_MAG_OFFSET_FWD   (0.0f)
#define S4_MAG_OFFSET_LEFT  (0.0f)

/* ------------------------------------------------------------------ *
 *  Timing
 * ------------------------------------------------------------------ */
#define S4_SAMPLE_MS        20      /* read the sensor every 20 ms           */
#define S4_REPORT_EVERY     5       /* a status message every 5 samples=100ms*/
#define S4_RETRY_MS         1000    /* sensor missing: try again every 1 s   */
#define S4_I2C_FAIL_LIMIT   3       /* this many failed reads in a row = lost*/

/* ------------------------------------------------------------------ *
 *  Calibration: average the car's readings while it is standing still
 * ------------------------------------------------------------------ */
#define S4_CAL_SAMPLES      100     /* 100 readings 10 ms apart = 1 second   */
#define S4_CAL_GAP_MS       10
#define S4_CAL_MAX_RANGE_G  0.05f   /* more spread than this = car is moving */
#define S4_CAL_RETRIES      3

/* ------------------------------------------------------------------ *
 *  Filters.  filtered += alpha * (new - filtered): a small alpha is smooth
 *  but slow, a big alpha is quick but noisy.
 * ------------------------------------------------------------------ */
#define S4_PITCH_ALPHA      0.25f   /* pitch and roll                        */
#define S4_BASE_ALPHA       0.01f   /* how fast "level ground" is re-learned */
#define S4_SLOW_ALPHA       0.10f   /* slow average of acceleration (gravity)*/
#define S4_TURN_ALPHA       0.20f   /* turn rate                             */

/* ------------------------------------------------------------------ *
 *  Hump detection (tune): angles are relative to level ground
 * ------------------------------------------------------------------ */
#define S4_HUMP_ARM_DEG     1.0f    /* start adding up height above this     */
#define S4_HUMP_ENTER_DEG   4.0f    /* a climb is confirmed above this ...   */
#define S4_HUMP_ENTER_N     3       /* ... for this many samples in a row    */
#define S4_HUMP_DOWN_DEG    (-3.0f) /* below this we are going down          */
#define S4_HUMP_EXIT_DEG    2.0f    /* within +/- this of level = finished   */
#define S4_HUMP_EXIT_N      5       /* ... for this many samples in a row    */
#define S4_HUMP_MAX_MS      4000    /* longer than this is a slope, not a hump*/
#define S4_HUMP_MIN_CM      0.3f    /* smaller than this is just noise       */

/* ------------------------------------------------------------------ *
 *  Picked up or overturned (tune)
 *
 *  Either one makes the car STOP COMPLETELY (see S4_STOP_MOTORS).  Three signs:
 *
 *   OVERTURNED  gravity is more than 60 deg away from the car's "up" for
 *               S4_OVERTURN_N samples: on its side, or upside down.
 *   PICKED UP   the total pull on the sensor is not 1 g: above 1 + S4_LIFT_G
 *               or below 1 - S4_LIFT_G for S4_LIFT_N samples in a row.  Lifting
 *               pushes it above 1 g; a lift or a fall that follows can take it
 *               below.  A knock is far too short to count.
 *   PICKED UP   gravity is more than 35 deg away from "up" for S4_CARRY_N
 *               samples: carried tilted (no slope on the course is that steep).
 *
 *  The firmware has no maths library, so the angle limits are written as the
 *  COSINE SQUARED of the angle (the code compares squares, so it needs no
 *  square root and no cosine):
 *
 *      angle     20      35      45      60      75
 *      cos^2   0.883   0.671   0.500   0.250   0.067
 * ------------------------------------------------------------------ */
#ifndef S4_LIFT_G                   /* (#ifndef: the PC tests try other values) */
#define S4_LIFT_G           0.15f   /* the pull is this far from 1 g (in g)  */
#endif
#ifndef S4_LIFT_N
#define S4_LIFT_N           6       /* ... for 6 samples = 120 ms            */
#endif
#define S4_CARRY_COS2       0.671f  /* tilted more than 35 deg ...           */
#define S4_CARRY_N          25      /* ... for 25 samples = 0.5 s            */
#define S4_OVERTURN_COS2    0.250f  /* tilted more than 60 deg ...           */
#define S4_OVERTURN_N       15      /* ... for 15 samples = 0.3 s            */
#define S4_ARM_MS           500     /* start looking this long after the first
                                     * reading, so the filters have settled  */

/* While the car is stopped, the REASON follows its tilt.  Past 60 deg
 * (S4_OVERTURN_*) it is OVERTURNED.  Once it is back within 45 deg of upright
 * (cos^2 = 0.500) for S4_RIGHTED_N samples it is PICKED_UP again: a hand has
 * turned it back up and it is not resting yet.  The motors stay stopped
 * throughout; only the reported state changes. */
#define S4_RIGHTED_COS2     0.500f
#define S4_RIGHTED_N        15      /* 15 samples = 0.3 s                    */

/* PUT DOWN: the car is resting when it is upright within 20 deg, the pull is
 * within S4_PUTDOWN_G of 1 g, and it is not shaking (S4_STILL_VIB_MG), all for
 * S4_PUTDOWN_MS in a row.  The motors are off by then, so any shaking is a hand
 * (a carried car never rests). */
#define S4_PUTDOWN_COS2     0.883f
#define S4_PUTDOWN_G        0.08f
#ifndef S4_PUTDOWN_MS               /* (#ifndef: the PC tests change it)     */
#define S4_PUTDOWN_MS       2000
#endif

/* What happens once it has been put down:
 *   0 = the condition is released by itself (FAULT_IMU_SENS is cleared and
 *       Subsystem 2 may drive again when it allows it)
 *   1 = it stays latched until s4_clear_lifted() is called.  That is manual
 *       clearance, which is what systemFaults.h says for the TIER 1 mask that
 *       FAULT_IMU_SENS belongs to.  A clear is only accepted while the car is
 *       resting level, so a CLEAR sent while it is still in a hand is ignored.
 *       Have whoever handles the CLEAR command call s4_clear_lifted(). */
#ifndef S4_LIFT_LATCH               /* (#ifndef: the PC tests change it)     */
#define S4_LIFT_LATCH       0
#endif

/* 1 = when the car is picked up or overturned, tell Subsystem 2 to stop the
 *     motors completely, at once: motion_trigger_safety_halt(FAULT_IMU_SENS).
 *     (Subsystem 2's automatic "fault means halt" override is commented out,
 *     so the fault bit alone does not stop the wheels.)
 * 0 = only report the fault bit (for testing without Subsystem 2). */
#define S4_STOP_MOTORS      1

/* ------------------------------------------------------------------ *
 *  Motion classification (tune)
 * ------------------------------------------------------------------ */
#define S4_STILL_VIB_MG     15      /* less shaking than this (mg) = standing*/
#define S4_ACCEL_MG         60      /* forward acceleration change (mg)      */
#define S4_TURN_DPS         30      /* turning faster than this (deg/s)      */
#define S4_TURN_MAX_TILT_DEG 5.0f   /* the compass is only trusted when level*/
#define S4_CLASS_PERSIST    2       /* same answer twice in a row to report  */

/* ------------------------------------------------------------------ *
 *  Working with the other subsystems
 * ------------------------------------------------------------------ */
/* 1 = report the IMU faults to Subsystem 1: s1_report_fault(FAULT_IMU_SENS)
 * while the IMU is not answering OR the car is picked up or overturned, and
 * s1_clear_fault(FAULT_IMU_SENS) when neither is.  The team's tiers decide
 * what the rest of the car does with that bit (systemFaults.h: IMU is in the
 * fatal tier).  0 = keep the faults inside subsystem 4 (for testing without
 * an IMU fitted). */
#define S4_PROPAGATE_FAULTS 1

/* Where the distance driven comes from (the hump height needs it):
 *   1 = Subsystem 2's get_distance_traveled_mm(): the mean of both wheel
 *       encoders, forward is positive.  Subsystem 2 owns the wheel size and the
 *       ticks per turn (WHEEL_DIAMETER_MM, ENCODER_TPR in encoder.h), so there
 *       is nothing to calibrate here.
 *   0 = no encoders: assume the robot drives at S4_FALLBACK_SPEED_CM_S.  Hump
 *       heights are then only rough. */
#ifndef S4_USE_ENCODER_DISTANCE     /* (#ifndef: the PC tests change it) */
#define S4_USE_ENCODER_DISTANCE 1
#endif
#define S4_FALLBACK_SPEED_CM_S 20.0f

/* The most distance one 20 ms sample can really cover, in cm.  A bigger jump
 * is a counter reset (motor_reset_ticks) or a glitch, not driving: it is
 * ignored.  5 cm per sample is 2.5 m/s: far above this car's top speed. */
#define S4_MAX_STEP_CM      5.0f

/* ------------------------------------------------------------------ *
 *  Debug output
 * ------------------------------------------------------------------ */
#define S4_PRINT_STATUS     0       /* quiet for the S5 test */       /* a status line twice a second          */
#define S4_PRINT_CSV        0       /* 1 = one CSV line per sample, for the  */
                                    /* plot tool (turn S4_PRINT_STATUS off)  */

#endif /* S4_CONFIG_H */
