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
 *  Collision (tune)
 * ------------------------------------------------------------------ */
#define S4_IMPACT_G         1.0f    /* sudden change in acceleration (in g)  */
#define S4_IMPACT_HOLDOFF_MS 500    /* ignore the rebound right after a hit  */
/* How long the collision condition stays active (so FAULT_IMU_SENS stays
 * set).  0 = stay active until s4_clear_collision() is called.
 * NOTE: systemFaults.h puts FAULT_IMU_SENS in the TIER 1 (fatal) mask, which
 * says "requires manual clearance".  A timed release is NOT manual.  If the
 * team wants manual clearance, set this to 0 and have whoever handles the
 * CLEAR command call s4_clear_collision(). */
#ifndef S4_COLLISION_HOLD_MS        /* (#ifndef: the PC tests change it) */
#define S4_COLLISION_HOLD_MS 3000
#endif

/* ------------------------------------------------------------------ *
 *  Motion classification (tune)
 * ------------------------------------------------------------------ */
#define S4_STILL_VIB_MG     15      /* less shaking than this (mg) = standing*/
#define S4_ACCEL_MG         60      /* forward acceleration change (mg)      */
#define S4_TURN_DPS         30      /* turning faster than this (deg/s)      */
#define S4_TURN_MAX_TILT_DEG 5.0f   /* the compass is only trusted when level*/
#define S4_CLASS_PERSIST    2       /* same answer twice in a row to report  */
#define S4_IMPACT_SHOW_MS   300     /* how long IMPACT is shown              */

/* ------------------------------------------------------------------ *
 *  Working with the other subsystems
 * ------------------------------------------------------------------ */
/* 1 = report the IMU faults to Subsystem 1: s1_report_fault(FAULT_IMU_SENS)
 * while the IMU is not answering OR a collision is active, and
 * s1_clear_fault(FAULT_IMU_SENS) when neither is.  The team's tiers decide
 * what the rest of the car does with that bit (systemFaults.h: IMU is in the
 * fatal tier).  0 = keep the faults inside subsystem 4 (for testing without
 * an IMU fitted). */
#define S4_PROPAGATE_FAULTS 1

/* Distance per wheel-encoder tick in cm (wheel circumference / ticks per
 * turn).  0 = not known yet: subsystem 2's encoder counters are NOT used
 * and a fixed speed is assumed instead, so hump heights are only rough. */
#ifndef S4_CM_PER_TICK              /* (#ifndef: the PC tests change it) */
#define S4_CM_PER_TICK      (0.0f)
#endif
#define S4_FALLBACK_SPEED_CM_S 20.0f

/* ------------------------------------------------------------------ *
 *  Debug output
 * ------------------------------------------------------------------ */
#define S4_PRINT_STATUS     1       /* a status line twice a second          */
#define S4_PRINT_CSV        0       /* 1 = one CSV line per sample, for the  */
                                    /* plot tool (turn S4_PRINT_STATUS off)  */

#endif /* S4_CONFIG_H */
