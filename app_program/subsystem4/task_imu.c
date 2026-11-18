/*
 *  task_imu.c  --  imu_task: the sensor task (priority 11, every 20 ms).
 *
 *  Each time round the loop it:
 *    1. reads the accelerometer and magnetometer,
 *    2. turns the readings into tilt, turn rate, vibration,
 *    3. runs the hump detector and the picked-up / overturned detector,
 *    4. puts a message in the buffer for report_task.
 *  It is the ONLY code that talks to the sensor.
 *
 *  Read the file from top to bottom: small helpers first, then calibration,
 *  then the hump and lift detectors, then the task itself.
 */

#include "subsystem4.h"
#include "lsm303d.h"
#include <stdint.h>

/* Subsystem 2's distance driven (IF-06): the mean of both wheel encoders in
 * millimetres, forward positive.  Declared here and not by including
 * subsystem2.h: that header is Subsystem 2's own business, and this one
 * function is all we use. */
extern float get_distance_traveled_mm(void);

/* A free-running 1 MHz hardware timer (RP2040 TIMERAWL).  The kernel's own
 * clock only ticks every 1 ms, which is too coarse for a 20 ms loop.  It wraps
 * after 71 minutes, which does not matter because we only ever subtract two
 * readings. */
#ifdef S4_HOST_SIM
extern UW s4_sim_timer_us(void);        /* PC simulation */
#define TIMER_US()      s4_sim_timer_us()
#else
#define TIMER_US()      (*(volatile UW*)0x40054028)
#endif

/* ======================================================================
 *  1. Small helpers
 * ====================================================================== */

typedef struct { float fwd, left, up; } CAR_VEC;   /* a reading in CAR axes */

/* Sensor axes -> car axes (signs in s4_config.h). */
static CAR_VEC to_car(const float s[3])
{
    CAR_VEC v;

    v.fwd  = S4_SIGN_FWD  * s[0];
    v.left = S4_SIGN_LEFT * s[1];
    v.up   = S4_SIGN_UP   * s[2];
    return v;
}

static float absf(float x)
{
    return (x < 0.0f) ? -x : x;
}

static float clampf(float x, float lo, float hi)
{
    return (x < lo) ? lo : ((x > hi) ? hi : x);
}

/* The template is built without the maths library, so there is no atan() or
 * sin().  These two small formulas are accurate enough for tilt angles:
 *
 *   atan(t) ~= t / (1 + 0.28 t^2)        for -1 <= t <= 1  (error < 0.3 deg)
 *   sin(x)  ~= x - x^3 / 6               for small angles x (in radians)
 */
static float atan_deg(float t)
{
    t = clampf(t, -1.0f, 1.0f);
    return 57.2958f * t / (1.0f + 0.28f * t * t);
}

static float sin_deg(float deg)
{
    float rad = deg * 0.0174533f;

    return rad - (rad * rad * rad) / 6.0f;
}

static UW now_ms(void)
{
    SYSTIM t;

    (void)tk_get_otm(&t);
    return t.lo;
}

/* Tell report_task something.  TMO_POL = never wait: if the buffer is full the
 * message is dropped rather than slowing the 20 ms loop down. */
static void send_msg(S4_MSG *m)
{
    (void)tk_snd_mbf(s4_mbf, m, (INT)sizeof(*m), TMO_POL);
}

static void send_fault_msg(void)
{
    S4_MSG m = {0};

    m.type           = S4_MSG_FAULT;
    m.time16         = (UINT)(now_ms() & 0xFFFFU);
    m.p.fault.faults = (UINT)s4_get_faults() & 0x3U;
    send_msg(&m);
}

/* ======================================================================
 *  2. Distance travelled (for hump height)
 *
 *  Height gained = distance driven along the slope * sin(slope angle).
 *  The distance comes from Subsystem 2's wheel encoders.  With
 *  S4_USE_ENCODER_DISTANCE 0 we assume the car drives at a fixed speed.
 * ====================================================================== */
static float distance_cm(void)
{
#if S4_USE_ENCODER_DISTANCE
    return get_distance_traveled_mm() * 0.1f;       /* mm -> cm */
#else
    return 0.0f;
#endif
}

/* ======================================================================
 *  3. Calibration
 *
 *  A real sensor never reads exactly 0 g / 0 g / 1 g when level (the
 *  datasheet allows about +/-60 mg, which is 3.4 degrees of fake tilt, as big
 *  as a hump).  So at start-up, with the car standing still on level ground,
 *  we average a second of readings and remember how far they are from
 *  (0, 0, 1).  That "offset" is subtracted from every later reading.
 * ====================================================================== */
static CAR_VEC g_offset;        /* subtracted from every reading */
static BOOL    calibrated;      /* imu_task only; others read the S4_FLG_CALIBRATED bit */

static BOOL calibrate(void)
{
    UINT attempt, i, axis;

    for (attempt = 0; attempt < S4_CAL_RETRIES; attempt++) {
        float sum[3] = {0, 0, 0};
        float lo[3]  = {9, 9, 9};
        float hi[3]  = {-9, -9, -9};
        float mean[3], range = 0.0f;
        UINT  n = 0;

        for (i = 0; i < S4_CAL_SAMPLES; i++) {
            float   raw[3];
            CAR_VEC a;
            float   v[3];

            if (lsm303d_read_accel(raw) == E_OK) {
                a = to_car(raw);
                v[0] = a.fwd;  v[1] = a.left;  v[2] = a.up;
                for (axis = 0; axis < 3; axis++) {
                    sum[axis] += v[axis];
                    if (v[axis] < lo[axis]) lo[axis] = v[axis];
                    if (v[axis] > hi[axis]) hi[axis] = v[axis];
                }
                n++;
            }
            tk_dly_tsk(S4_CAL_GAP_MS);
        }

        if (n < S4_CAL_SAMPLES / 2) {
            S4_LOG("calibration: too many failed reads\n");
            continue;
        }
        for (axis = 0; axis < 3; axis++) {
            mean[axis] = sum[axis] / (float)n;
            if (hi[axis] - lo[axis] > range) range = hi[axis] - lo[axis];
        }

        if (range > S4_CAL_MAX_RANGE_G) {
            S4_LOG("calibration: the car is moving, keep it still and level\n");
        } else if (mean[2] < -0.8f) {
            S4_LOG("calibration: up axis reads %d mg (negative): flip S4_SIGN_UP in s4_config.h\n",
                   (INT)(mean[2] * 1000.0f));
        } else if ((mean[2] < 0.8f) || (mean[2] > 1.2f)) {
            S4_LOG("calibration: up axis reads %d mg, expected about 1000: check the axis "
                   "mapping and the range setting\n", (INT)(mean[2] * 1000.0f));
        } else {
            g_offset.fwd  = mean[0];            /* should be 0 g */
            g_offset.left = mean[1];            /* should be 0 g */
            g_offset.up   = mean[2] - 1.0f;     /* should be 1 g */
            S4_LOG("calibrated, spread %d mg\n", (INT)(range * 1000.0f));
            return TRUE;
        }
        tk_dly_tsk(500);
    }

    /* Run anyway, uncalibrated: the fault bit is NOT raised (the car can
     * still drive), but tilt will be off by the sensor's offset. */
    g_offset.fwd = g_offset.left = g_offset.up = 0.0f;
    S4_LOG("calibration FAILED: running uncalibrated\n");
    return FALSE;
}

/* ======================================================================
 *  4. Hump detector
 *
 *  A hump is "climb, then descend".  We watch the pitch relative to level
 *  ground and add up the height gained, sample by sample:
 *
 *      height += distance driven * sin(pitch)
 *
 *  FLAT  --(pitch above 4 deg for 3 samples)-->  UP
 *  UP    --(pitch below -3 deg)------------------>  DOWN
 *  DOWN  --(pitch back within 2 deg of level)---->  FLAT, hump measured
 *
 *  While FLAT but slightly tilted (above 1 deg) we already add up the height,
 *  so the first part of the climb, before it was confirmed, is not lost.
 *  A "hump" that lasts over 4 s is a slope, not a hump: it is thrown away.
 * ====================================================================== */
typedef enum { HUMP_FLAT = 0, HUMP_UP, HUMP_DOWN } HUMP_STATE;

static HUMP_STATE hump;
static float      height_cm, length_cm, peak_cm;
static UB         enter_n, exit_n;     /* count 0..5: a byte is plenty */
static UW         hump_start_ms;
static float      base_deg;     /* the pitch of "level ground" */

static void hump_reset(void)
{
    hump = HUMP_FLAT;
    height_cm = length_cm = peak_cm = 0.0f;
    enter_n = exit_n = 0;
}

static void hump_finished(UW t_ms)
{
    if (peak_cm >= S4_HUMP_MIN_CM) {
        S4_MSG m = {0};
        INT    peak_mm = s4_round(peak_cm * 10.0f);

        /* Results are packed here, at the boundary.  The working values
         * (height_cm, peak_cm) stay float. */
        if (s4_hump_count < 255U) {
            s4_hump_count++;                        /* saturates, never wraps */
        }
        if (peak_mm > (INT)s4_max_hump_mm) {
            s4_max_hump_mm = (UB)s4_clip_u(peak_mm, 8);
        }
        m.type          = S4_MSG_HUMP;
        m.time16        = t_ms & 0xFFFFU;
        m.p.hump.peak   = s4_clip_u(s4_round(peak_cm * 10.0f / (float)S4_PEAK_STEP_MM), 7) & 0x7FU;
        m.p.hump.length = (UB)s4_clip_u(s4_round(length_cm * 10.0f / (float)S4_LENGTH_STEP_MM), 8);
        send_msg(&m);
    }
    hump_reset();
}

static void hump_update(float pitch_rel, float ds_cm, UW t_ms)
{
    float s = sin_deg(pitch_rel);

    if (hump == HUMP_FLAT) {
        /* Slightly tilted: start adding up already, in case this is a climb. */
        if (pitch_rel > S4_HUMP_ARM_DEG) {
            height_cm += ds_cm * s;
            length_cm += ds_cm;
        } else {
            height_cm = length_cm = 0.0f;
        }
        /* Clearly tilted for several samples in a row: it is a climb. */
        if (pitch_rel > S4_HUMP_ENTER_DEG) {
            if (++enter_n >= S4_HUMP_ENTER_N) {
                hump = HUMP_UP;
                peak_cm = height_cm;
                hump_start_ms = t_ms;
            }
        } else {
            enter_n = 0;
        }
        return;
    }

    /* UP or DOWN: keep adding up and remember the highest point. */
    height_cm += ds_cm * s;
    length_cm += ds_cm;
    if (height_cm > peak_cm) {
        peak_cm = height_cm;
    }

    if (hump == HUMP_UP && pitch_rel < S4_HUMP_DOWN_DEG) {
        hump = HUMP_DOWN;
        exit_n = 0;
    }
    if (hump == HUMP_DOWN) {
        if (absf(pitch_rel) < S4_HUMP_EXIT_DEG) {
            if (++exit_n >= S4_HUMP_EXIT_N) {
                hump_finished(t_ms);
                return;
            }
        } else {
            exit_n = 0;
        }
    }

    /* Too long to be a hump: it is a slope.  Forget it and take the current
     * tilt as the new "level". */
    if ((t_ms - hump_start_ms) > S4_HUMP_MAX_MS) {
        hump_reset();
        base_deg += pitch_rel;
    }
}

/* ======================================================================
 *  5. Motion filters: gravity and shaking
 *
 *  Gravity changes slowly (the car tilts slowly), a knock or a shake changes
 *  the reading suddenly.  So keep a slow average of the three readings (= the
 *  direction of gravity) and look at how far the newest reading is from it.
 *  The biggest distance, smoothed, is how much the car is shaking.  It tells
 *  "standing still" from "driving", and "resting on the floor" from "in a
 *  hand".  A knock is no longer a fault: it only shows up as shaking.
 * ====================================================================== */
static float   slow[3];             /* slow average of the readings (fwd,left,up) */
static float   vibration_g;

static void filters_update(const CAR_VEC *a)
{
    float v[3], dyn = 0.0f;
    UINT  axis;

    v[0] = a->fwd;  v[1] = a->left;  v[2] = a->up;
    for (axis = 0; axis < 3; axis++) {
        if (absf(v[axis] - slow[axis]) > dyn) {
            dyn = absf(v[axis] - slow[axis]);
        }
        slow[axis] += S4_SLOW_ALPHA * (v[axis] - slow[axis]);
    }
    vibration_g += S4_SLOW_ALPHA * (dyn - vibration_g);
}

/* ======================================================================
 *  5b. Picked up / overturned detector
 *
 *  Either one makes the car STOP COMPLETELY: the motors are halted through
 *  Subsystem 2 (s4_stop_motors) and the IMU fault bit is raised.  Three signs
 *  (the numbers are in s4_config.h):
 *
 *   - OVERTURNED: gravity is far from the car's "up" for 0.3 s.  Since gravity
 *     is the reading of the slow average, a shove or a knock does not move it.
 *   - PICKED UP, by force: the total pull on the sensor, |a|, is not 1 g.
 *     At rest, and while driving, it is 1 g whatever the tilt.  Lifting
 *     pushes it above 1 g; setting down, or a fall, takes it below.  It has to
 *     last 120 ms: a knock lasts one or two samples.
 *   - PICKED UP, tilted: gravity is more than 35 deg from "up" for 0.5 s.
 *
 *  Which of the two it is follows the tilt (60 deg to become OVERTURNED, back
 *  within 45 deg to become PICKED_UP again), so a tipped car that is righted by
 *  hand reads PICKED_UP, and a lifted car turned over in the hand reads
 *  OVERTURNED.  A car HELD tilted past 60 deg reads OVERTURNED too: tilt is
 *  all the sensor can see.
 *
 *  Once it has fired the condition is held until the car is PUT DOWN: upright,
 *  1 g, and not shaking, for S4_PUTDOWN_MS.  The motors are off, so any
 *  shaking is a hand: a carried car never counts as put down.  Everything is
 *  compared as squares, so there is no square root in the 20 ms loop (the
 *  one in tilt_deg() is only used when an event is reported).
 *
 *  Nothing fires until the sensor is calibrated (an uncalibrated car does not
 *  know which way is up) and S4_ARM_MS after the first reading.
 * ====================================================================== */
#define S4_ARM_SAMPLES  ((S4_ARM_MS + S4_SAMPLE_MS - 1) / S4_SAMPLE_MS)
#define S4_LIFT_HI2     ((1.0f + S4_LIFT_G) * (1.0f + S4_LIFT_G))
#define S4_LIFT_LO2     ((1.0f - S4_LIFT_G) * (1.0f - S4_LIFT_G))
#define S4_REST_HI2     ((1.0f + S4_PUTDOWN_G) * (1.0f + S4_PUTDOWN_G))
#define S4_REST_LO2     ((1.0f - S4_PUTDOWN_G) * (1.0f - S4_PUTDOWN_G))
#define S4_REST_VIB_G   ((float)S4_STILL_VIB_MG / 1000.0f)

static UB      lift_state;          /* S4_LIFT_*; written here only          */
static UB      force_n, carry_n, over_n, right_n;   /* samples in a row showing each sign */
static UB      armed_n;             /* samples since the first one (saturates) */
static BOOL    rest_timing;         /* is the "resting" clock running?       */
static UW      rest_start_ms;

static UB count_up(UB n)   { return (n < 255U) ? (UB)(n + 1U) : n; }
static UB count_down(UB n) { return (n > 0U)   ? (UB)(n - 1U) : n; }

/* Square root by Newton's method (no maths library).  Only used to report the
 * tilt when an event happens, never in the sampling loop. */
static float sqrt_f(float x)
{
    float r = (x > 1.0f) ? (x * 0.5f) : 1.0f;
    UINT  i;

    if (x <= 0.0f) {
        return 0.0f;
    }
    for (i = 0; i < 8; i++) {
        r = 0.5f * (r + x / r);
    }
    return r;
}

/* The angle between gravity and the car's "up" axis, 0 (upright) to 180
 * (upside down), from the slow average. */
static float tilt_deg(void)
{
    float horiz = sqrt_f(slow[0] * slow[0] + slow[1] * slow[1]);
    float up    = slow[2];
    float a;

    if (absf(up) >= horiz) {
        a = (absf(up) > 0.0f) ? atan_deg(horiz / absf(up)) : 90.0f;   /* 0..45 from vertical */
        return (up >= 0.0f) ? a : (180.0f - a);
    }
    a = atan_deg(absf(up) / horiz);                                   /* 0..45 from horizontal */
    return (up >= 0.0f) ? (90.0f - a) : (90.0f + a);
}

/* Is the car lying still and level? */
static BOOL is_resting(void)
{
    float g2 = slow[0] * slow[0] + slow[1] * slow[1] + slow[2] * slow[2];

    return ((slow[2] > 0.0f) &&
            (slow[2] * slow[2] > S4_PUTDOWN_COS2 * g2) &&
            (g2 < S4_REST_HI2) && (g2 > S4_REST_LO2) &&
            (vibration_g < S4_REST_VIB_G)) ? TRUE : FALSE;
}

static void send_lift_msg(UINT reason)
{
    S4_MSG m = {0};

    m.type          = S4_MSG_LIFT;
    m.time16        = (UINT)(now_ms() & 0xFFFFU);
    m.p.lift.reason = reason & 0x3U;
    m.p.lift.tilt   = s4_clip_u(s4_round(tilt_deg()), 8);
    send_msg(&m);
}

/* The car has just been picked up or overturned.  STOP FIRST (the motors are
 * the point), then raise the fault bit and tell report_task. */
static void lift_start(UB reason)
{
    lift_state  = reason;
    force_n = carry_n = over_n = right_n = 0U;
    rest_timing = FALSE;
    s4_stop_motors();
    s4_fault_set(S4_FAULT_LIFTED);
    send_lift_msg(reason);
    send_fault_msg();
}

/* The car is still stopped but the REASON has changed: it was tipped over while
 * being carried, or righted by hand.  Report it; the motors are already
 * stopped and the fault bit is already set, so neither is touched. */
static void lift_reason_change(UB reason)
{
    lift_state = reason;
    over_n = right_n = 0U;
    send_lift_msg(reason);
}

/* The car has been put down (or a clear was accepted). */
static void lift_release(void)
{
    lift_state  = S4_LIFT_NONE;
    force_n = carry_n = over_n = right_n = 0U;
    rest_timing = FALSE;
    s4_fault_clear(S4_FAULT_LIFTED);
    send_lift_msg(S4_LIFT_NONE);
    send_fault_msg();
}

/* The sensor was (re)started: forget what we thought we knew and wait for the
 * filters to settle again.  If it was lifted, the sensor going away and coming
 * back means we no longer know: start again from "on the ground". */
static void lift_reset(void)
{
    force_n = carry_n = over_n = right_n = 0U;
    armed_n = 0U;
    rest_timing = FALSE;
    if (lift_state != S4_LIFT_NONE) {
        lift_state = S4_LIFT_NONE;
        s4_fault_clear(S4_FAULT_LIFTED);
        send_fault_msg();
    }
}

static void lift_update(const CAR_VEC *a, UW t_ms)
{
    UINT  bits;
    /* A "clear" request (s4_clear_lifted) is consumed every sample, even when
     * nothing is lifted, so an old request can not release a later event. */
    BOOL  clear_req = (tk_wai_flg(s4_flg, S4_FLG_CLEAR_REQ, TWF_ORW | TWF_BITCLR,
                                  &bits, TMO_POL) == E_OK) ? TRUE : FALSE;
    float g2 = slow[0] * slow[0] + slow[1] * slow[1] + slow[2] * slow[2];
    float up = slow[2];
    float m2 = a->fwd * a->fwd + a->left * a->left + a->up * a->up;

    armed_n = count_up(armed_n);
    if ((!calibrated) || (armed_n < S4_ARM_SAMPLES)) {
        return;
    }

    if (lift_state == S4_LIFT_NONE) {
        BOOL force = ((m2 > S4_LIFT_HI2) || (m2 < S4_LIFT_LO2)) ? TRUE : FALSE;
        BOOL carry = ((up <= 0.0f) || (up * up < S4_CARRY_COS2 * g2)) ? TRUE : FALSE;
        BOOL over  = ((up <= 0.0f) || (up * up < S4_OVERTURN_COS2 * g2)) ? TRUE : FALSE;

        /* "In a row" is forgiving: a sample that misses counts one down, it
         * does not wipe the count (noise would otherwise hide a real lift). */
        force_n = force ? count_up(force_n) : count_down(force_n);
        carry_n = carry ? count_up(carry_n) : 0U;
        over_n  = over  ? count_up(over_n)  : 0U;

        if (over_n >= S4_OVERTURN_N) {
            lift_start(S4_LIFT_OVERTURNED);
        } else if ((force_n >= S4_LIFT_N) || (carry_n >= S4_CARRY_N)) {
            /* Lifted (or carried tilted).  If it is already tipped past the
             * overturn limit, that is what it is. */
            lift_start(over ? S4_LIFT_OVERTURNED : S4_LIFT_PICKED_UP);
        }
        return;
    }

    /* Stopped.  The REASON follows the tilt: tipped past the limit it is
     * OVERTURNED (also when a hand turns a lifted car over); back within 45 deg
     * of upright it is PICKED_UP again (a hand has righted it; it is not
     * resting yet).  Hysteresis: 60 deg to enter, 45 deg to leave. */
    {
        BOOL over_now = ((up <= 0.0f) || (up * up < S4_OVERTURN_COS2 * g2)) ? TRUE : FALSE;
        BOOL righted  = ((up > 0.0f) && (up * up > S4_RIGHTED_COS2 * g2)) ? TRUE : FALSE;

        over_n  = over_now ? count_up(over_n)  : 0U;
        right_n = righted  ? count_up(right_n) : 0U;
        if ((lift_state == S4_LIFT_PICKED_UP) && (over_n >= S4_OVERTURN_N)) {
            lift_reason_change(S4_LIFT_OVERTURNED);
        } else if ((lift_state == S4_LIFT_OVERTURNED) && (right_n >= S4_RIGHTED_N)) {
            lift_reason_change(S4_LIFT_PICKED_UP);
        }
    }

    /* Picked up or overturned: wait until it is resting level again. */
    {
        BOOL resting = is_resting();
        BOOL release = FALSE;

        if (resting) {
            if (!rest_timing) {
                rest_timing   = TRUE;
                rest_start_ms = t_ms;
            }
        } else {
            rest_timing = FALSE;
        }

#if !S4_LIFT_LATCH
        if (rest_timing && ((t_ms - rest_start_ms) >= S4_PUTDOWN_MS)) {
            release = TRUE;                 /* it has been resting long enough */
        }
#endif
        if (clear_req) {
            if (resting) {
                release = TRUE;             /* a clear is accepted while resting level */
            } else {
                S4_LOG("clear ignored: the car is not resting level\n");
            }
        }
        if (release) {
            lift_release();
        }
    }
}

/* ======================================================================
 *  6. Turn rate, from the magnetometer (compass)
 *
 *  This board has no gyroscope, so we watch the compass direction instead.
 *  The compass vector in car axes is (m.fwd, m.left).  Between two readings
 *  it turns by a small angle d, and
 *
 *      (x0 * y1 - y0 * x1) / (x0^2 + y0^2)  =  d   (in radians, small d)
 *
 *  Turning the car LEFT turns the compass vector RIGHT, so the sign is
 *  flipped to make "left" positive.  The compass is only trusted while the
 *  car is level (tilt would change the readings), and it is thrown off by
 *  the motors: measure S4_MAG_OFFSET_* in s4_config.h.
 * ====================================================================== */
static float  mf0, ml0;             /* the previous compass reading */
static BOOL   mag_prev_ok;
static float  turn_f;

static void turn_update(float mf, float ml, float dt, BOOL level)
{
    float m2 = mf0 * mf0 + ml0 * ml0;

    if (level && mag_prev_ok && (m2 > 0.01f) && (dt > 0.0f)) {
        float cross = mf0 * ml - ml0 * mf;
        float raw   = -(cross / m2) / dt * 57.2958f;     /* degrees per second */

        turn_f += S4_TURN_ALPHA * (raw - turn_f);
    } else {
        turn_f = 0.0f;
    }
    mf0 = mf;
    ml0 = ml;
    mag_prev_ok = level;
}

/* ======================================================================
 *  7. One sample: read, calculate, detect
 * ====================================================================== */
static float   pitch_f, roll_f;     /* filtered tilt in degrees */
static float   accel_hp_g;          /* forward acceleration change */
static UW      last_us;
static UW      last_mag_us;     /* when the compass was last read */
static float   last_dist;
static BOOL    first_sample = TRUE;

static void reset_filters(void)
{
    first_sample = TRUE;
    mag_prev_ok  = FALSE;
    turn_f       = 0.0f;
    hump_reset();
    lift_reset();
}

/* Returns FALSE if the sensor could not be read. */
static BOOL process_sample(void)
{
    float   raw_a[3], raw_m[3];
    CAR_VEC a;
    float   mf, ml, dist, dt, mag_dt, ds, pitch_raw, roll_raw, pitch_rel;
    UW      t_us, t_ms, mag_us;

    /* The time and the distance are noted back to back, so they describe the
     * same instant even if another task interrupts us a moment later. */
    t_us = TIMER_US();
    t_ms = now_ms();
    dist = distance_cm();

    if (lsm303d_read_accel(raw_a) != E_OK) return FALSE;
    if (lsm303d_read_mag(raw_m)   != E_OK) return FALSE;
    mag_us = TIMER_US();    /* the turn rate must use the time the COMPASS was read */

    /* Raw readings -> car axes, with the calibration offset removed. */
    a = to_car(raw_a);
    a.fwd  -= g_offset.fwd;
    a.left -= g_offset.left;
    a.up   -= g_offset.up;
    mf = S4_SIGN_FWD  * raw_m[0] - S4_MAG_OFFSET_FWD;
    ml = S4_SIGN_LEFT * raw_m[1] - S4_MAG_OFFSET_LEFT;

    /* Time step and distance step since the last sample. */
    dt = first_sample ? ((float)S4_SAMPLE_MS / 1000.0f)
                      : ((float)(UW)(t_us - last_us) * 1.0e-6f);
    (void)dt;               /* only the no-encoder fallback below uses it */
#if S4_USE_ENCODER_DISTANCE
    ds = first_sample ? 0.0f : (dist - last_dist);
    if ((ds > S4_MAX_STEP_CM) || (ds < -S4_MAX_STEP_CM)) {
        ds = 0.0f;                  /* a counter reset or a glitch, not driving */
    }
#else
    ds = S4_FALLBACK_SPEED_CM_S * dt;
#endif
    mag_dt = first_sample ? ((float)S4_SAMPLE_MS / 1000.0f)
                          : ((float)(UW)(mag_us - last_mag_us) * 1.0e-6f);
    last_us = t_us;
    last_mag_us = mag_us;
    last_dist = dist;

    /* Tilt.  When the car is tilted, gravity shows up on the forward axis
     * (nose up) and the left axis (left side up).  tan(tilt) = sideways
     * part / up part, so any scale error that is the same on both axes
     * cancels out. */
    if (a.up > 0.3f) {
        pitch_raw = atan_deg(a.fwd  / a.up);
        roll_raw  = atan_deg(a.left / a.up);
    } else {
        pitch_raw = pitch_f;                /* upside down: keep the last */
        roll_raw  = roll_f;
    }

    if (first_sample) {
        pitch_f = pitch_raw;
        roll_f  = roll_raw;
        base_deg = pitch_f;                 /* assume we start on level ground */
        slow[0] = a.fwd;  slow[1] = a.left;  slow[2] = a.up;
        vibration_g = 0.0f;
        first_sample = FALSE;
    } else {
        pitch_f += S4_PITCH_ALPHA * (pitch_raw - pitch_f);
        roll_f  += S4_PITCH_ALPHA * (roll_raw  - roll_f);
    }
    pitch_rel = pitch_f - base_deg;

    /* Slowly re-learn "level ground" while flat and almost level, so a
     * small drift or a gentle slope does not look like a hump. */
    if ((hump == HUMP_FLAT) && (absf(pitch_rel) < S4_HUMP_ARM_DEG)) {
        base_deg += S4_BASE_ALPHA * (pitch_f - base_deg);
    }

    /* Forward acceleration change = the forward reading minus its slow average. */
    accel_hp_g = a.fwd - slow[0];

    {
        UINT bits;

        /* s4_reset_run() leaves a bit in the event flag; consume it here. */
        if (tk_wai_flg(s4_flg, S4_FLG_RESET_REQ, TWF_ORW | TWF_BITCLR, &bits, TMO_POL) == E_OK) {
            s4_max_hump_mm = 0U;
            s4_hump_count  = 0U;
        }
    }

    hump_update(pitch_rel, ds, t_ms);
    filters_update(&a);
    lift_update(&a, t_ms);
    s4_lift = lift_state;
    turn_update(mf, ml, mag_dt,
                (absf(pitch_f) < S4_TURN_MAX_TILT_DEG) &&
                (absf(roll_f)  < S4_TURN_MAX_TILT_DEG));

    /* Publish pitch, roll and turn rate together in ONE store, so a reader
     * never sees a pitch from one sample and a roll from the next. */
    s4_snap = s4_snap_pack(s4_clip_s(s4_round(pitch_f), 8),
                           s4_clip_s(s4_round(roll_f), 8),
                           s4_clip_s(s4_round(turn_f / (float)S4_TURN_STEP_DPS), 8));
    s4_samples++;

#if S4_PRINT_CSV
    APP_LOG((UB*)"%d,%d,%d,%d,%d,%d,%d,%d\n", (INT)t_ms,
              (INT)(a.fwd * 1000.0f), (INT)(a.left * 1000.0f),
              (INT)(a.up * 1000.0f), (INT)(pitch_f * 100.0f),
              (INT)(pitch_rel * 100.0f), (INT)(height_cm * 10.0f), (INT)hump);
#endif
    return TRUE;
}

static void send_status(void)
{
    S4_MSG m = {0};

    m.type             = S4_MSG_STATUS;
    m.time16           = now_ms() & 0xFFFFU;
    m.hump_state       = (UINT)hump & 0x3U;
    m.vibration        = (UB)s4_clip_u(s4_round(vibration_g * 1000.0f), 8);    /* mg   */
    m.lift             = (UINT)lift_state & 0x3U;
    m.p.status.pitch   = (B)s4_clip_s(s4_round(pitch_f), 8);                  /* deg  */
    m.p.status.roll    = (B)s4_clip_s(s4_round(roll_f), 8);
    m.p.status.turn    = (B)s4_clip_s(s4_round(turn_f / (float)S4_TURN_STEP_DPS), 8);
    m.p.status.accel   = (B)s4_clip_s(s4_round(accel_hp_g * 1000.0f / (float)S4_ACCEL_STEP_MG), 8);
    send_msg(&m);
}

/* ======================================================================
 *  8. The task
 * ====================================================================== */
void imu_task(INT stacd, void *exinf)
{
    BOOL ok = FALSE;        /* is the sensor working? */
    UINT fail = 0;          /* failed reads in a row  */
    UINT tick = 0;          /* samples since the last status message */
    UINT cal_wait_ms = 0;   /* time since the last failed calibration */

    (void)stacd;
    (void)exinf;

#if S4_PRINT_CSV
    APP_LOG((UB*)"t_ms,ax_mg,ay_mg,az_mg,pitch_cdeg,prel_cdeg,height_mm,hump\n");
#endif

    for (;;) {
        if (!ok) {
            /* (Re)start the sensor.  Not answering is a FAULT: the motors halt. */
            ER err = lsm303d_init();

            if (err != E_OK) {
                S4_LOG("IMU not answering (error %d): check 3.3V/GND, SDA on GP26, "
                       "SCL on GP27\n", (INT)err);
                if ((s4_get_faults() & S4_FAULT_COMM) == 0U) {
                    s4_fault_set(S4_FAULT_COMM);
                    send_fault_msg();
                }
                tk_dly_tsk(S4_RETRY_MS);
                continue;
            }
            S4_LOG("IMU found (LSM303D)\n");
            if (!calibrated) {
                calibrated = calibrate();       /* the car must be still */
                if (calibrated) {
                    (void)tk_set_flg(s4_flg, S4_FLG_CALIBRATED);
                }
            }
            if ((s4_get_faults() & S4_FAULT_COMM) != 0U) {
                s4_fault_clear(S4_FAULT_COMM);
                send_fault_msg();
            }
            reset_filters();
            ok = TRUE;
            fail = 0;
        }

       /* 1. Record exactly when we start the work using the 1 MHz hardware timer */
        UW start_us = TIMER_US();

        /* 2. Do the physical I2C reading */
        if (process_sample()) {
            fail = 0;
            if (++tick >= S4_REPORT_EVERY) {
                tick = 0;
                s4_fault_assert();      /* re-report if the bit went missing; after a release, make sure it stays clear */
                send_status();
            }
        } else if (++fail >= S4_I2C_FAIL_LIMIT) {
            S4_LOG("IMU lost\n");
            s4_fault_set(S4_FAULT_COMM);
            send_fault_msg();
            ok = FALSE;
        }

        /* Calibration failed (the car was moving during start-up): try it
         * again every S4_CAL_AGAIN_MS until it works.  The Vehicle Controller
         * keeps the car in INIT until it does (S4_FLG_CALIBRATED), so the
         * car is standing still here and nothing is driving it. */
        if (ok && !calibrated) {
            cal_wait_ms += S4_SAMPLE_MS;
            if (cal_wait_ms >= S4_CAL_AGAIN_MS) {
                cal_wait_ms = 0;
                S4_LOG("calibration: trying again\n");
                calibrated = calibrate();       /* the car must be still */
                if (calibrated) {
                    (void)tk_set_flg(s4_flg, S4_FLG_CALIBRATED);
                    reset_filters();
                }
            }
        }
        
        /* 3. Calculate time spent in microseconds, convert to milliseconds, and sleep the remainder */
        UW elapsed_us = TIMER_US() - start_us;
        INT wait_time_ms = (INT)S4_SAMPLE_MS - (INT)(elapsed_us / 1000U);
        tk_dly_tsk((wait_time_ms > 0) ? (RELTIM)wait_time_ms : 1U);
    }
}
