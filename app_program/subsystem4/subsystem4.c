/*
 *  subsystem4.c  --  start-up, shared data, and the links to Subsystem 1.
 *
 *  subsystem4_init()   creates the message buffer and the event flag, and
 *                      starts I2C unit 1
 *  subsystem4_start()  starts the two tasks
 */

#include "subsystem4.h"
#include "../systemFaults.h"            /* FAULT_IMU_SENS                    */
#include "../subsystem1/subsystem1.h"   /* s1_set_pitch, s1_report_fault ... */
#include "../../device/include/dev_i2c.h"   /* dev_init_i2c                  */

/* ---- Shared data (declared in subsystem4.h; the writer is named there) -- */
ID                  s4_mbf;
ID                  s4_flg;
volatile UW         s4_snap;
volatile UB         s4_motion;
volatile UB         s4_lift;
volatile UB         s4_hump_count;
volatile UB         s4_max_hump_mm;
volatile UH         s4_samples;

/* Our two fault conditions, one bit each.  Only imu_task changes it, so the
 * "read, change a bit, write back" is safe. */
static volatile UB  s4_faults;

/* After the last condition is over we ask Subsystem 1 to clear the bit.  But
 * every s1_report_fault() ALSO leaves a bit in Subsystem 1's event flag, and
 * Subsystem 1's LED task copies whatever it finds there back into the fault
 * mask, up to one S1_TICK_MS later.  A report still waiting in that flag would
 * therefore bring the fault BACK right after we cleared it, and nothing would
 * clear it again.  So for a short while after a release we check, and clear
 * again if the bit came back (two LED ticks, counted in status messages). */
#define S4_RELEASE_WATCH_TICKS  ((2U * S1_TICK_MS) / (S4_SAMPLE_MS * S4_REPORT_EVERY) + 2U)
static volatile UB  s4_release_watch;

/* Subsystem 2's emergency override (IF-10): halts both wheels at once (PWM 0/0),
 * reports the fault bit to Subsystem 1 itself, and keeps the halt for as long as
 * a fault bit is set in Subsystem 1's mask.  Declared here, and not by including
 * subsystem2.h, for the same reason as get_distance_traveled_mm() in
 * task_imu.c: that header is Subsystem 2's own business and this one function
 * is all we use.  The prototype is exactly the one in subsystem2.h. */
extern void motion_trigger_safety_halt(UINT fault_bit);

/* TEMPORARY, see subsystem4.h: 0 = no such object, so S3's example send fails quietly. */
ID                  mbf_summary = 0;
ID                  flg_events = 0;

/* ---- Faults through Subsystem 1 (IF-07) ----------------------------------
 * The team has one IMU bit, FAULT_IMU_SENS.  Subsystem 1 owns the fault
 * word, so we never touch it ourselves: we ask Subsystem 1 to set or clear
 * the bit (each call takes Subsystem 1's lock, so it is race-free).
 *
 * The bit is ON while at least one of our two conditions is active.  We
 * report on every change.  s4_fault_assert() runs every status message and
 * does two jobs:
 *   - while a condition lasts: if the bit is MISSING (somebody cleared it
 *     while the problem is still here) report it again.  It reports ONLY when
 *     the bit is missing, never "just in case": see s4_release_watch above.
 *     If the car is picked up or overturned, the motors are stopped again as
 *     well: Subsystem 2 holds its halt only while a fault bit is set, so a
 *     missing bit means the halt may have ended.
 *   - just after a release: make sure the bit stays cleared. */
void s4_fault_assert(void)
{
#if S4_PROPAGATE_FAULTS
    S1_TELEMETRY_PKT snap;

    if (s4_faults != 0U) {
        s1_get_telemetry(&snap);
        if ((snap.fault_mask & FAULT_IMU_SENS) == 0U) {
            s1_report_fault(FAULT_IMU_SENS);
            if ((s4_faults & S4_FAULT_LIFTED) != 0U) {
                s4_stop_motors();
            }
        }
    } else if (s4_release_watch != 0U) {
        s4_release_watch--;
        s1_get_telemetry(&snap);
        if ((snap.fault_mask & FAULT_IMU_SENS) != 0U) {
            s1_clear_fault(FAULT_IMU_SENS);         /* a stale report came back: clear it again */
        }
    }
#endif
}

void s4_fault_set(UW bits)
{
    UB before = s4_faults;

    s4_faults = (UB)(before | bits);
#if S4_PROPAGATE_FAULTS
    if ((before == 0U) && (s4_faults != 0U)) {
        s4_release_watch = 0U;
        s1_report_fault(FAULT_IMU_SENS);
    }
#endif
}

void s4_fault_clear(UW bits)
{
    UB before = s4_faults;

    s4_faults = (UB)(before & ~bits);
#if S4_PROPAGATE_FAULTS
    if ((before != 0U) && (s4_faults == 0U)) {
        s1_clear_fault(FAULT_IMU_SENS);
        s4_release_watch = (UB)S4_RELEASE_WATCH_TICKS;
    }
#endif
}

/* ---- Stopping the motors (IF-10) -----------------------------------------
 * Called by imu_task the moment the car is picked up or overturned.  Subsystem
 * 2 stops the wheels immediately and stays halted while the IMU fault bit is
 * set.  Its automatic "any fault means halt" override is commented out in
 * subsystem2.c, so reporting the bit alone would not stop the wheels: this
 * direct call is what does.  motion_trigger_safety_halt() takes Subsystem 2's
 * motor lock, which Subsystem 2 holds only for one short step (every 20 ms),
 * so the wait is short. */
void s4_stop_motors(void)
{
#if S4_STOP_MOTORS
    motion_trigger_safety_halt(FAULT_IMU_SENS);
#endif
}

/* ---- Telemetry through Subsystem 1 (IF-01) ------------------------------
 * Each Subsystem 1 setter updates its field under Subsystem 1's lock, so
 * nobody else's field is overwritten.
 *   pitch     8 bits signed:  -128..127 degrees  (clip, never wrap)
 *   car_state 3 bits:         our motion state, S4_MOTION 0..7
 *   hump      2 bits:         0 flat, 1 going up, 2 going down
 * s1_set_car_state() sets the last two together in ONE call, which is why we
 * send them together. */
void s4_report_status(INT pitch_deg, UW motion, UW hump)
{
    s1_set_pitch(s4_clip_s(pitch_deg, 8));
    s1_set_car_state(motion, hump);
}

/* Structured log line through Subsystem 1's CSV logger (IF-08). */
void s4_log_csv(const char *event, const char *details)
{
    s1_csv_log_event(event, details);
}

/* ---- Public API --------------------------------------------------------- */
INT       s4_get_max_hump_mm(void)  { return (INT)s4_max_hump_mm; }
INT       s4_get_hump_count(void)   { return (INT)s4_hump_count; }
S4_MOTION s4_get_motion(void)       { return (S4_MOTION)s4_motion; }
S4_LIFT   s4_get_lift(void)         { return (S4_LIFT)s4_lift; }
UW        s4_get_faults(void)       { return (UW)s4_faults; }

/* One 32-bit read gets pitch, roll and turn rate from the SAME moment. */
void s4_get_attitude(INT *pitch_deg, INT *roll_deg)
{
    UW snap = s4_snap;

    *pitch_deg = s4_snap_field(snap, 0);
    *roll_deg  = s4_snap_field(snap, 1);
}

INT s4_get_turn_rate(void)
{
    return s4_snap_field(s4_snap, 2) * S4_TURN_STEP_DPS;
}

BOOL s4_is_calibrated(void)
{
    UINT bits = 0U;

    return (tk_wai_flg(s4_flg, S4_FLG_CALIBRATED, TWF_ORW, &bits, TMO_POL) == E_OK)
           ? TRUE : FALSE;
}

/* imu_task owns the hump counters and the lift condition, so these two only
 * leave it a note (one bit of the event flag); it acts on the note at its next
 * sample.  The kernel sets the bit atomically.  A clear request is honoured
 * only while the car is resting level, and is used up either way. */
void s4_reset_run(void)         { (void)tk_set_flg(s4_flg, S4_FLG_RESET_REQ); }
void s4_clear_lifted(void)      { (void)tk_set_flg(s4_flg, S4_FLG_CLEAR_REQ); }

void s4_publish_summary(void)
{
    UB text[48];

    (void)tm_sprintf(text, (const UB*)"humps=%d,highest_mm=%d",
                     (INT)s4_hump_count, (INT)s4_max_hump_mm);
    s4_log_csv("RUN_SUMMARY", (const char*)text);
}

const char *s4_motion_name(S4_MOTION m)
{
    switch (m) {
    case S4_STATIONARY:         return "STATIONARY";
    case S4_MOVING:             return "MOVING";
    case S4_ACCELERATING:       return "ACCELERATING";
    case S4_TURNING:            return "TURNING";
    case S4_CLIMBING_HUMP:      return "CLIMBING_HUMP";
    case S4_DESCENDING_HUMP:    return "DESCENDING_HUMP";
    case S4_PICKED_UP:          return "PICKED_UP";
    case S4_OVERTURNED:         return "OVERTURNED";
    default:                    return "UNKNOWN";
    }
}

/* ---- Start-up ----------------------------------------------------------- */
ER subsystem4_init(void)
{
    T_CMBF  cmbf = {0};
    T_CFLG  cflg = {0};
    ER      err;

    /* The kernel starts I2C unit 0 at boot but not unit 1, and the IMU is on
     * unit 1.  Needs DEVCNF_USE_IIC = 1 in config/config_device.h. */
    err = dev_init_i2c(S4_I2C_UNIT);
    if (err < E_OK) {
        S4_LOG("dev_init_i2c failed: %d (is DEVCNF_USE_IIC set to 1?)\n", (INT)err);
        return err;
    }

    /* The message buffer: room for 16 messages of 8 bytes (the kernel adds a
     * small header to each one, hence the +8): 256 bytes in all. */
    cmbf.mbfatr = TA_TFIFO;
    cmbf.maxmsz = (INT)sizeof(S4_MSG);
    cmbf.bufsz  = (SZ)(16 * (sizeof(S4_MSG) + 8));
    s4_mbf = tk_cre_mbf(&cmbf);
    if (s4_mbf < E_OK) {
        return s4_mbf;
    }

    /* The event flag: three one-bit values that several tasks may set. */
    cflg.flgatr  = TA_TFIFO | TA_WMUL;
    cflg.iflgptn = 0U;
    s4_flg = tk_cre_flg(&cflg);
    return (s4_flg < E_OK) ? s4_flg : E_OK;
}

ER subsystem4_start(void)
{
    ID tid;
    
    tid = app_start_task(imu_task, S4_PRI_IMU, 0, "imu_task");
    if (tid < E_OK) {
        return tid;
    }
    tid = app_start_task(report_task, S4_PRI_REPORT, 0, "report_task");
    return (tid < E_OK) ? tid : E_OK;
}