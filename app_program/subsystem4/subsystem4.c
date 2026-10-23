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
volatile UB         s4_hump_count;
volatile UB         s4_max_hump_mm;
volatile UH         s4_samples;

/* Our two fault conditions, one bit each.  Only imu_task changes it, so the
 * "read, change a bit, write back" is safe. */
static volatile UB  s4_faults;

/* TEMPORARY, see subsystem4.h: 0 = no such object, so S3's example send fails quietly. */
ID                  mbf_summary = 0;
ID                  flg_events = 0;

/* ---- Faults through Subsystem 1 (IF-07) ----------------------------------
 * The team has one IMU bit, FAULT_IMU_SENS.  Subsystem 1 owns the fault
 * word, so we never touch it ourselves: we ask Subsystem 1 to set or clear
 * the bit (each call takes Subsystem 1's lock, so it is race-free).
 *
 * The bit is ON while at least one of our two conditions is active.  We
 * report on every change, and repeat the report from s4_fault_assert() while
 * a condition lasts, so the bit cannot be lost if somebody else clears it
 * while the problem is still there. */
void s4_fault_assert(void)
{
#if S4_PROPAGATE_FAULTS
    if (s4_faults != 0U) {
        s1_report_fault(FAULT_IMU_SENS);
    }
#endif
}

void s4_fault_set(UW bits)
{
    UB before = s4_faults;

    s4_faults = (UB)(before | bits);
#if S4_PROPAGATE_FAULTS
    if ((before == 0U) && (s4_faults != 0U)) {
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
    }
#endif
}

/* ---- Telemetry through Subsystem 1 (IF-01) ------------------------------
 * Each Subsystem 1 setter updates its field under Subsystem 1's lock, so
 * nobody else's field is overwritten.
 *   pitch     8 bits signed:  -128..127 degrees  (clip, never wrap)
 *   car_state 3 bits:         our motion state, S4_MOTION 0..6
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

/* imu_task owns the hump counters and the collision condition, so these two
 * only leave it a note (one bit of the event flag); it acts on the note at
 * its next sample.  The kernel sets the bit atomically. */
void s4_reset_run(void)         { (void)tk_set_flg(s4_flg, S4_FLG_RESET_REQ); }
void s4_clear_collision(void)   { (void)tk_set_flg(s4_flg, S4_FLG_CLEAR_REQ); }

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
    default:                    return "IMPACT";
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