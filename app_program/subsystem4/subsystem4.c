/*
 *  subsystem4.c  --  start-up, shared data, and the links to Subsystems 1/2.
 *
 *  subsystem4_init()   creates the message buffer and starts I2C unit 1
 *  subsystem4_start()  starts the two tasks
 */

#include "subsystem4.h"
#include "../systemFaults.h"            /* active_fault_mask, SET_FAULT      */
#include "../subsystem1/subsystem1.h"   /* s1_update_telemetry, s1_csv_log_event */
#include "../../device/include/dev_i2c.h"   /* dev_init_i2c                  */

/* ---- Shared data (declared in subsystem4.h) ---------------------------- */
ID                  s4_mbf;
volatile float      s4_pitch_deg, s4_roll_deg, s4_turn_dps, s4_max_hump_cm;
volatile UH         s4_hump_count;
volatile UW         s4_samples;
volatile BOOL       s4_calibrated;
volatile S4_MOTION  s4_motion;
volatile BOOL       s4_reset_req, s4_clear_req;

static volatile UW  s4_faults;          /* our own fault bits                */

/* TEMPORARY, see subsystem4.h: 0 = no such object, so S3's example send fails quietly. */
ID                  mbf_summary = 0;
ID                  flg_events = 0;

/* ---- Faults ------------------------------------------------------------
 * Our bits are kept here and copied into the team's active_fault_mask.
 * Subsystem 2 halts the motors when ANY bit in that mask is set.
 * Only imu_task changes faults while the car runs, so no lock is used.
 * (SET_FAULT is a plain read-modify-write, like everyone else's: if two
 * subsystems on different cores change the mask at the same instant, one
 * change can be lost.  Making SET_FAULT atomic in systemFaults.h would fix
 * that for the whole team.) */
void s4_fault_set(UW bits)
{
    s4_faults |= bits;
#if S4_PROPAGATE_FAULTS
    s1_report_fault(bits);
#endif
}

void s4_fault_clear(UW bits)
{
    s4_faults &= ~bits;
#if S4_PROPAGATE_FAULTS
    s1_clear_fault(bits);
#endif
}

/* ---- Subsystem 1 (IF-01, IF-09) ----------------------------------------
 * s1_update_telemetry() takes EVERY field at once.  So read the current
 * values first and give them all back unchanged except the pitch;
 * otherwise Subsystem 2's speed and Subsystem 5's obstacle distance would
 * be overwritten with old numbers. */
void s4_push_pitch(float pitch_deg)
{
    S1_TELEMETRY_PKT    t;
    INT             pitch;

    pitch = (INT)((pitch_deg >= 0.0f) ? (pitch_deg + 0.5f) : (pitch_deg - 0.5f));
    s1_set_pitch(pitch);
}

void s4_log_csv(const char *event, const char *details)
{
    s1_csv_log_event(event, details);
}

/* ---- Public API --------------------------------------------------------- */
float s4_get_max_hump_cm(void)  { return s4_max_hump_cm; }
UH    s4_get_hump_count(void)   { return s4_hump_count; }
float s4_get_turn_rate(void)    { return s4_turn_dps; }
S4_MOTION s4_get_motion(void)   { return s4_motion; }
UW    s4_get_faults(void)       { return s4_faults; }

void s4_get_attitude(float *pitch_deg, float *roll_deg)
{
    *pitch_deg = s4_pitch_deg;
    *roll_deg  = s4_roll_deg;
}

/* imu_task owns the hump counters and the collision fault, so these two
 * only leave it a note; it acts on the note at its next sample. */
void s4_reset_run(void)         { s4_reset_req = TRUE; }
void s4_clear_collision(void)   { s4_clear_req = TRUE; }

void s4_publish_summary(void)
{
    UB text[48];

    (void)tm_sprintf(text, (const UB*)"humps=%d,highest_mm=%d",
                     (INT)s4_hump_count, (INT)(s4_max_hump_cm * 10.0f));
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
    ER      err;

    /* The kernel starts I2C unit 0 at boot but not unit 1, and the IMU is on
     * unit 1.  Needs DEVCNF_USE_IIC = 1 in config/config_device.h. */
    err = dev_init_i2c(S4_I2C_UNIT);
    if (err < E_OK) {
        S4_LOG("dev_init_i2c failed: %d (is DEVCNF_USE_IIC set to 1?)\n", (INT)err);
        return err;
    }

    /* The message buffer: room for 16 messages (the kernel adds a small
     * header to each one, hence the +8). */
    cmbf.mbfatr = TA_TFIFO;
    cmbf.maxmsz = (INT)sizeof(S4_MSG);
    cmbf.bufsz  = (SZ)(16 * (sizeof(S4_MSG) + 8));
    s4_mbf = tk_cre_mbf(&cmbf);
    return (s4_mbf < E_OK) ? s4_mbf : E_OK;
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
