/*
 *  subsystem4.h  --  Subsystem 4: IMU motion and terrain monitoring.
 *  Student ID: <fill in>
 *
 *  What it does: reads the LSM303D accelerometer/magnetometer every 20 ms,
 *  works out tilt, turn rate, humps (and their peak height) and collisions,
 *  and tells the rest of the system.
 *
 *  Two tasks (TEAM.md section 4), joined by one message buffer:
 *
 *    imu_task      (priority 11, every 20 ms)  the PRODUCER.  Owns the I2C bus.
 *                  Reads the sensor, does all the maths, and puts a message
 *                  in the buffer: a status every 100 ms, plus one message for
 *                  every hump, collision or fault change.
 *    report_task   (priority 18)               the CONSUMER.  Waits for
 *                  messages, decides the motion state, and reports to
 *                  Subsystem 1 (pitch for the dashboard, events for the log).
 *
 *  This is the same producer/consumer pattern as subsystem 2 -> subsystem 3.
 *  The sensor task never prints or waits on anything slow, so it keeps time.
 *
 *  Who talks to whom (TEAM.md section 5):
 *    S4 -> S1  IF-01  s1_update_telemetry(): pitch, every 100 ms
 *    S4 -> S1  IF-09  s1_csv_log_event():    HUMP, COLLISION, IMU_FAULT, ...
 *    S4 -> all IF-08  active_fault_mask bit 28 (IMU lost), bit 30 (collision)
 *    S2 -> S4  IF-07  get_left/right_encoder_ticks(): distance (when known)
 */

#ifndef SUBSYSTEM4_H
#define SUBSYSTEM4_H

#include "../app.h"
#include "s4_config.h"

/* ---- Task priorities (smaller number = more important) ------------------ */
#define S4_PRI_IMU      11
#define S4_PRI_REPORT   18

/* ---- Fault bits: their positions in the team's active_fault_mask -------- */
#define S4_FAULT_COMM       (1U << 28)  /* the IMU does not answer           */
#define S4_FAULT_COLLISION  (1U << 30)  /* a hit was detected                */

/* ---- Motion states ------------------------------------------------------ */
typedef enum {
    S4_STATIONARY = 0,
    S4_MOVING,
    S4_ACCELERATING,
    S4_TURNING,
    S4_CLIMBING_HUMP,
    S4_DESCENDING_HUMP,
    S4_IMPACT
} S4_MOTION;

IMPORT const char *s4_motion_name(S4_MOTION m);

/* =======================================================================
 *  PUBLIC: what other code may call
 * ======================================================================= */
IMPORT float     s4_get_max_hump_cm(void);      /* highest hump this run      */
IMPORT UH        s4_get_hump_count(void);       /* humps measured this run    */
IMPORT void      s4_get_attitude(float *pitch_deg, float *roll_deg);
IMPORT float     s4_get_turn_rate(void);        /* degrees/s, left is positive*/
IMPORT S4_MOTION s4_get_motion(void);
IMPORT UW        s4_get_faults(void);           /* S4_FAULT_* bits            */
IMPORT void      s4_clear_collision(void);      /* release a collision early  */
IMPORT void      s4_reset_run(void);            /* call when a mission starts */
IMPORT void      s4_publish_summary(void);      /* call when it completes     */

/* =======================================================================
 *  PRIVATE to subsystem 4 (shared by its own .c files; ignore elsewhere)
 * ======================================================================= */

/* The message buffer between the two tasks. */
typedef enum {
    S4_MSG_STATUS = 0,      /* every 100 ms: the latest readings           */
    S4_MSG_HUMP,            /* a hump was just measured                    */
    S4_MSG_COLLISION,       /* an impact was just detected                 */
    S4_MSG_FAULT            /* the fault bits changed                      */
} S4_MSG_TYPE;

typedef struct {
    S4_MSG_TYPE type;
    UW      t_ms;
    /* S4_MSG_STATUS */
    float   pitch_deg;      /* nose up is positive                         */
    float   roll_deg;       /* left side up is positive                    */
    float   turn_dps;       /* turning left is positive                    */
    float   accel_hp_g;     /* forward acceleration change (g)             */
    float   vibration_g;    /* how much the car is shaking (g)             */
    INT     hump_state;     /* 0 = flat, 1 = going up, 2 = going down      */
    /* S4_MSG_HUMP */
    float   peak_cm;
    float   length_cm;
    /* S4_MSG_COLLISION */
    float   impact_g;
    /* S4_MSG_FAULT */
    UW      faults;
} S4_MSG;

IMPORT ID    s4_mbf;                    /* the message buffer                */

/* Values written by imu_task, read by anyone (one writer, so no lock is
 * needed: a 32-bit read is a single instruction). */
IMPORT volatile float   s4_pitch_deg;
IMPORT volatile float   s4_roll_deg;
IMPORT volatile float   s4_turn_dps;
IMPORT volatile float   s4_max_hump_cm;
IMPORT volatile UH      s4_hump_count;
IMPORT volatile UW      s4_samples;     /* sensor readings so far            */
IMPORT volatile BOOL    s4_calibrated;
IMPORT volatile S4_MOTION s4_motion;    /* written by report_task            */
IMPORT volatile BOOL    s4_reset_req;   /* set by s4_reset_run()             */
IMPORT volatile BOOL    s4_clear_req;   /* set by s4_clear_collision()       */

IMPORT void s4_fault_set(UW bits);      /* set bits here AND in the mask     */
IMPORT void s4_fault_clear(UW bits);
IMPORT void s4_push_pitch(float pitch_deg);                     /* to S1     */
IMPORT void s4_log_csv(const char *event, const char *details); /* to S1     */

IMPORT void imu_task(INT stacd, void *exinf);
IMPORT void report_task(INT stacd, void *exinf);

/* ---- TEMPORARY: keeps the template's EXAMPLE subsystem 3 compiling -------
 * subsystem3/task_consumer.c still sends its summaries to the old EXAMPLE
 * subsystem 4 (IF-03) and uses these four names.  They are only declared
 * here, with no kernel objects behind them: the send then fails and the
 * example simply drops its summaries (it only acts when the send succeeds).
 * DELETE this block, and the two IDs in subsystem4.c, as soon as subsystem 3
 * no longer includes this header for IF-03. */
#define S4_SUMMARY_MAX  64
#define FLG_BATCH       0x0001U
IMPORT ID mbf_summary;
IMPORT ID flg_events;

/* Console output through the kernel's tm_printf (no %f: it has no floats). */
#define S4_LOG(...)     tm_printf((UB*)"[S4] " __VA_ARGS__)

#endif /* SUBSYSTEM4_H */
