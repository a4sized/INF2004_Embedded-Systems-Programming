/*
 *  subsystem4.h  --  Subsystem 4: IMU motion and terrain monitoring.
 *  Student ID: 2500787
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
 *                  Subsystem 1.
 *
 *  Who talks to whom (TEAM.md section 5):
 *    S4 -> S1  IF-01  s1_set_pitch(), s1_set_car_state():  tilt and state
 *    S4 -> S1  IF-07  s1_report_fault() / s1_clear_fault(): FAULT_IMU_SENS
 *    S4 -> S1  IF-08  s1_csv_log_event():  HUMP, COLLISION, IMU_FAULT, ...
 *    S2 -> S4  IF-06  get_left/right_encoder_ticks():  distance (when known)
 *
 *  ---- MINIMUM-BIT DATA (change requirement 1) ---------------------------
 *  Everything that crosses a task boundary is stored in the fewest bits its
 *  range and resolution need:
 *
 *    value             range / step                    bits   where
 *    ----------------  ------------------------------  -----  -------------
 *    message type      4 kinds                         2      S4_MSG
 *    hump state        flat / up / down                2      S4_MSG
 *    time              low 16 bits of the ms clock     16     S4_MSG
 *    vibration         0..255 mg, 1 mg                 8      S4_MSG
 *    pitch, roll       -128..127 deg, 1 deg            8, 8   S4_MSG, s4_snap
 *    turn rate         -256..254 deg/s, 2 deg/s        8      S4_MSG, s4_snap
 *    accel change      -1.28..1.27 g, 10 mg            8      S4_MSG
 *    hump peak         0..127 mm, 1 mm                 7      S4_MSG
 *    hump length       0..510 mm, 2 mm                 8      S4_MSG
 *    impact size       0..25.5 g, 0.1 g                8      S4_MSG
 *    local faults      comm lost, collision            2      s4_faults
 *
 *  Values that go OUT OF RANGE are clipped to the nearest limit, never
 *  wrapped (a 140 deg pitch must not turn into -116).
 *
 *  NOT packed, on purpose: the working values inside imu_task (the hump
 *  height, the filters, the level-ground baseline).  They add up tiny steps:
 *  a hump grows by about 0.04 cm per sample, and stored at the 1 mm step used
 *  for the OUTPUT every one of those steps would round to zero, so no hump
 *  would ever be seen.  Pack at the boundaries, keep the working state wide.
 *
 *  ---- RACE-FREE SHARED DATA ---------------------------------------------
 *  A bit-field update is "read the word, change some bits, write the word
 *  back".  Two writers on one word can lose a change.  So:
 *    - every shared word has exactly ONE writer task, and is published with
 *      ONE aligned store (s4_snap, s4_motion, s4_hump_count, ...), or
 *    - it is a bit of the kernel event flag s4_flg, which the kernel sets
 *      and clears atomically (several writers are fine), or
 *    - it travels through the message buffer, which the kernel protects.
 */

#ifndef SUBSYSTEM4_H
#define SUBSYSTEM4_H

#include "../app.h"
#include "s4_config.h"

/* ---- Task priorities (smaller number = more important) ------------------ */
#define S4_PRI_IMU      11
#define S4_PRI_REPORT   18

/* ---- Subsystem 4's own fault conditions ---------------------------------
 * The team has ONE IMU bit, FAULT_IMU_SENS (systemFaults.h).  It is reported
 * through Subsystem 1 while EITHER condition below is active, and cleared
 * when neither is.  The log tells the two apart. */
#define S4_FAULT_COMM       (1U << 0)   /* the IMU does not answer           */
#define S4_FAULT_COLLISION  (1U << 1)   /* a hit was detected                */

/* ---- Motion states (3 bits: Subsystem 1's car_state field) -------------- */
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
IMPORT INT       s4_get_max_hump_mm(void);      /* highest hump this run, mm  */
IMPORT INT       s4_get_hump_count(void);       /* humps measured (max 255)   */
IMPORT void      s4_get_attitude(INT *pitch_deg, INT *roll_deg);
IMPORT INT       s4_get_turn_rate(void);        /* deg/s, left is positive    */
IMPORT S4_MOTION s4_get_motion(void);
IMPORT UW        s4_get_faults(void);           /* S4_FAULT_* bits            */
IMPORT BOOL      s4_is_calibrated(void);
IMPORT void      s4_clear_collision(void);      /* release a collision early  */
IMPORT void      s4_reset_run(void);            /* call when a mission starts */
IMPORT void      s4_publish_summary(void);      /* call when it completes     */

/* =======================================================================
 *  PRIVATE to subsystem 4 (shared by its own .c files; ignore elsewhere)
 * ======================================================================= */

/* ---- The message between the two tasks: always exactly 8 bytes ---------- */
typedef enum {
    S4_MSG_STATUS = 0,      /* every 100 ms: the latest readings           */
    S4_MSG_HUMP,            /* a hump was just measured                    */
    S4_MSG_COLLISION,       /* an impact was just detected                 */
    S4_MSG_FAULT            /* the fault conditions changed                */
} S4_MSG_TYPE;

/* Step sizes: the stored count times the step = the real value. */
#define S4_TURN_STEP_DPS    2       /* turn rate: 2 deg/s per count        */
#define S4_ACCEL_STEP_MG    10      /* acceleration change: 10 mg per count*/
#define S4_PEAK_STEP_MM     1       /* hump peak: 1 mm per count           */
#define S4_LENGTH_STEP_MM   2       /* hump length: 2 mm per count         */
#define S4_IMPACT_STEP_DG   1       /* impact size: 0.1 g per count        */

typedef struct {
    /* word 0: the same for every message */
    unsigned int type       : 2;    /* S4_MSG_TYPE                         */
    unsigned int hump_state : 2;    /* STATUS: 0 flat, 1 going up, 2 down  */
    unsigned int time16     : 16;   /* low 16 bits of the ms clock         */
    unsigned int vibration  : 8;    /* STATUS: mg, clipped at 255          */
    unsigned int spare0     : 4;
    /* word 1: depends on the type */
    union {
        struct { signed int pitch : 8; signed int roll : 8;
                 signed int turn  : 8; signed int accel : 8; } status;
        struct { unsigned int peak : 7; unsigned int length : 8; } hump;
        struct { unsigned int impact : 8; } collision;
        struct { unsigned int faults : 2; } fault;
    } p;
} S4_MSG;

/* If the compiler lays this out differently, stop the build. */
typedef char s4_msg_must_be_8_bytes[(sizeof(S4_MSG) == 8) ? 1 : -1];

/* ---- Clipping and rounding (never wrap) --------------------------------- */
static inline INT s4_round(float v)
{
    return (INT)((v >= 0.0f) ? (v + 0.5f) : (v - 0.5f));
}

/* Clip v to what 'bits' bits can hold as a SIGNED number. */
static inline INT s4_clip_s(INT v, UINT bits)
{
    INT hi = (INT)((1U << (bits - 1U)) - 1U);
    INT lo = -hi - 1;

    return (v > hi) ? hi : ((v < lo) ? lo : v);
}

/* Clip v to what 'bits' bits can hold as an UNSIGNED number. */
static inline UINT s4_clip_u(INT v, UINT bits)
{
    INT hi = (INT)((1U << bits) - 1U);

    return (v < 0) ? 0U : ((v > hi) ? (UINT)hi : (UINT)v);
}

/* ---- s4_snap: pitch, roll and turn rate in ONE 32-bit word -------------- */
static inline UW s4_snap_pack(INT pitch, INT roll, INT turn_counts)
{
    return (UW)(UB)pitch | ((UW)(UB)roll << 8) | ((UW)(UB)turn_counts << 16);
}

/* field 0 = pitch, 1 = roll, 2 = turn (in counts); the sign is restored. */
static inline INT s4_snap_field(UW snap, UINT field)
{
    return (INT)(B)(UB)(snap >> (8U * field));
}

/* ---- Shared data.  Each line says who writes it. ------------------------ */
IMPORT ID    s4_mbf;                    /* the message buffer (kernel)       */
IMPORT ID    s4_flg;                    /* the event flag (kernel)           */

/* Bits of s4_flg.  Set and cleared by kernel calls, so any task may use them. */
#define S4_FLG_CALIBRATED   0x01U       /* imu_task set it                   */
#define S4_FLG_RESET_REQ    0x02U       /* anyone sets, imu_task consumes    */
#define S4_FLG_CLEAR_REQ    0x04U       /* anyone sets, imu_task consumes    */

IMPORT volatile UW  s4_snap;            /* writer: imu_task                  */
IMPORT volatile UB  s4_motion;          /* writer: report_task               */
IMPORT volatile UB  s4_hump_count;      /* writer: imu_task (saturates 255)  */
IMPORT volatile UB  s4_max_hump_mm;     /* writer: imu_task (saturates 255)  */
IMPORT volatile UH  s4_samples;         /* writer: imu_task (wraps: use differences) */

IMPORT void s4_fault_set(UW bits);      /* only imu_task changes faults      */
IMPORT void s4_fault_clear(UW bits);
IMPORT void s4_fault_assert(void);      /* repeat the report while faulted   */
IMPORT void s4_report_status(INT pitch_deg, UW motion, UW hump);   /* to S1  */
IMPORT void s4_log_csv(const char *event, const char *details);    /* to S1  */

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
