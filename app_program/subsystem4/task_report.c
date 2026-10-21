/*
 *  task_report.c  --  report_task: the consumer (priority 18).
 *
 *  Sleeps until imu_task puts a message in the buffer, then:
 *    STATUS     work out the motion state, tell Subsystem 1 (pitch, motion
 *               state, hump state), print a status line
 *    HUMP       log the hump (Subsystem 1's CSV logger) and print it
 *    COLLISION  log it and print it
 *    FAULT      log the new fault conditions
 *
 *  All the slow things (printing, waiting for Subsystem 1's lock) happen here,
 *  so imu_task can keep its 20 ms rhythm.
 *
 *  This task uses NO floating point: the message carries whole numbers, and
 *  every threshold below is a whole number in the same unit.  (The RP2040 has
 *  no floating-point hardware, so float maths costs real time.)
 */

#include "subsystem4.h"

static INT abs_i(INT v)
{
    return (v < 0) ? -v : v;
}

/* The motion state, from the newest status message.  The first rule that
 * matches wins, so the order is the priority.  All inputs are whole numbers:
 * degrees per second, milli-g.
 *
 * "Standing still" has two limits: it takes less shaking to ENTER the
 * STATIONARY state than to LEAVE it (hysteresis), otherwise a car shaking
 * right at the limit would flicker between the two. */
static S4_MOTION classify(const S4_MSG *m, BOOL impact_recent, S4_MOTION previous)
{
    INT turn_dps  = (INT)m->p.status.turn  * S4_TURN_STEP_DPS;
    INT accel_mg  = (INT)m->p.status.accel * S4_ACCEL_STEP_MG;
    INT vib_mg    = (INT)m->vibration;
    INT still_limit = (previous == S4_STATIONARY) ? ((S4_STILL_VIB_MG * 3) / 2)
                                                  : S4_STILL_VIB_MG;

    if (impact_recent)                  return S4_IMPACT;
    if (m->hump_state == 1U)            return S4_CLIMBING_HUMP;
    if (m->hump_state == 2U)            return S4_DESCENDING_HUMP;
    if (abs_i(turn_dps) > S4_TURN_DPS)  return S4_TURNING;
    if (abs_i(accel_mg) > S4_ACCEL_MG)  return S4_ACCELERATING;
    if (vib_mg < still_limit)           return S4_STATIONARY;
    return S4_MOVING;
}

void report_task(INT stacd, void *exinf)
{
    S4_MSG      m;
    S4_MOTION   shown = S4_STATIONARY;      /* the state we have announced    */
    S4_MOTION   candidate = S4_STATIONARY;  /* the state we are waiting to confirm */
    UINT        candidate_n = 0;
    UINT        lines = 0;                  /* status messages seen           */
    UINT        impact_time16 = 0;          /* 16-bit ms clock of the last impact */
    BOOL        had_impact = FALSE;

    (void)stacd;
    (void)exinf;

    for (;;) {
        UB text[64];

        /* Wait here until imu_task sends something. */
        if (tk_rcv_mbf(s4_mbf, &m, TMO_FEVR) != (INT)sizeof(m)) {
            continue;
        }

        switch (m.type) {

        case S4_MSG_STATUS: {
            S4_MOTION now;

            /* The timestamps are only 16 bits (about 65 s), so the age of the
             * last impact is a 16-bit subtraction.  Once the "show IMPACT"
             * window is over, forget the impact, so that the clock wrapping
             * round can never make an old impact look recent. */
            if (had_impact && ((UH)(m.time16 - impact_time16) >= S4_IMPACT_SHOW_MS)) {
                had_impact = FALSE;
            }
            now = classify(&m, had_impact, shown);

            /* Only announce a new state once it has been seen twice in a row
             * (an impact is announced at once). */
            if (now == candidate) {
                candidate_n++;
            } else {
                candidate = now;
                candidate_n = 1;
            }
            if ((candidate != shown) &&
                ((candidate_n >= S4_CLASS_PERSIST) || (candidate == S4_IMPACT))) {
                shown = candidate;
                s4_motion = (UB)shown;
                S4_LOG("motion: %s\n", s4_motion_name(shown));
            }

            /* IF-01: pitch, motion state and hump state, through Subsystem 1's
             * setters.  Each field is updated under Subsystem 1's lock. */
            s4_report_status((INT)m.p.status.pitch, (UW)shown, (UW)m.hump_state);

#if S4_PRINT_STATUS
            if ((++lines % 5U) == 0U) {     /* twice a second */
                S4_LOG("pitch %d roll %d turn %d dps  %s\n",
                       (INT)m.p.status.pitch, (INT)m.p.status.roll,
                       (INT)m.p.status.turn * S4_TURN_STEP_DPS, s4_motion_name(shown));
            }
#else
            (void)lines;
#endif
            break;
        }

        case S4_MSG_HUMP:
            (void)tm_sprintf(text, (const UB*)"n=%d,peak_mm=%d,len_mm=%d,max_mm=%d",
                             (INT)s4_hump_count,
                             (INT)m.p.hump.peak * S4_PEAK_STEP_MM,
                             (INT)m.p.hump.length * S4_LENGTH_STEP_MM,
                             (INT)s4_max_hump_mm);
            s4_log_csv("HUMP", (const char*)text);
            S4_LOG("hump %d: peak %d mm, length %d mm\n", (INT)s4_hump_count,
                   (INT)m.p.hump.peak * S4_PEAK_STEP_MM,
                   (INT)m.p.hump.length * S4_LENGTH_STEP_MM);
            break;

        case S4_MSG_COLLISION:
            impact_time16 = m.time16;
            had_impact = TRUE;
            (void)tm_sprintf(text, (const UB*)"impact_x10g=%d",
                             (INT)m.p.collision.impact * S4_IMPACT_STEP_DG);
            s4_log_csv("COLLISION", (const char*)text);
            S4_LOG("COLLISION %d.%d g\n",
                   ((INT)m.p.collision.impact * S4_IMPACT_STEP_DG) / 10,
                   ((INT)m.p.collision.impact * S4_IMPACT_STEP_DG) % 10);
            break;

        case S4_MSG_FAULT:
            (void)tm_sprintf(text, (const UB*)"imu_sens=%d,comm=%d,collision=%d",
                             (m.p.fault.faults != 0U) ? 1 : 0,
                             (INT)(m.p.fault.faults & S4_FAULT_COMM),
                             (INT)((m.p.fault.faults & S4_FAULT_COLLISION) >> 1));
            s4_log_csv("IMU_FAULT", (const char*)text);
            S4_LOG("faults now 0x%x\n", (INT)m.p.fault.faults);
            break;

        default:
            break;
        }
    }
}
