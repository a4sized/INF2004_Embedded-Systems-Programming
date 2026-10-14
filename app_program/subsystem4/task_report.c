/*
 *  task_report.c  --  report_task: the consumer (priority 18).
 *
 *  Sleeps until imu_task puts a message in the buffer, then:
 *    STATUS     work out the motion state, send the pitch to Subsystem 1,
 *               print a status line
 *    HUMP       log the hump (Subsystem 1's CSV logger) and print it
 *    COLLISION  log it and print it
 *    FAULT      log the new fault bits
 *
 *  All the slow things (printing, waiting for Subsystem 1's lock) happen here,
 *  so imu_task can keep its 20 ms rhythm.
 */

#include "subsystem4.h"

/* tm_printf can not print floats, so print "tenths" as whole numbers. */
static INT tenths(float v)
{
    return (INT)((v >= 0.0f) ? (v * 10.0f + 0.5f) : (v * 10.0f - 0.5f));
}

static INT abs_i(INT v)
{
    return (v < 0) ? -v : v;
}

static float absf(float x)
{
    return (x < 0.0f) ? -x : x;
}

/* The motion state, from the newest status message.  The first rule that
 * matches wins, so the order is the priority.
 *
 * "Standing still" has two limits: it takes less shaking to ENTER the
 * STATIONARY state than to LEAVE it (hysteresis), otherwise a car shaking
 * right at the limit would flicker between the two. */
static S4_MOTION classify(const S4_MSG *m, BOOL impact_recent, S4_MOTION previous)
{
    float still_limit = (previous == S4_STATIONARY) ? (S4_STILL_VIB_G * 1.5f)
                                                    : S4_STILL_VIB_G;

    if (impact_recent)                  return S4_IMPACT;
    if (m->hump_state == 1)             return S4_CLIMBING_HUMP;
    if (m->hump_state == 2)             return S4_DESCENDING_HUMP;
    if (absf(m->turn_dps) > S4_TURN_DPS) return S4_TURNING;
    if (absf(m->accel_hp_g) > S4_ACCEL_G) return S4_ACCELERATING;
    if (m->vibration_g < still_limit)   return S4_STATIONARY;
    return S4_MOVING;
}

void report_task(INT stacd, void *exinf)
{
    S4_MSG      m;
    S4_MOTION   shown = S4_STATIONARY;      /* the state we have announced    */
    S4_MOTION   candidate = S4_STATIONARY;  /* the state we are waiting to confirm */
    UINT        candidate_n = 0;
    UINT        lines = 0;                  /* status messages seen           */
    UW          impact_ms = 0;
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
            BOOL      impact = had_impact && ((m.t_ms - impact_ms) < S4_IMPACT_SHOW_MS);
            S4_MOTION now = classify(&m, impact, shown);

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
                s4_motion = shown;
                S4_LOG("motion: %s\n", s4_motion_name(shown));
            }

            s4_push_pitch(m.pitch_deg);     /* IF-01: pitch for the dashboard */

#if S4_PRINT_STATUS
            if ((++lines % 5U) == 0U) {     /* twice a second */
                INT p = tenths(m.pitch_deg), r = tenths(m.roll_deg), t = tenths(m.turn_dps);

                S4_LOG("pitch %s%d.%d roll %s%d.%d turn %s%d.%d dps  %s\n",
                       (p < 0) ? "-" : "", abs_i(p) / 10, abs_i(p) % 10,
                       (r < 0) ? "-" : "", abs_i(r) / 10, abs_i(r) % 10,
                       (t < 0) ? "-" : "", abs_i(t) / 10, abs_i(t) % 10,
                       s4_motion_name(shown));
            }
#else
            (void)lines;
#endif
            break;
        }

        case S4_MSG_HUMP:
            (void)tm_sprintf(text, (const UB*)"n=%d,peak_mm=%d,len_mm=%d,max_mm=%d",
                             (INT)s4_hump_count, (INT)(m.peak_cm * 10.0f),
                             (INT)(m.length_cm * 10.0f), (INT)(s4_max_hump_cm * 10.0f));
            s4_log_csv("HUMP", (const char*)text);
            S4_LOG("hump %d: peak %d.%d cm, length %d cm\n", (INT)s4_hump_count,
                   (INT)m.peak_cm, abs_i(tenths(m.peak_cm)) % 10, (INT)m.length_cm);
            break;

        case S4_MSG_COLLISION:
            impact_ms = m.t_ms;
            had_impact = TRUE;
            (void)tm_sprintf(text, (const UB*)"impact_x10g=%d", tenths(m.impact_g));
            s4_log_csv("COLLISION", (const char*)text);
            S4_LOG("COLLISION %d.%d g\n", tenths(m.impact_g) / 10,
                   abs_i(tenths(m.impact_g)) % 10);
            break;

        case S4_MSG_FAULT:
            (void)tm_sprintf(text, (const UB*)"faults=0x%x", (INT)m.faults);
            s4_log_csv("IMU_FAULT", (const char*)text);
            S4_LOG("faults now 0x%x\n", (INT)m.faults);
            break;

        default:
            break;
        }
    }
}
