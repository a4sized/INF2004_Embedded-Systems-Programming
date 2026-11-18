/** @file ir_uturn_calibration.c @brief USB one-shot pivot calibration. */
#if defined(BUDDY3_UTURN_CALIBRATION)
#include "../app.h"
#include "../subsystem1/subsystem1.h"
#include "../subsystem2/subsystem2.h"
#include "../systemFaults.h"
#include "ir_junction.h"
#include "ir_motion_consumer.h"

extern UW tm_usb_state (void);

static char * append_text (char * out, char const * value)
{
    while ('\0' != *value) *out++ = *value++;
    return out;
}

static char * append_number (char * out, int64_t value)
{
    char digits[20];
    uint32_t n = 0u;
    if (value < 0) { *out++ = '-'; value = -value; }
    do
    {
        digits[n++] = (char)('0' + (char)(value % 10));
        value /= 10;
    } while (value != 0);
    while (n != 0u) *out++ = digits[--n];
    return out;
}

static void log_sample (char const * phase, uint32_t elapsed,
                        int32_t left, int32_t right)
{
    char details[192];
    char * out = append_text(details, phase);
    out = append_text(out, ";ms=");
    out = append_number(out, elapsed);
    out = append_text(out, ";EL=");
    out = append_number(out, left);
    out = append_text(out, ";ER=");
    out = append_number(out, right);
    out = append_text(out, ";track_mm=");
    out = append_number(out, IR_UTURN_EFFECTIVE_TRACK_MM);
    out = append_text(out, ";wheel_target=");
    out = append_number(out, IR_UTURN_WHEEL_TICKS);
    out = append_text(out, ";pwm=");
    out = append_number(out, IR_START_TURN_SPEED);
    *out = '\0';
    s1_csv_log_event("UTURN_CAL", details);
}

static void calibration_task (INT code, void * extra)
{
    (void)code;
    (void)extra;
    s2_set_ir_velocity(0, 0);
    /* Opening USB arms the single trial. Losing USB during the countdown
     * cancels the countdown; after motion starts it stops without retrying.
     */
    for (;;)
    {
        while (3u != tm_usb_state()) (void)tk_dly_tsk(50u);
        s1_csv_log_event("UTURN_CAL", "USB_READY_WAIT_3S;target_deg=180");
        uint32_t steps = 0u;
        while ((steps < 60u) && (3u == tm_usb_state()))
        {
            (void)tk_dly_tsk(50u);
            ++steps;
        }
        if (3u == tm_usb_state()) break;
    }

    ir_junction_t turn = {0};
    /* Synthetic permission/IR only for this isolated motor trial: the
     * application starts S1/S2, not the follower, S4, S5 or controller.
     * The real production encoder pivot algorithm remains under test.
     */
    ir_line_snapshot_t sample = {0};
    sample.b_ready = true;
    sample.b_gpio_ready = true;
    sample.barcode_decode_sequence = 1u;
    sample.decoded_letter = 'F';
    SYSTIM now = {0};
    uint32_t start_ms = 0u;
    uint32_t elapsed = 0u;
    uint32_t last_log = 0u;
    char const * result = "CLOCK_ERROR";
    if (E_OK == tk_get_tim(&now))
    {
        start_ms = now.lo;
        log_sample("START", 0u, get_left_encoder_ticks(),
                    get_right_encoder_ticks());
        for (;;)
        {
            if (3u != tm_usb_state()) { result = "USB_LOST"; break; }
            if (E_OK != tk_get_tim(&now)) { result = "CLOCK_ERROR"; break; }
            elapsed = now.lo - start_ms;
            S1_TELEMETRY_PKT telemetry = {0};
            s1_get_telemetry(&telemetry);
            if (0u != (telemetry.fault_mask &
                       (TIER1_FATAL_MASK | TIER2_HAZARD_MASK)))
            {
                result = "FAULT_STOP";
                break;
            }
            int32_t const left = get_left_encoder_ticks();
            int32_t const right = get_right_encoder_ticks();
            ir_junction_note_odometry(&turn, left, right);
            sample.sampled_at_ms = now.lo;
            ir_motion_target_t target = {0};
            ir_junction_event_t event = IR_JUNCTION_NO_EVENT;
            (void)ir_junction_update(&turn, &sample, now.lo, 20u, true,
                                     IR_START_BASE_SPEED, IR_START_TURN_SPEED,
                                     &target, &event);
            s2_set_ir_velocity(target.left, target.right);
            if ((elapsed - last_log >= 200u) ||
                (IR_JUNCTION_TURN_STARTED == event))
            {
                log_sample((IR_JUNCTION_UTURN_SETTLING == turn.mode) ?
                            "SETTLING" : "PIVOT", elapsed, left, right);
                last_log = elapsed;
            }
            if (IR_JUNCTION_UTURN_DONE == event) { result = "DONE"; break; }
            if (IR_JUNCTION_TURN_TIMEOUT == event)
            {
                result = "ENCODER_TIMEOUT";
                s1_report_fault(FAULT_IR_NAV);
                break;
            }
            (void)tk_dly_tsk(20u);
        }
    }
    s2_set_ir_velocity(0, 0);
    log_sample(result, elapsed, get_left_encoder_ticks(),
                get_right_encoder_ticks());
    (void)tk_dly_tsk(500u);
    /* Repeated final totals expose any coast after the zero request and
     * remain visible if the monitor briefly disconnects. No automatic retry.
     */
    for (;;)
    {
        if (3u == tm_usb_state())
        {
            s1_csv_log_event("UTURN_CAL_RESULT", result);
            log_sample("STOPPED_RESET_TO_REPEAT", elapsed,
                        get_left_encoder_ticks(), get_right_encoder_ticks());
        }
        (void)tk_dly_tsk(1000u);
    }
}

ER ir_uturn_calibration_start (void)
{
    ID const task = app_start_task((FP)calibration_task, 16, TP_PRC1,
                                   "uturn_calibration");
    return (task < E_OK) ? task : E_OK;
}
#endif
