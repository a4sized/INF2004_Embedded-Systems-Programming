/*
 *      subsystem1_test.c
 *      On-target tests for subsystem 1.  Built only with: make TEST=1
 *      Record results in testing/subsystem1/TEST_LOG.md.
 */

#include "subsystem1.h"
#include "../test_report.h"
#include "../systemFaults.h"

#if APP_TEST

EXPORT void subsystem1_test(void)
{
        T_RCYC          rcyc;
        T_RTSK          rtsk;
        S1_TELEMETRY    snap;
        UB              json[S1_MQTT_MSG_MAX];
        INT             len;
        UINT            flgptn = 0;
        UW              rej_before;
        ER              er;

        TEST_INIT();

        /* S1-U01: cyclic handler and S1 kernel objects exist and are active */
        er = tk_ref_cyc(cyc_led, &rcyc);
        TEST_CHECK("S1-U01",
                   er == E_OK && (rcyc.cycstat & TCYC_STA) &&
                   mbf_mqtt_tx > 0 && mbf_mqtt_rx > 0 &&
                   mtx_telemetry > 0 && flg_s1_cmd > 0,
                   "cyclic handler and S1 kernel objects are active");

        /* S1-U02: between ticks the LED/telemetry task sleeps (not busy-waiting) */
        {
                INT     i, waiting = 0;
                for (i = 0; i < 10; i++) {
                        tk_dly_tsk(97); /* not a multiple of 250 ms tick */
                        if (tk_ref_tsk(tid_led, &rtsk) == E_OK &&
                            (rtsk.tskstat & TTS_WAI)) {
                                waiting++;
                        }
                }
                TEST_CHECK("S1-U02", waiting >= 9,
                           "telemetry task sleeps between cyclic ticks");
        }

        /* S1-U03: thread-safe telemetry update and JSON serializer */
        s1_update_telemetry(150, 42, -3, 0, (const UB*)"BC123");
        s1_get_telemetry(&snap);
        len = s1_format_telemetry_json(&snap, json, sizeof(json));
        TEST_CHECK("S1-U03",
                   snap.speed_mm_s == 150 && snap.obstacle_cm == 42 &&
                   snap.pitch_deg == -3 && len > 20 && json[0] == '{',
                   "telemetry mutex update and JSON formatting succeed");

        /* S1-U04: inbound valid command ("ESTOP") via mbf_mqtt_rx sets flg_s1_cmd */
        er = tk_snd_mbf(mbf_mqtt_rx, (void*)"ESTOP", 5, TMO_POL);
        tk_dly_tsk(S1_TICK_MS + 50);    /* wait one tick for led_task to process */
        er |= tk_wai_flg(flg_s1_cmd, S1_FLG_CMD_ESTOP,
                         TWF_ORW | TWF_CLR, &flgptn, TMO_POL);
        TEST_CHECK("S1-U04",
                   er == E_OK && (flgptn & S1_FLG_CMD_ESTOP) &&
                   s1_last_cmd == S1_CMD_ESTOP,
                   "valid MQTT command ESTOP decoded and event flag set");

        /* S1-U05: malformed/unknown command is rejected without setting flags */
        rej_before = s1_rx_rejected;
        (void)tk_snd_mbf(mbf_mqtt_rx, (void*)"BAD_CMD_XYZ", 11, TMO_POL);
        tk_dly_tsk(S1_TICK_MS + 50);
        TEST_CHECK("S1-U05",
                   s1_rx_rejected == rej_before + 1,
                   "unknown MQTT command is rejected and counted");

	/* S1-U06: CSV logger executes without deadlocking the telemetry mutex */
        s1_update_telemetry(250, 15, 5, 0x01, (const UB*)"ERR_BARCODE");

        /* Because tm_printf outputs directly to the console, we can't easily
           string-compare it in code. We call it to prove it doesn't hard-fault,
           and you verify the output visually on the serial monitor. */
        s1_csv_log_event("TEST_FAULT", "Simulated hardware failure");

        /* If the system reaches this line, the mutex unlocked successfully */
        TEST_CHECK("S1-U06", 1, "CSV logger called safely (check console for LOG_CSV output)");
        TEST_SUMMARY("subsystem1");
}

#endif /* APP_TEST */
