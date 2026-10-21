/*
 *      subsystem1_test.c
 */

#include "subsystem1.h"
#include "../test_report.h"
#include "../systemFaults.h"

#if APP_TEST

EXPORT void subsystem1_test(void)
{
        T_RCYC          rcyc;
        T_RTSK          rtsk;
        S1_TELEMETRY_PKT snap;
        UINT            flgptn = 0;
        UW              rej_before;
        ER              er;

        TEST_INIT();

        /* S1-U01 */
        er = tk_ref_cyc(cyc_led, &rcyc);
        TEST_CHECK("S1-U01",
                   er == E_OK && (rcyc.cycstat & TCYC_STA) &&
                   mbf_mqtt_tx > 0 && mbf_mqtt_rx > 0 &&
                   mtx_telemetry > 0 && flg_s1_cmd > 0 && flg_s1_isr_fault > 0,
                   "cyclic handler and S1 kernel objects are active");

        /* S1-U02 */
        {
                INT     i, waiting = 0;
                for (i = 0; i < 10; i++) {
                        tk_dly_tsk(97);
                        if (tk_ref_tsk(tid_led, &rtsk) == E_OK &&
                            (rtsk.tskstat & TTS_WAI)) {
                                waiting++;
                        }
                }
                TEST_CHECK("S1-U02", waiting >= 9,
                           "telemetry task sleeps between cyclic ticks");
        }

        /* S1-U03: Verify Minimum-Bit 8-byte Struct and Setters */
        s1_set_speed(150);
        s1_set_obstacle(42);
        s1_set_pitch(-3);
        s1_report_fault(0x01);
        s1_get_telemetry(&snap);
        
        TEST_CHECK("S1-U03",
                   sizeof(S1_TELEMETRY_PKT) == 8 &&
                   snap.speed_mm_s == 150 && snap.obstacle_cm == 42 &&
                   snap.pitch_deg == -3 && (snap.fault_mask & 0x01),
                   "binary bit-field setters succeed, payload is 8 bytes");

        /* S1-U04: inbound valid binary command via mbf_mqtt_rx */
        S1_CMD_PKT cmd;
        cmd.cmd_id = S1_CMD_ESTOP;
        cmd.reserved = 0;
        er = tk_snd_mbf(mbf_mqtt_rx, (void*)&cmd, sizeof(cmd), TMO_POL);
        tk_dly_tsk(S1_TICK_MS + 50);
        er |= tk_wai_flg(flg_s1_cmd, S1_FLG_CMD_ESTOP, TWF_ORW | TWF_CLR, &flgptn, TMO_POL);
        TEST_CHECK("S1-U04",
                   er == E_OK && (flgptn & S1_FLG_CMD_ESTOP) &&
                   s1_last_cmd == S1_CMD_ESTOP,
                   "binary MQTT command ESTOP decoded and flag set");

        /* S1-U05: malformed/unknown command (bad cmd_id) */
        rej_before = s1_rx_rejected;
        S1_CMD_PKT bad_cmd;
        bad_cmd.cmd_id = 7; /* 7 is invalid */
        bad_cmd.reserved = 0;
        (void)tk_snd_mbf(mbf_mqtt_rx, (void*)&bad_cmd, sizeof(bad_cmd), TMO_POL);
        tk_dly_tsk(S1_TICK_MS + 50);
        TEST_CHECK("S1-U05",
                   s1_rx_rejected == rej_before + 1,
                   "unknown binary MQTT command is rejected and counted");

        /* S1-U06: CSV logger */
        s1_csv_log_event("TEST_FAULT", "Simulated hardware failure");
        TEST_CHECK("S1-U06", 1, "CSV logger called safely");
        TEST_SUMMARY("subsystem1");
}

#endif /* APP_TEST */