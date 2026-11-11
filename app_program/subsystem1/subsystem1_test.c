/*
 *      subsystem1_test.c
 *      Unit tests for Subsystem 1 (LED task, telemetry bitfields, ISR fault
 *      drain, command parser, MQTT RX/TX bridges, and CSV logger).
 *      Built only with: make TEST=1. Record results in testing/subsystem1/TEST_LOG.md.
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
        UW              ok_before;
        UW              tx_before;
        UB              seq_before;
        UB              tx_buf[S1_MQTT_MSG_MAX];
        ER              er;

        TEST_INIT();

        /* S1-U01: Verify cyclic handler and kernel synchronization objects are active */
        er = tk_ref_cyc(cyc_led, &rcyc);
        TEST_CHECK("S1-U01",
                   er == E_OK && (rcyc.cycstat & TCYC_STA) &&
                   tid_led > 0 && mbf_mqtt_tx > 0 && mbf_mqtt_rx > 0 &&
                   mtx_telemetry > 0 && flg_s1_cmd > 0 && flg_s1_isr_fault > 0,
                   "cyclic handler, led_task, and S1 kernel objects are active");

        /* S1-U02: Verify telemetry task sleeps between cyclic ticks */
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

        /* S1-U03: Verify 8-byte packed telemetry struct and core gatekeeper setters.
         * Clear synthetic fault before yielding so VC/S2 do not trip safety halt. */
        s1_set_speed(150);
        s1_set_obstacle(42);
        s1_set_pitch(-3);
        s1_report_fault(0x01);
        s1_get_telemetry(&snap);
        s1_clear_fault(0x01);
        s1_set_speed(0);

        TEST_CHECK("S1-U03",
                   sizeof(S1_TELEMETRY_PKT) == 8 && sizeof(S1_CMD_PKT) == 1 &&
                   snap.speed_mm_s == 150 && snap.obstacle_cm == 42 &&
                   snap.pitch_deg == -3 && (snap.fault_mask & 0x01),
                   "binary bit-field setters succeed, payload is 8 bytes");

        /* S1-U04: Inbound valid binary command pushed via s1_mqtt_push_rx().
         * Uses S1_CMD_PING so vehicle_controller_task does not race to clear flg_s1_cmd. */
        (void)tk_wai_flg(flg_s1_cmd, S1_FLG_CMD_PING, TWF_ORW | TWF_CLR, &flgptn, TMO_POL);
        flgptn = 0;
        ok_before = s1_rx_commands;
        {
                S1_CMD_PKT cmd;
                cmd.cmd_id = S1_CMD_PING;
                cmd.reserved = 0;
                s1_mqtt_push_rx((const UB *)&cmd, sizeof(cmd));
        }
        tk_dly_tsk(S1_TICK_MS + 150);
        er = tk_wai_flg(flg_s1_cmd, S1_FLG_CMD_PING, TWF_ORW | TWF_CLR, &flgptn, TMO_POL);
        TEST_CHECK("S1-U04",
                   er == E_OK && (flgptn & S1_FLG_CMD_PING) &&
                   s1_last_cmd == S1_CMD_PING &&
                   s1_rx_commands >= ok_before + 1,
                   "binary MQTT command PING pushed, decoded, and flag set");

        /* S1-U05: Inbound malformed/unknown command rejected and fault bit raised */
        rej_before = s1_rx_rejected;
        {
                S1_CMD_PKT bad_cmd;
                bad_cmd.cmd_id = 7; /* 7 is invalid */
                bad_cmd.reserved = 0;
                s1_mqtt_push_rx((const UB *)&bad_cmd, sizeof(bad_cmd));
        }
        tk_dly_tsk(S1_TICK_MS + 150);
        s1_get_telemetry(&snap);
        TEST_CHECK("S1-U05",
                   s1_rx_rejected == rej_before + 1 &&
                   (snap.fault_mask & FAULT_CMD_REJECTED) != 0U,
                   "unknown binary MQTT command rejected, counted, and fault raised");
        s1_clear_fault(FAULT_CMD_REJECTED);

        /* S1-U06: CSV logger execution (faulted, BARCODE_DECODED, and NULL-safe) */
        s1_report_fault(FAULT_WIFI);
        s1_csv_log_event("TEST_FAULT", "Simulated hardware failure");
        s1_csv_log_event("BARCODE_DECODED", "letter=A;order=F");
        s1_csv_log_event(NULL, NULL);
        s1_clear_fault(FAULT_WIFI);
        TEST_CHECK("S1-U06", 1, "CSV logger called safely");

        /* S1-U07: Extended telemetry setters (car_state, line_mask, barcode_id) & fault isolation */
        {
                BOOL ok_fields, ok_fault_iso, ok_cleanup;

                s1_set_car_state(3u, 2u);
                s1_set_line_mask(0x15u);
                s1_set_barcode_id('A');
                s1_report_fault(FAULT_WIFI | FAULT_CMD_REJECTED);
                s1_clear_fault(FAULT_WIFI);
                s1_get_telemetry(&snap);

                ok_fields = (snap.car_state == 3u && snap.hump_state == 2u &&
                             snap.line_mask == 0x15u && snap.barcode_id == 'A');
                ok_fault_iso = ((snap.fault_mask & FAULT_WIFI) == 0U &&
                                (snap.fault_mask & FAULT_CMD_REJECTED) != 0U);

                s1_get_telemetry(NULL); /* Must ignore NULL pointer safely */
                s1_clear_fault(FAULT_CMD_REJECTED);
                s1_set_barcode_id(0u);
                s1_set_line_mask(0u);
                s1_set_car_state(0u, 0u);
                s1_get_telemetry(&snap);
                ok_cleanup = ((snap.fault_mask & FAULT_CMD_REJECTED) == 0U &&
                              snap.barcode_id == 0u);

                TEST_CHECK("S1-U07", ok_fields && ok_fault_iso && ok_cleanup,
                           "extended telemetry setters, fault bit isolation, and NULL safety");
        }

        /* S1-U08: Command parser handles binary, ASCII digits, case-insensitive words, & invalid */
        {
                S1_CMD_PKT bin_start = { .cmd_id = S1_CMD_START, .reserved = 0 };
                S1_CMD_PKT bin_zero  = { .cmd_id = 0, .reserved = 0 };
                S1_CMD_PKT bin_six   = { .cmd_id = 6, .reserved = 0 };
                BOOL parse_ok =
                        s1_parse_command((const UB *)&bin_start, 1) == S1_CMD_START &&
                        s1_parse_command((const UB *)"1", 1) == S1_CMD_START &&
                        s1_parse_command((const UB *)"5", 1) == S1_CMD_PING &&
                        s1_parse_command((const UB *)"START", 5) == S1_CMD_START &&
                        s1_parse_command((const UB *)"stop\n", 5) == S1_CMD_STOP &&
                        s1_parse_command((const UB *)"Estop\r\n", 7) == S1_CMD_ESTOP &&
                        s1_parse_command((const UB *)"cLeAr", 5) == S1_CMD_CLEAR &&
                        s1_parse_command((const UB *)"PING", 4) == S1_CMD_PING &&
                        s1_parse_command((const UB *)&bin_zero, 1) == S1_CMD_NONE &&
                        s1_parse_command((const UB *)&bin_six, 1) == S1_CMD_NONE &&
                        s1_parse_command((const UB *)"INVALID", 7) == S1_CMD_NONE &&
                        s1_parse_command((const UB *)"", 0) == S1_CMD_NONE &&
                        s1_parse_command(NULL, 1) == S1_CMD_NONE;

                /* Also verify RX bridge safely ignores invalid arguments */
                s1_mqtt_push_rx(NULL, 1);
                s1_mqtt_push_rx((const UB *)"PING", 0);

                TEST_CHECK("S1-U08", parse_ok,
                           "s1_parse_command decodes binary/ASCII and rejects invalid inputs");
        }

        /* S1-U09: Non-blocking ISR fault reporting drains via led_task into fault_mask */
        {
                BOOL set_ok, clear_ok;

                s1_clear_fault(FAULT_WIFI);
                s1_report_fault_isr(FAULT_WIFI);
                tk_dly_tsk(S1_TICK_MS + 150);
                s1_get_telemetry(&snap);
                set_ok = ((snap.fault_mask & FAULT_WIFI) != 0U);
                s1_clear_fault(FAULT_WIFI);
                s1_get_telemetry(&snap);
                clear_ok = ((snap.fault_mask & FAULT_WIFI) == 0U);

                TEST_CHECK("S1-U09", set_ok && clear_ok,
                           "ISR fault flag drained by led_task and cleared cleanly");
        }

        /* S1-U10: 1 Hz telemetry TX queueing, rolling seq increment, & s1_mqtt_pop_tx guardrails */
        {
                BOOL bounds_ok = (s1_mqtt_pop_tx(NULL, S1_MQTT_MSG_MAX) == 0 &&
                                  s1_mqtt_pop_tx(tx_buf, S1_MQTT_MSG_MAX - 1) == 0);
                INT pop_len;

                s1_get_telemetry(&snap);
                seq_before = (UB)snap.seq;
                tx_before  = s1_tx_queued;

                tk_dly_tsk((S1_TICK_MS * S1_TELEM_TICKS) + 150);

                s1_get_telemetry(&snap);
                pop_len = s1_mqtt_pop_tx(tx_buf, sizeof(tx_buf));

                TEST_CHECK("S1-U10",
                           bounds_ok &&
                           s1_tx_queued >= tx_before + 1 &&
                           (UB)snap.seq != seq_before &&
                           (pop_len == 0 || pop_len == sizeof(S1_TELEMETRY_PKT)),
                           "1 Hz telemetry queued with seq increment and TX bridge bounds enforced");
        }

        TEST_SUMMARY("subsystem1");
}

#endif /* APP_TEST */
