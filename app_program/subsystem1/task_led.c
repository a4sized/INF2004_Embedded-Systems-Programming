/*
 *      task_led.c
 *      Subsystem 1: LED heartbeat, telemetry gatekeeper, centralized fault manager,
 *      and inbound command processing task.
 */

#include "subsystem1.h"
#include "../systemFaults.h"
#include <tstdlib.h>

/* Telemetry and command counters */
EXPORT UW               s1_tx_queued = 0;          /* Total telemetry packets queued for TX */
EXPORT UW               s1_rx_commands = 0;        /* Total valid commands received and executed */
EXPORT UW               s1_rx_rejected = 0;        /* Total malformed or rejected commands */
EXPORT S1_CMD_TYPE      s1_last_cmd = S1_CMD_NONE; /* Last successfully parsed command */

/* Minimum-bit telemetry state (protected by mtx_telemetry) */
LOCAL S1_TELEMETRY_PKT  s1_telemetry = {
        .seq         = 0,
        .speed_mm_s  = 0,
        .obstacle_cm = 100,
        .car_state   = 0,
        .hump_state  = 0,
        .pitch_deg   = 0,
        .line_mask   = 0,
        .barcode_id  = 0,
        .fault_mask  = 0
};

/* Gatekeeper APIs: Thread-safe setters for telemetry state under mtx_telemetry */
EXPORT void s1_set_speed(INT speed_mm_s) {
    tk_loc_mtx(mtx_telemetry, TMO_FEVR);
    s1_telemetry.speed_mm_s = speed_mm_s;
    tk_unl_mtx(mtx_telemetry);
}

EXPORT void s1_set_obstacle(INT obstacle_cm) {
    tk_loc_mtx(mtx_telemetry, TMO_FEVR);
    s1_telemetry.obstacle_cm = obstacle_cm;
    tk_unl_mtx(mtx_telemetry);
}

EXPORT void s1_set_pitch(INT pitch_deg) {
    tk_loc_mtx(mtx_telemetry, TMO_FEVR);
    s1_telemetry.pitch_deg = pitch_deg;
    tk_unl_mtx(mtx_telemetry);
}

EXPORT void s1_set_barcode_id(UB barcode_id) {
    tk_loc_mtx(mtx_telemetry, TMO_FEVR);
    s1_telemetry.barcode_id = barcode_id;
    tk_unl_mtx(mtx_telemetry);
}

EXPORT void s1_set_car_state(UW state, UW hump) {
    tk_loc_mtx(mtx_telemetry, TMO_FEVR);
    s1_telemetry.car_state = state;
    s1_telemetry.hump_state = hump;
    tk_unl_mtx(mtx_telemetry);
}

EXPORT void s1_set_line_mask(UW mask) {
    tk_loc_mtx(mtx_telemetry, TMO_FEVR);
    s1_telemetry.line_mask = mask;
    tk_unl_mtx(mtx_telemetry);
}

/* Sets one or more fault bits in the consolidated telemetry fault mask */
EXPORT void s1_report_fault(UW fault_bit) {
    tk_loc_mtx(mtx_telemetry, TMO_FEVR);
    s1_telemetry.fault_mask |= fault_bit;
    tk_unl_mtx(mtx_telemetry);
}

/* Clears specified fault bits in the telemetry fault mask */
EXPORT void s1_clear_fault(UW fault_bit) {
    tk_loc_mtx(mtx_telemetry, TMO_FEVR);
    s1_telemetry.fault_mask &= ~fault_bit;
    tk_unl_mtx(mtx_telemetry);
}

/* Returns an atomic snapshot of current telemetry data */
EXPORT void s1_get_telemetry(S1_TELEMETRY_PKT *out) {
    if (out == NULL) return;
    tk_loc_mtx(mtx_telemetry, TMO_FEVR);
    *out = s1_telemetry;
    tk_unl_mtx(mtx_telemetry);
}

/* Reports fault bits safely from ISR context via non-blocking event flag */
EXPORT void s1_report_fault_isr(UW fault_bit) {
    tk_set_flg(flg_s1_isr_fault, fault_bit);
}

/* Parses 1-byte binary command or ASCII string into S1_CMD_TYPE */
EXPORT S1_CMD_TYPE s1_parse_command(const UB *payload, INT len) {
    INT i;
    char txt[16];
    INT clen;
    if (payload == NULL || len < 1) return S1_CMD_NONE;

    /* Binary packet (1-byte cmd_id or ASCII digit '1'..'5') */
    if (len == 1) {
        S1_CMD_PKT *cmd = (S1_CMD_PKT*)payload;
        if (cmd->cmd_id >= S1_CMD_START && cmd->cmd_id <= S1_CMD_PING) {
            return (S1_CMD_TYPE)cmd->cmd_id;
        }
        if (payload[0] >= '1' && payload[0] <= '5') {
            return (S1_CMD_TYPE)(payload[0] - '0');
        }
    }

    /* Plain ASCII text command string (case-insensitive prefix match) */
    clen = (len < 15) ? len : 15;
    for (i = 0; i < clen; i++) {
        char c = (char)payload[i];
        if (c >= 'a' && c <= 'z') c -= 32;
        if (c == '\r' || c == '\n' || c == ' ' || c == '\t' || c == '\0') break;
        txt[i] = c;
    }
    txt[i] = '\0';

    if (knl_strcmp(txt, "START") == 0) return S1_CMD_START;
    if (knl_strcmp(txt, "STOP")  == 0) return S1_CMD_STOP;
    if (knl_strcmp(txt, "ESTOP") == 0) return S1_CMD_ESTOP;
    if (knl_strcmp(txt, "CLEAR") == 0) return S1_CMD_CLEAR;
    if (knl_strcmp(txt, "PING")  == 0) return S1_CMD_PING;

    return S1_CMD_NONE;
}

/* Logs event and telemetry state in CSV format over USB serial */
EXPORT void s1_csv_log_event(const char *event_type, const char *details) {
    S1_TELEMETRY_PKT snap;
    s1_get_telemetry(&snap);

    /* Output CSV log only on active faults or command events to avoid serial bus congestion */
    if (snap.fault_mask != 0 ||
        (event_type != NULL && (event_type[0] == 'C' && event_type[1] == 'M' && event_type[2] == 'D'))) {

        tm_printf((UB*)"LOG_CSV,%u,%d,%d,%d,%u,%s,%s\n",
                  (UW)snap.seq, (INT)snap.speed_mm_s, (INT)snap.obstacle_cm,
                  (INT)snap.pitch_deg, (UW)snap.fault_mask,
                  event_type != NULL ? event_type : "NONE",
                  details != NULL ? details : "NONE");
    }
}

/* Cyclic handler: Wakes up LED task every 250 ms */
EXPORT void led_cyclic_handler(void *exinf) {
    (void)exinf;
    tk_wup_tsk(tid_led);
}

/* Core task: Heartbeat LED toggle, ISR fault draining, command dispatch, and 1s telemetry queuing */
EXPORT void led_task(INT stacd, void *exinf) {
    UINT            level = 0;
    UW              ticks = 0;
    UB              rx_buf[S1_CMD_MAX];
    INT             len;
    UINT            isr_faults = 0;
    S1_TELEMETRY_PKT snap;

    (void)stacd; (void)exinf;
    gpio_set_pin(S1_LED_PIN, GPIO_MODE_OUT);

    while (1) {
        /* Wait for periodic wakeup from cyclic timer */
        tk_slp_tsk(TMO_FEVR);
        tk_can_wup(TSK_SELF);

        /* Toggle status LED */
        level = !level;
        gpio_set_val(S1_LED_PIN, level);

        /* Drain pending ISR faults via event flag into telemetry mask */
        if (tk_wai_flg(flg_s1_isr_fault, 0xFFFFFFFF, TWF_ORW | TWF_CLR, &isr_faults, TMO_POL) == E_OK) {
            tk_loc_mtx(mtx_telemetry, TMO_FEVR);
            s1_telemetry.fault_mask |= isr_faults;
            tk_unl_mtx(mtx_telemetry);
        }

        /* Drain and execute inbound MQTT commands from message buffer */
        while ((len = tk_rcv_mbf(mbf_mqtt_rx, rx_buf, TMO_POL)) > 0) {
            S1_CMD_TYPE cmd = s1_parse_command(rx_buf, len);
            if (cmd != S1_CMD_NONE) {
                s1_last_cmd = cmd;
                s1_rx_commands++;
                switch (cmd) {
                case S1_CMD_START: tk_set_flg(flg_s1_cmd, S1_FLG_CMD_START); break;
                case S1_CMD_STOP:  tk_set_flg(flg_s1_cmd, S1_FLG_CMD_STOP); break;
                case S1_CMD_ESTOP: tk_set_flg(flg_s1_cmd, S1_FLG_CMD_ESTOP); break;
                case S1_CMD_CLEAR: 
                    tk_set_flg(flg_s1_cmd, S1_FLG_CMD_CLEAR); 
                    s1_clear_fault(FAULT_CMD_REJECTED | FAULT_ESTOP);
                    break;
                case S1_CMD_PING:  tk_set_flg(flg_s1_cmd, S1_FLG_CMD_PING); break;
                default: break;
                }
                s1_csv_log_event("CMD_ACCEPTED", "BINARY");
            } else {
                s1_rx_rejected++;
                s1_csv_log_event("CMD_REJECTED", "INVALID_BINARY");
                s1_report_fault(FAULT_CMD_REJECTED);
            }
        }

        /* Queue 8-byte telemetry packet every S1_TELEM_TICKS (1 second) */
        if (++ticks % S1_TELEM_TICKS == 0) {
            tk_loc_mtx(mtx_telemetry, TMO_FEVR);
            s1_telemetry.seq++;
            snap = s1_telemetry;
            tk_unl_mtx(mtx_telemetry);

            if (tk_snd_mbf(mbf_mqtt_tx, (UB*)&snap, sizeof(S1_TELEMETRY_PKT), TMO_POL) == E_OK) {
                s1_tx_queued++;
            }
        }
    }
}
