/*
 *      task_led.c
 *      Subsystem 1 -- the LED task and its cyclic handler.
 *
 *      APIs shown: tk_slp_tsk, tk_wup_tsk, tk_can_wup, cyclic handler,
 *                  gpio_set_pin / gpio_set_val
 *
 *      Instead of tk_dly_tsk(), this task sleeps until a cyclic handler
 *      wakes it.  The handler is called by the kernel's timer at a fixed
 *      rate, so the blink rate stays exact however long the task's own work
 *      takes.
 */

#include "subsystem1.h"
#include "../systemFaults.h"

EXPORT UW               s1_tx_queued = 0;
EXPORT UW               s1_rx_commands = 0;
EXPORT UW               s1_rx_rejected = 0;
EXPORT S1_CMD_TYPE      s1_last_cmd = S1_CMD_NONE;

LOCAL S1_TELEMETRY      s1_telemetry = {
        .seq         = 0,
        .speed_mm_s  = 0,
        .obstacle_cm = 100,
        .pitch_deg   = 0,
        .fault_code  = 0,
        .barcode     = "NONE"
};

/* Helper: compare two NUL-terminated ASCII strings for equality */
LOCAL BOOL str_eq(const UB *a, const char *b)
{
        INT i = 0;
        if (a == NULL || b == NULL) return FALSE;
        while (a[i] != '\0' && b[i] != '\0') {
                if (a[i] != (UB)b[i]) return FALSE;
                i++;
        }
        return (a[i] == '\0' && b[i] == '\0') ? TRUE : FALSE;
}

/* Helper: safe string copy with NUL termination */
LOCAL void str_copy(UB *dst, const UB *src, INT max_len)
{
        INT i = 0;
        if (dst == NULL || max_len <= 0) return;
        if (src != NULL) {
                while (i < (max_len - 1) && src[i] != '\0') {
                        dst[i] = src[i];
                        i++;
                }
        }
        dst[i] = '\0';
}

/* Public API: Thread-safe telemetry update callable from any subsystem task. */
EXPORT void s1_update_telemetry(INT speed_mm_s, INT obstacle_cm,
                                INT pitch_deg, UW fault_code,
                                const UB *barcode)
{
        tk_loc_mtx(mtx_telemetry, TMO_FEVR);
        s1_telemetry.speed_mm_s  = speed_mm_s;
        s1_telemetry.obstacle_cm = obstacle_cm;
        s1_telemetry.pitch_deg   = pitch_deg;
        s1_telemetry.fault_code  = fault_code;
        if (barcode != NULL) {
                str_copy(s1_telemetry.barcode, barcode, sizeof(s1_telemetry.barcode));
        }
        tk_unl_mtx(mtx_telemetry);
}

/* Public API: Thread-safe telemetry snapshot reader. */
EXPORT void s1_get_telemetry(S1_TELEMETRY *out)
{
        if (out == NULL) return;
        tk_loc_mtx(mtx_telemetry, TMO_FEVR);
        *out = s1_telemetry;
        tk_unl_mtx(mtx_telemetry);
}

/* Public API: Format a telemetry struct into a compact JSON string. */
EXPORT INT s1_format_telemetry_json(const S1_TELEMETRY *t, UB *buf, INT max_len)
{
        if (t == NULL || buf == NULL || max_len < S1_MQTT_MSG_MAX) return -1;
        return tm_sprintf(buf,
                (const UB*)"{\"seq\":%u,\"spd\":%d,\"obs\":%d,\"pit\":%d,\"flt\":%u,\"bc\":\"%s\"}",
                t->seq, t->speed_mm_s, t->obstacle_cm,
                t->pitch_deg, t->fault_code, t->barcode);
}

/* Public API: Parse an inbound MQTT command string into an S1_CMD_TYPE enum. */
EXPORT S1_CMD_TYPE s1_parse_command(const UB *payload)
{
        if (payload == NULL) return S1_CMD_NONE;
        if (str_eq(payload, "START")) return S1_CMD_START;
        if (str_eq(payload, "STOP"))  return S1_CMD_STOP;
        if (str_eq(payload, "ESTOP")) return S1_CMD_ESTOP;
        if (str_eq(payload, "CLEAR")) return S1_CMD_CLEAR;
        if (str_eq(payload, "PING"))  return S1_CMD_PING;
        return S1_CMD_NONE;
}

/* Public API: Prints a CSV-formatted line to the serial console. */
EXPORT void s1_csv_log_event(const char *event_type, const char *details)
{
        S1_TELEMETRY snap;
        s1_get_telemetry(&snap);

        tm_printf((UB*)"LOG_CSV,%u,%d,%d,%d,%u,%s,%s\n",
                  snap.seq, 
                  snap.speed_mm_s, 
                  snap.obstacle_cm,
                  snap.pitch_deg, 
                  snap.fault_code,
                  event_type != NULL ? event_type : "NONE",
                  details != NULL ? details : "NONE");
}

/*
 * Cyclic handler.  Runs in interrupt context, NOT as a task, so it must
 * be short and must never wait (no tk_slp_tsk, tk_wai_sem, tm_printf...).
 * Signalling a task is the normal thing to do here.
 */
EXPORT void led_cyclic_handler(void *exinf)
{
        (void)exinf;
        tk_wup_tsk(tid_led);    /* wake the LED task (the request queues
                                   up if the task is not asleep yet) */
}

EXPORT void led_task(INT stacd, void *exinf)
{
        UINT            level = 0;
        UW              ticks = 0;
        UB              rx_buf[S1_MQTT_MSG_MAX + 1];
        UB              tx_buf[S1_MQTT_MSG_MAX];
        INT             len;
        S1_TELEMETRY    snap;

        (void)stacd; (void)exinf;

        gpio_set_pin(S1_LED_PIN, GPIO_MODE_OUT);        /* make the pin an output */

        while (1) {
                /* Sleep until led_cyclic_handler calls tk_wup_tsk() */
                tk_slp_tsk(TMO_FEVR);

                /* tk_can_wup: discard wake-ups that queued up while we were
                   not sleeping (e.g. while subsystem 4 had us suspended), so
                   the LED does not "catch up" with a burst of fast blinks. */
                tk_can_wup(TSK_SELF);

                level = !level;
                gpio_set_val(S1_LED_PIN, level);

                /* Drain any inbound MQTT commands queued by mqtt_utk.c */
                while ((len = tk_rcv_mbf(mbf_mqtt_rx, rx_buf, TMO_POL)) > 0) {
                        S1_CMD_TYPE cmd;
                        rx_buf[len] = '\0';
                        cmd = s1_parse_command(rx_buf);
                        if (cmd != S1_CMD_NONE) {
                                s1_last_cmd = cmd;
                                s1_rx_commands++;
                                switch (cmd) {
                                case S1_CMD_START: tk_set_flg(flg_s1_cmd, S1_FLG_CMD_START); break;
                                case S1_CMD_STOP:  tk_set_flg(flg_s1_cmd, S1_FLG_CMD_STOP); break;
                                case S1_CMD_ESTOP: tk_set_flg(flg_s1_cmd, S1_FLG_CMD_ESTOP); break;
                                case S1_CMD_CLEAR: 
                                        tk_set_flg(flg_s1_cmd, S1_FLG_CMD_CLEAR); 
                                        CLEAR_FAULT(active_fault_mask, FAULT_CMD_REJECTED); /* Clear the fault */
                                        break;
                                case S1_CMD_PING:  tk_set_flg(flg_s1_cmd, S1_FLG_CMD_PING); break;
                                default: break;
                                }
                                tm_printf((UB*)"s1_telem: CMD accepted \"%s\" (id=%d)\n", rx_buf, (INT)cmd);
                                s1_csv_log_event("CMD_ACCEPTED", (const char*)rx_buf);
                        } else {
                                s1_rx_rejected++;
                                tm_printf((UB*)"s1_telem: CMD rejected \"%s\"\n", rx_buf);
                                s1_csv_log_event("CMD_REJECTED_FAULT", (const char*)rx_buf);
                                
                                /* Set the global fault mask */
                                SET_FAULT(active_fault_mask, FAULT_CMD_REJECTED);
                        }
                }

                /* Every S1_TELEM_TICKS (1 s), format & queue JSON telemetry */
                if (++ticks % S1_TELEM_TICKS == 0) {
                        tk_loc_mtx(mtx_telemetry, TMO_FEVR);
                        s1_telemetry.seq++;
                        s1_telemetry.fault_code = active_fault_mask;
                        snap = s1_telemetry;
                        tk_unl_mtx(mtx_telemetry);

                        len = s1_format_telemetry_json(&snap, tx_buf, sizeof(tx_buf));
                        if (len > 0) {
                                if (tk_snd_mbf(mbf_mqtt_tx, tx_buf, len, TMO_POL) == E_OK) {
                                        s1_tx_queued++;
                                }
                        }
                }
        }
}
