/*
 *      subsystem1.c
 */

#include "subsystem1.h"

EXPORT ID       tid_led;
EXPORT ID       cyc_led;
EXPORT ID       mbf_mqtt_tx;
EXPORT ID       mbf_mqtt_rx;
EXPORT ID       mtx_telemetry;
EXPORT ID       flg_s1_cmd;
EXPORT ID       flg_s1_isr_fault; /* Exposing the new ISR flag */

EXPORT ER subsystem1_init(void)
{
        T_CCYC  ccyc = {0};
        T_CMBF  cmbf = {0};
        T_CMTX  cmtx = {0};
        T_CFLG  cflg = {0};

        /* 1. Mutex with priority inheritance */
        cmtx.mtxatr = TA_TFIFO | TA_INHERIT;
        mtx_telemetry = tk_cre_mtx(&cmtx);
        if (mtx_telemetry < E_OK) return mtx_telemetry;

        /* 2. Outbound MQTT TX (8-byte packets) */
        cmbf.mbfatr = TA_TFIFO;
        cmbf.bufsz  = 4 * S1_MQTT_MSG_MAX; 
        cmbf.maxmsz = S1_MQTT_MSG_MAX;
        mbf_mqtt_tx = tk_cre_mbf(&cmbf);
        if (mbf_mqtt_tx < E_OK) return mbf_mqtt_tx;

        /* 3. Inbound MQTT RX (1-byte commands) */
        cmbf.mbfatr = TA_TFIFO;
        cmbf.bufsz  = 4 * S1_CMD_MAX; 
        cmbf.maxmsz = S1_CMD_MAX;
        mbf_mqtt_rx = tk_cre_mbf(&cmbf);
        if (mbf_mqtt_rx < E_OK) return mbf_mqtt_rx;

        /* 4. Event flag for decoded remote MQTT commands */
        cflg.flgatr  = TA_TFIFO | TA_WMUL;
        cflg.iflgptn = 0;
        flg_s1_cmd = tk_cre_flg(&cflg);
        if (flg_s1_cmd < E_OK) return flg_s1_cmd;

        /* 5. Event flag for ISRs to report faults non-blocking */
        cflg.flgatr  = TA_TFIFO | TA_WMUL;
        cflg.iflgptn = 0;
        flg_s1_isr_fault = tk_cre_flg(&cflg);
        if (flg_s1_isr_fault < E_OK) return flg_s1_isr_fault;

        ccyc.cycatr = TA_HLNG;
        ccyc.cychdr = (FP)led_cyclic_handler;
        ccyc.cyctim = S1_TICK_MS;
        ccyc.cycphs = S1_TICK_MS;
        cyc_led = tk_cre_cyc(&ccyc);
        return (cyc_led < E_OK) ? cyc_led : E_OK;
}

EXPORT ER subsystem1_start(void)
{
        tid_led = app_start_task(led_task, S1_PRI_LED, 0, "led_task");
        if (tid_led < E_OK) return tid_led;
        return tk_sta_cyc(cyc_led);
}

EXPORT void s1_mqtt_push_rx(const UB *data, INT len)
{
        if (data == NULL || len <= 0 || mbf_mqtt_rx <= 0) return;
        if (len > S1_CMD_MAX) len = S1_CMD_MAX; /* Truncate to 1 byte limit */
        (void)tk_snd_mbf(mbf_mqtt_rx, (void *)data, len, TMO_POL);
}

EXPORT INT s1_mqtt_pop_tx(UB *buf, INT max_len)
{
        INT len;
        if (buf == NULL || max_len < S1_MQTT_MSG_MAX || mbf_mqtt_tx <= 0) return 0;
        len = tk_rcv_mbf(mbf_mqtt_tx, buf, TMO_POL);
        return (len > 0) ? len : 0;
}