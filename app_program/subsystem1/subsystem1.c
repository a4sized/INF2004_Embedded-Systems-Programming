/*
 *      subsystem1.c
 *      Subsystem 1 initialization and micro T-Kernel / lwIP bridge functions.
 */

#include "subsystem1.h"

/* Subsystem 1 task and kernel object IDs */
EXPORT ID       tid_led;          /* Task ID for LED heartbeat and telemetry */
EXPORT ID       cyc_led;          /* Cyclic timer ID (250 ms period) */
EXPORT ID       mbf_mqtt_tx;      /* Message buffer for outbound telemetry */
EXPORT ID       mbf_mqtt_rx;      /* Message buffer for inbound commands */
EXPORT ID       mtx_telemetry;    /* Priority-inheritance mutex protecting telemetry state */
EXPORT ID       flg_s1_cmd;       /* Event flag for decoded remote commands */
EXPORT ID       flg_s1_isr_fault; /* Event flag for non-blocking ISR fault reporting */

/* Initializes Subsystem 1 synchronization objects, buffers, and cyclic timer */
EXPORT ER subsystem1_init(void)
{
        T_CCYC  ccyc = {0};
        T_CMBF  cmbf = {0};
        T_CMTX  cmtx = {0};
        T_CFLG  cflg = {0};

        /* Priority-inheritance mutex protecting telemetry struct */
        cmtx.mtxatr = TA_TFIFO | TA_INHERIT;
        mtx_telemetry = tk_cre_mtx(&cmtx);
        if (mtx_telemetry < E_OK) return mtx_telemetry;

        /* Outbound MQTT message buffer (8-byte binary packets) */
        cmbf.mbfatr = TA_TFIFO;
        cmbf.bufsz  = 256;
        cmbf.maxmsz = S1_MQTT_MSG_MAX;
        mbf_mqtt_tx = tk_cre_mbf(&cmbf);
        if (mbf_mqtt_tx < E_OK) return mbf_mqtt_tx;

        /* Inbound MQTT message buffer (1-byte or short commands) */
        cmbf.mbfatr = TA_TFIFO;
        cmbf.bufsz  = 256;
        cmbf.maxmsz = S1_CMD_MAX;
        mbf_mqtt_rx = tk_cre_mbf(&cmbf);
        if (mbf_mqtt_rx < E_OK) return mbf_mqtt_rx;

        /* Event flag for decoded remote MQTT commands */
        cflg.flgatr  = TA_TFIFO | TA_WMUL;
        cflg.iflgptn = 0;
        flg_s1_cmd = tk_cre_flg(&cflg);
        if (flg_s1_cmd < E_OK) return flg_s1_cmd;

        /* Event flag for non-blocking ISR fault reporting */
        cflg.flgatr  = TA_TFIFO | TA_WMUL;
        cflg.iflgptn = 0;
        flg_s1_isr_fault = tk_cre_flg(&cflg);
        if (flg_s1_isr_fault < E_OK) return flg_s1_isr_fault;

        /* Cyclic timer handler (250 ms period) */
        ccyc.cycatr = TA_HLNG;
        ccyc.cychdr = (FP)led_cyclic_handler;
        ccyc.cyctim = S1_TICK_MS;
        ccyc.cycphs = S1_TICK_MS;
        cyc_led = tk_cre_cyc(&ccyc);
        return (cyc_led < E_OK) ? cyc_led : E_OK;
}

/* Starts LED task and activates 250 ms cyclic timer */
EXPORT ER subsystem1_start(void)
{
        tid_led = app_start_task(led_task, S1_PRI_LED, 0, "led_task");
        if (tid_led < E_OK) return tid_led;
        return tk_sta_cyc(cyc_led);
}

/* Pushes incoming MQTT command payload into inbound message buffer (non-blocking) */
EXPORT void s1_mqtt_push_rx(const UB *data, INT len)
{
        if (data == NULL || len <= 0 || mbf_mqtt_rx <= 0) return;
        if (len > S1_CMD_MAX) len = S1_CMD_MAX; /* Truncate to buffer limit */
        (void)tk_snd_mbf(mbf_mqtt_rx, (void *)data, len, TMO_POL);
}

/* Pops queued telemetry packet from outbound message buffer (non-blocking) */
EXPORT INT s1_mqtt_pop_tx(UB *buf, INT max_len)
{
        INT len;
        if (buf == NULL || max_len < S1_MQTT_MSG_MAX || mbf_mqtt_tx <= 0) return 0;
        len = tk_rcv_mbf(mbf_mqtt_tx, buf, TMO_POL);
        return (len > 0) ? len : 0;
}

