/*
 *      subsystem1.h
 *      Subsystem 1 -- LED heartbeat & Wi-Fi/MQTT Telemetry.  Student ID: 2501283.
 *
 *      This header is subsystem 1's PUBLIC interface: only what other
 *      subsystems are allowed to use.
 */

#ifndef SUBSYSTEM1_H
#define SUBSYSTEM1_H

#include "../app.h"
#include <stdint.h>

#define S1_PRI_LED              12
#define S1_LED_PIN              19
#define S1_TICK_MS              250
#define S1_TELEM_TICKS          4

#define S1_MQTT_MSG_MAX         8               /* Shrunk from 96 to 8 for minimum-bit binary payload */
#define S1_CMD_MAX              1               /* Commands are now 1 byte */

/* Remote command event flag bits (published on flg_s1_cmd) */
#define S1_FLG_CMD_START        0x0001U
#define S1_FLG_CMD_STOP         0x0002U
#define S1_FLG_CMD_ESTOP        0x0004U
#define S1_FLG_CMD_CLEAR        0x0008U
#define S1_FLG_CMD_PING         0x0010U

/* Remote command enum */
typedef enum {
        S1_CMD_NONE  = 0,
        S1_CMD_START = 1,
        S1_CMD_STOP  = 2,
        S1_CMD_ESTOP = 3,
        S1_CMD_CLEAR = 4,
        S1_CMD_PING  = 5
} S1_CMD_TYPE;

/* Minimum-Bit Compact Binary Telemetry Payload (8 bytes total) */
typedef struct __attribute__((packed)){
    uint32_t seq         : 8;   /* 0..255 rolling counter */
    int32_t  speed_mm_s  : 10;  /* -512..+511 mm/s */
    uint32_t obstacle_cm : 9;   /* 0..511 cm */
    uint32_t car_state   : 3;   /* 0..7 state machine */
    uint32_t hump_state  : 2;   /* 0..3 terrain state */

    int32_t  pitch_deg   : 8;   /* -128..+127 deg */
    uint32_t line_mask   : 5;   /* 5-bit IR sensor array */
    uint32_t barcode_id  : 7;   /* 7-bit ASCII char / ID */
    uint32_t fault_mask  : 12;  /* 12-bit consolidated fault bitmask */
} S1_TELEMETRY_PKT;

/* Minimum-Bit Compact Binary Command Payload (1 byte total) */
typedef struct __attribute__((packed)){
    uint8_t cmd_id   : 3; /* 1=START, 2=STOP, 3=ESTOP, 4=CLEAR, 5=PING */
    uint8_t reserved : 5;
} S1_CMD_PKT;

/* IF-05: the LED task's ID */
IMPORT ID       tid_led;

/* Kernel objects owned by Subsystem 1 */
IMPORT ID       mbf_mqtt_tx;
IMPORT ID       mbf_mqtt_rx;
IMPORT ID       mtx_telemetry;
IMPORT ID       flg_s1_cmd;
IMPORT ID       flg_s1_isr_fault; /* NEW: Non-blocking ISR fault reporting */

/* Counters & state */
IMPORT UW               s1_tx_queued;
IMPORT UW               s1_rx_commands;
IMPORT UW               s1_rx_rejected;
IMPORT S1_CMD_TYPE      s1_last_cmd;

/* Public Gatekeeper APIs (Replaces s1_update_telemetry) */
IMPORT void     s1_set_speed(INT speed_mm_s);
IMPORT void     s1_set_obstacle(INT obstacle_cm);
IMPORT void     s1_set_pitch(INT pitch_deg);
IMPORT void     s1_set_barcode_id(UB barcode_id);
IMPORT void     s1_set_car_state(UW state, UW hump);
IMPORT void     s1_set_line_mask(UW mask);

/* Centralized Fault Management */
IMPORT void     s1_report_fault(UW fault_bit); /* Normal tasks */
IMPORT void     s1_clear_fault(UW fault_bit);
IMPORT void	s1_report_fault_isr(UW fault_bit); /* ISR Safe */
/* Note: ISRs must call tk_set_flg(flg_s1_isr_fault, fault_bit) directly */

IMPORT void     s1_get_telemetry(S1_TELEMETRY_PKT *out);
IMPORT S1_CMD_TYPE s1_parse_command(const UB *payload, INT len);

/* CSV logger */
IMPORT void     s1_csv_log_event(const char *event_type, const char *details);

/* Non-blocking bridge helpers */
IMPORT void     s1_mqtt_push_rx(const UB *data, INT len);
IMPORT INT      s1_mqtt_pop_tx(UB *buf, INT max_len);

IMPORT ID       cyc_led;
IMPORT void     led_task(INT stacd, void *exinf);
IMPORT void     led_cyclic_handler(void *exinf);

#endif /* SUBSYSTEM1_H */
