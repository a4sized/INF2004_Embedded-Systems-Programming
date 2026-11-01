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

/* Task and timing configuration */
#define S1_PRI_LED              12              /* Priority of LED/telemetry task (band 12-13) */
#define S1_LED_PIN              19              /* GPIO pin for status heartbeat LED */
#define S1_TICK_MS              250             /* Cyclic timer wakeup interval (250 ms) */
#define S1_TELEM_TICKS          4               /* Ticks per telemetry transmission (4 * 250ms = 1s) */

/* MQTT buffer capacities */
#define S1_MQTT_MSG_MAX         8               /* Minimum-bit binary payload (8 bytes) */
#define S1_CMD_MAX              16              /* 1 byte binary or short ASCII inbound commands */

/* Remote command event flag bits (published on flg_s1_cmd) */
#define S1_FLG_CMD_START        0x0001U         /* Start autonomous drive command */
#define S1_FLG_CMD_STOP         0x0002U         /* Normal stop command */
#define S1_FLG_CMD_ESTOP        0x0004U         /* Emergency stop command */
#define S1_FLG_CMD_CLEAR        0x0008U         /* Clear active faults command */
#define S1_FLG_CMD_PING         0x0010U         /* Ping / keepalive request command */

/* Remote command enum */
typedef enum {
        S1_CMD_NONE  = 0,
        S1_CMD_START = 1,
        S1_CMD_STOP  = 2,
        S1_CMD_ESTOP = 3,
        S1_CMD_CLEAR = 4,
        S1_CMD_PING  = 5
} S1_CMD_TYPE;

/* Minimum-Bit Compact Binary Telemetry Payload (8 bytes total)
 * Packed using bit-fields across two 32-bit words (Little-Endian) */
typedef struct __attribute__((packed)){
    /* Word 0 (32 bits) */
    uint32_t seq         : 8;   /* 0..255 rolling packet sequence counter */
    int32_t  speed_mm_s  : 10;  /* -512..+511 mm/s signed wheel speed */
    uint32_t obstacle_cm : 9;   /* 0..511 cm ultrasonic distance reading */
    uint32_t car_state   : 3;   /* 0..7 vehicle state machine state */
    uint32_t hump_state  : 2;   /* 0..3 terrain/ramp estimation state */

    /* Word 1 (32 bits) */
    int32_t  pitch_deg   : 8;   /* -128..+127 deg signed IMU pitch angle */
    uint32_t line_mask   : 5;   /* 5-bit line tracker IR sensor pattern */
    uint32_t barcode_id  : 7;   /* 7-bit ASCII barcode character or ID */
    uint32_t fault_mask  : 12;  /* 12-bit consolidated fault bitmask */
} S1_TELEMETRY_PKT;

/* Minimum-Bit Compact Binary Command Payload (1 byte total) */
typedef struct __attribute__((packed)){
    uint8_t cmd_id   : 3;       /* 3-bit command ID (1=START..5=PING) */
    uint8_t reserved : 5;       /* Reserved bits for byte alignment */
} S1_CMD_PKT;

/* IF-05: the LED task's ID */
IMPORT ID       tid_led;

/* Kernel objects owned by Subsystem 1 */
IMPORT ID       mbf_mqtt_tx;      /* Outbound queue for 8-byte telemetry packets */
IMPORT ID       mbf_mqtt_rx;      /* Inbound queue for received remote commands */
IMPORT ID       mtx_telemetry;    /* Priority-inheritance mutex protecting telemetry struct */
IMPORT ID       flg_s1_cmd;       /* Event flag signaling decoded remote commands */
IMPORT ID       flg_s1_isr_fault; /* Event flag for non-blocking ISR fault reporting */

/* Counters & state */
IMPORT UW               s1_tx_queued;    /* Total telemetry packets queued for TX */
IMPORT UW               s1_rx_commands;  /* Total valid commands received and processed */
IMPORT UW               s1_rx_rejected;  /* Total invalid/unrecognized commands rejected */
IMPORT S1_CMD_TYPE      s1_last_cmd;     /* Most recently processed command */

/* Public Gatekeeper APIs (Thread-safe setters under mtx_telemetry) */
IMPORT void     s1_set_speed(INT speed_mm_s);
IMPORT void     s1_set_obstacle(INT obstacle_cm);
IMPORT void     s1_set_pitch(INT pitch_deg);
IMPORT void     s1_set_barcode_id(UB barcode_id);
IMPORT void     s1_set_car_state(UW state, UW hump);
IMPORT void     s1_set_line_mask(UW mask);

/* Centralized Fault Management */
IMPORT void     s1_report_fault(UW fault_bit);      /* Report fault from normal task context */
IMPORT void     s1_clear_fault(UW fault_bit);       /* Clear fault from normal task context */
IMPORT void     s1_report_fault_isr(UW fault_bit);  /* Report fault safely from ISR context */

/* Telemetry and Command Helpers */
IMPORT void     s1_get_telemetry(S1_TELEMETRY_PKT *out);          /* Atomic snapshot of telemetry */
IMPORT S1_CMD_TYPE s1_parse_command(const UB *payload, INT len); /* Parse binary or ASCII command */

/* CSV logger */
IMPORT void     s1_csv_log_event(const char *event_type, const char *details); /* Output event line to USB serial */

/* Non-blocking bridge helpers between lwIP and micro T-Kernel */
IMPORT void     s1_mqtt_push_rx(const UB *data, INT len);         /* Push inbound MQTT command */
IMPORT INT      s1_mqtt_pop_tx(UB *buf, INT max_len);             /* Pop outbound telemetry packet */

IMPORT ID       cyc_led;                                          /* Cyclic handler ID (250 ms) */
IMPORT void     led_task(INT stacd, void *exinf);                 /* Core LED & telemetry task */
IMPORT void     led_cyclic_handler(void *exinf);                  /* Cyclic handler callback */

#endif /* SUBSYSTEM1_H */
