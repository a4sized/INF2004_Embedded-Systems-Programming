/*
 *      subsystem1.h
 *      Subsystem 1 -- LED heartbeat & Wi-Fi/MQTT Telemetry.  Student ID: 2501283.
 *
 *      This header is subsystem 1's PUBLIC interface: only what other
 *      subsystems are allowed to use.  Everything else stays LOCAL in the .c
 *      files.  Other subsystems include it as "../subsystem1/subsystem1.h".
 */

#ifndef SUBSYSTEM1_H
#define SUBSYSTEM1_H

#include "../app.h"

#define S1_PRI_LED              12              /* priority band 12-13 (TEAM.md) */
#define S1_LED_PIN              19              /* GP19: LED + 330R resistor to GND */
#define S1_TICK_MS              250             /* LED toggles every tick */
#define S1_TELEM_TICKS          4               /* publish telemetry every 4 ticks (1 s) */

#define S1_MQTT_MSG_MAX         96              /* max bytes per MQTT TX/RX message */
#define S1_CMD_MAX              32              /* max bytes for parsed command string */

/* Remote command event flag bits (published on flg_s1_cmd) */
#define S1_FLG_CMD_START        0x0001U
#define S1_FLG_CMD_STOP         0x0002U
#define S1_FLG_CMD_ESTOP        0x0004U
#define S1_FLG_CMD_CLEAR        0x0008U
#define S1_FLG_CMD_PING         0x0010U

/* Remote command enum for state inspection and unit tests */
typedef enum {
        S1_CMD_NONE  = 0,
        S1_CMD_START = 1,
        S1_CMD_STOP  = 2,
        S1_CMD_ESTOP = 3,
        S1_CMD_CLEAR = 4,
        S1_CMD_PING  = 5
} S1_CMD_TYPE;

/* Shared robot telemetry state aggregated by Subsystem 1 */
typedef struct {
        UW      seq;            /* telemetry packet sequence number */
        INT     speed_mm_s;     /* current robot speed (mm/s) */
        INT     obstacle_cm;    /* ultrasonic obstacle distance (cm) */
        INT     pitch_deg;      /* IMU pitch angle (deg) */
        UW      fault_code;     /* active fault bitmask (0 = OK) */
        UB      barcode[16];    /* last decoded barcode string */
} S1_TELEMETRY;

/* IF-05: the LED task's ID, so subsystem 4 may pause/resume it */
IMPORT ID       tid_led;

/* Kernel objects owned by Subsystem 1 */
IMPORT ID       mbf_mqtt_tx;    /* outbound telemetry lines -> mqtt_utk.c */
IMPORT ID       mbf_mqtt_rx;    /* inbound command payloads <- mqtt_utk.c */
IMPORT ID       mtx_telemetry;  /* protects s1_telemetry state */
IMPORT ID       flg_s1_cmd;     /* command event flags for other subsystems */

/* Counters & state exposed for on-target unit/integration testing */
IMPORT UW               s1_tx_queued;
IMPORT UW               s1_rx_commands;
IMPORT UW               s1_rx_rejected;
IMPORT S1_CMD_TYPE      s1_last_cmd;

/* Public helper functions (safe to call from any task) */
IMPORT void     s1_update_telemetry(INT speed_mm_s, INT obstacle_cm,
                                    INT pitch_deg, UW fault_code,
                                    const UB *barcode);
IMPORT void     s1_get_telemetry(S1_TELEMETRY *out);
IMPORT INT      s1_format_telemetry_json(const S1_TELEMETRY *t, UB *buf, INT max_len);
IMPORT S1_CMD_TYPE s1_parse_command(const UB *payload);

/* CSV logger for tracking failures, state changes and commands */
IMPORT void     s1_csv_log_event(const char *event_type, const char *details);

/* Non-blocking bridge helpers called by mqtt_utk.c */
IMPORT void     s1_mqtt_push_rx(const UB *data, INT len);
IMPORT INT      s1_mqtt_pop_tx(UB *buf, INT max_len);

/* Private to subsystem 1, shared between its own .c files */
IMPORT ID       cyc_led;
IMPORT void     led_task(INT stacd, void *exinf);
IMPORT void     led_cyclic_handler(void *exinf);

#endif /* SUBSYSTEM1_H */
