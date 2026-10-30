/*
 *      mqtt_utk.c
 *      MQTT client implementation for micro T-Kernel (RP2040, NO_SYS=1).
 *      Handles broker connection, telemetry publishing, and inbound command subscription.
 */

#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include "lwip/timeouts.h"
#include "wifi_secrets.h"
#include "mqtt_utk.h"
#include <tm/tmonitor.h>

/* Subsystem 1 non-blocking bridge functions and fault reporting */
extern void s1_mqtt_push_rx(const UB *data, INT len);
extern INT  s1_mqtt_pop_tx(UB *buf, INT max_len);
extern void s1_report_fault(UW fault_bit);
extern void s1_clear_fault(UW fault_bit);
#define FAULT_WIFI (1U << 5)   /* Wi-Fi / MQTT link fault bit */

/* Timing configuration */
#define MQTT_UTK_HEARTBEAT_MS   1000    /* Telemetry transmission interval (1s) */
#define MQTT_UTK_KEEPALIVE_S    60      /* MQTT broker keep-alive timer (60s) */
#define MQTT_UTK_RETRY_MS       3000    /* Broker reconnect retry interval (3s) */

/* Inbound message buffer capacities */
#define MQTT_UTK_RX_TOPIC_MAX   64      /* Maximum incoming topic length */
#define MQTT_UTK_RX_DATA_MAX    128     /* Maximum incoming payload length */

static mqtt_client_t    *client;
static ip_addr_t        broker_addr;
static u8_t             connecting;

static char             rx_topic[MQTT_UTK_RX_TOPIC_MAX];
static UB               rx_data[MQTT_UTK_RX_DATA_MAX];
static u16_t            rx_len;
static u8_t             rx_truncated;

static void heartbeat_timeout(void *arg);

/* Handles incoming publish topic name from broker */
static void incoming_publish_cb(void *arg, const char *topic, u32_t tot_len)
{
        u32_t   i;

        (void)arg;
        (void)tot_len;

        for(i = 0; i < (MQTT_UTK_RX_TOPIC_MAX - 1) && topic[i] != '\0'; i++) {
                rx_topic[i] = topic[i];
        }
        rx_topic[i] = '\0';

        rx_len = 0;
        rx_truncated = 0;
}

/* Handles incoming publish data payload from broker */
static void incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags)
{
        u16_t   i;

        (void)arg;

        for(i = 0; i < len; i++) {
                if(rx_len < (MQTT_UTK_RX_DATA_MAX - 1)) {
                        rx_data[rx_len++] = data[i];
                } else {
                        rx_truncated = 1;
                }
        }

        if((flags & MQTT_DATA_FLAG_LAST) != 0) {
                rx_data[rx_len] = '\0';
                tm_printf((UB*)"mqtt: RX [%s] \"%s\"%s\n", rx_topic, rx_data,
                          rx_truncated? " (truncated)": "");

                /* Forward non-JSON command payloads into Subsystem 1's inbound message buffer */
                if(rx_len > 0 && rx_data[0] != '{') {
                        s1_mqtt_push_rx(rx_data, (INT)rx_len);
                }
        }
}

/* Callback for topic subscription confirmation */
static void subscribe_done_cb(void *arg, err_t result)
{
        (void)arg;
        if(result == ERR_OK) {
                tm_printf((UB*)"mqtt: subscribed to " MQTT_TOPIC_CMD "\n");
        } else {
                tm_printf((UB*)"mqtt: subscribe failed, err=%d\n", (int)result);
        }
}

/* Callback for publish completion */
static void publish_done_cb(void *arg, err_t result)
{
        (void)arg;
        if(result != ERR_OK) {
                tm_printf((UB*)"mqtt: publish failed, err=%d\n", (int)result);
        }
}

/* Periodic timer: Drains 8-byte telemetry packets from Subsystem 1 and publishes over MQTT */
static void heartbeat_timeout(void *arg)
{
        UB      payload[MQTT_UTK_RX_DATA_MAX];
        INT     len;

        (void)arg;

        if(mqtt_client_is_connected(client)) {
                /* Drain queued 8-byte telemetry packets from Subsystem 1 */
                while((len = s1_mqtt_pop_tx(payload, sizeof(payload) - 1)) > 0) {
                        payload[len] = '\0';
                        INT k;
                        tm_printf((UB*)"mqtt: TX [" MQTT_TOPIC "] (%d bytes: ", len);
                        for(k = 0; k < len; k++) {
                                tm_printf((UB*)"%02X ", payload[k]);
                        }
                        tm_printf((UB*)")\n");
                        (void)mqtt_publish(client, MQTT_TOPIC, payload, (u16_t)len,
                                           0 /* qos */, 0 /* retain */,
                                           publish_done_cb, NULL);
                }
        }

        /* Re-arms itself every 1 second */
        sys_timeout(MQTT_UTK_HEARTBEAT_MS, heartbeat_timeout, NULL);
}

/* Timer callback to retry MQTT broker connection on failure */
static void retry_timeout(void *arg)
{
        (void)arg;
        connecting = 0;
        mqtt_utk_start();
}

/* Connection status callback: handles link up/down events and topic subscription */
static void connection_cb(mqtt_client_t *c, void *arg, mqtt_connection_status_t status)
{
        (void)c;
        (void)arg;

        if(status == MQTT_CONNECT_ACCEPTED) {
                err_t   err;

                /* Connection successful: Clear Wi-Fi/MQTT fault */
                s1_clear_fault(FAULT_WIFI);
                connecting = 0;

                tm_printf((UB*)"mqtt: connected to " MQTT_BROKER_IP "\n");

                /* Register publish callbacks and subscribe to command topic */
                mqtt_set_inpub_callback(client, incoming_publish_cb,
                                        incoming_data_cb, NULL);

                err = mqtt_subscribe(client, MQTT_TOPIC_CMD, 0 /* qos */,
                                     subscribe_done_cb, NULL);
                if(err != ERR_OK) {
                        tm_printf((UB*)"mqtt: mqtt_subscribe() failed, err=%d\n",
                                  (int)err);
                }

                /* Restart telemetry publish timer */
                sys_untimeout(heartbeat_timeout, NULL);
                sys_timeout(MQTT_UTK_HEARTBEAT_MS, heartbeat_timeout, NULL);
        } else {
                /* Connection failed or dropped: Report Wi-Fi/MQTT fault and schedule retry */
                s1_report_fault(FAULT_WIFI);
                tm_printf((UB*)"mqtt: connection status %d, retrying in %ds\n",
                          (int)status, MQTT_UTK_RETRY_MS / 1000);
                sys_timeout(MQTT_UTK_RETRY_MS, retry_timeout, NULL);
        }
}

/* Initializes MQTT client and connects to configured broker */
void mqtt_utk_start(void)
{
        struct mqtt_connect_client_info_t       client_info = {0};
        err_t                                   err;

        if(connecting || (client != NULL && mqtt_client_is_connected(client))) {
                return;
        }

        if(client == NULL) {
                client = mqtt_client_new();
                if(client == NULL) {
                        tm_printf((UB*)"mqtt: mqtt_client_new() failed\n");
                        return;
                }
        }

        if(!ip4addr_aton(MQTT_BROKER_IP, ip_2_ip4(&broker_addr))) {
                tm_printf((UB*)"mqtt: bad MQTT_BROKER_IP\n");
                return;
        }
        IP_SET_TYPE(&broker_addr, IPADDR_TYPE_V4);

        client_info.client_id = MQTT_CLIENT_ID;
        client_info.keep_alive = MQTT_UTK_KEEPALIVE_S;

        connecting = 1;
        err = mqtt_client_connect(client, &broker_addr, MQTT_BROKER_PORT,
                                  connection_cb, NULL, &client_info);
        if(err != ERR_OK) {
                tm_printf((UB*)"mqtt: mqtt_client_connect() failed, err=%d\n", (int)err);
                connecting = 0;
        }
}
