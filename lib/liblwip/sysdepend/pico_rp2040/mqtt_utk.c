/*
 *      mqtt_utk.c
 *      MQTT client wiring for micro T-Kernel (RP2040, NO_SYS=1)
 *
 *      Runs entirely on the single processor-1-owned task that already owns
 *      the CYW43439 and the rest of lwIP (docs/RELEASE.md safety rule 6):
 *      cyw43_utk_task() (cyw43_utk.c) calls mqtt_utk_start() once, the first
 *      time it observes CYW43_LINK_UP. mqtt.c's own keepalive/ping/retry
 *      timers, and the heartbeat below, both ride the same sys_check_timeouts()
 *      polling that task already does for DHCP and TCP -- nothing extra runs
 *      on any other task or core.
 *
 *      This file does not include tk/tkernel.h or any pico-sdk/cyw43 header
 *      (unlike cyw43_utk.c and sys_arch.c), so it isn't subject to either
 *      file's size_t workaround -- tm/tmonitor.h alone is namespace-safe here.
 */

#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include "lwip/timeouts.h"
#include "wifi_secrets.h"
#include "mqtt_utk.h"
#include <tm/tmonitor.h>

/* Subsystem 1 non-blocking bridge functions (defined in subsystem1.c).
 * systemFaults.h is intentionally NOT included here: it pulls tk/tkernel.h
 * which conflicts with lwIP's size_t (see file header comment).
 * Forward-declare what we need with namespace-safe types instead. */
extern void s1_mqtt_push_rx(const UB *data, INT len);
extern INT  s1_mqtt_pop_tx(UB *buf, INT max_len);
extern void s1_report_fault(UW fault_bit);
extern void s1_clear_fault(UW fault_bit);
#define FAULT_WIFI (1U << 5)   /* must match systemFaults.h */

#define MQTT_UTK_HEARTBEAT_MS   1000
#define MQTT_UTK_KEEPALIVE_S    60
#define MQTT_UTK_RETRY_MS       3000

/* Inbound message staging. A publish arrives as one topic callback followed by
 * one or more data callbacks, so the topic has to be kept until the fragment
 * carrying MQTT_DATA_FLAG_LAST shows up. Both buffers are sized well past the
 * heartbeat this demo publishes; anything longer is truncated and flagged
 * rather than being allowed to run off the end. */
#define MQTT_UTK_RX_TOPIC_MAX   64
#define MQTT_UTK_RX_DATA_MAX    128

static mqtt_client_t    *client;
static ip_addr_t        broker_addr;
static u8_t             connecting;

static char             rx_topic[MQTT_UTK_RX_TOPIC_MAX];
static UB               rx_data[MQTT_UTK_RX_DATA_MAX];
static u16_t            rx_len;
static u8_t             rx_truncated;

static void heartbeat_timeout(void *arg);

/* mqtt.c NUL-terminates the topic before handing it over, but it is only
 * valid for the duration of this call -- copy it out. */
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

                /* Forward non-JSON payloads (e.g. START, STOP, ESTOP) into
                 * Subsystem 1's inbound command buffer without blocking. */
                if(rx_len > 0 && rx_data[0] != '{') {
                        s1_mqtt_push_rx(rx_data, (INT)rx_len);
                }
        }
}

static void subscribe_done_cb(void *arg, err_t result)
{
        (void)arg;
        if(result == ERR_OK) {
                tm_printf((UB*)"mqtt: subscribed to " MQTT_TOPIC_CMD "\n");
        } else {
                tm_printf((UB*)"mqtt: subscribe failed, err=%d\n", (int)result);
        }
}

static void publish_done_cb(void *arg, err_t result)
{
        (void)arg;
        if(result != ERR_OK) {
                tm_printf((UB*)"mqtt: publish failed, err=%d\n", (int)result);
        }
}

static void heartbeat_timeout(void *arg)
{
        UB      payload[MQTT_UTK_RX_DATA_MAX];
        INT     len;

        (void)arg;

        if(mqtt_client_is_connected(client)) {
                /* Drain queued telemetry packets from Subsystem 1 */
                while((len = s1_mqtt_pop_tx(payload, sizeof(payload) - 1)) > 0) {
                        payload[len] = '\0';
                        //tm_printf((UB*)"mqtt: TX [" MQTT_TOPIC_TELEMETRY "] (8 bytes binary)\n");
                        (void)mqtt_publish(client, MQTT_TOPIC_TELEMETRY, payload, (u16_t)len,
                                           0 /* qos */, 0 /* retain */,
                                           publish_done_cb, NULL);
                }
        }

        /* Re-arms itself every call -- mqtt.c's connection callback (below)
         * only starts this chain once, on the transition into MQTT_CONNECT_
         * ACCEPTED, so there is exactly one heartbeat chain alive at a time. */
        sys_timeout(MQTT_UTK_HEARTBEAT_MS, heartbeat_timeout, NULL);
}

static void retry_timeout(void *arg)
{
        (void)arg;
        connecting = 0;
        mqtt_utk_start();
}

static void connection_cb(mqtt_client_t *c, void *arg, mqtt_connection_status_t status)
{
        (void)c;
        (void)arg;

        if(status == MQTT_CONNECT_ACCEPTED) {
                err_t   err;

                /* Connection successful: Clear the MQTT drop fault */
                s1_clear_fault(FAULT_WIFI);

                /* No longer *connecting* -- clear the guard so that if this
                 * connection later drops, the poll loop's mqtt_utk_start() call
                 * is allowed to reconnect. Leaving it set would wedge the client
                 * permanently after the first disconnect. */
                connecting = 0;

                tm_printf((UB*)"mqtt: connected to " MQTT_BROKER_IP "\n");

                /* Subscribe to the same topic this client publishes to, so every
                 * heartbeat comes back from the broker and is printed by
                 * incoming_data_cb(). A matching RX for each TX is end-to-end
                 * proof of the round trip without needing mosquitto_sub on the
                 * broker host. Callbacks must be registered before subscribing,
                 * or the first delivery has nowhere to go. */
                mqtt_set_inpub_callback(client, incoming_publish_cb,
                                        incoming_data_cb, NULL);

                err = mqtt_subscribe(client, MQTT_TOPIC_CMD, 0 /* qos */,
                                     subscribe_done_cb, NULL);
                if(err != ERR_OK) {
                        tm_printf((UB*)"mqtt: mqtt_subscribe() failed, err=%d\n",
                                  (int)err);
                }

                /* Drop any chain left over from a previous connection before
                 * arming a new one. Each reconnect would otherwise add another
                 * self-perpetuating heartbeat, and every one of them holds a
                 * MEMP_SYS_TIMEOUT slot until the pool is exhausted. */
                sys_untimeout(heartbeat_timeout, NULL);
                sys_timeout(MQTT_UTK_HEARTBEAT_MS, heartbeat_timeout, NULL);
        } else {
                /* Connection failed or dropped: Set the MQTT drop fault */
                s1_report_fault(FAULT_WIFI);
                
                /* mqtt_client_connect() reconnects on its own for a dropped
                 * keepalive; a rejected/never-established connection does not
                 * retry itself, so do that here -- on a delay, so a broker
                 * that keeps rejecting (e.g. bad client ID) can't turn this
                 * into a tight reconnect loop. */
                tm_printf((UB*)"mqtt: connection status %d, retrying in %ds\n",
                          (int)status, MQTT_UTK_RETRY_MS / 1000);
                sys_timeout(MQTT_UTK_RETRY_MS, retry_timeout, NULL);
        }
}

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