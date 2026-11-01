/*
 *      mqtt_utk.h
 *      MQTT client interface for micro T-Kernel (RP2040, NO_SYS=1).
 */

#ifndef MQTT_UTK_H
#define MQTT_UTK_H

/* Starts MQTT client connection to broker and initializes periodic telemetry publishing */
void mqtt_utk_start(void);

#endif /* MQTT_UTK_H */
