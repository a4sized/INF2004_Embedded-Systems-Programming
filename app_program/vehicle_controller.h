/** @file vehicle_controller.h @brief Whole-car startup and follow authority. */
#ifndef VEHICLE_CONTROLLER_H
#define VEHICLE_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>

typedef enum
{
    VC_INIT = 0,
    VC_READY,
    VC_FOLLOWING,
    VC_STOPPED,
    VC_FAULT
} vc_state_t;

typedef struct
{
    bool b_imu_ready;
    bool b_ultrasonic_ready;
    bool b_ir_ready;
    bool b_wifi_connected;
    bool b_start;
    bool b_stop;
    bool b_estop;
    bool b_clear;
    uint32_t fault_mask;
} vc_inputs_t;

/** Pure policy: Wi-Fi is required only before the first start. */
vc_state_t vehicle_controller_step (vc_state_t state,
                                    vc_inputs_t const * p_inputs);

/** Buddy 1 calls this from task context when its link status changes. */
void vehicle_controller_set_wifi_connected (bool b_connected);

/** Start the controller after subsystem objects and tasks are started. */
ER vehicle_controller_start (void);

#endif
