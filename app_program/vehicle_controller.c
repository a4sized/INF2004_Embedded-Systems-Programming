/** @file vehicle_controller.c @brief RTOS adapter for mission permission. */
#include "vehicle_controller.h"

#include "app.h"
#include "subsystem1/subsystem1.h"
#include "subsystem2/subsystem2.h"
#include "subsystem3/ir_line_snapshot_shared.h"
#include "subsystem3/ir_motion_consumer.h"
#include "subsystem4/subsystem4.h"
#include "subsystem5/subsystem5.h"
#include "systemFaults.h"

#define VC_PERIOD_MS 20u
#define VC_PRIORITY 17

static bool b_wifi_connected;

/* Bits for "what the controller is still waiting for" (startup prints). */
#define VC_WAIT_IMU    (1u << 0)
#define VC_WAIT_US     (1u << 1)
#define VC_WAIT_IR     (1u << 2)
#define VC_WAIT_WIFI   (1u << 3)
#define VC_WAIT_START  (1u << 4)

/* Print what INIT / READY is still waiting for, whenever that changes. */
static void vehicle_controller_print_waiting (vc_state_t state,
                                              vc_inputs_t const * p_inputs,
                                              UINT * p_last)
{
    UINT waiting = 0u;

    if ((VC_INIT == state) || (VC_READY == state))
    {
        waiting |= p_inputs->b_imu_ready ? 0u : VC_WAIT_IMU;
        waiting |= p_inputs->b_ultrasonic_ready ? 0u : VC_WAIT_US;
        waiting |= p_inputs->b_ir_ready ? 0u : VC_WAIT_IR;
        waiting |= (p_inputs->b_wifi_connected &&
                    (0u == (p_inputs->fault_mask & FAULT_WIFI))) ?
                   0u : VC_WAIT_WIFI;
        waiting |= (VC_READY == state) ? VC_WAIT_START : 0u;
    }

    if (waiting != *p_last)
    {
        *p_last = waiting;
        if (0u != waiting)
        {
            tm_printf((UB *)"VC waiting for:%s%s%s%s%s\n",
                      (0u != (waiting & VC_WAIT_IMU)) ? " IMU" : "",
                      (0u != (waiting & VC_WAIT_US)) ? " ULTRASONIC" : "",
                      (0u != (waiting & VC_WAIT_IR)) ? " IR" : "",
                      (0u != (waiting & VC_WAIT_WIFI)) ? " WIFI" : "",
                      (0u != (waiting & VC_WAIT_START)) ?
                      " MQTT_START" : "");
        }
    }
}

static char const * vehicle_controller_state_name (vc_state_t state)
{
    switch (state)
    {
        case VC_INIT: return "INIT";
        case VC_READY: return "READY";
        case VC_FOLLOWING: return "FOLLOWING";
        case VC_AVOIDING: return "AVOIDING";
        case VC_STOPPED: return "STOPPED";
        case VC_FAULT: return "FAULT";
        default: return "INVALID";
    }
}

void vehicle_controller_set_wifi_connected (bool b_connected)
{
    __atomic_store_n(&b_wifi_connected, b_connected, __ATOMIC_RELEASE);
}

static void vehicle_controller_task (INT start_code, void * p_extra)
{
    vc_state_t state = VC_INIT;
    UINT last_waiting = 0u;
    ir_motion_settings_t const permitted =
    {
        true, IR_START_BASE_SPEED, IR_START_TURN_SPEED, IR_START_MAX_AGE_MS
    };
    ir_motion_settings_t const denied =
    {
        false, IR_START_BASE_SPEED, IR_START_TURN_SPEED, IR_START_MAX_AGE_MS
    };
    (void)start_code;
    (void)p_extra;
    if (E_OK != ir_motion_consumer_set_following(&denied))
    {
        motion_trigger_safety_halt(FAULT_MOTOR);
        state = VC_FAULT;
    }
    s1_csv_log_event("VC_STATE", vehicle_controller_state_name(state));
    tm_printf((UB *)"VC state: %s\n", vehicle_controller_state_name(state));

    for (;;)
    {
        UINT commands = 0u;
        S1_TELEMETRY_PKT telemetry = {0};
        ir_line_snapshot_t ir = {0};
        SYSTIM now = {0};
        vc_inputs_t inputs = {0};
        vc_state_t next;

        (void)tk_wai_flg(flg_s1_cmd,
                        S1_FLG_CMD_START | S1_FLG_CMD_STOP |
                        S1_FLG_CMD_ESTOP | S1_FLG_CMD_CLEAR,
                        TWF_ORW | TWF_CLR, &commands, TMO_POL);
        inputs.b_start = (0u != (commands & S1_FLG_CMD_START));
        inputs.b_stop = (0u != (commands & S1_FLG_CMD_STOP));
        inputs.b_estop = (0u != (commands & S1_FLG_CMD_ESTOP));
        inputs.b_clear = (0u != (commands & S1_FLG_CMD_CLEAR));
        s1_get_telemetry(&telemetry);
        inputs.fault_mask = telemetry.fault_mask;
        inputs.b_imu_ready = (TRUE == s4_is_calibrated());
        inputs.b_ultrasonic_ready = subsystem5_startup_scan_ready();
        inputs.b_avoiding = subsystem5_is_avoiding();
        inputs.b_wifi_connected = __atomic_load_n(&b_wifi_connected,
                                                  __ATOMIC_ACQUIRE);
        if (ir_line_snapshot_shared_read(&ir) &&
            (E_OK == tk_get_tim(&now)))
        {
            inputs.b_ir_ready = ir.b_ready && ir.b_gpio_ready &&
                ((uint32_t)(now.lo - ir.sampled_at_ms) <=
                 IR_START_MAX_AGE_MS);
        }

        vehicle_controller_print_waiting(state, &inputs, &last_waiting);

        next = vehicle_controller_step(state, &inputs);
        if (next != state)
        {
            /* Leaving AVOIDING: take Subsystem 5's drive permission away
             * first (it stops the wheels), then let the line follower write
             * to Buddy 2 again (it writes zero unless following is
             * permitted below). */
            if (VC_AVOIDING == state)
            {
                subsystem5_set_drive_permission(false);
                if (E_OK != ir_motion_consumer_set_paused(false))
                {
                    motion_trigger_safety_halt(FAULT_MOTOR);
                    next = VC_FAULT;
                }
            }
            /* Revoke first; the consumer serializes this with its S2 write. */
            if (VC_FOLLOWING != next)
            {
                if (E_OK != ir_motion_consumer_set_following(&denied))
                {
                    motion_trigger_safety_halt(FAULT_MOTOR);
                    next = VC_FAULT;
                }
            }
            state = next;
            if ((VC_FAULT == state) &&
                (inputs.b_estop ||
                 (0u != (inputs.fault_mask & TIER1_FATAL_MASK))))
            {
                motion_trigger_safety_halt(
                    inputs.b_estop ? FAULT_ESTOP :
                    (inputs.fault_mask & TIER1_FATAL_MASK));
            }
            if (VC_FOLLOWING == state)
            {
                if (E_OK != ir_motion_consumer_set_following(&permitted))
                {
                    motion_trigger_safety_halt(FAULT_MOTOR);
                    state = VC_FAULT;
                }
            }
            /* Entering AVOIDING: following is already revoked above. Stop
             * the line follower writing to Buddy 2 at all, then let
             * Subsystem 5 drive the bypass. */
            if (VC_AVOIDING == state)
            {
                if (E_OK != ir_motion_consumer_set_paused(true))
                {
                    motion_trigger_safety_halt(FAULT_MOTOR);
                    state = VC_FAULT;
                }
                else
                {
                    subsystem5_set_drive_permission(true);
                }
            }
            s1_csv_log_event("VC_STATE",
                             vehicle_controller_state_name(state));
            /* Always on the serial monitor (the CSV log above is only
             * printed while a fault is active). */
            tm_printf((UB *)"VC state: %s\n",
                      vehicle_controller_state_name(state));
        }
        (void)tk_dly_tsk(VC_PERIOD_MS);
    }
}

ER vehicle_controller_start (void)
{
    ID const task_id = app_start_task((FP)vehicle_controller_task,
                                      VC_PRIORITY, TP_PRC1,
                                      "vehicle_controller_task");
    return (task_id < E_OK) ? task_id : E_OK;
}
