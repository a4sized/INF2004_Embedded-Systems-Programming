/** @file vehicle_controller_policy.c @brief Testable mission-start policy. */
#include "vehicle_controller.h"

#include "systemFaults.h"

vc_state_t vehicle_controller_step (vc_state_t state,
                                    vc_inputs_t const * p_inputs)
{
    if ((void *)0 == p_inputs)
    {
        return VC_FAULT;
    }

    if ((state < VC_INIT) || (state > VC_FAULT))
    {
        return VC_FAULT;
    }

    if (p_inputs->b_estop ||
        (0u != (p_inputs->fault_mask & TIER1_FATAL_MASK)))
    {
        return VC_FAULT;
    }

    if (VC_FAULT == state)
    {
        return p_inputs->b_clear ? VC_INIT : VC_FAULT;
    }

    if (p_inputs->b_stop)
    {
        return VC_STOPPED;
    }

    if (VC_STOPPED == state)
    {
        return p_inputs->b_clear ? VC_INIT : VC_STOPPED;
    }

    if (VC_FOLLOWING == state)
    {
        /* Loss of an onboard sensor revokes movement. Link loss does not. */
        if (!p_inputs->b_imu_ready || !p_inputs->b_ultrasonic_ready ||
            !p_inputs->b_ir_ready ||
            (0u != (p_inputs->fault_mask & FAULT_US_SENS)))
        {
            return VC_STOPPED;
        }
        return VC_FOLLOWING;
    }

    bool const b_startup_ready = p_inputs->b_imu_ready &&
        p_inputs->b_ultrasonic_ready && p_inputs->b_ir_ready &&
        p_inputs->b_wifi_connected &&
        (0u == (p_inputs->fault_mask &
                (FAULT_US_SENS | FAULT_WIFI)));

    if (!b_startup_ready)
    {
        return VC_INIT;
    }
    if ((VC_READY == state) && p_inputs->b_start)
    {
        return VC_FOLLOWING;
    }
    return VC_READY;
}
