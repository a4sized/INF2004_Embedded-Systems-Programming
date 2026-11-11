#include "vehicle_controller.h"

#include <assert.h>

#include "systemFaults.h"

int main (void)
{
    vc_inputs_t inputs = {0};
    vc_state_t state = VC_INIT;

    inputs.b_start = true;
    assert(VC_INIT == vehicle_controller_step(state, &inputs));

    inputs.b_imu_ready = true;
    inputs.b_ultrasonic_ready = true;
    inputs.b_ir_ready = true;
    inputs.b_wifi_connected = true;
    inputs.fault_mask = FAULT_WIFI;
    assert(VC_INIT == vehicle_controller_step(state, &inputs));
    inputs.fault_mask = 0u;
    state = vehicle_controller_step(state, &inputs);
    assert(VC_READY == state); /* START during INIT is not remembered. */

    inputs.b_start = false;
    assert(VC_READY == vehicle_controller_step(state, &inputs));
    inputs.b_start = true;
    state = vehicle_controller_step(state, &inputs);
    assert(VC_FOLLOWING == state);

    inputs.b_wifi_connected = false;
    inputs.b_start = false;
    assert(VC_FOLLOWING == vehicle_controller_step(state, &inputs));
    inputs.fault_mask = FAULT_WIFI;
    assert(VC_FOLLOWING == vehicle_controller_step(state, &inputs));

    inputs.fault_mask = FAULT_US_SENS;
    assert(VC_STOPPED == vehicle_controller_step(state, &inputs));
    inputs.fault_mask = FAULT_WIFI;

    inputs.b_ir_ready = false;
    state = vehicle_controller_step(state, &inputs);
    assert(VC_STOPPED == state);
    inputs.b_ir_ready = true;
    assert(VC_STOPPED == vehicle_controller_step(state, &inputs));
    inputs.b_clear = true;
    state = vehicle_controller_step(state, &inputs);
    assert(VC_INIT == state);

    inputs.b_clear = false;
    inputs.b_wifi_connected = true;
    inputs.fault_mask = 0u;
    state = vehicle_controller_step(state, &inputs);
    assert(VC_READY == state);
    inputs.b_start = true;
    inputs.b_stop = true;
    state = vehicle_controller_step(state, &inputs);
    assert(VC_STOPPED == state); /* STOP wins over START. */

    inputs.b_stop = false;
    inputs.b_estop = true;
    state = vehicle_controller_step(state, &inputs);
    assert(VC_FAULT == state);
    inputs.b_estop = false;
    assert(VC_FAULT == vehicle_controller_step(state, &inputs));
    inputs.b_clear = true;
    inputs.fault_mask = FAULT_MOTOR;
    assert(VC_FAULT == vehicle_controller_step(state, &inputs));
    inputs.fault_mask = 0u;
    assert(VC_INIT == vehicle_controller_step(state, &inputs));

    inputs = (vc_inputs_t){
        .b_imu_ready = true,
        .b_ultrasonic_ready = true,
        .b_ir_ready = true,
        .b_wifi_connected = true,
    };
    state = VC_FOLLOWING;
    inputs.b_avoiding = true;
    state = vehicle_controller_step(state, &inputs);
    assert(VC_AVOIDING == state);
    /* While S5 owns bypass driving, b_ultrasonic_ready may drop false without stopping. */
    inputs.b_ultrasonic_ready = false;
    state = vehicle_controller_step(state, &inputs);
    assert(VC_AVOIDING == state);
    /* Returning from bypass resumes FOLLOWING once ultrasonic ready is restored. */
    inputs.b_avoiding = false;
    inputs.b_ultrasonic_ready = true;
    state = vehicle_controller_step(state, &inputs);
    assert(VC_FOLLOWING == state);
    /* Outside AVOIDING, losing ultrasonic readiness still stops the car. */
    inputs.b_ultrasonic_ready = false;
    assert(VC_STOPPED == vehicle_controller_step(state, &inputs));
    inputs.b_ultrasonic_ready = true;

    state = VC_AVOIDING;
    inputs.b_avoiding = true;
    inputs.fault_mask = FAULT_US_SENS;
    assert(VC_STOPPED == vehicle_controller_step(state, &inputs));
    inputs.fault_mask = 0u;
    inputs.b_imu_ready = false;
    assert(VC_STOPPED == vehicle_controller_step(state, &inputs));
    inputs.b_imu_ready = true;
    inputs.b_stop = true;
    assert(VC_STOPPED == vehicle_controller_step(state, &inputs));
    inputs.b_stop = false;

    /* FAULT_IR_NAV is Tier-1 fatal and blocks CLEAR until cleared. */
    inputs.fault_mask = FAULT_IR_NAV;
    state = vehicle_controller_step(VC_AVOIDING, &inputs);
    assert(VC_FAULT == state);
    inputs.b_avoiding = false;
    inputs.b_clear = true;
    assert(VC_FAULT == vehicle_controller_step(state, &inputs));
    inputs.fault_mask = 0u;
    assert(VC_INIT == vehicle_controller_step(state, &inputs));

    assert(VC_FAULT == vehicle_controller_step(VC_INIT, NULL));
    assert(VC_FAULT == vehicle_controller_step((vc_state_t)99, &inputs));
    return 0;
}
