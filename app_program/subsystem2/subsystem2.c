/*
 *  subsystem2.c
 *  Subsystem 2 -- Motion Control & Navigation Arbitrator
 */

#ifndef __size_t_defined
#define __size_t_defined
#endif

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>
#include <bsp/libbsp.h>
#include "../app.h"
#include "subsystem2.h"
#include "../systemFaults.h"
#include "buzzer.h"
#include "encoder.h"      /* wheel encoder counting (encoder.c) */
#include "../subsystem1/subsystem1.h" 

/* RTOS Synchronization & IPC Objects */
EXPORT ID   sem_slots;
EXPORT ID   mpf_samples;
EXPORT ID   mtx_motor_intent; /* Priority-Inheritance Mutex (TA_INHERIT) */

/* The encoder counters live in encoder.c (see encoder.h): an interrupt counts
 * every edge of both channels of both wheels. */

/* Minimum-Bit Packed Navigation Blackboard & FSM State */
LOCAL MotorControl_t motor_ctrl = {
    .ir_left_target        = 0,
    .ir_right_target       = 0,
    .us_speed_cap          = 255,
    .current_state         = MOTOR_STOPPED,
    .reverse_delay_counter = 0,
    .active_left_speed     = 0,
    .active_right_speed    = 0
};

LOCAL int32_t prev_left_ticks = 0;
LOCAL int32_t prev_right_ticks = 0;
LOCAL int32_t current_left_velocity = 0;
LOCAL int32_t current_right_velocity = 0;
LOCAL int32_t sync_error_integral = 0;

/* --------------------------------------------------------------------------
 * PUBLIC ARBITRATOR SETTERS (Thread-Safe via Priority-Inheritance Mutex)
 * -------------------------------------------------------------------------- */
EXPORT void s2_set_ir_velocity(int16_t left_target, int16_t right_target) {
    if (left_target > 255)   left_target = 255;
    if (left_target < -255)  left_target = -255;
    if (right_target > 255)  right_target = 255;
    if (right_target < -255) right_target = -255;

    tk_loc_mtx(mtx_motor_intent, TMO_FEVR);
    motor_ctrl.ir_left_target  = left_target;
    motor_ctrl.ir_right_target = right_target;
    tk_unl_mtx(mtx_motor_intent);
}

EXPORT void s2_set_us_clearance(uint8_t max_speed) {
    tk_loc_mtx(mtx_motor_intent, TMO_FEVR);
    motor_ctrl.us_speed_cap = max_speed;
    tk_unl_mtx(mtx_motor_intent);
}

/* --------------------------------------------------------------------------
 * PUBLIC ODOMETRY GETTERS & RESET
 * -------------------------------------------------------------------------- */
EXPORT void motor_reset_ticks(void) {
    encoder_reset();
    prev_left_ticks = 0;
    prev_right_ticks = 0;
    current_left_velocity = 0;
    current_right_velocity = 0;
    sync_error_integral = 0;
}

EXPORT int32_t get_left_encoder_ticks(void) {
    return encoder_get_left();
}

EXPORT int32_t get_right_encoder_ticks(void) {
    return encoder_get_right();
}

/* --------------------------------------------------------------------------
 * PRIVATE HARDWARE DRIVERS (Encapsulated inside Subsystem 2)
 * -------------------------------------------------------------------------- */
LOCAL void motor_init(void) {
    pwm_set_pin(M1A_PIN);
    pwm_set_pin(M1B_PIN);
    pwm_set_pin(M2A_PIN);
    pwm_set_pin(M2B_PIN);

    pwm_set_wrap(M1A_PIN, 255);
    pwm_set_wrap(M1B_PIN, 255);
    pwm_set_wrap(M2A_PIN, 255);
    pwm_set_wrap(M2B_PIN, 255);

    pwm_set_cc(M1A_PIN, 0);
    pwm_set_cc(M1B_PIN, 0);
    pwm_set_cc(M2A_PIN, 0);
    pwm_set_cc(M2B_PIN, 0);

    pwm_set_enabled(M1A_PIN, TRUE);
    pwm_set_enabled(M1B_PIN, TRUE);
    pwm_set_enabled(M2A_PIN, TRUE);
    pwm_set_enabled(M2B_PIN, TRUE);

    gpio_set_pin(LEFT_ENC_A, GPIO_MODE_IN);
    gpio_set_pin(LEFT_ENC_B, GPIO_MODE_IN);
    gpio_set_pin(RIGHT_ENC_A, GPIO_MODE_IN);
    gpio_set_pin(RIGHT_ENC_B, GPIO_MODE_IN);

    motor_reset_ticks();

    /* Start counting: the interrupt on all four encoder pins (encoder.c). */
    {
        ER enc_err = encoder_init();
        if (enc_err != E_OK) {
            tm_printf((UB*)"[S2] encoder_init failed: %d\n", (int)enc_err);
        }
    }

    motor_ctrl.current_state = MOTOR_STOPPED;
}

LOCAL void motor_speed(int16_t left_speed, int16_t right_speed) {
    if (left_speed > 255)   left_speed = 255;
    if (left_speed < -255)  left_speed = -255;
    if (right_speed > 255)  right_speed = 255;
    if (right_speed < -255) right_speed = -255;

    if (left_speed > 0) {
        pwm_set_cc(M1B_PIN, 0);
        pwm_set_cc(M1A_PIN, left_speed);
    } else if (left_speed < 0) {
        pwm_set_cc(M1A_PIN, 0);
        pwm_set_cc(M1B_PIN, -left_speed);
    } else {
        pwm_set_cc(M1A_PIN, 0);
        pwm_set_cc(M1B_PIN, 0);
    }

    if (right_speed > 0) {
        pwm_set_cc(M2B_PIN, 0);
        pwm_set_cc(M2A_PIN, right_speed);
    } else if (right_speed < 0) {
        pwm_set_cc(M2A_PIN, 0);
        pwm_set_cc(M2B_PIN, -right_speed);
    } else {
        pwm_set_cc(M2A_PIN, 0);
        pwm_set_cc(M2B_PIN, 0);
    }
}

/* --------------------------------------------------------------------------
 * SAFETY & FSM STATE HANDLERS
 * -------------------------------------------------------------------------- */
LOCAL bool has_active_fault(void) {
    S1_TELEMETRY_PKT t;
    s1_get_telemetry(&t); /* Safely grabs the persistent, latched mask via Mutex */
    
    /* If Subsystem 1 says there is a FATAL or HAZARD fault, we halt. */
    /* If Subsystem 1 clears the fault via MQTT, this automatically becomes false! */
    return ((t.fault_mask & (TIER1_FATAL_MASK | TIER2_HAZARD_MASK)) != 0U);
}

EXPORT void motion_trigger_safety_halt(UINT fault_bit) {
    s1_report_fault(fault_bit);
    tk_loc_mtx(mtx_motor_intent, TMO_FEVR);
    motor_ctrl.current_state      = MOTOR_SAFETY_HALT;
    motor_ctrl.active_left_speed  = 0;
    motor_ctrl.active_right_speed = 0;
    tk_unl_mtx(mtx_motor_intent);
    motor_speed(0, 0);
}


LOCAL int16_t ramp_step(int16_t current, int16_t target, int16_t step_size) {
    if (current < target) {
        current += step_size;
        if (current > target) {
            current = target;
        }
    } else if (current > target) {
        current -= step_size;
        if (current < target) {
            current = target;
        }
    }
    return current;
}

LOCAL void handle_motor_stopped(int16_t left_speed, int16_t right_speed) {
    motor_ctrl.active_left_speed  = 0;
    motor_ctrl.active_right_speed = 0;
    motor_speed(0, 0);

    if (left_speed != 0 || right_speed != 0) {
        motor_ctrl.current_state = MOTOR_RAMPING;
    }
}

LOCAL void handle_motor_ramping(int16_t left_speed, int16_t right_speed) {
    motor_ctrl.active_left_speed  = ramp_step(motor_ctrl.active_left_speed, left_speed, 10);
    motor_ctrl.active_right_speed = ramp_step(motor_ctrl.active_right_speed, right_speed, 10);

    motor_speed(motor_ctrl.active_left_speed, motor_ctrl.active_right_speed);
    if (motor_ctrl.active_left_speed == left_speed &&
        motor_ctrl.active_right_speed == right_speed) {
        motor_ctrl.current_state = MOTOR_STEADY_DRIVE;
    }
}

LOCAL void handle_motor_steady_drive(int16_t left_speed, int16_t right_speed) {
    bool left_reversing  = (left_speed > 0 && motor_ctrl.active_left_speed < 0) ||
                           (left_speed < 0 && motor_ctrl.active_left_speed > 0);
    bool right_reversing = (right_speed > 0 && motor_ctrl.active_right_speed < 0) ||
                           (right_speed < 0 && motor_ctrl.active_right_speed > 0);

    if (left_reversing || right_reversing) {
        motor_ctrl.reverse_delay_counter = 0;
        motor_ctrl.current_state         = MOTOR_REVERSING;
        return;
    }

    if (left_speed == 0 && right_speed == 0) {
        motor_ctrl.current_state = MOTOR_RAMPING;
        return;
    }

    /* 1. Update the FSM state trackers FIRST */
    motor_ctrl.active_left_speed  = left_speed;
    motor_ctrl.active_right_speed = right_speed;

    /* 2. Apply the physical motor speeds SECOND */
    /* NEW: Closed-Loop Straight Line Synchronization (PI Controller) */
    if (left_speed == right_speed) {
        /* Calculate speed difference in this 20ms window */
        int32_t speed_error = current_left_velocity - current_right_velocity;
        
        /* Accumulate the total lost ticks over time (Integral) */
        sync_error_integral += speed_error;

        /* PI-Controller: Kp = 2 (Instant speed), Ki = 1 (Total accumulated drift) */
        int16_t correction = (int16_t)(speed_error * 2) + (int16_t)(sync_error_integral);

        /* Anti-windup cap: Prevent the correction from violently jerking the robot */
        if (correction > 80) correction = 80;
        if (correction < -80) correction = -80;

        int16_t adj_left  = left_speed - correction;
        int16_t adj_right = right_speed + correction;

        motor_speed(adj_left, adj_right);
    } else {
        /* Reset drift memory immediately if the robot is commanded to steer/turn */
        sync_error_integral = 0; 
        
        /* Open-loop turning (Targets are deliberately unequal) */
        motor_speed(motor_ctrl.active_left_speed, motor_ctrl.active_right_speed);
    }
    
    /* REMOVED the redundant motor_speed() call that was down here */
}

LOCAL void handle_motor_reversing(int16_t left_speed, int16_t right_speed) {
    (void)left_speed;
    (void)right_speed;
    motor_ctrl.active_left_speed  = 0;
    motor_ctrl.active_right_speed = 0;
    motor_speed(0, 0);

    motor_ctrl.reverse_delay_counter++;
    if (motor_ctrl.reverse_delay_counter >= 5U) {
        motor_ctrl.reverse_delay_counter = 0;
        motor_ctrl.current_state         = MOTOR_RAMPING;
    }
}

LOCAL void handle_motor_safety_halt(int16_t left_speed, int16_t right_speed, bool fault_active) {
    motor_ctrl.active_left_speed  = 0;
    motor_ctrl.active_right_speed = 0;
    motor_speed(0, 0);

    if (!fault_active && left_speed == 0 && right_speed == 0) {
        motor_ctrl.current_state = MOTOR_STOPPED;
    }
}

LOCAL void motion_fsm_update(void) {

    int32_t live_l = get_left_encoder_ticks();
    int32_t live_r = get_right_encoder_ticks();
    current_left_velocity  = live_l - prev_left_ticks;
    current_right_velocity = live_r - prev_right_ticks;
    prev_left_ticks  = live_l;
    prev_right_ticks = live_r;

    bool fault_active = has_active_fault();

    tk_loc_mtx(mtx_motor_intent, TMO_FEVR);

    /* 1. Read packed target speeds and apply Ultrasonic speed cap */
    int16_t left_speed  = (int16_t)motor_ctrl.ir_left_target;
    int16_t right_speed = (int16_t)motor_ctrl.ir_right_target;
    int16_t cap         = (int16_t)motor_ctrl.us_speed_cap;

    if (left_speed > cap)   left_speed = cap;
    if (left_speed < -cap)  left_speed = -cap;
    if (right_speed > cap)  right_speed = cap;
    if (right_speed < -cap) right_speed = -cap;

    /* 2. Master safety override */
    if (fault_active) {
        motor_ctrl.current_state = MOTOR_SAFETY_HALT;
    }

    /* 3. Step the FSM */
    switch ((MotionState_t)motor_ctrl.current_state) {
        case MOTOR_STOPPED:
            handle_motor_stopped(left_speed, right_speed);
            break;

        case MOTOR_RAMPING:
            handle_motor_ramping(left_speed, right_speed);
            break;

        case MOTOR_STEADY_DRIVE:
            handle_motor_steady_drive(left_speed, right_speed);
            break;

        case MOTOR_REVERSING:
            handle_motor_reversing(left_speed, right_speed);
            break;

        case MOTOR_SAFETY_HALT:
            handle_motor_safety_halt(left_speed, right_speed, fault_active);
            break;

        default:
            motor_ctrl.current_state = MOTOR_STOPPED;
            break;
    }

    tk_unl_mtx(mtx_motor_intent);
}

EXPORT MotionState_t motion_fsm_get_state(void) {
    MotionState_t state;
    tk_loc_mtx(mtx_motor_intent, TMO_FEVR);
    state = (MotionState_t)motor_ctrl.current_state;
    tk_unl_mtx(mtx_motor_intent);
    return state;
}

/* --------------------------------------------------------------------------
 * RTOS CONTROL LOOP TASK & SUBSYSTEM INIT
 * -------------------------------------------------------------------------- */
LOCAL void s2_motor_task(INT stacd, void *exinf) {
#if ENC_PRINT_MS > 0
    UINT print_n = 0;
#endif
    (void)stacd;
    (void)exinf;

    while (1) {
        motion_fsm_update();

#if ENC_PRINT_MS > 0
        /* Show the encoder counters on the serial monitor (every ENC_PRINT_MS).
         * "interrupts 0" means no edge ever arrived: check the wiring/pins.
         * "pins L=ab R=cd" are the live levels of the four channels. */
        if (++print_n >= (UINT)(ENC_PRINT_MS / S2_PERIOD_MS)) {
            print_n = 0;
            tm_printf((UB*)"[S2] encoder ticks: left %d  right %d   (interrupts %d, pins L=%d%d R=%d%d)\n",
                      (int)get_left_encoder_ticks(), (int)get_right_encoder_ticks(),
                      (int)encoder_irq_count(),
                      (int)gpio_get_val(LEFT_ENC_A),  (int)gpio_get_val(LEFT_ENC_B),
                      (int)gpio_get_val(RIGHT_ENC_A), (int)gpio_get_val(RIGHT_ENC_B));
        }
#endif

        tk_dly_tsk(S2_PERIOD_MS); /* 20ms (50Hz) deterministic control loop */
    }
}

EXPORT ER subsystem2_init(void) {
    T_CSEM  csem = {0};
    T_CMPF  cmpf = {0};
    T_CMTX  cmtx = {0};

    motor_init();
    buzzer_init();

    /* Initialize Priority-Inheritance Mutex for MotorControl_t */
    cmtx.mtxatr = TA_INHERIT;
    mtx_motor_intent = tk_cre_mtx(&cmtx);
    if (mtx_motor_intent < E_OK) {
        return mtx_motor_intent;
    }

    csem.sematr  = TA_TFIFO;
    csem.isemcnt = S2_POOL_COUNT;
    csem.maxsem  = S2_POOL_COUNT;
    sem_slots = tk_cre_sem(&csem);
    if (sem_slots < E_OK) {
        return sem_slots;
    }

    cmpf.mpfatr = TA_TFIFO | TA_RNG3;
    cmpf.mpfcnt = S2_POOL_COUNT;
    cmpf.blfsz  = sizeof(SAMPLE_MSG);
    mpf_samples = tk_cre_mpf(&cmpf);
    return (mpf_samples < E_OK) ? mpf_samples : E_OK;
}

EXPORT ER subsystem2_start(void) {
    ID tid = app_start_task(s2_motor_task, S2_PRI_MOTOR, TP_PRC2,
                "s2_motor_task");
    return (tid < E_OK) ? tid : E_OK;
}