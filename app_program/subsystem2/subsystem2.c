/*
 *  subsystem2.c
 *  Subsystem 2 -- Producer
 */

#ifndef __size_t_defined
#define __size_t_defined
#endif

#include <stdint.h>
#include <tk/tkernel.h>
#include <bsp/libbsp.h>
#include "../app.h"
#include "subsystem2.h"
#include "buzzer.h"

EXPORT ID   sem_slots;
EXPORT ID   mpf_samples;

volatile int32_t left_encoder_ticks = 0;
volatile int32_t right_encoder_ticks = 0;
LOCAL int16_t active_left_speed = 0;
LOCAL int16_t active_right_speed = 0;
LOCAL uint8_t reverse_delay_counter = 0;
LOCAL MotionState_t current_state = MOTOR_STOPPED;

EXPORT void motor_reset_ticks(void) {
    left_encoder_ticks = 0;
    right_encoder_ticks = 0;    
}

EXPORT int32_t get_left_encoder_ticks(void) {
    return left_encoder_ticks;
}

EXPORT int32_t get_right_encoder_ticks(void) {
    return right_encoder_ticks;
}

EXPORT void motor_init(void) {
    /* 1. Assign motor pins to PWM using BSP */
    pwm_set_pin(M1A_PIN);
    pwm_set_pin(M1B_PIN);
    pwm_set_pin(M2A_PIN);
    pwm_set_pin(M2B_PIN);

    /* 2. Set PWM wrap value to 255 (8-bit resolution) */
    pwm_set_wrap(M1A_PIN, 255);
    pwm_set_wrap(M1B_PIN, 255);
    pwm_set_wrap(M2A_PIN, 255);
    pwm_set_wrap(M2B_PIN, 255);

    /* 3. Initial stop state (0 duty cycle) */
    pwm_set_cc(M1A_PIN, 0);
    pwm_set_cc(M1B_PIN, 0);
    pwm_set_cc(M2A_PIN, 0);
    pwm_set_cc(M2B_PIN, 0);

    /* 4. Enable PWM channels */
    pwm_set_enabled(M1A_PIN, TRUE);
    pwm_set_enabled(M1B_PIN, TRUE);
    pwm_set_enabled(M2A_PIN, TRUE);
    pwm_set_enabled(M2B_PIN, TRUE);

    /* 5. Configure encoder inputs */
    gpio_set_pin(LEFT_ENC_A, GPIO_MODE_IN);
    gpio_set_pin(LEFT_ENC_B, GPIO_MODE_IN);
    gpio_set_pin(RIGHT_ENC_A, GPIO_MODE_IN);
    gpio_set_pin(RIGHT_ENC_B, GPIO_MODE_IN);

    /* Power up Encoder 2 power pin */
    gpio_set_pin(ENC2_PWR_PIN, GPIO_MODE_OUT);
    gpio_set_val(ENC2_PWR_PIN, 1);

    /* 
     * TODO: Standard Pico SDK interrupts clash with the RTOS. 
     * You will need to implement tk_def_int() later to read encoders.
     */

    motor_reset_ticks();
    current_state = MOTOR_STOPPED;
}

EXPORT void motor_speed(int16_t left_speed, int16_t right_speed) {
    if (left_speed > 255) left_speed = 255;
    if (left_speed < -255) left_speed = -255;
    if (right_speed > 255) right_speed = 255;
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

EXPORT void motion_trigger_safety_halt(SystemFault_t cause) {
    SET_FAULT(active_fault_mask, cause);
    current_state = MOTOR_SAFETY_HALT;
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
    active_left_speed = 0;
    active_right_speed = 0;
    motor_speed(0, 0);

    if (left_speed != 0 || right_speed != 0) {
        current_state = MOTOR_RAMPING;
    }
}

LOCAL void handle_motor_ramping(int16_t left_speed, int16_t right_speed) {
    active_left_speed = ramp_step(active_left_speed, left_speed, 10);
    active_right_speed = ramp_step(active_right_speed, right_speed, 10);
   
    motor_speed(active_left_speed, active_right_speed);
    if (active_left_speed == left_speed && active_right_speed == right_speed) {
        current_state = MOTOR_STEADY_DRIVE;
    }
}

LOCAL void handle_motor_steady_drive(int16_t left_speed, int16_t right_speed) {
    bool left_reversing = (left_speed > 0 && active_left_speed < 0) || (left_speed < 0 && active_left_speed > 0);
    bool right_reversing = (right_speed > 0 && active_right_speed < 0) || (right_speed < 0 && active_right_speed > 0);

    if (left_reversing || right_reversing) {
        reverse_delay_counter = 0;
        current_state = MOTOR_REVERSING;
        return;
    }

    if (left_speed == 0 && right_speed == 0) {
        current_state = MOTOR_RAMPING;
        return;
    }
    
    active_left_speed = left_speed;
    active_right_speed = right_speed;
    motor_speed(active_left_speed, active_right_speed);
}

LOCAL void handle_motor_reversing(int16_t left_speed, int16_t right_speed) {
    (void)left_speed;
    (void)right_speed; 
    active_left_speed = 0;
    active_right_speed = 0;
    motor_speed(0, 0);

    reverse_delay_counter++;
    if (reverse_delay_counter >= 5) {
        reverse_delay_counter = 0;
        current_state = MOTOR_RAMPING;
    }
}

LOCAL void handle_motor_safety_halt(int16_t left_speed, int16_t right_speed) {
    active_left_speed = 0;
    active_right_speed = 0;
    motor_speed(0, 0);

    if (!HAS_ANY_FAULT(active_fault_mask) && left_speed == 0 && right_speed == 0) {
        current_state = MOTOR_STOPPED;
    }
}

EXPORT void motion_fsm_update(int16_t left_speed, int16_t right_speed) {
    if (HAS_ANY_FAULT(active_fault_mask)) {
        current_state = MOTOR_SAFETY_HALT;
    }

    switch (current_state) {
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
            handle_motor_safety_halt(left_speed, right_speed);
            break;

        default:
            current_state = MOTOR_STOPPED;
            break;
    }
}

EXPORT MotionState_t motion_fsm_get_state(void) {
    return current_state;
}

EXPORT ER subsystem2_init(void)
{
    T_CSEM  csem = {0};
    T_CMPF  cmpf = {0};

    motor_init();
    buzzer_init();

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

EXPORT ER subsystem2_start(void)
{
    ID tid = app_start_task(producer_task, S2_PRI_PRODUCER, TP_PRC2,
                "producer_task");
    return (tid < E_OK) ? tid : E_OK;
}