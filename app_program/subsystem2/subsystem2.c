/*
 *	subsystem2.c
 *	Subsystem 2 -- Producer: creates its objects and starts its task.
 *
 *	APIs shown: tk_cre_sem, tk_cre_mpf
 */

#include <stddef.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "subsystem2.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"

EXPORT ID	sem_slots;	/* IF-02, see subsystem2.h */
EXPORT ID	mpf_samples;

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

LOCAL void encoder_callback(uint gpio, uint32_t events) {
    (void)events;
    if (gpio == LEFT_ENC_A) {
        if (gpio_get(LEFT_ENC_B)) {
            left_encoder_ticks++;
        } else {
            left_encoder_ticks--;
        }
    } else if (gpio == RIGHT_ENC_A) {
        if (gpio_get(RIGHT_ENC_B)) {
            right_encoder_ticks++;
        } else {
            right_encoder_ticks--;
        }
    }
}

EXPORT void motor_init(void) {
    // 1. Set motor pins to PWM function
    gpio_set_function(M1A_PIN, GPIO_FUNC_PWM);
    gpio_set_function(M1B_PIN, GPIO_FUNC_PWM);
    gpio_set_function(M2A_PIN, GPIO_FUNC_PWM);
    gpio_set_function(M2B_PIN, GPIO_FUNC_PWM);

    // 2. Get PWM slice numbers for each motor pin
    uint slice_m1a = pwm_gpio_to_slice_num(M1A_PIN);
    uint slice_m1b = pwm_gpio_to_slice_num(M1B_PIN);
    uint slice_m2a = pwm_gpio_to_slice_num(M2A_PIN);
    uint slice_m2b = pwm_gpio_to_slice_num(M2B_PIN);

    // 3. Set PWM wrap value to 255 (8-bit resolution)
    pwm_set_wrap(slice_m1a, 255);
    pwm_set_wrap(slice_m1b, 255);
    pwm_set_wrap(slice_m2a, 255);
    pwm_set_wrap(slice_m2b, 255);

    // 4. Enable PWM slices
    pwm_set_enabled(slice_m1a, true);
    pwm_set_enabled(slice_m1b, true);
    pwm_set_enabled(slice_m2a, true);
    pwm_set_enabled(slice_m2b, true);

    // Initial stop state
    pwm_set_gpio_level(M1A_PIN, 0);
    pwm_set_gpio_level(M1B_PIN, 0);
    pwm_set_gpio_level(M2A_PIN, 0);
    pwm_set_gpio_level(M2B_PIN, 0);

    // 5. Configure encoder inputs with internal pull-ups
    gpio_init(LEFT_ENC_A);  gpio_set_dir(LEFT_ENC_A, GPIO_IN);  gpio_pull_up(LEFT_ENC_A);
    gpio_init(LEFT_ENC_B);  gpio_set_dir(LEFT_ENC_B, GPIO_IN);  gpio_pull_up(LEFT_ENC_B);
    gpio_init(RIGHT_ENC_A); gpio_set_dir(RIGHT_ENC_A, GPIO_IN); gpio_pull_up(RIGHT_ENC_A);
    gpio_init(RIGHT_ENC_B); gpio_set_dir(RIGHT_ENC_B, GPIO_IN); gpio_pull_up(RIGHT_ENC_B);

    // Attach GPIO interrupt callback for Channel A pins
    gpio_set_irq_enabled_with_callback(LEFT_ENC_A, GPIO_IRQ_EDGE_RISE, true, &encoder_callback);
    gpio_set_irq_enabled(RIGHT_ENC_A, GPIO_IRQ_EDGE_RISE, true);

    motor_reset_ticks();
    current_state = MOTOR_STOPPED;
}

EXPORT void motor_speed(int16_t left_speed, int16_t right_speed){ // initialize motor speed function
    
    // clamp values
    if (left_speed > 255) left_speed = 255;
    if (left_speed < -255) left_speed = -255;
    if (right_speed > 255) right_speed = 255;
    if (right_speed < -255) right_speed = -255;

    // left motor
    if (left_speed > 0){
        pwm_set_gpio_level(M1B_PIN, 0);
        pwm_set_gpio_level(M1A_PIN, left_speed);
    } else if (left_speed < 0){
        pwm_set_gpio_level(M1A_PIN, 0);
        pwm_set_gpio_level(M1B_PIN, -left_speed);
    } else{
        pwm_set_gpio_level(M1A_PIN, 0);
        pwm_set_gpio_level(M1B_PIN, 0);
    }

    // right motor
    if (right_speed > 0){
        pwm_set_gpio_level(M2B_PIN, 0);
        pwm_set_gpio_level(M2A_PIN, right_speed);
    } else if (right_speed < 0){
        pwm_set_gpio_level(M2A_PIN, 0);
        pwm_set_gpio_level(M2B_PIN, -right_speed);
    } else{
        pwm_set_gpio_level(M2A_PIN, 0);
        pwm_set_gpio_level(M2B_PIN, 0);
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



// state 1
LOCAL void handle_motor_ramping(int16_t left_speed, int16_t right_speed){

    active_left_speed = ramp_step(active_left_speed, left_speed, 10);
    active_right_speed = ramp_step(active_right_speed, right_speed, 10);
   
    motor_speed(active_left_speed, active_right_speed);
    if (active_left_speed == left_speed && active_right_speed == right_speed){
        current_state = MOTOR_STEADY_DRIVE;
    }
}

// state 2
LOCAL void handle_motor_steady_drive(int16_t left_speed, int16_t right_speed){

    bool left_reversing = (left_speed > 0 && active_left_speed < 0) || (left_speed < 0 && active_left_speed > 0);
    bool right_reversing = (right_speed > 0 && active_right_speed < 0) || (right_speed < 0 && active_right_speed > 0);

    if (left_reversing || right_reversing){
        reverse_delay_counter = 0; // reset counter when a new reverse command is invoked
        current_state = MOTOR_REVERSING;
        return;
    }

    if (left_speed == 0 && right_speed == 0){
        current_state = MOTOR_RAMPING; // ramp down to 0
        return;
    }
    
    active_left_speed = left_speed;
    active_right_speed = right_speed;
    motor_speed(active_left_speed, active_right_speed);
}

// state 3
LOCAL void handle_motor_reversing(int16_t left_speed, int16_t right_speed){

	(void)left_speed;
	(void)right_speed; 
    active_left_speed = 0;
    active_right_speed = 0;
    motor_speed(0, 0);

    reverse_delay_counter++;
    if (reverse_delay_counter >= 5){
        reverse_delay_counter = 0;
        current_state = MOTOR_RAMPING; // ramp to new speed
    }
}

// state 4
LOCAL void handle_motor_safety_halt(int16_t left_speed, int16_t right_speed){

    active_left_speed = 0;
    active_right_speed = 0;
    motor_speed(0, 0);

    if (!HAS_ANY_FAULT(active_fault_mask) && left_speed == 0 && right_speed == 0){
        current_state = MOTOR_STOPPED;
    }
}



// main state machine update loop
EXPORT void motion_fsm_update(int16_t left_speed, int16_t right_speed){

    if (HAS_ANY_FAULT(active_fault_mask)){
        current_state = MOTOR_SAFETY_HALT;
    }

    switch (current_state){

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

EXPORT MotionState_t motion_fsm_get_state(void){
    return current_state;
}



EXPORT ER subsystem2_init(void)
{
	T_CSEM	csem = {0};
	T_CMPF	cmpf = {0};

	motor_init();
	/* Semaphore: a counter of free "slots".  The producer takes one
	   (tk_wai_sem) before each sample; the consumer gives one back
	   (tk_sig_sem).  It never exceeds the number of memory blocks, so
	   the producer can never run out of memory. */
	csem.sematr  = TA_TFIFO;	/* waiting tasks queue in FIFO order */
	csem.isemcnt = S2_POOL_COUNT;	/* initial count */
	csem.maxsem  = S2_POOL_COUNT;	/* maximum count */
	sem_slots = tk_cre_sem(&csem);
	if (sem_slots < E_OK) {
		return sem_slots;
	}

	/* Fixed-size memory pool: S2_POOL_COUNT blocks, each exactly one
	   SAMPLE_MSG.  Faster and more predictable than malloc(). */
	cmpf.mpfatr = TA_TFIFO | TA_RNG3;
	cmpf.mpfcnt = S2_POOL_COUNT;		/* number of blocks */
	cmpf.blfsz  = sizeof(SAMPLE_MSG);	/* size of one block */
	mpf_samples = tk_cre_mpf(&cmpf);
	return (mpf_samples < E_OK) ? mpf_samples : E_OK;
}

EXPORT ER subsystem2_start(void)
{
	/* Pinned to core 1 (TP_PRC2): samples cross to subsystem 3 on core 0 */
	ID tid = app_start_task(producer_task, S2_PRI_PRODUCER, TP_PRC2,
				"producer_task");
	return (tid < E_OK) ? tid : E_OK;
}
