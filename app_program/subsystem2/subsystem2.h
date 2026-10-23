/*
 *  subsystem2.h
 *  Subsystem 2 -- Motion Control & Navigation Arbitrator.
 */

#ifndef SUBSYSTEM2_H
#define SUBSYSTEM2_H

#include "../app.h"
#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>
#include "../systemFaults.h"

/* RTOS Task Configuration */
#define S2_PRI_MOTOR        14  /* Priority band 14-15 (TEAM.md) */
#define S2_PERIOD_MS        20  /* 50Hz deterministic control & ramp loop */

/* Hardware Pin Mappings (Cytron Robo Pico) */
#define M1A_PIN             8   /* Left motor forward PWM */
#define M1B_PIN             9   /* Left motor reverse PWM */
#define M2A_PIN             10  /* Right motor forward PWM */
#define M2B_PIN             11  /* Right motor reverse PWM */

#define LEFT_ENC_A          14  /* Left encoder channel A */
#define LEFT_ENC_B          15  /* Left encoder channel B */
#define RIGHT_ENC_A         2   /* Right encoder channel A */
#define RIGHT_ENC_B         3   /* Right encoder channel B */
/* Note: ENC2_PWR_PIN (16) removed to fix hardware conflict with Barcode IR */

/* ---------------------------------------------------------------------
 *  IF-01 data format: Minimum-bit packed mailbox sample message.
 * --------------------------------------------------------------------- */
typedef struct {
    T_MSG    hdr;           /* Kernel linked-list header (do not touch) */
    uint32_t seq     : 16;  /* Sample sequence number (0..65535) */
    uint32_t value   : 7;   /* Measurement range 0..100 (7 bits) */
    uint32_t prc     : 2;   /* Processor core ID 1..2 (2 bits) */
    uint32_t reserved: 7;   /* Explicit alignment to 32-bit boundary */
    UW       time_ms;       /* System timestamp in ms */
} SAMPLE_MSG;

#define S2_POOL_COUNT       4   /* Maximum 4 samples in flight */

IMPORT ID sem_slots;
IMPORT ID mpf_samples;
IMPORT ID mtx_motor_intent;     /* Priority-Inheritance Mutex (TA_INHERIT) */

/* ---------------------------------------------------------------------
 *  Motion FSM States & Minimum-Bit Packed Control Structure
 * --------------------------------------------------------------------- */
typedef enum {
    MOTOR_STOPPED      = 0,
    MOTOR_RAMPING      = 1,
    MOTOR_STEADY_DRIVE = 2,
    MOTOR_REVERSING    = 3,
    MOTOR_SAFETY_HALT  = 4
} MotionState_t;

/* 32-bit base types used so ARM GCC packs fields without boundary padding */
typedef struct {
    /* Word 1: 32 bits (4 bytes) */
    int32_t  ir_left_target        : 9;  /* Bits 0-8:   Range -255 to +255 */
    int32_t  ir_right_target       : 9;  /* Bits 9-17:  Range -255 to +255 */
    uint32_t us_speed_cap          : 8;  /* Bits 18-25: Range 0 to 255 */
    uint32_t current_state         : 3;  /* Bits 26-28: Range 0 to 4 (5 states) */
    uint32_t reverse_delay_counter : 3;  /* Bits 29-31: Range 0 to 5 ticks */

    /* Word 2: 18 active bits inside 32-bit word (4 bytes) */
    int32_t  active_left_speed     : 9;  /* Bits 0-8:   Range -255 to +255 */
    int32_t  active_right_speed    : 9;  /* Bits 9-17:  Range -255 to +255 */
} MotorControl_t;

/* ---------------------------------------------------------------------
 *  PUBLIC INTERFACES (Callable by other Subsystems)
 * --------------------------------------------------------------------- */
/* Arbitrator Setters (Mutex-Protected) */
IMPORT void s2_set_ir_velocity(int16_t left_target, int16_t right_target);
IMPORT void s2_set_us_clearance(uint8_t max_speed);

/* Safe Telemetry & Odometry Getters */
IMPORT MotionState_t motion_fsm_get_state(void);
IMPORT int32_t get_left_encoder_ticks(void);
IMPORT int32_t get_right_encoder_ticks(void);
IMPORT void motor_reset_ticks(void);

/* Emergency Override */
IMPORT void motion_trigger_safety_halt(UINT fault_bit);

/* Subsystem Lifecycle (Called by app_main.c) */
IMPORT ER subsystem2_init(void);
IMPORT ER subsystem2_start(void);

#endif /* SUBSYSTEM2_H */