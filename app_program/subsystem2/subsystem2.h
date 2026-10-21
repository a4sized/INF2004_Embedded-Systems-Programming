/*
 *	subsystem2.h
 *	Subsystem 2 -- Producer (sample source).  Student ID: 2501820
 *
 *	Public interface of subsystem 2.  Other subsystems include it as
 *	"../subsystem2/subsystem2.h".
 */

#ifndef SUBSYSTEM2_H
#define SUBSYSTEM2_H

#include "../app.h"
#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>
#include "systemFaults.h"

#define S2_PRI_PRODUCER		14	/* priority band 14-15 (TEAM.md) */
#define S2_PERIOD_MS		300	/* one sample every 300 ms */




/* ---------------------------------------------------------------------
 *  IF-01 data format: one sample, sent to subsystem 3's mailbox.
 *  A mailbox message MUST start with a T_MSG header: the kernel links
 *  messages together through it, so the data itself is never copied.
 * --------------------------------------------------------------------- */
typedef struct {
	T_MSG	hdr;		/* used by the kernel, do not touch */
	UW	seq;		/* sample sequence number */
	UW	time_ms;	/* system time when the sample was taken */
	INT	value;		/* the "measurement", 0..100 */
	INT	prc;		/* processor (core) that produced it: 1 or 2 */
} SAMPLE_MSG;

#define S2_POOL_COUNT	4	/* at most 4 samples in flight */

/* IF-02: the receiver of a sample gives its memory block back to
   mpf_samples (tk_rel_mpf) and then a slot back to sem_slots
   (tk_sig_sem, count 1).  Nothing else may use these two objects. */
IMPORT ID	sem_slots;
IMPORT ID	mpf_samples;

/* Private to subsystem 2 */
IMPORT void	producer_task(INT stacd, void *exinf);
IMPORT INT	s2_make_value(UW seq);






// declaring motor pins
#define M1A_PIN 8  // Left forward
#define M1B_PIN 9  // Left backward
#define M2A_PIN 10 // Right forward
#define M2B_PIN 11 // Right backward

// declaring encoder pins 
#define LEFT_ENC_A  14  // Left encoder A
#define LEFT_ENC_B  15  // Left encoder B
#define RIGHT_ENC_A 2   // Right encoder A
#define RIGHT_ENC_B 3   // Right encoder B  

extern volatile int32_t left_encoder_ticks;
extern volatile int32_t right_encoder_ticks;


typedef enum{ // motion states indicated by numbers

    MOTOR_STOPPED = 0,
    MOTOR_RAMPING = 1,
    MOTOR_STEADY_DRIVE = 2,
    MOTOR_REVERSING = 3,
    MOTOR_SAFETY_HALT = 4
} MotionState_t;

void motor_init(void);
void motor_speed(int16_t left_speed, int16_t right_speed);
//void motor_duration(int16_t left_speed, int16_t right_speed, uint32_t duration_ms);
void motor_reset_ticks(void);
void motion_fsm_update(int16_t left_speed, int16_t right_speed);
void motion_trigger_safety_halt(SystemFault_t cause);
MotionState_t motion_fsm_get_state(void);

int32_t get_left_encoder_ticks(void);
int32_t get_right_encoder_ticks(void);


#endif /* SUBSYSTEM2_H */
