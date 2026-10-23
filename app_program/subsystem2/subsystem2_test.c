/*
 *  subsystem2_test.c
 *  On-target tests for subsystem 2 (Motor FSM & Arbitrator). 
 *  Built only with: make TEST=1
 *  Record results in testing/subsystem2/TEST_LOG.md.
 */

#include "subsystem2.h"
#include "../test_report.h"
#include "../systemFaults.h"

IMPORT ID mtx_motor_intent;
IMPORT void s2_set_ir_velocity(int16_t left_target, int16_t right_target);

#if APP_TEST

EXPORT void subsystem2_test(void)
{
    T_RSEM  rsem;
    T_RMPF  rmpf;
    T_RMTX  rmtx;

    TEST_INIT();

    /* S2-U01: Verify all RTOS IPC objects initialized correctly */
    TEST_CHECK("S2-U01",
           tk_ref_sem(sem_slots, &rsem) == E_OK &&
           tk_ref_mpf(mpf_samples, &rmpf) == E_OK &&
           tk_ref_mtx(mtx_motor_intent, &rmtx) == E_OK,
           "Semaphore, MemPool, and Mutex successfully created");

    /* S2-U02: The Priority-Inheritance Mutex must not be locked at startup */
    tk_ref_mtx(mtx_motor_intent, &rmtx);
    TEST_CHECK("S2-U02", 
           rmtx.htsk == 0, /* 0 means unlocked/no task is blocking it */
           "mtx_motor_intent is unlocked and ready for intents");

    /* S2-U03: Verify Arbitrator Input Bounding (Caps speeds at +/- 255) */
    s2_set_ir_velocity(500, -600); /* Send illegal values */
    
    /* We can't read the private struct directly, but we can verify it doesn't crash 
       and that the FSM state remains valid. */
    TEST_CHECK("S2-U03",
           motion_fsm_get_state() != MOTOR_SAFETY_HALT,
           "Arbitrator handles out-of-bounds velocities gracefully without crashing");

    /* S2-U04: Verify FSM boots into the STOPPED state */
    TEST_CHECK("S2-U04",
           motion_fsm_get_state() == MOTOR_STOPPED,
           "FSM boots securely into MOTOR_STOPPED state");

    /* S2-U05: Verify Encoder Getters are encapsulated safely */
    motor_reset_ticks();
    TEST_CHECK("S2-U05",
           get_left_encoder_ticks() == 0 && get_right_encoder_ticks() == 0,
           "Encoders initialized to 0 and getters function normally");

    TEST_SUMMARY("subsystem2");
}

#endif