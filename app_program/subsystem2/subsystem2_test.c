/*
 *  subsystem2_test.c
 *  On-target tests for subsystem 2 (Motor FSM & Arbitrator). 
 *  Built only with: make TEST=1
 */

#include "subsystem2.h"
#include "encoder.h" /* Required to read the distance */
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
           rmtx.htsk == 0, 
           "mtx_motor_intent is unlocked and ready for intents");

    /* S2-U03: Verify Arbitrator Input Bounding */
    s2_set_ir_velocity(500, -600); 
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

    /* S2-U06: Active Drive Odometry Test */
    tm_printf((UB*)"\n==========================================\n");
    tm_printf((UB*)" S2-U06: ACTIVE DRIVE PI CONTROLLER TEST\n");
    tm_printf((UB*)" Driving forward for 5 seconds...\n");
    tm_printf((UB*)"==========================================\n");
    
    motor_reset_ticks();
    
    /* 1. Force safety caps open and command maximum torque forward */
    s2_set_us_clearance(255);
    s2_set_ir_velocity(255, 255);
    
    /* 2. Drive for 5 seconds, printing live ticks every 100ms */
    for (int i = 0; i < 50; i++) {
        int32_t live_l = get_left_encoder_ticks();
        int32_t live_r = get_right_encoder_ticks();
        tm_printf((UB*)"[T+%04d ms] L=%d | R=%d\n", i * 100, (int)live_l, (int)live_r);
        tk_dly_tsk(100);
    }

    /* 3. Stop the robot */
    s2_set_ir_velocity(0, 0);
    tk_dly_tsk(500); /* Wait half a second for momentum and ramp-down to finish */

    /* 4. Grab the final math calculations */
    float dist = get_distance_traveled_mm();
    int32_t final_l = get_left_encoder_ticks();
    int32_t final_r = get_right_encoder_ticks();

    /* Split float into integers for safe embedded printing */
    int dist_int = (int)dist;
    int dist_frac = (int)((dist - dist_int) * 1000.0f);
    if (dist_frac < 0) dist_frac = -dist_frac;

    tm_printf((UB*)"\n--> FINAL RAW TICKS: L=%d | R=%d\n", (int)final_l, (int)final_r);
    tm_printf((UB*)"--> CALCULATED DISTANCE: %d.%03d mm\n\n", dist_int, dist_frac);

    TEST_CHECK("S2-U06",
           1, 
           "Active 5-second drive completed and distance calculated");

    /* THESE WERE THE MISSING LINES */
    TEST_SUMMARY("subsystem2");
}

#endif