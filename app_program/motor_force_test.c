/** @file motor_force_test.c @brief One-shot test through Subsystem 2. */
#if defined(BUDDY3_FORCE_MOTOR_TEST)

#include "motor_force_test.h"
#include "app.h"
#include "subsystem1/subsystem1.h"
#include "subsystem2/subsystem2.h"

#define MOTOR_TEST_PWM 50
#define MOTOR_TEST_WAIT_STEPS 60u
#define MOTOR_TEST_WAIT_STEP_MS 50u
#define MOTOR_TEST_PULSE_STEPS 50u
#define MOTOR_TEST_PULSE_STEP_MS 20u
#define MOTOR_TEST_TASK_PRIORITY 16

extern UW tm_usb_state (void);

static bool motor_test_usb_ready (void)
{
    return (3u == tm_usb_state());
}

static void motor_force_test_task (INT start_code, void * p_extra)
{
    (void)start_code;
    (void)p_extra;
    s2_set_ir_velocity(0, 0);
    for (;;)
    {
        if (!motor_test_usb_ready())
        {
            (void)tk_dly_tsk(MOTOR_TEST_WAIT_STEP_MS);
            continue;
        }
        s1_csv_log_event("MOTOR_TEST", "USB_READY_WAIT_3S");
        uint32_t wait_steps = 0u;
        while ((wait_steps < MOTOR_TEST_WAIT_STEPS) &&
               motor_test_usb_ready())
        {
            (void)tk_dly_tsk(MOTOR_TEST_WAIT_STEP_MS);
            ++wait_steps;
        }
        if (!motor_test_usb_ready())
        {
            continue;
        }

        s1_csv_log_event("MOTOR_TEST", "FORWARD_50_50_1S");
        s2_set_ir_velocity(MOTOR_TEST_PWM, MOTOR_TEST_PWM);
        for (uint32_t pulse_steps = 0u;
             (pulse_steps < MOTOR_TEST_PULSE_STEPS) &&
             motor_test_usb_ready(); ++pulse_steps)
        {
            (void)tk_dly_tsk(MOTOR_TEST_PULSE_STEP_MS);
        }
        s2_set_ir_velocity(0, 0);
        s1_csv_log_event("MOTOR_TEST", "STOP_0_0");
        for (;;)
        {
            (void)tk_dly_tsk(1000u);
        }
    }
}

ER motor_force_test_start (void)
{
    ID const task_id = app_start_task((FP)motor_force_test_task,
                                      MOTOR_TEST_TASK_PRIORITY, TP_PRC1,
                                      "motor_force_test_task");
    return (task_id < E_OK) ? task_id : E_OK;
}

#endif
