/** @file motor_force_test.h @brief One-shot USB motor diagnostic. */
#ifndef MOTOR_FORCE_TEST_H
#define MOTOR_FORCE_TEST_H

#if defined(BUDDY3_FORCE_MOTOR_TEST)
#include <tk/tkernel.h>

ER motor_force_test_start (void);
#endif

#endif
