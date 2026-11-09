/**
 * @file    s5_log.h
 * @brief   Subsystem 5 console and CSV-log output, switched off for the
 *          clearance test (S5_CLEARANCE_TEST in s5_config.h).
 *
 * Every ordinary Subsystem 5 console line goes through S5_LOG() and every
 * CSV event through s5_log_event(). With S5_CLEARANCE_TEST = 1 both are
 * silent, so only the clearance and bypass decision lines (printed with
 * tm_printf() directly in s5_profile.c and s5_avoid.c) reach the console.
 */
#ifndef S5_LOG_H
#define S5_LOG_H

#include <tm/tmonitor.h>
#include "s5_config.h"

/*
 * Ordinary console line, used exactly like tm_printf(). Printed only while
 * S5_CLEARANCE_TEST is 0; the condition is a constant, so the compiler drops
 * the call when the test is on, but still checks the arguments.
 * A macro, not a function (Barr-C 6.3.a): T-Monitor has no vprintf-style
 * call, so a function could not pass a variable argument list on to
 * tm_printf(). The arguments are not put in parentheses (6.3.b.ii):
 * (__VA_ARGS__) would turn them into one comma expression. Each argument is
 * used once and the body has no transfer of control.
 */
#define S5_LOG(...)     ((0u == S5_CLEARANCE_TEST) ? \
                         (void)tm_printf(__VA_ARGS__) : (void)0)

/* Ordinary CSV event: passed to Subsystem 1's s1_csv_log_event() (which
 * prints on the console) only while S5_CLEARANCE_TEST is 0. */
void s5_log_event (const char * const p_event_type,
                   const char * const p_details);

#endif /* S5_LOG_H */
