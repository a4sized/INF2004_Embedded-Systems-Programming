/**
 * @file    s5_log.h
 * @brief   Subsystem 5 console and CSV-log output.
 *
 * By default Subsystem 5 prints only these, with tm_printf() directly:
 *   - the distance straight ahead, every S5_FORWARD_PRINT_MS (s5_sense.c);
 *   - each fine-scan angle's distance (s5_scan.c);
 *   - the obstacle's width, and how far its left and right edges are from
 *     the car's centre line (s5_profile.c);
 *   - each bypass decision (s5_avoid.c).
 * (The vehicle controller prints the state changes.) Everything else is a
 * diagnostic, switched on with S5_VERBOSE = 1 in s5_config.h:
 *   S5_LOG()         ordinary diagnostic lines;
 *   S5_RESULT_LOG()  room to go round each side, also printed in the
 *                    clearance test (S5_CLEARANCE_TEST = 1).
 * With S5_CLEARANCE_TEST = 1 the straight-ahead distance line and S5_LOG()
 * are silent, so only the fine scan's lines and the decision are printed.
 */
#ifndef S5_LOG_H
#define S5_LOG_H

#include <tm/tmonitor.h>
#include "s5_config.h"

/*
 * Diagnostic console line, used exactly like tm_printf(). Printed only with
 * S5_VERBOSE = 1 and S5_CLEARANCE_TEST = 0; the condition is a constant, so
 * the compiler drops the call otherwise, but still checks the arguments.
 * A macro, not a function (Barr-C 6.3.a): T-Monitor has no vprintf-style
 * call, so a function could not pass a variable argument list on to
 * tm_printf(). The arguments are not put in parentheses (6.3.b.ii):
 * (__VA_ARGS__) would turn them into one comma expression. Each argument is
 * used once and the body has no transfer of control.
 */
#define S5_LOG(...)         (((0u != S5_VERBOSE) && \
                              (0u == S5_CLEARANCE_TEST)) ? \
                             (void)tm_printf(__VA_ARGS__) : (void)0)

/* Fine-scan detail line: printed with S5_VERBOSE = 1 or in the clearance
 * test. Same macro rules as S5_LOG(). */
#define S5_RESULT_LOG(...)  (((0u != S5_VERBOSE) || \
                              (0u != S5_CLEARANCE_TEST)) ? \
                             (void)tm_printf(__VA_ARGS__) : (void)0)

/* Ordinary CSV event: passed to Subsystem 1's s1_csv_log_event() (which
 * prints on the console unless APP_QUIET_CONSOLE) only while
 * S5_CLEARANCE_TEST is 0. */
void s5_log_event (const char * const p_event_type,
                   const char * const p_details);

#endif /* S5_LOG_H */
