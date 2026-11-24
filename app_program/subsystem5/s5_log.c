/**
 * @file    s5_log.c
 * @brief   Subsystem 5 CSV-log output, switched off for the clearance test.
 *
 * See s5_log.h.
 */
#include <tk/tkernel.h>
#include "../app.h"
#include "../subsystem1/subsystem1.h"
#include "s5_config.h"
#include "s5_log.h"

void s5_log_event (const char * const p_event_type,
                   const char * const p_details)
{
    if (0u == S5_CLEARANCE_TEST)
    {
        s1_csv_log_event(p_event_type, p_details);
    }
}
