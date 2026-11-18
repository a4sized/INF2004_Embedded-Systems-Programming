/**
 * @file ir_sensing_main.h
 * @brief IR sensing service and isolated firmware entry point.
 */
#ifndef IR_SENSING_MAIN_H
#define IR_SENSING_MAIN_H

#include <tk/tkernel.h>

ER ir_sensing_service_init (void);
ER ir_sensing_service_start (void);

#if defined(BUDDY3_SENSING_STANDALONE)
INT usermain (void);
#endif

#endif

/* End of file. */
