/**
 * @file ir_sensing_main.h
 * @brief Isolated Buddy 3 sensing firmware entry point.
 */
#ifndef IR_SENSING_MAIN_H
#define IR_SENSING_MAIN_H

#if defined(BUDDY3_SENSING_STANDALONE)
#include <tk/tkernel.h>

INT usermain (void);
#endif

#endif

/* End of file. */
