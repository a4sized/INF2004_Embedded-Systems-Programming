/**
 * @file ir_time.h
 * @brief Microsecond clock for standalone sensing timing diagnostics.
 */
#ifndef IR_TIME_H
#define IR_TIME_H

#include <stdint.h>

/** @brief Low 32 bits of RP2040's free-running microsecond timer. */
uint32_t ir_time_us (void);

#endif

/* End of file. */
