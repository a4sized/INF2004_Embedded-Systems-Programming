/** @file ir_fault_status.h @brief Local IR failure evidence for later system integration. */
#ifndef IR_FAULT_STATUS_H
#define IR_FAULT_STATUS_H

#include <stdbool.h>
#include <stdint.h>

#define IR_FAULT_SAMPLE_GAP_LIMIT_US 10000u

typedef struct
{
    bool b_gpio_configured;
    bool b_gpio_init_failed;
    bool b_have_sample;
    uint32_t last_sample_us;
    uint32_t gaps_over_limit;
} ir_fault_status_t;

/** Reset only as part of an accepted subsystem reset/reinitialization. */
void ir_fault_status_reset (ir_fault_status_t * p_status);

/** A failed initialization remains latched until reset. */
void ir_fault_status_note_gpio_init (ir_fault_status_t * p_status,
                                     bool b_success);

/** Record a sample and count gaps above the NFR9 10 ms limit. */
void ir_fault_status_note_sample (ir_fault_status_t * p_status,
                                  uint32_t sample_us);

#endif
