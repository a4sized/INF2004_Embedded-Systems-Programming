#ifndef IR_BARCODE_GPIO_H
#define IR_BARCODE_GPIO_H

#include "ir_barcode_capture.h"

/** Reset the task-side GP6 transition history and bounded edge queue. */
void ir_barcode_gpio_start (void);
/** Record a changed electrical level at the 5 ms sensing timestamp. */
void ir_barcode_gpio_poll (bool b_high, uint32_t sampled_at_us);
bool ir_barcode_gpio_pop (ir_barcode_edge_t * p_edge);
uint32_t ir_barcode_gpio_overflows (void);

#endif
