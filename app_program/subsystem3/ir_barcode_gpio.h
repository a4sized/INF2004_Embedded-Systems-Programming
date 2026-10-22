#ifndef IR_BARCODE_GPIO_H
#define IR_BARCODE_GPIO_H

#include "ir_barcode_capture.h"

/** Install the standalone GP16 interrupt on the calling core. */
void ir_barcode_gpio_start (void);
bool ir_barcode_gpio_pop (ir_barcode_edge_t * p_edge);
uint32_t ir_barcode_gpio_overflows (void);

#endif
