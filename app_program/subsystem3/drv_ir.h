/**
 * @file drv_ir.h
 * @brief Buddy 3 digital inputs for the two line sensors and barcode sensor.
 */
#ifndef DRV_IR_H
#define DRV_IR_H

#include <stdbool.h>

#define DRV_IR_LEFT_PIN    7u
#define DRV_IR_RIGHT_PIN   4u
#define DRV_IR_BARCODE_PIN 16u

typedef struct
{
    bool b_left_high;
    bool b_right_high;
    bool b_barcode_high;
} drv_ir_levels_t;

/** @brief Configure all three DO signals as unbiased GPIO inputs. */
bool drv_ir_init (void);

/** @brief Read electrical levels without assuming a surface polarity. */
drv_ir_levels_t drv_ir_read (void);

#endif

/* End of file. */
