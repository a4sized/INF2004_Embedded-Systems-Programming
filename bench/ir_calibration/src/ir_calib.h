/**
 * @file ir_calib.h
 * @brief Single-sensor digital polarity calibration for Buddy 3 (2501107).
 */
#ifndef IR_CALIB_H
#define IR_CALIB_H

#include <stdbool.h>
#include <stdint.h>

/* NOTE: These are bench defaults, not measured robot requirements. */
#define IR_CALIB_SAMPLES       100u
#define IR_CALIB_MIN_AGREEMENT 95u

typedef struct
{
    uint32_t count;
    uint32_t high_count;
    uint32_t transitions;
    bool     b_previous_high;
} ir_calib_window_t;

typedef enum
{
    IR_CALIB_OK = 0,
    IR_CALIB_INCOMPLETE,
    IR_CALIB_UNSTABLE,
    IR_CALIB_NO_CONTRAST
} ir_calib_status_t;

typedef struct
{
    bool b_valid;
    bool b_black_high;
} ir_calib_result_t;

/** @brief Clear a window. The caller must supply a non-null pointer. */
void ir_calib_reset (ir_calib_window_t * p_window);

/** @brief Add one level; return false if the window is already full. */
bool ir_calib_add (ir_calib_window_t * p_window, bool b_high);

/**
 * @brief Learn polarity from complete black and white windows.
 * @details All pointers must be non-null. A failure clears the result.
 */
ir_calib_status_t ir_calib_evaluate (
    ir_calib_window_t const * p_black,
    ir_calib_window_t const * p_white,
    ir_calib_result_t * p_result);

#endif

/* End of file. */

