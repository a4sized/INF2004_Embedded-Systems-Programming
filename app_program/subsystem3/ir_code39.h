/** @file ir_code39.h @brief Single-letter Code 39 decoding and fixed actions. */
#ifndef IR_CODE39_H
#define IR_CODE39_H

#include "ir_barcode_window.h"

typedef enum
{
    IR_BARCODE_NO_ACTION = 0,
    IR_BARCODE_TURN_LEFT,
    IR_BARCODE_TURN_RIGHT,
    IR_BARCODE_U_TURN
} ir_barcode_action_t;

typedef enum
{
    /* Pattern order relative to Code 39 encoding, not vehicle travel. */
    IR_CODE39_FORWARD = 0,
    IR_CODE39_REVERSE
} ir_code39_direction_t;

/** Validate the first Code 39 asterisk after its ten boundary edges. */
bool ir_code39_start_marker_seen (ir_barcode_window_t const * p_window);

/** Validate *LETTER* as soon as all 30 edges arrive, before trailing quiet. */
bool ir_code39_frame_single_with_direction (
    ir_barcode_window_t const * p_window,
    char * p_letter, ir_code39_direction_t * p_direction);

/**
 * Decode exactly *LETTER* without a check character. The window must contain
 * 30 alternating edges (15 bars) and a trailing quiet zone at now_us.
 * Either scan direction is accepted.
 * Returns false for incomplete, invalid, or non-letter symbols.
 */
bool ir_code39_decode_single (ir_barcode_window_t const * p_window,
                              uint32_t now_us, char * p_letter);

/** As above, also report the direction of a valid scan. */
bool ir_code39_decode_single_with_direction (
    ir_barcode_window_t const * p_window, uint32_t now_us,
    char * p_letter, ir_code39_direction_t * p_direction);

/** Explain a frame rejection in one pattern order; returned text is static.
 * Width failures include insufficient narrow/wide separation or wide spread.
 */
char const * ir_code39_frame_reason (
    ir_barcode_window_t const * p_window, bool b_reverse);

/** Fixed course trial mapping; W is left, Z is right, F is U-turn. */
ir_barcode_action_t ir_barcode_action_for (char letter);

#endif
