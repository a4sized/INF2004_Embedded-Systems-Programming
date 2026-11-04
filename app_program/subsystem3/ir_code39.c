/** @file ir_code39.c @brief Code 39 widths from timed GP6 transitions. */
#include "ir_code39.h"

#include <stddef.h>

/* Code 39's nine elements, left to right, encoded as wide=1/narrow=0.
 * Character assignments cross-checked with ZXing Code39Reader's table.
 */
#define CODE39_ASTERISK 0x094u
#define CODE39_EDGE_COUNT IR_BARCODE_COMPLETE_EDGES
#define CODE39_CHARACTER_ELEMENTS 9u

static uint16_t const code39_letter_patterns[26] =
{
    0x109u, 0x049u, 0x148u, 0x019u, 0x118u, 0x058u, 0x00du,
    0x10cu, 0x04cu, 0x01cu, 0x103u, 0x043u, 0x142u, 0x013u,
    0x112u, 0x052u, 0x007u, 0x106u, 0x046u, 0x016u, 0x181u,
    0x0c1u, 0x1c0u, 0x091u, 0x190u, 0x0d0u
};

static uint32_t edge_duration (ir_barcode_window_t const * p_window,
                               uint32_t index, bool b_reverse)
{
    if (b_reverse)
    {
        return p_window->edges[CODE39_EDGE_COUNT - 1u - index].time_us -
               p_window->edges[CODE39_EDGE_COUNT - 2u - index].time_us;
    }
    return p_window->edges[index + 1u].time_us -
           p_window->edges[index].time_us;
}

static bool decode_character (ir_barcode_window_t const * p_window,
                              uint32_t start, bool b_reverse,
                              uint16_t * p_pattern,
                              uint32_t * p_narrow_us)
{
    uint32_t sorted[CODE39_CHARACTER_ELEMENTS] = {0};
    uint32_t durations[CODE39_CHARACTER_ELEMENTS] = {0};

    for (uint32_t i = 0u; i < CODE39_CHARACTER_ELEMENTS; ++i)
    {
        uint32_t const duration =
            edge_duration(p_window, start + i, b_reverse);
        if (0u == duration)
        {
            return false;
        }
        durations[i] = duration;
        uint32_t j = i;
        while ((j > 0u) && (sorted[j - 1u] > duration))
        {
            sorted[j] = sorted[j - 1u];
            --j;
        }
        sorted[j] = duration;
    }

    /* Six narrow elements must be clearly separated from three wide ones.
     * These bounds are provisional until GP6 scans the actual printouts.
     */
    if (((uint64_t)sorted[6] * 5u) < ((uint64_t)sorted[5] * 7u) ||
        ((uint64_t)sorted[8]) > ((uint64_t)sorted[6] * 2u))
    {
        return false;
    }

    uint16_t pattern = 0u;
    uint32_t wide_count = 0u;
    for (uint32_t i = 0u; i < CODE39_CHARACTER_ELEMENTS; ++i)
    {
        if (durations[i] >= sorted[6])
        {
            pattern |= (uint16_t)(1u << (8u - i));
            ++wide_count;
        }
    }
    if (3u != wide_count)
    {
        return false;
    }
    *p_pattern = pattern;
    *p_narrow_us = sorted[5];
    return true;
}

static bool decode_direction (ir_barcode_window_t const * p_window,
                              uint32_t now_us, bool b_reverse,
                              char * p_letter)
{
    uint16_t start = 0u;
    uint16_t data = 0u;
    uint16_t stop = 0u;
    uint32_t narrow_start = 0u;
    uint32_t narrow_data = 0u;
    uint32_t narrow_stop = 0u;
    if (!decode_character(p_window, 0u, b_reverse, &start, &narrow_start) ||
        !decode_character(p_window, 10u, b_reverse, &data, &narrow_data) ||
        !decode_character(p_window, 20u, b_reverse, &stop, &narrow_stop) ||
        (CODE39_ASTERISK != start) || (CODE39_ASTERISK != stop))
    {
        return false;
    }

    /* The intercharacter gaps are white and near the narrow width.
     * A long blank interval inside the symbol is not a valid letter.
     */
    uint32_t const gap1 = edge_duration(p_window, 9u, b_reverse);
    uint32_t const gap2 = edge_duration(p_window, 19u, b_reverse);
    uint32_t const narrow = (narrow_start + narrow_data + narrow_stop) / 3u;
    if ((0u == narrow) || (gap1 < (narrow / 2u)) ||
        (gap2 < (narrow / 2u)) ||
        ((uint64_t)gap1 > ((uint64_t)narrow * 3u)) ||
        ((uint64_t)gap2 > ((uint64_t)narrow * 3u)) ||
        ((uint64_t)(now_us - p_window->edges[29].time_us) <
         ((uint64_t)narrow * 10u)))
    {
        return false;
    }

    for (uint32_t i = 0u; i < 26u; ++i)
    {
        if (data == code39_letter_patterns[i])
        {
            *p_letter = (char)('A' + (char)i);
            return true;
        }
    }
    return false;
}

bool ir_code39_decode_single_with_direction (
    ir_barcode_window_t const * p_window, uint32_t now_us,
    char * p_letter, ir_code39_direction_t * p_direction)
{
    if ((NULL == p_window) || (NULL == p_letter) || p_window->b_invalid ||
        (CODE39_EDGE_COUNT != p_window->count))
    {
        return false;
    }
    for (uint32_t i = 0u; i < CODE39_EDGE_COUNT; ++i)
    {
        if (p_window->edges[i].b_ambiguous ||
            (p_window->edges[i].b_high != (0u == (i & 1u))))
        {
            return false;
        }
        if ((i > 0u) &&
            (0u == (p_window->edges[i].time_us -
                    p_window->edges[i - 1u].time_us)))
        {
            return false;
        }
    }

    char forward = '\0';
    char reverse = '\0';
    bool const b_forward = decode_direction(p_window, now_us, false, &forward);
    bool const b_reverse = decode_direction(p_window, now_us, true, &reverse);
    if ((!b_forward && !b_reverse) ||
        (b_forward && b_reverse && (forward != reverse)))
    {
        return false;
    }
    *p_letter = b_forward ? forward : reverse;
    if (NULL != p_direction)
    {
        *p_direction = b_forward ? IR_CODE39_FORWARD : IR_CODE39_REVERSE;
    }
    return true;
}

bool ir_code39_decode_single (ir_barcode_window_t const * p_window,
                              uint32_t now_us, char * p_letter)
{
    return ir_code39_decode_single_with_direction(p_window, now_us,
                                                   p_letter, NULL);
}

ir_barcode_action_t ir_barcode_action_for (char letter)
{
    if ('A' == letter)
    {
        return IR_BARCODE_TURN_LEFT;
    }
    if ('Z' == letter)
    {
        return IR_BARCODE_TURN_RIGHT;
    }
    return IR_BARCODE_NO_ACTION;
}
