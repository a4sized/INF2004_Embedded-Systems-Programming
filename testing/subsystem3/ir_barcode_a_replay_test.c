/* Replay one complete physical A scan supplied on 4 November 2026. */
#include "../../app_program/subsystem3/ir_barcode_window.h"
#include "../../app_program/subsystem3/ir_code39.h"

#include <assert.h>

static uint32_t const a_edges_us[IR_BARCODE_COMPLETE_EDGES] =
{
    633014785u, 633039882u, 633094785u, 633189879u, 633239826u,
    633329824u, 633379819u, 633399886u, 633509845u, 633524785u,
    633569824u, 633649820u, 633694783u, 633714847u, 633819880u,
    633839830u, 633879882u, 633899854u, 633944785u, 634029880u,
    634079826u, 634099844u, 634149888u, 634249822u, 634309837u,
    634414786u, 634474782u, 634494843u, 634634873u, 634654783u
};

/* A complete forward-pattern capture rejected by the earlier 1.5 threshold. */
static uint32_t const a_forward_edges_us[IR_BARCODE_COMPLETE_EDGES] =
{
    92329880u, 92344845u, 92429825u, 92439832u, 92474784u,
    92539883u, 92579824u, 92654782u, 92699829u, 92719883u,
    92769825u, 92854845u, 92899885u, 92924786u, 92964790u,
    92989886u, 93095316u, 93114788u, 93159827u, 93244842u,
    93290521u, 93309828u, 93424844u, 93444785u, 93489852u,
    93579829u, 93624786u, 93709881u, 93749823u, 93764786u
};

int main (void)
{
    ir_barcode_window_t window = {0};
    ir_barcode_window_reset(&window);
    for (uint32_t i = 0u; i < IR_BARCODE_COMPLETE_EDGES; ++i)
    {
        ir_barcode_edge_t const edge =
        {
            .time_us = a_edges_us[i],
            .b_high = (0u == (i & 1u)),
            .b_ambiguous = false
        };
        ir_barcode_window_feed(&window, edge);
    }
    assert(IR_BARCODE_COMPLETE_EDGES == window.count);
    assert(!window.b_invalid);
    window.count = 10u;
    assert(ir_code39_start_marker_seen(&window));
    window.count = IR_BARCODE_COMPLETE_EDGES;

    uint32_t const early_quiet = a_edges_us[29] + 100u;
    char letter = '\0';
    ir_code39_direction_t direction = IR_CODE39_FORWARD;
    assert(!ir_code39_decode_single_with_direction(
        &window, early_quiet, &letter, &direction));
    assert(!ir_barcode_window_expired(&window, early_quiet));

    uint32_t const quiet_complete = a_edges_us[29] + 600000u;
    assert(!ir_barcode_window_expired(&window, quiet_complete));
    assert(ir_code39_decode_single_with_direction(
        &window, quiet_complete, &letter, &direction));
    assert('A' == letter);
    assert(IR_CODE39_REVERSE == direction);
    assert(ir_barcode_window_expired(
        &window, a_edges_us[29] + IR_BARCODE_FINAL_QUIET_MAX_US));

    ir_barcode_window_reset(&window);
    ir_barcode_edge_t const first =
    {
        .time_us = a_edges_us[0], .b_high = true, .b_ambiguous = false
    };
    ir_barcode_window_feed(&window, first);
    assert(ir_barcode_window_expired(
        &window, a_edges_us[0] + IR_BARCODE_EDGE_IDLE_US));

    ir_barcode_window_reset(&window);
    for (uint32_t i = 0u; i < IR_BARCODE_COMPLETE_EDGES; ++i)
    {
        ir_barcode_edge_t const forward_edge =
        {
            .time_us = a_forward_edges_us[i],
            .b_high = (0u == (i & 1u)),
            .b_ambiguous = false
        };
        ir_barcode_window_feed(&window, forward_edge);
    }
    assert(!window.b_invalid);
    window.count = 10u;
    assert(ir_code39_start_marker_seen(&window));
    window.count = IR_BARCODE_COMPLETE_EDGES;
    assert(ir_code39_decode_single_with_direction(
        &window, a_forward_edges_us[29] + 600000u, &letter, &direction));
    assert('A' == letter);
    assert(IR_CODE39_FORWARD == direction);
    return 0;
}
