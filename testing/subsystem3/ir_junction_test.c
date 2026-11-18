#include "ir_junction.h"

#include <assert.h>

static void tick (ir_junction_t * p_junction, ir_line_snapshot_t * p_snapshot,
                  uint32_t now_ms, bool b_allowed,
                  bool b_expected_owned, int16_t left, int16_t right,
                  ir_junction_event_t expected_event)
{
    ir_motion_target_t target = {0};
    ir_junction_event_t event = IR_JUNCTION_NO_EVENT;
    p_snapshot->sampled_at_ms = now_ms;
    assert(b_expected_owned == ir_junction_update(
        p_junction, p_snapshot, now_ms, 20u, b_allowed, 64u, 64u,
        &target, &event));
    assert(expected_event == event);
    if (b_expected_owned)
    {
        assert(left == target.left);
        assert(right == target.right);
    }
}

static void test_turn (char letter, int16_t expected_left,
                       int16_t expected_right)
{
    ir_junction_t junction = {0};
    ir_line_snapshot_t snapshot = {0};
    snapshot.b_ready = true;
    snapshot.b_gpio_ready = true;
    snapshot.barcode_surface = IR_SURFACE_WHITE;
    snapshot.left_surface = IR_SURFACE_BLACK;
    snapshot.right_surface = IR_SURFACE_BLACK;
    snapshot.barcode_decode_sequence = 1u;
    snapshot.decoded_letter = letter;

    tick(&junction, &snapshot, 0u, true, false, 0, 0,
         IR_JUNCTION_ARMED);
    tick(&junction, &snapshot, 10u, true, false, 0, 0,
         IR_JUNCTION_NO_EVENT);
    assert(IR_JUNCTION_WAIT_BLACK == junction.mode);
    snapshot.barcode_surface = IR_SURFACE_BLACK;
    tick(&junction, &snapshot, 20u, true, false, 0, 0,
         IR_JUNCTION_NO_EVENT);
    tick(&junction, &snapshot, 30u, true, true, 64, 64,
         IR_JUNCTION_CROSSING);
    assert(IR_JUNCTION_WAIT_EXIT == junction.mode);
    snapshot.barcode_surface = IR_SURFACE_WHITE;
    tick(&junction, &snapshot, 40u, true, true, 64, 64,
         IR_JUNCTION_NO_EVENT);
    tick(&junction, &snapshot, 50u, true, true, expected_left,
         expected_right, IR_JUNCTION_TURN_STARTED);

    /* Current black readings cannot complete without white then black. */
    tick(&junction, &snapshot, 60u, true, true, expected_left,
         expected_right, IR_JUNCTION_NO_EVENT);
    snapshot.left_surface = IR_SURFACE_WHITE;
    snapshot.right_surface = IR_SURFACE_WHITE;
    tick(&junction, &snapshot, 70u, true, true, expected_left,
         expected_right, IR_JUNCTION_NO_EVENT);
    snapshot.left_surface = IR_SURFACE_BLACK;
    tick(&junction, &snapshot, 80u, true, true, expected_left,
         expected_right, IR_JUNCTION_NO_EVENT);
    snapshot.barcode_surface = IR_SURFACE_BLACK;
    tick(&junction, &snapshot, 90u, true, true, expected_left,
         expected_right, IR_JUNCTION_NO_EVENT);
    snapshot.barcode_surface = IR_SURFACE_WHITE;
    tick(&junction, &snapshot, 100u, true, false, 0, 0,
         IR_JUNCTION_TURN_DONE);
    tick(&junction, &snapshot, 110u, true, false, 0, 0,
         IR_JUNCTION_NO_EVENT);
    assert(IR_JUNCTION_IDLE == junction.mode);
}

static void test_timeout_and_permission (void)
{
    ir_junction_t junction = {0};
    ir_line_snapshot_t snapshot = {0};
    snapshot.b_ready = true;
    snapshot.b_gpio_ready = true;
    snapshot.barcode_surface = IR_SURFACE_WHITE;
    snapshot.barcode_decode_sequence = 1u;
    snapshot.decoded_letter = 'W';

    tick(&junction, &snapshot, 0u, false, false, 0, 0,
         IR_JUNCTION_NO_EVENT);
    tick(&junction, &snapshot, 10u, true, false, 0, 0,
         IR_JUNCTION_NO_EVENT);
    assert(IR_JUNCTION_IDLE == junction.mode);
    snapshot.barcode_decode_sequence = 2u;
    tick(&junction, &snapshot, 20u, true, false, 0, 0,
         IR_JUNCTION_ARMED);
    tick(&junction, &snapshot, 30u, true, false, 0, 0,
         IR_JUNCTION_NO_EVENT);
    snapshot.barcode_surface = IR_SURFACE_BLACK;
    tick(&junction, &snapshot, 40u, true, false, 0, 0,
         IR_JUNCTION_NO_EVENT);
    tick(&junction, &snapshot, 50u, true, true, 64, 64,
         IR_JUNCTION_CROSSING);
    snapshot.barcode_surface = IR_SURFACE_WHITE;
    tick(&junction, &snapshot, 60u, true, true, 64, 64,
         IR_JUNCTION_NO_EVENT);
    tick(&junction, &snapshot, 70u, true, true, 0, 64,
         IR_JUNCTION_TURN_STARTED);
    tick(&junction, &snapshot, 3070u, true, true, 0, 0,
         IR_JUNCTION_TURN_TIMEOUT);
    assert(IR_JUNCTION_FAILED == junction.mode);
    tick(&junction, &snapshot, 3080u, true, true, 0, 0,
         IR_JUNCTION_NO_EVENT);
    tick(&junction, &snapshot, 3090u, false, false, 0, 0,
         IR_JUNCTION_NO_EVENT);
    assert(IR_JUNCTION_IDLE == junction.mode);
}

static void test_unmapped_and_stale (void)
{
    ir_junction_t junction = {0};
    ir_line_snapshot_t snapshot = {0};
    ir_motion_target_t target = {0};
    ir_junction_event_t event = IR_JUNCTION_NO_EVENT;
    snapshot.b_ready = true;
    snapshot.b_gpio_ready = true;
    snapshot.barcode_surface = IR_SURFACE_WHITE;
    snapshot.barcode_decode_sequence = 1u;
    snapshot.decoded_letter = 'A';
    tick(&junction, &snapshot, 0u, true, false, 0, 0,
         IR_JUNCTION_NO_EVENT);
    assert(IR_JUNCTION_IDLE == junction.mode);
    snapshot.barcode_decode_sequence = 2u;
    snapshot.decoded_letter = 'Z';
    snapshot.sampled_at_ms = 10u;
    assert(!ir_junction_update(&junction, &snapshot, 40u, 20u, true,
                               64u, 64u,
                               &target, &event));
    assert(IR_JUNCTION_IDLE == junction.mode);
    tick(&junction, &snapshot, 50u, true, false, 0, 0,
         IR_JUNCTION_NO_EVENT);
}

static void test_black_junction_does_not_deadlock (void)
{
    ir_junction_t junction = {0};
    ir_line_snapshot_t snapshot = {0};
    snapshot.b_ready = true;
    snapshot.b_gpio_ready = true;
    snapshot.barcode_surface = IR_SURFACE_WHITE;
    snapshot.barcode_decode_sequence = 1u;
    snapshot.decoded_letter = 'Z';

    tick(&junction, &snapshot, 0u, true, false, 0, 0,
         IR_JUNCTION_ARMED);
    tick(&junction, &snapshot, 10u, true, false, 0, 0,
         IR_JUNCTION_NO_EVENT);
    snapshot.barcode_surface = IR_SURFACE_BLACK;
    snapshot.left_surface = IR_SURFACE_WHITE;
    snapshot.right_surface = IR_SURFACE_WHITE;
    tick(&junction, &snapshot, 20u, true, false, 0, 0,
         IR_JUNCTION_NO_EVENT);
    tick(&junction, &snapshot, 30u, true, true, 64, 64,
         IR_JUNCTION_CROSSING);
    tick(&junction, &snapshot, 3029u, true, true, 64, 64,
         IR_JUNCTION_NO_EVENT);
    tick(&junction, &snapshot, 3030u, true, true, 0, 0,
         IR_JUNCTION_CROSSING_TIMEOUT);
    assert(IR_JUNCTION_FAILED == junction.mode);
}

static void test_next_decode_after_turn (void)
{
    ir_junction_t junction = {0};
    ir_line_snapshot_t snapshot = {0};
    snapshot.b_ready = true;
    snapshot.b_gpio_ready = true;
    snapshot.barcode_surface = IR_SURFACE_WHITE;
    snapshot.left_surface = IR_SURFACE_BLACK;
    snapshot.right_surface = IR_SURFACE_BLACK;
    snapshot.barcode_decode_sequence = 1u;
    snapshot.decoded_letter = 'W';

    tick(&junction, &snapshot, 0u, true, false, 0, 0,
         IR_JUNCTION_ARMED);
    tick(&junction, &snapshot, 10u, true, false, 0, 0,
         IR_JUNCTION_NO_EVENT);
    snapshot.barcode_surface = IR_SURFACE_BLACK;
    tick(&junction, &snapshot, 20u, true, false, 0, 0,
         IR_JUNCTION_NO_EVENT);
    tick(&junction, &snapshot, 30u, true, true, 64, 64,
         IR_JUNCTION_CROSSING);
    snapshot.barcode_surface = IR_SURFACE_WHITE;
    tick(&junction, &snapshot, 40u, true, true, 64, 64,
         IR_JUNCTION_NO_EVENT);
    tick(&junction, &snapshot, 50u, true, true, 0, 64,
         IR_JUNCTION_TURN_STARTED);
    snapshot.left_surface = IR_SURFACE_WHITE;
    snapshot.right_surface = IR_SURFACE_WHITE;
    tick(&junction, &snapshot, 60u, true, true, 0, 64,
         IR_JUNCTION_NO_EVENT);
    snapshot.left_surface = IR_SURFACE_BLACK;
    snapshot.barcode_surface = IR_SURFACE_BLACK;
    tick(&junction, &snapshot, 70u, true, true, 0, 64,
         IR_JUNCTION_NO_EVENT);
    snapshot.barcode_surface = IR_SURFACE_WHITE;
    tick(&junction, &snapshot, 80u, true, false, 0, 0,
         IR_JUNCTION_TURN_DONE);
    tick(&junction, &snapshot, 90u, true, false, 0, 0,
         IR_JUNCTION_NO_EVENT);
    assert(IR_JUNCTION_IDLE == junction.mode);

    snapshot.barcode_decode_sequence = 2u;
    snapshot.decoded_letter = 'Z';
    tick(&junction, &snapshot, 100u, true, false, 0, 0,
         IR_JUNCTION_ARMED);
    assert(IR_BARCODE_TURN_RIGHT == junction.action);
    tick(&junction, &snapshot, 110u, true, false, 0, 0,
         IR_JUNCTION_NO_EVENT);
    snapshot.barcode_surface = IR_SURFACE_BLACK;
    tick(&junction, &snapshot, 120u, true, false, 0, 0,
         IR_JUNCTION_NO_EVENT);
    tick(&junction, &snapshot, 130u, true, true, 64, 64,
         IR_JUNCTION_CROSSING);
    snapshot.barcode_surface = IR_SURFACE_WHITE;
    tick(&junction, &snapshot, 140u, true, true, 64, 64,
         IR_JUNCTION_NO_EVENT);
    tick(&junction, &snapshot, 150u, true, true, 64, 0,
         IR_JUNCTION_TURN_STARTED);
}

int main (void)
{
    test_turn('W', 0, 64);
    test_turn('Z', 64, 0);
    test_timeout_and_permission();
    test_unmapped_and_stale();
    test_black_junction_does_not_deadlock();
    test_next_decode_after_turn();
    return 0;
}
