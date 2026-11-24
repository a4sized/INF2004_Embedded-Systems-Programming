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

/* Drive an armed letter through the GP6 junction cue (white, black, exit
 * white) up to the tick before the exit edge completes. */
static void cross_junction (ir_junction_t * p_junction,
                            ir_line_snapshot_t * p_snapshot, uint32_t t0)
{
    p_snapshot->barcode_surface = IR_SURFACE_WHITE;
    tick(p_junction, p_snapshot, t0, true, false, 0, 0,
         IR_JUNCTION_ARMED);
    tick(p_junction, p_snapshot, t0 + 1u, true, false, 0, 0,
         IR_JUNCTION_NO_EVENT);
    p_snapshot->barcode_surface = IR_SURFACE_BLACK;
    tick(p_junction, p_snapshot, t0 + 2u, true, false, 0, 0,
         IR_JUNCTION_NO_EVENT);
    tick(p_junction, p_snapshot, t0 + 3u, true, true, 64, 64,
         IR_JUNCTION_CROSSING);
    p_snapshot->barcode_surface = IR_SURFACE_WHITE;
    tick(p_junction, p_snapshot, t0 + 4u, true, true, 64, 64,
         IR_JUNCTION_NO_EVENT);
}

static void test_straight (void)
{
    ir_junction_t junction = {0};
    ir_line_snapshot_t snapshot = {0};
    snapshot.b_ready = true;
    snapshot.b_gpio_ready = true;
    snapshot.left_surface = IR_SURFACE_BLACK;
    snapshot.right_surface = IR_SURFACE_BLACK;
    snapshot.barcode_decode_sequence = 1u;
    /* No straight letter by default (the dashboard maps one). */
    assert(IR_BARCODE_NO_ACTION == ir_barcode_action_for('S'));
    assert(ir_code39_set_action_map('W', 'Z', 'S', 'F'));
    snapshot.decoded_letter = 'S';
    cross_junction(&junction, &snapshot, 0u);
    assert(IR_BARCODE_STRAIGHT == junction.action);
    tick(&junction, &snapshot, 5u, true, false, 0, 0,
         IR_JUNCTION_STRAIGHT_DONE);
    assert(IR_JUNCTION_IDLE == junction.mode);
    tick(&junction, &snapshot, 6u, true, false, 0, 0,
         IR_JUNCTION_NO_EVENT);
    assert(ir_code39_set_action_map('W', 'Z', '-', 'F'));
}

static void test_uturn (void)
{
    ir_junction_t junction = {0};
    ir_line_snapshot_t snapshot = {0};
    snapshot.b_ready = true;
    snapshot.b_gpio_ready = true;
    snapshot.barcode_surface = IR_SURFACE_BLACK;
    snapshot.left_surface = IR_SURFACE_BLACK;
    snapshot.right_surface = IR_SURFACE_BLACK;
    snapshot.barcode_decode_sequence = 1u;
    snapshot.decoded_letter = 'F';
    int32_t const goal = (int32_t)IR_UTURN_WHEEL_TICKS;
    ir_junction_note_odometry(&junction, 1000, 2000);
    /* The decode alone must not pivot: it waits for the junction. */
    snapshot.barcode_surface = IR_SURFACE_WHITE;
    tick(&junction, &snapshot, 0u, true, false, 0, 0,
         IR_JUNCTION_ARMED);
    assert(IR_JUNCTION_WAIT_CLEAR == junction.mode);
    junction = (ir_junction_t){0};
    ir_junction_note_odometry(&junction, 1000, 2000);
    cross_junction(&junction, &snapshot, 0u);
    tick(&junction, &snapshot, 5u, true, true, -64, 64,
         IR_JUNCTION_TURN_STARTED);
    /* Driving straight must not count as a half-turn. */
    ir_junction_note_odometry(&junction, 1000 + goal, 2000 + goal);
    tick(&junction, &snapshot, 10u, true, true, -64, 64,
         IR_JUNCTION_NO_EVENT);
    ir_junction_note_odometry(&junction, 1000 - goal / 2, 2000 + goal / 2);
    snapshot.left_surface = IR_SURFACE_WHITE;
    snapshot.right_surface = IR_SURFACE_WHITE;
    tick(&junction, &snapshot, 20u, true, true, -64, 64,
         IR_JUNCTION_NO_EVENT);
    snapshot.left_surface = IR_SURFACE_BLACK; /* Early reacquisition ignored. */
    tick(&junction, &snapshot, 30u, true, true, -64, 64,
         IR_JUNCTION_NO_EVENT);
    ir_junction_note_odometry(&junction, 1000 - goal + 1, 2000 + goal - 1);
    tick(&junction, &snapshot, 40u, true, true, -64, 64,
         IR_JUNCTION_NO_EVENT);
    snapshot.left_surface = IR_SURFACE_WHITE;
    ir_junction_note_odometry(&junction, 1000 - goal, 2000 + goal);
    tick(&junction, &snapshot, 50u, true, true, 0, 0,
         IR_JUNCTION_NO_EVENT); /* Finishes encoder rotation with NO black. */
    assert(IR_JUNCTION_UTURN_SETTLING == junction.mode);
    tick(&junction, &snapshot, 249u, true, true, 0, 0,
         IR_JUNCTION_NO_EVENT);
    tick(&junction, &snapshot, 250u, true, false, 0, 0,
         IR_JUNCTION_UTURN_DONE);
    tick(&junction, &snapshot, 260u, true, false, 0, 0,
         IR_JUNCTION_NO_EVENT);

    snapshot.barcode_decode_sequence = 2u;
    cross_junction(&junction, &snapshot, 295u);
    tick(&junction, &snapshot, 300u, true, true, -64, 64,
         IR_JUNCTION_TURN_STARTED);
    tick(&junction, &snapshot, 8300u, true, true, 0, 0,
         IR_JUNCTION_TURN_TIMEOUT);
    tick(&junction, &snapshot, 8310u, true, true, 0, 0,
         IR_JUNCTION_NO_EVENT);
    tick(&junction, &snapshot, 8320u, false, false, 0, 0,
         IR_JUNCTION_NO_EVENT);
    snapshot.barcode_decode_sequence = 3u;
    cross_junction(&junction, &snapshot, 8325u);
    tick(&junction, &snapshot, 8330u, true, true, -64, 64,
         IR_JUNCTION_TURN_STARTED);
    ir_motion_target_t target = {0};
    ir_junction_event_t event = IR_JUNCTION_NO_EVENT;
    assert(!ir_junction_update(&junction, &snapshot, 8351u, 20u, true,
                              64u, 64u, &target, &event));
    assert(0 == target.left && 0 == target.right);
    tick(&junction, &snapshot, 8360u, true, false, 0, 0,
         IR_JUNCTION_NO_EVENT);

    /* Signed wheel totals may wrap during a pivot. */
    ir_junction_note_odometry(&junction, INT32_MIN + 10, INT32_MAX - 10);
    snapshot.barcode_decode_sequence = 4u;
    cross_junction(&junction, &snapshot, 8395u);
    tick(&junction, &snapshot, 8400u, true, true, -64, 64,
         IR_JUNCTION_TURN_STARTED);
    ir_junction_note_odometry(&junction, INT32_MAX - goal + 11,
                              INT32_MIN + goal - 11);
    tick(&junction, &snapshot, 8500u, true, true, 0, 0,
         IR_JUNCTION_NO_EVENT);
    tick(&junction, &snapshot, 8700u, true, false, 0, 0,
         IR_JUNCTION_UTURN_DONE);

    /* No supplied odometry must stop rather than execute a blind pivot. */
    junction = (ir_junction_t){0};
    snapshot.barcode_decode_sequence = 5u;
    cross_junction(&junction, &snapshot, 8795u);
    tick(&junction, &snapshot, 8800u, true, true, 0, 0,
         IR_JUNCTION_TURN_TIMEOUT);
}

int main (void)
{
    test_uturn();
    test_straight();
    test_turn('W', 0, 64);
    test_turn('Z', 64, 0);
    test_timeout_and_permission();
    test_unmapped_and_stale();
    test_black_junction_does_not_deadlock();
    test_next_decode_after_turn();
    return 0;
}
