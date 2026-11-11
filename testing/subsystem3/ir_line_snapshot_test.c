#include "ir_line_snapshot_shared.h"
#include "ir_motion_handoff.h"

#include <assert.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdint.h>

static pthread_mutex_t test_mutex = PTHREAD_MUTEX_INITIALIZER;
static atomic_bool b_writer_done;

ID tk_cre_mtx (T_CMTX const * p_config)
{
    assert(NULL != p_config);
    assert((TA_TFIFO | TA_INHERIT) == p_config->mtxatr);
    return 1;
}

ER tk_loc_mtx (ID mutex, int timeout)
{
    assert(1 == mutex);
    assert(TMO_FEVR == timeout);
    return pthread_mutex_lock(&test_mutex);
}

ER tk_unl_mtx (ID mutex)
{
    assert(1 == mutex);
    return pthread_mutex_unlock(&test_mutex);
}

static void * write_snapshots (void * p_unused)
{
    ir_line_snapshot_t snapshot = {0};

    (void)p_unused;
    snapshot.b_gpio_ready = true;
    snapshot.b_ready = true;
    snapshot.position = IR_LINE_LEFT_BLACK;
    snapshot.steer = IR_LINE_STEER_LEFT;
    for (uint32_t index = 1u; index <= 20000u; ++index)
    {
        snapshot.sequence = index;
        snapshot.sampled_at_ms = index * 5u;
        snapshot.raw.b_left_high = (0u != (index & 1u));
        snapshot.stable.b_left_high = snapshot.raw.b_left_high;
        assert(ir_line_snapshot_shared_publish(&snapshot));
    }
    atomic_store(&b_writer_done, true);
    return NULL;
}

static void * read_snapshots (void * p_unused)
{
    ir_line_snapshot_t snapshot = {0};
    uint32_t reads = 0u;

    (void)p_unused;
    while (!atomic_load(&b_writer_done) || (reads < 1000u))
    {
        assert(ir_line_snapshot_shared_read(&snapshot));
        assert(snapshot.sampled_at_ms == (snapshot.sequence * 5u));
        assert(snapshot.raw.b_left_high == snapshot.stable.b_left_high);
        assert(snapshot.raw.b_left_high == (0u != (snapshot.sequence & 1u)));
        ++reads;
    }
    return NULL;
}

int main (void)
{
    ir_line_snapshot_t snapshot = {0};
    ir_line_snapshot_t readback = {0};
    pthread_t writer;
    pthread_t reader;

    assert(E_OK == ir_line_snapshot_shared_init());
    assert(!ir_line_snapshot_shared_read(&readback));

    /* A 5 ms producer and 20 ms reader exchange the newest sample. */
    snapshot.b_gpio_ready = true;
    snapshot.b_ready = true;
    snapshot.position = IR_LINE_LEFT_BLACK;
    snapshot.steer = IR_LINE_STEER_LEFT;
    for (uint32_t now_ms = 0u; now_ms <= 100u; now_ms += 5u)
    {
        snapshot.sequence = 1u + (now_ms / 5u);
        snapshot.sampled_at_ms = now_ms;
        assert(ir_line_snapshot_shared_publish(&snapshot));
        if (0u == (now_ms % 20u))
        {
            assert(ir_line_snapshot_shared_read(&readback));
            assert(snapshot.sequence == readback.sequence);
            assert(IR_STEERING_AVAILABLE == ir_line_snapshot_steering_status(
                &readback, now_ms, 20u, true));
        }
    }

    assert(IR_STEERING_STALE == ir_line_snapshot_steering_status(
        &readback, 121u, 20u, true));
    assert(IR_STEERING_PAUSED == ir_line_snapshot_steering_status(
        &readback, 100u, 20u, false));
    readback.b_ready = false;
    assert(IR_STEERING_UNREADY == ir_line_snapshot_steering_status(
        &readback, 100u, 20u, true));
    readback.b_ready = true;
    readback.position = IR_LINE_BOTH_WHITE;
    readback.steer = IR_LINE_STEER_NONE;
    assert(IR_STEERING_NO_LINE == ir_line_snapshot_steering_status(
        &readback, 100u, 20u, true));
    readback.position = IR_LINE_CENTERED;
    assert(IR_STEERING_AVAILABLE == ir_line_snapshot_steering_status(
        &readback, 100u, 20u, true));
    readback.b_lost = true;
    assert(IR_STEERING_NO_LINE == ir_line_snapshot_steering_status(
        &readback, 100u, 20u, true));

    readback.b_lost = false;
    readback.sampled_at_ms = UINT32_MAX - 9u;
    assert(IR_STEERING_AVAILABLE == ir_line_snapshot_steering_status(
        &readback, 10u, 20u, true));
    assert(IR_STEERING_STALE == ir_line_snapshot_steering_status(
        &readback, 11u, 20u, true));

    assert(ir_motion_snapshot_is_new(&readback, 1u, false));
    assert(ir_motion_snapshot_is_new(&readback, readback.sequence - 1u, true));
    assert(!ir_motion_snapshot_is_new(&readback, readback.sequence, true));
    assert(!ir_motion_snapshot_is_new(NULL, 0u, false));

    /* Motion intent is zero unless every gate passes. */
    readback.sampled_at_ms = 100u;
    ir_motion_target_t target = ir_motion_target_for_snapshot(
        &readback, 100u, 20u, false, 80u, 25u);
    assert(IR_STEERING_PAUSED == target.status);
    assert((0 == target.left) && (0 == target.right));

    /* Moving barcode test: the first GP6 black edge holds straight motion
     * through a temporary both-white line reading, then ordinary steering
     * resumes when the capture window ends. Permission and freshness win.
     */
    readback.b_ready = true;
    readback.b_lost = true;
    readback.b_barcode_capturing = true;
    readback.sampled_at_ms = 100u;
    assert(!ir_barcode_capture_target(&readback, 100u, 20u, true,
                                     false, 20u, &target));
    assert(ir_barcode_capture_target(&readback, 100u, 20u, true,
                                    true, 20u, &target));
    assert(IR_STEERING_AVAILABLE == target.status);
    assert((20 == target.left) && (20 == target.right));
    readback.b_barcode_decoded = true;
    assert(!ir_barcode_capture_target(&readback, 100u, 20u, true,
                                      true, 20u, &target));
    readback.b_barcode_decoded = false;
    assert(!ir_barcode_capture_target(&readback, 100u, 20u, false,
                                     true, 20u, &target));
    assert(!ir_barcode_capture_target(&readback, 121u, 20u, true,
                                     true, 20u, &target));
    readback.b_barcode_capturing = false;
    assert(!ir_barcode_capture_target(&readback, 100u, 20u, true,
                                     true, 20u, &target));
    readback.b_lost = false;
    target = ir_motion_target_for_snapshot(&readback, 121u, 20u,
                                           true, 80u, 25u);
    assert(IR_STEERING_STALE == target.status);
    assert((0 == target.left) && (0 == target.right));
    readback.position = IR_LINE_LEFT_BLACK;
    readback.steer = IR_LINE_STEER_LEFT;
    target = ir_motion_target_for_snapshot(&readback, 100u, 20u,
                                           true, 80u, 25u);
    assert(IR_STEERING_AVAILABLE == target.status);
    assert((80 == target.left) && (80 == target.right));
    readback.position = IR_LINE_RIGHT_BLACK;
    readback.steer = IR_LINE_STEER_RIGHT;
    target = ir_motion_target_for_snapshot(&readback, 100u, 20u,
                                           true, 250u, 50u);
    assert((250 == target.left) && (250 == target.right));
    readback.position = IR_LINE_CENTERED;
    readback.steer = IR_LINE_STEER_NONE;
    target = ir_motion_target_for_snapshot(&readback, 100u, 20u,
                                           true, 80u, 25u);
    assert((80 == target.left) && (80 == target.right));
    readback.position = IR_LINE_BOTH_WHITE;
    target = ir_motion_target_for_snapshot(&readback, 100u, 20u,
                                           true, 80u, 25u);
    assert(IR_STEERING_NO_LINE == target.status);
    assert((0 == target.left) && (0 == target.right));
    readback.b_ready = false;
    target = ir_motion_target_for_snapshot(&readback, 100u, 20u,
                                           true, 80u, 25u);
    assert(IR_STEERING_UNREADY == target.status);
    assert((0 == target.left) && (0 == target.right));

    /* The integrated policy drives straight, corrects loss, stops and searches. */
    ir_follow_controller_t follow = {0};
    readback.b_ready = true;
    readback.b_lost = false;
    readback.position = IR_LINE_BOTH_WHITE;
    readback.steer = IR_LINE_STEER_NONE;
    readback.sampled_at_ms = 100u;
    target = ir_follow_controller_update(&follow, &readback, 100u,
                                         20u, true, 20u, 22u);
    assert(IR_FOLLOW_WAIT == follow.mode);
    assert((0 == target.left) && (0 == target.right));

    readback.position = IR_LINE_LEFT_BLACK;
    readback.steer = IR_LINE_STEER_LEFT;
    readback.sampled_at_ms = 110u;
    target = ir_follow_controller_update(&follow, &readback, 110u,
                                         20u, true, 20u, 22u);
    assert(IR_FOLLOW_TRACK == follow.mode);
    assert((20 == target.left) && (22 == target.right));

    readback.position = IR_LINE_BOTH_WHITE;
    readback.steer = IR_LINE_STEER_NONE;
    readback.sampled_at_ms = 120u;
    target = ir_follow_controller_update(&follow, &readback, 120u,
                                         20u, true, 20u, 22u);
    assert(IR_FOLLOW_TRACK == follow.mode);
    assert((20 == target.left) && (22 == target.right));
    readback.sampled_at_ms = 130u;
    target = ir_follow_controller_update(&follow, &readback, 130u,
                                         20u, true, 20u, 22u);
    assert(IR_FOLLOW_CORRECT == follow.mode);
    assert((0 == target.left) && (51 == target.right));
    readback.sampled_at_ms = 379u;
    target = ir_follow_controller_update(&follow, &readback, 379u,
                                         20u, true, 20u, 22u);
    assert(IR_FOLLOW_CORRECT == follow.mode);
    readback.sampled_at_ms = 380u;
    target = ir_follow_controller_update(&follow, &readback, 380u,
                                         20u, true, 20u, 22u);
    assert(IR_FOLLOW_STOPPING == follow.mode);
    assert((0 == target.left) && (0 == target.right));
    readback.sampled_at_ms = 629u;
    target = ir_follow_controller_update(&follow, &readback, 629u,
                                         20u, true, 20u, 22u);
    assert(IR_FOLLOW_STOPPING == follow.mode);
    assert((0 == target.left) && (0 == target.right));
    readback.sampled_at_ms = 630u;
    target = ir_follow_controller_update(&follow, &readback, 630u,
                                         20u, true, 20u, 22u);
    assert(IR_FOLLOW_SEARCH == follow.mode);
    assert((0 == target.left) && (0 == target.right));
    readback.sampled_at_ms = 640u;
    target = ir_follow_controller_update(&follow, &readback, 640u,
                                         20u, true, 20u, 22u);
    assert((0 == target.left) && (22 == target.right));

    readback.position = IR_LINE_RIGHT_BLACK;
    readback.steer = IR_LINE_STEER_RIGHT;
    readback.sampled_at_ms = 1400u;
    target = ir_follow_controller_update(&follow, &readback, 1400u,
                                         20u, true, 20u, 22u);
    assert(IR_FOLLOW_TRACK == follow.mode);
    assert((22 == target.left) && (20 == target.right));
    readback.position = IR_LINE_BOTH_WHITE;
    readback.steer = IR_LINE_STEER_NONE;
    readback.sampled_at_ms = 1410u;
    (void)ir_follow_controller_update(&follow, &readback, 1410u,
                                      20u, true, 20u, 22u);
    readback.sampled_at_ms = 1420u;
    target = ir_follow_controller_update(&follow, &readback, 1420u,
                                      20u, true, 20u, 22u);
    assert(IR_FOLLOW_CORRECT == follow.mode);
    assert((51 == target.left) && (0 == target.right));
    readback.position = IR_LINE_RIGHT_BLACK;
    readback.steer = IR_LINE_STEER_RIGHT;
    readback.sampled_at_ms = 1430u;
    target = ir_follow_controller_update(&follow, &readback, 1430u,
                                         20u, true, 20u, 22u);
    assert(IR_FOLLOW_TRACK == follow.mode);
    assert((22 == target.left) && (20 == target.right));
    target = ir_follow_controller_update(&follow, &readback, 1430u,
                                         20u, false, 20u, 22u);
    assert(IR_FOLLOW_WAIT == follow.mode);
    assert((0 == target.left) && (0 == target.right));

    /* Without a lone-black history, loss skips directed correction. */
    readback.position = IR_LINE_CENTERED;
    readback.steer = IR_LINE_STEER_NONE;
    readback.sampled_at_ms = 1440u;
    target = ir_follow_controller_update(&follow, &readback, 1440u,
                                         20u, true, 20u, 22u);
    assert((20 == target.left) && (20 == target.right));
    readback.position = IR_LINE_BOTH_WHITE;
    readback.sampled_at_ms = 1450u;
    (void)ir_follow_controller_update(&follow, &readback, 1450u,
                                      20u, true, 20u, 22u);
    readback.sampled_at_ms = 1460u;
    target = ir_follow_controller_update(&follow, &readback, 1460u,
                                         20u, true, 20u, 22u);
    assert(IR_FOLLOW_STOPPING == follow.mode);
    assert((0 == target.left) && (0 == target.right));

    /* The moving test freezes the ordinary controller while barcode edges
     * are being scanned. Its last-side history must still be usable when
     * the scan ends with both line sensors white.
     */
    ir_follow_controller_reset(&follow);
    readback.b_lost = false;
    readback.position = IR_LINE_RIGHT_BLACK;
    readback.steer = IR_LINE_STEER_RIGHT;
    readback.sampled_at_ms = 2000u;
    target = ir_follow_controller_update(&follow, &readback, 2000u,
                                         20u, true, 51u, 22u);
    assert(IR_FOLLOW_TRACK == follow.mode);
    assert((51 == target.left) && (51 == target.right));
    readback.b_barcode_capturing = true;
    readback.position = IR_LINE_BOTH_WHITE;
    readback.steer = IR_LINE_STEER_NONE;
    readback.b_lost = true;
    readback.sampled_at_ms = 5000u;
    assert(ir_barcode_capture_target(&readback, 5000u, 20u, true,
                                    true, 51u, &target));
    assert(IR_FOLLOW_TRACK == follow.mode);
    readback.b_barcode_capturing = false;
    target = ir_follow_controller_update(&follow, &readback, 5000u,
                                         20u, true, 51u, 22u);
    assert((51 == target.left) && (51 == target.right));
    readback.sampled_at_ms = 5010u;
    target = ir_follow_controller_update(&follow, &readback, 5010u,
                                         20u, true, 51u, 22u);
    assert(IR_FOLLOW_CORRECT == follow.mode);
    assert((51 == target.left) && (0 == target.right));

    /* With the integrated 51/64 targets, a lone-black sensor biases the
     * outside wheel gradually. A side flip passes through equal requests;
     * GP6 capture keeps both wheels equal to protect barcode timing.
     */
    ir_follow_controller_reset(&follow);
    readback.b_lost = false;
    readback.position = IR_LINE_LEFT_BLACK;
    readback.steer = IR_LINE_STEER_LEFT;
    readback.sampled_at_ms = 6000u;
    target = ir_follow_controller_update(&follow, &readback, 6000u,
                                         20u, true, 51u, 64u);
    assert((51 == target.left) && (55 == target.right));
    readback.sampled_at_ms = 6010u;
    target = ir_follow_controller_update(&follow, &readback, 6010u,
                                         20u, true, 51u, 64u);
    assert((51 == target.left) && (59 == target.right));
    readback.sampled_at_ms = 6015u;
    target = ir_follow_controller_update(&follow, &readback, 6015u,
                                         20u, true, 51u, 64u);
    assert((51 == target.left) && (63 == target.right));
    readback.sampled_at_ms = 6018u;
    target = ir_follow_controller_update(&follow, &readback, 6018u,
                                         20u, true, 51u, 64u);
    assert((51 == target.left) && (64 == target.right));
    readback.position = IR_LINE_BOTH_WHITE;
    readback.steer = IR_LINE_STEER_NONE;
    readback.sampled_at_ms = 6019u;
    target = ir_follow_controller_update(&follow, &readback, 6019u,
                                         20u, true, 51u, 64u);
    assert(IR_FOLLOW_TRACK == follow.mode);
    assert((51 == target.left) && (64 == target.right));
    readback.position = IR_LINE_CENTERED;
    readback.steer = IR_LINE_STEER_NONE;
    readback.sampled_at_ms = 6020u;
    target = ir_follow_controller_update(&follow, &readback, 6020u,
                                         20u, true, 51u, 64u);
    assert((51 == target.left) && (60 == target.right));
    readback.position = IR_LINE_RIGHT_BLACK;
    readback.steer = IR_LINE_STEER_RIGHT;
    readback.sampled_at_ms = 6030u;
    target = ir_follow_controller_update(&follow, &readback, 6030u,
                                         20u, true, 51u, 64u);
    assert((51 == target.left) && (56 == target.right));
    readback.sampled_at_ms = 6040u;
    target = ir_follow_controller_update(&follow, &readback, 6040u,
                                         20u, true, 51u, 64u);
    assert((51 == target.left) && (52 == target.right));
    readback.b_barcode_capturing = true;
    readback.sampled_at_ms = 6050u;
    target = ir_follow_controller_update(&follow, &readback, 6050u,
                                         20u, true, 51u, 64u);
    assert((51 == target.left) && (51 == target.right));
    readback.b_barcode_capturing = false;
    readback.sampled_at_ms = 6060u;
    target = ir_follow_controller_update(&follow, &readback, 6060u,
                                         20u, true, 51u, 64u);
    assert((55 == target.left) && (51 == target.right));
    target = ir_follow_controller_update(&follow, &readback, 6081u,
                                         20u, true, 51u, 64u);
    assert(IR_FOLLOW_WAIT == follow.mode);
    assert((0 == target.left) && (0 == target.right));

    /* Concurrent writer/reader must never expose a mixed struct copy. */
    snapshot.sequence = 1u;
    snapshot.sampled_at_ms = 5u;
    snapshot.raw.b_left_high = true;
    snapshot.stable.b_left_high = true;
    assert(ir_line_snapshot_shared_publish(&snapshot));
    atomic_store(&b_writer_done, false);
    assert(0 == pthread_create(&writer, NULL, write_snapshots, NULL));
    assert(0 == pthread_create(&reader, NULL, read_snapshots, NULL));
    assert(0 == pthread_join(writer, NULL));
    assert(0 == pthread_join(reader, NULL));
    assert(ir_line_snapshot_shared_invalidate());
    assert(!ir_line_snapshot_shared_read(&readback));
    return 0;
}
