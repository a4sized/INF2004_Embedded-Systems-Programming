#include "ir_line_snapshot_shared.h"

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
