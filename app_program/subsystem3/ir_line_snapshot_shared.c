/** @file ir_line_snapshot_shared.c @brief Mutex-protected latest IR state. */
#include "ir_line_snapshot_shared.h"

static ID snapshot_mutex;
static ir_line_snapshot_t latest_snapshot;
static bool b_have_snapshot;

ER ir_line_snapshot_shared_init (void)
{
    T_CMTX config = {0};

    config.mtxatr = TA_TFIFO | TA_INHERIT;
    snapshot_mutex = tk_cre_mtx(&config);
    if (snapshot_mutex < E_OK)
    {
        return snapshot_mutex;
    }
    b_have_snapshot = false;
    return E_OK;
}

bool ir_line_snapshot_shared_publish (ir_line_snapshot_t const * p_snapshot)
{
    if ((NULL == p_snapshot) || (snapshot_mutex <= E_OK))
    {
        return false;
    }
    if (E_OK != tk_loc_mtx(snapshot_mutex, TMO_FEVR))
    {
        return false;
    }

    latest_snapshot = *p_snapshot;
    b_have_snapshot = true;
    return (E_OK == tk_unl_mtx(snapshot_mutex));
}

bool ir_line_snapshot_shared_read (ir_line_snapshot_t * p_snapshot)
{
    bool b_found;

    if ((NULL == p_snapshot) || (snapshot_mutex <= E_OK))
    {
        return false;
    }
    if (E_OK != tk_loc_mtx(snapshot_mutex, TMO_FEVR))
    {
        return false;
    }

    b_found = b_have_snapshot;
    if (b_found)
    {
        *p_snapshot = latest_snapshot;
    }
    if (E_OK != tk_unl_mtx(snapshot_mutex))
    {
        return false;
    }
    return b_found;
}

bool ir_line_snapshot_shared_invalidate (void)
{
    if (snapshot_mutex <= E_OK)
    {
        return false;
    }
    if (E_OK != tk_loc_mtx(snapshot_mutex, TMO_FEVR))
    {
        return false;
    }
    b_have_snapshot = false;
    latest_snapshot = (ir_line_snapshot_t){0};
    return (E_OK == tk_unl_mtx(snapshot_mutex));
}
