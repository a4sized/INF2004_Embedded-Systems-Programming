/** @file ir_line_snapshot_shared.h @brief Task-safe latest IR snapshot. */
#ifndef IR_LINE_SNAPSHOT_SHARED_H
#define IR_LINE_SNAPSHOT_SHARED_H

#include "ir_line_snapshot.h"

#include <tk/tkernel.h>

/** Create the mutex before starting the sensing task or any reader. */
ER ir_line_snapshot_shared_init (void);

/** Replace the one stored snapshot from task context; never call in an ISR. */
bool ir_line_snapshot_shared_publish (ir_line_snapshot_t const * p_snapshot);

/** Copy one coherent snapshot from task context; false before first publish. */
bool ir_line_snapshot_shared_read (ir_line_snapshot_t * p_snapshot);

/** Withdraw the snapshot on an accepted reset or loss of publisher state. */
bool ir_line_snapshot_shared_invalidate (void);

#endif
