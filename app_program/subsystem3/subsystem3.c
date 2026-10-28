/*
 *	subsystem3.c
 *	Subsystem 3 -- Consumer: creates its objects and starts its task.
 *
 *	APIs shown: tk_cre_mbx, tk_cre_mtx
 */

#include "subsystem3.h"

/* Legacy example objects remain linkable in the team image, but are idle. */
EXPORT ID mbx_samples;
EXPORT ID mtx_stats;
EXPORT APP_STATS app_stats;

#if defined(BUDDY3_TEAM_IR)
#include "ir_sensing_main.h"
#include "ir_motion_consumer.h"

EXPORT ER subsystem3_init(void)
{
    return ir_sensing_service_init();
}

EXPORT ER subsystem3_start(void)
{
    ER const status = ir_sensing_service_start();
    if (E_OK != status)
    {
        return status;
    }
    ID const task_id = app_start_task(ir_motion_consumer_task,
                                      S3_PRI_CONSUMER, TP_PRC1,
                                      "ir_motion_consumer_task");
    return (task_id < E_OK) ? task_id : E_OK;
}

#else

EXPORT ER subsystem3_init(void)
{
	T_CMBX	cmbx = {0};
	T_CMTX	cmtx = {0};

	/* Mailbox: passes a POINTER to a message (zero copy).  The message
	   memory must stay valid until the receiver is done with it, which
	   is why subsystem 2 takes it from its memory pool. */
	cmbx.mbxatr = TA_TFIFO | TA_MFIFO;	/* FIFO waiters, FIFO messages */
	mbx_samples = tk_cre_mbx(&cmbx);
	if (mbx_samples < E_OK) {
		return mbx_samples;
	}

	/* Mutex: only one task at a time may touch app_stats.  TA_INHERIT
	   (priority inheritance) avoids priority inversion: a low-priority
	   holder is temporarily raised to the waiter's priority. */
	cmtx.mtxatr = TA_INHERIT;
	mtx_stats = tk_cre_mtx(&cmtx);
	return (mtx_stats < E_OK) ? mtx_stats : E_OK;
}

EXPORT ER subsystem3_start(void)
{
	/* Pinned to core 0 (TP_PRC1): opposite core to the producer */
	ID tid = app_start_task(consumer_task, S3_PRI_CONSUMER, TP_PRC1,
				"consumer_task");
	return (tid < E_OK) ? tid : E_OK;
}
#endif
