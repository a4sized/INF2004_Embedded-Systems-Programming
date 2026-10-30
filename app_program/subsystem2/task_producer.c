/*
 *	task_producer.c
 *	Subsystem 2 -- the producer task.
 *
 *	APIs shown: tk_wai_sem, tk_get_mpf, tk_snd_mbx, tk_get_tim,
 *	            tk_dly_tsk, tk_get_prc
 */

#include "subsystem2.h"
#include "encoder.h"                    /* Required for get_distance_traveled_mm() */
#include "../subsystem3/subsystem3.h"	/* IF-01: mbx_samples */

EXPORT void producer_task(INT stacd, void *exinf)
{
    UW          seq = 0;
    SAMPLE_MSG  *msg;
    SYSTIM      now;
    ER          er;

    (void)stacd; (void)exinf;

    while (1) {
        /* 1. Take a free slot. If the consumer is behind and all
              slots are in use, this task blocks here until one is
              returned (flow control). */
        er = tk_wai_sem(sem_slots, 1, TMO_FEVR);
        CHECK(er, "tk_wai_sem");

        /* 2. Get a memory block for the message. The semaphore
              guarantees one is free, so poll (TMO_POL = don't wait). */
        er = tk_get_mpf(mpf_samples, (void **)&msg, TMO_POL);
        if (er < E_OK) {
            CHECK(er, "tk_get_mpf");
            tk_sig_sem(sem_slots, 1);	/* give the slot back */
            tk_dly_tsk(S2_PERIOD_MS);
            continue;
        }

        /* 3. Fill in sequence and timestamps. tk_get_tim gives milliseconds 
              since boot as a 64-bit hi:lo pair; lo is enough for ~49 days. */
        tk_get_tim(&now);
        msg->seq     = seq++;
        msg->time_ms = now.lo;

        /* 4. THE CRITICAL UPDATE: Read real odometry instead of fake data */
        msg->left_encoder  = get_left_encoder_ticks();
        msg->right_encoder = get_right_encoder_ticks();
        msg->distance_mm   = get_distance_traveled_mm();

#if TK_SUPPORT_SMP
        msg->prc     = tk_get_prc();	/* which core am I on? */
#else
        msg->prc     = 1;
#endif

        /* 5. Send the POINTER to subsystem 3. From here on the block
              belongs to the receiver: do not touch msg again. */
        er = tk_snd_mbx(mbx_samples, (T_MSG *)msg);
        CHECK(er, "tk_snd_mbx");

        /* 6. Wait for the next period. Other tasks run meanwhile. */
        tk_dly_tsk(S2_PERIOD_MS);
    }
}