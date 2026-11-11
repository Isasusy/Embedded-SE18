/*
 *	enc_task.c -- Subsystem 2: starts the wheel-encoder edge interrupt on core 0 (this task is pinned there),
 *	then has nothing left to do. If the interrupt cannot be set up it polls the pins every 1 ms instead.
 *	APIs shown: tk_def_int / EnableInt (inside drv_encoder.c), tk_slp_tsk, tk_dly_tsk
 */
#include "subsystem2.h"
#include "drv_encoder.h"
#include "s2_port.h"

void enc_task(INT stacd, void *exinf)
{
	int err;
	(void)stacd; (void)exinf;

	err = enc_init();
	if (err == 0) {
		for (;;) tk_slp_tsk(TMO_FEVR);		/* the ISR does all the work */
	}
	S2_PRINTF("[S2] encoder interrupt failed (error %d): polling at 1 kHz instead\n", err);
	for (;;) {
		enc_poll();
		tk_dly_tsk(1);
	}
}
