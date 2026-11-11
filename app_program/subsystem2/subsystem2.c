/*
 *	subsystem2.c
 *	Subsystem 2 -- Motion Control: creates its objects and starts its task.
 *
 *	APIs shown: tk_cre_mbf, tk_cre_flg, tk_cre_mtx
 */

#include "subsystem2.h"

EXPORT ID		mbf_motion_cmd;		/* IF-09 */
EXPORT ID		flg_motion;		/* IF-10 */
EXPORT ID		mtx_motion;		/* IF-08 */
EXPORT motion_status_t	app_motion;
EXPORT MOTION_DEBUG	s2_motion_debug;

/* IF-11 is owned by Line Follow. Weak defaults (0) so S2 links and runs alone; the real strong
   definitions replace them at link time. While mtx_line is 0, Motion Control just holds its heading. */
__attribute__((weak)) ID		mtx_line;
__attribute__((weak)) line_status_t	app_line;

EXPORT ER subsystem2_init(void)
{
	T_CMBF	cmbf = {0};
	T_CFLG	cflg = {0};
	T_CMTX	cmtx = {0};

	/* Message buffer: commands from the Vehicle Controller, FIFO, depth 4. */
	cmbf.mbfatr = TA_TFIFO;
	cmbf.bufsz  = S2_CMD_BUFSZ;
	cmbf.maxmsz = S2_CMD_MAXMSZ;
	mbf_motion_cmd = tk_cre_mbf(&cmbf);
	if (mbf_motion_cmd < E_OK) {
		return mbf_motion_cmd;
	}

	/* Event flag: STOP / RESET come from the VC, READY / ALIVE / FAULT from us. Several tasks may touch it. */
	cflg.flgatr  = TA_TFIFO | TA_WMUL;
	cflg.iflgptn = 0;
	flg_motion = tk_cre_flg(&cflg);
	if (flg_motion < E_OK) {
		return flg_motion;
	}

	/* Mutex around app_motion. Priority inheritance so a low-priority reader (the IMU) cannot
	   hold up the 10 ms control loop. */
	cmtx.mtxatr = TA_INHERIT;
	mtx_motion = tk_cre_mtx(&cmtx);
	return (mtx_motion < E_OK) ? mtx_motion : E_OK;
}

EXPORT ER subsystem2_start(void)
{
	/* Both tasks pinned to core 0 (TP_PRC1) so counting and control never cross cores. */
	ID tid = app_start_task(enc_task, S2_PRI_ENC, TP_PRC1, "enc_task");
	if (tid < E_OK) return tid;
	tid = app_start_task(motion_task, S2_PRI_MOTION, TP_PRC1, "motion_task");
	return (tid < E_OK) ? tid : E_OK;
}
