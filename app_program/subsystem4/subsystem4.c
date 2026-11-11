/*
 *	subsystem4.c
 *	Subsystem 4 -- IMU and terrain monitoring: creates its objects and starts its task.
 *
 *	APIs shown: tk_cre_flg
 */

#include "subsystem4.h"

EXPORT ID	flg_imu;		/* IF-07 */
EXPORT IMU_DEBUG s4_imu_debug;

/* Objects other subsystems will own (IF-06, IF-08). Weak defaults let subsystem 4 build, link and run
   on its own: while they stay 0 the IMU task prints its events and uses S4_FAKE_SPEED_MM_S. When the
   Vehicle Controller / Motion Control define the real (strong) ones, the linker picks theirs. */
__attribute__((weak)) ID		mbf_imu_evt;
__attribute__((weak)) ID		mtx_motion;
__attribute__((weak)) motion_status_t	app_motion;

EXPORT ER subsystem4_init(void)
{
	T_CFLG	cflg = {0};

	/* Event flag: RESET (from VC), READY and ALIVE (from us). Several tasks may wait/clear. */
	cflg.flgatr  = TA_TFIFO | TA_WMUL;
	cflg.iflgptn = 0;
	flg_imu = tk_cre_flg(&cflg);
	if (flg_imu < E_OK) {
		return flg_imu;
	}

	return E_OK;
}

EXPORT ER subsystem4_start(void)
{
	/* No affinity: the scheduler may run it on either core. The task only touches I2C0
	   (which it owns) and its own data, so that is safe. */
	ID tid = app_start_task(imu_task, S4_PRI_IMU, 0, "imu_task");
	return (tid < E_OK) ? tid : E_OK;
}
