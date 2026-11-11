/*
 *	subsystem5.c
 *	Subsystem 5 -- creates the buffers other subsystems send to, and starts the Vehicle Controller + scan tasks.
 *	APIs shown: tk_cre_mbf, tk_cre_flg
 */
#include "subsystem5.h"

EXPORT ID	mbf_imu_evt;	/* IF-06 (replaces S4's weak default) */
EXPORT ID	mbf_nav_cmd;	/* IF-12 (replaces S3's weak default) */
EXPORT ID	mbf_line_evt;	/* IF-13 */
EXPORT ID	mbf_remote_cmd;	/* IF-15 */
EXPORT ID	flg_scan;
EXPORT ID	mbf_obstacle;

/* Owned by Telemetry (S1). Weak default 0 so S5 links and runs without it. */
__attribute__((weak)) ID	mbf_status;

static ID make_mbf(INT maxmsz, INT bufsz)
{
	T_CMBF c = {0};
	c.mbfatr = TA_TFIFO;
	c.maxmsz = maxmsz;
	c.bufsz  = bufsz;
	return tk_cre_mbf(&c);
}

EXPORT ER subsystem5_init(void)
{
	T_CFLG cflg = {0};

	mbf_imu_evt = make_mbf((INT)sizeof(imu_evt_t), (INT)S5_MSG_BUFSZ(imu_evt_t, 8));
	if (mbf_imu_evt < E_OK) return mbf_imu_evt;
	mbf_nav_cmd = make_mbf((INT)sizeof(nav_cmd_t), (INT)S5_MSG_BUFSZ(nav_cmd_t, 4));
	if (mbf_nav_cmd < E_OK) return mbf_nav_cmd;
	mbf_line_evt = make_mbf((INT)sizeof(line_evt_t), (INT)S5_MSG_BUFSZ(line_evt_t, 4));
	if (mbf_line_evt < E_OK) return mbf_line_evt;
	mbf_remote_cmd = make_mbf((INT)sizeof(remote_cmd_t), (INT)S5_MSG_BUFSZ(remote_cmd_t, 4));
	if (mbf_remote_cmd < E_OK) return mbf_remote_cmd;
	mbf_obstacle = make_mbf((INT)sizeof(obstacle_t), (INT)S5_MSG_BUFSZ(obstacle_t, 2));
	if (mbf_obstacle < E_OK) return mbf_obstacle;

	cflg.flgatr  = TA_TFIFO | TA_WMUL;
	cflg.iflgptn = 0;
	flg_scan = tk_cre_flg(&cflg);
	return (flg_scan < E_OK) ? flg_scan : E_OK;
}

EXPORT ER subsystem5_start(void)
{
	ID tid = app_start_task(vc_task, S5_PRI_VC, 0, "vc_task");
	if (tid < E_OK) return tid;
	tid = app_start_task(scan_task, S5_PRI_SCAN, 0, "scan_task");
	return (tid < E_OK) ? tid : E_OK;
}
