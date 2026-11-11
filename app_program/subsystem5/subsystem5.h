/*
 *	subsystem5.h
 *	Subsystem 5 -- Vehicle Controller + Obstacle Scan (Student ID 2503464 per the design review).
 *	Other subsystems include it as "../subsystem5/subsystem5.h".
 */
#ifndef SUBSYSTEM5_H
#define SUBSYSTEM5_H

#include "../app.h"
#include "vc_msgs.h"
#include "../subsystem4/imu_msgs.h"		/* imu_evt_t */

#define S5_PRI_VC		20		/* priority band 20-21 (TEAM.md) */
#define S5_PRI_SCAN		21

/* ---- message buffers owned by the Vehicle Controller: the other subsystems only SEND to them ---- */
#define S5_MSG_BUFSZ(T, depth)	((depth) * (sizeof(T) + 4))

/* IF-06  S4 -> VC   imu_evt_t      depth 8  (S4's header: S4_IMU_EVT_MAXMSZ / S4_IMU_EVT_BUFSZ) */
IMPORT ID	mbf_imu_evt;
/* IF-12  S3 -> VC   nav_cmd_t      depth 4 */
IMPORT ID	mbf_nav_cmd;
/* IF-13  S3 -> VC   line_evt_t     depth 4 */
IMPORT ID	mbf_line_evt;
/* IF-15  S1 -> VC   remote_cmd_t   depth 4 */
IMPORT ID	mbf_remote_cmd;

/* IF-14  VC -> S1   robot_status_t, message buffer owned by S1 (S5 only sends). Weak default 0 = S1 absent. */
IMPORT ID	mbf_status;

/* IF-10 / IF-07 / IF-16: the VC uses the flag objects of S2, S4 and S3 (see their headers):
     flg_motion: STOP, RESET (VC sets), READY, ALIVE, FAULT (S2 sets)
     flg_imu:    RESET (VC sets), READY, ALIVE (S4 sets)
     flg_line:   READY, ALIVE (S3 sets)
   and reads app_motion under mtx_motion (IF-08). */

/* Obstacle Scan <-> VC (internal to S5) */
#define FLG_SCAN_REQUEST	0x1		/* VC -> scan_task: start a sweep           */
#define FLG_SCAN_CLOSE		0x2		/* scan_task -> VC: something close ahead   */
IMPORT ID	flg_scan;
IMPORT ID	mbf_obstacle;			/* scan_task -> VC: obstacle_t, depth 2     */

IMPORT void	vc_task(INT stacd, void *exinf);
IMPORT void	scan_task(INT stacd, void *exinf);

#endif
