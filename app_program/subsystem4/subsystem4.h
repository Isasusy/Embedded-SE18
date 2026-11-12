/*
 *	subsystem4.h
 *	Subsystem 4 -- IMU and terrain monitoring.  Student ID: 2501101.
 *
 *	Public interface of subsystem 4. Other subsystems include it as "../subsystem4/subsystem4.h".
 *
 *	Replaces the template's example "Monitor" subsystem (and its IF-03 / IF-04 / IF-05 rows).
 */

#ifndef SUBSYSTEM4_H
#define SUBSYSTEM4_H

#include "../app.h"
#include "imu_msgs.h"

#define S4_PRI_IMU		18	/* priority band 18-19 (TEAM.md) */

/* ------------------------------------------------------------------------------------------
 * IF-06  S4 -> Vehicle Controller : imu_evt_t, message buffer owned by the Vehicle Controller.
 *        The VC subsystem creates mbf_imu_evt with at least:
 *            cmbf.maxmsz = S4_IMU_EVT_MAXMSZ;   cmbf.bufsz = S4_IMU_EVT_BUFSZ;
 *        S4 only sends (never blocks). On IMU_EVT_IMPACT / IMU_EVT_SENSOR_FAULT with a full
 *        buffer, S4 discards the oldest queued event so the VC always sees them.
 *        VC: on IMPACT set the STOP flag; on HUMP keep max_hump_mm; on SENSOR_FAULT go to FAULT.
 * ------------------------------------------------------------------------------------------ */
#define S4_IMU_EVT_MAXMSZ	((INT)sizeof(imu_evt_t))
#define S4_IMU_EVT_BUFSZ	(8 * (sizeof(imu_evt_t) + 4))	/* 8 events (4 bytes header each) */
IMPORT ID	mbf_imu_evt;

/* ------------------------------------------------------------------------------------------
 * IF-07  Vehicle Controller <-> S4 : event flag owned by S4 (this subsystem creates it).
 *        VC sets  FLG_IMU_RESET  on a mission RESET -> IMU clears the running max hump (FR9).
 *        S4 sets  FLG_IMU_READY  once the IMU answers and is calibrated (INIT self-test);
 *                 clears it again on a sensor fault.
 *        S4 sets  FLG_IMU_ALIVE  every 10 ms. VC checks it, then tk_clr_flg(flg_imu, ~FLG_IMU_ALIVE),
 *                 before feeding the watchdog (NFR10).
 * ------------------------------------------------------------------------------------------ */
#define FLG_IMU_RESET		0x0001	/* set by VC */
#define FLG_IMU_READY		0x0002	/* set by S4 */
#define FLG_IMU_ALIVE		0x0004	/* set by S4 */
IMPORT ID	flg_imu;

/* ------------------------------------------------------------------------------------------
 * IF-08  Motion Control -> S4 : motion_status_t, shared data under a mutex, owned by Motion Control.
 *        Motion Control updates app_motion every 10 ms:
 *            tk_loc_mtx(mtx_motion, TMO_POL); app_motion = ...; tk_unl_mtx(mtx_motion);
 *        S4 reads it with TMO_POL (never blocks); if the mutex is busy it reuses its last copy.
 *        distance_mm is a cumulative odometer (an unexpected drop just resyncs).
 * ------------------------------------------------------------------------------------------ */
IMPORT ID		mtx_motion;
IMPORT motion_status_t	app_motion;

/* ------------------------------------------------------------------------------------------
 * Private to subsystem 4
 * ------------------------------------------------------------------------------------------ */
typedef struct {		/* snapshot for the bench status line and the tests */
	motion_class_t	cls;
	int		hump_state;
	float		pitch_deg;
	float		turn_rate_dps;
	float		accel_mag_g;
	float		hump_h_mm;
	float		max_hump_mm;
	uint32_t	samples;
	uint32_t	read_fails;
	uint32_t	motion_busy;	/* times the Motion Control mutex was busy */
	uint32_t	exec_us_max;	/* worst-case loop body time          (assumption A6) */
	uint32_t	jitter_us_max;	/* worst-case deviation from 10 ms    (assumption A6) */
	bool		fault;
} IMU_DEBUG;

IMPORT IMU_DEBUG	s4_imu_debug;
IMPORT void		imu_task(INT stacd, void *exinf);

#endif /* SUBSYSTEM4_H */
