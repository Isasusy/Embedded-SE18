/*
 *	subsystem2.h
 *	Subsystem 2 -- Motion Control (Student ID 2500297 per the design review).
 *
 *	Public interface of subsystem 2.  Other subsystems include it as "../subsystem2/subsystem2.h".
 *	Replaces the template's example Producer (sem_slots / mpf_samples / SAMPLE_MSG are gone).
 */
#ifndef SUBSYSTEM2_H
#define SUBSYSTEM2_H

#include "../app.h"
#include "motion_msgs.h"

#define S2_PRI_ENC		14	/* encoder polling task, 1 ms (band 14-15, TEAM.md) */
#define S2_PRI_MOTION		15	/* control loop, 10 ms; below enc_task so counts are fresh */

/* ---------------------------------------------------------------------
 *  IF-09  Vehicle Controller -> Motion Control : motion_cmd_t
 *  Message buffer owned by S2.  Vehicle Controller sends with tk_snd_mbf(mbf_motion_cmd, &cmd,
 *  sizeof(cmd), TMO_POL).  Motion Control reads one command per 10 ms loop.
 * --------------------------------------------------------------------- */
#define S2_CMD_MAXMSZ		((INT)sizeof(motion_cmd_t))
#define S2_CMD_BUFSZ		(4 * (sizeof(motion_cmd_t) + 4))	/* depth 4 (Section 6.6) */
IMPORT ID	mbf_motion_cmd;

/* ---------------------------------------------------------------------
 *  IF-10  Vehicle Controller <-> Motion Control : event flag flg_motion (owned by S2)
 *    FLG_MOTION_STOP   VC sets on impact / fault / remote stop and CLEARS it when it is safe to go again.
 *                      While set, motors are off (FAULT_STOP) within one 10 ms loop (NFR6).
 *    FLG_MOTION_RESET  VC sets on a mission reset (clear STOP first).  Motion Control clears it itself.
 *    FLG_MOTION_READY  S2 sets once motors and encoders are initialised.
 *    FLG_MOTION_ALIVE  S2 sets every loop; the VC checks and clears it before feeding the watchdog (NFR10).
 *    FLG_MOTION_FAULT  S2 sets when a wheel stalls or a manoeuvre times out; the VC reacts (-> FAULT)
 *                      and clears it.
 * --------------------------------------------------------------------- */
#define FLG_MOTION_STOP		0x01
#define FLG_MOTION_RESET	0x02
#define FLG_MOTION_READY	0x04
#define FLG_MOTION_ALIVE	0x08
#define FLG_MOTION_FAULT	0x10
IMPORT ID	flg_motion;

/* ---------------------------------------------------------------------
 *  IF-08  Motion Control -> Vehicle Controller and IMU : motion_status_t, shared data + mutex (owned by S2)
 *  Readers: tk_loc_mtx(mtx_motion, TMO_POL); copy app_motion; tk_unl_mtx(mtx_motion).
 * --------------------------------------------------------------------- */
IMPORT ID		mtx_motion;
IMPORT motion_status_t	app_motion;

/* ---------------------------------------------------------------------
 *  IF-11  Line Follow -> Motion Control : line_status_t, shared data + mutex (owned by S3, proposed names)
 *  subsystem 2 only READS these.  Weak defaults in subsystem2.c let S2 run without Line Follow.
 * --------------------------------------------------------------------- */
IMPORT ID		mtx_line;
IMPORT line_status_t	app_line;

/* Private to subsystem 2 */
typedef struct {			/* read-only snapshot for the debugger / tests */
	int	state, fault;
	int	enc_l, enc_r;
	int	pwm_l, pwm_r;
	unsigned exec_max_us, late_max_us;
} MOTION_DEBUG;
IMPORT MOTION_DEBUG s2_motion_debug;

IMPORT void	motion_task(INT stacd, void *exinf);
IMPORT void	enc_task(INT stacd, void *exinf);

#endif /* SUBSYSTEM2_H */
