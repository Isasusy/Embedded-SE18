/*
 *	vc_logic.h / vc_logic.c -- Vehicle Controller state machine (system statechart, Figure 5.1).
 *	No kernel or hardware calls: one vc_step() per 20 ms tick, testable on a PC.
 *
 *	A barcode is remembered; its turn is made at the NEXT JUNCTION (drive VC_JUNCTION_ADVANCE_MM, then turn).
 *	INIT -> IDLE -> LINE_FOLLOW <-> BARCODE_ACTION / HUMP_CROSSING / OBSTACLE_AVOID / RECOVER
 *	-> COMPLETE; FAULT from anywhere. A stop or reset can arrive in any state and takes priority.
 */
#ifndef VC_LOGIC_H
#define VC_LOGIC_H

#include "vc_msgs.h"
#include "../subsystem2/motion_msgs.h"
#include "../subsystem4/imu_msgs.h"

typedef struct {
	/* private */
	vc_state_t	state;
	fault_code_t	fault;
	uint32_t	t_state, t_last_alive, t_last_status;
	float		max_hump_mm;
	int		speed_cmd;
	int		step, settle;			/* OBSTACLE_AVOID sequence position, settle ticks */
	int		sub;				/* OBSTACLE_AVOID: 0 wait scan, 1 bypass steps, 2 search line */
	obstacle_t	ob;
	bool		started, sp_line_found;
	bool		bench_restart;			/* bench: drive on again after an automatic reset */
	bool		nav_has;			/* a barcode command is waiting for the next junction */
	int		nav_cmd;			/* nav_cmd_type_t of that command */
	uint32_t	t_nav;
	uint32_t	t_sub;
} vc_t;

typedef struct {
	uint32_t		now_ms;
	const remote_cmd_t	*remote;
	const nav_cmd_t		*nav;
	const line_evt_t	*line;
	const imu_evt_t		*imu;
	const obstacle_t	*obst;			/* scan result, if one arrived */
	bool	motion_done;				/* motion_status_t.cmd_done */
	bool	motion_fault;				/* FLG_MOTION_FAULT seen */
	bool	obstacle_close;				/* scan task saw something close ahead */
	bool	all_ready;				/* S2, S3, S4 reported READY */
	bool	all_alive;				/* S2, S3, S4 each set ALIVE since the last tick */
	int	speed_mm_s;				/* measured speed, for the status report */
} vc_in_t;

typedef struct {
	bool		send_motion;
	motion_cmd_t	motion;
	bool		stop_set;			/* set FLG_MOTION_STOP and hold it */
	bool		stop_clear_reset;		/* clear STOP, then set FLG_MOTION_RESET and FLG_IMU_RESET */
	bool		scan_request;
	bool		send_status;
	robot_status_t	status;
	bool		feed_watchdog;
} vc_out_t;

void		vc_init(vc_t *v);
void		vc_step(vc_t *v, const vc_in_t *in, vc_out_t *out);
const char *	vc_state_name(int s);

#endif
