/*
 *	vc_logic.h / vc_logic.c -- Vehicle Controller state machine (system statechart, Figure 5.1).
 *	No kernel or hardware calls: one vc_step() per 20 ms tick, testable on a PC.
 *
 *	A barcode is remembered; its turn is made at the NEXT JUNCTION (drive VC_JUNCTION_ADVANCE_MM, then turn).
 *	An obstacle: STOP, CONFIRM it stays (VC_OBST_STILL_MS at a steady distance; it going away = drive on), PROFILE it
 *	with a servo sweep, AVOID it on the more open side (turn by an angle planned from its width, drive until beside it,
 *	turn back), then LINE FINDING (past it, turn toward the line, creep until found) -> LINE_FOLLOW. No way round
 *	(no usable side, or no profile in time): REVERSE VC_OBST_RETRY_REVERSE_MM and PROFILE again, up to
 *	VC_OBST_RETRIES times; then bench = watch and re-profile, mission = FAULT_OBSTACLE.
 *	INIT -> IDLE -> LINE_FOLLOW <-> BARCODE_ACTION / HUMP_CROSSING / OBSTACLE_AVOID / RECOVER
 *	-> COMPLETE; FAULT from anywhere. A stop or reset can arrive in any state and takes priority.
 */
#ifndef VC_LOGIC_H
#define VC_LOGIC_H

#include "vc_msgs.h"
#include "../subsystem2/motion_msgs.h"
#include "../subsystem4/imu_msgs.h"

/* OBSTACLE_AVOID sub-states (vc_t.sub) */
enum { OB_CONFIRM, OB_PROFILE, OB_BYPASS, OB_SEARCH, OB_HOLD, OB_REVERSE };

typedef struct {
	/* private */
	vc_state_t	state;
	fault_code_t	fault;
	uint32_t	t_state, t_last_alive, t_last_status;
	float		max_hump_mm;
	int		speed_cmd;
	int		step, settle;			/* OBSTACLE_AVOID sequence position, settle ticks */
	int		move_left;			/* OBSTACLE_AVOID: mm of the current MOVE still to send (VC_MOVE_MAX_MM pieces) */
	int		sub;
	int		ob_retries;			/* OBSTACLE_AVOID: reverse-and-re-profile tries used on this obstacle */
	obstacle_t	ob;
	uint32_t	t_close;			/* OBSTACLE_AVOID: last tick the obstacle was seen close */
	uint32_t	t_still;			/* OBSTACLE_AVOID: start of the current steady-distance stretch */
	int		still_min, still_max;		/* tracked distance band of that stretch (0 = none yet) */
	bool		started, sp_line_found;
	uint32_t	t_ready;			/* INIT: when all subsystems were first READY (0 = not yet) */
	bool		bench_restart;			/* bench: drive on again after an automatic reset */
	bool		nav_has;			/* a barcode command is waiting for the next junction */
	int		nav_cmd;			/* nav_cmd_type_t of that command */
	int		junction_cmd;			/* what this junction does: nav_cmd_type_t, or -1 = straight on */
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
	int	obstacle_mm;				/* distance the scan task is tracking, <= 0 = not tracking */
	bool	obstacle_present;			/* scan task: the obstacle it tracks is still there (SCAN_HOLD_MM) */
	bool	all_ready;				/* S2, S3, S4 reported READY */
	uint32_t waiting;				/* VC_WAIT_MOTION / LINE / IMU bits of those not READY yet */
	bool	net_ready;				/* MQTT connected (S1's s1_online_count moving) */
	bool	all_alive;				/* S2, S3, S4 each set ALIVE since the last tick */
	bool	tilted;					/* S4: tipped over / held tilted (FLG_IMU_TILTED); an IMU_EVT_TILT in imu counts too */
	bool	lifted;					/* S3: no floor under the line sensors (FLG_LINE_LIFTED) */
	int	speed_mm_s;				/* measured speed, for the status report */
} vc_in_t;

typedef struct {
	bool		send_motion;
	motion_cmd_t	motion;
	bool		stop_set;			/* set FLG_MOTION_STOP and hold it */
	bool		stop_clear_reset;		/* clear STOP, then set FLG_MOTION_RESET and FLG_IMU_RESET */
	bool		scan_request;
	bool		scan_hold;			/* the car is stopped for an obstacle: track it out to SCAN_HOLD_MM */
	bool		send_status;
	robot_status_t	status;
	bool		feed_watchdog;
} vc_out_t;

void		vc_init(vc_t *v);
void		vc_step(vc_t *v, const vc_in_t *in, vc_out_t *out);
const char *	vc_state_name(int s);

#endif
