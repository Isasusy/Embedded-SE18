/*
 *	vc_msgs.h
 *	Subsystem 5 -- message structures shared by S1, S3 and S5 (Design Review Section 6.6).
 *	Plain C only (no kernel headers), so the logic builds on a PC for testing.
 *	Owner: subsystem 5.  Other subsystems include it as "../subsystem5/vc_msgs.h".
 */
#ifndef VC_MSGS_H
#define VC_MSGS_H

#include <stdint.h>
#include <stdbool.h>

/* Barcode Decoder (S3) -> Vehicle Controller : nav_cmd_t  (IF-12, message buffer owned by S5) */
typedef enum { NAV_LEFT, NAV_RIGHT, NAV_STRAIGHT, NAV_UTURN, NAV_STOP } nav_cmd_type_t;
typedef struct {
	int32_t		cmd;		/* nav_cmd_type_t */
	int32_t		code_id;	/* raw decoded value, for the log */
	uint32_t	timestamp_ms;
} nav_cmd_t;

/* Line Follow (S3) -> Vehicle Controller : line_evt_t  (IF-13, message buffer owned by S5) */
typedef enum { LINE_EVT_JUNCTION, LINE_EVT_LOST, LINE_EVT_FOUND } line_evt_type_t;
typedef struct {
	int32_t		type;		/* line_evt_type_t */
	uint32_t	timestamp_ms;
} line_evt_t;

/* Telemetry (S1) -> Vehicle Controller : remote_cmd_t  (IF-15, message buffer owned by S5) */
typedef enum { REMOTE_START, REMOTE_STOP, REMOTE_RESET,
	       REMOTE_SPIN		/* turn calibration, in IDLE only: one TURN of 360 deg on the spot */
} remote_cmd_type_t;
typedef struct {
	int32_t		cmd;		/* remote_cmd_type_t */
} remote_cmd_t;

/* Vehicle Controller -> Telemetry (S1) : robot_status_t  (IF-14, message buffer owned by S1)
   (the design review calls it status_t; renamed to avoid clashing with SDK / lwIP names) */
typedef enum {
	VC_INIT, VC_IDLE, VC_LINE_FOLLOW, VC_BARCODE_ACTION, VC_HUMP_CROSSING,
	VC_OBSTACLE_AVOID, VC_RECOVER, VC_COMPLETE, VC_FAULT
} vc_state_t;

typedef enum {
	FAULT_NONE, FAULT_SELFTEST, FAULT_IMPACT, FAULT_IMU_SENSOR, FAULT_REMOTE_STOP,
	FAULT_WATCHDOG, FAULT_MOTION, FAULT_OBSTACLE, FAULT_LINE_LOST, FAULT_TIMEOUT,
	FAULT_TILT,		/* tipped over / held tilted (S4 FLG_IMU_TILTED / IMU_EVT_TILT) */
	FAULT_LIFTED		/* picked up, no floor under the line sensors (S3 FLG_LINE_LIFTED) */
} fault_code_t;

/* what INIT is still waiting for (robot_status_t.waiting), shown on the dashboard while initialising */
#define VC_WAIT_MOTION	0x1u		/* S2 not READY yet */
#define VC_WAIT_LINE	0x2u		/* S3 line sensors still calibrating */
#define VC_WAIT_IMU	0x4u		/* S4 IMU not found / still calibrating */
#define VC_WAIT_NET	0x8u		/* WiFi / MQTT not connected yet */

typedef struct {
	int32_t		state;		/* vc_state_t */
	int32_t		speed_mm_s;
	float		max_hump_mm;	/* running maximum of the run (FR9) */
	int32_t		fault_code;	/* fault_code_t */
	uint32_t	timestamp_ms;
	uint32_t	waiting;	/* VC_WAIT_* bits, only while state == VC_INIT; 0 = nothing */
} robot_status_t;

/* Obstacle Scan -> Vehicle Controller : obstacle_t  (internal to S5, message buffer) */
typedef struct {
	bool		found;
	float		distance_mm;	/* to the nearest point of the obstacle */
	float		width_mm;
	float		left_clear_mm;	/* free space beside the obstacle, left  */
	float		right_clear_mm;	/* free space beside the obstacle, right */
	int32_t		side;		/* chosen side: +1 left, -1 right, 0 = neither is wide enough */
	float		left_edge_mm;	/* sideways position of the obstacle's left edge,  + = left of the car's centre line */
	float		right_edge_mm;	/* sideways position of the obstacle's right edge, + = left of the car's centre line */
	float		scan_left_mm;	/* SCANNED edges: half a step beyond the outermost rays that echoed, no beam correction */
	float		scan_right_mm;	/* (+ = left). The route is planned from these: the widest the sensor saw it */
	float		depth_mm;	/* planned depth = scanned width x SCAN_PLAN_WIDTH_PCT (the sensor cannot see behind), clamped */
	/* planned avoidance (all 0 if side == 0): turn turn_deg, drive diag_mm until beside the obstacle, turn back
	   -turn_deg; then line finding: drive pass_mm past it, turn -turn_deg toward the line, creep until it is found */
	int32_t		turn_deg;	/* + = left; its sign is the chosen side */
	int32_t		diag_mm;	/* the angled leg */
	int32_t		lat_mm;		/* sideways offset that leg reaches (car centre from the original path) */
	int32_t		pass_mm;	/* parallel leg from beside the obstacle until the car is past its back face */
} obstacle_t;

/* Obstacle Scan -> Telemetry (S1) : the last profiling sweep, raw rays plus the analysis.
   Shared memory, not a message buffer (S1 runs inside the WiFi task on the other core and must never block):
   scan_task.c writes s5_profile under the sequence counter s5_profile_seq (odd while writing), S1 copies it and
   keeps the copy only if the counter was even and unchanged across the copy. */
#define OBST_PROFILE_MAX_RAYS	25	/* 30..150 in 5 deg steps; the JSON must fit lwIP's MQTT output buffer (512 bytes, lwipopts.h) */
typedef struct {
	int32_t		n;				/* rays used */
	int16_t		servo_deg[OBST_PROFILE_MAX_RAYS];	/* absolute servo angle, 90 = straight ahead */
	int16_t		dist_mm[OBST_PROFILE_MAX_RAYS];	/* -1 = no echo */
	obstacle_t	ob;				/* scan_analyze() result */
	uint32_t	timestamp_ms;
} obstacle_profile_t;

#endif /* VC_MSGS_H */
