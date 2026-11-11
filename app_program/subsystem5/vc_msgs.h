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
typedef enum { REMOTE_START, REMOTE_STOP, REMOTE_RESET } remote_cmd_type_t;
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
	FAULT_WATCHDOG, FAULT_MOTION, FAULT_OBSTACLE, FAULT_LINE_LOST, FAULT_TIMEOUT
} fault_code_t;

typedef struct {
	int32_t		state;		/* vc_state_t */
	int32_t		speed_mm_s;
	float		max_hump_mm;	/* running maximum of the run (FR9) */
	int32_t		fault_code;	/* fault_code_t */
	uint32_t	timestamp_ms;
} robot_status_t;

/* Obstacle Scan -> Vehicle Controller : obstacle_t  (internal to S5, message buffer) */
typedef struct {
	bool		found;
	float		distance_mm;	/* to the nearest point of the obstacle */
	float		width_mm;
	float		left_clear_mm;	/* free space beside the obstacle, left  */
	float		right_clear_mm;	/* free space beside the obstacle, right */
	int32_t		side;		/* chosen side: +1 left, -1 right, 0 = neither is wide enough */
} obstacle_t;

#endif /* VC_MSGS_H */
