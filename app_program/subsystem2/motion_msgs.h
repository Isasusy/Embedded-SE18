/*
 *	motion_msgs.h
 *	Subsystem 2 -- message structures Motion Control sends or receives
 *	(Design Review Section 6.6).  Plain C only, so the control logic builds on a PC for testing.
 *
 *	motion_status_t is IDENTICAL (same fields, same order) to the copy subsystem 4 uses in imu_msgs.h.
 *	If one file ever includes both headers, delete one of the two definitions.
 */
#ifndef MOTION_MSGS_H
#define MOTION_MSGS_H

#include <stdint.h>
#include <stdbool.h>

/* ---------------- Vehicle Controller -> Motion Control : motion_cmd_t (IF-09, message buffer) ---- */
typedef enum {
	MOTION_CMD_GO,		/* drive straight at speed_mm_s (0 = default speed); line steering applies */
	MOTION_CMD_STOP,	/* PWM = 0, state STOPPED                                                  */
	MOTION_CMD_TURN,	/* spin on the spot by angle_deg (+ = left / counter-clockwise)            */
	MOTION_CMD_UTURN,	/* spin 180 degrees on the spot                                            */
	MOTION_CMD_MOVE		/* drive distance_mm at speed_mm_s, heading held (negative = reverse)      */
} motion_cmd_type_t;

typedef struct {
	motion_cmd_type_t	type;
	int32_t			speed_mm_s;
	int32_t			angle_deg;
	int32_t			distance_mm;
} motion_cmd_t;

/* ---------------- Line Follow -> Motion Control : line_status_t (IF-11, shared data + mutex) ----
 * steering_corr is a WHEEL SPEED DIFFERENCE in mm/s: right wheel gets +corr/2, left wheel -corr/2,
 * so a POSITIVE value turns the robot LEFT.  Agree this sign with the Line Follow owner.            */
typedef struct {
	float	offset;		/* line offset (Line Follow's units, informational here) */
	float	steering_corr;	/* mm/s, + = steer left                                  */
	bool	on_line;	/* false = line lost: Motion Control holds its heading   */
} line_status_t;

/* ---------------- Motion Control -> Vehicle Controller and IMU : motion_status_t (IF-08) --------
 * Shared data under mutex mtx_motion.  distance_mm is a CUMULATIVE odometer (forward positive).
 * heading_deg is the encoder-derived heading, + = left / counter-clockwise, 0 at boot.
 * cmd_done latches TRUE when a TURN / UTURN / MOVE finishes and is cleared by the next command.     */
#ifndef MOTION_STATUS_T_DEFINED
#define MOTION_STATUS_T_DEFINED
typedef struct {
	float	speed_l;	/* mm/s */
	float	speed_r;	/* mm/s */
	float	distance_mm;
	float	heading_deg;
	bool	cmd_done;
} motion_status_t;
#endif

#endif /* MOTION_MSGS_H */
