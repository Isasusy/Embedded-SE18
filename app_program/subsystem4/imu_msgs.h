/*
 *	imu_msgs.h
 *	Subsystem 4 -- message structures the IMU subsystem sends or receives
 *	(Design Review Section 6.6, trimmed to what subsystem 4 uses).
 *
 *	Plain C only (no kernel / Pico headers) so the algorithm builds on a PC for testing.
 *	Fields marked ADDED are small extensions to the 6.6 draft; agree them with the team.
 */
#ifndef IMU_MSGS_H
#define IMU_MSGS_H

#include <stdint.h>
#include <stdbool.h>

/* ---------------- IMU -> Vehicle Controller : imu_evt_t  (IF-06, message buffer) ----------- */
typedef enum {
	MOTION_CLASS_STATIONARY,
	MOTION_CLASS_ACCELERATING,
	MOTION_CLASS_CRUISING,
	MOTION_CLASS_TURNING,
	MOTION_CLASS_CLIMBING,
	MOTION_CLASS_DESCENDING,
	MOTION_CLASS_SUDDEN_IMPACT
} motion_class_t;

typedef enum {
	IMU_EVT_HUMP,		/* one hump finished: peak_mm + max_hump_mm valid        */
	IMU_EVT_IMPACT,		/* collision / shock: Vehicle Controller sets STOP       */
	IMU_EVT_MOTION_CLASS,	/* class changed, or periodic status (turn rate etc.)    */
	IMU_EVT_SENSOR_FAULT	/* repeated I2C failure / calibration failure            */
} imu_evt_type_t;

typedef struct {
	imu_evt_type_t	type;
	float		peak_mm;	/* this hump's estimated height                      */
	float		max_hump_mm;	/* running maximum since last mission reset (FR9)    */
	motion_class_t	motion_class;
	float		turn_rate_dps;	/* ADDED: yaw rate, + = left turn (FR10)             */
	float		pitch_deg;	/* ADDED: + = nose up                                */
	float		accel_g;	/* ADDED: accel magnitude (useful on IMPACT)         */
	uint32_t	timestamp_ms;
} imu_evt_t;

/* ---------------- Motion Control -> IMU : motion_status_t (IF-08, shared data + mutex) ------
 * distance_mm is treated as a CUMULATIVE odometer value. */
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

#endif /* IMU_MSGS_H */
