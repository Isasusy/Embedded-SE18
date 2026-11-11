/*
 *	motion_ctrl.h
 *	Subsystem 2 -- the Motion Control logic (statechart 6.2).  No kernel, no hardware: one function
 *	call per 10 ms loop, so it can be tested on a PC.
 *
 *	States: STOPPED, CRUISE, TURNING, U_TURN, DISTANCE_MOVE, FAULT_STOP.
 */
#ifndef MOTION_CTRL_H
#define MOTION_CTRL_H

#include "motion_msgs.h"

typedef enum {
	MC_ST_STOPPED,
	MC_ST_CRUISE,
	MC_ST_TURNING,
	MC_ST_U_TURN,
	MC_ST_DISTANCE_MOVE,
	MC_ST_FAULT_STOP
} mc_state_t;

typedef enum {
	MC_FAULT_NONE,
	MC_FAULT_STOP_BIT,	/* the STOP event-flag bit was set (impact, fault, remote stop) */
	MC_FAULT_STALL,		/* a wheel got power but never moved                            */
	MC_FAULT_TIMEOUT	/* a TURN / UTURN / MOVE did not finish in time                 */
} mc_fault_t;

typedef struct {
	/* private state -- do not touch, use mc_init() */
	mc_state_t	state, resume;
	float		cruise_speed;
	float		v_l, v_r;		/* filtered wheel speeds, mm/s */
	float		i_l, i_r;		/* PI integrators              */
	float		dl_mm, dr_mm;		/* cumulative wheel distances  */
	float		dist_mm, heading;
	float		hold;			/* heading kept when no line   */
	float		man_h0, man_angle;	/* TURN / U_TURN start heading and signed angle */
	float		man_d0, man_dist, man_speed;	/* MOVE */
	unsigned	man_cycles;
	unsigned	stall_l, stall_r;
	int		pwm_l, pwm_r;
	mc_fault_t	fault;
	bool		cmd_done;
} mc_t;

typedef struct {
	int		dticks_l, dticks_r;	/* encoder edges since the last call, forward positive        */
	const motion_cmd_t *cmd;		/* a new command, or NULL                                     */
	bool		line_valid;		/* line_status_t is fresh and on_line                         */
	float		steer_mm_s;		/* line_status_t.steering_corr                                */
	bool		stop_bit;		/* FLG_MOTION_STOP currently set                              */
	bool		reset;			/* FLG_MOTION_RESET currently set                             */
} mc_in_t;

typedef struct {
	int		pwm_l, pwm_r;		/* -1000..1000, forward positive                              */
	motion_status_t	status;
	mc_state_t	state;
	mc_fault_t	fault;
	bool		fault_new;		/* TRUE in the one call where a stall / timeout fault started */
	bool		reset_consumed;		/* TRUE when this call acted on `reset`                       */
	float		tgt_l, tgt_r;		/* wheel speed targets, for the debug line                    */
} mc_out_t;

void		mc_init(mc_t *m);
void		mc_step(mc_t *m, const mc_in_t *in, mc_out_t *out);
const char *	mc_state_name(mc_state_t s);

#endif /* MOTION_CTRL_H */
