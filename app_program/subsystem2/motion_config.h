/*
 *	motion_config.h
 *	Subsystem 2 -- every number you will want to tune.  All values are starting points.
 *	The three you MUST check on the real robot are marked CALIBRATE.
 */
#ifndef MOTION_CONFIG_H
#define MOTION_CONFIG_H

/* ---- timing (NFR2: 10 ms PID loop, jitter < 2 ms) ---- */
#define MC_PERIOD_MS		10

/* ---- geometry (CALIBRATE) ----
 * MC_TICKS_PER_REV counts the edges the driver decodes per wheel revolution (x4 quadrature on A and B).
 * Calibrate: set S2_DEBUG_PRINT 1, turn one wheel exactly one revolution by hand, read "tick" in the status line. */
#define MC_WHEEL_DIAM_MM	65.0f
#define MC_TRACK_MM		130.0f		/* distance between the two wheels' contact points (CALIBRATE) */
#define MC_TICKS_PER_REV	860.0f		/* measured by hand: ~830-890 per revolution (verify with a 1 m push test) */

/* ---- speeds (mm/s) ---- */
#define MC_DEFAULT_SPEED	150.0f		/* used when GO / MOVE carries speed 0 */
#define MC_MAX_SPEED		400.0f
#define MC_MIN_SPEED		40.0f		/* ramp-down floor in the last part of a MOVE */
#define MC_V_FILTER		0.4f		/* measured-speed low-pass (1 = none); speeds are quantised by 10 ms ticks */

/* ---- wheel speed PI + feed-forward (output in 1/1000 duty, -1000..1000) ---- */
#define MC_FF_PWM_MIN		120.0f		/* duty needed just to start the wheel turning (dead band) */
#define MC_FF_PER_MM_S		2.2f		/* duty per mm/s of target speed                           */
#define MC_KP			1.2f		/* duty per mm/s of speed error                            */
#define MC_KI			6.0f		/* duty per (mm/s * s)                                     */
#define MC_INTEG_MAX		300.0f		/* anti-windup clamp on the integral term                  */

/* ---- straight-line correction ---- */
#define MC_KH			4.0f		/* mm/s of wheel speed difference per degree of heading error */
#define MC_STEER_MAX		200.0f		/* clamp on the total speed difference, mm/s                  */

/* ---- turning (TURNING and U_TURN) ---- */
#define MC_TURN_SPEED		120.0f		/* wheel speed while spinning, mm/s                  */
#define MC_TURN_MIN		45.0f		/* slowest wheel speed near the target               */
#define MC_TURN_SLOW_DEG	30.0f		/* start slowing this many degrees before the target */
#define MC_TURN_STOP_AHEAD_DEG	2.0f		/* stop commanding when this close (coasting finishes the rest) */

/* ---- MOVE ---- */
#define MC_MOVE_SLOW_MM		40.0f		/* ramp down over the last this-many mm */
#define MC_MOVE_STOP_AHEAD_MM	1.0f

/* ---- faults ---- */
#define MC_STALL_PWM		200		/* |duty| at or above this with no encoder edge ...        */
#define MC_STALL_CYCLES		100		/* ... for this many loops (1 s) = stalled wheel           */
#define MC_STALL_MIN_TGT	30.0f		/* only when that wheel is asked for at least this speed (mm/s): a wheel
						   slowed to ~0 on a curve, or changing direction, is not stalled */
#define MC_CMD_TIMEOUT_MS	8000		/* a TURN / UTURN / MOVE that takes longer is a fault      */

/* ---- hardware sign conventions (flip if a wheel runs backwards or counts the wrong way) ---- */
#define MC_MOTOR_L_SIGN		(+1)
#define MC_MOTOR_R_SIGN		(+1)
#define MC_ENC_L_SIGN		(-1)
#define MC_ENC_R_SIGN		(+1)

/* ---- bench switches ---- */
#define S2_DEBUG_PRINT		1	/* 4 Hz status line on the console; set 0 at integration            */
#define S2_MOTOR_TEST		0	/* 1 = at power-up, spin each wheel open-loop (no feedback) for 2 s and print the encoder ticks:
					   checks motor wiring, direction and encoder counting. WHEELS OFF THE GROUND. Set 0 afterwards. */
#define S2_BENCH_SCRIPT		0	/* 1 = run a built-in GO/TURN/MOVE/UTURN/STOP sequence when no Vehicle
					   Controller exists.  LIFT THE WHEELS OFF THE TABLE FIRST.           */

#endif /* MOTION_CONFIG_H */
