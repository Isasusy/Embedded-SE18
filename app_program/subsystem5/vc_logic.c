#include <string.h>
#include "vc_logic.h"
#include "vc_config.h"

const char *vc_state_name(int s)
{
	static const char *const n[] = { "INIT", "IDLE", "LINE_FOLLOW", "BARCODE_ACTION", "HUMP_CROSSING",
					 "OBSTACLE_AVOID", "RECOVER", "COMPLETE", "FAULT" };
	return (unsigned)s < 9 ? n[s] : "?";
}

void vc_init(vc_t *v)
{
	memset(v, 0, sizeof *v);
	v->state = VC_INIT;
}

/* ---- helpers that fill the single motion command allowed per tick ---- */
static void cmd(vc_out_t *o, motion_cmd_type_t t, int speed, int angle, int dist)
{
	o->send_motion = true;
	o->motion = (motion_cmd_t){ t, speed, angle, dist };
}

static void enter(vc_t *v, vc_out_t *o, vc_state_t s, uint32_t now)
{
	v->state = s;
	v->t_state = now;
	v->settle = 0;
	o->send_status = true;
}

static void enter_fault(vc_t *v, vc_out_t *o, fault_code_t f, uint32_t now)
{
	v->fault = f;
	o->stop_set = true;
	cmd(o, MOTION_CMD_STOP, 0, 0, 0);
	enter(v, o, VC_FAULT, now);
}

static void go(vc_t *v, vc_out_t *o, int speed)
{
	v->speed_cmd = speed;
	cmd(o, MOTION_CMD_GO, speed, 0, 0);
}

static void manoeuvre(vc_t *v, vc_out_t *o, motion_cmd_type_t t, int angle, int dist)
{
	cmd(o, t, VC_CRUISE_SPEED, angle, dist);
	v->settle = VC_SETTLE_TICKS;
}


/* OBSTACLE_AVOID route, planned by scan_analyze() from the profile (turn angle from the obstacle's width and edge):
     avoidance:    0 TURN turn_deg, 1 MOVE diag_mm (ends past the obstacle), 2 TURN -turn_deg (parallel again)
     line finding: 3 MOVE pass_mm (past the back face, depth = width), 4 TURN -turn_deg (toward the line),
                   then creep at VC_RECOVER_SPEED, heading held, until a line sensor finds the line -> LINE_FOLLOW */
#define AVOID_STEPS 5

/* the next piece of the current MOVE: at most VC_MOVE_MAX_MM, so S2 never times a long one out */
static void move_part(vc_t *v, vc_out_t *o)
{
	int d = v->move_left > VC_MOVE_MAX_MM ? VC_MOVE_MAX_MM : v->move_left;
	v->move_left -= d;
	manoeuvre(v, o, MOTION_CMD_MOVE, 0, d);
}

static void avoid_step(vc_t *v, vc_out_t *o, int i)
{
	int t = (int)v->ob.turn_deg;					/* + = round the left side */

	switch (i) {
	case 0: manoeuvre(v, o, MOTION_CMD_TURN, t, 0); break;
	case 1: v->move_left = (int)v->ob.diag_mm; move_part(v, o); break;
	case 2: manoeuvre(v, o, MOTION_CMD_TURN, -t, 0); break;
	case 3: v->move_left = (int)v->ob.pass_mm; move_part(v, o); break;	/* 0 = done at once */
	case 4: manoeuvre(v, o, MOTION_CMD_TURN, -t, 0); break;
	}
}

/* the creep back to the line covers about diag_mm; give up after that plus VC_REJOIN_EXTRA_MM at creep speed */
static uint32_t search_ms(const vc_t *v)
{
	return 1000u + (uint32_t)(1000 * ((int)v->ob.diag_mm + VC_REJOIN_EXTRA_MM) / VC_RECOVER_SPEED);
}

static void resume(vc_t *v, vc_out_t *o, uint32_t now)
{
	go(v, o, VC_CRUISE_SPEED);
	enter(v, o, VC_LINE_FOLLOW, now);
}

/* (re)start the CONFIRM wait: the obstacle must now stay VC_OBST_STILL_MS */
static void confirm_start(vc_t *v, int mm, uint32_t now)
{
	v->sub = OB_CONFIRM;
	v->t_close = v->t_still = now;
	v->still_min = v->still_max = mm > 0 ? mm : 0;
}

/* something close ahead while driving: stop and start watching whether it stays */
static void obstacle_stop(vc_t *v, vc_out_t *o, const vc_in_t *in, uint32_t now)
{
	cmd(o, MOTION_CMD_STOP, 0, 0, 0);
	enter(v, o, VC_OBSTACLE_AVOID, now);
	confirm_start(v, in->obstacle_mm, now);
}

/* a still object keeps its distance; one that moves more than VC_OBST_STILL_TOL_MM restarts the wait */
static void still_track(vc_t *v, int mm, uint32_t now)
{
	if (mm <= 0) return;
	if (v->still_min <= 0) { v->still_min = v->still_max = mm; return; }
	if (mm < v->still_min) v->still_min = mm;
	if (mm > v->still_max) v->still_max = mm;
	if (v->still_max - v->still_min > VC_OBST_STILL_TOL_MM) {
		v->t_still = now;
		v->still_min = v->still_max = mm;
	}
}

/* no usable side (or no profile): bench = watch it again and re-profile; mission = fault */
static void no_way(vc_t *v, vc_out_t *o, const vc_in_t *in, uint32_t now)
{
#if VC_BENCH_LINE_ONLY
	(void)o;
	confirm_start(v, in->obstacle_mm, now);
#else
	(void)in;
	enter_fault(v, o, FAULT_OBSTACLE, now);
#endif
}

/* A junction: stop, pause VC_JUNCTION_PAUSE_MS, then do the remembered barcode command (drive to the junction centre
   and turn), or drive straight on when there is none. */
static void junction_action(vc_t *v, vc_out_t *o, uint32_t now)
{
	v->junction_cmd = v->nav_has ? v->nav_cmd : -1;
	v->nav_has = false;
	if (v->junction_cmd < 0 && !VC_JUNCTION_PAUSE_ALWAYS)
		return;
	cmd(o, MOTION_CMD_STOP, 0, 0, 0);
	enter(v, o, VC_BARCODE_ACTION, now);
	v->sub = 0;					/* 0 = pausing */
	v->t_sub = now;
	v->settle = 0;
}

void vc_step(vc_t *v, const vc_in_t *in, vc_out_t *o)
{
	uint32_t now = in->now_ms;
	memset(o, 0, sizeof *o);

	if (!v->started) {
		v->started = true;
		v->t_state = v->t_last_alive = v->t_last_status = now;
		o->send_status = true;
	}
	if (in->all_alive) v->t_last_alive = now;
	o->feed_watchdog = (now - v->t_last_alive) <= VC_ALIVE_MS;

	/* running maximum hump height of the whole run (FR9), cleared only by a reset */
	if (in->imu && in->imu->type == IMU_EVT_HUMP && in->imu->max_hump_mm > v->max_hump_mm)
		v->max_hump_mm = in->imu->max_hump_mm;

	if (in->remote && in->remote->cmd == REMOTE_RESET) {
		o->stop_clear_reset = true;			/* clear STOP first, then RESET (task does it in this order) */
		cmd(o, MOTION_CMD_STOP, 0, 0, 0);
		v->fault = FAULT_NONE;
		v->max_hump_mm = 0.0f;
		v->started = true;
		enter(v, o, VC_INIT, now);
		goto status;
	}
	if (v->state != VC_FAULT && v->state != VC_INIT) {
		if (in->remote && in->remote->cmd == REMOTE_STOP) { enter_fault(v, o, FAULT_REMOTE_STOP, now); goto status; }
		if (VC_REQUIRE_IMU && in->imu && in->imu->type == IMU_EVT_IMPACT)   { enter_fault(v, o, FAULT_IMPACT, now); goto status; }
		if (VC_REQUIRE_IMU && in->imu && in->imu->type == IMU_EVT_SENSOR_FAULT) { enter_fault(v, o, FAULT_IMU_SENSOR, now); goto status; }
		if (in->motion_fault)                             { enter_fault(v, o, FAULT_MOTION, now); goto status; }
		if (v->state != VC_COMPLETE && (now - v->t_last_alive) > VC_ALIVE_MS) {
			enter_fault(v, o, FAULT_WATCHDOG, now); goto status;
		}
	}

	switch (v->state) {
	case VC_INIT:
		if (in->all_ready)
			enter(v, o, VC_IDLE, now);
		else if (now - v->t_state > VC_SELFTEST_MS)
			enter_fault(v, o, FAULT_SELFTEST, now);
		break;

	case VC_IDLE:
#if VC_BENCH_LINE_ONLY
		if (v->bench_restart && now - v->t_state >= 300) {
			v->bench_restart = false;
			go(v, o, VC_CRUISE_SPEED);
			enter(v, o, VC_LINE_FOLLOW, now);
			break;
		}
#endif
		if (in->remote && in->remote->cmd == REMOTE_START) {
			go(v, o, VC_CRUISE_SPEED);
			enter(v, o, VC_LINE_FOLLOW, now);
		}
		break;

	case VC_LINE_FOLLOW:
#if VC_BENCH_LINE_ONLY
		if (in->obstacle_close) {
			obstacle_stop(v, o, in, now);
			break;
		}
#if VC_BENCH_BARCODES
		if (in->nav && in->nav->cmd != NAV_STOP) {
			v->nav_has = true;
			v->nav_cmd = in->nav->cmd;
			v->t_nav = now;
		}
		if (v->nav_has && VC_NAV_EXPIRE_MS > 0 && now - v->t_nav > VC_NAV_EXPIRE_MS)
			v->nav_has = false;
		if (in->line && in->line->type == LINE_EVT_JUNCTION)
			junction_action(v, o, now);
#endif
		break;
#endif
		if (in->nav) {
			if (in->nav->cmd == NAV_STOP) {
				cmd(o, MOTION_CMD_STOP, 0, 0, 0);
				enter(v, o, VC_COMPLETE, now);
				break;
			}
			v->nav_has = true;
			v->nav_cmd = in->nav->cmd;
			v->t_nav = now;
		}
		if (v->nav_has && VC_NAV_EXPIRE_MS > 0 && now - v->t_nav > VC_NAV_EXPIRE_MS)
			v->nav_has = false;

		if (in->obstacle_close) {
			obstacle_stop(v, o, in, now);
		} else if (in->line && in->line->type == LINE_EVT_JUNCTION && (v->nav_has || VC_JUNCTION_PAUSE_ALWAYS)) {
			junction_action(v, o, now);
		} else if (in->line && in->line->type == LINE_EVT_LOST) {
			go(v, o, VC_RECOVER_SPEED);
			enter(v, o, VC_RECOVER, now);
		} else if (in->imu && in->imu->type == IMU_EVT_MOTION_CLASS && in->imu->motion_class == MOTION_CLASS_CLIMBING) {
			go(v, o, VC_HUMP_SPEED);
			enter(v, o, VC_HUMP_CROSSING, now);
		}
		break;

	case VC_BARCODE_ACTION:
		if (v->sub == 0) {				/* PAUSE at the junction */
			if (in->nav && in->nav->cmd != NAV_STOP && v->junction_cmd < 0)
				v->junction_cmd = in->nav->cmd;	/* the read finished as we reached the junction: use it */
			if (now - v->t_sub < VC_JUNCTION_PAUSE_MS) break;
			if (v->junction_cmd < 0 || v->junction_cmd == NAV_STRAIGHT) {
				go(v, o, VC_CRUISE_SPEED);
				enter(v, o, VC_LINE_FOLLOW, now);
			} else {				/* drive to the junction centre first */
				manoeuvre(v, o, MOTION_CMD_MOVE, 0, VC_JUNCTION_ADVANCE_MM);
				v->sub = 1;
			}
			break;
		}
		if (v->settle > 0) { v->settle--; break; }
		if (in->motion_done) {
			if (v->sub == 1) {			/* at the junction centre: now make the turn */
				v->sub = 2;
				switch (v->junction_cmd) {
				case NAV_LEFT:	manoeuvre(v, o, MOTION_CMD_TURN, +VC_TURN_ANGLE, 0); break;
				case NAV_RIGHT:	manoeuvre(v, o, MOTION_CMD_TURN, -VC_TURN_ANGLE, 0); break;
				default:	manoeuvre(v, o, MOTION_CMD_UTURN, 0, 0); break;
				}
			} else {
				go(v, o, VC_CRUISE_SPEED);
				enter(v, o, VC_LINE_FOLLOW, now);
			}
		}
		break;

	case VC_HUMP_CROSSING:
		if ((in->imu && in->imu->type == IMU_EVT_HUMP) || now - v->t_state > VC_HUMP_TIMEOUT_MS) {
			go(v, o, VC_CRUISE_SPEED);
			enter(v, o, VC_LINE_FOLLOW, now);
		}
		break;

	case VC_RECOVER:
		if (in->line && in->line->type == LINE_EVT_FOUND) {
			go(v, o, VC_CRUISE_SPEED);
			enter(v, o, VC_LINE_FOLLOW, now);
		} else if (now - v->t_state > VC_RECOVER_TIMEOUT_MS) {
			enter_fault(v, o, FAULT_LINE_LOST, now);
		}
		break;

	case VC_OBSTACLE_AVOID:
		if (v->sub == OB_CONFIRM) {			/* STOPPED: does it stay, at a steady distance? */
			if (in->obstacle_close || in->obstacle_present) {	/* still there (hold distance, hysteresis) */
				v->t_close = now;
				if (in->obstacle_close)
					still_track(v, in->obstacle_mm, now);
			} else if (now - v->t_close >= VC_OBST_CLEAR_MS) {
				resume(v, o, now);		/* it went away */
				break;
			}
			if (now - v->t_still >= VC_OBST_STILL_MS) {
				o->scan_request = true;		/* a still object: profile it */
				v->sub = OB_PROFILE;
				v->t_sub = now;
			}
		} else if (v->sub == OB_PROFILE) {		/* WAIT FOR THE SCAN */
			if (in->obst) {
				v->ob = *in->obst;
				if (!v->ob.found) {
					resume(v, o, now);
				} else if (!VC_OBST_BYPASS) {
					v->sub = OB_HOLD;
					v->t_close = now;
				} else if (v->ob.side == 0) {
					no_way(v, o, in, now);
				} else {
					v->sub = OB_BYPASS; v->step = 0; v->move_left = 0;
					v->sp_line_found = false;
					avoid_step(v, o, 0);
				}
			} else if (now - v->t_sub > VC_SCAN_TIMEOUT_MS) {
				no_way(v, o, in, now);
			}
		} else if (v->sub == OB_BYPASS) {		/* AVOID, then turn toward the line: one step at a time */
			if (in->line && in->line->type == LINE_EVT_FOUND && v->step >= 4)
				v->sp_line_found = true;	/* found while turning toward it */
			if (v->settle > 0) { v->settle--; break; }
			if (in->motion_done) {
				if (v->move_left > 0) {
					move_part(v, o);	/* the rest of a long MOVE */
				} else if (++v->step < AVOID_STEPS) {
					avoid_step(v, o, v->step);
				} else if (v->sp_line_found) {	/* REJOIN: already on the line */
					resume(v, o, now);
				} else {			/* LINE FINDING: creep toward it, heading held */
					go(v, o, VC_RECOVER_SPEED);
					v->sub = OB_SEARCH;
					v->t_sub = now;
				}
			}
		} else if (v->sub == OB_SEARCH) {		/* LINE FINDING: a sensor reaches the line -> follow it */
			if (in->line && in->line->type == LINE_EVT_FOUND) {
				resume(v, o, now);
			} else if (now - v->t_sub > search_ms(v)) {
				enter_fault(v, o, FAULT_LINE_LOST, now);
			}
		} else {					/* HOLD (VC_OBST_BYPASS 0): wait until it has gone */
			if (in->obstacle_close || in->obstacle_present) {
				v->t_close = now;
			} else if (now - v->t_close >= VC_OBST_CLEAR_MS) {
				resume(v, o, now);
			}
		}
		break;

	case VC_COMPLETE:
	case VC_FAULT:
#if VC_BENCH_LINE_ONLY
		/* bench: a stalled wheel (e.g. a weak battery on a tight curve) must not end the run: reset and go again */
		if (v->fault == FAULT_MOTION && now - v->t_state >= VC_BENCH_RETRY_MS) {
			o->stop_clear_reset = true;
			cmd(o, MOTION_CMD_STOP, 0, 0, 0);
			v->fault = FAULT_NONE;
			v->bench_restart = true;
			enter(v, o, VC_IDLE, now);
		}
#endif
		break;
	default:
		break;
	}

status:
	/* stopped for an obstacle (watching it or profiling it): the scan task tracks it with the hold distance */
	o->scan_hold = v->state == VC_OBSTACLE_AVOID && (v->sub == OB_CONFIRM || v->sub == OB_PROFILE || v->sub == OB_HOLD);
	if (now - v->t_last_status >= VC_STATUS_MS) {
		v->t_last_status = now;
		o->send_status = true;
	}
	if (o->send_status) {
		v->t_last_status = now;
		o->status.state = v->state;
		o->status.speed_mm_s = in->speed_mm_s;
		o->status.max_hump_mm = v->max_hump_mm;
		o->status.fault_code = v->fault;
		o->status.timestamp_ms = now;
	}
}
