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

/* OBSTACLE_AVOID bypass: turn aside, drive past, turn back toward the line (statechart 6.4 BYPASS / REJOIN) */
#define BYPASS_STEPS 7
static void bypass_step(vc_t *v, vc_out_t *o, int i)
{
	int s = v->ob.side;						/* +1 = bypass on the left */
	int lat = (int)(v->ob.width_mm / 2) + VC_ROBOT_HALF_W_MM + VC_SIDE_MARGIN_MM;
	int fwd = (int)v->ob.distance_mm + VC_OBST_DEPTH_MM + VC_ROBOT_LEN_MM;

	switch (i) {
	case 0: manoeuvre(v, o, MOTION_CMD_TURN, s * 90, 0); break;
	case 1: manoeuvre(v, o, MOTION_CMD_MOVE, 0, lat); break;
	case 2: manoeuvre(v, o, MOTION_CMD_TURN, -s * 90, 0); break;
	case 3: manoeuvre(v, o, MOTION_CMD_MOVE, 0, fwd); break;
	case 4: manoeuvre(v, o, MOTION_CMD_TURN, -s * 90, 0); break;
	case 5: manoeuvre(v, o, MOTION_CMD_MOVE, 0, lat); break;
	case 6: manoeuvre(v, o, MOTION_CMD_TURN, s * 90, 0); break;
	}
}

/* A junction: stop, pause VC_JUNCTION_PAUSE_MS, then do the remembered barcode command (drive to the junction centre
   and turn), or drive straight on when there is none. */
static void junction_action(vc_t *v, vc_out_t *o, uint32_t now)
{
	v->junction_cmd = v->nav_has ? v->nav_cmd : -1;
	v->nav_has = false;
	if (v->junction_cmd < 0 && !VC_JUNCTION_PAUSE_ALWAYS)
		return;					/* no barcode waiting: just carry on */
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

	/* ---- 1. things that take priority in any state ---- */
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

	/* ---- 2. per state ---- */
	switch (v->state) {
	case VC_INIT:
		if (in->all_ready)
			enter(v, o, VC_IDLE, now);
		else if (now - v->t_state > VC_SELFTEST_MS)
			enter_fault(v, o, FAULT_SELFTEST, now);
		break;

	case VC_IDLE:
#if VC_BENCH_LINE_ONLY
		if (v->bench_restart && now - v->t_state >= 300) {	/* after an automatic reset: drive on */
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
		if (in->obstacle_close) {			/* bench: the ONLY reason to stop is an obstacle */
			cmd(o, MOTION_CMD_STOP, 0, 0, 0);
			o->scan_request = true;			/* sweep the servo and profile it while we wait */
			v->sub = 0;
			enter(v, o, VC_OBSTACLE_AVOID, now);
			break;
		}
#if VC_BENCH_BARCODES
		if (in->nav && in->nav->cmd != NAV_STOP) {	/* remember the barcode until the next junction */
			v->nav_has = true;
			v->nav_cmd = in->nav->cmd;
			v->t_nav = now;
		}
		if (v->nav_has && VC_NAV_EXPIRE_MS > 0 && now - v->t_nav > VC_NAV_EXPIRE_MS)
			v->nav_has = false;
		if (in->line && in->line->type == LINE_EVT_JUNCTION)
			junction_action(v, o, now);
#endif
		break;						/* otherwise keep creeping along the line */
#endif
		if (in->nav) {					/* a barcode was read: remember it until the next junction */
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
			v->nav_has = false;			/* no junction followed: forget it */

		if (in->obstacle_close) {
			cmd(o, MOTION_CMD_STOP, 0, 0, 0);
			o->scan_request = true;
			v->sub = 0;
			enter(v, o, VC_OBSTACLE_AVOID, now);
		} else if (in->line && in->line->type == LINE_EVT_JUNCTION && (v->nav_has || VC_JUNCTION_PAUSE_ALWAYS)) {
			junction_action(v, o, now);		/* stop, pause, then the barcode command */
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
			if (now - v->t_sub < VC_JUNCTION_PAUSE_MS) break;
			if (v->junction_cmd < 0 || v->junction_cmd == NAV_STRAIGHT) {
				go(v, o, VC_CRUISE_SPEED);	/* straight on */
				enter(v, o, VC_LINE_FOLLOW, now);
			} else {				/* drive to the junction centre first */
				manoeuvre(v, o, MOTION_CMD_MOVE, 0, VC_JUNCTION_ADVANCE_MM);
				v->settle = VC_SETTLE_TICKS;
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
				v->settle = VC_SETTLE_TICKS;
			} else {				/* turned: follow the line again */
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
#if VC_BENCH_LINE_ONLY
		if (v->sub == 0) {				/* wait for the sweep result (it is printed by the scan task) */
			if (in->obst) { v->ob = *in->obst; v->sub = 3; v->t_sub = now; }
			else if (now - v->t_state > VC_SCAN_TIMEOUT_MS) { v->sub = 3; v->t_sub = now; }
		} else {					/* HOLD: stay stopped until nothing has been close for a while */
			if (in->obstacle_close) {
				v->t_sub = now;
			} else if (now - v->t_sub >= VC_BENCH_CLEAR_MS) {
				go(v, o, VC_CRUISE_SPEED);
				enter(v, o, VC_LINE_FOLLOW, now);
			}
		}
		break;
#endif
		if (v->sub == 0) {				/* WAIT FOR THE SCAN */
			if (in->obst) {
				v->ob = *in->obst;
				if (!v->ob.found) {		/* false alarm */
					go(v, o, VC_CRUISE_SPEED);
					enter(v, o, VC_LINE_FOLLOW, now);
				} else if (v->ob.side == 0) {	/* neither side wide enough */
					enter_fault(v, o, FAULT_OBSTACLE, now);
				} else {
					v->sub = 1; v->step = 0;
					v->sp_line_found = false;
					bypass_step(v, o, 0);
				}
			} else if (now - v->t_state > VC_SCAN_TIMEOUT_MS) {
				enter_fault(v, o, FAULT_OBSTACLE, now);
			}
		} else if (v->sub == 1) {			/* BYPASS: one step at a time */
			if (in->line && in->line->type == LINE_EVT_FOUND && v->step >= 4)
				v->sp_line_found = true;
			if (v->settle > 0) { v->settle--; break; }
			if (in->motion_done) {
				if (++v->step < BYPASS_STEPS) {
					bypass_step(v, o, v->step);
				} else if (v->sp_line_found) {	/* REJOIN: already back on the line */
					go(v, o, VC_CRUISE_SPEED);
					enter(v, o, VC_LINE_FOLLOW, now);
				} else {			/* SEARCH_LINE */
					go(v, o, VC_RECOVER_SPEED);
					v->sub = 2;
					v->t_sub = now;
				}
			}
		} else {					/* SEARCH_LINE */
			if ((in->line && in->line->type == LINE_EVT_FOUND)) {
				go(v, o, VC_CRUISE_SPEED);
				enter(v, o, VC_LINE_FOLLOW, now);
			} else if (now - v->t_sub > VC_SEARCH_TIMEOUT_MS) {
				enter_fault(v, o, FAULT_LINE_LOST, now);
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
		break;						/* otherwise wait for a remote RESET */
	default:
		break;
	}

status:
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
