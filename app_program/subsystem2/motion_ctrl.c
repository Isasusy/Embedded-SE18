/*
 *	motion_ctrl.c
 *	Subsystem 2 -- Motion Control logic.  See motion_ctrl.h.  No kernel or hardware calls in here.
 */
#include "s2_math.h"
#include <string.h>
#include <stdlib.h>
#include "motion_ctrl.h"
#include "motion_config.h"

#define PI_F		3.14159265f
#define RAD2DEG		(180.0f / PI_F)
#define MM_PER_TICK	(PI_F * MC_WHEEL_DIAM_MM / MC_TICKS_PER_REV)
#define DT		((float)MC_PERIOD_MS / 1000.0f)

static float clampf(float x, float lo, float hi)
{
	return x < lo ? lo : (x > hi ? hi : x);
}

const char *mc_state_name(mc_state_t s)
{
	static const char *const n[] = { "STOPPED", "CRUISE", "TURNING", "U_TURN", "DIST_MOVE", "FAULT_STOP" };
	return (unsigned)s < 6 ? n[s] : "?";
}

void mc_init(mc_t *m)
{
	memset(m, 0, sizeof *m);
	m->state = MC_ST_STOPPED;
	m->resume = MC_ST_STOPPED;
	m->cruise_speed = MC_DEFAULT_SPEED;
}

static void clear_control(mc_t *m)
{
	m->i_l = m->i_r = 0.0f;
	m->stall_l = m->stall_r = 0;
	m->pwm_l = m->pwm_r = 0;
}

static void enter_fault(mc_t *m, mc_fault_t f)
{
	m->state = MC_ST_FAULT_STOP;
	m->fault = f;
	m->resume = MC_ST_STOPPED;
	clear_control(m);
}

static void start_manoeuvre(mc_t *m, mc_state_t st)
{
	/* where to go when it ends: back to CRUISE only if we were cruising (so a TURN from STOPPED
	   does not suddenly drive off), otherwise keep whatever the interrupted manoeuvre would do */
	if (m->state == MC_ST_CRUISE)
		m->resume = MC_ST_CRUISE;
	else if (m->state == MC_ST_STOPPED)
		m->resume = MC_ST_STOPPED;
	m->state = st;
	m->man_cycles = 0;
	m->cmd_done = false;
}

static void finish_manoeuvre(mc_t *m)
{
	m->cmd_done = true;
	m->state = m->resume;
	m->hold = m->heading;
	m->i_l = m->i_r = 0.0f;
}

static void apply_cmd(mc_t *m, const motion_cmd_t *c)
{
	float sp = (float)c->speed_mm_s;

	switch (c->type) {
	case MOTION_CMD_STOP:
		m->state = MC_ST_STOPPED;
		m->resume = MC_ST_STOPPED;
		m->cmd_done = false;
		clear_control(m);
		break;

	case MOTION_CMD_GO:
		m->cruise_speed = (sp > 0.0f) ? clampf(sp, 0.0f, MC_MAX_SPEED) : MC_DEFAULT_SPEED;
		if (m->state != MC_ST_CRUISE)
			m->hold = m->heading;
		m->state = MC_ST_CRUISE;
		m->resume = MC_ST_CRUISE;
		m->cmd_done = false;
		break;

	case MOTION_CMD_TURN:
		if (c->angle_deg == 0) {
			m->cmd_done = true;
			break;
		}
		start_manoeuvre(m, MC_ST_TURNING);
		m->man_h0 = m->heading;
		m->man_angle = (float)c->angle_deg;
		break;

	case MOTION_CMD_UTURN:
		start_manoeuvre(m, MC_ST_U_TURN);
		m->man_h0 = m->heading;
		m->man_angle = 180.0f;
		break;

	case MOTION_CMD_MOVE:
		if (c->distance_mm == 0) {
			m->cmd_done = true;
			break;
		}
		start_manoeuvre(m, MC_ST_DISTANCE_MOVE);
		m->hold = m->heading;
		m->man_d0 = m->dist_mm;
		m->man_dist = (float)c->distance_mm;		/* signed */
		sp = (sp > 0.0f) ? clampf(sp, MC_MIN_SPEED, MC_MAX_SPEED) : m->cruise_speed;
		m->man_speed = sp;
		break;
	}
}

/* one wheel: feed-forward + PI.  Returns duty -1000..1000 */
static int wheel_pid(float tgt, float meas, float *integ)
{
	if (mm_fabsf(tgt) < 1.0f) {
		*integ = 0.0f;
		return 0;
	}
	float sgn = tgt > 0.0f ? 1.0f : -1.0f;
	float ff  = sgn * (MC_FF_PWM_MIN + MC_FF_PER_MM_S * mm_fabsf(tgt));
	float err = tgt - meas;

	*integ = clampf(*integ + MC_KI * err * DT, -MC_INTEG_MAX, MC_INTEG_MAX);
	return (int)clampf(ff + MC_KP * err + *integ, -1000.0f, 1000.0f);
}

void mc_step(mc_t *m, const mc_in_t *in, mc_out_t *out)
{
	bool fault_new = false;
	bool reset_used = false;

	/* ---- 1. measurements --------------------------------------------------------------- */
	float dl = (float)in->dticks_l * MM_PER_TICK;
	float dr = (float)in->dticks_r * MM_PER_TICK;

	m->dl_mm += dl;
	m->dr_mm += dr;
	m->dist_mm = 0.5f * (m->dl_mm + m->dr_mm);
	m->heading += (dr - dl) / MC_TRACK_MM * RAD2DEG;	/* right wheel further = turned left */
	m->v_l += MC_V_FILTER * (dl / DT - m->v_l);
	m->v_r += MC_V_FILTER * (dr / DT - m->v_r);

	/* ---- 2. stop bit / reset: checked first, every loop (NFR6) ---------------------------- */
	if (in->stop_bit) {
		if (m->state != MC_ST_FAULT_STOP || m->fault == MC_FAULT_NONE)
			enter_fault(m, MC_FAULT_STOP_BIT);
	} else if (in->reset) {
		m->state = MC_ST_STOPPED;
		m->resume = MC_ST_STOPPED;
		m->fault = MC_FAULT_NONE;
		m->cmd_done = false;
		clear_control(m);
		reset_used = true;
	}

	/* ---- 3. new command (ignored while in FAULT_STOP) ------------------------------------ */
	if (in->cmd && m->state != MC_ST_FAULT_STOP)
		apply_cmd(m, in->cmd);

	/* ---- 4. manoeuvre completion and timeout ---------------------------------------------- */
	if (m->state == MC_ST_TURNING || m->state == MC_ST_U_TURN || m->state == MC_ST_DISTANCE_MOVE) {
		if (++m->man_cycles * MC_PERIOD_MS > MC_CMD_TIMEOUT_MS) {
			enter_fault(m, MC_FAULT_TIMEOUT);
			fault_new = true;
		}
	}
	if (m->state == MC_ST_TURNING || m->state == MC_ST_U_TURN) {
		float sgn = m->man_angle > 0.0f ? 1.0f : -1.0f;
		float remaining = mm_fabsf(m->man_angle) - sgn * (m->heading - m->man_h0);
		if (remaining <= MC_TURN_STOP_AHEAD_DEG)
			finish_manoeuvre(m);
	} else if (m->state == MC_ST_DISTANCE_MOVE) {
		float sgn = m->man_dist > 0.0f ? 1.0f : -1.0f;
		float remaining = mm_fabsf(m->man_dist) - sgn * (m->dist_mm - m->man_d0);
		if (remaining <= MC_MOVE_STOP_AHEAD_MM)
			finish_manoeuvre(m);
	}

	/* ---- 5. wheel speed targets for the (possibly new) state ------------------------------- */
	float tl = 0.0f, tr = 0.0f;

	switch (m->state) {
	case MC_ST_CRUISE: {
		float corr;
		if (in->line_valid) {
			corr = in->steer_mm_s;			/* Line Follow steers */
			m->hold = m->heading;			/* ... and we remember where we point */
		} else {
			corr = MC_KH * (m->hold - m->heading);	/* line lost: keep straight */
		}
		corr = clampf(corr, -MC_STEER_MAX, MC_STEER_MAX);
		tl = m->cruise_speed - 0.5f * corr;
		tr = m->cruise_speed + 0.5f * corr;
		break;
	}
	case MC_ST_DISTANCE_MOVE: {
		float sgn = m->man_dist > 0.0f ? 1.0f : -1.0f;
		float remaining = mm_fabsf(m->man_dist) - sgn * (m->dist_mm - m->man_d0);
		float v = m->man_speed;
		if (remaining < MC_MOVE_SLOW_MM)
			v = clampf(m->man_speed * remaining / MC_MOVE_SLOW_MM, MC_MIN_SPEED, m->man_speed);
		float corr = clampf(MC_KH * (m->hold - m->heading), -MC_STEER_MAX, MC_STEER_MAX);
		/* in reverse the same wheel difference turns the other way */
		tl = sgn * v - 0.5f * corr * sgn;
		tr = sgn * v + 0.5f * corr * sgn;
		break;
	}
	case MC_ST_TURNING:
	case MC_ST_U_TURN: {
		float sgn = m->man_angle > 0.0f ? 1.0f : -1.0f;
		float remaining = mm_fabsf(m->man_angle) - sgn * (m->heading - m->man_h0);
		float s = MC_TURN_SPEED;
		if (remaining < MC_TURN_SLOW_DEG)
			s = clampf(MC_TURN_SPEED * remaining / MC_TURN_SLOW_DEG, MC_TURN_MIN, MC_TURN_SPEED);
		tl = -sgn * s;
		tr = +sgn * s;
		break;
	}
	case MC_ST_STOPPED:
	case MC_ST_FAULT_STOP:
	default:
		break;
	}

	/* ---- 6. PI per wheel ------------------------------------------------------------------ */
	int pl = 0, pr = 0;
	if (tl == 0.0f && tr == 0.0f) {
		m->i_l = m->i_r = 0.0f;
	} else {
		pl = wheel_pid(tl, m->v_l, &m->i_l);
		pr = wheel_pid(tr, m->v_r, &m->i_r);
	}

	/* ---- 7. stall detection: power but no encoder edges (uses last loop's duty) ------------- */
	bool moving = (m->state == MC_ST_CRUISE || m->state == MC_ST_TURNING ||
		       m->state == MC_ST_U_TURN || m->state == MC_ST_DISTANCE_MOVE);
	if (moving) {
		m->stall_l = (abs(m->pwm_l) >= MC_STALL_PWM && in->dticks_l == 0 && mm_fabsf(tl) >= MC_STALL_MIN_TGT) ? m->stall_l + 1 : 0;
		m->stall_r = (abs(m->pwm_r) >= MC_STALL_PWM && in->dticks_r == 0 && mm_fabsf(tr) >= MC_STALL_MIN_TGT) ? m->stall_r + 1 : 0;
		if (m->stall_l >= MC_STALL_CYCLES || m->stall_r >= MC_STALL_CYCLES) {
			enter_fault(m, MC_FAULT_STALL);
			fault_new = true;
			pl = pr = 0;
			tl = tr = 0.0f;
		}
	} else {
		m->stall_l = m->stall_r = 0;
	}
	if (m->state == MC_ST_FAULT_STOP || m->state == MC_ST_STOPPED)
		pl = pr = 0;			/* belt and braces: nothing drives in these states */

	m->pwm_l = pl;
	m->pwm_r = pr;

	/* ---- 8. outputs ----------------------------------------------------------------------- */
	out->pwm_l = pl;
	out->pwm_r = pr;
	out->state = m->state;
	out->fault = m->fault;
	out->fault_new = fault_new;
	out->reset_consumed = reset_used;
	out->tgt_l = tl;
	out->tgt_r = tr;
	out->status.speed_l = m->v_l;
	out->status.speed_r = m->v_r;
	out->status.distance_mm = m->dist_mm;
	out->status.heading_deg = m->heading;
	out->status.cmd_done = m->cmd_done;
}
