#include <string.h>
#include "line_algo.h"
#include "line_config.h"

void la_init(la_t *a)
{
	memset(a, 0, sizeof *a);
	a->min_l = a->min_r = 4095;
	a->max_l = a->max_r = 0;
}

static float norm(int raw, int white, int black)
{
	float span = (float)(black - white);
	if (span == 0.0f) return 0.0f;
	float v = (float)(raw - white) / span;
	return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v);
}

static void finish_cal(la_t *a)
{
	int span_l = a->max_l - a->min_l, span_r = a->max_r - a->min_r;
	a->cal_ok = span_l >= LINE_CAL_MIN_RANGE && span_r >= LINE_CAL_MIN_RANGE;
	if (a->cal_ok) {
#if LINE_BLACK_IS_HIGH
		a->white_l = a->min_l; a->black_l = a->max_l; a->white_r = a->min_r; a->black_r = a->max_r;
#else
		a->white_l = a->max_l; a->black_l = a->min_l; a->white_r = a->max_r; a->black_r = a->min_r;
#endif
	} else {
#if LINE_BLACK_IS_HIGH
		a->white_l = a->white_r = LINE_DEF_WHITE; a->black_l = a->black_r = LINE_DEF_BLACK;
#else
		a->white_l = a->white_r = LINE_DEF_BLACK; a->black_l = a->black_r = LINE_DEF_WHITE;
#endif
	}
	a->state = 1;
}

void la_step(la_t *a, int raw_l, int raw_r, la_out_t *o)
{
	la_step_dt(a, raw_l, raw_r, LINE_PERIOD_MS, o);
}

void la_step_dt(la_t *a, int raw_l, int raw_r, unsigned dt, la_out_t *o)
{
	memset(o, 0, sizeof *o);
	if (dt == 0) dt = 1;
	if (dt > 200) dt = 200;				/* a long stall must not jump every timer at once */

	if (a->state == 0) {				/* CALIBRATE */
		if (raw_l < a->min_l) a->min_l = raw_l;
		if (raw_l > a->max_l) a->max_l = raw_l;
		if (raw_r < a->min_r) a->min_r = raw_r;
		if (raw_r > a->max_r) a->max_r = raw_r;
		a->cal_ms += dt;
		if (a->cal_ms >= LINE_CAL_MS)
			finish_cal(a);
		o->ready = (a->state == 1);
		return;
	}
	o->ready = true;

	float nl = norm(raw_l, a->white_l, a->black_l);
	float nr = norm(raw_r, a->white_r, a->black_r);
	bool bl = nl >= LINE_ON_THRESH, br = nr >= LINE_ON_THRESH;

	unsigned white_before = a->both_white_ms;		/* how long both were white up to the previous sample */

	o->offset = nr - nl;				/* line under the right sensor = positive */
#if LINE_STEER_TOWARD_BLACK
	o->steering_corr = -LINE_STEER_GAIN * o->offset;	/* black on the right -> steer right (negative) */
#else
	o->steering_corr = LINE_STEER_GAIN * o->offset;		/* black on the right -> steer left (lane-edge sensors) */
#endif
#if LINE_STRADDLE
	bool el = nl >= LINE_STEER_ENGAGE, er = nr >= LINE_STEER_ENGAGE;
	if (el != er) {					/* exactly one sensor touches the line: steer back, ramped */
		int dir = o->steering_corr < 0.0f ? -1 : +1;
		if (dir != a->eng_dir) { a->eng_dir = dir; a->eng_ms = 0.0f; }	/* the other side: start gently again */
		a->eng_ms += (float)dt;
		float limit = LINE_STEER_START + LINE_STEER_RAMP * a->eng_ms / 1000.0f;
		if (limit > LINE_STEER_MAX) limit = LINE_STEER_MAX;
		float mag = o->steering_corr < 0.0f ? -o->steering_corr : o->steering_corr;
		if (mag < LINE_STEER_START) mag = LINE_STEER_START;		/* touching at all = at least the gentle start */
		if (mag > limit) mag = limit;						/* ... but no more than the ramp allows yet */
		o->steering_corr = (float)dir * mag;
		a->hold_corr = LINE_STEER_HOLD_GAIN * o->steering_corr;
		a->hold_ms = LINE_STEER_HOLD_MS;
	} else {
		/* both white (or both black): the ramp's build-up fades, so a curve touching again keeps some of it */
		float fade = LINE_STEER_DECAY * (float)dt;
		a->eng_ms = a->eng_ms > fade ? a->eng_ms - fade : 0.0f;
		if (!el && !er && a->hold_ms > 0) {	/* the sensor just left the line: a little more, fading out */
			o->steering_corr = a->hold_corr * (float)a->hold_ms / (float)LINE_STEER_HOLD_MS;
			a->hold_ms = a->hold_ms > dt ? a->hold_ms - dt : 0;
		} else {
			a->hold_ms = 0;
			if (!el && !er) o->steering_corr = 0.0f;
		}
	}
	o->on_line = true;				/* both white is the normal, centred state */
#else
	o->on_line = bl || br;
#endif

	a->both_black_ms = (bl && br) ? a->both_black_ms + dt : 0;
	a->both_white_ms = (!bl && !br) ? a->both_white_ms + dt : 0;
	a->found_hold_ms = a->found_hold_ms > dt ? a->found_hold_ms - dt : 0;

#if LINE_STRADDLE
	if ((bl != br) && white_before >= LINE_FOUND_QUIET_MS && a->found_hold_ms == 0) {	/* a sensor reaches the line */
		o->event = LA_LINE_FOUND;
		a->found_hold_ms = LINE_FOUND_HOLDOFF_MS;
	}
#else
	(void)white_before;
	if (a->both_white_ms >= LINE_LOST_MS && !a->lost) {
		a->lost = true;
		o->event = LA_LINE_LOST;
	} else if (a->lost && (bl || br)) {
		a->lost = false;
		o->event = LA_LINE_FOUND;
	}
#endif
	if (a->both_black_ms >= LINE_JUNCTION_MS) {
		if (!a->in_junction) { a->in_junction = true; if (!o->event) o->event = LA_JUNCTION; }
		o->steering_corr = 0.0f;
		a->hold_ms = 0;
		a->eng_ms = 0.0f;
	} else if (!(bl && br)) {
		a->in_junction = false;
	}
}
