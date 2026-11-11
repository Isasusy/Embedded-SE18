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
	memset(o, 0, sizeof *o);

	if (a->state == 0) {				/* CALIBRATE */
		if (raw_l < a->min_l) a->min_l = raw_l;
		if (raw_l > a->max_l) a->max_l = raw_l;
		if (raw_r < a->min_r) a->min_r = raw_r;
		if (raw_r > a->max_r) a->max_r = raw_r;
		a->cal_ms += LINE_PERIOD_MS;
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
	if (el != er) {					/* exactly one sensor touches the line: steer back, at least LINE_STEER_MIN */
		if (o->steering_corr > -LINE_STEER_MIN && o->steering_corr < LINE_STEER_MIN)
			o->steering_corr = o->steering_corr < 0.0f ? -LINE_STEER_MIN : LINE_STEER_MIN;
		a->hold_corr = LINE_STEER_HOLD_GAIN * o->steering_corr;
		a->hold_ms = LINE_STEER_HOLD_MS;
	} else if (!el && !er && a->hold_ms > 0) {	/* the sensor just left the line: keep turning, fading out */
		o->steering_corr = a->hold_corr * (float)a->hold_ms / (float)LINE_STEER_HOLD_MS;
		a->hold_ms = a->hold_ms > LINE_PERIOD_MS ? a->hold_ms - LINE_PERIOD_MS : 0;
	} else {
		a->hold_ms = 0;
	}
	o->on_line = true;				/* both white is the normal, centred state */
#else
	o->on_line = bl || br;
#endif

	a->both_black_ms = (bl && br) ? a->both_black_ms + LINE_PERIOD_MS : 0;
	a->both_white_ms = (!bl && !br) ? a->both_white_ms + LINE_PERIOD_MS : 0;
	a->found_hold_ms = a->found_hold_ms > LINE_PERIOD_MS ? a->found_hold_ms - LINE_PERIOD_MS : 0;

#if LINE_STRADDLE
	if ((bl != br) && white_before >= LINE_FOUND_QUIET_MS && a->found_hold_ms == 0) {	/* a sensor reaches the line */
		o->event = LA_LINE_FOUND;
		a->found_hold_ms = LINE_FOUND_HOLDOFF_MS;
	}
#else
	(void)white_before;
	if (a->both_white_ms >= LINE_LOST_MS && !a->lost) {		/* LINE_LOST */
		a->lost = true;
		o->event = LA_LINE_LOST;
	} else if (a->lost && (bl || br)) {				/* line found again */
		a->lost = false;
		o->event = LA_LINE_FOUND;
	}
#endif
	if (a->both_black_ms >= LINE_JUNCTION_MS) {			/* JUNCTION: report once per crossing */
		if (!a->in_junction) { a->in_junction = true; if (!o->event) o->event = LA_JUNCTION; }
		o->steering_corr = 0.0f;				/* drive straight across it */
		a->hold_ms = 0;
	} else if (!(bl && br)) {
		a->in_junction = false;
	}
}
