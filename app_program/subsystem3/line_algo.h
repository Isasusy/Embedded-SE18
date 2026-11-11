/*
 *	line_algo.h / line_algo.c -- Line Follow logic (statechart 6.1: CALIBRATE, TRACKING, JUNCTION, LINE_LOST).
 *	No kernel or hardware calls: one call per 5 ms sample, testable on a PC.
 */
#ifndef LINE_ALGO_H
#define LINE_ALGO_H

#include <stdint.h>
#include <stdbool.h>

typedef enum { LA_NO_EVENT, LA_JUNCTION, LA_LINE_LOST, LA_LINE_FOUND } la_event_t;

typedef struct {
	/* private */
	int	state;				/* 0 = calibrating, 1 = tracking */
	unsigned cal_ms;
	int	min_l, max_l, min_r, max_r;
	int	white_l, black_l, white_r, black_r;
	unsigned both_black_ms, both_white_ms, found_hold_ms;
	bool	lost, in_junction;
	bool	cal_ok;
	float	hold_corr;			/* last steer, held (fading) after the sensor leaves the line */
	unsigned hold_ms;
} la_t;

typedef struct {
	float	offset;		/* -1 (line far left) .. +1 (line far right) */
	float	steering_corr;	/* mm/s wheel speed difference, + = steer left (same convention as Motion Control) */
	bool	on_line;
	la_event_t event;
	bool	ready;		/* calibration finished */
} la_out_t;

void la_init(la_t *a);
void la_step(la_t *a, int raw_l, int raw_r, la_out_t *out);	/* LINE_PERIOD_MS per call */

#endif
