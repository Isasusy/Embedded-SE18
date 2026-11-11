#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "../../../app_program/subsystem3/line_algo.h"
#include "../../../app_program/subsystem3/barcode_algo.h"
#include "../../../app_program/subsystem3/line_config.h"
#include "../../../app_program/subsystem5/vc_msgs.h"
static int fails, checks;
#define CHECK(c, ...) do { checks++; if (!(c)) { fails++; printf("FAIL line %d: ", __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)
#define W 700
#define B 3300

static la_event_t run(la_t *a, int l, int r, int ms, la_out_t *last)
{
	la_event_t ev = LA_NO_EVENT;
	for (int i = 0; i < ms / LINE_PERIOD_MS; i++) {
		la_step(a, l, r, last);
		if (last->event != LA_NO_EVENT) ev = last->event;
	}
	return ev;
}
static void calibrate(la_t *a)
{
	la_out_t o; la_init(a);
	run(a, W, W, 1000, &o); run(a, B, B, 1000 + 10, &o);	/* sees both white and black */
}
static bd_result_t bar(bd_t *b, uint32_t *t, uint32_t w, uint32_t gap, int *nav, int *code)
{
	bd_result_t r = bd_edge(b, *t, true, nav, code);
	*t += w;
	bd_result_t r2 = bd_edge(b, *t, false, nav, code);
	*t += gap;
	return r2 != BD_NONE ? r2 : r;
}
static bd_result_t code_read(bd_t *b, uint32_t t0, uint32_t narrow, int bits, int *nav, int *code)
{
	uint32_t t = t0; bd_result_t r;
	r = bar(b, &t, narrow, narrow, nav, code);
	for (int i = 2; i >= 0; i--) r = bar(b, &t, ((bits >> i) & 1) ? narrow * 2 : narrow, narrow, nav, code);
	return r;
}

int main(void)
{
	la_t a; la_out_t o;

	/* calibration */
	calibrate(&a);
	CHECK(a.state == 1 && a.cal_ok, "calibration ok");
	CHECK(abs(a.white_l - W) <= 1 && abs(a.black_l - B) <= 1, "thresholds %d..%d", a.white_l, a.black_l);
	la_init(&a); run(&a, 1000, 1010, 2100, &o);
	CHECK(a.state == 1 && !a.cal_ok, "flat readings rejected, defaults used");

	/* offset and steering sign */
	calibrate(&a);
	run(&a, W, B, 100, &o);
	CHECK(o.offset > 0.9f && o.on_line, "line under right sensor: offset +%.2f", o.offset);
	CHECK(o.steering_corr < 0, "line on the right -> steer right (negative corr), got %.1f", o.steering_corr);
	run(&a, B, W, 100, &o);
	CHECK(o.offset < -0.9f && o.steering_corr > 0, "line on the left -> steer left (positive corr)");
	run(&a, (W + B) / 2 + 600, (W + B) / 2 + 600, 10, &o);
	CHECK(fabsf(o.offset) < 0.01f, "centred -> offset 0");

	/* junction once, steering zeroed, then re-arms */
	calibrate(&a);
	int junctions = 0;
	for (int i = 0; i < 40; i++) { la_step(&a, B, B, &o); if (o.event == LA_JUNCTION) junctions++; }
	CHECK(junctions == 1 && o.steering_corr == 0.0f, "one JUNCTION per crossing (%d)", junctions);
	run(&a, W, B, 50, &o);
	for (int i = 0; i < 40; i++) { la_step(&a, B, B, &o); if (o.event == LA_JUNCTION) junctions++; }
	CHECK(junctions == 2, "second crossing reported again (%d)", junctions);

	/* line lost / found */
	calibrate(&a);
	run(&a, B, W, 100, &o);
	la_event_t e = run(&a, W, W, LINE_LOST_MS - 20, &o);
	CHECK(e == LA_NO_EVENT, "not lost too early");
	e = run(&a, W, W, 60, &o);
	CHECK(e == LA_LINE_LOST, "LINE_LOST after %d ms", LINE_LOST_MS);
	e = run(&a, W, B, 20, &o);
	CHECK(e == LA_LINE_FOUND, "LINE_FOUND when a sensor sees black");
	e = run(&a, W, B, 200, &o);
	CHECK(e == LA_NO_EVENT, "found only once");

	/* barcode: every code, at two speeds (decode must not depend on speed) */
	static const struct { int bits, nav; } tab[] = { {0,NAV_LEFT},{1,NAV_RIGHT},{2,NAV_STRAIGHT},{3,NAV_UTURN},{7,NAV_STOP} };
	for (int sp = 0; sp < 2; sp++)
		for (unsigned k = 0; k < sizeof tab / sizeof tab[0]; k++) {
			bd_t b; bd_init(&b); int nav = -1, code = -1;
			bd_result_t r = code_read(&b, 1000, sp ? 9000 : 3000, tab[k].bits, &nav, &code);
			CHECK(r == BD_VALID && nav == tab[k].nav && code == tab[k].bits, "code %d speed %d -> nav %d (r=%d)", tab[k].bits, sp, nav, r);
		}
	{ bd_t b; bd_init(&b); int nav, code;
	  bd_result_t r = code_read(&b, 1000, 3000, 4, &nav, &code);	/* bits 4 is not in the table (value 4) */
	  CHECK(r == BD_INVALID, "unmapped code rejected (r=%d)", r); }
	{ bd_t b; bd_init(&b); int nav, code; uint32_t t = 1000;
	  bd_result_t r = bar(&b, &t, 100, 1000, &nav, &code);		/* 100 us bar = noise */
	  CHECK(r == BD_INVALID, "noise bar rejected"); }
	{ bd_t b; bd_init(&b); int nav, code; uint32_t t = 1000;
	  bar(&b, &t, 3000, 3000, &nav, &code); bar(&b, &t, 3000, 3000, &nav, &code);
	  CHECK(bd_poll(&b, t + BAR_TIMEOUT_US + 1000) == BD_INVALID, "half-read times out");
	  CHECK(bd_poll(&b, t + 2 * BAR_TIMEOUT_US) == BD_NONE, "timeout reported once");
	  bd_result_t r = code_read(&b, t + 3 * BAR_TIMEOUT_US, 3000, 1, &nav, &code);
	  CHECK(r == BD_VALID && nav == NAV_RIGHT, "decoder works again after a timeout"); }
	{ bd_t b; bd_init(&b); int nav, code; uint32_t t = 1000;		/* clock wrap-around */
	  t = 0xFFFFFFFFu - 5000u;
	  bd_result_t r = code_read(&b, t, 3000, 2, &nav, &code);
	  CHECK(r == BD_VALID && nav == NAV_STRAIGHT, "timestamps wrap around cleanly"); }

	printf("%d checks, %d failed\n", checks, fails);
	printf(fails ? "SOME FAILED\n" : "ALL PASSED\n");
	return fails != 0;
}
