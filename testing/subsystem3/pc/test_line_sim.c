/* Closed-loop line-following simulation (23 Nov: the car weaved and over-corrected after every drift).
 * The car: two wheels MC_TRACK_MM apart, each following its target speed with a lag; the line steering runs every 15 ms
 * (the "5 ms" task really runs every 10-20 ms with the 10 ms kernel tick) and Motion Control applies it as
 * left = v - corr/2, right = v + corr/2. Two IR sensors 60 mm ahead of the axle, 20 mm either side of the centre line,
 * straddle an 18 mm black line; the IR spot is blurred (sigma 3 mm).
 *   make sim       new steering (asserts)      make sim_old   the original code, for comparison (prints only)
 * Scenarios: A = drifting 6 deg off a straight line; B = a 350 mm radius curve (90 deg) then straight. */
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include "line_algo.h"
#include "line_config.h"

#define V_CRUISE	90.0f		/* mm/s, the bench cruise speed */
#define TRACK		115.0f
#define STEER_MAX	200.0f		/* MC_STEER_MAX */
#define TAU_S		0.06f		/* wheel speed lag */
#define DEAD_MM_S	45.0f		/* below this target a wheel does not turn (motor dead band) */
#define LOOP_MS		15
#define SENS_AHEAD	60.0f
#define SENS_SIDE	20.0f
#define LINE_HALF	9.0f
#define SPOT_SIGMA	3.0f
#define DEADZONE	(SENS_SIDE - LINE_HALF)	/* both sensors white = centred as far as the car can tell */

static int fails, checks __attribute__((unused));
#define CHECK(c, ...) do { checks++; if (!(c)) { fails++; printf("FAIL line %d: ", __LINE__); printf(__VA_ARGS__); printf("\n"); } \
			   else { printf("  ok:   "); printf(__VA_ARGS__); printf("\n"); } } while (0)

static float phi(float x) { return 0.5f * (1.0f + erff(x / 1.41421356f)); }
static int raw_at(float ys)		/* sensor lateral position relative to the line centre -> ADC */
{
	float cover = phi((LINE_HALF - ys) / SPOT_SIGMA) - phi((-LINE_HALF - ys) / SPOT_SIGMA);
	return (int)(180.0f + 1820.0f * cover);
}

typedef struct { float flips, max_steer, max_y_late, rms_y_late, lost_ms; } res_t;

/* y: lateral offset of the axle from the line, + = left; psi: heading relative to the line, + = left */
static res_t run(float psi0_deg, float curve_r, float curve_from_s, float curve_deg, float secs)
{
	la_t a; la_out_t o = { 0 };
	la_init(&a);
	for (int t = 0; !o.ready && t < 100000; t += LOOP_MS) {		/* calibration: rocked over the line, until ready */
		int w = 180, b = 2000;
#ifdef OLD
		la_step(&a, (t / LOOP_MS) & 1 ? b : w, (t / LOOP_MS) & 1 ? w : b, &o);
#else
		la_step_dt(&a, (t / LOOP_MS) & 1 ? b : w, (t / LOOP_MS) & 1 ? w : b, LOOP_MS, &o);
#endif
	}
	float y = 0.0f, psi = psi0_deg * 0.0174533f, vl = V_CRUISE, vr = V_CRUISE, corr = 0.0f, turned = 0.0f;
	float prev_sign = 0.0f, sum_y2 = 0.0f; int n_late = 0;
	res_t r = { 0 };
	int next_ctrl = 0, last_ctrl = 0;
	for (int ms = 0; ms < (int)(secs * 1000.0f); ms++) {
		if (ms >= next_ctrl) {
			/* like the robot: every 15 ms, but every 1.5 s the task is held up ~130 ms (WiFi busy) */
			next_ctrl = ms + ((ms / 1500) != ((ms + LOOP_MS) / 1500) ? 130 : LOOP_MS);
			float yc = y + SENS_AHEAD * sinf(psi);
			int rl = raw_at(yc + SENS_SIDE), rr = raw_at(yc - SENS_SIDE);
#ifdef OLD
			la_step(&a, rl, rr, &o);					/* believes 5 ms passed */
			(void)last_ctrl;
#else
			la_step_dt(&a, rl, rr, (unsigned)(ms - last_ctrl ? ms - last_ctrl : LOOP_MS), &o);
#endif
			last_ctrl = ms;
			corr = o.steering_corr > STEER_MAX ? STEER_MAX : (o.steering_corr < -STEER_MAX ? -STEER_MAX : o.steering_corr);
			float s = corr > 1.0f ? 1.0f : (corr < -1.0f ? -1.0f : 0.0f);
			if (s != 0.0f) { if (prev_sign != 0.0f && s != prev_sign) r.flips++; prev_sign = s; }
			if (fabsf(corr) > r.max_steer) r.max_steer = fabsf(corr);
			if (fabsf(yc) > SENS_SIDE + LINE_HALF) r.lost_ms += LOOP_MS;	/* line outside both sensors */
		}
		const float dt = 0.001f;
		/* motor dead band: a wheel asked for less than ~45 mm/s does not turn at all (the inner wheel stops dead) */
		float tl = V_CRUISE - 0.5f * corr, tr = V_CRUISE + 0.5f * corr;
		if (fabsf(tl) < DEAD_MM_S) tl = 0.0f;
		if (fabsf(tr) < DEAD_MM_S) tr = 0.0f;
		vl += (tl - vl) * dt / TAU_S;
		vr += (tr - vr) * dt / TAU_S;
		float v = 0.5f * (vl + vr), w = (vr - vl) / TRACK;
		float line_w = 0.0f;						/* the line turning left under the car */
		if (curve_r > 0.0f && ms >= (int)(curve_from_s * 1000.0f) && turned < curve_deg * 0.0174533f) {
			line_w = v / curve_r; turned += line_w * dt;
		}
		psi += (w - line_w) * dt;
		y += v * sinf(psi) * dt;
		if (ms >= (int)(secs * 1000.0f) - 3000) {
			sum_y2 += y * y; n_late++;
			if (fabsf(y) > r.max_y_late) r.max_y_late = fabsf(y);
		}
	}
	r.rms_y_late = sqrtf(sum_y2 / (float)(n_late ? n_late : 1));
	return r;
}

static void show(const char *what, res_t r)
{
	printf("%-34s steer flips %3.0f  max steer %4.0f mm/s  last 3 s: |y| max %5.1f rms %5.1f mm  line lost %4.0f ms\n",
	       what, r.flips, r.max_steer, r.max_y_late, r.rms_y_late, r.lost_ms);
}

int main(void)
{
#ifdef OLD
	printf("ORIGINAL steering (hold 0.7 x 200 'samples' of 5 ms = 0.6 s real at 15 ms; min 110, gain 240)\n");
#else
	printf("NEW steering (ramped, real-time hold)\n");
#endif
	res_t a = run(6.0f, 0, 0, 0, 10.0f);
	show("A. drift 6 deg off a straight", a);
	res_t a2 = run(12.0f, 0, 0, 0, 10.0f);
	show("A2. drift 12 deg off a straight", a2);
	res_t b = run(0.0f, 350.0f, 1.0f, 90.0f, 10.0f);
	show("B. 350 mm curve, 90 deg", b);
	res_t c = run(0.0f, 250.0f, 1.0f, 90.0f, 10.0f);
	show("C. 250 mm curve, 90 deg", c);
#ifndef OLD
	CHECK(a.flips <= 4 && a.max_y_late < DEADZONE, "drift: settles (%.0f steer flips, last 3 s within %.1f mm)", a.flips, a.max_y_late);
	CHECK(a2.lost_ms == 0 && a2.max_y_late < DEADZONE, "12 deg drift: line never lost, settles (%.1f mm)", a2.max_y_late);
	CHECK(b.lost_ms == 0, "350 mm curve followed (line lost %.0f ms)", b.lost_ms);
	CHECK(c.lost_ms == 0, "250 mm curve followed (line lost %.0f ms)", c.lost_ms);
	CHECK(b.max_y_late < DEADZONE && c.max_y_late < DEADZONE, "calm again after the curves (%.1f / %.1f mm)", b.max_y_late, c.max_y_late);
	printf("%d checks, %d failed\n%s\n", checks, fails, fails ? "SOME FAILED" : "ALL PASSED");
#endif
	return fails != 0;
}
