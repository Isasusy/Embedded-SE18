/* PC test for motion_ctrl.c: a simple motor + wheel + encoder simulation closes the loop. */
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "../../../app_program/subsystem2/motion_ctrl.h"
#include "../../../app_program/subsystem2/motion_config.h"

static int fails, checks;
#define CHECK(c, ...) do { checks++; if (!(c)) { fails++; printf("FAIL line %d: ", __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

#define PI 3.14159265f
#define MMT (PI * MC_WHEEL_DIAM_MM / MC_TICKS_PER_REV)

/* ---- plant: duty -> wheel speed (first order, dead band), ticks with carried fraction ---- */
typedef struct { float v, frac, dist; } wheel_t;
typedef struct {
	wheel_t l, r;
	float gain_l, gain_r;	/* mm/s per duty above dead band */
	float dead, tau;
	float track_true;	/* the real track differs a little from MC_TRACK_MM */
	bool  frozen;		/* wheels cannot turn (stall test) */
	mc_t mc; mc_in_t in; mc_out_t out;
	float heading_true;	/* from true geometry */
	int   t_ms;
} sim_t;

static void sim_init(sim_t *s)
{
	*s = (sim_t){0};
	s->gain_l = s->gain_r = 0.45f;
	s->dead = 90.0f; s->tau = 0.08f;
	s->track_true = MC_TRACK_MM;
	mc_init(&s->mc);
}

static int wheel_ticks(wheel_t *w, int pwm, float gain, float dead, float tau, bool frozen)
{
	float eff = 0.0f;
	if (!frozen && abs(pwm) > dead) eff = (pwm > 0 ? 1.0f : -1.0f) * (abs(pwm) - dead) * gain;
	w->v += (eff - w->v) * (0.010f / tau);
	float d = w->v * 0.010f;
	w->dist += d;
	w->frac += d / MMT;
	int t = (int)w->frac;
	w->frac -= (float)t;
	return t;
}

static void tick(sim_t *s, const motion_cmd_t *cmd, bool stop, bool reset, bool line, float steer)
{
	/* the controller sees the edges the plant produced during the previous 10 ms, then answers */
	s->in.cmd = cmd; s->in.stop_bit = stop; s->in.reset = reset;
	s->in.line_valid = line; s->in.steer_mm_s = steer;
	mc_step(&s->mc, &s->in, &s->out);
	float dl0 = s->l.dist, dr0 = s->r.dist;
	s->in.dticks_l = wheel_ticks(&s->l, s->out.pwm_l, s->gain_l, s->dead, s->tau, s->frozen);
	s->in.dticks_r = wheel_ticks(&s->r, s->out.pwm_r, s->gain_r, s->dead, s->tau, s->frozen);
	s->heading_true += (s->r.dist - dr0 - (s->l.dist - dl0)) / s->track_true * 180.0f / PI;
	s->t_ms += 10;
}

static void run(sim_t *s, int ms, bool line, float steer)
{
	for (int i = 0; i < ms / 10; i++) tick(s, NULL, false, false, line, steer);
}

static void send(sim_t *s, motion_cmd_type_t t, int speed, int ang, int dist)
{
	motion_cmd_t c = { t, speed, ang, dist };
	tick(s, &c, false, false, false, 0);
}

static void run_until_done(sim_t *s, int max_ms)
{
	for (int i = 0; i < max_ms / 10 && !s->out.status.cmd_done && s->out.state != MC_ST_FAULT_STOP; i++)
		tick(s, NULL, false, false, false, 0);
}

int main(void)
{
	sim_t s;

	/* 1. boots STOPPED, no power */
	sim_init(&s);
	run(&s, 200, false, 0);
	CHECK(s.out.state == MC_ST_STOPPED && s.out.pwm_l == 0 && s.out.pwm_r == 0, "STOPPED outputs zero");

	/* 2. GO 150: reaches speed, then keeps it */
	send(&s, MOTION_CMD_GO, 150, 0, 0);
	CHECK(s.out.state == MC_ST_CRUISE, "GO -> CRUISE");
	run(&s, 1500, false, 0);
	CHECK(fabsf(s.out.status.speed_l - 150) < 20 && fabsf(s.out.status.speed_r - 150) < 20,
	      "speed near 150: %.1f / %.1f", s.out.status.speed_l, s.out.status.speed_r);
	float d0 = s.out.status.distance_mm;
	run(&s, 1000, false, 0);
	float dd = s.out.status.distance_mm - d0;
	CHECK(dd > 135 && dd < 165, "1 s at 150 mm/s covers about 150 mm, got %.1f", dd);

	/* 3. straightness: right motor 12% weaker, no line -> heading hold keeps it straight */
	sim_init(&s); s.gain_r = 0.45f * 0.88f;
	send(&s, MOTION_CMD_GO, 150, 0, 0);
	run(&s, 4000, false, 0);
	CHECK(fabsf(s.heading_true) < 4.0f, "weak right motor, true heading drift %.2f deg", s.heading_true);

	/* 4. STOP command */
	send(&s, MOTION_CMD_STOP, 0, 0, 0);
	CHECK(s.out.state == MC_ST_STOPPED && s.out.pwm_l == 0 && s.out.pwm_r == 0, "STOP cmd: zero PWM in same loop");

	/* 5. line steering: + steer turns left (heading increases) */
	sim_init(&s);
	send(&s, MOTION_CMD_GO, 150, 0, 0);
	run(&s, 1000, true, 0);
	float h0 = s.heading_true;
	run(&s, 1000, true, +40.0f);
	CHECK(s.heading_true - h0 > 5.0f, "positive steer turns left, change %.1f", s.heading_true - h0);
	h0 = s.heading_true;
	run(&s, 1000, true, -40.0f);
	CHECK(s.heading_true - h0 < -5.0f, "negative steer turns right, change %.1f", s.heading_true - h0);

	/* 6. TURN +90 / -90 from CRUISE: within +-10 deg (FR3), cmd_done, back to CRUISE */
	for (int k = 0; k < 2; k++) {
		int a = k ? -90 : 90;
		sim_init(&s); s.track_true = MC_TRACK_MM * 1.05f;		/* 5% model error */
		send(&s, MOTION_CMD_GO, 150, 0, 0);
		run(&s, 800, false, 0);
		float hs = s.heading_true;
		send(&s, MOTION_CMD_TURN, 0, a, 0);
		CHECK(s.out.state == MC_ST_TURNING, "TURN -> TURNING");
		run_until_done(&s, 5000);
		run(&s, 50, false, 0);
		float got = s.heading_true - hs;
		CHECK(s.out.status.cmd_done || s.out.state == MC_ST_CRUISE, "turn %d finished", a);
		CHECK(fabsf(got - (float)a) < 10.0f, "turn %d deg: true change %.1f", a, got);
	}

	/* 7. U-turn 180 within +-10 deg, returns to CRUISE */
	sim_init(&s); s.track_true = MC_TRACK_MM * 0.97f;
	send(&s, MOTION_CMD_GO, 150, 0, 0);
	run(&s, 800, false, 0);
	float hs = s.heading_true;
	send(&s, MOTION_CMD_UTURN, 0, 0, 0);
	CHECK(s.out.state == MC_ST_U_TURN, "UTURN -> U_TURN");
	run_until_done(&s, 6000);
	run(&s, 300, false, 0);
	CHECK(fabsf(fabsf(s.heading_true - hs) - 180.0f) < 10.0f, "U-turn true change %.1f", s.heading_true - hs);
	CHECK(s.out.state == MC_ST_CRUISE && s.out.status.cmd_done, "U-turn ends in CRUISE with cmd_done");

	/* 8. a turn from STOPPED does not drive off afterwards */
	sim_init(&s);
	send(&s, MOTION_CMD_TURN, 0, 90, 0);
	run_until_done(&s, 5000);
	run(&s, 500, false, 0);
	CHECK(s.out.state == MC_ST_STOPPED && s.out.pwm_l == 0, "turn from STOPPED returns to STOPPED");
	CHECK(fabsf(s.heading_true - 90.0f) < 10.0f, "stopped turn: %.1f", s.heading_true);

	/* 9. MOVE 300 mm forward and 200 mm in reverse */
	sim_init(&s);
	send(&s, MOTION_CMD_MOVE, 150, 0, 300);
	CHECK(s.out.state == MC_ST_DISTANCE_MOVE, "MOVE -> DISTANCE_MOVE");
	run_until_done(&s, 6000);
	run(&s, 300, false, 0);
	CHECK(fabsf(s.l.dist - 300) < 12 && fabsf(s.r.dist - 300) < 12, "MOVE 300: wheels %.1f / %.1f", s.l.dist, s.r.dist);
	CHECK(s.out.status.cmd_done, "MOVE sets cmd_done");
	float before = s.out.status.distance_mm;
	send(&s, MOTION_CMD_MOVE, 100, 0, -200);
	CHECK(s.out.status.cmd_done == false, "new command clears cmd_done");
	run_until_done(&s, 6000);
	run(&s, 300, false, 0);
	CHECK(fabsf((s.out.status.distance_mm - before) + 200) < 12, "reverse MOVE: %.1f", s.out.status.distance_mm - before);

	/* 10. STOP bit: motors off in the very same loop, stays in FAULT_STOP until bit clear AND reset */
	sim_init(&s);
	send(&s, MOTION_CMD_GO, 150, 0, 0);
	run(&s, 1000, false, 0);
	tick(&s, NULL, true, false, false, 0);
	CHECK(s.out.state == MC_ST_FAULT_STOP && s.out.pwm_l == 0 && s.out.pwm_r == 0, "STOP bit: 0 PWM within the loop");
	tick(&s, NULL, false, false, false, 0);
	CHECK(s.out.state == MC_ST_FAULT_STOP, "stays in FAULT_STOP after the bit clears");
	{ motion_cmd_t c = { MOTION_CMD_GO, 150, 0, 0 }; tick(&s, &c, false, false, false, 0); }
	CHECK(s.out.state == MC_ST_FAULT_STOP && s.out.pwm_l == 0, "commands ignored in FAULT_STOP");
	tick(&s, NULL, true, true, false, 0);
	CHECK(!s.out.reset_consumed && s.out.state == MC_ST_FAULT_STOP, "reset ignored while STOP bit set");
	tick(&s, NULL, false, true, false, 0);
	CHECK(s.out.reset_consumed && s.out.state == MC_ST_STOPPED, "reset -> STOPPED");
	send(&s, MOTION_CMD_GO, 150, 0, 0);
	CHECK(s.out.state == MC_ST_CRUISE, "can drive again after reset");

	/* 11. stall: wheels frozen while powered -> fault within ~400 ms, fault_new once */
	sim_init(&s); s.frozen = true;
	send(&s, MOTION_CMD_GO, 150, 0, 0);
	int news = 0, t_fault = -1;
	for (int i = 0; i < 100; i++) {
		tick(&s, NULL, false, false, false, 0);
		if (s.out.fault_new) { news++; t_fault = s.t_ms; }
	}
	CHECK(news == 1 && s.out.state == MC_ST_FAULT_STOP && s.out.fault == MC_FAULT_STALL, "stall -> FAULT_STOP once (news=%d)", news);
	CHECK(t_fault > 0 && t_fault <= 500, "stall detected in %d ms", t_fault);
	CHECK(s.out.pwm_l == 0 && s.out.pwm_r == 0, "motors off after stall");

	/* 12. manoeuvre timeout: a MOVE that cannot finish (wheels turn too slowly) */
	sim_init(&s); s.gain_l = s.gain_r = 0.0105f;	/* barely moves but not zero edges */
	s.dead = 0;
	send(&s, MOTION_CMD_MOVE, 150, 0, 5000);
	for (int i = 0; i < 1200 && s.out.state != MC_ST_FAULT_STOP; i++) tick(&s, NULL, false, false, false, 0);
	CHECK(s.out.state == MC_ST_FAULT_STOP && (s.out.fault == MC_FAULT_TIMEOUT || s.out.fault == MC_FAULT_STALL),
	      "never-finishing MOVE ends in fault (%d)", s.out.fault);

	/* 13. NFR6: STOP bit while turning */
	sim_init(&s);
	send(&s, MOTION_CMD_TURN, 0, 180, 0);
	run(&s, 300, false, 0);
	tick(&s, NULL, true, false, false, 0);
	CHECK(s.out.pwm_l == 0 && s.out.pwm_r == 0 && s.out.state == MC_ST_FAULT_STOP, "STOP bit while TURNING");

	/* 14. zero-size commands finish at once */
	sim_init(&s);
	send(&s, MOTION_CMD_TURN, 0, 0, 0);
	CHECK(s.out.status.cmd_done && s.out.state == MC_ST_STOPPED, "TURN 0 done immediately");

	printf("%d checks, %d failed\n", checks, fails);
	printf(fails ? "SOME FAILED\n" : "ALL PASSED\n");
	return fails != 0;
}
