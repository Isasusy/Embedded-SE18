#include <stdio.h>
#include <math.h>
#include <string.h>
#include "../../../app_program/subsystem5/vc_logic.h"
#include "../../../app_program/subsystem5/scan_algo.h"
#include "../../../app_program/subsystem5/vc_config.h"
static int fails, checks;
#define CHECK(c, ...) do { checks++; if (!(c)) { fails++; printf("FAIL line %d: ", __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

/* ---------- scan ---------- */
static void sweep(int *ang, int *d, int *n, float obst_w, float obst_d, float obst_off)
{
	*n = 0;
	for (int a = -SCAN_ANGLE_MAX; a <= SCAN_ANGLE_MAX; a += SCAN_ANGLE_STEP) {
		float lat = obst_d * tanf(a * 0.0174533f);		/* where this ray crosses the obstacle's range */
		bool hit = lat >= obst_off - obst_w / 2 && lat <= obst_off + obst_w / 2;
		ang[*n] = a; d[*n] = hit ? (int)(obst_d / cosf(a * 0.0174533f)) : -1; (*n)++;
	}
}

/* ---------- VC harness ---------- */
typedef struct { vc_t v; vc_in_t in; vc_out_t o; uint32_t now; } h_t;
static void tick(h_t *h, const remote_cmd_t *r, const nav_cmd_t *n, const line_evt_t *l, const imu_evt_t *i,
		 const obstacle_t *ob, bool done, bool close)
{
	h->now += VC_TICK_MS;
	h->in = (vc_in_t){ .now_ms = h->now, .remote = r, .nav = n, .line = l, .imu = i, .obst = ob,
			   .motion_done = done, .obstacle_close = close, .all_ready = true, .all_alive = true };
	vc_step(&h->v, &h->in, &h->o);
}
static void idle(h_t *h, int ticks) { for (int k = 0; k < ticks; k++) tick(h, 0, 0, 0, 0, 0, false, false); }
static void init(h_t *h) { memset(h, 0, sizeof *h); h->now = 1000; vc_init(&h->v); idle(h, 2); }
static const remote_cmd_t START = { REMOTE_START }, STOPC = { REMOTE_STOP }, RESET = { REMOTE_RESET };

int main(void)
{
	int ang[32], d[32], n;
	obstacle_t ob;

	/* 1. scan: centred 120 mm wide box at 300 mm: width within 20% (NFR4), both sides clear */
	sweep(ang, d, &n, 120, 300, 0);
	scan_analyze(ang, d, n, &ob);
	CHECK(ob.found && fabsf(ob.distance_mm - 300) < 15, "found at %.0f mm", ob.distance_mm);
	CHECK(fabsf(ob.width_mm - 120) < 24, "width %.0f mm (true 120)", ob.width_mm);
	CHECK(ob.left_clear_mm > 100 && ob.right_clear_mm > 100, "both sides clear: %.0f / %.0f", ob.left_clear_mm, ob.right_clear_mm);
	/* 2. obstacle shifted right: more room on the left -> choose left (+1) */
	sweep(ang, d, &n, 120, 300, -60);
	scan_analyze(ang, d, n, &ob);
	CHECK(ob.found, "shifted obstacle still ahead");
	/* 3. nothing ahead */
	sweep(ang, d, &n, 0, 300, 5000);
	scan_analyze(ang, d, n, &ob);
	CHECK(!ob.found, "empty sweep -> not found");
	/* 4. wall across the whole sweep: no clear side */
	for (int i = 0; i < n; i++) d[i] = 250;
	scan_analyze(ang, d, n, &ob);
	CHECK(ob.found && ob.side == 0, "wall everywhere -> side 0 (side=%d)", ob.side);
	/* 5. left blocked by a second object, right free -> right (-1) */
	sweep(ang, d, &n, 100, 300, 0);
	for (int i = 0; i < n; i++) if (ang[i] >= 20) d[i] = 250;
	scan_analyze(ang, d, n, &ob);
	CHECK(ob.found && ob.side == -1, "left blocked -> choose right (side=%d, L=%.0f R=%.0f)", ob.side, ob.left_clear_mm, ob.right_clear_mm);

	/* ---------- VC ---------- */
	h_t h;
	init(&h);
	CHECK(h.v.state == VC_IDLE, "INIT -> IDLE when all ready");
	{ h_t g; memset(&g, 0, sizeof g); g.now = 1000; vc_init(&g.v);
	  for (int k = 0; k < 400; k++) { g.now += VC_TICK_MS; g.in = (vc_in_t){ .now_ms = g.now, .all_alive = true }; vc_step(&g.v, &g.in, &g.o); }
	  CHECK(g.v.state == VC_FAULT && g.v.fault == FAULT_SELFTEST, "self-test timeout -> FAULT"); }

	/* IDLE never moves without START */
	idle(&h, 20);
	CHECK(h.v.state == VC_IDLE && !h.o.send_motion, "IDLE stays idle");
	tick(&h, &START, 0, 0, 0, 0, false, false);
	CHECK(h.v.state == VC_LINE_FOLLOW && h.o.send_motion && h.o.motion.type == MOTION_CMD_GO && h.o.motion.speed_mm_s == VC_CRUISE_SPEED, "START -> LINE_FOLLOW + GO");

	/* barcode LEFT: TURN +90, wait, back */
	nav_cmd_t left = { NAV_LEFT, 0, 0 };
	tick(&h, 0, &left, 0, 0, 0, false, false);
	CHECK(h.v.state == VC_BARCODE_ACTION && h.o.motion.type == MOTION_CMD_TURN && h.o.motion.angle_deg == 90, "LEFT -> TURN +90");
	tick(&h, 0, 0, 0, 0, 0, true, false);	/* stale latched done right after sending must be ignored */
	tick(&h, 0, 0, 0, 0, 0, true, false);
	CHECK(h.v.state == VC_BARCODE_ACTION, "stale cmd_done ignored during settle");
	tick(&h, 0, 0, 0, 0, 0, true, false);
	CHECK(h.v.state == VC_LINE_FOLLOW && h.o.motion.type == MOTION_CMD_GO, "turn done -> LINE_FOLLOW + GO");
	nav_cmd_t right = { NAV_RIGHT, 0, 0 };
	tick(&h, 0, &right, 0, 0, 0, false, false);
	CHECK(h.o.motion.type == MOTION_CMD_TURN && h.o.motion.angle_deg == -90, "RIGHT -> TURN -90");
	idle(&h, 3);
	tick(&h, 0, 0, 0, 0, 0, true, false);
	nav_cmd_t ut = { NAV_UTURN, 0, 0 };
	tick(&h, 0, &ut, 0, 0, 0, false, false);
	CHECK(h.o.motion.type == MOTION_CMD_UTURN, "UTURN -> UTURN");
	idle(&h, 3); tick(&h, 0, 0, 0, 0, 0, true, false);
	nav_cmd_t st = { NAV_STRAIGHT, 0, 0 };
	tick(&h, 0, &st, 0, 0, 0, false, false);
	CHECK(h.v.state == VC_LINE_FOLLOW && !h.o.send_motion, "STRAIGHT -> nothing");

	/* line lost -> RECOVER -> found */
	line_evt_t lost = { LINE_EVT_LOST, 0 }, found = { LINE_EVT_FOUND, 0 };
	tick(&h, 0, 0, &lost, 0, 0, false, false);
	CHECK(h.v.state == VC_RECOVER && h.o.motion.speed_mm_s == VC_RECOVER_SPEED, "LINE_LOST -> RECOVER slow");
	tick(&h, 0, 0, &found, 0, 0, false, false);
	CHECK(h.v.state == VC_LINE_FOLLOW, "LINE_FOUND -> LINE_FOLLOW");
	tick(&h, 0, 0, &lost, 0, 0, false, false);
	for (int k = 0; k < VC_RECOVER_TIMEOUT_MS / VC_TICK_MS + 2; k++) tick(&h, 0, 0, 0, 0, 0, false, false);
	CHECK(h.v.state == VC_FAULT && h.v.fault == FAULT_LINE_LOST, "RECOVER timeout -> FAULT");
	tick(&h, &RESET, 0, 0, 0, 0, false, false);
	CHECK(h.o.stop_clear_reset && h.v.state == VC_INIT && h.v.fault == FAULT_NONE, "RESET -> INIT, stop cleared");
	idle(&h, 2);
	CHECK(h.v.state == VC_IDLE, "self-test passes again -> IDLE");

	/* hump: CLIMBING slows, HUMP event restores and keeps the max */
	tick(&h, &START, 0, 0, 0, 0, false, false);
	imu_evt_t climb = { .type = IMU_EVT_MOTION_CLASS, .motion_class = MOTION_CLASS_CLIMBING };
	tick(&h, 0, 0, 0, &climb, 0, false, false);
	CHECK(h.v.state == VC_HUMP_CROSSING && h.o.motion.speed_mm_s == VC_HUMP_SPEED, "CLIMBING -> HUMP_CROSSING slow");
	imu_evt_t hump = { .type = IMU_EVT_HUMP, .peak_mm = 22, .max_hump_mm = 22 };
	tick(&h, 0, 0, 0, &hump, 0, false, false);
	CHECK(h.v.state == VC_LINE_FOLLOW && h.v.max_hump_mm == 22, "HUMP -> LINE_FOLLOW, max 22");
	tick(&h, 0, 0, 0, &climb, 0, false, false);
	imu_evt_t hump2 = { .type = IMU_EVT_HUMP, .peak_mm = 10, .max_hump_mm = 22 };
	tick(&h, 0, 0, 0, &hump2, 0, false, false);
	CHECK(h.v.max_hump_mm == 22, "smaller hump keeps the max");
	for (int k = 0; k < 30; k++) tick(&h, 0, 0, 0, 0, 0, false, false);
	CHECK(h.o.status.max_hump_mm == 22 || h.v.max_hump_mm == 22, "max kept in status");

	/* impact -> FAULT with STOP bit and stop command, status carries the max */
	imu_evt_t hit = { .type = IMU_EVT_IMPACT };
	tick(&h, 0, 0, 0, &hit, 0, false, false);
	CHECK(h.v.state == VC_FAULT && h.o.stop_set && h.o.motion.type == MOTION_CMD_STOP && h.v.fault == FAULT_IMPACT, "IMPACT -> FAULT + STOP bit");
	CHECK(h.o.send_status && h.o.status.state == VC_FAULT, "fault published at once");
	tick(&h, &START, 0, 0, 0, 0, false, false);
	CHECK(h.v.state == VC_FAULT, "START ignored in FAULT");
	tick(&h, &RESET, 0, 0, 0, 0, false, false);
	CHECK(h.v.max_hump_mm == 0, "reset clears running max (FR9)");
	idle(&h, 2);

	/* remote stop in any state; stop bit */
	tick(&h, &START, 0, 0, 0, 0, false, false);
	tick(&h, &STOPC, 0, 0, 0, 0, false, false);
	CHECK(h.v.state == VC_FAULT && h.v.fault == FAULT_REMOTE_STOP && h.o.stop_set, "remote STOP -> FAULT");
	tick(&h, &RESET, 0, 0, 0, 0, false, false); idle(&h, 2);

	/* motion fault, IMU sensor fault */
	tick(&h, &START, 0, 0, 0, 0, false, false);
	h.now += VC_TICK_MS; h.in = (vc_in_t){ .now_ms = h.now, .motion_fault = true, .all_ready = true, .all_alive = true }; vc_step(&h.v, &h.in, &h.o);
	CHECK(h.v.state == VC_FAULT && h.v.fault == FAULT_MOTION, "motion stall -> FAULT");
	tick(&h, &RESET, 0, 0, 0, 0, false, false); idle(&h, 2);
	tick(&h, &START, 0, 0, 0, 0, false, false);
	imu_evt_t sf = { .type = IMU_EVT_SENSOR_FAULT };
	tick(&h, 0, 0, 0, &sf, 0, false, false);
	CHECK(h.v.state == VC_FAULT && h.v.fault == FAULT_IMU_SENSOR, "IMU sensor fault -> FAULT");
	tick(&h, &RESET, 0, 0, 0, 0, false, false); idle(&h, 2);

	/* watchdog: a task stops reporting alive for > 100 ms */
	tick(&h, &START, 0, 0, 0, 0, false, false);
	int fed = 0;
	for (int k = 0; k < 10 && h.v.state != VC_FAULT; k++) {
		h.now += VC_TICK_MS; h.in = (vc_in_t){ .now_ms = h.now, .all_ready = true, .all_alive = false };
		vc_step(&h.v, &h.in, &h.o);
		if (h.o.feed_watchdog) fed++;
	}
	CHECK(h.v.state == VC_FAULT && h.v.fault == FAULT_WATCHDOG, "silent task -> FAULT_WATCHDOG");
	CHECK(fed >= 3 && fed <= 6 && !h.o.feed_watchdog, "watchdog fed while recent, not once stale (fed %d)", fed);
	tick(&h, &RESET, 0, 0, 0, 0, false, false); idle(&h, 2);

	/* obstacle: STOP + scan, then the bypass sequence, rejoin */
	tick(&h, &START, 0, 0, 0, 0, false, false);
	tick(&h, 0, 0, 0, 0, 0, false, true);
	CHECK(h.v.state == VC_OBSTACLE_AVOID && h.o.scan_request && h.o.motion.type == MOTION_CMD_STOP, "obstacle close -> STOP + scan request");
	obstacle_t res = { true, 200, 120, 200, 100, +1 };
	tick(&h, 0, 0, 0, 0, &res, false, false);
	CHECK(h.o.motion.type == MOTION_CMD_TURN && h.o.motion.angle_deg == 90, "bypass left: TURN +90 first");
	int moves = 0, turns = 0, steps = 1;
	line_evt_t *lp = 0; line_evt_t fnd = found;
	for (int k = 0; k < 200 && h.v.state == VC_OBSTACLE_AVOID; k++) {
		bool done = (k % 5) == 4;
		lp = (steps >= 6) ? &fnd : 0;
		tick(&h, 0, 0, lp, 0, 0, done, false);
		if (h.o.send_motion) {
			steps++;
			if (h.o.motion.type == MOTION_CMD_MOVE) moves++;
			if (h.o.motion.type == MOTION_CMD_TURN) turns++;
			if (h.o.motion.type == MOTION_CMD_MOVE && moves == 1)
				CHECK(h.o.motion.distance_mm == 60 + VC_ROBOT_HALF_W_MM + VC_SIDE_MARGIN_MM, "lateral move %d", h.o.motion.distance_mm);
			if (h.o.motion.type == MOTION_CMD_MOVE && moves == 2)
				CHECK(h.o.motion.distance_mm == 200 + VC_OBST_DEPTH_MM + VC_ROBOT_LEN_MM, "forward move %d", h.o.motion.distance_mm);
		}
	}
	CHECK(moves == 3 && turns == 3, "bypass = 3 moves + 3 more turns (moves %d turns %d)", moves, turns);
	CHECK(h.v.state == VC_LINE_FOLLOW, "bypass finished -> LINE_FOLLOW (state %s)", vc_state_name(h.v.state));
	/* obstacle with no usable side -> FAULT */
	tick(&h, 0, 0, 0, 0, 0, false, true);
	obstacle_t blocked = { true, 200, 120, 20, 20, 0 };
	tick(&h, 0, 0, 0, 0, &blocked, false, false);
	CHECK(h.v.state == VC_FAULT && h.v.fault == FAULT_OBSTACLE, "no clear side -> FAULT");
	tick(&h, &RESET, 0, 0, 0, 0, false, false); idle(&h, 2);
	/* false alarm */
	tick(&h, &START, 0, 0, 0, 0, false, false);
	tick(&h, 0, 0, 0, 0, 0, false, true);
	obstacle_t none = { 0 };
	tick(&h, 0, 0, 0, 0, &none, false, false);
	CHECK(h.v.state == VC_LINE_FOLLOW && h.o.motion.type == MOTION_CMD_GO, "scan finds nothing -> resume");
	/* scan timeout */
	tick(&h, 0, 0, 0, 0, 0, false, true);
	for (int k = 0; k < VC_SCAN_TIMEOUT_MS / VC_TICK_MS + 2; k++) tick(&h, 0, 0, 0, 0, 0, false, false);
	CHECK(h.v.state == VC_FAULT && h.v.fault == FAULT_OBSTACLE, "scan timeout -> FAULT");
	tick(&h, &RESET, 0, 0, 0, 0, false, false); idle(&h, 2);

	/* stop barcode -> COMPLETE, final report with max hump; only RESET leaves it */
	tick(&h, &START, 0, 0, 0, 0, false, false);
	tick(&h, 0, 0, 0, &hump, 0, false, false);
	nav_cmd_t stop = { NAV_STOP, 0, 0 };
	tick(&h, 0, &stop, 0, 0, 0, false, false);
	CHECK(h.v.state == VC_COMPLETE && h.o.motion.type == MOTION_CMD_STOP && h.o.status.max_hump_mm == 22, "stop barcode -> COMPLETE with max hump 22");
	tick(&h, &START, 0, 0, 0, 0, false, false);
	CHECK(h.v.state == VC_COMPLETE, "START ignored in COMPLETE");

	/* status cadence: about 2 Hz in steady state (NFR5) */
	tick(&h, &RESET, 0, 0, 0, 0, false, false); idle(&h, 2); tick(&h, &START, 0, 0, 0, 0, false, false);
	int sent = 0;
	for (int k = 0; k < 100; k++) { tick(&h, 0, 0, 0, 0, 0, false, false); sent += h.o.send_status; }	/* 2 s */
	CHECK(sent >= 3 && sent <= 5, "status about every 500 ms: %d in 2 s", sent);

	printf("%d checks, %d failed\n", checks, fails);
	printf(fails ? "SOME FAILED\n" : "ALL PASSED\n");
	return fails != 0;
}
