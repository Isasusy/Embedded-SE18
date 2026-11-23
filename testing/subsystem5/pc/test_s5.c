#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "../../../app_program/subsystem5/vc_logic.h"
#include "../../../app_program/subsystem5/scan_algo.h"
#include "../../../app_program/subsystem5/vc_config.h"
static int fails, checks;
#define CHECK(c, ...) do { checks++; if (!(c)) { fails++; printf("FAIL line %d: ", __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

/* ---------- scan ---------- */
/* a fine sweep for the algorithm's accuracy checks (the robot's profile sweep is tested further down) */
#define T_ANGLE_MAX	40
#define T_ANGLE_STEP	4
/* A box face obst_w wide at obst_d ahead, centred obst_off to the left. The sensor's beam is SCAN_BEAM_HALF_DEG either
   side of the ray, so a ray echoes if its cone overlaps the face, from the nearest point of the face inside the cone. */
#define D2R	0.0174533f
static void sweep_range(int *ang, int *d, int *n, float obst_w, float obst_d, float obst_off, int lo, int hi, int step)
{
	float aR = atanf((obst_off - obst_w / 2) / obst_d) / D2R, aL = atanf((obst_off + obst_w / 2) / obst_d) / D2R;
	*n = 0;
	for (int a = lo; a <= hi; a += step) {
		float c0 = (float)a - SCAN_BEAM_HALF_DEG, c1 = (float)a + SCAN_BEAM_HALF_DEG;	/* the beam cone */
		float o0 = c0 > aR ? c0 : aR, o1 = c1 < aL ? c1 : aL;				/* cone overlap face */
		bool hit = obst_w > 0 && o0 <= o1;
		float near = (o0 <= 0 && o1 >= 0) ? 0 : (fabsf(o0) < fabsf(o1) ? o0 : o1);	/* most head-on point */
		ang[*n] = a; d[*n] = hit ? (int)(obst_d / cosf(near * D2R)) : -1; (*n)++;
	}
}
/* the car's turning circle about its axle (farthest corner: front or rear overhang, and half the width) */
static float spin_r(void)
{
	float h = VC_ROBOT_HALF_W_MM, f = VC_SENSOR_AHEAD_MM, r = VC_ROBOT_LEN_MM - VC_SENSOR_AHEAD_MM;
	float o = f > r ? f : r;
	return sqrtf(o * o + h * h);
}
/* the planned obstacle: SCAN_PLAN_WIDTH_PCT of the BEAM-CORRECTED width, centred on it, as deep as it is wide */
static float plan_edge_t(const obstacle_t *o, int side)
{
	float c = 0.5f * (o->left_edge_mm + o->right_edge_mm), half = 0.5f * (o->left_edge_mm - o->right_edge_mm) * SCAN_PLAN_WIDTH_PCT / 100.0f;
	float e = side > 0 ? c + half : -(c - half);
	return e < 0 ? 0 : e;
}
/* the planned route is geometrically sound: the heading aims outside the planned obstacle as the sensor saw it
   (atan((ep + h) / d)), the car's side passes the planned corner, the angled leg ends with the centre line
   ep + h + SCAN_SIDE_MARGIN_MM to the side, and leg + drive-on carry the axle a turning circle past the planned back
   face (the whole car has passed it) before it turns toward the line */
static void route_ok(const obstacle_t *o, const char *what)
{
	float h = VC_ROBOT_HALF_W_MM, s = VC_SENSOR_AHEAD_MM, R = spin_r();
	float e = plan_edge_t(o, o->side), d = o->distance_mm;
	float Y = d + s, t = fabsf((float)o->turn_deg) * D2R;
	float ex = o->diag_mm * sinf(t), ey = o->diag_mm * cosf(t);			/* end of the angled leg */
	float corner = (Y * tanf(t) - e) * cosf(t);					/* corner to the leg, square to it */
	float aim = atanf((e + h) / d) / D2R;
	float wp = o->width_mm * SCAN_PLAN_WIDTH_PCT / 100.0f;
	float dp = wp < SCAN_DEPTH_MIN_MM ? SCAN_DEPTH_MIN_MM : (wp > SCAN_DEPTH_MAX_MM ? SCAN_DEPTH_MAX_MM : wp);
	CHECK(o->side != 0 && (o->turn_deg > 0) == (o->side > 0), "%s: turn %d on side %d", what, (int)o->turn_deg, (int)o->side);
	CHECK(abs((int)o->turn_deg) >= 1 && abs((int)o->turn_deg) <= VC_AVOID_MAX_DEG, "%s: turn %d in range", what, (int)o->turn_deg);
	CHECK(fabsf(o->depth_mm - dp) < 0.5f, "%s: planned depth %.0f = planned width %.0f (clamped)", what, o->depth_mm, wp);
	int td = abs((int)o->turn_deg);
	float t1 = (td - 1) * D2R;
	bool less_fails = td - 1 < aim || (Y * tanf(t1) - e) * cosf(t1) < h;		/* one degree less breaks a rule */
	CHECK(td >= aim - 0.01f && less_fails, "%s: turn %d is the shallowest aiming outside the planned obstacle (%.1f deg)",
	      what, td, aim);
	CHECK(corner >= h - 1, "%s: car side passes the planned corner (%.0f from the leg, half width %.0f)", what, corner, h);
	CHECK(fabsf(ex - o->lat_mm) <= 2, "%s: leg reaches lat %d (%.0f)", what, (int)o->lat_mm, ex);
	CHECK(fabsf((float)o->lat_mm - (e + h + SCAN_SIDE_MARGIN_MM)) <= 1, "%s: beside it, side %d mm clear of the planned edge",
	      what, (int)(o->lat_mm - e - h));
	CHECK((o->pass_mm >= 0 && fabsf(ey + o->pass_mm - (Y + o->depth_mm + R)) <= 2) ||
	      (o->pass_mm == 0 && ey >= Y + o->depth_mm + R - 2),
	      "%s: leg + drive-on end a turning circle past the back face (%.0f + %d vs %.0f)", what, ey, (int)o->pass_mm,
	      Y + o->depth_mm + R);
	CHECK(o->lat_mm < 600, "%s: sideways detour stays modest (%d mm)", what, (int)o->lat_mm);
}

/* the robot's profile sweep: servo 30..150 deg = relative -60..+60 */
static void profile(int *ang, int *d, int *n, float obst_w, float obst_d, float obst_off)
{
	sweep_range(ang, d, n, obst_w, obst_d, obst_off, SCAN_PROFILE_START_DEG - SCAN_SERVO_CENTRE_DEG,
		    SCAN_PROFILE_END_DEG - SCAN_SERVO_CENTRE_DEG, SCAN_PROFILE_STEP_DEG);
}
static void sweep(int *ang, int *d, int *n, float obst_w, float obst_d, float obst_off)
{
	sweep_range(ang, d, n, obst_w, obst_d, obst_off, -T_ANGLE_MAX, T_ANGLE_MAX, T_ANGLE_STEP);
}

/* ---------- VC harness ---------- */
typedef struct { vc_t v; vc_in_t in; vc_out_t o; uint32_t now; } h_t;
static void tick(h_t *h, const remote_cmd_t *r, const nav_cmd_t *n, const line_evt_t *l, const imu_evt_t *i,
		 const obstacle_t *ob, bool done, bool close)
{
	h->now += VC_TICK_MS;
	h->in = (vc_in_t){ .now_ms = h->now, .remote = r, .nav = n, .line = l, .imu = i, .obst = ob,
			   .motion_done = done, .obstacle_close = close, .all_ready = true, .all_alive = true, .net_ready = true };
	vc_step(&h->v, &h->in, &h->o);
}
/* one tick with the scan task's view: close flag, tracked distance, a profile result, motion done, a line event */
static void tick_ob(h_t *h, bool close, int mm, const obstacle_t *ob, bool done, const line_evt_t *l)
{
	h->now += VC_TICK_MS;
	h->in = (vc_in_t){ .now_ms = h->now, .line = l, .obst = ob, .motion_done = done, .obstacle_close = close,
			   .obstacle_mm = close ? mm : 0, .all_ready = true, .all_alive = true, .net_ready = true };
	vc_step(&h->v, &h->in, &h->o);
}
/* a tick in which the scan task only reports the obstacle as still there (within the hold distance, not close) */
static void tick_pr(h_t *h, int mm)
{
	h->now += VC_TICK_MS;
	h->in = (vc_in_t){ .now_ms = h->now, .obstacle_present = true, .obstacle_mm = mm, .all_ready = true, .all_alive = true, .net_ready = true };
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
	/* 5. left blocked by a second object, right free -> right (-1) (robot sweep: +-40 deg cannot see 200 mm of room) */
	profile(ang, d, &n, 100, 300, 0);
	for (int i = 0; i < n; i++) if (ang[i] >= 35) d[i] = 250;	/* a separate object, not touching the first */
	scan_analyze(ang, d, n, &ob);
	CHECK(ob.found && ob.side == -1, "left blocked -> choose right (side=%d, L=%.0f R=%.0f)", ob.side, ob.left_clear_mm, ob.right_clear_mm);

	/* 6. the robot's profile sweep: servo 30..150 deg in 5 deg steps = 25 rays, relative -60..+60 */
	sweep_range(ang, d, &n, 120, 200, 0, SCAN_PROFILE_START_DEG - SCAN_SERVO_CENTRE_DEG,
		    SCAN_PROFILE_END_DEG - SCAN_SERVO_CENTRE_DEG, SCAN_PROFILE_STEP_DEG);
	CHECK(n == 25 && ang[0] == -60 && ang[12] == 0 && ang[24] == 60 && n <= OBST_PROFILE_MAX_RAYS, "profile sweep is 25 rays -60..+60 (n=%d)", n);
	scan_analyze(ang, d, n, &ob);
	CHECK(ob.found && fabsf(ob.distance_mm - 200) < 10, "profile: found at %.0f mm", ob.distance_mm);
	CHECK(ob.side != 0 && ob.left_clear_mm > 2 * VC_ROBOT_HALF_W_MM, "profile: a side is free (L=%.0f R=%.0f)", ob.left_clear_mm, ob.right_clear_mm);
	CHECK(fabsf(ob.width_mm - 120) < 24, "profile: beam-corrected width %.0f mm (true 120)", ob.width_mm);
	CHECK(fabsf(ob.left_edge_mm - 60) < 15 && fabsf(ob.right_edge_mm + 60) < 15, "profile: edges %.0f / %.0f (true +60 / -60)",
	      ob.left_edge_mm, ob.right_edge_mm);
	/* 7. profile: obstacle covering centre and the right rays -> go left */
	for (int i = 0; i < n; i++) d[i] = ang[i] <= 0 ? 220 : -1;
	scan_analyze(ang, d, n, &ob);
	CHECK(ob.found && ob.side == +1 && ob.right_clear_mm == 0, "profile: right blocked -> left (side=%d)", ob.side);

	/* 8. clearance comes from enlarging the obstacle, never shrinking it */
	CHECK(SCAN_PLAN_WIDTH_PCT >= 100, "planned width ratio %d%% >= 100%%", SCAN_PLAN_WIDTH_PCT);
	/* 8b. the clearance scales with the obstacle: a wider one is passed further from its edge */
	{
		obstacle_t a, b;
		profile(ang, d, &n, 60, 300, 0); scan_analyze(ang, d, n, &a);
		profile(ang, d, &n, 200, 300, 0); scan_analyze(ang, d, n, &b);
		/* measured from the beam-corrected edge, which is what the plan enlarges */
		float ga = a.lat_mm - fabsf(a.side > 0 ? a.left_edge_mm : a.right_edge_mm);
		float gb = b.lat_mm - fabsf(b.side > 0 ? b.left_edge_mm : b.right_edge_mm);
		float sa = a.width_mm, sb = b.width_mm;		/* the plan enlarges the beam-corrected width */
		CHECK(gb - ga > 0.5f * (sb - sa) * (SCAN_PLAN_WIDTH_PCT - 100) / 100.0f - 2,
		      "clearance grows with width: %.0f mm (w %.0f) -> %.0f mm (w %.0f)", ga, a.width_mm, gb, b.width_mm);
	}
	/* 9. obstacle off to the left, both sides open: take the shorter detour (right) and move just past its right edge */
	profile(ang, d, &n, 120, 250, +60);
	scan_analyze(ang, d, n, &ob);
	CHECK(ob.found && ob.side == -1, "off-centre left, both open -> right (side=%d, edges %.0f..%.0f)", ob.side, ob.right_edge_mm, ob.left_edge_mm);
	CHECK(fabsf(ob.right_edge_mm) < 20 && fabsf(ob.left_edge_mm - 120) < 25, "off-centre edges %.0f / %.0f (true 0 / 120)", ob.right_edge_mm, ob.left_edge_mm);
	CHECK(ob.turn_deg < 0, "turns right to pass on the right (turn %d)", (int)ob.turn_deg);
	route_ok(&ob, "off-centre left");
	/* 10. the same obstacle mirrored: detour on the left */
	profile(ang, d, &n, 120, 250, -60);
	scan_analyze(ang, d, n, &ob);
	CHECK(ob.found && ob.side == +1, "off-centre right, both open -> left (side=%d)", ob.side);
	/* 11. a wall 300 mm to the left (beyond the obstacle): left is less open than the open right -> right */
	profile(ang, d, &n, 100, 200, 0);
	for (int i = 0; i < n; i++)
		if (ang[i] > 0 && d[i] < 0) d[i] = (int)(300 / sinf(ang[i] * D2R));
	scan_analyze(ang, d, n, &ob);
	CHECK(ob.found && ob.side == -1 && ob.left_clear_mm < ob.right_clear_mm && fabsf(ob.left_clear_mm - (300 - ob.scan_left_mm)) < 10,
	      "wall on the left -> right (side=%d, L=%.0f R=%.0f)", ob.side, ob.left_clear_mm, ob.right_clear_mm);
	/* 12. the wall 200 mm to the left: the left is too narrow for the car */
	profile(ang, d, &n, 100, 200, 0);
	for (int i = 0; i < n; i++)
		if (ang[i] > 0 && d[i] < 0) d[i] = (int)(200 / sinf(ang[i] * D2R));
	scan_analyze(ang, d, n, &ob);
	CHECK(ob.found && ob.side == -1 && ob.left_clear_mm < 2 * VC_ROBOT_HALF_W_MM, "narrow left -> right (side=%d, L=%.0f)", ob.side, ob.left_clear_mm);
	/* 13. a far wall that is past the obstacle's back face does not block the side */
	profile(ang, d, &n, 100, 200, 0);
	for (int i = 0; i < n; i++)
		if (ang[i] > 0 && d[i] < 0 && ang[i] <= 25) d[i] = (int)(800 / cosf(ang[i] * D2R));
	scan_analyze(ang, d, n, &ob);
	CHECK(ob.found && ob.left_clear_mm > 2 * VC_ROBOT_HALF_W_MM, "echo beyond the back face is not a block (L=%.0f)", ob.left_clear_mm);
	/* 14. planned depth = planned width (clamped); route checked against the planned obstacle */
	profile(ang, d, &n, 160, 300, 0);
	scan_analyze(ang, d, n, &ob);
	route_ok(&ob, "160 wide");
	profile(ang, d, &n, 30, 220, 0);
	scan_analyze(ang, d, n, &ob);
	CHECK(ob.found && fabsf(ob.depth_mm - fmaxf(SCAN_DEPTH_MIN_MM, ob.width_mm * SCAN_PLAN_WIDTH_PCT / 100.0f)) < 0.5f, "thin obstacle: depth %d (width %.0f, never below the minimum)", (int)ob.depth_mm, ob.width_mm);
	/* 15. no usable side: no route */
	for (int i = 0; i < n; i++) d[i] = 250;
	scan_analyze(ang, d, n, &ob);
	CHECK(ob.side == 0 && ob.turn_deg == 0 && ob.diag_mm == 0 && ob.lat_mm == 0 && ob.pass_mm == 0, "no side -> no route");
	/* 16. the turn grows with the obstacle's width, at the same distance */
	{
		int prev = 0;
		for (int w = 60; w <= 180; w += 60) {		/* wider ones need more room than the sweep can see */
			profile(ang, d, &n, (float)w, 250, 0);
			scan_analyze(ang, d, n, &ob);
			route_ok(&ob, "width sweep");
			int t = abs((int)ob.turn_deg);
			if (w > 60)
				CHECK(t > prev || t == VC_AVOID_MAX_DEG, "width %d: turn %d steeper than %d", w, t, prev);
			prev = t;
		}
	}
	/* 17. a 220 mm gap is wider than the car but too narrow for the route (clearance + room to turn on the spot) */
	profile(ang, d, &n, 100, 250, 0);
	scan_analyze(ang, d, n, &ob);
	{
		float le = ob.scan_left_mm;
		for (int i = 0; i < n; i++) {
			if (ang[i] < 0 && d[i] < 0) d[i] = 260;			/* right: more obstacle, out to the sweep's end */
			if (ang[i] > 0 && d[i] < 0) d[i] = (int)((le + 220) / sinf(ang[i] * D2R));
		}
	}
	scan_analyze(ang, d, n, &ob);
	CHECK(ob.found && ob.side == 0 && ob.left_clear_mm >= 2 * VC_ROBOT_HALF_W_MM, "220 mm gap: no route (side %d, L=%.0f)", ob.side, ob.left_clear_mm);

	/* 18. an obstacle to the LEFT of the path (60 mm wide, 100..160 mm left, 230 mm ahead): the straight-ahead rays
	   (beam +-SCAN_BEAM_HALF_DEG) miss it, so from 0 deg it is not found; from the angle the watch sweep hit it at
	   (30 deg) it is */
	profile(ang, d, &n, 60, 230, +130);
	{
		int c0 = 12;					/* index of 0 deg in the 25-ray profile */
		CHECK(ang[c0] == 0 && d[c0] < 0 && d[c0 - 1] < 0 && d[c0 + 1] < 0, "off-centre case: rays at 0 and +-5 deg miss it");
	}
	scan_analyze(ang, d, n, &ob);
	CHECK(!ob.found, "profiled from straight ahead: off-centre obstacle missed (found=%d)", ob.found);
	scan_analyze_from(ang, d, n, 30, &ob);
	CHECK(ob.found && ob.scan_right_mm > 0 && ob.scan_left_mm > 100, "profiled from the hit angle: found (scanned %.0f..%.0f)",
	      ob.scan_right_mm, ob.scan_left_mm);
	CHECK(ob.side == -1, "obstacle on the left -> pass it on the right (side %d)", ob.side);
	if (ob.side) route_ok(&ob, "off-centre, from the hit angle");
	/* 19. the hit-angle ray itself dropped out: a neighbour starts it */
	{
		int c30 = 18;					/* index of +30 deg */
		CHECK(ang[c30] == 30, "index of 30 deg");
		d[c30] = -1;
		scan_analyze_from(ang, d, n, 30, &ob);
		CHECK(ob.found, "hit-angle reading dropped out -> a neighbour starts the profile");
	}
	/* 20. nothing near the hit angle (the obstacle is gone): falls back to straight ahead, here nothing -> not found */
	profile(ang, d, &n, 0, 230, 0);
	scan_analyze_from(ang, d, n, 20, &ob);
	CHECK(!ob.found, "nothing at the hit angle or ahead -> not found");

	/* 21. a real profile from the robot (23 Nov): box ~104 mm ahead, echoes from servo 45 to 125 deg because of the wide
	   beam, open space beyond on both sides. The old plan (200 % of the uncorrected width, one long diagonal) needed
	   2398 mm of free space on each side, so the car never passed it. It must now have a route. */
	{
		static const int real_mm[25] = { 407, 296, 416, 140, 140, 122, 104, 122, 122, 107, 108, 108, 108,
						 125, 125, 125, 116, 125, 125, 155, 366, 366, 515, 510, 510 };
		for (int i = 0; i < 25; i++) { ang[i] = SCAN_PROFILE_START_DEG + 5 * i - SCAN_SERVO_CENTRE_DEG; d[i] = real_mm[i]; }
		scan_analyze_from(ang, d, 25, -30, &ob);
		CHECK(ob.found && fabsf(ob.distance_mm - 104) < 1, "robot sweep: found at %.0f mm", ob.distance_mm);
		CHECK(ob.width_mm > 60 && ob.width_mm < 160, "robot sweep: beam-corrected width %.0f mm (box ~100)", ob.width_mm);
		CHECK(ob.side != 0, "robot sweep: a route exists (side %d, L free %.0f need %.0f, R free %.0f need %.0f)", ob.side,
		      ob.left_clear_mm, scan_route_need(&ob, +1), ob.right_clear_mm, scan_route_need(&ob, -1));
		if (ob.side) {
			route_ok(&ob, "robot sweep");
			printf("  robot sweep plan: side %d, turn %d deg, leg %d mm, beside at %d mm, drive on %d mm\n",
			       ob.side, (int)ob.turn_deg, (int)ob.diag_mm, (int)ob.lat_mm, (int)ob.pass_mm);
		}
	}

	/* ---------- VC ---------- */
	h_t h;
	init(&h);
	CHECK(h.v.state == VC_IDLE, "INIT -> IDLE when all ready");
	{ h_t g; memset(&g, 0, sizeof g); g.now = 1000; vc_init(&g.v);
	  for (int k = 0; k < VC_SELFTEST_MS / VC_TICK_MS + 10; k++) { g.now += VC_TICK_MS; g.in = (vc_in_t){ .now_ms = g.now, .all_alive = true }; vc_step(&g.v, &g.in, &g.o); }
	  CHECK(g.v.state == VC_FAULT && g.v.fault == FAULT_SELFTEST, "self-test timeout -> FAULT"); }

	/* initialising: stays in INIT (and says what it waits for) until everything is READY; START is ignored meanwhile */
	{ h_t g; memset(&g, 0, sizeof g); g.now = 1000; vc_init(&g.v);
	  robot_status_t last = { 0 };				/* the last status actually sent */
	  for (int k = 0; k < 50; k++) {
		g.now += VC_TICK_MS;
		g.in = (vc_in_t){ .now_ms = g.now, .all_alive = true, .waiting = VC_WAIT_IMU | VC_WAIT_LINE,
				  .remote = (k == 10) ? &START : 0 };
		vc_step(&g.v, &g.in, &g.o);
		if (g.o.send_status) last = g.o.status;
	  }
	  CHECK(g.v.state == VC_INIT && last.state == VC_INIT &&
		last.waiting == (VC_WAIT_IMU | VC_WAIT_LINE | (VC_WAIT_FOR_NET ? VC_WAIT_NET : 0)),
		"INIT while IMU/line not ready, START ignored, status.waiting = 0x%x", (unsigned)last.waiting);
	  for (int k = 0; k < 5; k++) {
		g.now += VC_TICK_MS;
		g.in = (vc_in_t){ .now_ms = g.now, .all_alive = true, .all_ready = true, .net_ready = true };
		vc_step(&g.v, &g.in, &g.o);
		if (g.o.send_status) last = g.o.status;
	  }
	  CHECK(g.v.state == VC_IDLE && last.state == VC_IDLE && last.waiting == 0, "all READY (+ MQTT) -> IDLE, nothing waiting");
#if VC_WAIT_FOR_NET
	  h_t n; memset(&n, 0, sizeof n); n.now = 1000; vc_init(&n.v);
	  for (int k = 0; k < 100; k++) {
		n.now += VC_TICK_MS;
		n.in = (vc_in_t){ .now_ms = n.now, .all_alive = true, .all_ready = true, .net_ready = false };
		vc_step(&n.v, &n.in, &n.o);
		if (n.o.send_status) last = n.o.status;
	  }
	  CHECK(n.v.state == VC_INIT && last.waiting == VC_WAIT_NET, "subsystems ready, MQTT not: still INIT, waiting WIFI");
	  n.now += VC_TICK_MS; n.in = (vc_in_t){ .now_ms = n.now, .all_alive = true, .all_ready = true, .net_ready = true };
	  vc_step(&n.v, &n.in, &n.o);
	  CHECK(n.v.state == VC_IDLE, "MQTT up -> IDLE");
	  h_t q; memset(&q, 0, sizeof q); q.now = 1000; vc_init(&q.v);
	  for (int k = 0; k < VC_NET_WAIT_MS / VC_TICK_MS + 10; k++) {
		q.now += VC_TICK_MS;
		q.in = (vc_in_t){ .now_ms = q.now, .all_alive = true, .all_ready = true, .net_ready = false };
		vc_step(&q.v, &q.in, &q.o);
	  }
	  CHECK(q.v.state == VC_IDLE, "no WiFi for VC_NET_WAIT_MS: IDLE anyway (runs without the dashboard)");
#endif
	}

	/* turn calibration: SPIN in IDLE = one TURN 360, state stays IDLE; ignored while driving */
	{ static const remote_cmd_t SPIN = { REMOTE_SPIN };
	  tick(&h, &SPIN, 0, 0, 0, 0, false, false);
	  CHECK(h.v.state == VC_IDLE && h.o.send_motion && h.o.motion.type == MOTION_CMD_TURN && h.o.motion.angle_deg == 360,
		"SPIN in IDLE -> TURN 360, still IDLE");
	  tick(&h, &START, 0, 0, 0, 0, false, false);
	  tick(&h, &SPIN, 0, 0, 0, 0, false, false);
	  CHECK(h.v.state == VC_LINE_FOLLOW && !(h.o.send_motion && h.o.motion.type == MOTION_CMD_TURN), "SPIN ignored while driving");
	  tick(&h, &RESET, 0, 0, 0, 0, false, false); idle(&h, 2); }

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
	const int st_before = (int)h.v.state;
	tick(&h, 0, 0, 0, &hit, 0, false, false);
#if VC_STOP_ON_IMPACT
	CHECK(h.v.state == VC_FAULT && h.o.stop_set && h.o.motion.type == MOTION_CMD_STOP && h.v.fault == FAULT_IMPACT, "IMPACT -> FAULT + STOP bit");
#else
	CHECK((int)h.v.state == st_before && !h.o.stop_set, "IMPACT ignored (VC_STOP_ON_IMPACT 0): the car keeps driving");
	tick(&h, &STOPC, 0, 0, 0, 0, false, false);		/* into FAULT another way for the checks below */
#endif
	(void)st_before;
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
	h.now += VC_TICK_MS; h.in = (vc_in_t){ .now_ms = h.now, .motion_fault = true, .all_ready = true, .all_alive = true, .net_ready = true }; vc_step(&h.v, &h.in, &h.o);
	CHECK(h.v.state == VC_FAULT && h.v.fault == FAULT_MOTION, "motion stall -> FAULT");
	tick(&h, &RESET, 0, 0, 0, 0, false, false); idle(&h, 2);
	tick(&h, &START, 0, 0, 0, 0, false, false);
	imu_evt_t sf = { .type = IMU_EVT_SENSOR_FAULT };
	tick(&h, 0, 0, 0, &sf, 0, false, false);
	CHECK(h.v.state == VC_FAULT && h.v.fault == FAULT_IMU_SENSOR, "IMU sensor fault -> FAULT");
	tick(&h, &RESET, 0, 0, 0, 0, false, false); idle(&h, 2);

	/* tipped over / picked up while driving -> stop; refuse to start while still tilted or lifted */
	tick(&h, &START, 0, 0, 0, 0, false, false);
	imu_evt_t tl = { .type = IMU_EVT_TILT, .pitch_deg = 60.0f };
	tick(&h, 0, 0, 0, &tl, 0, false, false);
	CHECK(h.v.state == VC_FAULT && h.v.fault == FAULT_TILT && h.o.stop_set && h.o.motion.type == MOTION_CMD_STOP,
	      "TILT event -> FAULT_TILT + STOP bit");
	tick(&h, &RESET, 0, 0, 0, 0, false, false); idle(&h, 2);
	tick(&h, &START, 0, 0, 0, 0, false, false);
	h.now += VC_TICK_MS; h.in = (vc_in_t){ .now_ms = h.now, .tilted = true, .all_ready = true, .all_alive = true, .net_ready = true }; vc_step(&h.v, &h.in, &h.o);
	CHECK(h.v.state == VC_FAULT && h.v.fault == FAULT_TILT && h.o.stop_set, "tilted flag while driving -> FAULT_TILT");
	tick(&h, &RESET, 0, 0, 0, 0, false, false); idle(&h, 2);
	tick(&h, &START, 0, 0, 0, 0, false, false);
	h.now += VC_TICK_MS; h.in = (vc_in_t){ .now_ms = h.now, .lifted = true, .all_ready = true, .all_alive = true, .net_ready = true }; vc_step(&h.v, &h.in, &h.o);
	CHECK(h.v.state == VC_FAULT && h.v.fault == FAULT_LIFTED && h.o.stop_set, "lifted while driving -> FAULT_LIFTED");
	tick(&h, &RESET, 0, 0, 0, 0, false, false); idle(&h, 2);
	CHECK(h.v.state == VC_IDLE, "back to IDLE after reset");
	h.now += VC_TICK_MS; h.in = (vc_in_t){ .now_ms = h.now, .lifted = true, .all_ready = true, .all_alive = true, .net_ready = true }; vc_step(&h.v, &h.in, &h.o);
	CHECK(h.v.state == VC_IDLE && !h.o.send_motion, "lifted while IDLE: no fault, nothing moves");
	h.now += VC_TICK_MS; h.in = (vc_in_t){ .now_ms = h.now, .remote = &START, .tilted = true, .all_ready = true, .all_alive = true, .net_ready = true }; vc_step(&h.v, &h.in, &h.o);
	CHECK(h.v.state == VC_FAULT && h.v.fault == FAULT_TILT && !(h.o.send_motion && h.o.motion.type == MOTION_CMD_GO),
	      "START while tilted is refused -> FAULT_TILT, never GO");
	tick(&h, &RESET, 0, 0, 0, 0, false, false); idle(&h, 2);
	tick(&h, &START, 0, 0, 0, 0, false, false);
	CHECK(h.v.state == VC_LINE_FOLLOW, "START works again once level and on the floor");
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

	/* ---------- obstacle: STOP -> CONFIRM still 5 s -> PROFILE -> BYPASS -> rejoin ---------- */
	tick(&h, &RESET, 0, 0, 0, 0, false, false); idle(&h, 2);
	tick(&h, &START, 0, 0, 0, 0, false, false);
	CHECK(h.v.state == VC_LINE_FOLLOW, "obstacle tests start in LINE_FOLLOW (state %s)", vc_state_name(h.v.state));
	CHECK(!h.o.scan_hold, "driving -> scan task watches normally (no hold)");
	/* "still there" alone (beyond the stop distance) never stops a driving car: only a close hit does */
	for (int k = 0; k < 20; k++) tick_pr(&h, SCAN_HOLD_MM - 10);
	CHECK(h.v.state == VC_LINE_FOLLOW, "hold-distance echoes do not stop a driving car (state %s)", vc_state_name(h.v.state));
	/* a. close -> STOP at once, but no sweep yet */
	tick_ob(&h, true, 300, 0, false, 0);
	CHECK(h.v.state == VC_OBSTACLE_AVOID && h.o.motion.type == MOTION_CMD_STOP && !h.o.scan_request, "obstacle close -> STOP, no sweep yet");
	CHECK(h.o.scan_hold, "stopped for an obstacle -> scan task told to hold it");
	/* b. it moves away after 2 s: drive on, never swept */
	int scans = 0;
	for (int k = 0; k < 2000 / VC_TICK_MS; k++) { tick_ob(&h, true, 300, 0, false, 0); scans += h.o.scan_request; }
	for (int k = 0; k < VC_OBST_CLEAR_MS / VC_TICK_MS + 1 && h.v.state == VC_OBSTACLE_AVOID; k++) { tick_ob(&h, false, 0, 0, false, 0); scans += h.o.scan_request; }
	CHECK(h.v.state == VC_LINE_FOLLOW && h.o.motion.type == MOTION_CMD_GO && scans == 0, "transient obstacle -> drive on, no sweep (scans %d)", scans);
	/* c. a still obstacle: the sweep is requested after VC_OBST_STILL_MS, not before; a 500 ms gap is tolerated */
	tick_ob(&h, true, 300, 0, false, 0);
	uint32_t t0 = h.now, t_scan = 0;
	for (int k = 0; k < 8000 / VC_TICK_MS && !t_scan; k++) {
		bool gap = k >= 100 && k < 125;			/* 2.0 .. 2.5 s: no pings hit */
		tick_ob(&h, !gap, 300 + (k % 3) * 10, 0, false, 0);
		if (h.o.scan_request) t_scan = h.now;
	}
	CHECK(t_scan && t_scan - t0 >= VC_OBST_STILL_MS && t_scan - t0 < VC_OBST_STILL_MS + 2 * VC_TICK_MS,
	      "still obstacle -> sweep after %u ms", (unsigned)(t_scan - t0));
	CHECK(h.o.scan_hold, "profiling -> scan task still holds the obstacle");
	/* d. the scan timeout runs from the sweep request, not from the stop */
	for (int k = 0; k < (VC_SCAN_TIMEOUT_MS - 200) / VC_TICK_MS; k++) tick_ob(&h, false, 0, 0, false, 0);
	CHECK(h.v.state == VC_OBSTACLE_AVOID && h.v.sub == 1, "still waiting for the profile before the timeout");
	/* e. the profile arrives: turn, angled leg until beside it, turn back; line finding: pass, turn toward the line,
	   creep; a LINE_FOUND before that turn is ignored, one during the creep -> LINE_FOLLOW */
	obstacle_t res = { .found = true, .distance_mm = 200, .width_mm = 120, .left_clear_mm = 300, .right_clear_mm = 210, .side = +1,
			   .left_edge_mm = 50, .right_edge_mm = -70, .depth_mm = 120, .turn_deg = 31, .diag_mm = 420, .lat_mm = 216, .pass_mm = 0 };
	tick_ob(&h, false, 0, &res, false, 0);
	CHECK(!h.o.scan_hold, "avoiding -> hold released");
	motion_cmd_t seq[8];
	int ns = 0;
	seq[ns++] = h.o.motion;
	line_evt_t fnd = found;
	for (int k = 0; k < 300 && h.v.state == VC_OBSTACLE_AVOID; k++) {
		bool creeping = ns >= 6;
		bool done = !creeping && (k % 5) == 4;
		const line_evt_t *ev = ((ns == 2 && (k % 5) == 1) || (creeping && (k % 5) == 3)) ? &fnd : 0;	/* an early one on the leg */
		tick_ob(&h, false, 0, 0, done, ev);
		if (h.o.send_motion && ns < 8) seq[ns++] = h.o.motion;
	}
	CHECK(ns == 7, "avoid + line finding = 7 commands (got %d)", ns);
	CHECK(seq[0].type == MOTION_CMD_TURN && seq[0].angle_deg == 31, "1: turn +31 (%d)", seq[0].angle_deg);
	CHECK(seq[1].type == MOTION_CMD_MOVE && seq[1].distance_mm == 420, "2: angled leg 420 (%d)", seq[1].distance_mm);
	CHECK(seq[2].type == MOTION_CMD_TURN && seq[2].angle_deg == -31, "3: turn back -31 (%d)", seq[2].angle_deg);
	CHECK(seq[3].type == MOTION_CMD_MOVE && seq[3].distance_mm == 0, "4: no drive-on (%d)", seq[3].distance_mm);
	CHECK(seq[4].type == MOTION_CMD_TURN && seq[4].angle_deg == -31, "5: turn toward the line -31 (%d)", seq[4].angle_deg);
	CHECK(seq[5].type == MOTION_CMD_GO && seq[5].speed_mm_s == VC_RECOVER_SPEED, "6: creep to find the line");
	CHECK(seq[6].type == MOTION_CMD_GO && seq[6].speed_mm_s == VC_CRUISE_SPEED, "7: line found -> follow at cruise");
	CHECK(h.v.state == VC_LINE_FOLLOW, "avoidance finished -> LINE_FOLLOW (state %s)", vc_state_name(h.v.state));
	/* f. a moving obstacle (distance keeps changing) is never profiled */
	tick_ob(&h, true, 380, 0, false, 0);
	scans = 0;
	for (int k = 0; k < 7000 / VC_TICK_MS; k++) { tick_ob(&h, true, 380 - k, 0, false, 0); scans += h.o.scan_request; }
	CHECK(h.v.state == VC_OBSTACLE_AVOID && scans == 0, "approaching object -> keeps waiting, no sweep (scans %d)", scans);
	/* g. it stops moving: profiled 5 s later; the scan finds nothing -> drive on */
	for (int k = 0; k < VC_OBST_STILL_MS / VC_TICK_MS + 1 && !h.o.scan_request; k++) tick_ob(&h, true, 100, 0, false, 0);
	CHECK(h.o.scan_request, "stopped moving -> swept");
	obstacle_t none = { 0 };
	tick_ob(&h, false, 0, &none, false, 0);
	CHECK(h.v.state == VC_LINE_FOLLOW && h.o.motion.type == MOTION_CMD_GO, "scan finds nothing -> resume");
	/* h. no usable side: mission = FAULT, bench = watch it again and re-profile */
	tick_ob(&h, true, 200, 0, false, 0);
	for (int k = 0; k < VC_OBST_STILL_MS / VC_TICK_MS + 1 && !h.o.scan_request; k++) tick_ob(&h, true, 200, 0, false, 0);
	obstacle_t blocked = { .found = true, .distance_mm = 200, .width_mm = 120, .left_clear_mm = 20, .right_clear_mm = 20, .side = 0 };
	tick_ob(&h, true, 200, &blocked, false, 0);
#if VC_BENCH_LINE_ONLY
	CHECK(h.v.state == VC_OBSTACLE_AVOID && h.v.sub == 0, "bench: no clear side -> confirm again (sub %d)", h.v.sub);
	scans = 0;
	for (int k = 0; k < VC_OBST_STILL_MS / VC_TICK_MS + 1; k++) { tick_ob(&h, true, 200, 0, false, 0); scans += h.o.scan_request; }
	CHECK(scans == 1, "bench: re-profiled once more after %d ms (scans %d)", VC_OBST_STILL_MS, scans);
	for (int k = 0; k < VC_SCAN_TIMEOUT_MS / VC_TICK_MS + 2; k++) tick_ob(&h, true, 200, 0, false, 0);
	CHECK(h.v.state == VC_OBSTACLE_AVOID && h.v.sub == 0, "bench: scan timeout -> confirm again");
#else
	CHECK(h.v.state == VC_FAULT && h.v.fault == FAULT_OBSTACLE, "no clear side -> FAULT");
	tick(&h, &RESET, 0, 0, 0, 0, false, false); idle(&h, 2);
	tick(&h, &START, 0, 0, 0, 0, false, false);
	tick_ob(&h, true, 200, 0, false, 0);
	for (int k = 0; k < VC_OBST_STILL_MS / VC_TICK_MS + 1 && !h.o.scan_request; k++) tick_ob(&h, true, 200, 0, false, 0);
	for (int k = 0; k < VC_SCAN_TIMEOUT_MS / VC_TICK_MS + 2; k++) tick_ob(&h, false, 0, 0, false, 0);
	CHECK(h.v.state == VC_FAULT && h.v.fault == FAULT_OBSTACLE, "scan timeout -> FAULT");
#endif
	tick(&h, &RESET, 0, 0, 0, 0, false, false); idle(&h, 2);
	/* i. line finding gives up: the creep never reaches the line -> FAULT_LINE_LOST */
	tick(&h, &START, 0, 0, 0, 0, false, false);
	tick_ob(&h, true, 250, 0, false, 0);
	for (int k = 0; k < VC_OBST_STILL_MS / VC_TICK_MS + 1 && !h.o.scan_request; k++) tick_ob(&h, true, 250, 0, false, 0);
	obstacle_t longer = res;
	longer.diag_mm = 1100;						/* longer than VC_MOVE_MAX_MM: sent in pieces */
	tick_ob(&h, false, 0, &longer, false, 0);
	int pieces[6] = {0}, np = 0;
	for (int k = 0; k < 200 && h.v.sub != 3; k++) {
		tick_ob(&h, false, 0, 0, (k % 5) == 4, 0);
		if (h.o.send_motion && h.o.motion.type == MOTION_CMD_MOVE && np < 6) pieces[np++] = h.o.motion.distance_mm;
	}
	CHECK(np == 4 && pieces[0] == VC_MOVE_MAX_MM && pieces[1] == VC_MOVE_MAX_MM && pieces[2] == 1100 - 2 * VC_MOVE_MAX_MM && pieces[3] == 0,
	      "long leg in pieces: %d %d %d, then %d", pieces[0], pieces[1], pieces[2], pieces[3]);
	uint32_t t_creep = h.now, lim = 1000u + (uint32_t)(1000 * (1100 + VC_REJOIN_EXTRA_MM) / VC_RECOVER_SPEED);
	while (h.v.state == VC_OBSTACLE_AVOID && h.now - t_creep < lim + 1000) tick_ob(&h, false, 0, 0, false, 0);
	CHECK(h.v.state == VC_FAULT && h.v.fault == FAULT_LINE_LOST && h.now - t_creep > lim - VC_TICK_MS,
	      "line not found -> FAULT_LINE_LOST after %u ms (limit %u)", (unsigned)(h.now - t_creep), (unsigned)lim);
	tick(&h, &RESET, 0, 0, 0, 0, false, false); idle(&h, 2);

	/* j. hysteresis: the car stopped right at the stop distance and the obstacle then reads just beyond it (noise, a
	   weak echo off its edge), but within the hold distance. It must stay stopped and profile it after
	   VC_OBST_STILL_MS, not decide it has gone and drive on into it. */
	{
		tick(&h, &START, 0, 0, 0, 0, false, false);
		tick_ob(&h, true, SCAN_CLOSE_MM - 5, 0, false, 0);
		uint32_t t_stop = h.now, t_sw = 0;
		int resumed = 0;
		for (int k = 0; k < 6000 / VC_TICK_MS && !t_sw; k++) {
			if (k < 20 && k % 4 == 0) tick_ob(&h, true, SCAN_CLOSE_MM - 5, 0, false, 0);	/* a few close readings */
			else if (k % 4 == 0) tick_pr(&h, SCAN_CLOSE_MM + 10);			/* then only "still there" */
			else tick_ob(&h, false, 0, 0, false, 0);
			resumed += h.v.state != VC_OBSTACLE_AVOID;
			if (h.o.scan_request) t_sw = h.now;
		}
		CHECK(!resumed && t_sw && t_sw - t_stop >= VC_OBST_STILL_MS,
		      "obstacle just past the stop distance -> stays stopped and is profiled (resumed %d, sweep after %u ms)",
		      resumed, (unsigned)(t_sw - t_stop));
		obstacle_t gone = { 0 };
		tick_ob(&h, false, 0, &gone, false, 0);
		CHECK(h.v.state == VC_LINE_FOLLOW && !h.o.scan_hold, "profile finds nothing -> drive on, hold released");
		/* and when it really goes (nothing close, nothing within the hold), the car drives on as before */
		tick_ob(&h, true, SCAN_CLOSE_MM - 20, 0, false, 0);
		for (int k = 0; k < VC_OBST_CLEAR_MS / VC_TICK_MS + 1 && h.v.state == VC_OBSTACLE_AVOID; k++) tick_ob(&h, false, 0, 0, false, 0);
		CHECK(h.v.state == VC_LINE_FOLLOW, "obstacle gone (no echo within the hold) -> drive on");
		tick(&h, &RESET, 0, 0, 0, 0, false, false); idle(&h, 2);
	}

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
