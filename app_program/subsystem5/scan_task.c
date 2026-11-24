/*
 *	scan_task.c -- Obstacle Scan. Owns the servo (GP15) and the ultrasonic sensor (TRIG / ECHO: PIN_US_TRIG / PIN_US_ECHO in drv_scan.h).
 *	WATCH: the servo sweeps continuously left <-> right (SCAN_WATCH_SWEEP_DEG) and pings at every step. An echo inside
 *	the car's path closer than SCAN_CLOSE_MM (two pings in a row at that angle) sets FLG_SCAN_CLOSE, and the servo then
 *	stays pointed at the obstacle until it has gone; s5_track_mm holds its distance (for the VC's still-object check).
 On FLG_SCAN_REQUEST from the Vehicle Controller: sweep the servo 30..150 deg in 5 deg steps, ping at each, build
 *	the profile, send obstacle_t to the VC and leave the rays in s5_profile for the MQTT telemetry (S1).
 *	APIs shown: tk_dly_tsk, tk_set_flg, tk_wai_flg (polling), tk_clr_flg, tk_snd_mbf
 */
#include "subsystem5.h"
#include "drv_scan.h"
#include "scan_algo.h"
#include "vc_config.h"
#include "s5_port.h"

_Static_assert((SCAN_PROFILE_END_DEG - SCAN_PROFILE_START_DEG) / SCAN_PROFILE_STEP_DEG + 1 <= OBST_PROFILE_MAX_RAYS &&
	       OBST_PROFILE_MAX_RAYS <= SCAN_MAX_RAYS, "profile sweep has more rays than the buffers hold");
_Static_assert(SCAN_CLOSE_MM < SCAN_NEAR_MM, "an obstacle that stops the car must also count as near in the profile sweep");

/* Servo angle last commanded (0 = straight ahead, + = left). servo_glide() moves from here one degree at a time, so
   the servo never jumps and does not jolt the sensor or the car. */
static int s_servo_deg;

static void servo_to(int deg)
{
	servo_set_deg(deg);
	s_servo_deg = deg;
}

static void servo_glide(int deg)
{
	while (s_servo_deg != deg) {
		s_servo_deg += deg > s_servo_deg ? 1 : -1;
		servo_set_deg(s_servo_deg);
		tk_dly_tsk(SCAN_GLIDE_MS_PER_DEG);
	}
	servo_set_deg(deg);			/* also re-arms the pulses after servo_off() */
}

static int ping_median3(void)
{
	int a = ultra_ping_mm(); tk_dly_tsk(2);
	int b = ultra_ping_mm(); tk_dly_tsk(2);
	int c = ultra_ping_mm();
	/* median of three; -1 (no echo) sorts lowest, so treat two no-echoes as no echo */
	if (a > b) { int t = a; a = b; b = t; }
	if (b > c) { int t = b; b = c; c = t; }
	if (a > b) { int t = a; a = b; b = t; }
	return b;
}

/* Distance of the obstacle being tracked (0 = none), for the VC's still-object check. */
volatile int			s5_track_mm;

/* The last profile, for the telemetry (S1, other core). Seqlock: odd while being written. */
volatile obstacle_profile_t	s5_profile;
volatile unsigned		s5_profile_seq;

static void publish_profile(const int *ang, const int *dist, int n, const obstacle_t *ob)
{
	s5_profile_seq++;				/* odd: S1 must not use the record now */
	__asm__ volatile ("dmb" ::: "memory");
	s5_profile.n = n;
	for (int i = 0; i < n; i++) {
		s5_profile.servo_deg[i] = (int16_t)(ang[i] + SCAN_SERVO_CENTRE_DEG);
		s5_profile.dist_mm[i] = (int16_t)dist[i];
	}
	s5_profile.ob.found = ob->found;
	s5_profile.ob.distance_mm = ob->distance_mm;
	s5_profile.ob.width_mm = ob->width_mm;
	s5_profile.ob.left_clear_mm = ob->left_clear_mm;
	s5_profile.ob.right_clear_mm = ob->right_clear_mm;
	s5_profile.ob.side = ob->side;
	s5_profile.ob.left_edge_mm = ob->left_edge_mm;
	s5_profile.ob.right_edge_mm = ob->right_edge_mm;
	s5_profile.ob.scan_left_mm = ob->scan_left_mm;
	s5_profile.ob.scan_right_mm = ob->scan_right_mm;
	s5_profile.ob.depth_mm = ob->depth_mm;
	s5_profile.ob.turn_deg = ob->turn_deg;
	s5_profile.ob.diag_mm = ob->diag_mm;
	s5_profile.ob.lat_mm = ob->lat_mm;
	s5_profile.ob.pass_mm = ob->pass_mm;
	s5_profile.timestamp_ms = s5_now_ms();
	__asm__ volatile ("dmb" ::: "memory");
	s5_profile_seq++;				/* even again: a new, complete profile */
}

/* Profile the obstacle: servo SCAN_PROFILE_START_DEG .. SCAN_PROFILE_END_DEG (30, 35, ... 150) in
   SCAN_PROFILE_STEP_DEG steps, gliding between them, one ultrasonic reading (median of 3) at each, then analyse and
   publish the rays for MQTT. The servo glides back to back_deg (the obstacle it was tracking) BEFORE the result goes
   to the VC: if the VC has to watch the obstacle again (no route on the bench) the tracking pings must start at once,
   not after a glide of up to ~0.7 s that the VC would take for "it has gone". seed_deg = the angle the obstacle was
   hit at (it stopped the car): the profile grows the obstacle from that ray. */
static void do_sweep(int back_deg, int seed_deg)
{
	int ang[OBST_PROFILE_MAX_RAYS], dist[OBST_PROFILE_MAX_RAYS], n = 0;
	obstacle_t ob;

	for (int a = SCAN_PROFILE_START_DEG; a <= SCAN_PROFILE_END_DEG && n < OBST_PROFILE_MAX_RAYS; a += SCAN_PROFILE_STEP_DEG) {
		int rel = a - SCAN_SERVO_CENTRE_DEG;	/* servo_set_deg() wants 0 = straight ahead, + = left */
		servo_glide(rel);
		tk_dly_tsk(SCAN_SETTLE_MS);
		ang[n] = rel;				/* ascending, as scan_analyze() needs */
		dist[n] = ping_median3();
#if S5_DEBUG_PRINT
		S5_PRINTF("[S5] profile: servo %d deg -> %d mm\n", a, dist[n]);
#endif
		n++;
	}
	scan_analyze_from(ang, dist, n, seed_deg, &ob);
	publish_profile(ang, dist, n, &ob);
#if S5_DEBUG_PRINT
	S5_PRINTF("[S5] scan (from %d deg): found=%d dist=%d width=%d (edges %d..%d, scanned %d..%d, planned depth %d) left=%d right=%d side=%d\n",
		  seed_deg, ob.found, (int)ob.distance_mm, (int)ob.width_mm, (int)ob.right_edge_mm, (int)ob.left_edge_mm,
		  (int)ob.scan_right_mm, (int)ob.scan_left_mm, (int)ob.depth_mm, (int)ob.left_clear_mm, (int)ob.right_clear_mm, (int)ob.side);
	if (ob.side)
		S5_PRINTF("[S5] plan: turn %d deg, %d mm until beside it, turn back; then %d mm on, turn %d deg, find the line\n",
			  (int)ob.turn_deg, (int)ob.diag_mm, (int)ob.pass_mm, -(int)ob.turn_deg);
	else if (ob.found)
		S5_PRINTF("[S5] no route: left %d mm free, needs %d; right %d mm free, needs %d (-1 = too wide at any angle)\n",
			  (int)ob.left_clear_mm, (int)scan_route_need(&ob, +1), (int)ob.right_clear_mm, (int)scan_route_need(&ob, -1));
#endif
	servo_glide(back_deg);
	tk_snd_mbf(mbf_obstacle, &ob, sizeof ob, TMO_POL);
}

/* two pings at the current servo angle. *close: BOTH are an obstacle in the path closer than SCAN_CLOSE_MM (stop the
   car). *held: EITHER is in the path closer than SCAN_HOLD_MM (a stopped-for obstacle is still there). *dist gets the
   smaller valid reading. */
static void ping_pair(int ang, int *dist, bool *close, bool *held)
{
	int a = ultra_ping_mm();
	tk_dly_tsk(2);
	int b = ultra_ping_mm();
	*dist = (a > 0 && (b <= 0 || a < b)) ? a : b;
	*close = scan_in_path(ang, a, SCAN_CLOSE_MM, SCAN_PATH_HALF_W_MM) &&
		 scan_in_path(ang, b, SCAN_CLOSE_MM, SCAN_PATH_HALF_W_MM);
	*held = scan_in_path(ang, a, SCAN_HOLD_MM, SCAN_PATH_HALF_W_MM) ||
		scan_in_path(ang, b, SCAN_HOLD_MM, SCAN_PATH_HALF_W_MM);
}

void scan_task(INT stacd, void *exinf)
{
	const int lo = SCAN_WATCH_DEG - SCAN_WATCH_SWEEP_DEG, hi = SCAN_WATCH_DEG + SCAN_WATCH_SWEEP_DEG;
	int ang = lo, dir = +1;
	bool tracking = false;			/* parked on an obstacle until it has gone */
	int track_ang = 0, clear_pings = 0, no_echo = 0, last_d = 0;
	unsigned n_ping = 0; (void)n_ping; (void)no_echo;
	(void)stacd; (void)exinf;

	servo_init();
	ultra_init();
	servo_to(SCAN_WATCH_DEG);
	tk_dly_tsk(300);			/* let the servo reach its watch angle */
#if S5_SERVO_TEST
	{
		static const int seq[] = { 0, -60, -40, -20, 0, 20, 40, 60, 0 };
		for (unsigned i = 0; i < sizeof seq / sizeof seq[0]; i++) {
			servo_set_deg(seq[i]);
			tk_dly_tsk(500);
			S5_PRINTF("[S5] servo test: angle %d deg (+ = left), ultra=%d mm\n", seq[i], ultra_ping_mm());
		}
		servo_to(SCAN_WATCH_DEG);
		tk_dly_tsk(300);
	}
#endif

	servo_off();
	bool active = false;

	for (;;) {
		UINT ptn;
		bool run = tk_wai_flg(flg_scan, FLG_SCAN_RUN, TWF_ORW, &ptn, TMO_POL) == E_OK;	/* level: not consumed */
		if (!run) {
			if (active) {			/* just stopped: park straight ahead, then let go of the servo */
				active = false;
				tracking = false;
				s5_track_mm = 0;
				ang = lo; dir = +1;
				no_echo = 0; last_d = 0;
				servo_glide(SCAN_WATCH_DEG);
				servo_off();
				tk_clr_flg(flg_scan, ~(UINT)(FLG_SCAN_CLOSE | FLG_SCAN_PRESENT | FLG_SCAN_REQUEST));
#if S5_ULTRA_PRINT
				S5_PRINTF("[S5] scan: idle (car stopped)\n");
#endif
			}
			tk_dly_tsk(SCAN_PERIOD_MS);
			continue;
		}
		if (!active) {				/* just started: re-arm the servo and let it reach the watch angle */
			active = true;
			servo_glide(SCAN_WATCH_DEG);
#if S5_ULTRA_PRINT
			S5_PRINTF("[S5] scan: active\n");
#endif
		}

		if (tk_wai_flg(flg_scan, FLG_SCAN_REQUEST, TWF_ORW | TWF_BITCLR, &ptn, TMO_POL) == E_OK) {
			do_sweep(tracking ? track_ang : ang, tracking ? track_ang : 0);	/* ends back on the obstacle */
			tk_clr_flg(flg_scan, ~(UINT)(FLG_SCAN_CLOSE | FLG_SCAN_PRESENT));
			tk_dly_tsk(SCAN_PERIOD_MS);
			continue;
		}

		if (tracking) {
			int d;
			bool close, held;
			bool hold = tk_wai_flg(flg_scan, FLG_SCAN_HOLD, TWF_ORW, &ptn, TMO_POL) == E_OK;	/* level */
			ping_pair(track_ang, &d, &close, &held);
			if (close || (hold && held)) {		/* still there: close, or (car stopped for it) within the hold */
				s5_track_mm = d;
				tk_set_flg(flg_scan, close ? (FLG_SCAN_CLOSE | FLG_SCAN_PRESENT) : FLG_SCAN_PRESENT);
				clear_pings = 0;
			} else if (++clear_pings >= SCAN_TRACK_CLEAR_PINGS) {
				tracking = false;
				s5_track_mm = 0;
			}
			last_d = d;
			tk_dly_tsk(SCAN_PERIOD_MS);
		} else {
			int d;
			bool close, held;
			servo_to(ang);
			tk_dly_tsk(SCAN_WATCH_SETTLE_MS);
			ping_pair(ang, &d, &close, &held);
			if (close) {
				s5_track_mm = d;
				tk_set_flg(flg_scan, FLG_SCAN_CLOSE);
				tracking = true;
				track_ang = ang;
				clear_pings = 0;
			}
			last_d = d;
			if (SCAN_WATCH_SWEEP_DEG > 0) {
				ang += dir * SCAN_WATCH_STEP_DEG;
				if (ang >= hi) { ang = hi; dir = -1; }
				if (ang <= lo) { ang = lo; dir = +1; }
			}
		}

		/* No echo for a while only means nothing is in range (an open track). The wiring search runs ONCE, at
		   power-up in ultra_init(): here it would busy-wait ~1 s with no obstacle watch and drive unclaimed pins. */
		no_echo = (last_d <= 0) ? no_echo + 1 : 0;
#if S5_ULTRA_PRINT
		if (++n_ping % 10 == 0) {
			int shown = tracking ? track_ang : ang;
			S5_PRINTF("[S5] ultra=%d mm at %d deg %s (stop if < %d mm ahead)\n", last_d, shown,
				  tracking ? "TRACKING OBSTACLE" : "sweeping", SCAN_CLOSE_MM);
			if (no_echo >= 30)
				S5_PRINTF("[S5] ultra: NO ECHO. Check the sensor's 5V and GND (TRIG GP%d, ECHO GP%d)\n", ultra_trig_pin(), ultra_echo_pin());
		}
#endif
	}
}
