/*
 *	scan_task.c -- Obstacle Scan. Owns the servo (GP15) and the ultrasonic sensor (TRIG GP17 / ECHO GP16).
 *	WATCH: the servo sweeps continuously left <-> right (SCAN_WATCH_SWEEP_DEG) and pings at every step. An echo inside
 *	the car's path closer than SCAN_CLOSE_MM (two pings in a row at that angle) sets FLG_SCAN_CLOSE, and the servo then
 *	stays pointed at the obstacle until it has gone.
 *	On FLG_SCAN_REQUEST from the Vehicle Controller: sweep the servo, build the profile, send obstacle_t.
 *	APIs shown: tk_dly_tsk, tk_set_flg, tk_wai_flg (polling), tk_clr_flg, tk_snd_mbf
 */
#include "subsystem5.h"
#include "drv_scan.h"
#include "scan_algo.h"
#include "vc_config.h"
#include "s5_port.h"

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

static void do_sweep(void)
{
	int ang[SCAN_MAX_RAYS], dist[SCAN_MAX_RAYS], n = 0;
	obstacle_t ob;

	for (int a = -SCAN_ANGLE_MAX; a <= SCAN_ANGLE_MAX && n < SCAN_MAX_RAYS; a += SCAN_ANGLE_STEP) {
		servo_set_deg(a);
		tk_dly_tsk(SCAN_SETTLE_MS);
		ang[n] = a;
		dist[n] = ping_median3();
		n++;
	}
	scan_analyze(ang, dist, n, &ob);
#if S5_DEBUG_PRINT
	S5_PRINTF("[S5] scan: found=%d dist=%d width=%d left=%d right=%d side=%d\n", ob.found, (int)ob.distance_mm,
		  (int)ob.width_mm, (int)ob.left_clear_mm, (int)ob.right_clear_mm, (int)ob.side);
#endif
	tk_snd_mbf(mbf_obstacle, &ob, sizeof ob, TMO_POL);
}

/* two pings at the current servo angle; true if both are an obstacle in the path. *dist gets the smaller valid reading. */
static bool ping_hit(int ang, int *dist)
{
	int a = ultra_ping_mm();
	tk_dly_tsk(2);
	int b = ultra_ping_mm();
	*dist = (a > 0 && (b <= 0 || a < b)) ? a : b;
	return scan_in_path(ang, a, SCAN_CLOSE_MM, SCAN_PATH_HALF_W_MM) &&
	       scan_in_path(ang, b, SCAN_CLOSE_MM, SCAN_PATH_HALF_W_MM);
}

void scan_task(INT stacd, void *exinf)
{
	const int lo = SCAN_WATCH_DEG - SCAN_WATCH_SWEEP_DEG, hi = SCAN_WATCH_DEG + SCAN_WATCH_SWEEP_DEG;
	int ang = lo, dir = +1;			/* where the sweep is, and which way it is going */
	bool tracking = false;			/* parked on an obstacle until it has gone */
	int track_ang = 0, clear_pings = 0, no_echo = 0, last_d = 0;
	unsigned n_ping = 0; (void)n_ping;
	(void)stacd; (void)exinf;

	servo_init();
	ultra_init();
	servo_set_deg(SCAN_WATCH_DEG);
	tk_dly_tsk(300);			/* let the servo reach its watch angle */
#if S5_SERVO_TEST
	{
		static const int seq[] = { 0, -60, -40, -20, 0, 20, 40, 60, 0 };
		for (unsigned i = 0; i < sizeof seq / sizeof seq[0]; i++) {
			servo_set_deg(seq[i]);
			tk_dly_tsk(500);
			S5_PRINTF("[S5] servo test: angle %d deg (+ = left), ultra=%d mm\n", seq[i], ultra_ping_mm());
		}
		servo_set_deg(SCAN_WATCH_DEG);
		tk_dly_tsk(300);
	}
#endif

	for (;;) {
		UINT ptn;
		if (tk_wai_flg(flg_scan, FLG_SCAN_REQUEST, TWF_ORW | TWF_BITCLR, &ptn, TMO_POL) == E_OK) {
			do_sweep();			/* the VC stopped the car and wants the obstacle profiled */
			tk_clr_flg(flg_scan, ~(UINT)FLG_SCAN_CLOSE);
			if (tracking)
				servo_set_deg(track_ang);	/* go back to the obstacle and keep watching it */
			tk_dly_tsk(SCAN_PERIOD_MS);
			continue;
		}

		if (tracking) {				/* parked: ping the obstacle until it has gone */
			int d;
			if (ping_hit(track_ang, &d)) {
				tk_set_flg(flg_scan, FLG_SCAN_CLOSE);
				clear_pings = 0;
			} else if (++clear_pings >= SCAN_TRACK_CLEAR_PINGS) {
				tracking = false;
			}
			last_d = d;
			tk_dly_tsk(SCAN_PERIOD_MS);
		} else {				/* sweep: move one step, settle, ping */
			int d;
			servo_set_deg(ang);
			tk_dly_tsk(SCAN_WATCH_SETTLE_MS);
			if (ping_hit(ang, &d)) {
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

		no_echo = (last_d <= 0) ? no_echo + 1 : 0;
		if (no_echo == 60 || (no_echo > 60 && no_echo % 300 == 0)) {	/* still nothing: look for the wiring again */
			if (ultra_autodetect()) {
				S5_PRINTF("[S5] ultrasonic FOUND: TRIG=GP%d ECHO=GP%d\n", ultra_trig_pin(), ultra_echo_pin());
				no_echo = 0;
			}
		}
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
