/*
 *	vc_task.c -- Vehicle Controller: mission state machine, fault handling, command arbitration, watchdog supervision.
 *	Periodic 20 ms. It owns NO hardware except the watchdog feed; every motor command goes to Motion Control (S2).
 *
 *	APIs shown: tk_rcv_mbf, tk_snd_mbf, tk_set_flg, tk_clr_flg, tk_wai_flg (polling, with TWF_BITCLR), tk_loc_mtx
 *	Every call to another subsystem's object uses TMO_POL: the controller never blocks.
 */
#include "subsystem5.h"
#include "../subsystem2/subsystem2.h"		/* flg_motion, mbf_motion_cmd, app_motion, mtx_motion */
#include "../subsystem3/subsystem3.h"		/* flg_line */
#include "../subsystem4/subsystem4.h"		/* flg_imu */
#include "vc_logic.h"
#include "vc_config.h"
#include "drv_scan.h"			/* button_init, button_down */
#include "s5_port.h"
#if VC_WATCHDOG_ENABLE
#endif

static uint32_t s_next_us;

static void period_wait(void)
{
	s_next_us += (uint32_t)VC_TICK_MS * 1000u;
	int32_t delta = (int32_t)(s_next_us - S5_US32());
	if (delta <= 0) { s_next_us = S5_US32(); return; }
	uint32_t ms = (uint32_t)delta / 1000u;
	tk_dly_tsk(ms ? ms : 1);
}

/* true if `bit` of flag `f` is set; with clear = true the bit is consumed */
static bool flag(ID f, UINT bit, bool clear)
{
	UINT p;
	if (f <= 0) return false;
	return tk_wai_flg(f, bit, clear ? (TWF_ORW | TWF_BITCLR) : TWF_ORW, &p, TMO_POL) == E_OK;
}

#define RECV(id, var)	(((id) > 0 && tk_rcv_mbf((id), &(var), TMO_POL) == (INT)sizeof(var)) ? &(var) : NULL)

void vc_task(INT stacd, void *exinf)
{
	vc_t v;
	vc_in_t in;
	vc_out_t out;
	remote_cmd_t remote; nav_cmd_t nav; line_evt_t line; imu_evt_t imu; obstacle_t obst;
	int last_state = -1;
#if VC_AUTOSTART_MS && !VC_START_BUTTON
	bool autostarted = false;
#endif
#if VC_START_BUTTON
	int btn_ticks = 0;			/* consecutive 20 ms ticks held down */
	bool btn_held = false;			/* a debounced press is in progress */
	bool start_pending = false;		/* start once IDLE and the delay has passed */
	uint32_t start_at = 0;
	int btn_cmd = -1;			/* REMOTE_STOP / REMOTE_RESET from the button, waiting for a free tick */
	remote_cmd_t btn_rc;			/* separate from `remote`, so a received MQTT command is never overwritten */
	button_init();
#endif
	(void)stacd; (void)exinf;

	vc_init(&v);
#if VC_WATCHDOG_ENABLE
	s5_wdt_enable(VC_WATCHDOG_MS);	/* NFR10: Pico resets (motor pins fall low) if we stop feeding it */
#endif
	s_next_us = S5_US32();

	for (;;) {
		in = (vc_in_t){0};
		in.now_ms = s5_now_ms();
		in.remote = RECV(mbf_remote_cmd, remote);
		in.nav    = RECV(mbf_nav_cmd, nav);
		in.line   = RECV(mbf_line_evt, line);
		in.imu    = RECV(mbf_imu_evt, imu);
		in.obst   = RECV(mbf_obstacle, obst);
#if VC_START_BUTTON
		{
			btn_ticks = button_down() ? btn_ticks + 1 : 0;
			if (!btn_held && btn_ticks >= 2) {			/* pressed (40 ms, debounced) */
				btn_held = true;
			} else if (btn_held && btn_ticks == 0) {		/* released: act now */
				btn_held = false;
				if (v.state == VC_IDLE || v.state == VC_INIT) {
					start_pending = true;
					start_at = in.now_ms + VC_BUTTON_DELAY_MS;
					S5_PRINTF("[S5] button: starting in %d ms\n", VC_BUTTON_DELAY_MS);
				} else if (v.state == VC_FAULT) {		/* stopped: reset, then start */
					btn_cmd = REMOTE_RESET;
					start_pending = true;
					start_at = in.now_ms + VC_BUTTON_DELAY_MS;
					S5_PRINTF("[S5] button: reset, starting in %d ms\n", VC_BUTTON_DELAY_MS);
				} else {					/* running: stop */
					start_pending = false;
					btn_cmd = REMOTE_STOP;
					S5_PRINTF("[S5] button: STOP (press again to start)\n");
				}
			}
			/* one command per tick: an MQTT command received this tick goes first, the button's waits one tick */
			if (btn_cmd >= 0 && !in.remote) {
				btn_rc.cmd = btn_cmd;
				in.remote = &btn_rc;
				btn_cmd = -1;
			}
			if (start_pending && !in.remote && v.state == VC_IDLE && (int32_t)(in.now_ms - start_at) >= 0) {
				start_pending = false;
				btn_rc.cmd = REMOTE_START;
				in.remote = &btn_rc;
			}
		}
#endif
#if VC_AUTOSTART_MS && !VC_START_BUTTON
		if (!autostarted && !in.remote && v.state == VC_IDLE && in.now_ms >= VC_AUTOSTART_MS) {
			remote.cmd = REMOTE_START;	/* bench start: pretend the MQTT start command arrived */
			in.remote  = &remote;
			autostarted = true;
		}
#endif

		in.obstacle_close = flag(flg_scan, FLG_SCAN_CLOSE, true);
		in.motion_fault   = flag(flg_motion, FLG_MOTION_FAULT, true);

		in.all_ready = flag(flg_motion, FLG_MOTION_READY, false) &&
			       (!VC_REQUIRE_IMU || flag(flg_imu, FLG_IMU_READY, false)) &&
			       flag(flg_line, FLG_LINE_READY, false);
		/* NFR10: each of the three must have set ALIVE since the last tick (consumed here) */
		bool a2 = flag(flg_motion, FLG_MOTION_ALIVE, true);
		bool a3 = flag(flg_line, FLG_LINE_ALIVE, true);
		bool a4 = flag(flg_imu, FLG_IMU_ALIVE, true) || !VC_REQUIRE_IMU;
		in.all_alive = a2 && a3 && a4;

		if (mtx_motion > 0 && tk_loc_mtx(mtx_motion, TMO_POL) == E_OK) {
			motion_status_t ms = app_motion;
			tk_unl_mtx(mtx_motion);
			in.motion_done = ms.cmd_done;
			in.speed_mm_s = (int)(0.5f * (ms.speed_l + ms.speed_r));
		}

#if S5_DEBUG_PRINT
		{
			static const char *const nn[] = { "LEFT", "RIGHT", "STRAIGHT", "U-TURN", "STOP" };
			if (in.nav)
				S5_PRINTF("[S5] barcode command %s received: will do it at %s junction\n",
					  (in.nav->cmd >= 0 && in.nav->cmd <= 4) ? nn[in.nav->cmd] : "?",
					  (v.state == VC_BARCODE_ACTION && v.sub == 0 && v.junction_cmd < 0) ? "THIS" : "the next");
			if (in.line && in.line->type == LINE_EVT_JUNCTION)
				S5_PRINTF("[S5] JUNCTION: %s\n", v.nav_has ? ((v.nav_cmd >= 0 && v.nav_cmd <= 4) ? nn[v.nav_cmd] : "?")
								  : "no barcode yet (if one finishes in the pause it is used)");
		}
#endif
		vc_step(&v, &in, &out);

		/* ---- outputs, safety first ---- */
		if (out.stop_set)
			tk_set_flg(flg_motion, FLG_MOTION_STOP);
		if (out.stop_clear_reset) {
			tk_clr_flg(flg_motion, ~(UINT)(FLG_MOTION_STOP | FLG_MOTION_FAULT));	/* STOP off first ... */
			tk_set_flg(flg_motion, FLG_MOTION_RESET);				/* ... then RESET */
			tk_set_flg(flg_imu, FLG_IMU_RESET);					/* clears the IMU's running max hump */
			tk_clr_flg(flg_scan, ~(UINT)FLG_SCAN_CLOSE);
		}
		if (out.send_motion)
			tk_snd_mbf(mbf_motion_cmd, &out.motion, sizeof out.motion, TMO_POL);
		if (out.scan_request) {
			obstacle_t stale;
			while (tk_rcv_mbf(mbf_obstacle, &stale, TMO_POL) > 0) { }	/* drop an old result */
			tk_set_flg(flg_scan, FLG_SCAN_REQUEST);
		}
		if (out.send_status && mbf_status > 0)
			tk_snd_mbf(mbf_status, &out.status, sizeof out.status, TMO_POL);	/* a full buffer drops it; telemetry never blocks us */
#if VC_WATCHDOG_ENABLE
		if (out.feed_watchdog)
			s5_wdt_feed(VC_WATCHDOG_MS);
#endif
#if S5_DEBUG_PRINT
		if ((int)v.state != last_state) {
			last_state = (int)v.state;
			S5_PRINTF("[S5] state -> %s  fault=%d  max_hump=%d mm\n", vc_state_name(v.state), (int)v.fault, (int)v.max_hump_mm);
#if VC_START_BUTTON
			if (v.state == VC_IDLE && !start_pending)
				S5_PRINTF("[S5] ready: press a button on the Robo Pico (GP20 or GP21) to start\n");
#endif
		}
#endif
		period_wait();
	}
}
