/*
 *	imu_task.c
 *	Subsystem 4 -- imu_task: periodic 10 ms, owns I2C0 (GP0/GP1).
 *
 *	APIs shown: tk_dly_tsk, tk_set_flg, tk_clr_flg, tk_wai_flg (polling), tk_loc_mtx / tk_unl_mtx,
 *	            tk_snd_mbf, tk_rcv_mbf
 *
 *	Flow (statechart 6.3):
 *	  bring up I2C + chip -> CALIBRATE (2 s stationary average) -> loop every 10 ms:
 *	      read IMU -> map axes -> read Motion Control's status -> imu_algo_step()
 *	      (Region A: motion class, Region B: hump height, SUDDEN_IMPACT) -> send imu_evt_t
 *
 *	Rules from the design review / TEAM.md that this file follows:
 *	  - Only this task touches the IMU and I2C0.
 *	  - It never drives motors and never sets the STOP flag (the Vehicle Controller does that
 *	    when it receives IMU_EVT_IMPACT).
 *	  - It sets FLG_IMU_ALIVE every cycle so the Vehicle Controller can feed the watchdog (NFR10).
 *	  - It never blocks the 10 ms loop on another subsystem's object (all calls use TMO_POL).
 */

#include "subsystem4.h"
#include "drv_imu.h"
#include "imu_algo.h"
#include "imu_config.h"
#include "s4_port.h"

/* ------------------------------------------------------------------ helpers */

/* fixed 10 ms cadence without drift: remember the absolute deadline, re-sync after an overrun */
static uint32_t s_next_us;

static void period_start(void)
{
	s_next_us = S4_US32();
}

static void period_wait(void)
{
	s_next_us += (uint32_t)IMU_PERIOD_MS * 1000u;
	int32_t delta = (int32_t)(s_next_us - S4_US32());
	if (delta <= 0) {			/* overran: do not try to catch up */
		s_next_us = S4_US32();
		return;
	}
	uint32_t ms = (uint32_t)delta / 1000u;	/* tk_dly_tsk may add up to one tick; the deadline absorbs it */
	tk_dly_tsk(ms ? ms : 1);
}

static void alive(void)
{
	tk_set_flg(flg_imu, FLG_IMU_ALIVE);
}

/* wait without ever letting the ALIVE flag go stale */
static void delay_alive(uint32_t ms)
{
	for (uint32_t t = 0; t < ms; t += IMU_PERIOD_MS) {
		alive();
		tk_dly_tsk(IMU_PERIOD_MS);
	}
}

/* Never block. If the buffer is full, drop non-critical events; for IMPACT / SENSOR_FAULT make
   room by discarding the oldest queued event so the Vehicle Controller always sees them. */
static void print_evt(const imu_evt_t *e)
{
	char b0[16], b1[16];
	switch (e->type) {
	case IMU_EVT_HUMP:
		S4_PRINTF(">> HUMP    peak = %s mm   running max = %s mm\n",
			  s4_fx(b0, e->peak_mm, 1), s4_fx(b1, e->max_hump_mm, 1));
		break;
	case IMU_EVT_IMPACT:
		S4_PRINTF(">> IMPACT  |a| = %s g\n", s4_fx(b0, e->accel_g, 2));
		break;
	case IMU_EVT_MOTION_CLASS:
		S4_PRINTF(">> CLASS   %s  (turn %s dps, pitch %s deg)\n", motion_class_name(e->motion_class),
			  s4_fx(b0, e->turn_rate_dps, 1), s4_fx(b1, e->pitch_deg, 1));
		break;
	case IMU_EVT_SENSOR_FAULT:
		S4_PRINTF(">> SENSOR_FAULT\n");
		break;
	}
}

/* Until the Vehicle Controller creates mbf_imu_evt (IF-06), events are printed instead. */
static void publish_evt(const imu_evt_t *e)
{
#if S4_DEBUG_PRINT
	/* class changes only; the 500 ms heartbeat would flood the console */
	static motion_class_t last_cls = MOTION_CLASS_STATIONARY;
	if (mbf_imu_evt <= 0) {
		if (e->type != IMU_EVT_MOTION_CLASS || e->motion_class != last_cls) print_evt(e);
		if (e->type == IMU_EVT_MOTION_CLASS) last_cls = e->motion_class;
		return;
	}
#else
	if (mbf_imu_evt <= 0) return;
#endif
	if (tk_snd_mbf(mbf_imu_evt, e, S4_IMU_EVT_MAXMSZ, TMO_POL) == E_OK) return;
	if (e->type == IMU_EVT_IMPACT || e->type == IMU_EVT_SENSOR_FAULT) {
		imu_evt_t dropped;
		(void)tk_rcv_mbf(mbf_imu_evt, &dropped, TMO_POL);
		(void)tk_snd_mbf(mbf_imu_evt, e, S4_IMU_EVT_MAXMSZ, TMO_POL);
	}
}

static void report_fault_once(bool *reported, const char *why)
{
	if (*reported) return;
	*reported = true;
	imu_evt_t e;
	e = (imu_evt_t){0};
	e.type = IMU_EVT_SENSOR_FAULT;
	e.timestamp_ms = s4_now_ms();
	publish_evt(&e);
	S4_PRINTF("[IMU] SENSOR_FAULT: %s\n", why);
}

/* sensor axes -> robot axes (X fwd, Y left, Z up); see IMU_MAP_* in imu_config.h */
static void map_axes(const imu_raw_t *r, float a[3], float g[3])
{
	a[0] = IMU_MAP_X(r->ax_g,   r->ay_g,   r->az_g);
	a[1] = IMU_MAP_Y(r->ax_g,   r->ay_g,   r->az_g);
	a[2] = IMU_MAP_Z(r->ax_g,   r->ay_g,   r->az_g);
	g[0] = IMU_MAP_X(r->gx_dps, r->gy_dps, r->gz_dps);
	g[1] = IMU_MAP_Y(r->gx_dps, r->gy_dps, r->gz_dps);
	g[2] = IMU_MAP_Z(r->gx_dps, r->gy_dps, r->gz_dps);
}

/* ------------------------------------------------------------------ CALIBRATE */
static bool calibrate(imu_cal_result_t *res)
{
	for (int attempt = 1; attempt <= IMU_CAL_MAX_TRIES; attempt++) {
		S4_PRINTF("[IMU] calibrating (attempt %d/%d): keep the robot still and level...\n",
			  attempt, IMU_CAL_MAX_TRIES);

		imu_cal_t cal;
		imu_cal_reset(&cal);
		uint32_t fails = 0;
		period_start();

		while (cal.n < IMU_CAL_SAMPLES) {
			period_wait();
			alive();

			imu_raw_t raw;
			if (!imu_hw_read(&raw)) {
				if (++fails >= IMU_MAX_READ_FAILS) return false;
				continue;
			}
			fails = 0;
			float a[3], g[3];
			map_axes(&raw, a, g);
			imu_cal_add(&cal, a[0], a[1], a[2], g[0], g[1], g[2]);
		}

		const imu_cal_status_t st = imu_cal_finish(&cal, res);
		float worst = res->gyro_std_dps[0];
		if (res->gyro_std_dps[1] > worst) worst = res->gyro_std_dps[1];
		if (res->gyro_std_dps[2] > worst) worst = res->gyro_std_dps[2];
		char b0[16], b1[16], b2[16], b3[16], b4[16], b5[16];
		S4_PRINTF("[IMU]   |a|=%s g  gyro bias = %s %s %s dps  noise(std) max = %s dps  mount pitch offset = %s deg\n",
			  s4_fx(b0, res->accel_mag_g, 3),
			  s4_fx(b1, res->gyro_bias_dps[0], 2), s4_fx(b2, res->gyro_bias_dps[1], 2),
			  s4_fx(b3, res->gyro_bias_dps[2], 2),
			  s4_fx(b4, worst, 2), s4_fx(b5, res->pitch_offset_deg, 2));

		if (st == IMU_CAL_OK) { S4_PRINTF("[IMU] calibration OK\n"); return true; }
		if (st == IMU_CAL_MOVED)
			S4_PRINTF("[IMU]   robot moved during calibration - retrying\n");
		else
			S4_PRINTF("[IMU]   |a| is not ~1 g: wrong chip/register map, or the board is not at rest - retrying\n");
		delay_alive(500);
	}
	return false;
}

/* ------------------------------------------------------------------ 10 ms loop */
/* returns only on repeated I2C failure */
static void run_loop(imu_algo_t *algo, const imu_cal_result_t *cal, IMU_DEBUG *dbg)
{
	motion_status_t last_motion;
	last_motion = (motion_status_t){0};

	uint32_t t_prev_wake_us = S4_US32();
	uint32_t t_prev_step_us = t_prev_wake_us;
	uint32_t fails = 0;
	uint8_t  dbg_div = 0;
	uint8_t  print_div = 0;
	bool     first = true;

	period_start();
	for (;;) {
		period_wait();
		const uint32_t t0 = S4_US32();
		alive();							/* alive, every cycle */

		if (!first) {							/* A6: log jitter */
			int32_t j = (int32_t)(t0 - t_prev_wake_us) - (int32_t)(IMU_PERIOD_MS * 1000);
			if (j < 0) j = -j;
			if ((uint32_t)j > dbg->jitter_us_max) dbg->jitter_us_max = (uint32_t)j;
		}
		first = false;
		t_prev_wake_us = t0;

		/* ---- read the sensor ---- */
		imu_raw_t raw;
		if (!imu_hw_read(&raw)) {
			dbg->read_fails++;
			if (++fails >= IMU_MAX_READ_FAILS) return;		/* -> SENSOR_FAULT */
			continue;
		}
		fails = 0;

		/* ---- mission reset requested by the Vehicle Controller? (FR9) ---- */
		UINT ptn;
		if (tk_wai_flg(flg_imu, FLG_IMU_RESET, TWF_ORW | TWF_BITCLR, &ptn, TMO_POL) == E_OK) {
			imu_algo_reset_mission(algo);
		}

		/* ---- Motion Control's status (IF-08): poll the mutex, reuse the last copy if busy ---- */
		if (mtx_motion > 0) {
			if (tk_loc_mtx(mtx_motion, TMO_POL) == E_OK) {
				last_motion = app_motion;
				tk_unl_mtx(mtx_motion);
			} else {
				dbg->motion_busy++;
			}
		} else {
			/* Motion Control not there yet: pretend to drive at S4_FAKE_SPEED_MM_S so the hump
			   maths can be tested by tilting the board by hand (0 = robot standing still). */
			last_motion.speed_l = last_motion.speed_r = S4_FAKE_SPEED_MM_S;
			last_motion.distance_mm += S4_FAKE_SPEED_MM_S * 0.010f;
		}

		/* ---- build algorithm input ---- */
		imu_algo_in_t in;
		float a[3], g[3];
		map_axes(&raw, a, g);
		in.ax_g = a[0];  in.ay_g = a[1];  in.az_g = a[2];
		in.gx_dps = g[0] - cal->gyro_bias_dps[0];
		in.gy_dps = g[1] - cal->gyro_bias_dps[1];
		in.gz_dps = g[2] - cal->gyro_bias_dps[2];

		float dt = (float)(t0 - t_prev_step_us) * 1e-6f;		/* measured, clamped */
		if (dt < 0.005f) dt = 0.005f;
		if (dt > 0.020f) dt = 0.020f;
		t_prev_step_us = t0;
		in.dt_s   = dt;
		in.now_ms = s4_now_ms();

		in.speed_valid = true;
		in.speed_mm_s  = 0.5f * (last_motion.speed_l + last_motion.speed_r);
		in.dist_valid  = true;
		in.distance_mm = last_motion.distance_mm;

		/* ---- the statechart ---- */
		imu_algo_out_t out;
		imu_algo_step(algo, &in, &out);
		for (uint8_t i = 0; i < out.n; i++) publish_evt(&out.evt[i]);

		/* ---- bookkeeping ---- */
		dbg->samples++;
		const uint32_t exec = S4_US32() - t0;
		if (exec > dbg->exec_us_max) dbg->exec_us_max = exec;

		if (++dbg_div >= 5) {						/* snapshot at 20 Hz */
			dbg_div = 0;
			dbg->cls           = algo->cls;
			dbg->hump_state    = (int)algo->hump_state;
			dbg->pitch_deg     = algo->pitch_deg;
			dbg->turn_rate_dps = algo->turn_rate_dps;
			dbg->accel_mag_g   = algo->accel_mag_g;
			dbg->hump_h_mm     = algo->hump_h_mm;
			dbg->max_hump_mm   = algo->max_hump_mm;
			s4_imu_debug = *dbg;
#if S4_DEBUG_PRINT
			if ((++print_div % 5) == 0) {				/* status line, 4 Hz */
				char p0[16], p1[16], p2[16], p3[16], p4[16];
				S4_PRINTF("pitch %s  yaw %s dps  |a| %s g  %s  hump:%s  h=%s mm  MAX=%s mm"
					  "  | exec<=%u us jitter<=%u us fails=%u\n",
					  s4_fx(p0, dbg->pitch_deg, 1), s4_fx(p1, dbg->turn_rate_dps, 1),
					  s4_fx(p2, dbg->accel_mag_g, 2), motion_class_name(dbg->cls),
					  hump_state_name((hump_state_t)dbg->hump_state),
					  s4_fx(p3, dbg->hump_h_mm, 1), s4_fx(p4, dbg->max_hump_mm, 1),
					  (unsigned)dbg->exec_us_max, (unsigned)dbg->jitter_us_max,
					  (unsigned)dbg->read_fails);
			}
#endif
		}
	}
}

/* ------------------------------------------------------------------ the task */
EXPORT void imu_task(INT stacd, void *exinf)
{
	(void)stacd;
	(void)exinf;

	imu_algo_t       algo;
	imu_cal_result_t cal;
	IMU_DEBUG        dbg;
	bool calibrated     = false;
	bool fault_reported = false;

	dbg = (IMU_DEBUG){0};
	cal = (imu_cal_result_t){0};

	for (;;) {
		/* ---- bring up the chip (retries forever, ALIVE flag stays set) ---- */
		if (!imu_hw_init(delay_alive)) {
			dbg.fault = true;
			s4_imu_debug = dbg;
			report_fault_once(&fault_reported, "IMU not responding on I2C0");
			delay_alive(IMU_RETRY_PERIOD_MS);
			continue;
		}

		/* ---- CALIBRATE (once per boot; biases are kept across a bus recovery) ---- */
		if (!calibrated) {
			if (!calibrate(&cal)) {
				dbg.fault = true;
				s4_imu_debug = dbg;
				report_fault_once(&fault_reported, "calibration failed");
				delay_alive(IMU_RETRY_PERIOD_MS);
				continue;
			}
			imu_algo_init(&algo, cal.pitch_offset_deg, s4_now_ms());
			calibrated = true;
		}

		dbg.fault      = false;
		fault_reported = false;
		tk_set_flg(flg_imu, FLG_IMU_READY);
		S4_PRINTF("[IMU] running at %d Hz\n", 1000 / IMU_PERIOD_MS);

		run_loop(&algo, &cal, &dbg);		/* only returns on repeated I2C failure */

		tk_clr_flg(flg_imu, ~(UINT)FLG_IMU_READY);
		dbg.fault = true;
		s4_imu_debug = dbg;
		report_fault_once(&fault_reported, "repeated I2C read failures");
	}
}
