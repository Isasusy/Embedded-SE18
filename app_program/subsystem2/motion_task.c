/*
 *	motion_task.c
 *	Subsystem 2 -- motion_task: periodic 10 ms, the ONLY writer of the motors and reader of the encoders.
 *
 *	APIs shown: tk_dly_tsk, tk_set_flg, tk_clr_flg, tk_wai_flg (polling), tk_rcv_mbf, tk_loc_mtx / tk_unl_mtx
 *
 *	Each loop (statechart 6.2):
 *	  read encoders -> poll STOP/RESET flags -> take one command -> read line steering
 *	  -> mc_step() (state machine + PI per wheel) -> motors -> publish motion_status_t -> set ALIVE
 *
 *	Rules from the design review / TEAM.md followed here:
 *	  - no other task touches GP8-11 (PWM) or GP2-5 (encoders);
 *	  - the STOP flag is checked every loop, so motors are off within 10 ms (NFR6);
 *	  - ALIVE is set every loop for the Vehicle Controller's watchdog (NFR10);
 *	  - nothing blocks: every call to another subsystem's object uses TMO_POL.
 */

#include "subsystem2.h"
#include "drv_motor.h"
#include "drv_encoder.h"
#include "motion_ctrl.h"
#include "motion_config.h"
#include "s2_port.h"

/* ------------------------------------------------------------ 10 ms cadence without drift */
static uint32_t s_next_us;
static unsigned s_late_max_us;

static void period_start(void)
{
	s_next_us = S2_US32();
}

static void period_wait(void)
{
	s_next_us += (uint32_t)MC_PERIOD_MS * 1000u;
	int32_t delta = (int32_t)(s_next_us - S2_US32());
	if (delta <= 0) {			/* overran: do not try to catch up */
		s_next_us = S2_US32();
		return;
	}
	uint32_t ms = (uint32_t)delta / 1000u;
	tk_dly_tsk(ms ? ms : 1);
	int32_t late = (int32_t)(S2_US32() - s_next_us);	/* how late did we wake (NFR2: < 2 ms) */
	if (late > 0 && (unsigned)late > s_late_max_us)
		s_late_max_us = (unsigned)late;
}

/* ------------------------------------------------------------ optional bench script */
#if S2_BENCH_SCRIPT
typedef struct { motion_cmd_t cmd; unsigned wait_cycles; } step_t;	/* wait_cycles 0 = until cmd_done */
static const step_t s_script[] = {
	{ { MOTION_CMD_GO,    150,   0,   0 }, 300 },
	{ { MOTION_CMD_TURN,    0,  90,   0 },   0 },
	{ { MOTION_CMD_MOVE,  150,   0, 300 },   0 },
	{ { MOTION_CMD_UTURN,   0,   0,   0 },   0 },
	{ { MOTION_CMD_STOP,    0,   0,   0 }, 200 },
};
#define N_STEPS	(sizeof s_script / sizeof s_script[0])
#endif

/* ------------------------------------------------------------ the task */
void motion_task(INT stacd, void *exinf)
{
	mc_t		mc;
	mc_in_t		in;
	mc_out_t	out;
	motion_cmd_t	cmd;
	int32_t		enc_l, enc_r, last_l, last_r;
	uint32_t	loops = 0, last_print_ms = 0;
	unsigned	exec_max_us = 0;
#if S2_BENCH_SCRIPT
	unsigned	step = 0, step_cycles = 0;
	bool		step_sent = false;
#endif
	(void)stacd; (void)exinf;

	motor_init();
	motor_off();
	enc_read(&last_l, &last_r);		/* enc_task owns enc_init() and the 1 ms polling */
	mc_init(&mc);

#if S2_MOTOR_TEST
	/* Open-loop test: raw power, no speed control. Read the tick change after each phase. */
	{
		static const int pw[4][2] = { {400, 0}, {-400, 0}, {0, 400}, {0, -400} };
		static const char *const nm[4] = { "LEFT forward", "LEFT backward", "RIGHT forward", "RIGHT backward" };
		tk_dly_tsk(3000);			/* time to open the serial monitor */
		for (int i = 0; i < 4; i++) {
			int32_t a_l, a_r, b_l, b_r;
			enc_read(&a_l, &a_r);
			motor_set(pw[i][0], pw[i][1]);
			tk_dly_tsk(2000);
			motor_off();
			enc_read(&b_l, &b_r);
			S2_PRINTF("[S2] TEST %s: left ticks %d, right ticks %d (pins=%u irq=%u)\n", nm[i],
				  (int)(b_l - a_l), (int)(b_r - a_r), enc_pins(), enc_irq_count());
			tk_dly_tsk(1000);
		}
		S2_PRINTF("[S2] TEST done\n");
	}
#endif
	enc_read(&last_l, &last_r);

	tk_set_flg(flg_motion, FLG_MOTION_READY);
	period_start();

	for (;;) {
		uint32_t t0 = S2_US32();
		in = (mc_in_t){0};

		/* 1. encoders: edges since the last loop */
		enc_read(&enc_l, &enc_r);
		in.dticks_l = (int)(enc_l - last_l);
		in.dticks_r = (int)(enc_r - last_r);
		last_l = enc_l;
		last_r = enc_r;

		/* 2. STOP / RESET bits, read without clearing (STOP is level-sensitive) */
		UINT ptn = 0;
		if (tk_wai_flg(flg_motion, FLG_MOTION_STOP | FLG_MOTION_RESET, TWF_ORW, &ptn, TMO_POL) == E_OK) {
			in.stop_bit = (ptn & FLG_MOTION_STOP) != 0;
			in.reset    = (ptn & FLG_MOTION_RESET) != 0;
		}

		/* 3. one command per loop, never waiting for it */
		if (tk_rcv_mbf(mbf_motion_cmd, &cmd, TMO_POL) == (INT)sizeof cmd)
			in.cmd = &cmd;
#if S2_BENCH_SCRIPT
		if (in.cmd == NULL && step < N_STEPS && !in.stop_bit) {
			const step_t *s = &s_script[step];
			if (!step_sent) {
				cmd = s->cmd;
				in.cmd = &cmd;
				step_sent = true;
				step_cycles = 0;
			} else {
				step_cycles++;
				bool done = s->wait_cycles ? (step_cycles >= s->wait_cycles) : app_motion.cmd_done;
				if (done && step_cycles > 5) { step++; step_sent = false; }
			}
		}
#endif

		/* 4. Line Follow's steering, if it exists and is readable right now */
		line_status_t ls;
		if (mtx_line > 0 && tk_loc_mtx(mtx_line, TMO_POL) == E_OK) {
			ls = app_line;
			tk_unl_mtx(mtx_line);
			in.line_valid = ls.on_line;
			in.steer_mm_s = ls.steering_corr;
		}

		/* 5. control */
		mc_step(&mc, &in, &out);

		/* 6. motors: the only place they are written */
		if (out.pwm_l == 0 && out.pwm_r == 0)
			motor_off();
		else
			motor_set(out.pwm_l, out.pwm_r);

		/* 7. flags back to the Vehicle Controller */
		if (out.reset_consumed)
			tk_clr_flg(flg_motion, ~(UINT)FLG_MOTION_RESET);
		if (out.fault_new)
			tk_set_flg(flg_motion, FLG_MOTION_FAULT);
		tk_set_flg(flg_motion, FLG_MOTION_ALIVE);

		/* 8. publish motion_status_t (skip this loop if a reader holds the mutex; the next loop catches up) */
		if (tk_loc_mtx(mtx_motion, TMO_POL) == E_OK) {
			app_motion = out.status;
			tk_unl_mtx(mtx_motion);
		}

		/* 9. debug snapshot and 4 Hz console line */
		unsigned exec = (unsigned)(S2_US32() - t0);
		if (exec > exec_max_us) exec_max_us = exec;
		s2_motion_debug.state = out.state;
		s2_motion_debug.fault = out.fault;
		s2_motion_debug.enc_l = (int)enc_l;
		s2_motion_debug.enc_r = (int)enc_r;
		s2_motion_debug.pwm_l = out.pwm_l;
		s2_motion_debug.pwm_r = out.pwm_r;
		s2_motion_debug.exec_max_us = exec_max_us;
		s2_motion_debug.late_max_us = s_late_max_us;
		loops++;
#if S2_DEBUG_PRINT
		uint32_t now = s2_now_ms();
		if (now - last_print_ms >= 250u) {
			last_print_ms = now;
			S2_PRINTF("[S2] %s fault=%d vL=%d vR=%d tgt=%d/%d pwm=%d/%d dist=%dmm head=%ddeg tick=%d/%d pins=%u irq=%u exec<=%uus late<=%uus\n",
				  mc_state_name(out.state), (int)out.fault, (int)out.status.speed_l, (int)out.status.speed_r,
				  (int)out.tgt_l, (int)out.tgt_r, out.pwm_l, out.pwm_r,
				  (int)out.status.distance_mm, (int)out.status.heading_deg,
				  (int)enc_l, (int)enc_r, enc_pins(), enc_irq_count(), exec_max_us, s_late_max_us);
		}
#endif
		(void)loops;
		period_wait();
	}
}
