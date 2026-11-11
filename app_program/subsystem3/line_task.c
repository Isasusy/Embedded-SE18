/*
 *	line_task.c -- Line Follow: periodic 5 ms (NFR1). Owns the ADC and pins GP26 / GP27.
 *	APIs shown: tk_dly_tsk, tk_set_flg, tk_loc_mtx / tk_unl_mtx, tk_snd_mbf
 *	It never commands the motors: it publishes line_status_t for Motion Control and line_evt_t for the VC.
 */
#include "subsystem3.h"
#include "drv_line.h"
#include "line_algo.h"
#include "line_config.h"
#include "s3_port.h"

static uint32_t s_next_us;

static void period_wait(void)
{
	s_next_us += (uint32_t)LINE_PERIOD_MS * 1000u;
	int32_t delta = (int32_t)(s_next_us - S3_US32());
	if (delta <= 0) { s_next_us = S3_US32(); return; }	/* overran: do not catch up */
	uint32_t ms = (uint32_t)delta / 1000u;
	tk_dly_tsk(ms ? ms : 1);
}

IMPORT volatile bool	s3_bar_active;		/* barcode_task.c */
IMPORT volatile uint32_t s3_bar_last_edge_ms;

static void send_evt(la_event_t e)
{
	static const int map[] = { 0, LINE_EVT_JUNCTION, LINE_EVT_LOST, LINE_EVT_FOUND };
	static const char *const nm[] = { "", "JUNCTION", "LINE_LOST", "LINE_FOUND" };
	line_evt_t ev = { map[e], s3_now_ms() };

	if (e == LA_JUNCTION) {
		uint32_t since = s3_now_ms() - s3_bar_last_edge_ms;
		if (s3_bar_active || since < BAR_JUNCTION_BLANK_MS) {	/* both sensors on a barcode bar, not a junction */
			S3_PRINTF("[S3] both line sensors black during a barcode: not a junction\n");
			return;
		}
		S3_PRINTF("[S3] >> JUNCTION (both line sensors black)\n");
	} else if (e == LA_LINE_LOST) {		/* LINE_FOUND is too frequent with the outside-the-line sensors to print */
		S3_PRINTF("[S3] >> %s\n", nm[e]);
	}
	if (mbf_line_evt > 0)
		tk_snd_mbf(mbf_line_evt, &ev, sizeof ev, TMO_POL);	/* never blocks; a full buffer drops it */
}

void line_task(INT stacd, void *exinf)
{
	la_t a; la_out_t o;
	int l, r;
	unsigned n = 0; (void)n;
	(void)stacd; (void)exinf;

	la_init(&a);
	s_next_us = S3_US32();

	for (;;) {
		line_adc_read(&l, &r);
		la_step(&a, l, r, &o);

		static bool ready_sent;
		if (o.ready && !ready_sent) {
			ready_sent = true;
			tk_set_flg(flg_line, FLG_LINE_READY);
#if S3_DEBUG_PRINT
			S3_PRINTF("[S3] calibration %s: L %d..%d  R %d..%d\n", a.cal_ok ? "ok" : "REJECTED (defaults used)",
				  a.white_l, a.black_l, a.white_r, a.black_r);
#endif
		}

		if (o.ready) {
			line_status_t ls = { o.offset, o.steering_corr, o.on_line };
			if (tk_loc_mtx(mtx_line, TMO_POL) == E_OK) {	/* skip this sample if S2 is reading */
				app_line = ls;
				tk_unl_mtx(mtx_line);
			}
			if (o.event != LA_NO_EVENT)
				send_evt(o.event);
		}
		tk_set_flg(flg_line, FLG_LINE_ALIVE);

#if S3_DEBUG_PRINT
		if (++n % (500 / LINE_PERIOD_MS) == 0)
			S3_PRINTF("[S3] adc %d/%d barcode=%d (%s) offset=%d%% steer=%d on=%d\n", l, r,
				  barcode_raw(), barcode_is_black() ? "black" : "white",
				  (int)(o.offset * 100), (int)o.steering_corr, o.on_line);
#endif
		period_wait();
	}
}
