/*
 *	barcode_task.c -- Barcode Decoder. Owns GP28, sampled by polling every 1 ms (no interrupt).
 *	Every read is printed (also when it fails, with the raw wide/narrow pattern), and a valid command is sent to
 *	the Vehicle Controller. APIs shown: tk_dly_tsk, tk_snd_mbf
 */
#include "subsystem3.h"
#include "drv_line.h"
#include "barcode_algo.h"
#include "line_config.h"
#include "s3_port.h"

static const char *nav_name(int nav)
{
	static const char *nm[] = { "LEFT", "RIGHT", "STRAIGHT", "U-TURN", "STOP" };
	return (nav >= 0 && nav <= 4) ? nm[nav] : "no command";
}

/* The last read, valid or not, for the telemetry (S1, which runs on the other core). s3_barcode_count is
   bumped AFTER the other fields are written, so a reader that sees the same count before and after copying
   has a consistent copy. */
volatile char s3_last_barcode[8];		/* "" when the read did not decode */
volatile char s3_last_pattern[BAR_MAX_ELEMENTS + 12];
volatile int s3_last_nav;			/* nav_cmd_type_t or -1 */
volatile unsigned s3_barcode_count;

/* for line_task: is the barcode sensor in the middle of a code (then a "junction" is really the bars)? */
volatile bool s3_bar_active;
volatile uint32_t s3_bar_last_edge_ms;

static void keep(const bd_read_t *rd, bool ok)
{
	for (int i = 0; i < 8; i++) s3_last_barcode[i] = ok ? rd->text[i] : 0;
	for (unsigned i = 0; i < sizeof rd->pattern; i++) s3_last_pattern[i] = rd->pattern[i];
	s3_last_nav = ok ? rd->nav : -1;
	__asm__ volatile ("dmb" ::: "memory");
	s3_barcode_count++;
}

void barcode_task(INT stacd, void *exinf)
{
	static bd_t b;					/* static: keeps the task stack small */
	static bd_read_t rd;
	bar_edge_t e;
	(void)stacd; (void)exinf;

	bd_init(&b);
	barcode_init();

	for (;;) {
		barcode_sample();
		while (barcode_edge_pop(&e)) {
#if S3_BARCODE_TRACE
			static uint32_t prev_us;
			static unsigned lines, sec;
			if (s3_now_ms() / 1000u != sec) { sec = s3_now_ms() / 1000u; lines = 0; }
			if (prev_us && ++lines <= 40)		/* at most 40 lines a second */
				S3_PRINTF("[S3] bar: %s %d ms (adc %d)\n", e.black ? "white" : "BLACK",
					  (int)((e.t_us - prev_us) / 1000u), e.peak);
			prev_us = e.t_us;
#endif
			bd_edge_p(&b, e.t_us, e.black, e.peak);
			s3_bar_last_edge_ms = s3_now_ms();
		}
		s3_bar_active = b.active;

		bd_result_t r = bd_poll(&b, S3_US32(), &rd);
		if (r == BD_VALID) {
			keep(&rd, true);
			S3_PRINTF("[S3] >> BARCODE \"%s\" -> %s%s  (pattern %s, narrow %d ms)\n", rd.text, nav_name(rd.nav),
				  rd.reversed ? " (read backwards)" : "", rd.pattern, (int)(rd.narrow_us / 1000u));
			if (rd.nav >= 0 && mbf_nav_cmd > 0) {
				nav_cmd_t m = { rd.nav, (int32_t)rd.text[0], s3_now_ms() };
				tk_snd_mbf(mbf_nav_cmd, &m, sizeof m, TMO_POL);
			}
		} else if (r == BD_INVALID) {
			keep(&rd, false);
			S3_PRINTF("[S3] barcode NOT decoded: %d elements, pattern %s (narrow %d ms)\n",
				  rd.n_elements, rd.pattern, (int)(rd.narrow_us / 1000u));
		}
		tk_dly_tsk(1);
	}
}
