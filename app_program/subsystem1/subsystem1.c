/*
 *	subsystem1.c
 *	Subsystem 1 -- creates the status message buffer and implements the two hooks that run inside the WiFi service task.
 *	APIs shown: tk_cre_mbf, tk_rcv_mbf (polling), tk_snd_mbf (polling)
 */
#include "subsystem1.h"

EXPORT ID	mbf_status;

/* Owned by the Vehicle Controller (S5). Weak default 0 so S1 links and runs without it. */
__attribute__((weak)) ID	mbf_remote_cmd;

IMPORT volatile char	s3_last_barcode[8];
IMPORT volatile char	s3_last_pattern[];
IMPORT volatile int	s3_last_nav;
IMPORT volatile unsigned s3_barcode_count;
IMPORT volatile unsigned s3_barcode_seq;	/* odd while barcode_task is writing the fields above */

/* The last obstacle profiling sweep, kept by the Obstacle Scan (S5, scan_task.c). s5_profile_seq is odd while it is
   being written and goes up by 2 per profile. */
IMPORT volatile obstacle_profile_t s5_profile;
IMPORT volatile unsigned s5_profile_seq;

EXPORT ER subsystem1_init(void)
{
	T_CMBF cmbf = {0};

	cmbf.mbfatr = TA_TFIFO;
	cmbf.maxmsz = S1_STATUS_MAXMSZ;
	cmbf.bufsz  = (INT)S1_STATUS_BUFSZ;
	mbf_status = tk_cre_mbf(&cmbf);
	return (mbf_status < E_OK) ? mbf_status : E_OK;
}

EXPORT ER subsystem1_start(void)
{
	return E_OK;
}

EXPORT void s1_service(s1_publish_fn publish, void *ctx, uint32_t now_ms)
{
	static uint32_t last_hb_ms, hb_seq;
	static bool hb_started;
	static unsigned bc_seen, ob_seen;
	static char buf[512];			/* static: the obstacle JSON is too big for the WiFi task's stack */

	/* A status lwIP refuses (all MQTT requests still waiting for a TCP ACK) is kept and sent on the next call. */
	static robot_status_t st_pend;
	static bool st_have;
	for (int i = 0; i < 2 && mbf_status > 0; i++) {	/* small bites: lwIP's MQTT output buffer is small */
		if (!st_have) {
			if (tk_rcv_mbf(mbf_status, &st_pend, TMO_POL) != (INT)sizeof st_pend)
				break;
			st_have = true;
		}
		int n = tm_format_status(buf, sizeof buf, &st_pend);
		if (n > 0 && publish(ctx, S1_TOPIC_STATUS, buf, n) != 0)
			break;
		st_have = false;
	}

	unsigned c1 = s3_barcode_count, b1 = s3_barcode_seq;
	if (c1 != bc_seen && !(b1 & 1u)) {
		char code[8], pat[80];
		__asm__ volatile ("dmb" ::: "memory");
		for (int i = 0; i < 8; i++) code[i] = s3_last_barcode[i];
		for (int i = 0; i < 76; i++) { pat[i] = s3_last_pattern[i]; if (!pat[i]) break; }
		code[7] = 0; pat[79] = 0;
		int nav = s3_last_nav;
		__asm__ volatile ("dmb" ::: "memory");
		if (s3_barcode_seq == b1 && s3_barcode_count == c1) {
			int n = tm_format_barcode(buf, sizeof buf, code, nav, pat, c1, now_ms);
			if (n > 0 && publish(ctx, S1_TOPIC_BARCODE, buf, n) == 0)
				bc_seen = c1;
		}
	}

	/* a new obstacle profile: publish it once (seqlock copy, same as the barcode above) */
	unsigned q1 = s5_profile_seq;
	if (q1 != ob_seen && !(q1 & 1u)) {
		obstacle_profile_t pr;
		__asm__ volatile ("dmb" ::: "memory");
		const volatile unsigned char *src = (const volatile unsigned char *)&s5_profile;
		unsigned char *dst = (unsigned char *)&pr;
		for (unsigned i = 0; i < sizeof pr; i++) dst[i] = src[i];
		__asm__ volatile ("dmb" ::: "memory");
		if (s5_profile_seq == q1) {			/* not rewritten while we copied */
			int n = tm_format_obstacle(buf, sizeof buf, &pr, q1 / 2u);
			if (n > 0 && publish(ctx, S1_TOPIC_OBSTACLE, buf, n) == 0)
				ob_seen = q1;			/* else try again on the next call */
		}
	}

	if (!hb_started) { hb_started = true; last_hb_ms = now_ms; }
	if ((uint32_t)(now_ms - last_hb_ms) >= 1000u) {
		last_hb_ms = now_ms;
		int n = tm_format_heartbeat(buf, sizeof buf, ++hb_seq, now_ms);
		if (n > 0)
			publish(ctx, S1_TOPIC_HEARTBEAT, buf, n);
	}
}

EXPORT void s1_rx(const char *topic, const char *payload, int len)
{
	(void)topic;			/* only S1_TOPIC_CMD is subscribed */
	int c = tm_parse_cmd(payload, len);
	if (c < 0 || mbf_remote_cmd <= 0)
		return;
	remote_cmd_t m = { c };
	tk_snd_mbf(mbf_remote_cmd, &m, sizeof m, TMO_POL);
}
