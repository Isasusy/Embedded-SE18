/*
 *	subsystem1.c
 *	Subsystem 1 -- creates the status message buffer and implements the two hooks that run inside the WiFi service task.
 *	APIs shown: tk_cre_mbf, tk_rcv_mbf (polling), tk_snd_mbf (polling)
 */
#include "subsystem1.h"

EXPORT ID	mbf_status;		/* IF-14 */

/* Owned by the Vehicle Controller (S5). Weak default 0 so S1 links and runs without it. */
__attribute__((weak)) ID	mbf_remote_cmd;

/* The last barcode read, kept by the Barcode Decoder (S3, barcode_task.c). */
IMPORT volatile char	s3_last_barcode[8];
IMPORT volatile char	s3_last_pattern[];
IMPORT volatile int	s3_last_nav;
IMPORT volatile unsigned s3_barcode_count;

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
	return E_OK;			/* no task: the work is done by hooks inside the kernel's WiFi service task */
}

/* ---- runs inside the WiFi service task: never block, never wait for the network ---- */
EXPORT void s1_service(s1_publish_fn publish, void *ctx, uint32_t now_ms)
{
	static uint32_t last_hb_ms, hb_seq;
	static bool hb_started;
	static unsigned bc_seen;
	char buf[200];
	robot_status_t st;

	/* status at 2 Hz: the Vehicle Controller decides the rate, we publish whatever is queued (at most 2 per call) */
	for (int i = 0; i < 2 && mbf_status > 0; i++) {	/* small bites: lwIP's MQTT output buffer is small */
		if (tk_rcv_mbf(mbf_status, &st, TMO_POL) != (INT)sizeof st)
			break;
		int n = tm_format_status(buf, sizeof buf, &st);
		if (n > 0)
			publish(ctx, S1_TOPIC_STATUS, buf, n);
	}

	/* a new barcode read (valid or not): publish it once */
	unsigned c1 = s3_barcode_count;
	if (c1 != bc_seen) {
		char code[8], pat[80];
		__asm__ volatile ("dmb" ::: "memory");
		for (int i = 0; i < 8; i++) code[i] = s3_last_barcode[i];
		for (int i = 0; i < 76; i++) { pat[i] = s3_last_pattern[i]; if (!pat[i]) break; }
		code[7] = 0; pat[79] = 0;
		int nav = s3_last_nav;
		__asm__ volatile ("dmb" ::: "memory");
		if (s3_barcode_count == c1) {			/* not overwritten while we copied */
			int n = tm_format_barcode(buf, sizeof buf, code, nav, pat, c1, now_ms);
			if (n > 0 && publish(ctx, S1_TOPIC_BARCODE, buf, n) == 0)
				bc_seen = c1;			/* else try again on the next call */
		}
	}

	/* heartbeat every 1 s (NFR5), independent of the robot's state */
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
	tk_snd_mbf(mbf_remote_cmd, &m, sizeof m, TMO_POL);	/* a full buffer drops it: the sender can resend */
}
