/*
 *	subsystem1.c
 *	Subsystem 1 -- creates the status message buffer and implements the two hooks that run inside the WiFi service task.
 *	APIs shown: tk_cre_mbf, tk_rcv_mbf (polling), tk_snd_mbf (polling)
 */
#include "subsystem1.h"
#include "../subsystem4/subsystem4.h"	/* s4_last_hump / s4_hump_seq: each measured hump, for robot/se18/hump */
#include "../subsystem3/barcode_algo.h"	/* bd_map_*: barcode letter -> direction table, set from the dashboard */
#include "../subsystem2/motion_ctrl.h"	/* mc_track_mm / mc_set_track: turn calibration, set from the dashboard */

/* Owned by S4. Weak defaults (0 = no hump yet) so S1 links and runs without it. */
__attribute__((weak)) volatile s4_hump_t	s4_last_hump;
__attribute__((weak)) volatile unsigned		s4_hump_seq;

EXPORT ID	mbf_status;

/* Goes up on every s1_service() call, which mqtt_utk.c makes only while MQTT is connected (at least every 100 ms).
   The Vehicle Controller watches it to know the dashboard link is up before it leaves INIT (see subsystem1.h). */
EXPORT volatile unsigned	s1_online_count;

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
	static unsigned map_sent_ver;
	static bool map_sent;

	s1_online_count++;

	/* the barcode direction table: once when MQTT first connects, then only when it changes */
	{
		unsigned v = bd_map_version();
		if (!map_sent || v != map_sent_ver) {
			int8_t tab[BD_MAP_N];
			static char mbuf[400];
			bd_map_copy(tab);
			int n = tm_format_map(mbuf, sizeof mbuf, tab);
			if (n > 0 && publish(ctx, S1_TOPIC_BARCODE_MAP, mbuf, n) == 0) {
				map_sent = true;
				map_sent_ver = v;
			}
		}
	}

	/* calibration in use (turn track): on every change, and every 5 s */
	{
		static float cfg_sent_track = -1.0f;
		static uint32_t cfg_sent_ms;
		float t = mc_track_mm;
		if (t != cfg_sent_track || (uint32_t)(now_ms - cfg_sent_ms) >= 5000u) {
			char cbuf[40];
			int n = tm_format_config(cbuf, sizeof cbuf, t);
			if (n > 0 && publish(ctx, S1_TOPIC_CONFIG, cbuf, n) == 0) {
				cfg_sent_track = t;
				cfg_sent_ms = now_ms;
			}
		}
	}
	static unsigned bc_seen, ob_seen, hump_seen;
	static char buf[768];			/* static: the obstacle JSON (up to ~521 bytes) is too big for the WiFi task's stack */

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

	/* a new hump from the IMU: publish it once (seqlock copy, same as above) */
	unsigned h1 = s4_hump_seq;
	if (h1 != hump_seen && !(h1 & 1u)) {
		__asm__ volatile ("dmb" ::: "memory");
		const float peak = s4_last_hump.peak_mm, max = s4_last_hump.max_hump_mm;
		const uint32_t t = s4_last_hump.t_ms;
		__asm__ volatile ("dmb" ::: "memory");
		if (s4_hump_seq == h1) {
			int n = tm_format_hump(buf, sizeof buf, h1 / 2u, peak, max, t);
			if (n > 0 && publish(ctx, S1_TOPIC_HUMP, buf, n) == 0)
				hump_seen = h1;			/* else try again on the next call */
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

	/* "cal track=138": turn calibration (Motion Control's effective wheel spacing) */
	float tr;
	if (tm_parse_cal(payload, len, &tr) > 0) {
		bool ok = mc_set_track(tr);
		tm_printf((UB *)"[S1] cal track=%d.%d mm %s\n", (int)tr, (int)(tr * 10.0f) % 10,
			  ok ? "set" : "REFUSED (out of range)");
		return;
	}

	/* "map A=LEFT B=RIGHT ...": change the barcode direction table (checked first: it may contain the word STOP) */
	char ch[BD_MAP_N];
	int nv[BD_MAP_N];
	int k = tm_parse_map(payload, len, ch, nv, BD_MAP_N);
	if (k > 0) {
		for (int i = 0; i < k; i++) (void)bd_map_set(ch[i], nv[i]);
		tm_printf((UB *)"[S1] barcode map changed (%d letters)\n", k);
		return;
	}

	int c = tm_parse_cmd(payload, len);
	if (c < 0 || mbf_remote_cmd <= 0)
		return;
	remote_cmd_t m = { c };
	tk_snd_mbf(mbf_remote_cmd, &m, sizeof m, TMO_POL);
}
