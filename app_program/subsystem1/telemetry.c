#include "telemetry.h"

const char *tm_state_name(int s)
{
	static const char *const n[] = { "INIT", "IDLE", "LINE_FOLLOW", "BARCODE_ACTION", "HUMP_CROSSING",
					 "OBSTACLE_AVOID", "RECOVER", "COMPLETE", "FAULT" };
	return (unsigned)s < 9 ? n[s] : "UNKNOWN";
}

const char *tm_fault_name(int f)
{
	static const char *const n[] = { "NONE", "SELFTEST", "IMPACT", "IMU_SENSOR", "REMOTE_STOP",
					 "WATCHDOG", "MOTION", "OBSTACLE", "LINE_LOST", "TIMEOUT" };
	return (unsigned)f < 10 ? n[f] : "UNKNOWN";
}

/* ---- tiny bounded writer ---- */
typedef struct { char *p; int cap, len; } w_t;

static void put_c(w_t *w, char c) { if (w->len + 1 < w->cap) w->p[w->len] = c; w->len++; }
static void put_s(w_t *w, const char *s) { while (*s) put_c(w, *s++); }
static void put_u(w_t *w, uint32_t v)
{
	char t[11]; int n = 0;
	do { t[n++] = (char)('0' + v % 10u); v /= 10u; } while (v);
	while (n) put_c(w, t[--n]);
}
static void put_i(w_t *w, int32_t v) { if (v < 0) { put_c(w, '-'); put_u(w, (uint32_t)(-(int64_t)v)); } else put_u(w, (uint32_t)v); }
static void put_tenths(w_t *w, float x)		/* 12.34 -> 12.3 */
{
	int32_t t = (int32_t)(x * 10.0f + (x < 0 ? -0.5f : 0.5f));
	if (t < 0) { put_c(w, '-'); t = -t; }
	put_u(w, (uint32_t)(t / 10)); put_c(w, '.'); put_u(w, (uint32_t)(t % 10));
}
static int finish(w_t *w)
{
	if (w->len >= w->cap) return -1;
	w->p[w->len] = '\0';
	return w->len;
}

int tm_format_status(char *buf, int cap, const robot_status_t *s)
{
	w_t w = { buf, cap, 0 };
	put_s(&w, "{\"state\":\""); put_s(&w, tm_state_name(s->state));
	put_s(&w, "\",\"speed\":");  put_i(&w, s->speed_mm_s);
	put_s(&w, ",\"max_hump_mm\":"); put_tenths(&w, s->max_hump_mm);
	put_s(&w, ",\"fault\":\""); put_s(&w, tm_fault_name(s->fault_code));
	put_s(&w, "\",\"t\":"); put_u(&w, s->timestamp_ms);
	put_c(&w, '}');
	return finish(&w);
}

int tm_format_heartbeat(char *buf, int cap, uint32_t seq, uint32_t t_ms)
{
	w_t w = { buf, cap, 0 };
	put_s(&w, "{\"hb\":"); put_u(&w, seq);
	put_s(&w, ",\"t\":"); put_u(&w, t_ms);
	put_c(&w, '}');
	return finish(&w);
}

/* case-insensitive search for `word` in the payload */
static int has_word(const char *p, int len, const char *word)
{
	for (int i = 0; i < len; i++) {
		int k = 0;
		while (word[k] && i + k < len) {
			char c = p[i + k];
			if (c >= 'A' && c <= 'Z') c = (char)(c + 32);
			if (c != word[k]) break;
			k++;
		}
		if (!word[k]) return 1;
	}
	return 0;
}

int tm_parse_cmd(const char *p, int len)
{
	if (!p || len <= 0 || len > 128) return -1;
	int s = has_word(p, len, "start"), t = has_word(p, len, "stop"), r = has_word(p, len, "reset");
	if (s + t + r != 1) return -1;			/* none, or ambiguous: ignore */
	return s ? REMOTE_START : (t ? REMOTE_STOP : REMOTE_RESET);
}

int tm_format_barcode(char *buf, int cap, const char *code, int nav, const char *pattern, uint32_t n, uint32_t t_ms)
{
	static const char *const nm[] = { "LEFT", "RIGHT", "STRAIGHT", "UTURN", "STOP" };
	w_t w = { buf, cap, 0 };
	put_s(&w, "{\"code\":\""); put_s(&w, code);
	put_s(&w, "\",\"cmd\":\""); put_s(&w, (nav >= 0 && nav <= 4) ? nm[nav] : (code[0] ? "NONE" : "UNREADABLE"));
	put_s(&w, "\",\"n\":"); put_u(&w, n);
	put_s(&w, ",\"pattern\":\""); put_s(&w, pattern);
	put_s(&w, "\",\"t\":"); put_u(&w, t_ms);
	put_c(&w, '}');
	return finish(&w);
}
