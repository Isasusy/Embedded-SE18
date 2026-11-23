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
					 "WATCHDOG", "MOTION", "OBSTACLE", "LINE_LOST", "TIMEOUT", "TILT", "LIFTED" };
	return (unsigned)f < sizeof n / sizeof n[0] ? n[f] : "UNKNOWN";
}

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
static void put_tenths(w_t *w, float x)
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
	if (s->waiting) {			/* INIT: what it is still waiting for, e.g. "waiting":["IMU","WIFI"] */
		static const struct { uint32_t bit; const char *nm; } wt[] = {
			{ VC_WAIT_MOTION, "MOTORS" }, { VC_WAIT_LINE, "LINE" }, { VC_WAIT_IMU, "IMU" }, { VC_WAIT_NET, "WIFI" } };
		int first = 1;
		put_s(&w, ",\"waiting\":[");
		for (unsigned i = 0; i < sizeof wt / sizeof wt[0]; i++) {
			if (!(s->waiting & wt[i].bit)) continue;
			if (!first) put_c(&w, ',');
			first = 0;
			put_c(&w, '"'); put_s(&w, wt[i].nm); put_c(&w, '"');
		}
		put_c(&w, ']');
	}
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

int tm_format_hump(char *buf, int cap, uint32_t n, float peak_mm, float max_hump_mm, uint32_t t_ms)
{
	w_t w = { buf, cap, 0 };
	put_s(&w, "{\"n\":"); put_u(&w, n);
	put_s(&w, ",\"peak_mm\":"); put_tenths(&w, peak_mm);
	put_s(&w, ",\"max_hump_mm\":"); put_tenths(&w, max_hump_mm);
	put_s(&w, ",\"t\":"); put_u(&w, t_ms);
	put_c(&w, '}');
	return finish(&w);
}

static int is_letter(char c) { return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z'); }

/* case-insensitive search for `word` as a whole word in the payload ("restart" does not contain the word "start") */
static int has_word(const char *p, int len, const char *word)
{
	for (int i = 0; i < len; i++) {
		if (i > 0 && is_letter(p[i - 1])) continue;
		int k = 0;
		while (word[k] && i + k < len) {
			char c = p[i + k];
			if (c >= 'A' && c <= 'Z') c = (char)(c + 32);
			if (c != word[k]) break;
			k++;
		}
		if (!word[k] && !(i + k < len && is_letter(p[i + k]))) return 1;
	}
	return 0;
}

int tm_parse_cmd(const char *p, int len)
{
	if (!p || len <= 0 || len > 128) return -1;
	int s = has_word(p, len, "start"), t = has_word(p, len, "stop"), r = has_word(p, len, "reset");
	int sp = has_word(p, len, "spin");
	if (s + t + r + sp != 1) return -1;
	return s ? REMOTE_START : (t ? REMOTE_STOP : (r ? REMOTE_RESET : REMOTE_SPIN));
}

static char up(char c);

/* "cal track=138.5" -> 138.5 (tenths at most). Returns 1 and sets *track_mm, or -1 if it is not a cal command. */
int tm_parse_cal(const char *p, int len, float *track_mm)
{
	static const char key[] = "CAL TRACK=";
	if (!p || len < 11 || len > 40) return -1;
	int i = 0;
	while (i < len && p[i] == ' ') i++;
	for (int k = 0; key[k]; k++, i++)
		if (i >= len || up(p[i]) != key[k]) return -1;
	long whole = 0, tenths = 0;
	int digits = 0, frac = 0;
	while (i < len && p[i] >= '0' && p[i] <= '9' && digits < 5) { whole = whole * 10 + (p[i] - '0'); i++; digits++; }
	if (i < len && p[i] == '.') {
		i++;
		if (i < len && p[i] >= '0' && p[i] <= '9') { tenths = p[i] - '0'; i++; frac = 1; }
		while (i < len && p[i] >= '0' && p[i] <= '9') i++;		/* further decimals ignored */
	}
	while (i < len && p[i] == ' ') i++;
	if (!digits || i != len) return -1;
	(void)frac;
	*track_mm = (float)whole + 0.1f * (float)tenths;
	return 1;
}

int tm_format_config(char *buf, int cap, float track_mm)
{
	w_t w = { buf, cap, 0 };
	put_s(&w, "{\"track_mm\":"); put_tenths(&w, track_mm);
	put_c(&w, '}');
	return finish(&w);
}

/* ---- barcode direction table ---- */
static const char *const s_nav_nm[] = { "LEFT", "RIGHT", "STRAIGHT", "UTURN", "STOP" };

static char up(char c) { return (c >= 'a' && c <= 'z') ? (char)(c - 'a' + 'A') : c; }
static int is_map_char(char c) { c = up(c); return (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9'); }

/* "LEFT" / "right" / "U-TURN" / "none" ... at p (n chars) -> nav, -1 = NONE, -2 = not a direction */
static int nav_word(const char *p, int n)
{
	static const struct { const char *w; int nav; } t[] = {
		{"LEFT", 0}, {"RIGHT", 1}, {"STRAIGHT", 2}, {"UTURN", 3}, {"U-TURN", 3}, {"STOP", 4}, {"NONE", -1}, {"-", -1} };
	for (unsigned k = 0; k < sizeof t / sizeof t[0]; k++) {
		int i = 0;
		while (i < n && t[k].w[i] && up(p[i]) == t[k].w[i]) i++;
		if (i == n && !t[k].w[i]) return t[k].nav;
	}
	return -2;
}

int tm_parse_map(const char *p, int len, char *chars, int *navs, int max)
{
	if (!p || len <= 3 || len > 400) return -1;
	int i = 0;
	while (i < len && p[i] == ' ') i++;
	if (len - i < 3 || up(p[i]) != 'M' || up(p[i + 1]) != 'A' || up(p[i + 2]) != 'P') return -1;
	i += 3;
	if (i < len && p[i] != ' ' && p[i] != ':') return -1;	/* "maple" is not a map command */
	int n = 0;
	while (i < len) {
		while (i < len && (p[i] == ' ' || p[i] == ',' || p[i] == ':' || p[i] == ';')) i++;
		if (i >= len) break;
		/* <char>=<DIRECTION> */
		if (i + 2 >= len || !is_map_char(p[i]) || p[i + 1] != '=') return -1;
		char c = up(p[i]);
		int s = i + 2, e = s;
		while (e < len && p[e] != ' ' && p[e] != ',' && p[e] != ';') e++;
		int nav = nav_word(p + s, e - s);
		if (nav == -2 || n >= max) return -1;
		chars[n] = c; navs[n] = nav; n++;
		i = e;
	}
	return n > 0 ? n : -1;
}

int tm_format_map(char *buf, int cap, const int8_t nav_by_index[TM_MAP_N])
{
	w_t w = { buf, cap, 0 };
	int first = 1;
	put_c(&w, '{');
	for (int i = 0; i < TM_MAP_N; i++) {
		int nav = nav_by_index[i];
		if (nav < 0 || nav > 4) continue;
		if (!first) put_c(&w, ',');
		first = 0;
		put_c(&w, '"'); put_c(&w, i < 26 ? (char)('A' + i) : (char)('0' + i - 26));
		put_s(&w, "\":\""); put_s(&w, s_nav_nm[nav]); put_c(&w, '"');
	}
	put_c(&w, '}');
	return finish(&w);
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

static int32_t round_i(float x) { return (int32_t)(x + (x < 0 ? -0.5f : 0.5f)); }

int tm_format_obstacle(char *buf, int cap, const obstacle_profile_t *p, uint32_t n)
{
	w_t w = { buf, cap, 0 };
	int rays = p->n < 0 ? 0 : (p->n > OBST_PROFILE_MAX_RAYS ? OBST_PROFILE_MAX_RAYS : p->n);
	put_s(&w, "{\"n\":"); put_u(&w, n);
	put_s(&w, ",\"deg\":[");
	for (int i = 0; i < rays; i++) { if (i) put_c(&w, ','); put_i(&w, p->servo_deg[i]); }
	put_s(&w, "],\"mm\":[");
	for (int i = 0; i < rays; i++) { if (i) put_c(&w, ','); put_i(&w, p->dist_mm[i] < 0 ? -1 : p->dist_mm[i]); }
	put_s(&w, "],\"found\":"); put_s(&w, p->ob.found ? "true" : "false");
	put_s(&w, ",\"dist_mm\":"); put_i(&w, round_i(p->ob.distance_mm));
	put_s(&w, ",\"width_mm\":"); put_i(&w, round_i(p->ob.width_mm));
	put_s(&w, ",\"left_clear_mm\":"); put_i(&w, round_i(p->ob.left_clear_mm));
	put_s(&w, ",\"right_clear_mm\":"); put_i(&w, round_i(p->ob.right_clear_mm));
	put_s(&w, ",\"side\":\""); put_s(&w, p->ob.side > 0 ? "LEFT" : (p->ob.side < 0 ? "RIGHT" : "NONE"));
	put_s(&w, "\",\"depth_mm\":"); put_i(&w, round_i(p->ob.depth_mm));
	put_s(&w, ",\"turn_deg\":"); put_i(&w, p->ob.turn_deg);
	put_s(&w, ",\"diag_mm\":"); put_i(&w, p->ob.diag_mm);
	put_s(&w, ",\"lat_mm\":"); put_i(&w, p->ob.lat_mm);
	put_s(&w, ",\"pass_mm\":"); put_i(&w, p->ob.pass_mm);
	put_s(&w, ",\"t\":"); put_u(&w, p->timestamp_ms);
	put_c(&w, '}');
	return finish(&w);
}
