#include <string.h>
#include "barcode_algo.h"
#include "../subsystem5/vc_msgs.h"

/* Code 39: 9 elements (bar space bar space bar space bar space bar), bit 8 = first element, 1 = wide */
static const struct { char c; uint16_t p; } c39[] = {
	{'0',0x034},{'1',0x121},{'2',0x061},{'3',0x160},{'4',0x031},{'5',0x130},{'6',0x070},{'7',0x025},
	{'8',0x124},{'9',0x064},{'A',0x109},{'B',0x049},{'C',0x148},{'D',0x019},{'E',0x118},{'F',0x058},
	{'G',0x00D},{'H',0x10C},{'I',0x04C},{'J',0x01C},{'K',0x103},{'L',0x043},{'M',0x142},{'N',0x013},
	{'O',0x112},{'P',0x052},{'Q',0x007},{'R',0x106},{'S',0x046},{'T',0x016},{'U',0x181},{'V',0x0C1},
	{'W',0x1C0},{'X',0x091},{'Y',0x190},{'Z',0x0D0},{'-',0x085},{'.',0x184},{' ',0x0C4},{'*',0x094},
	{'$',0x0A8},{'/',0x0A2},{'+',0x08A},{'%',0x02A},
};

void bd_init(bd_t *b) { memset(b, 0, sizeof *b); }

/* ---- barcode character -> direction table (see barcode_algo.h). Stored as nav + 1, so 0 = no command. ---- */
static volatile int8_t   s_map[BD_MAP_N];
static volatile unsigned s_map_ver;
static volatile bool     s_map_ready;

int bd_map_index(char c)
{
	if (c >= 'a' && c <= 'z') c = (char)(c - 'a' + 'A');
	if (c >= 'A' && c <= 'Z') return c - 'A';
	if (c >= '0' && c <= '9') return 26 + (c - '0');
	return -1;
}

char bd_map_char(int i) { return (i >= 0 && i < 26) ? (char)('A' + i) : ((i >= 26 && i < BD_MAP_N) ? (char)('0' + i - 26) : 0); }

void bd_map_defaults(void)
{
	for (int i = 0; i < BD_MAP_N; i++) s_map[i] = 0;
	s_map[bd_map_index(BAR_CHAR_LEFT)]     = NAV_LEFT + 1;
	s_map[bd_map_index(BAR_CHAR_RIGHT)]    = NAV_RIGHT + 1;
	s_map[bd_map_index(BAR_CHAR_STRAIGHT)] = NAV_STRAIGHT + 1;
	s_map[bd_map_index(BAR_CHAR_UTURN)]    = NAV_UTURN + 1;
	s_map_ready = true;
	s_map_ver++;
}

static void map_ensure(void) { if (!s_map_ready) bd_map_defaults(); }

int bd_map_get(char c)
{
	map_ensure();
	int i = bd_map_index(c);
	return i < 0 ? -1 : s_map[i] - 1;
}

bool bd_map_set(char c, int nav)
{
	map_ensure();
	int i = bd_map_index(c);
	if (i < 0 || nav < -1 || nav > NAV_STOP) return false;
	s_map[i] = (int8_t)(nav + 1);		/* one byte: the barcode task never sees half an update */
	s_map_ver++;
	return true;
}

void bd_map_copy(int8_t nav_out[BD_MAP_N])
{
	map_ensure();
	for (int i = 0; i < BD_MAP_N; i++) nav_out[i] = (int8_t)(s_map[i] - 1);
}

unsigned bd_map_version(void) { map_ensure(); return s_map_ver; }

static int nav_of(char c) { return bd_map_get(c); }

/* ---- per-character fit ----
   No global thin / thick threshold and no fixed wide:narrow ratio. Each character (9 elements) is compared with every
   Code 39 pattern, and the one that explains its widths best wins. For a candidate pattern the widths are modelled as
       bar   = u * k - b        space = u * k + b        (k = 1 narrow, R wide)
   u = this character's narrow width (so the car's speed may differ from character to character), b = blur (the IR
   spot makes bars look shorter and spaces longer by the same amount), R = the print's wide:narrow ratio, one value for
   the whole read, chosen from BAR_RATIO_MIN..BAR_RATIO_MAX as the one that fits all characters best. u, b and a speed
   gradient inside the character come from a least-squares fit; the cost is the remaining error relative to u. When
   the bars' darkness differs enough (BAR_PEAK_SPLIT), a candidate that calls a bar narrow although it is darker than a
   bar it calls wide pays extra. A character is accepted only if its best fit is good (BAR_FIT_MAX) and clearly better
   than the next best character (BAR_FIT_MARGIN).
   The first and last stripe of the read are left out of the fit (a slow rise into the first bar, the junction line
   after the last); both belong to a '*', whose pattern the other 8 elements still pin down. */
#define FIT_RATIOS	3				/* ratios tried, evenly from BAR_RATIO_MIN to BAR_RATIO_MAX */
#define FIT_DARK_W	0.3f				/* darkness penalty weight (cost units per normalised step) */
#define FIT_MAX_CHARS	((BAR_MAX_ELEMENTS + 1) / 10)
#define N_C39		(int)(sizeof c39 / sizeof c39[0])

typedef struct { char ch; uint16_t bits; float cost, second; } fit_t;	/* second = next best pattern's cost */

/* One character's 9 elements, prepared once and then compared with all 44 patterns. A pattern only decides which 3
   elements are wide (k = R) and which narrow (k = 1), so every least-squares sum is "all elements" plus (R - 1) or
   (R^2 - 1) times "the 3 wide ones": the per-element products are kept, the pattern just picks 3 of them. */
typedef struct {
	float x[9], wt[9], tau[9], s[9], dk[9];		/* width (scaled to mean 1), weight, position -0.5..0.5, -1 bar / +1 space */
	bool dark;					/* dk[] holds bar darkness 0..1 */
	float pw[9], pws[9], pwt[9], pwst[9], pwt2[9], pwx[9], pwtx[9];
	float W, Ws, Wt, Wst, Wt2, Wx, Wsx, Wtx;
} chr_t;

static void prep_char(const uint32_t *w, const float *dark, int n, int c, chr_t *q)
{
	int b0 = c * 10;
	float tot = 0, at = 0, sw = 0, swx = 0;
	for (int i = 0; i < 9; i++) {
		q->x[i] = (float)w[b0 + i];
		q->wt[i] = (b0 + i == 0 || b0 + i == n - 1) ? 0.0f : 1.0f;
		q->s[i] = (i % 2 == 0) ? -1.0f : 1.0f;
		q->dk[i] = dark ? dark[b0 + i] : 0.0f;
		tot += q->x[i];
		sw += q->wt[i]; swx += q->wt[i] * q->x[i];
	}
	q->dark = dark != NULL;
	float scale = swx > 0.0f ? sw / swx : 1.0f;	/* widths to mean 1: keeps the float sums well conditioned */
	q->W = q->Ws = q->Wt = q->Wst = q->Wt2 = q->Wx = q->Wsx = q->Wtx = 0;
	for (int i = 0; i < 9; i++) {
		q->tau[i] = (at + 0.5f * q->x[i]) / tot - 0.5f;
		at += q->x[i];
		q->x[i] *= scale;
		float wt = q->wt[i], s = q->s[i], t = q->tau[i], x = q->x[i];
		q->pw[i] = wt; q->pws[i] = wt * s; q->pwt[i] = wt * t; q->pwst[i] = wt * s * t;
		q->pwt2[i] = wt * t * t; q->pwx[i] = wt * x; q->pwtx[i] = wt * t * x;
		q->W += wt; q->Ws += q->pws[i]; q->Wt += q->pwt[i]; q->Wst += q->pwst[i]; q->Wt2 += q->pwt2[i];
		q->Wx += q->pwx[i]; q->Wsx += wt * s * x; q->Wtx += q->pwtx[i];
	}
}

static float det3(float a, float b, float c, float d, float e, float f, float g, float h, float i)
{
	return a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
}

/* cost of pattern p for character q at ratio R. Model: element = (u + g * tau) * k + b * s, i.e. narrow width u, a
   speed gradient g across the character, blur b (bars shorter, spaces longer). Cost = remaining error relative to u. */
static float fit_cost(const chr_t *q, uint16_t p, float R)
{
	float a1 = R - 1.0f, a2 = R * R - 1.0f;
	float Sw = 0, Sws = 0, Swt = 0, Swst = 0, Swt2 = 0, Swx = 0, Swtx = 0;
	for (int i = 0; i < 9; i++) {
		if (!(p & (1u << (8 - i)))) continue;
		Sw += q->pw[i]; Sws += q->pws[i]; Swt += q->pwt[i]; Swst += q->pwst[i];
		Swt2 += q->pwt2[i]; Swx += q->pwx[i]; Swtx += q->pwtx[i];
	}
	float A00 = q->W + a2 * Sw, A01 = q->Ws + a1 * Sws, A02 = q->Wt + a2 * Swt;
	float A11 = q->W, A12 = q->Wst + a1 * Swst, A22 = q->Wt2 + a2 * Swt2;
	float r0 = q->Wx + a1 * Swx, r1 = q->Wsx, r2 = q->Wtx + a1 * Swtx;
	float u, b, g;
	float det = det3(A00, A01, A02, A01, A11, A12, A02, A12, A22);
	if (det > -1e-9f && det < 1e-9f) return 1e9f;
	u = det3(r0, A01, A02, r1, A11, A12, r2, A12, A22) / det;
	b = det3(A00, r0, A02, A01, r1, A12, A02, r2, A22) / det;
	g = det3(A00, A01, r0, A01, A11, r1, A02, A12, r2) / det;
	if (b < 0.0f || b > 0.8f * u) {		/* blur only shortens bars, and never to nothing: pin it, refit u and g */
		float bf = b < 0.0f ? 0.0f : 0.8f;
		float B00 = A00 + 2.0f * bf * A01 + bf * bf * A11, B01 = A02 + bf * A12, B11 = A22;
		float s0 = r0 + bf * r1, s1 = r2;
		float d2 = B00 * B11 - B01 * B01;
		if (d2 > -1e-9f && d2 < 1e-9f) return 1e9f;
		u = (s0 * B11 - B01 * s1) / d2;
		g = (B00 * s1 - B01 * s0) / d2;
		b = bf * u;
	}
	if (u <= 0.0f) return 1e9f;
	if (g > 0.8f * u) g = 0.8f * u;			/* at most +-40% speed change across one character */
	if (g < -0.8f * u) g = -0.8f * u;
	float e = 0;
	for (int i = 0; i < 9; i++) {
		float k = (p & (1u << (8 - i))) ? R : 1.0f;
		float r = q->x[i] - ((u + g * q->tau[i]) * k + b * q->s[i]);
		e += q->wt[i] * r * r;
	}
	float cost = e / (u * u * q->W);
	if (q->dark)					/* a narrow bar darker than a wide one is unlikely */
		for (int i = 0; i < 9; i += 2)
			for (int j = 0; j < 9; j += 2)
				if (!(p & (1u << (8 - i))) && (p & (1u << (8 - j))) && q->dk[i] > q->dk[j])
					cost += FIT_DARK_W * (q->dk[i] - q->dk[j]);
	return cost;
}

static fit_t fit_char(const chr_t *q, float R)
{
	fit_t best = { 0, 0, 1e9f, 1e9f };
	for (int t = 0; t < N_C39; t++) {
		float cst = fit_cost(q, c39[t].p, R);
		if (cst < best.cost) { best.second = best.cost; best.ch = c39[t].c; best.bits = c39[t].p; best.cost = cst; }
		else if (cst < best.second) best.second = cst;
	}
	return best;
}

/* the best pattern of every character of a whole-character read (n = 10k - 1), at the ratio that fits it best */
static bool fit_all(const uint32_t *w, const int16_t *pk, int n, fit_t *fit, float *ratio_out)
{
	if (n < 29 || (n + 1) % 10 != 0) return false;

	/* bar darkness 0..1 across this read, used only when bars really differ in darkness */
	static float dark[BAR_MAX_ELEMENTS];
	bool use_dark = false;
	if (pk) {
		int lo = 32767, hi = -1;
		for (int i = 2; i < n - 1; i += 2) {
			if (pk[i] < 0) { lo = 32767; break; }
			if (pk[i] < lo) lo = pk[i];
			if (pk[i] > hi) hi = pk[i];
		}
		if (lo != 32767 && hi - lo >= BAR_PEAK_SPLIT) {
			use_dark = true;
			for (int i = 0; i < n; i++) {
				float d = (i % 2 == 0) ? (float)(pk[i] - lo) / (float)(hi - lo) : 0.0f;
				dark[i] = d < 0.0f ? 0.0f : (d > 1.0f ? 1.0f : d);
			}
			dark[0] = dark[n - 1] = 0.0f;		/* the unreliable end bars: no darkness vote */
		}
	}

	static chr_t q[FIT_MAX_CHARS];
	static fit_t cur[FIT_MAX_CHARS];
	int k = (n + 1) / 10;
	for (int c = 0; c < k; c++) prep_char(w, use_dark ? dark : NULL, n, c, &q[c]);
	float best_total = 1e30f;
	for (int r = 0; r < FIT_RATIOS; r++) {
		float R = BAR_RATIO_MIN + (BAR_RATIO_MAX - BAR_RATIO_MIN) * (float)r / (float)(FIT_RATIOS - 1);
		float tot = 0;
		for (int c = 0; c < k; c++) { cur[c] = fit_char(&q[c], R); tot += cur[c].cost; }
		if (tot < best_total) {
			best_total = tot;
			*ratio_out = R;
			memcpy(fit, cur, (size_t)k * sizeof *fit);
		}
	}
	return true;
}

/* does the fitted read decode? `tolerant`: accept it when ONE of the two '*' is unreadable, as long as the other '*'
   and every character between them are good (*damaged is then set) */
static bool fit_accept(const fit_t *fit, int k, bool tolerant, char *text, bool *damaged)
{
	int t = 0, bad = 0;
	*damaged = false;
	text[0] = 0;
	for (int c = 0; c < k; c++) {
		const fit_t *f = &fit[c];
		bool guard = (c == 0 || c == k - 1);
		bool good = f->cost <= BAR_FIT_MAX && f->second - f->cost >= BAR_FIT_MARGIN &&
			    (guard ? f->ch == '*' : f->ch != '*');
		if (!good) {
			if (!guard || !tolerant || ++bad > 1) { text[0] = 0; return false; }
			continue;
		}
		if (!guard && t < 7) text[t++] = f->ch;
	}
	text[t] = 0;
	*damaged = bad > 0;
	return t > 0;
}

/* Dark fragments: very dark bars flicker by more than BAR_SWING, so one thick bar can come in as many stripes whose
   "spaces" never get anywhere near as light as real spaces.
   Any space whose lightest reading stays in the top BAR_DIP_LEVEL_PCT % between white (the lightest space) and dark
   (the upper quartile of the bar peaks) is part of the bar around it: merge it, whatever the stripe count. */
/* this read's own levels: white = lightest space, dark = upper quartile of the bar peaks. false if unknown or the
   contrast is too low to tell a dip from a space */
static bool read_levels(const int16_t *pk, int n, int *white_out, int *dark_out)
{
	static int16_t s[BAR_MAX_ELEMENTS];
	int nb = 0, white = 32767;

	for (int i = 0; i < n; i++) {
		if (pk[i] < 0) return false;
		if (i % 2 == 0) s[nb++] = pk[i];
		else if (pk[i] < white) white = pk[i];
	}
	if (nb < 2 || white == 32767) return false;
	for (int i = 1; i < nb; i++)
		for (int j = i; j > 0 && s[j] < s[j - 1]; j--) { int16_t x = s[j]; s[j] = s[j - 1]; s[j - 1] = x; }
	int dark = s[(3 * nb) / 4];
	if (dark - white < BAR_PEAK_SPLIT) return false;
	*white_out = white;
	*dark_out = dark;
	return true;
}

static void merge_dark_dips(uint32_t *w, int16_t *pk, int *pn)
{
	int n = *pn, white, dark;
	if (!read_levels(pk, n, &white, &dark)) return;
	int lim = white + (BAR_DIP_LEVEL_PCT * (dark - white)) / 100;

	int o = 0;
	for (int i = 0; i < n; i++) {
		if (i % 2 == 1 && i + 1 < n && pk[i] >= lim) {
			w[o - 1] += w[i] + w[i + 1];
			if (pk[i + 1] > pk[o - 1]) pk[o - 1] = pk[i + 1];
			i++;
			continue;
		}
		w[o] = w[i]; pk[o] = pk[i]; o++;
	}
	*pn = o;
}

/* contrast of interior stripe i with its two neighbours: how much darker a bar is than the darker space beside it,
   or how much lighter a space is than the lighter bar beside it */
static int local_contrast(const int16_t *pk, int i)
{
	if (i % 2 == 0) return pk[i] - (pk[i - 1] > pk[i + 1] ? pk[i - 1] : pk[i + 1]);
	return (pk[i - 1] < pk[i + 1] ? pk[i - 1] : pk[i + 1]) - pk[i];
}

/* Noise on a small barcode: its stripes never reach full white or black, so sensor noise can split one stripe into
   three (a shallow dip inside a bar, or a shallow bump inside a space). Such a split has much less contrast with its
   neighbours than any real stripe. While the count is not whole characters (and odd: merging removes 2), merge the
   flattest stripe, if it is below BAR_DIP_LOCAL_PCT % of the read's median stripe contrast. Works on levels relative
   to this read, so any barcode size. */
static void merge_flat_stripes(uint32_t *w, int16_t *pk, int *pn)
{
#if BAR_DIP_LOCAL_PCT > 0
	static int c[BAR_MAX_ELEMENTS];
	int n = *pn;
	if (n < 5) return;
	for (int i = 0; i < n; i++) if (pk[i] < 0) return;
	/* median contrast of the read as it came in (noise stripes are few, so they barely move it) */
	int m = 0;
	for (int i = 1; i < n - 1; i++) c[m++] = local_contrast(pk, i);
	for (int i = 1; i < m; i++)
		for (int j = i; j > 0 && c[j] < c[j - 1]; j--) { int x = c[j]; c[j] = c[j - 1]; c[j - 1] = x; }
	int lim = (BAR_DIP_LOCAL_PCT * c[m / 2]) / 100;
	if (lim <= 0) return;
	while (n >= 31 && n % 2 == 1 && (n + 1) % 10 != 0) {
		int best = -1, bc = 0;
		for (int i = 1; i < n - 1; i++) {
			int lc = local_contrast(pk, i);
			if (lc < lim && (best < 0 || lc < bc)) { best = i; bc = lc; }
		}
		if (best < 0) break;
		int bar = (best % 2 == 0);		/* merged stripe takes the level of its neighbours */
		w[best - 1] += w[best] + w[best + 1];
		if (bar ? pk[best + 1] < pk[best - 1] : pk[best + 1] > pk[best - 1]) pk[best - 1] = pk[best + 1];
		for (int i = best; i + 2 < n; i++) { w[i] = w[i + 2]; pk[i] = pk[i + 2]; }
		n -= 2;
	}
	*pn = n;
#else
	(void)w; (void)pk; (void)pn;
#endif
}

/* The stripes one decode attempt works on: as read, or (merge) with dark dips joined back into their bars. */
static int prepare(const uint32_t *w_in, const int16_t *pk_in, int n, bool merge, uint32_t *w, int16_t *pkm)
{
	for (int i = 0; i < n; i++) { w[i] = w_in[i]; pkm[i] = pk_in ? pk_in[i] : -1; }
	if (!pk_in || !merge) return n;
	merge_dark_dips(w, pkm, &n);
	/* then, if the count still does not fit, the darkest of the remaining dips, one at a time: any space that stays
	   BAR_DIP_MIN_PCT % of the way from this read's white toward its dark. Only for an ODD count (a code starts and
	   ends with a bar): merging removes 2 stripes, so an even count can never reach 10k - 1 and would only lose
	   information */
	int white, dark;
	if (!read_levels(pkm, n, &white, &dark)) { merge_flat_stripes(w, pkm, &n); return n; }
	int dip_min = white + (BAR_DIP_MIN_PCT * (dark - white)) / 100;
	while (n >= 31 && n % 2 == 1 && (n + 1) % 10 != 0) {
		int dip = -1;
		for (int i = 1; i < n - 1; i += 2)
			if (pkm[i] >= dip_min && (dip < 0 || pkm[i] > pkm[dip])) dip = i;
		if (dip < 0) break;
		w[dip - 1] += w[dip] + w[dip + 1];
		if (pkm[dip + 1] > pkm[dip - 1]) pkm[dip - 1] = pkm[dip + 1];
		for (int i = dip; i + 2 < n; i++) { w[i] = w[i + 2]; pkm[i] = pkm[i + 2]; }
		n -= 2;
	}
	merge_flat_stripes(w, pkm, &n);
	return n;
}

static void fill_result(bd_read_t *out, const char *txt, bool dmg, int dir, float ratio)
{
	static char method[2][40];			/* one per kept result: strict and tolerant */
	char *m = method[dmg ? 1 : 0];
	static const char head[] = "per-character fit, ratio ";	/* + "2.50" (no printf here: no stdio in the firmware) */
	int h = (int)(ratio * 100.0f + 0.5f);
	memcpy(m, head, sizeof head - 1);
	char *e = m + sizeof head - 1;
	*e++ = (char)('0' + h / 100); *e++ = '.'; *e++ = (char)('0' + h / 10 % 10); *e++ = (char)('0' + h % 10); *e = 0;
	memcpy(out->text, txt, sizeof out->text);
	out->method = m;
	out->reversed = dir;
	out->guard_damaged = dmg;
	out->nav = nav_of(out->text[0]);
}

/* One variant of the stripes (as read, or merged): fit them read forward, and backward if forward does not decode.
   Returns BD_VALID with *out filled when a strict read (both '*' good) decodes. Otherwise, with want_tol, *tol gets
   the tolerant read (one '*' unreadable) and *tol_ok says whether there is one. out->n_elements is the raw count; the
   pattern is of the stripes actually tried. */
static bd_result_t decode_pass(const uint32_t *w_in, const int16_t *pk_in, int n_raw, bool merge, bool want_tol,
			       bd_read_t *out, bd_read_t *tol, bool *tol_ok)
{
	static uint32_t w[BAR_MAX_ELEMENTS], r[BAR_MAX_ELEMENTS];	/* static: big, and only the barcode task decodes */
	static int16_t pkm[BAR_MAX_ELEMENTS], rpk[BAR_MAX_ELEMENTS];
	static fit_t fit[2][FIT_MAX_CHARS];
	float ratio[2] = { 0, 0 };
	bool have[2] = { false, false };
	char txt[8];
	bool dmg = false;
	int sdir = -1, tdir = -1;

	*tol_ok = false;
	memset(out, 0, sizeof *out);
	out->nav = -1;
	out->n_elements = n_raw;
	int n = prepare(w_in, pk_in, n_raw, merge, w, pkm);
	const int16_t *pk = pk_in ? pkm : NULL;
	int k = (n + 1) / 10;

	uint32_t lo = 0xFFFFFFFFu, hi = 0;
	for (int i = 0; i < n; i++) { if (w[i] < lo) lo = w[i]; if (w[i] > hi) hi = w[i]; }
	out->narrow_us = n ? lo : 0;

	have[0] = fit_all(w, pk, n, fit[0], &ratio[0]);
	if (have[0] && fit_accept(fit[0], k, false, txt, &dmg)) {
		sdir = 0;
	} else if (have[0]) {				/* read backwards (n is odd: bars stay bars) */
		for (int i = 0; i < n; i++) { r[i] = w[n - 1 - i]; rpk[i] = pkm[n - 1 - i]; }
		have[1] = fit_all(r, pk ? rpk : NULL, n, fit[1], &ratio[1]);
		if (have[1] && fit_accept(fit[1], k, false, txt, &dmg)) sdir = 1;
	}
	char ttxt[8];
	bool tdmg = false;
	if (sdir < 0 && want_tol)
		for (int d = 0; d < 2 && tdir < 0; d++)
			if (have[d] && fit_accept(fit[d], k, true, ttxt, &tdmg)) tdir = d;

	/* the pattern, in reading order: the fitted one when the count is whole characters, else a rough midpoint */
	int dir = sdir >= 0 ? sdir : (tdir >= 0 ? tdir : 0);
	int p = 0;
	for (int i = 0; i < n && p < (int)sizeof out->pattern - 2; i++) {
		if (i > 0 && (i % 10 == 9 || i % 10 == 0) && have[dir]) out->pattern[p++] = ' ';
		bool wd;
		if (have[dir]) {
			int j = dir ? n - 1 - i : i;		/* element j of the direction that was fitted */
			wd = (j % 10 != 9) && (fit[dir][j / 10].bits & (1u << (8 - j % 10)));
		} else {
			wd = w[i] > (lo + hi) / 2;
		}
		out->pattern[p++] = wd ? 'W' : 'n';
	}
	out->pattern[p] = 0;

	if (sdir >= 0) {
		fill_result(out, txt, dmg, sdir, ratio[sdir]);
		return BD_VALID;
	}
	out->text[0] = 0;
	if (tdir >= 0) {
		*tol = *out;
		fill_result(tol, ttxt, tdmg, tdir, ratio[tdir]);
		*tol_ok = true;
	}
	return BD_INVALID;
}

/* allow_damaged: accept one unreadable '*' (see fit_accept). Off for the EARLY finish in bd_poll: there the code
   may still be going on, and a letter in the place of the closing '*' would look like a damaged '*'.
   When the raw count is already whole characters (10k - 1), the stripes are first tried exactly as read: on a small
   or blurred barcode a real narrow space between two thick bars never gets very light, and merge_dark_dips would
   take it for a dip inside one bar and lose two stripes. The merged stripes are still tried next (a real flicker can
   also leave a whole-character count). Strict reads of both are tried before a tolerant one is accepted. */
static bd_result_t decode(const uint32_t *w_in, const int16_t *pk_in, int n, bd_read_t *out, bool allow_damaged)
{
	static bd_read_t tolr[2], raw_out;		/* static: only the barcode task decodes */
	bool tok[2] = { false, false };
	bool raw_fits = pk_in && n >= 29 && (n + 1) % 10 == 0;

	if (raw_fits) {
		if (decode_pass(w_in, pk_in, n, false, allow_damaged, out, &tolr[0], &tok[0]) == BD_VALID) return BD_VALID;
		raw_out = *out;
	}
	/* the merged stripes, unless merging changes nothing (merging always removes stripes): then it is the same read */
	static uint32_t mw[BAR_MAX_ELEMENTS];
	static int16_t mpk[BAR_MAX_ELEMENTS];
	bool merged_same = raw_fits && prepare(w_in, pk_in, n, true, mw, mpk) == n;
	if (!merged_same &&
	    decode_pass(w_in, pk_in, n, true, allow_damaged, out, &tolr[1], &tok[1]) == BD_VALID) return BD_VALID;
	for (int v = 0; v < 2; v++)
		if (tok[v]) { *out = tolr[v]; return BD_VALID; }
	if (raw_fits) *out = raw_out;			/* report the stripes as read */
	return BD_INVALID;
}

bd_result_t bd_decode(const uint32_t *w_in, const int16_t *pk_in, int n, bd_read_t *out)
{
	return decode(w_in, pk_in, n, out, BAR_ALLOW_BAD_GUARD);
}

void bd_edge(bd_t *b, uint32_t t, bool black) { bd_edge_p(b, t, black, -1); }

void bd_edge_p(bd_t *b, uint32_t t, bool black, int peak) { bd_edge_pos(b, t, BD_NO_POS, black, peak); }

void bd_edge_pos(bd_t *b, uint32_t t, int32_t pos, bool black, int peak)
{
	if (!b->active) {
		if (black) {
			b->active = true; b->black = true; b->t_start = t; b->n = 0; b->widest = 0; b->tried_n = 0; b->narrowest = 0; b->widest_space = 0;
			b->p_start = pos; b->pos_ok = pos != BD_NO_POS;
		}
		return;
	}
	if (pos == BD_NO_POS) b->pos_ok = false;
	if (black == b->black) return;
	uint32_t d = t - b->t_start;
	if (d < BAR_MIN_US && b->n > 0) {		/* glitch: re-open the previous element and carry on with it */
		b->n--;
		b->t_start -= b->w[b->n];
		b->p_start -= (int32_t)b->d[b->n];
		b->black = black;
		/* the re-opened element no longer counts: rebuild the widest / narrowest it may have set
		   (element 0 is always a bar, so even index = bar, odd = space; the first bar is not counted as widest) */
		b->widest = 0; b->narrowest = 0; b->widest_space = 0;
		for (int i = 0; i < b->n; i++) {
			if (i % 2 == 0) {
				if (i > 0 && b->w[i] > b->widest) b->widest = b->w[i];
				if (b->narrowest == 0 || b->w[i] < b->narrowest) b->narrowest = b->w[i];
			} else if (b->w[i] > b->widest_space) {
				b->widest_space = b->w[i];
			}
		}
		return;
	}
	if (b->black && d > BAR_MAX_US && (peak < 0 || peak >= BAR_LINE_PEAK)) {
		b->active = false;			/* long AND dark: the line or a junction, not a barcode */
		return;
	}
	if (b->n >= BAR_MAX_ELEMENTS) { b->active = false; return; }
	b->pk[b->n] = (int16_t)peak;
	int32_t dp = b->pos_ok ? pos - b->p_start : 0;
	b->d[b->n] = dp > 0 ? (uint32_t)dp : 0;
	b->w[b->n++] = d;
	if (b->black && b->n > 1 && d > b->widest)	/* widest BAR, not counting the first one (it can be a slow */
		b->widest = d;				/* rise) nor white gaps: they must not stretch the end-of-code wait */
	if (b->black && (b->narrowest == 0 || d < b->narrowest))
		b->narrowest = d;
	if (!b->black && d > b->widest_space)
		b->widest_space = d;
	b->black = black;
	b->t_start = t;
	b->p_start = pos;
}

static bool dist_ok(const bd_t *b, int n)
{
	if (!b->pos_ok) return false;
	for (int i = 0; i < n; i++)
		if (b->d[i] < BAR_DIST_MIN) return false;
	return true;
}

static bd_result_t decode_read(const bd_t *b, int n, bd_read_t *out, bool allow_damaged)
{
	if (!dist_ok(b, n)) return decode(b->w, b->pk, n, out, allow_damaged);
	bd_result_t r = decode(b->d, b->pk, n, out, allow_damaged);
	uint32_t lo = 0xFFFFFFFFu;
	for (int i = 0; i < n; i++) if (b->w[i] < lo) lo = b->w[i];
	out->narrow_us = lo;
	out->by_distance = true;
	return r;
}

bd_result_t bd_poll(bd_t *b, uint32_t now, bd_read_t *out)
{
	if (!b->active) return BD_NONE;
	uint32_t d = now - b->t_start;
	if (b->black) {
		if (d > BAR_MAX_WEAK_US) b->active = false;	/* sitting on black: not a barcode */
		return BD_NONE;
	}
	uint32_t widest = b->widest ? b->widest : (b->n ? b->w[0] : 0);

	/* Early finish: whole characters are in (29, 39 ... stripes) and the white after them is already longer than
	   any wide element: if they decode, report now, so the command is known well before the junction. */
	if (b->n >= 29 && (b->n + 1) % 10 == 0 && b->tried_n != b->n && d > widest + widest / 2) {
		b->tried_n = b->n;
		if (decode_read(b, b->n, out, false) == BD_VALID) {
			b->active = false;
			return BD_VALID;
		}
	}

	/* The white after a code is a quiet zone of at least 10 narrow bars (Code 39); inside a code no white is much wider
	   than the widest seen so far. Blur makes spaces beside thin bars look several times wider than the thin bars, so
	   wait for the LONGEST of: 4 x widest bar, 10 x narrowest bar, 1.5 x widest space so far.
	   With only the first bar seen, the next space may be a wide one that blur makes look even wider: wait longer. */
	uint32_t quiet = widest * (b->widest ? BAR_END_FACTOR : 3 * BAR_END_FACTOR);
	if (b->narrowest * 10u > quiet) quiet = b->narrowest * 10u;
	if (b->widest_space + b->widest_space / 2 > quiet) quiet = b->widest_space + b->widest_space / 2;
	if (quiet < BAR_END_MIN_US) quiet = BAR_END_MIN_US;
	if (d < quiet) return BD_NONE;
	b->active = false;
	if (b->n < 3) return BD_NONE;
	return decode_read(b, b->n, out, BAR_ALLOW_BAD_GUARD);
}

bd_result_t bd_finish(bd_t *b, bd_read_t *out)
{
	static bd_read_t alt;				/* static: only the barcode task decodes */
	if (!b->active) return BD_NONE;
	b->active = false;
	int n = b->n;					/* completed stripes; the one still being timed is not in w[] */
	if (n % 2 == 0) n--;				/* a code ends with a bar: drop a trailing white */
	if (n < 3) return BD_NONE;
	bd_result_t r = decode_read(b, n, out, BAR_ALLOW_BAD_GUARD);
	if (r == BD_VALID || n < 5) return r;
	if (decode_read(b, n - 2, &alt, BAR_ALLOW_BAD_GUARD) == BD_VALID) {	/* the last bar may be the junction's line: try without it */
		*out = alt;
		return BD_VALID;
	}
	return r;
}
