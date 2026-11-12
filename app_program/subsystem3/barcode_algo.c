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

static char lookup(uint16_t p)
{
	for (unsigned i = 0; i < sizeof c39 / sizeof c39[0]; i++)
		if (c39[i].p == p) return c39[i].c;
	return 0;
}

static int nav_of(char c)
{
	switch (c) {
	case BAR_CHAR_LEFT:	return NAV_LEFT;
	case BAR_CHAR_RIGHT:	return NAV_RIGHT;
	case BAR_CHAR_STRAIGHT:	return NAV_STRAIGHT;
	case BAR_CHAR_UTURN:	return NAV_UTURN;
	default:		return -1;
	}
}

/* one character = 9 widths: the 3 widest are wide */
static uint16_t char_bits(const uint32_t *w)
{
	uint16_t bits = 0;
	for (int k = 0; k < 3; k++) {
		int best = -1;
		for (int i = 0; i < 9; i++)
			if (!(bits & (1u << (8 - i))) && (best < 0 || w[i] > w[best])) best = i;
		bits |= (uint16_t)(1u << (8 - best));
	}
	return bits;
}

/* the middle value of a small list (destroys the list) */
static uint32_t median(uint32_t *v, int n)
{
	for (int i = 1; i < n; i++)
		for (int j = i; j > 0 && v[j] < v[j - 1]; j--) { uint32_t t = v[j]; v[j] = v[j - 1]; v[j - 1] = t; }
	return n ? v[n / 2] : 0;
}

/* ---- which bars are thick ----
   Tried in this order until the whole code decodes. Every character must come out as a valid Code 39 character and
   the code must be "*...*", so a wrong guess is rejected and the next method is tried.
     BAR_BY_LENGTH    sort the bar durations and split them where they jump the most (a 12 Nov bench run: thin bars
                      99-140 ms, thick 380-519 ms). Any print size and speed, as long as the speed is roughly steady.
     BAR_BY_LENGTH2   split at the SECOND biggest jump. If the course's bars come in three sizes (thin, medium, thick),
                      the two splits are the two possible meanings of "medium" (counted wide, or counted narrow).
     BAR_BY_DARKNESS  a blurred thin bar never gets as dark as a thick one (11 Nov 18:48 run: thin ~300, thick ~2400).
   The first and the last bar of a code are always THIN: the '*' start/stop character begins and ends with a thin bar,
   read in either direction. They are also the least reliable stripes (a slow rise into the first bar, the junction
   line right after the last), so they are set thin and left out of the split. (That 12 Nov read failed only
   because the last bar, 118 ms long, got as dark as a thick one.) */
typedef enum { BAR_BY_LENGTH, BAR_BY_LENGTH2, BAR_BY_DARKNESS, BAR_N_METHODS } bar_method_t;
static const char *const method_name[] = { "bar length", "bar length, 2nd split", "bar darkness", "element widths" };

static void sort_u32(uint32_t *v, int n)
{
	for (int i = 1; i < n; i++)
		for (int j = i; j > 0 && v[j] < v[j - 1]; j--) { uint32_t t = v[j]; v[j] = v[j - 1]; v[j - 1] = t; }
}

/* fills thick[i] for every bar (even i < n); false = this method cannot tell thin from thick here */
static bool classify_bars(int method, const uint32_t *w, const int16_t *pk, int n, bool *thick)
{
	if (n < 5) return false;
	for (int i = 0; i < n; i += 2) thick[i] = false;	/* first and last bar stay thin */

	if (method == BAR_BY_DARKNESS) {
		if (!pk) return false;
		int lo = 32767, hi = -1;
		for (int i = 2; i < n - 1; i += 2) {
			if (pk[i] < 0) return false;
			if (pk[i] < lo) lo = pk[i];
			if (pk[i] > hi) hi = pk[i];
		}
		if (hi - lo < BAR_PEAK_SPLIT) return false;
		int thr = (lo + hi) / 2;
		for (int i = 2; i < n - 1; i += 2) thick[i] = pk[i] > thr;
		return true;
	}

	static uint32_t s[BAR_MAX_ELEMENTS];
	int nb = 0;
	for (int i = 2; i < n - 1; i += 2) s[nb++] = w[i];
	if (nb < 2) return false;
	sort_u32(s, nb);
	int j1 = -1, j2 = -1;				/* where the biggest and second biggest jumps are */
	float r1 = 0.0f, r2 = 0.0f;
	for (int j = 0; j + 1 < nb; j++) {
		float r = s[j] ? (float)s[j + 1] / (float)s[j] : 0.0f;
		if (r > r1)      { j2 = j1; r2 = r1; j1 = j; r1 = r; }
		else if (r > r2) { j2 = j; r2 = r; }
	}
	int j = (method == BAR_BY_LENGTH) ? j1 : j2;
	float r = (method == BAR_BY_LENGTH) ? r1 : r2;
	if (j < 0 || r < BAR_LEN_SPLIT_RATIO) return false;	/* no clear jump: bars all look alike */
	uint32_t thr = s[j] + (s[j + 1] - s[j]) / 2;
	for (int i = 2; i < n - 1; i += 2) thick[i] = w[i] > thr;
	return true;
}

/* Blur-proof decode with the bars already classified (the course's IR sensor sees a spot wider than a 3 mm bar):
   which space is wide comes from the distance between bar CENTRES, minus the half-widths of the two bars
   (thin = 1 unit, thick = 3): a blurred thin bar looks short, which makes its neighbouring spaces look long,
   but its centre stays put. Each character has exactly 3 wide elements, so 2 thick bars -> 1 wide space. */
/* `tolerant`: accept the code when ONE of the two '*' start/stop characters is unreadable, as long as the other '*'
   and every character between them are valid (*damaged is then set). The '*' only marks the ends; the letter is
   what matters, and the first '*' is where a read most often goes wrong (12 Nov: the car was steering onto it). */
static bool try_decode_bars(const uint32_t *w, const bool *thickbar, int n, char *text, bool *wide_out,
			    bool tolerant, bool *damaged)
{
	*damaged = false;
	if (n < 29 || (n + 1) % 10 != 0) return false;

	static uint32_t thin[BAR_MAX_ELEMENTS], thick[BAR_MAX_ELEMENTS];
	int nt = 0, nk = 0;
	for (int i = 0; i < n; i += 2) { if (thickbar[i]) thick[nk++] = w[i]; else thin[nt++] = w[i]; }
	float med_thin = (float)median(thin, nt), med_thick = (float)median(thick, nk);

	static float centre[BAR_MAX_ELEMENTS];
	float start = 0.0f;
	for (int i = 0; i < n; i++) {
		if (i % 2 == 0) {
			float m = thickbar[i] ? med_thick : med_thin;
			float d = (float)w[i];
			/* a bar that lasted far longer than its kind (a slow rise into the first bar): trust its END */
			centre[i] = (m > 0.0f && d > 2.0f * m) ? start + d - 0.5f * m : start + 0.5f * d;
		}
		start += (float)w[i];
	}

	int k = (n + 1) / 10, t = 0, bad = 0;
	for (int c = 0; c < k; c++) {
		int b0 = c * 10, wu[5], nwb = 0;
		char ch = 0;					/* stays 0 if this character cannot be read */
		for (int j = 0; j < 5; j++) { wu[j] = thickbar[b0 + 2 * j] ? 3 : 1; if (wu[j] == 3) nwb++; }
		int nws = 3 - nwb;
		if (nws == 1 || nws == 3) {
			float d[4], half[4], sd = 0.0f, sh = 0.0f, r[4];
			for (int j = 0; j < 4; j++) {
				d[j] = centre[b0 + 2 * j + 2] - centre[b0 + 2 * j];
				half[j] = 0.5f * (float)(wu[j] + wu[j + 1]);
				sd += d[j]; sh += half[j];
			}
			float unit = sd / (sh + (float)(3 * nws + (4 - nws)));
			if (unit > 0.0f) {
				for (int j = 0; j < 4; j++) r[j] = d[j] / unit - half[j];	/* ~1 narrow space, ~3 wide */
				bool ws[4] = { false, false, false, false };
				for (int m = 0; m < nws; m++) {
					int best = -1;
					for (int j = 0; j < 4; j++) if (!ws[j] && (best < 0 || r[j] > r[best])) best = j;
					ws[best] = true;
				}
				uint16_t bits = 0;
				for (int e = 0; e < 9; e++) {
					bool wide = (e % 2 == 0) ? (wu[e / 2] == 3) : ws[e / 2];
					bits |= (uint16_t)((wide ? 1u : 0u) << (8 - e));
					if (wide_out) wide_out[b0 + e] = wide;
				}
				ch = lookup(bits);
			}
		}
		if (wide_out && c < k - 1) wide_out[b0 + 9] = false;	/* the gap between characters */
		bool guard = (c == 0 || c == k - 1);
		if (guard ? ch != '*' : (!ch || ch == '*')) {
			if (!guard || !tolerant || ++bad > 1) return false;
			continue;				/* one damaged '*': the letters between still count */
		}
		if (!guard && t < 7) text[t++] = ch;
	}
	text[t] = 0;
	*damaged = bad > 0;
	return t > 0;
}

/* try one orientation; true when it is "*...*" and every character is valid (`tolerant`: see try_decode_bars) */
static bool try_decode(const uint32_t *w, int n, char *text, bool tolerant, bool *damaged)
{
	*damaged = false;
	if (n < 29 || (n + 1) % 10 != 0) return false;	/* k characters = 10k - 1 elements, at least "*X*" */
	int k = (n + 1) / 10, t = 0, bad = 0;
	for (int i = 0; i < k; i++) {
		char c = lookup(char_bits(&w[i * 10]));
		bool guard = (i == 0 || i == k - 1);
		if (guard ? c != '*' : (!c || c == '*')) {
			if (!guard || !tolerant || ++bad > 1) return false;
			continue;
		}
		if (!guard && t < 7) text[t++] = c;
	}
	text[t] = 0;
	*damaged = bad > 0;
	return t > 0;
}

/* Dark fragments (12 Nov bench log): very dark bars (3500-4000) flicker by more than BAR_SWING, so one thick bar came
   in as up to 13 stripes, the "spaces" between them never getting lighter than ~3000 while real spaces read ~200.
   Any space whose lightest reading stays in the top BAR_DIP_LEVEL_PCT % between white (the lightest space) and dark
   (the upper quartile of the bar peaks) is part of the bar around it: merge it, whatever the stripe count. */
static void merge_dark_dips(uint32_t *w, int16_t *pk, int *pn)
{
	static int16_t s[BAR_MAX_ELEMENTS];
	int n = *pn, nb = 0, white = 32767;

	for (int i = 0; i < n; i++) {
		if (pk[i] < 0) return;			/* peaks unknown: nothing to go on */
		if (i % 2 == 0) s[nb++] = pk[i];
		else if (pk[i] < white) white = pk[i];
	}
	if (nb < 2 || white == 32767) return;
	for (int i = 1; i < nb; i++)
		for (int j = i; j > 0 && s[j] < s[j - 1]; j--) { int16_t x = s[j]; s[j] = s[j - 1]; s[j - 1] = x; }
	int dark = s[(3 * nb) / 4];
	if (dark - white < BAR_PEAK_SPLIT) return;	/* bars hardly darker than the spaces: leave it alone */
	int lim = white + (BAR_DIP_LEVEL_PCT * (dark - white)) / 100;

	int o = 0;					/* write position (never ahead of i) */
	for (int i = 0; i < n; i++) {
		if (i % 2 == 1 && i + 1 < n && pk[i] >= lim) {	/* a dip: it and the bar after it join the bar before */
			w[o - 1] += w[i] + w[i + 1];
			if (pk[i + 1] > pk[o - 1]) pk[o - 1] = pk[i + 1];
			i++;
			continue;
		}
		w[o] = w[i]; pk[o] = pk[i]; o++;
	}
	*pn = o;
}

/* allow_damaged: accept one unreadable '*' (see try_decode_bars). Off for the EARLY finish in bd_poll: there the code
   may still be going on, and a letter in the place of the closing '*' would look like a damaged '*'. */
static bd_result_t decode(const uint32_t *w_in, const int16_t *pk_in, int n, bd_read_t *out, bool allow_damaged)
{
	memset(out, 0, sizeof *out);
	out->nav = -1;
	out->n_elements = n;

	/* Too many stripes for whole characters? A "white" that stayed dark (a shallow dip, seen on 11 Nov as 99 ms
	   that only got down to 2173) split one bar in two: merge it back, darkest dip first. */
	static uint32_t w[BAR_MAX_ELEMENTS];		/* static: big, and only the barcode task decodes */
	static int16_t pkm[BAR_MAX_ELEMENTS];
	const int16_t *pk = pk_in ? pkm : NULL;
	for (int i = 0; i < n; i++) { w[i] = w_in[i]; pkm[i] = pk_in ? pk_in[i] : -1; }
	if (pk) merge_dark_dips(w, pkm, &n);		/* spaces that never got light: always part of a bar */
	/* then, if the count still does not fit, the darkest of the remaining dips, one at a time;
	   only for an ODD count (a code starts and ends with a bar): merging removes 2 stripes, so an even count can never
	   reach 10k - 1 and would only lose information */
	while (pk && n >= 31 && n % 2 == 1 && (n + 1) % 10 != 0) {
		int dip = -1;
		for (int i = 1; i < n - 1; i += 2)
			if (pkm[i] >= BAR_DIP_MIN && (dip < 0 || pkm[i] > pkm[dip])) dip = i;
		if (dip < 0) break;
		w[dip - 1] += w[dip] + w[dip + 1];
		if (pkm[dip + 1] > pkm[dip - 1]) pkm[dip - 1] = pkm[dip + 1];
		for (int i = dip; i + 2 < n; i++) { w[i] = w[i + 2]; pkm[i] = pkm[i + 2]; }
		n -= 2;
	}

	/* the log pattern: bars by the first method that can classify them, spaces (and bars if none can) by the
	   midpoint of all the widths */
	static bool thick[BAR_MAX_ELEMENTS], rthick[BAR_MAX_ELEMENTS], wide[BAR_MAX_ELEMENTS];
	uint32_t lo = 0xFFFFFFFFu, hi = 0;
	for (int i = 0; i < n; i++) { if (w[i] < lo) lo = w[i]; if (w[i] > hi) hi = w[i]; }
	bool have_bars = false;
	for (int m = 0; m < BAR_N_METHODS && !have_bars; m++) have_bars = classify_bars(m, w, pk, n, thick);
	out->narrow_us = n ? lo : 0;
	int p = 0;
	for (int i = 0; i < n && p < (int)sizeof out->pattern - 2; i++) {
		if (i > 0 && (i % 10 == 9 || i % 10 == 0) && (n + 1) % 10 == 0) out->pattern[p++] = ' ';	/* gap between characters */
		bool wd = (have_bars && i % 2 == 0) ? thick[i] : w[i] > (lo + hi) / 2;
		out->pattern[p++] = wd ? 'W' : 'n';
	}
	out->pattern[p] = 0;

	static uint32_t r[BAR_MAX_ELEMENTS];
	for (int i = 0; i < n; i++) r[i] = w[n - 1 - i];

	/* pass 0: every character must be valid. pass 1 (if allowed): one '*' may be damaged. A fully valid read by any
	   method always wins over a damaged one. */
	for (int pass = 0; pass < (allow_damaged ? 2 : 1); pass++) {
		bool tol = (pass == 1), dmg = false;
		for (int m = 0; m < BAR_N_METHODS; m++) {
			if (!classify_bars(m, w, pk, n, thick)) continue;
			for (int i = 0; i < n; i += 2) rthick[i] = thick[n - 1 - i];	/* n is odd here: bars map to bars */
			for (int dir = 0; dir < 2; dir++) {
				if (try_decode_bars(dir ? r : w, dir ? rthick : thick, n, out->text, wide, tol, &dmg)) {
					out->method = method_name[m];
					out->reversed = dir;
					out->guard_damaged = dmg;
					out->nav = nav_of(out->text[0]);
					return BD_VALID;
				}
			}
		}
		/* last resort, no blur handling: in each character the 3 longest elements are the wide ones */
		for (int dir = 0; dir < 2; dir++) {
			if (try_decode(dir ? r : w, n, out->text, tol, &dmg)) {
				out->method = method_name[BAR_N_METHODS];
				out->reversed = dir;
				out->guard_damaged = dmg;
				out->nav = nav_of(out->text[0]);
				return BD_VALID;
			}
		}
	}
	out->text[0] = 0;
	return BD_INVALID;
}

bd_result_t bd_decode(const uint32_t *w_in, const int16_t *pk_in, int n, bd_read_t *out)
{
	return decode(w_in, pk_in, n, out, BAR_ALLOW_BAD_GUARD);
}

void bd_edge(bd_t *b, uint32_t t, bool black) { bd_edge_p(b, t, black, -1); }

void bd_edge_p(bd_t *b, uint32_t t, bool black, int peak)
{
	if (!b->active) {
		if (black) { b->active = true; b->black = true; b->t_start = t; b->n = 0; b->widest = 0; b->tried_n = 0; b->narrowest = 0; b->widest_space = 0; }
		return;					/* waiting for the first bar */
	}
	if (black == b->black) return;			/* not a change */
	uint32_t d = t - b->t_start;
	if (d < BAR_MIN_US && b->n > 0) {		/* glitch: re-open the previous element and carry on with it */
		b->n--;
		b->t_start -= b->w[b->n];
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
	b->w[b->n++] = d;
	if (b->black && b->n > 1 && d > b->widest)	/* widest BAR, not counting the first one (it can be a slow */
		b->widest = d;				/* rise) nor white gaps: they must not stretch the end-of-code wait */
	if (b->black && (b->narrowest == 0 || d < b->narrowest))
		b->narrowest = d;
	if (!b->black && d > b->widest_space)
		b->widest_space = d;
	b->black = black;
	b->t_start = t;
}

bd_result_t bd_poll(bd_t *b, uint32_t now, bd_read_t *out)
{
	if (!b->active) return BD_NONE;
	uint32_t d = now - b->t_start;
	if (b->black) {
		if (d > BAR_MAX_WEAK_US) b->active = false;	/* sitting on black: not a barcode */
		return BD_NONE;
	}
	uint32_t widest = b->widest ? b->widest : (b->n ? b->w[0] : 0);	/* only one bar so far: use it */

	/* Early finish: whole characters are in (29, 39 ... stripes) and the white after them is already longer than
	   any wide element: if they decode, report now, so the command is known well before the junction. */
	if (b->n >= 29 && (b->n + 1) % 10 == 0 && b->tried_n != b->n && d > widest + widest / 2) {
		b->tried_n = b->n;
		if (decode(b->w, b->pk, b->n, out, false) == BD_VALID) {	/* strict: the code may still go on */
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
	b->active = false;				/* the quiet zone after the code: decode what we have */
	if (b->n < 3) return BD_NONE;			/* a single stripe or a speck: not worth a log line */
	return bd_decode(b->w, b->pk, b->n, out);
}

bd_result_t bd_finish(bd_t *b, bd_read_t *out)
{
	static bd_read_t alt;				/* static: only the barcode task decodes */
	if (!b->active) return BD_NONE;
	b->active = false;
	int n = b->n;					/* completed stripes; the one still being timed is not in w[] */
	if (n % 2 == 0) n--;				/* a code ends with a bar: drop a trailing white */
	if (n < 3) return BD_NONE;
	bd_result_t r = bd_decode(b->w, b->pk, n, out);
	if (r == BD_VALID || n < 5) return r;
	if (bd_decode(b->w, b->pk, n - 2, &alt) == BD_VALID) {	/* the last bar may be the junction's line: try without it */
		*out = alt;
		return BD_VALID;
	}
	return r;
}
