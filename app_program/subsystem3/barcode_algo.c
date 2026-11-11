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

/* Blur-proof decode (the course's IR sensor sees a spot wider than a 3 mm bar):
   - a bar is THICK or THIN by how dark it gets (pk), not by how long it lasts: a thin bar is only a small bump;
   - which space is wide comes from the distance between bar CENTRES, minus the half-widths of the two bars
     (thin = 1 unit, thick = 3): a blurred thin bar looks short, which makes its neighbouring spaces look long,
     but its centre stays put. Each character has exactly 3 wide elements, so 2 thick bars -> 1 wide space. */
static bool try_decode_peaks(const uint32_t *w, const int16_t *pk, int n, char *text, bool *wide_out)
{
	if (n < 29 || (n + 1) % 10 != 0) return false;
	int lo = 32767, hi = -1;
	for (int i = 0; i < n; i += 2) {
		if (pk[i] < 0) return false;
		if (pk[i] < lo) lo = pk[i];
		if (pk[i] > hi) hi = pk[i];
	}
	if (hi - lo < BAR_PEAK_SPLIT) return false;		/* cannot tell thin from thick by darkness */
	int thr = (lo + hi) / 2;

	static uint32_t thin[BAR_MAX_ELEMENTS], thick[BAR_MAX_ELEMENTS];
	int nt = 0, nk = 0;
	for (int i = 0; i < n; i += 2) { if (pk[i] > thr) thick[nk++] = w[i]; else thin[nt++] = w[i]; }
	float med_thin = (float)median(thin, nt), med_thick = (float)median(thick, nk);

	static float centre[BAR_MAX_ELEMENTS];
	float start = 0.0f;
	for (int i = 0; i < n; i++) {
		if (i % 2 == 0) {
			float m = (pk[i] > thr) ? med_thick : med_thin;
			float d = (float)w[i];
			/* a bar that lasted far longer than its kind (a slow rise into the first bar): trust its END */
			centre[i] = (m > 0.0f && d > 2.0f * m) ? start + d - 0.5f * m : start + 0.5f * d;
		}
		start += (float)w[i];
	}

	int k = (n + 1) / 10, t = 0;
	for (int c = 0; c < k; c++) {
		int b0 = c * 10, wu[5], nwb = 0;
		for (int j = 0; j < 5; j++) { wu[j] = (pk[b0 + 2 * j] > thr) ? 3 : 1; if (wu[j] == 3) nwb++; }
		int nws = 3 - nwb;
		if (nws != 1 && nws != 3) return false;
		float d[4], half[4], sd = 0.0f, sh = 0.0f, r[4];
		for (int j = 0; j < 4; j++) {
			d[j] = centre[b0 + 2 * j + 2] - centre[b0 + 2 * j];
			half[j] = 0.5f * (float)(wu[j] + wu[j + 1]);
			sd += d[j]; sh += half[j];
		}
		float unit = sd / (sh + (float)(3 * nws + (4 - nws)));
		if (unit <= 0.0f) return false;
		for (int j = 0; j < 4; j++) r[j] = d[j] / unit - half[j];	/* ~1 for a narrow space, ~3 for a wide one */
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
		if (wide_out && c < k - 1) wide_out[b0 + 9] = false;	/* the gap between characters */
		char ch = lookup(bits);
		if (!ch) return false;
		if ((c == 0 || c == k - 1) ? ch != '*' : ch == '*') return false;
		if (c > 0 && c < k - 1 && t < 7) text[t++] = ch;
	}
	text[t] = 0;
	return t > 0;
}

/* try one orientation; true when it is "*...*" and every character is valid */
static bool try_decode(const uint32_t *w, int n, char *text)
{
	if (n < 29 || (n + 1) % 10 != 0) return false;	/* k characters = 10k - 1 elements, at least "*X*" */
	int k = (n + 1) / 10, t = 0;
	for (int i = 0; i < k; i++) {
		char c = lookup(char_bits(&w[i * 10]));
		if (!c) return false;
		if ((i == 0 || i == k - 1) ? c != '*' : c == '*') return false;
		if (i > 0 && i < k - 1 && t < 7) text[t++] = c;
	}
	text[t] = 0;
	return t > 0;
}

bd_result_t bd_decode(const uint32_t *w_in, const int16_t *pk_in, int n, bd_read_t *out)
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
	while (pk && n >= 31 && (n + 1) % 10 != 0) {
		int dip = -1;
		for (int i = 1; i < n - 1; i += 2)
			if (pkm[i] >= BAR_DIP_MIN && (dip < 0 || pkm[i] > pkm[dip])) dip = i;
		if (dip < 0) break;
		w[dip - 1] += w[dip] + w[dip + 1];
		if (pkm[dip + 1] > pkm[dip - 1]) pkm[dip - 1] = pkm[dip + 1];
		for (int i = dip; i + 2 < n; i++) { w[i] = w[i + 2]; pkm[i] = pkm[i + 2]; }
		n -= 2;
	}

	/* the log pattern: bars thin/thick by darkness when known, otherwise everything by the midpoint of the widths */
	uint32_t lo = 0xFFFFFFFFu, hi = 0;
	int plo = 32767, phi = -1;
	for (int i = 0; i < n; i++) { if (w[i] < lo) lo = w[i]; if (w[i] > hi) hi = w[i]; }
	if (pk) for (int i = 0; i < n; i += 2) { if (pk[i] < plo) plo = pk[i]; if (pk[i] > phi) phi = pk[i]; }
	bool by_peak = pk && plo >= 0 && phi - plo >= BAR_PEAK_SPLIT;
	out->narrow_us = n ? lo : 0;
	int p = 0;
	for (int i = 0; i < n && p < (int)sizeof out->pattern - 2; i++) {
		if (i > 0 && (i % 10 == 9 || i % 10 == 0) && (n + 1) % 10 == 0) out->pattern[p++] = ' ';	/* gap between characters */
		bool wide = (by_peak && i % 2 == 0) ? pk[i] > (plo + phi) / 2 : w[i] > (lo + hi) / 2;
		out->pattern[p++] = wide ? 'W' : 'n';
	}
	out->pattern[p] = 0;

	static uint32_t r[BAR_MAX_ELEMENTS];
	static int16_t rp[BAR_MAX_ELEMENTS];
	for (int i = 0; i < n; i++) { r[i] = w[n - 1 - i]; rp[i] = pk ? pk[n - 1 - i] : -1; }

	if (by_peak) {
		static bool wide[BAR_MAX_ELEMENTS];
		if (try_decode_peaks(w, pk, n, out->text, wide)) {
			out->nav = nav_of(out->text[0]);
			return BD_VALID;
		}
		if (try_decode_peaks(r, rp, n, out->text, wide)) {
			out->reversed = true;
			out->nav = nav_of(out->text[0]);
			return BD_VALID;
		}
	}
	if (try_decode(w, n, out->text)) {
		out->nav = nav_of(out->text[0]);
		return BD_VALID;
	}
	if (try_decode(r, n, out->text)) {
		out->reversed = true;
		out->nav = nav_of(out->text[0]);
		return BD_VALID;
	}
	out->text[0] = 0;
	return BD_INVALID;
}

void bd_edge(bd_t *b, uint32_t t, bool black) { bd_edge_p(b, t, black, -1); }

void bd_edge_p(bd_t *b, uint32_t t, bool black, int peak)
{
	if (!b->active) {
		if (black) { b->active = true; b->black = true; b->t_start = t; b->n = 0; b->widest = 0; b->tried_n = 0; }
		return;					/* waiting for the first bar */
	}
	if (black == b->black) return;			/* not a change */
	uint32_t d = t - b->t_start;
	if (d < BAR_MIN_US && b->n > 0) {		/* glitch: re-open the previous element and carry on with it */
		b->n--;
		b->t_start -= b->w[b->n];
		b->black = black;
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
		if (bd_decode(b->w, b->pk, b->n, out) == BD_VALID) {
			b->active = false;
			return BD_VALID;
		}
	}

	uint32_t quiet = widest * BAR_END_FACTOR;
	if (quiet < BAR_END_MIN_US) quiet = BAR_END_MIN_US;
	if (d < quiet) return BD_NONE;
	b->active = false;				/* the quiet zone after the code: decode what we have */
	if (b->n < 3) return BD_NONE;			/* a single stripe or a speck: not worth a log line */
	return bd_decode(b->w, b->pk, b->n, out);
}
