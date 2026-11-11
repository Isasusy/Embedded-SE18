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

bd_result_t bd_decode(const uint32_t *w, int n, bd_read_t *out)
{
	memset(out, 0, sizeof *out);
	out->nav = -1;
	out->n_elements = n;

	/* the log pattern: wide/narrow by the midpoint of the shortest and longest element */
	uint32_t lo = 0xFFFFFFFFu, hi = 0;
	for (int i = 0; i < n; i++) { if (w[i] < lo) lo = w[i]; if (w[i] > hi) hi = w[i]; }
	out->narrow_us = n ? lo : 0;
	int p = 0;
	for (int i = 0; i < n && p < (int)sizeof out->pattern - 2; i++) {
		if (i > 0 && (i % 10 == 9 || i % 10 == 0) && (n + 1) % 10 == 0) out->pattern[p++] = ' ';	/* gap between characters */
		out->pattern[p++] = (w[i] > (lo + hi) / 2) ? 'W' : 'n';
	}
	out->pattern[p] = 0;

	if (try_decode(w, n, out->text)) {
		out->nav = nav_of(out->text[0]);
		return BD_VALID;
	}
	uint32_t r[BAR_MAX_ELEMENTS];
	for (int i = 0; i < n; i++) r[i] = w[n - 1 - i];
	if (try_decode(r, n, out->text)) {
		out->reversed = true;
		out->nav = nav_of(out->text[0]);
		return BD_VALID;
	}
	return BD_INVALID;
}

void bd_edge(bd_t *b, uint32_t t, bool black)
{
	if (!b->active) {
		if (black) { b->active = true; b->black = true; b->t_start = t; b->n = 0; b->widest = 0; }
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
	if (b->black && d > BAR_MAX_US) {		/* far too long for a bar: the line or a junction, not a barcode */
		b->active = false;
		return;
	}
	if (b->n >= BAR_MAX_ELEMENTS) { b->active = false; return; }
	b->w[b->n++] = d;
	if (d > b->widest) b->widest = d;
	b->black = black;
	b->t_start = t;
}

bd_result_t bd_poll(bd_t *b, uint32_t now, bd_read_t *out)
{
	if (!b->active) return BD_NONE;
	uint32_t d = now - b->t_start;
	if (b->black) {
		if (d > BAR_MAX_US) b->active = false;	/* sitting on black: not a barcode */
		return BD_NONE;
	}
	uint32_t quiet = b->widest * BAR_END_FACTOR;
	if (quiet < BAR_END_MIN_US) quiet = BAR_END_MIN_US;
	if (d < quiet) return BD_NONE;
	b->active = false;				/* the quiet zone after the code: decode what we have */
	if (b->n < 3) return BD_NONE;			/* a single stripe or a speck: not worth a log line */
	return bd_decode(b->w, b->n, out);
}
