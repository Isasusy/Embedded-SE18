/*
 *	barcode_algo.h / barcode_algo.c -- Barcode Decoder logic (statechart 6.1: BARCODE_SCAN, DECODED).
 *	Code 39, either direction. Fed with timestamped edges of IR3 (GP28). No kernel or hardware calls.
 */
#ifndef BARCODE_ALGO_H
#define BARCODE_ALGO_H

#include <stdint.h>
#include <stdbool.h>
#include "line_config.h"

typedef struct {
	bool		active;				/* inside a barcode */
	bool		black;				/* level of the element being timed */
	uint32_t	t_start;			/* when that element started */
	int		n;				/* completed elements */
	uint32_t	w[BAR_MAX_ELEMENTS];		/* their widths, us: bar, space, bar, ... */
	int16_t		pk[BAR_MAX_ELEMENTS];		/* their peak reading (darkest for a bar), -1 = unknown */
	uint32_t	widest;				/* widest black bar so far */
	uint32_t	narrowest;			/* narrowest black bar so far */
	uint32_t	widest_space;			/* widest white inside the code so far */
	int		tried_n;			/* element count of the last early decode attempt */
} bd_t;

typedef struct {
	char		text[8];			/* decoded characters without the '*'s, e.g. "A" */
	int		nav;				/* nav_cmd_type_t, or -1 if the first character is not a command */
	bool		reversed;			/* read back to front */
	int		n_elements;
	char		pattern[BAR_MAX_ELEMENTS + 12];	/* 'W' wide / 'n' narrow, a space between characters */
	uint32_t	narrow_us;			/* typical narrow element, for the log */
	const char	*method;			/* how the thick bars were found (for the log), NULL if not decoded */
	bool		guard_damaged;			/* decoded although one '*' start/stop character was unreadable */
} bd_read_t;

typedef enum { BD_NONE, BD_VALID, BD_INVALID } bd_result_t;

void		bd_init(bd_t *b);
/* feed one edge; `black` = level after the edge, `peak` = darkest (bar) / lightest (space) reading of the stripe that
   just ended, or -1 if unknown. Never completes a read (the quiet zone after the code does). */
void		bd_edge_p(bd_t *b, uint32_t t_us, bool black, int peak);
void		bd_edge(bd_t *b, uint32_t t_us, bool black);		/* same, peak unknown */
/* call every ~1 ms: when the white after a code has lasted long enough, decodes it into *out.
   BD_VALID = a Code 39 code was read, BD_INVALID = something was read but did not decode (out->pattern says what). */
bd_result_t	bd_poll(bd_t *b, uint32_t now_us, bd_read_t *out);

/* end an open read NOW (the junction has arrived, so the barcode is behind the car) and decode what was read.
   A trailing white, or a last bar that is really the junction's own line, is left out. BD_NONE if no read was open. */
bd_result_t	bd_finish(bd_t *b, bd_read_t *out);

/* decode a finished list of element widths and peaks (pk may be NULL) (exposed for tests) */
bd_result_t	bd_decode(const uint32_t *w, const int16_t *pk, int n, bd_read_t *out);

#endif
