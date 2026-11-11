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
	uint32_t	widest;
} bd_t;

typedef struct {
	char		text[8];			/* decoded characters without the '*'s, e.g. "A" */
	int		nav;				/* nav_cmd_type_t, or -1 if the first character is not a command */
	bool		reversed;			/* read back to front */
	int		n_elements;
	char		pattern[BAR_MAX_ELEMENTS + 12];	/* 'W' wide / 'n' narrow, a space between characters */
	uint32_t	narrow_us;			/* typical narrow element, for the log */
} bd_read_t;

typedef enum { BD_NONE, BD_VALID, BD_INVALID } bd_result_t;

void		bd_init(bd_t *b);
/* feed one edge; `black` = level after the edge. Never completes a read (the quiet zone after the code does). */
void		bd_edge(bd_t *b, uint32_t t_us, bool black);
/* call every ~1 ms: when the white after a code has lasted long enough, decodes it into *out.
   BD_VALID = a Code 39 code was read, BD_INVALID = something was read but did not decode (out->pattern says what). */
bd_result_t	bd_poll(bd_t *b, uint32_t now_us, bd_read_t *out);

/* decode a finished list of element widths (exposed for tests) */
bd_result_t	bd_decode(const uint32_t *w, int n, bd_read_t *out);

#endif
