/*
 *	s4_fmt.h
 *	Float -> text without needing "%f" support in the kernel's printf
 *	(small embedded printf implementations often leave it out).
 *	Pure C, host-testable.
 */
#ifndef S4_FMT_H
#define S4_FMT_H

/* Writes v with `dec` decimals (0..3) into b (needs >= 16 bytes). Returns b.
   Example: s4_fx(buf, -12.34f, 1) -> "-12.3" */
static inline char *s4_fx(char *b, float v, int dec)
{
	long scale = 1;
	for (int i = 0; i < dec; i++) scale *= 10;

	long n = (long)(v * (float)scale + (v < 0.0f ? -0.5f : 0.5f));
	char *p = b;
	unsigned long u;
	if (n < 0) { *p++ = '-'; u = (unsigned long)(-n); } else { u = (unsigned long)n; }

	unsigned long ip = u / (unsigned long)scale;
	unsigned long fp = u % (unsigned long)scale;

	char tmp[12];
	int t = 0;
	do { tmp[t++] = (char)('0' + (ip % 10)); ip /= 10; } while (ip && t < 11);
	while (t) *p++ = tmp[--t];

	if (dec) {
		*p++ = '.';
		unsigned long d = (unsigned long)scale / 10;
		for (int i = 0; i < dec; i++) { *p++ = (char)('0' + ((fp / d) % 10)); d = d ? d / 10 : 0; if (!d) d = 1; }
	}
	*p = '\0';
	return b;
}

#endif /* S4_FMT_H */
