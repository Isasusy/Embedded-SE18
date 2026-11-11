/*
 *	s5_math.h -- the few float maths functions this subsystem needs, written out so the firmware does not depend on
 *	the C math library (the kernel build does not link libm). Accuracy ~1e-5, plenty for these uses.
 *	Valid for the ranges used here (angles within a few radians, finite floats).
 */
#ifndef S5_MATH_H
#define S5_MATH_H

#define MM_PI		3.14159265358979f
#define MM_HALF_PI	1.57079632679490f

static inline float mm_fabsf(float x) { return x < 0.0f ? -x : x; }

static inline float mm_sqrtf(float x)
{
	union { float f; unsigned int u; } v;
	float y;
	int i;

	if (!(x > 0.0f)) return 0.0f;
	v.f = x;
	v.u = 0x1FBD1DF5u + (v.u >> 1);		/* first guess: halve the exponent */
	y = v.f;
	for (i = 0; i < 4; i++)			/* Newton-Raphson */
		y = 0.5f * (y + x / y);
	return y;
}

/* sine for any reasonable angle in radians */
static inline float mm_sinf(float x)
{
	float x2;

	while (x >  MM_PI) x -= 2.0f * MM_PI;
	while (x < -MM_PI) x += 2.0f * MM_PI;
	if (x >  MM_HALF_PI) x =  MM_PI - x;		/* fold into -pi/2 .. pi/2 */
	if (x < -MM_HALF_PI) x = -MM_PI - x;
	x2 = x * x;
	return x * (1.0f + x2 * (-1.0f / 6.0f + x2 * (1.0f / 120.0f + x2 * (-1.0f / 5040.0f + x2 * (1.0f / 362880.0f)))));
}

static inline float mm_cosf(float x) { return mm_sinf(x + MM_HALF_PI); }

static inline float mm_tanf(float x)
{
	float c = mm_cosf(x);
	if (mm_fabsf(c) < 1e-6f) c = (c < 0.0f) ? -1e-6f : 1e-6f;
	return mm_sinf(x) / c;
}

/* arctangent, |error| < 1e-4 rad */
static inline float mm_atanf(float x)
{
	float a = mm_fabsf(x), r, a2;
	int inv = a > 1.0f;

	if (inv) a = 1.0f / a;
	a2 = a * a;
	r = a * (0.9998660f + a2 * (-0.3302995f + a2 * (0.1801410f + a2 * (-0.0851330f + a2 * 0.0208351f))));
	if (inv) r = MM_HALF_PI - r;
	return x < 0.0f ? -r : r;
}

static inline float mm_atan2f(float y, float x)
{
	if (x > 0.0f) return mm_atanf(y / x);
	if (x < 0.0f) return y >= 0.0f ? mm_atanf(y / x) + MM_PI : mm_atanf(y / x) - MM_PI;
	if (y > 0.0f) return MM_HALF_PI;
	if (y < 0.0f) return -MM_HALF_PI;
	return 0.0f;
}

#endif
