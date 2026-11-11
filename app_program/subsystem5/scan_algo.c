#include "s5_math.h"
#include <stdlib.h>
#include "scan_algo.h"
#include "vc_config.h"

#define DEG2RAD	0.01745329f

static bool is_near(int d) { return d >= 0 && d < SCAN_NEAR_MM; }

bool scan_in_path(int angle_deg, int dist_mm, int close_mm, int half_w_mm)
{
	if (dist_mm <= 0) return false;			/* no echo */
	float a = (float)angle_deg * DEG2RAD, d = (float)dist_mm;
	return d * mm_cosf(a) < (float)close_mm && mm_fabsf(d * mm_sinf(a)) <= (float)half_w_mm;
}

/* Angles must be ascending (index 0 = most right, last = most left). */
void scan_analyze(const int *ang, const int *dist, int n, obstacle_t *o)
{
	*o = (obstacle_t){0};
	if (n < 3 || n > SCAN_MAX_RAYS) return;

	int c = 0;					/* the ray closest to straight ahead must see the obstacle */
	for (int i = 1; i < n; i++)
		if (abs(ang[i]) < abs(ang[c])) c = i;
	if (!is_near(dist[c])) return;

	int lft = c, rgt = c;				/* grow while rays stay near */
	while (lft + 1 < n && is_near(dist[lft + 1])) lft++;
	while (rgt - 1 >= 0 && is_near(dist[rgt - 1])) rgt--;

	int dmin = dist[c];
	for (int i = rgt; i <= lft; i++) if (dist[i] < dmin) dmin = dist[i];

	float step = (float)(ang[1] - ang[0]);
	float aL = ((float)ang[lft] + 0.5f * step) * DEG2RAD;	/* left edge of the obstacle  */
	float aR = ((float)ang[rgt] - 0.5f * step) * DEG2RAD;	/* right edge of the obstacle */
	float d = (float)dmin;

	o->found = true;
	o->distance_mm = d;
	o->width_mm = d * (mm_tanf(aL) - mm_tanf(aR));

	/* clearance = sideways distance, at the obstacle's range, covered by consecutive clear rays beside it */
	int k = lft + 1, last = -1;
	while (k < n && !is_near(dist[k])) { last = k; k++; }
	o->left_clear_mm = (last >= 0) ? d * (mm_tanf(((float)ang[last] + 0.5f * step) * DEG2RAD) - mm_tanf(aL)) : 0.0f;

	k = rgt - 1; last = -1;
	while (k >= 0 && !is_near(dist[k])) { last = k; k--; }
	o->right_clear_mm = (last >= 0) ? d * (mm_tanf(aR) - mm_tanf(((float)ang[last] - 0.5f * step) * DEG2RAD)) : 0.0f;

	if (o->left_clear_mm < 0) o->left_clear_mm = 0;
	if (o->right_clear_mm < 0) o->right_clear_mm = 0;

	float best = o->left_clear_mm >= o->right_clear_mm ? o->left_clear_mm : o->right_clear_mm;
	if (best < SCAN_MIN_CLEAR_MM)
		o->side = 0;
	else
		o->side = (o->left_clear_mm >= o->right_clear_mm) ? +1 : -1;
}
