#include "s5_math.h"
#include <stdlib.h>
#include "scan_algo.h"
#include "vc_config.h"

#define DEG2RAD	0.01745329f

static bool is_near(int d) { return d >= 0 && d < SCAN_NEAR_MM; }
static float pos(float x) { return x > 0.0f ? x : 0.0f; }

bool scan_in_path(int angle_deg, int dist_mm, int close_mm, int half_w_mm)
{
	if (dist_mm <= 0) return false;			/* no echo */
	float a = (float)angle_deg * DEG2RAD, d = (float)dist_mm;
	return d * mm_cosf(a) < (float)close_mm && mm_fabsf(d * mm_sinf(a)) <= (float)half_w_mm;
}

/* Free space beside the obstacle on one side (dir +1 = left, -1 = right), walking outward from ray `start`.
   edge = sideways distance of the obstacle's edge on that side (+ = away from the centre line, toward that side).
   A ray is blocked if its echo lies within `reach` mm ahead (inside the strip the car drives through beside the
   obstacle, whose depth is assumed); the free space then ends at that echo's sideways position. If no ray is blocked up
   to the end of the sweep, the free space is what the sweep could see at the obstacle's range, and *open_end is set. */
static float side_clear(const int *ang, const int *dist, int n, int start, int dir, float step, float d, float reach,
			float edge, bool *open_end)
{
	int last = -1;

	*open_end = false;
	for (int k = start; k >= 0 && k < n; k += dir) {
		if (dist[k] > 0) {
			float a = (float)ang[k] * DEG2RAD, r = (float)dist[k];
			if (r * mm_cosf(a) < reach)
				return pos((float)dir * r * mm_sinf(a) - edge);
		}
		last = k;
	}
	if (last < 0) return 0.0f;			/* the obstacle fills the sweep on this side */
	*open_end = true;
	return pos((float)dir * d * mm_tanf(((float)ang[last] + (float)dir * 0.5f * step) * DEG2RAD) - edge);
}

/* The avoidance route, from the axle (turning centre) where the car stopped. y = forward, x = sideways toward the
   chosen side. Clearance comes only from the planned obstacle: SCAN_PLAN_WIDTH_PCT of the SCANNED width (the full
   extent of the rays that echoed), centred on it, as deep as it is wide, front face where it was measured.
   Its near corner on that side is at (ep, Y), Y = distance + VC_SENSOR_AHEAD_MM, its back face at Y + depth. The car
   is h = VC_ROBOT_HALF_W_MM either side of its path, and spinning on the spot it sweeps a circle of radius R through
   its farthest corner (front or rear overhang, and h).
   1. Turn by t: the heading must aim OUTSIDE the planned obstacle as the sensor saw it: the car's centre line passes
      ep + h sideways at the obstacle's distance d, so t >= atan((ep + h) / d). The car's side must also pass the planned
      corner from the axle (Y*tan(t) - ep >= h / cos(t)). Whole degrees, rounded up, at most VC_AVOID_MAX_DEG.
   2. Drive the angled leg until the car is PAST the obstacle: the axle ends R beyond the planned back face
      (y_end = Y + depth + R), so the whole car has passed it and can spin back without touching it.
   3. Turn back by -t: parallel to the original path, beyond the obstacle. No further drive-on is needed (pass_mm 0).
   4. Line finding: turn -t toward the line and creep until a sensor finds it. */
typedef struct { int32_t turn_deg, diag_mm, lat_mm, pass_mm; float need_mm; } plan_t;

/* turning circle of the car about its axle: its farthest corner */
static float spin_radius(void)
{
	const float h = (float)VC_ROBOT_HALF_W_MM, front = (float)VC_SENSOR_AHEAD_MM;
	const float rear = (float)(VC_ROBOT_LEN_MM - VC_SENSOR_AHEAD_MM);
	float over = front > rear ? front : rear;
	return mm_sqrtf(over * over + h * h);
}

/* scanned edge on that side, sideways from the centre line toward it (the widest the sensor saw the obstacle) */
static float scan_edge(const obstacle_t *o, int side)
{
	return side > 0 ? o->scan_left_mm : -o->scan_right_mm;
}

/* planned (enlarged) edge on that side: SCAN_PLAN_WIDTH_PCT of the SCANNED width, centred on the scanned obstacle */
static float plan_edge(const obstacle_t *o, int side)
{
	float centre = 0.5f * (o->scan_left_mm + o->scan_right_mm);
	float half = 0.5f * (o->scan_left_mm - o->scan_right_mm) * (float)SCAN_PLAN_WIDTH_PCT / 100.0f;
	return side > 0 ? centre + half : -(centre - half);
}

static int ceil_deg(float rad)
{
	float t = rad / DEG2RAD;
	int deg = (int)t;
	if ((float)deg < t) deg++;
	return deg;
}

/* from the axle, a leg at deg degrees keeps the car's side (h from its centre line) clear of the corner (edge, Y) */
static bool corner_clear(float Y, float edge, int deg)
{
	float tr = (float)deg * DEG2RAD;
	return Y * mm_tanf(tr) - edge >= (float)VC_ROBOT_HALF_W_MM / mm_cosf(tr);
}

static plan_t scan_plan(const obstacle_t *o, int side)
{
	const float h = (float)VC_ROBOT_HALF_W_MM, R = spin_radius();
	float edge = pos(plan_edge(o, side));			/* planned edge */
	float real = pos(scan_edge(o, side));			/* scanned edge (free space starts here) */
	float d = o->distance_mm;
	float Y = d + (float)VC_SENSOR_AHEAD_MM;		/* forward from the axle to the obstacle's face */
	plan_t p;

	/* aim outside the planned obstacle as the sensor saw it, then make sure the side clears the corner */
	int aim = ceil_deg(mm_atan2f(edge + h, d));
	int deg = aim < 1 ? 1 : (aim > VC_AVOID_MAX_DEG ? VC_AVOID_MAX_DEG : aim);
	while (deg < VC_AVOID_MAX_DEG && !corner_clear(Y, edge, deg)) deg++;
	bool fits = aim <= VC_AVOID_MAX_DEG && corner_clear(Y, edge, deg);	/* else too wide even at the steepest */
	float tr = (float)deg * DEG2RAD;
	float y_end = Y + o->depth_mm + R;			/* the whole car past the back face, room to spin */
	float x = y_end * mm_tanf(tr);

	p.turn_deg = side * deg;
	p.diag_mm = (int32_t)(y_end / mm_cosf(tr) + 0.5f);
	p.lat_mm = (int32_t)(x + 0.5f);
	p.pass_mm = 0;
	p.need_mm = fits ? x + R - real : 1e9f;	/* free space from the scanned edge to where the far corner swings */
	return p;
}

float scan_route_need(const obstacle_t *o, int side)
{
	plan_t p = scan_plan(o, side);
	return p.need_mm >= 1e8f ? -1.0f : p.need_mm;
}

static int nearest_ray(const int *ang, int n, int deg)
{
	int c = 0;
	for (int i = 1; i < n; i++)
		if (abs(ang[i] - deg) < abs(ang[c] - deg)) c = i;
	return c;
}

void scan_analyze(const int *ang, const int *dist, int n, obstacle_t *o)
{
	scan_analyze_from(ang, dist, n, 0, o);
}

/* Angles must be ascending (index 0 = most right, last = most left). */
void scan_analyze_from(const int *ang, const int *dist, int n, int seed_deg, obstacle_t *o)
{
	*o = (obstacle_t){0};
	if (n < 3 || n > SCAN_MAX_RAYS) return;

	/* the obstacle is the one that stopped the car: start from the ray nearest seed_deg (the angle the watch sweep hit
	   it at), or a neighbour if that one reading dropped out, and only then from straight ahead */
	int c = nearest_ray(ang, n, seed_deg);
	if (!is_near(dist[c])) {
		if (c + 1 < n && is_near(dist[c + 1]))
			c = c + 1;
		else if (c - 1 >= 0 && is_near(dist[c - 1]))
			c = c - 1;
		else if (!is_near(dist[c = nearest_ray(ang, n, 0)]))
			return;
	}

	int lft = c, rgt = c;
	while (lft + 1 < n && is_near(dist[lft + 1])) lft++;
	while (rgt - 1 >= 0 && is_near(dist[rgt - 1])) rgt--;

	int dmin = dist[c];
	for (int i = rgt; i <= lft; i++) if (dist[i] < dmin) dmin = dist[i];

	/* edges: half a step beyond the outermost hit rays, then pulled in by the beam's half-width (a wide beam sees the
	   obstacle from rays that do not point at it) */
	float step = (float)(ang[1] - ang[0]);
	float degL = (float)ang[lft] + 0.5f * step - (float)SCAN_BEAM_HALF_DEG;
	float degR = (float)ang[rgt] - 0.5f * step + (float)SCAN_BEAM_HALF_DEG;
	if (degL < degR) degL = degR = 0.5f * (degL + degR);
	float d = (float)dmin;

	o->found = true;
	o->distance_mm = d;
	o->left_edge_mm = d * mm_tanf(degL * DEG2RAD);
	o->right_edge_mm = d * mm_tanf(degR * DEG2RAD);
	o->width_mm = o->left_edge_mm - o->right_edge_mm;
	/* scanned edges: the full extent of the rays that echoed. The route uses these, not the beam-corrected ones, so a
	   sensor whose beam is narrower than SCAN_BEAM_HALF_DEG cannot make the car turn too little */
	o->scan_left_mm = d * mm_tanf(((float)ang[lft] + 0.5f * step) * DEG2RAD);
	o->scan_right_mm = d * mm_tanf(((float)ang[rgt] - 0.5f * step) * DEG2RAD);
	float wp = (o->scan_left_mm - o->scan_right_mm) * (float)SCAN_PLAN_WIDTH_PCT / 100.0f;	/* planned width = depth */
	o->depth_mm = wp < SCAN_DEPTH_MIN_MM ? SCAN_DEPTH_MIN_MM : (wp > SCAN_DEPTH_MAX_MM ? SCAN_DEPTH_MAX_MM : wp);

	float reach = d + o->depth_mm;			/* the strip beside the obstacle runs to its planned back face */
	bool l_open, r_open;
	o->left_clear_mm = side_clear(ang, dist, n, lft + 1, +1, step, d, reach, o->scan_left_mm, &l_open);
	o->right_clear_mm = side_clear(ang, dist, n, rgt - 1, -1, step, d, reach, -o->scan_right_mm, &r_open);

	/* a side is usable if nothing echoed on it right out to the sweep's edge (the sweep cannot see further, so its
	   reach does not limit the route), or if its free space up to the first echo holds that side's route. Of two usable
	   sides the more open one; if the two are about equal (or both open to the end of the sweep) the one whose edge is
	   nearer the centre line, so the detour is shorter */
	plan_t pl = scan_plan(o, +1), pr = scan_plan(o, -1);
	bool l_ok = l_open || o->left_clear_mm >= pl.need_mm, r_ok = r_open || o->right_clear_mm >= pr.need_mm;
	if (!l_ok && !r_ok)
		o->side = 0;
	else if (l_ok != r_ok)
		o->side = l_ok ? +1 : -1;
	else if ((l_open && r_open) || mm_fabsf(o->left_clear_mm - o->right_clear_mm) <= SCAN_SIDE_TIE_MM)
		o->side = (o->scan_left_mm <= -o->scan_right_mm) ? +1 : -1;
	else
		o->side = (o->left_clear_mm > o->right_clear_mm) ? +1 : -1;

	if (o->side != 0) {
		const plan_t *p = o->side > 0 ? &pl : &pr;
		o->turn_deg = p->turn_deg;
		o->diag_mm = p->diag_mm;
		o->lat_mm = p->lat_mm;
		o->pass_mm = p->pass_mm;
	}
}
