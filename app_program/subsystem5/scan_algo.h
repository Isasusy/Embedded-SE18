/*
 *	scan_algo.h / scan_algo.c -- Obstacle Scan logic: turns one servo sweep into obstacle_t
 *	(width, edges, assumed depth = width, free space each side, chosen side, planned bypass). No hardware: testable on a PC.
 */
#ifndef SCAN_ALGO_H
#define SCAN_ALGO_H

#include "vc_msgs.h"

#define SCAN_MAX_RAYS	32

/* angles in degrees ASCENDING (most right first), + = left, evenly spaced; dist_mm < 0 = no echo (treated as clear).
   scan_analyze_from: the obstacle is grown from the ray nearest seed_deg (the angle that stopped the car; a neighbour
   if that reading dropped out; straight ahead as the last resort). scan_analyze = scan_analyze_from(..., 0, ...). */
void scan_analyze(const int *angle_deg, const int *dist_mm, int n, obstacle_t *out);
void scan_analyze_from(const int *angle_deg, const int *dist_mm, int n, int seed_deg, obstacle_t *out);

/* free space a side needs for its planned route, mm from the scanned edge, or -1 if no angle fits (console only) */
float scan_route_need(const obstacle_t *o, int side);

/* true if an echo of dist_mm at angle_deg (+ = left) is an obstacle in the car's path: closer than close_mm AHEAD and
   no more than half_w_mm to either side of the path centre line (so a wall beside the track does not stop the car) */
bool scan_in_path(int angle_deg, int dist_mm, int close_mm, int half_w_mm);

#endif
