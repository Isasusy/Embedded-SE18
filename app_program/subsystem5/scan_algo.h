/*
 *	scan_algo.h / scan_algo.c -- Obstacle Scan logic: turns one servo sweep into obstacle_t
 *	(width, clearance each side, chosen side). No hardware: testable on a PC.
 */
#ifndef SCAN_ALGO_H
#define SCAN_ALGO_H

#include "vc_msgs.h"

#define SCAN_MAX_RAYS	32

/* angles in degrees ASCENDING (most right first), + = left, evenly spaced; dist_mm < 0 = no echo (treated as clear) */
void scan_analyze(const int *angle_deg, const int *dist_mm, int n, obstacle_t *out);

/* true if an echo of dist_mm at angle_deg (+ = left) is an obstacle in the car's path: closer than close_mm AHEAD and
   no more than half_w_mm to either side of the path centre line (so a wall beside the track does not stop the car) */
bool scan_in_path(int angle_deg, int dist_mm, int close_mm, int half_w_mm);

#endif
