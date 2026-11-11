/*
 *	line_config.h -- every tunable number of subsystem 3 (line following + barcode).
 */
#ifndef LINE_CONFIG_H
#define LINE_CONFIG_H

/* ---- line following (FR1, NFR1: 5 ms) ---- */
#define LINE_PERIOD_MS		5
#define LINE_CAL_MS		2000		/* calibration window: rock the robot across the line by hand */
#define LINE_CAL_MIN_RANGE	400		/* ADC counts (0..4095); smaller spread = calibration rejected, defaults used */
/* defaults if calibration is rejected; measured on the robot: white floor about 150-190, black line about 2000-2100 */
#define LINE_DEF_WHITE		200
#define LINE_DEF_BLACK		2000
#define LINE_BLACK_IS_HIGH	1		/* 1 = ADC reading rises over black (TCRT5000 analog module) */
#define LINE_ON_THRESH		0.5f		/* normalised reading (0 white .. 1 black) counted as "on black" */
#define LINE_STEER_GAIN		240.0f		/* mm/s of wheel-speed difference per unit of offset (-1..+1) */
#define LINE_JUNCTION_MS	30		/* both sensors black this long = junction */
#define LINE_LOST_MS		250		/* LINE_STRADDLE 0 only: both sensors white this long = line lost */

/* Sensor geometry (track briefing: the two IR sensors sit just OUTSIDE the black line, one each side of it).
 *   LINE_STRADDLE 1: both sensors white = on course, driving straight. A sensor that turns black means the car has
 *                    drifted, so it steers back (toward the sensor that sees black when LINE_STEER_TOWARD_BLACK is 1).
 *                    Both white is NORMAL, so LINE_LOST is never raised; LINE_FOUND is raised when a sensor first
 *                    sees black after a quiet stretch (used by the obstacle bypass / line search).
 *   LINE_STRADDLE 0: the line runs under one or both sensors (both white = line lost).
 * LINE_STEER_TOWARD_BLACK: 1 = ONE line between the sensors (steer toward the side that sees black).
 *                          0 = the sensors sit outside two lane edges (steer away from the side that sees black).
 *                          If the car turns the wrong way when only one sensor sees black, flip this. */
#define LINE_STRADDLE		1
#define LINE_STEER_TOWARD_BLACK	1
/* Curves: the line only grazes a sensor, so the proportional steer alone is too weak and the car runs wide.
   LINE_STEER_MIN: as soon as ONE sensor touches the line (above LINE_STEER_ENGAGE), steer at least this hard (mm/s of wheel-speed difference).
   LINE_STEER_HOLD_*: when the sensor leaves the line again, keep steering the same way for a moment, fading out,
   so a curve is followed smoothly instead of going straight the instant the sensor goes white. */
#define LINE_STEER_ENGAGE	0.25f		/* a sensor reading above this (0 white .. 1 black) counts as touching the line for steering */
#define LINE_STEER_MIN		110.0f
#define LINE_STEER_HOLD_MS	200
#define LINE_STEER_HOLD_GAIN	0.7f		/* hold this fraction of the last steer, fading to 0 over LINE_STEER_HOLD_MS */
#define LINE_FOUND_QUIET_MS	100		/* both white this long before a black sensor counts as "line found" */
#define LINE_FOUND_HOLDOFF_MS	500		/* at most one LINE_FOUND per this time */

/* ---- barcode (FR2): Code 39, read by IR3 on GP28 ----
 * The course barcodes are assumed to be Code 39 (the usual robot-car format): every character is 9 elements
 * (5 bars, 4 spaces, 3 of them wide) between the start/stop character '*', e.g. "*A*".
 * Read in either direction. Widths are compared within each character, so the robot's speed does not matter.
 * Every read is printed:  [S3] >> BARCODE "A" -> LEFT   and the raw wide/narrow pattern, also when it fails.
 * Command letters (briefing): A = turn left, B = turn right, C = go straight, D = U-turn. */
#define BAR_MIN_US		3000		/* an element shorter than this is a glitch and is merged away (GP28 polled every 1 ms) */
#define BAR_MAX_US		400000		/* a bar longer than this is not a barcode (the line, a junction): read dropped */
#define BAR_END_FACTOR		4		/* the read ends when white lasts this many times the widest element ... */
#define BAR_END_MIN_US		150000		/* ... and at least this long */
#define BAR_MAX_ELEMENTS	64		/* up to 6 characters including '*' and '*' */
#define BARCODE_BLACK_IS_HIGH	1		/* 1 = GP28 reads HIGHER over black (like the line sensors); 0 = lower */
#define BAR_ADC_THRESH		1000		/* GP28 is read by the ADC (0..4095): black/white split ...         */
#define BAR_ADC_HYST		150		/* ... with this much hysteresis either side (white ~200, black ~2000) */

#define BAR_CHAR_LEFT		'A'
#define BAR_CHAR_RIGHT		'B'
#define BAR_CHAR_STRAIGHT	'C'
#define BAR_CHAR_UTURN		'D'

#define S3_DEBUG_PRINT		1		/* print events and a 2 Hz line; 0 at integration */

#endif
