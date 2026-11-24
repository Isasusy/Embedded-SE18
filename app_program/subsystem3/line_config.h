/*
 *	line_config.h -- every tunable number of subsystem 3 (line following + barcode).
 */
#ifndef LINE_CONFIG_H
#define LINE_CONFIG_H

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
/* Ramped steering (23 Nov: the car over-corrected and weaved after every drift). While ONE sensor touches the line the
   correction starts gentle and grows with the time it has been touching: a small drift is corrected softly and released
   before it overshoots; on a curve the sensor stays on the line and the steer builds up to LINE_STEER_MAX.
   The build-up fades (LINE_STEER_DECAY) rather than resets while both sensors are white, so a curve that touches the
   line again and again keeps its steer. All times are REAL ms (the task runs every 10-20 ms, not LINE_PERIOD_MS). */
#define LINE_STEER_START	40.0f		/* mm/s of wheel-speed difference the moment a sensor touches the line */
#define LINE_STEER_RAMP		600.0f		/* ... growing by this many mm/s per second while it stays on the line */
#define LINE_STEER_MAX		180.0f		/* ... up to this (keep below MC_STEER_MAX 200) */
#define LINE_STEER_DECAY	2.0f		/* the build-up time fades this many times faster than it grew, when both are white */
#define LINE_STEER_MIN		LINE_STEER_START	/* (old name) */
#define LINE_STEER_HOLD_MS	120
#define LINE_STEER_HOLD_GAIN	0.3f		/* after the sensor leaves the line keep this fraction of the steer, fading to 0
						   over LINE_STEER_HOLD_MS (was 0.7 / 200 "5 ms" samples = 0.4-0.8 s real) */
#define LINE_FOUND_QUIET_MS	100		/* both white this long before a black sensor counts as "line found" */
#define LINE_FOUND_HOLDOFF_MS	500		/* at most one LINE_FOUND per this time */

/* ---- barcode (FR2): Code 39, read by IR3 on GP28 ----
 * The course barcodes are assumed to be Code 39 (the usual robot-car format): every character is 9 elements
 * (5 bars, 4 spaces, 3 of them wide) between the start/stop character '*', e.g. "*A*".
 * Read in either direction. Widths are compared within each character, so the robot's speed does not matter.
 * Every read is printed:  [S3] >> BARCODE "A" -> LEFT   and the raw wide/narrow pattern, also when it fails.
 * Command letters (briefing): A = turn left, B = turn right, C = go straight, D = U-turn. */
#define BAR_MIN_US		3000		/* an element shorter than this is a glitch and is merged away (GP28 polled every 1 ms) */
/* No barcode SIZE is assumed anywhere: every decision compares stripes with each other (Code 39: wide = 2-3 x narrow),
   so any print size and any speed work. Only these two absolute limits remain, and they are far beyond any bar: */
#define BAR_MAX_US		2000000		/* a DARK stripe longer than this (180 mm at 90 mm/s) is the line or a junction */
#define BAR_MAX_WEAK_US		3000000		/* any black longer than this: read dropped */
#define BAR_LINE_PEAK		1400		/* "dark" = the stripe's peak reading is at least this (thick bars and the line ~2300+) */
#define BAR_DIP_MIN		1000		/* a "white" that never got lighter than this is only a dip inside one dark bar */
#define BAR_LEN_SPLIT_RATIO	1.5f		/* bars are split into thin / thick where their sorted durations jump by at least
						   this factor (Code 39: wide = 2-3 x narrow; 12 Nov bench: 140 -> 380 ms = 2.7x) */
#define BAR_DIP_LEVEL_PCT	75		/* a "space" whose lightest reading stays this far (%) from white toward the dark
						   bars is a flicker inside one bar and is merged into it (12 Nov: dips ~3000-3500,
						   real spaces ~200, bars ~3700) */
#define BAR_ALLOW_BAD_GUARD	1		/* 1 = accept a code when ONE of its two '*' is unreadable but the other '*' and the
						   letter are valid (12 Nov: the first '*' was misread while the car steered) */
#define S3_BARCODE_DUMP		1		/* 1 = after a read that does not decode, print every stripe's duration and peak,
						   so the real course barcode can be worked out from the log on the day */
#define BAR_PEAK_SPLIT		600		/* thin and thick bars are told apart by how dark they get (thin ~300, thick ~2400)
						   when the darkest and lightest bar differ by at least this much */
#define BAR_END_FACTOR		4		/* the read ends when white lasts this many times the widest BLACK bar ... */
#define BAR_END_MIN_US		150000		/* ... and at least this long */
#define BAR_MAX_ELEMENTS	64		/* up to 6 characters including '*' and '*' */
#define BARCODE_BLACK_IS_HIGH	1		/* 1 = GP28 reads HIGHER over black (like the line sensors); 0 = lower */
#define BAR_ADC_THRESH		700		/* only for the very first sample: is the sensor on black or white?    */
/* The course barcode is Code 39 "*X*", 141 mm long: narrow = 3 mm, wide = 9 mm. The IR spot is wider than 3 mm,
   so a narrow bar or gap never reaches full black or full white. Instead of a fixed threshold the reading is
   followed like a peak detector: BLACK starts when it rises BAR_SWING above the lowest value since the last
   change, WHITE starts when it falls BAR_SWING below the highest. Small blurred swings still count. */
#define BAR_SWING		70 		/* ADC counts; raise if the stripe trace shows noise, lower if bars are missed */

/* The course barcode sits BESIDE the line (20 mm gap), so the line sensors never see its bars: both line sensors
   black is always a junction (or the car lifted, below). A junction that arrives while a barcode read is still open
   ends that read at once, so its command is known before the car acts on the junction. */
#define LINE_LIFTED_ADC		3200		/* both line sensors above this (no floor under them) = car lifted, not a junction */
#ifndef LINE_LIFTED_MS
#define LINE_LIFTED_MS		200		/* ... for this long = FLG_LINE_LIFTED set (VC stops the car); the same time back on
						   the floor clears it. Must be longer than the car takes to cross a junction's black
						   tape, in case black ever reads above LINE_LIFTED_ADC. 0 = lift detection off */
#endif
#define S3_BARCODE_TRACE	1		/* 1 = print every stripe the barcode sensor sees (colour and ms); 0 later */

/* Start-up defaults only: the dashboard can change any letter's direction over MQTT (barcode_algo.h, bd_map_*) */
#define BAR_CHAR_LEFT		'A'
#define BAR_CHAR_RIGHT		'B'
#define BAR_CHAR_STRAIGHT	'C'
#define BAR_CHAR_UTURN		'D'

#define S3_DEBUG_PRINT		1

#endif
