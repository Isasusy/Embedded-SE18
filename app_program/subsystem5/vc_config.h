/*
 *	vc_config.h -- every tunable number of subsystem 5 (Vehicle Controller + obstacle scan).
 */
#ifndef VC_CONFIG_H
#define VC_CONFIG_H

#define VC_TICK_MS		20		/* event-driven plus 20 ms tick (task table) */
/* 1 = BENCH LINE FOLLOW: creep along the line at VC_BENCH_SPEED, continuously, and do nothing else: barcodes, junctions,
   humps and the IMU are ignored (barcodes still work if VC_BENCH_BARCODES is 1). The car stops only for: an OBSTACLE seen by the ultrasonic sensor (it stops,
   drives on if the way clears within VC_OBST_STILL_MS, else profiles it and goes round it: see VC_OBST_BYPASS),
   a remote STOP, a stalled wheel or a dead subsystem. 0 = the full mission (set 0 at integration). */
#ifndef VC_BENCH_LINE_ONLY
#define VC_BENCH_LINE_ONLY	1
#endif
#define VC_BENCH_BARCODES	1		/* bench: still act on barcodes: remember one, turn at the next junction */
#define VC_BENCH_SPEED		90		/* mm/s: slow and steady */
#define VC_BENCH_RETRY_MS	1500		/* bench: after a stalled-wheel fault, reset and drive on again after this long */
#if VC_BENCH_LINE_ONLY
#define VC_CRUISE_SPEED		VC_BENCH_SPEED
#else
#define VC_CRUISE_SPEED		150		/* mm/s, normal line following            */
#endif
#define VC_HUMP_SPEED		90		/* mm/s while the IMU measures a hump     */
#define VC_RECOVER_SPEED	60		/* mm/s while searching for a lost line   */
#define VC_TURN_ANGLE		90		/* degrees for a LEFT / RIGHT barcode     */
#define VC_JUNCTION_ADVANCE_MM	60		/* after the sensors see a junction, drive this far (axle to the junction centre) before turning (CALIBRATE) */
#define VC_JUNCTION_PAUSE_MS	1000		/* at a junction: stop this long before acting on the barcode */
#define VC_JUNCTION_PAUSE_ALWAYS 1		/* 1 = also stop at junctions with no barcode waiting (shows that the junction
						   was seen; then drives straight on). 0 = only when a barcode is waiting */
#define VC_NAV_EXPIRE_MS	15000		/* a barcode command is dropped if no junction comes within this time (0 = never) */
#define VC_SELFTEST_MS		15000		/* INIT: all subsystems must report READY in this time. Long enough for the
						   IMU's calibration retries (up to 5 x (2 s + 0.5 s)) and S3's 2 s calibration */
/* INIT also waits for the MQTT link, so the dashboard is live before the car can be started. Only in builds with MQTT
   (make ... NET=lwip MQTT=1 defines TM_MQTT). If WiFi / MQTT is still not up VC_NET_WAIT_MS after everything else was
   READY, INIT gives up waiting and goes to IDLE anyway (the car still runs without the dashboard). */
#ifndef VC_WAIT_FOR_NET
#if defined(TM_MQTT) && TM_MQTT
#define VC_WAIT_FOR_NET		1
#else
#define VC_WAIT_FOR_NET		0
#endif
#endif
#define VC_NET_WAIT_MS		30000
#define VC_ALIVE_MS		100		/* NFR10: a task silent longer than this = fault */
#define VC_RECOVER_TIMEOUT_MS	4000
#define VC_HUMP_TIMEOUT_MS	6000
#define VC_SCAN_TIMEOUT_MS	9000
#define VC_STATUS_MS		500		/* NFR5: telemetry at 2 Hz */
#define VC_SETTLE_TICKS		2		/* ignore cmd_done this long after sending a manoeuvre */

/* ---- obstacle handling: STOP -> CONFIRM (still?) -> PROFILE sweep -> BYPASS -> rejoin the line ---- */
#ifndef VC_OBST_BYPASS
#define VC_OBST_BYPASS		1		/* 1 = go round a still obstacle (bench and mission). 0 = profile it, then
						   only wait until it has gone (the old bench behaviour) */
#endif
#define VC_OBST_STILL_MS	2000		/* an obstacle must stay in front, at a steady distance, this long before the
						   car profiles it and goes round it */
#define VC_OBST_STILL_TOL_MM	40		/* "steady": tracked distance stays within this band (else the 5 s restart) */
#define VC_OBST_CLEAR_MS	800		/* nothing close for this long while stopped = it has gone: drive on */

/* ---- obstacle bypass geometry (CALIBRATE on the robot) ---- */
#define VC_ROBOT_HALF_W_MM	100		/* half the car's width */
#define VC_ROBOT_LEN_MM		180		/* the car's length, front to back (rear overhang = this - VC_SENSOR_AHEAD_MM) */
#define VC_SENSOR_AHEAD_MM	70		/* wheel axle (turning centre) to the ultrasonic sensor at the front (CALIBRATE) */
#define VC_AVOID_MAX_DEG	75		/* steepest avoidance turn */
#define VC_REJOIN_EXTRA_MM	150		/* line finding: creep this much further than the way back before giving up */
#define VC_MOVE_MAX_MM		500		/* a longer avoidance MOVE is sent in pieces, so each one finishes well within
						   S2's MC_CMD_TIMEOUT_MS at VC_CRUISE_SPEED */

/* ---- hardware watchdog (NFR10: 500 ms) ---- */
#define VC_WATCHDOG_ENABLE	1		/* 1 at integration; 0 on the bench so a stand-alone S5 does not reset */
#define VC_WATCHDOG_MS		500

/* ---- bench start: no MQTT needed ----
   0 = wait for a REMOTE_START (from S1/MQTT). N > 0 = start the mission by itself N ms after power-up, once,
   as soon as every subsystem reports READY. LIFT THE WHEELS OFF THE TABLE FIRST. Set 0 at integration. */
#define VC_AUTOSTART_MS		5000

/* ---- start / stop button (Robo Pico GP20 or GP21) ----
   1 = the car waits in IDLE until a button is pressed and released, then starts VC_BUTTON_DELAY_MS later (hand clear).
   Pressing it while the car runs stops it; pressing again starts it again. */
#ifndef VC_START_BUTTON
#define VC_START_BUTTON		1
#endif
#define VC_BUTTON_DELAY_MS	1000

/* 1 = the mission needs a working IMU (normal). 0 = BENCH ONLY: ignore the IMU's READY/ALIVE so the motors and line
   sensors can be tested while the IMU is not wired yet. The robot then has no impact stop and no hump detection. */
#ifndef VC_REQUIRE_IMU
#define VC_REQUIRE_IMU		1
#endif

/* 1 = an IMU IMPACT (sudden shock, IMU_SHOCK_G) stops the car with FAULT_IMPACT. 0 = ignored: the IMU still detects and
   prints it (">> IMPACT"), the car keeps driving. Set 0 by the team (23 Nov): the car's own jolts kept stopping it. */
#ifndef VC_STOP_ON_IMPACT
#define VC_STOP_ON_IMPACT	0
#endif

/* Stop when the car is tipped over / held tilted (IMU, needs the IMU working) or picked up (line sensors).
   Both work in bench mode too. While tilted or lifted the car also refuses to start. 0 = ignore. */
#ifndef VC_STOP_ON_TILT
#define VC_STOP_ON_TILT		1
#endif
#ifndef VC_STOP_ON_LIFT
#define VC_STOP_ON_LIFT		1
#endif

#define SCAN_PERIOD_MS		60		/* ping period while WATCHing (task table: 60 ms per ping) */
#define SCAN_CLOSE_MM		200		/* WATCH: closer than this twice in a row = obstacle close = STOP (adjust; keep below SCAN_NEAR_MM) */
#define SCAN_HOLD_PCT		120		/* while the car is STOPPED for an obstacle it still counts as there out to this %
						   of SCAN_CLOSE_MM, on either of the two pings. Hysteresis: an obstacle that stopped
						   the car right at SCAN_CLOSE_MM must not flicker out of it (noise, a weak echo off
						   its edge) and make the car think it has gone and drive into it */
#define SCAN_HOLD_MM		(SCAN_CLOSE_MM * SCAN_HOLD_PCT / 100)
#define SCAN_WATCH_DEG		0		/* centre of the watch sweep: 0 = straight ahead, + = left (adjust) */
#define SCAN_WATCH_SWEEP_DEG	30		/* WATCH: the servo sweeps continuously from centre-this to centre+this (0 = fixed, no sweep) */
#define SCAN_WATCH_STEP_DEG	5		/* ... in steps of this many degrees */
#define SCAN_WATCH_SETTLE_MS	45		/* ... waiting this long for the servo at each step before pinging */
#define SCAN_PATH_HALF_W_MM	120		/* an echo only counts as an obstacle if it is within this sideways distance of the path */
#define SCAN_TRACK_CLEAR_PINGS	4		/* after a hit the servo stays on the obstacle until this many clear pings in a row */
#define SCAN_NEAR_MM		400		/* a ray shorter than this belongs to the obstacle */
#define SCAN_SIDE_TIE_MM	30		/* sides this close in free space count as equal: take the shorter detour */
#define SCAN_BEAM_HALF_DEG	15		/* HC-SR04 beam half-width: each edge is pulled in by this. Was 7: on the robot
						   (23 Nov) a box at 104 mm echoed from servo 45 to 125 deg (80 deg), i.e. a
						   beam of ~15 deg each side. The route is planned on the corrected width.
						   CALIBRATE: profile a box of known width; raise this if it reads too wide */
#ifndef SCAN_PLAN_WIDTH_PCT
#define SCAN_PLAN_WIDTH_PCT	150		/* the route is planned round an obstacle this % of the beam-corrected width,
						   centred on it, with the same depth. Was 200 (on the uncorrected width):
						   every real obstacle then needed > 2 m of free space and was never passed */
#endif
#define SCAN_SIDE_MARGIN_MM	40		/* gap between the car's side and the planned obstacle while passing it */
#define SCAN_DEPTH_MIN_MM	80		/* planned depth (= planned width) is clamped to this range */
#define SCAN_DEPTH_MAX_MM	400
/* PROFILE sweep (after the car has stopped for an obstacle). Angles are ABSOLUTE servo degrees, SCAN_SERVO_CENTRE_DEG
   = straight ahead, so 30..150 is 60 degrees either side. Below the centre = the robot's right, above = its left. */
#define SCAN_SERVO_CENTRE_DEG	90
#define SCAN_PROFILE_START_DEG	30
#define SCAN_PROFILE_END_DEG	150
#define SCAN_PROFILE_STEP_DEG	5		/* 30, 35, ... 150 = 25 pings */
#define SCAN_GLIDE_MS_PER_DEG	8		/* smooth servo moves: 1 degree every this many ms (125 deg/s) */
#define SCAN_SETTLE_MS		40		/* after the glide reaches an angle, wait this long before pinging */
#define SERVO_US_MIN		500		/* SG90 pulse widths */
#define SERVO_US_MAX		2500
#define SERVO_TRIM_DEG		(-25)		/* adjust if "straight ahead" is not 0 (+ = turn left / anticlockwise, - = right / clockwise).
						   -30: the horn pointed ~30 deg too far left */
#define SERVO_REVERSE		0		/* 1 = swap left and right if + turns the sensor to the robot's right */
#define S5_SERVO_TEST		0		/* 1 = at power-up sweep the servo -60..+60..0 and print the distance at each angle,
						   to set SERVO_TRIM_DEG / SERVO_REVERSE. Set 0 when it points correctly. */
#define S5_ULTRA_PRINT		1		/* 2 Hz "[S5] ultra=... mm" line while watching; 0 at integration */

#define S5_DEBUG_PRINT		1
#endif
