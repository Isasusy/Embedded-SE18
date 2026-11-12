/*
 *	vc_config.h -- every tunable number of subsystem 5 (Vehicle Controller + obstacle scan).
 */
#ifndef VC_CONFIG_H
#define VC_CONFIG_H

/* ---- Vehicle Controller ---- */
#define VC_TICK_MS		20		/* event-driven plus 20 ms tick (task table) */
/* 1 = BENCH LINE FOLLOW: creep along the line at VC_BENCH_SPEED, continuously, and do nothing else: barcodes, junctions,
   humps and the IMU are ignored (barcodes still work if VC_BENCH_BARCODES is 1). The car stops only for: an OBSTACLE seen by the ultrasonic sensor (it stops, sweeps the
   servo left/centre/right, prints the profile, and drives on again when the way has been clear for VC_BENCH_CLEAR_MS),
   a remote STOP, a stalled wheel or a dead subsystem. 0 = the full mission (set 0 at integration). */
#ifndef VC_BENCH_LINE_ONLY
#define VC_BENCH_LINE_ONLY	1
#endif
#define VC_BENCH_BARCODES	1		/* bench: still act on barcodes: remember one, turn at the next junction */
#define VC_BENCH_SPEED		90		/* mm/s: slow and steady */
#define VC_BENCH_RETRY_MS	1500		/* bench: after a stalled-wheel fault, reset and drive on again after this long */
#define VC_BENCH_CLEAR_MS	800		/* bench: after an obstacle stop, resume once nothing has been close for this long */
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
#define VC_ALIVE_MS		100		/* NFR10: a task silent longer than this = fault */
#define VC_RECOVER_TIMEOUT_MS	4000
#define VC_HUMP_TIMEOUT_MS	6000
#define VC_SCAN_TIMEOUT_MS	6000
#define VC_SEARCH_TIMEOUT_MS	5000
#define VC_STATUS_MS		500		/* NFR5: telemetry at 2 Hz */
#define VC_SETTLE_TICKS		2		/* ignore cmd_done this long after sending a manoeuvre */

/* ---- obstacle bypass geometry (CALIBRATE on the robot) ---- */
#define VC_ROBOT_HALF_W_MM	80
#define VC_SIDE_MARGIN_MM	40
#define VC_OBST_DEPTH_MM	200		/* the ultrasonic sensor cannot measure depth: assumed from the track briefing */
#define VC_ROBOT_LEN_MM		150

/* ---- hardware watchdog (NFR10: 500 ms) ---- */
#define VC_WATCHDOG_ENABLE	0		/* 1 at integration; 0 on the bench so a stand-alone S5 does not reset */
#define VC_WATCHDOG_MS		500

/* ---- bench start: no MQTT needed ----
   0 = wait for a REMOTE_START (from S1/MQTT). N > 0 = start the mission by itself N ms after power-up, once,
   as soon as every subsystem reports READY. LIFT THE WHEELS OFF THE TABLE FIRST. Set 0 at integration. */
#define VC_AUTOSTART_MS		5000		/* only used when VC_START_BUTTON is 0 */

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
#define VC_REQUIRE_IMU		0
#endif

/* ---- Obstacle Scan ---- */
#define SCAN_PERIOD_MS		60		/* ping period while WATCHing (task table: 60 ms per ping) */
#define SCAN_CLOSE_MM		200		/* WATCH: closer than this twice in a row = obstacle close = STOP (adjust) */
#define SCAN_WATCH_DEG		0		/* centre of the watch sweep: 0 = straight ahead, + = left (adjust) */
#define SCAN_WATCH_SWEEP_DEG	30		/* WATCH: the servo sweeps continuously from centre-this to centre+this (0 = fixed, no sweep) */
#define SCAN_WATCH_STEP_DEG	5		/* ... in steps of this many degrees */
#define SCAN_WATCH_SETTLE_MS	45		/* ... waiting this long for the servo at each step before pinging */
#define SCAN_PATH_HALF_W_MM	120		/* an echo only counts as an obstacle if it is within this sideways distance of the path */
#define SCAN_TRACK_CLEAR_PINGS	4		/* after a hit the servo stays on the obstacle until this many clear pings in a row */
#define SCAN_NEAR_MM		400		/* a ray shorter than this belongs to the obstacle */
#define SCAN_MIN_CLEAR_MM	150		/* a side narrower than this cannot be used */
#define SCAN_ANGLE_MAX		40		/* sweep -40 .. +40 degrees (+ = left) */
#define SCAN_ANGLE_STEP		4
#define SCAN_SETTLE_MS		60		/* servo settling time per angle */
#define SERVO_US_MIN		500		/* SG90 pulse widths */
#define SERVO_US_MAX		2500
#define SERVO_TRIM_DEG		0		/* adjust if "straight ahead" is not 0 (try +-5, +-10 ...) */
#define SERVO_REVERSE		0		/* 1 = swap left and right if + turns the sensor to the robot's right */
#define S5_SERVO_TEST		0		/* 1 = at power-up sweep the servo -60..+60..0 and print the distance at each angle,
						   to set SERVO_TRIM_DEG / SERVO_REVERSE. Set 0 when it points correctly. */
#define S5_ULTRA_PRINT		1		/* 2 Hz "[S5] ultra=... mm" line while watching; 0 at integration */

#define S5_DEBUG_PRINT		1
#endif
