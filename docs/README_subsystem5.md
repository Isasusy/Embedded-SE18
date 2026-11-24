
## Barcode then junction (from the project write-up, 6 Nov)

A decoded barcode is only **remembered** (`nav_has`). The car keeps line following. At the next JUNCTION event it
drives `VC_JUNCTION_ADVANCE_MM` forward (so the axle is at the junction centre) and then turns: LEFT +90, RIGHT -90,
UTURN 180; STRAIGHT drives on. (A STOP command would complete the mission, but no barcode letter maps to STOP.) Every
junction first pauses `VC_JUNCTION_PAUSE_MS`; with `VC_JUNCTION_PAUSE_ALWAYS 1` (current) the car also pauses at a junction
with no barcode pending and then drives straight on (0 = such junctions are ignored). An unused command expires after
`VC_NAV_EXPIRE_MS`.

## Bench obstacle stop and servo adjustment (8 Nov)

`VC_BENCH_LINE_ONLY 1`: the car creeps along the line and stops ONLY for an obstacle. When the ultrasonic sensor reads
closer than `SCAN_CLOSE_MM` twice in a row the car stops, the servo sweeps left / centre / right and the profile is
printed (`[S5] scan: found= dist= width= left= right= side=`). It stays stopped while the obstacle is there and drives
on again after `VC_OBST_CLEAR_MS` of clear path (superseded 19 Nov: see "Still obstacle bypass" below). Adjusting the servo: `S5_SERVO_TEST 1` sweeps it at power-up and prints
the distance at each angle; fix `SERVO_TRIM_DEG` (centre) and `SERVO_REVERSE` (left/right) from that, then set it to 0.
`SCAN_WATCH_DEG` is the angle the sensor points at while driving.

## Continuous servo sweep (8 Nov)

While the car drives, the servo sweeps the ultrasonic sensor continuously left <-> right (`SCAN_WATCH_SWEEP_DEG` either
side of `SCAN_WATCH_DEG`, in `SCAN_WATCH_STEP_DEG` steps) and pings at every step. An echo counts as an obstacle if it is
closer than `SCAN_CLOSE_MM` ahead and within `SCAN_PATH_HALF_W_MM` of the path (a wall beside the track is ignored). On a
hit the car stops, the servo stays pointed at the obstacle, a full profile sweep is printed, and the car drives on once the
obstacle has gone. If every ping returns no echo the console prints `[S5] ultra: NO ECHO ...` (check wiring).

## Ultrasonic wiring search (8 Nov)

The ultrasonic driver now uses the GPIO registers directly. At power-up it checks the configured pins (`PIN_US_TRIG`,
`PIN_US_ECHO` in drv_scan.h); if nothing answers it tries the swapped pair and then every pair of free pins and prints
`[S5] ultrasonic FOUND: TRIG=GPx ECHO=GPy`. Put those numbers into drv_scan.h. The S2 status line now also prints `fault=`.

## Still obstacle bypass (19 Nov)

In bench and mission mode alike (`VC_OBST_BYPASS 1`), OBSTACLE_AVOID now runs these steps:

1. **STOP / CONFIRM**: the car stops, and the scan task keeps the servo on the obstacle and publishes its distance in
   `s5_track_mm`. If nothing is close for `VC_OBST_CLEAR_MS` (800 ms), the obstacle has gone and the car drives on. If it
   stays close for `VC_OBST_STILL_MS` (5 s) with its distance inside a `VC_OBST_STILL_TOL_MM` (40 mm) band, it is a
   still object. Something moving more than that restarts the 5 s. While the car is stopped for it (confirming,
   profiling, holding) the VC sets `FLG_SCAN_HOLD`, and the obstacle still counts as there out to `SCAN_HOLD_MM`
   (`SCAN_HOLD_PCT` = 120 % of `SCAN_CLOSE_MM`) on either of the two pings (`FLG_SCAN_PRESENT`). Without this
   hysteresis, an obstacle that stopped the car right at the stop distance flickered out of it and the car drove on
   into it. After a profile sweep, the servo is back on the obstacle before the result reaches the VC, so watching it
   again (no route on the bench) has no gap. When no side is usable, the console prints why
   (`[S5] no route: left ... free, needs ...`).
2. **PROFILE**: the 30..150 deg sweep. The obstacle is grown from the ray at the angle that stopped the car (the
   watch sweep's hit, `[S5] scan (from N deg)`): from a neighbour if that one reading dropped out, and from straight
   ahead only as the last resort. An obstacle to one side of the path is therefore found, not reported as
   `found=0` because the straight-ahead ray missed it. `scan_analyze_from()` gives both edges (`left_edge_mm`, `right_edge_mm`, + = left).
   Each edge is pulled in by the beam half-width `SCAN_BEAM_HALF_DEG`. It also gives the measured width.
   **Clearance comes from a planned obstacle, not from fixed margins.** The route is planned round an obstacle
   `SCAN_PLAN_WIDTH_PCT` (200 %) of the SCANNED width (`scan_left_mm`..`scan_right_mm`: half a step beyond the
   outermost rays that echoed, with no beam correction, so the widest the sensor saw it), centred on it. Its depth equals its planned width
   (`depth_mm`, clamped to `SCAN_DEPTH_MIN_MM..SCAN_DEPTH_MAX_MM`). At 200 % the car keeps half the obstacle's width
   clear on each side, and a full width behind it. So a wider obstacle gets more room.
3. **SIDE**: a side's free space ends at the first echo that is closer ahead than the planned back face
   (distance + planned depth). Its value is that echo's sideways position minus the measured edge. If no echo blocks
   it, the free space runs to the edge of the sweep. A side with no echo out to the sweep's edge is usable (the sweep
   cannot see further). A side with an echo is usable if its free space reaches the car's far corner during the route
   (`lat_mm` + turning circle − scanned edge). Of two usable sides the car takes the more open one.
   If the two are within `SCAN_SIDE_TIE_MM`, or both are open to the sweep's edge, it takes the side whose edge is
   nearer the centre line (the shorter detour).
4. **AVOID** (planned from the profile by `scan_plan()`):
   - **Turn** by `turn_deg`, aiming outside the planned obstacle as the sensor saw it. The car's centre line must pass
     the planned edge plus half the car's width at the obstacle's distance: `t ≥ atan((planned edge + VC_ROBOT_HALF_W_MM)
     / distance)`. The car's side must also clear the planned corner from the axle. The angle is the shallowest whole
     degree that meets both, at most `VC_AVOID_MAX_DEG`.
   - **Drive** the angled leg `diag_mm` until the car has **passed the obstacle**: the axle ends a turning circle
     beyond the planned back face (face + planned depth + turning circle). The whole car is then past it and can turn
     on the spot. A MOVE longer than `VC_MOVE_MAX_MM` is sent in pieces, so S2's 8 s command timeout never trips.
   - **Turn back** by `-turn_deg`: parallel to the original path, beyond the obstacle. There is no drive-on leg
     (`pass_mm` = 0).
5. **LINE FINDING**: turn `-turn_deg` toward the line, and creep at `VC_RECOVER_SPEED` with the heading held. When a
   line sensor reaches the line (`LINE_FOUND`), the car goes to LINE_FOLLOW and the line steering lines it up. It stops
   with `FAULT_LINE_LOST` if the creep covers `diag_mm + VC_REJOIN_EXTRA_MM` without finding the line.

   **Size of the detour:** the sideways move is large. For a 100 mm object at 250 mm it is about 812 mm at
   `SCAN_PLAN_WIDTH_PCT` 200, 635 mm at 150, and 455 mm at 100. Lower the ratio if the track is narrow.
6. **No usable side, or no profile within `VC_SCAN_TIMEOUT_MS`**: in bench mode the car goes back to CONFIRM and
   profiles again 5 s later. In mission mode it stops with `FAULT_OBSTACLE`.

`VC_OBST_BYPASS 0` profiles the still obstacle, then only waits for it to go away. The obstacle MQTT JSON also carries
`depth_mm`, `turn_deg`, `diag_mm`, `lat_mm` and `pass_mm`. The console prints the plan (`[S5] plan: ...`), then
after each step how far the car really turned and drove (`[S5] avoid: step N done, heading ... drove ...`): compare
the two to tell a wrong plan from a turn or move that undershoots.

## Stop when tipped over or picked up (23 Nov)
`VC_STOP_ON_TILT` and `VC_STOP_ON_LIFT` (both 1, in `vc_config.h`) work in bench mode and in the full mission:
* **Tilted** (S4 `FLG_IMU_TILTED` or `IMU_EVT_TILT`: leaning more than 35 deg) while driving -> STOP, `FAULT_TILT`.
* **Lifted** (S3 `FLG_LINE_LIFTED`: both line sensors above `LINE_LIFTED_ADC` for `LINE_LIFTED_MS` = 200 ms, i.e. no floor) while
  driving -> STOP, `FAULT_LIFTED`.
* In IDLE nothing happens (the car is not moving). A START (button or MQTT) while still tilted or lifted is refused with the
  same fault, so the car never drives off in your hand. Put it down level, press the button again: reset, then start.
* The dashboard shows the fault as `TILT` or `LIFTED`. The tilt stop needs a working IMU. The lift stop does not.

## Initialising phase, impact stop off, barcode directions from the dashboard (23 Nov)
* **INIT waits for everything:** motors, line sensors, IMU and (MQTT builds) the MQTT link, before IDLE. While waiting,
  `robot/se18/status` carries `"waiting":["IMU","WIFI",...]` and the dashboard shows INITIALISING with that list; its
  START button only unlocks in IDLE. The robot button is refused in INIT (`[S5] button: still initialising ...`).
  If WiFi never connects, INIT gives up after `VC_NET_WAIT_MS` (30 s) and goes to IDLE, so the car still runs without
  the dashboard. `VC_WAIT_FOR_NET` is 1 only when built with `MQTT=1`.
* **Impact no longer stops the car:** `VC_STOP_ON_IMPACT 0`. The IMU still prints `>> IMPACT`.
* **Barcode directions:** see `docs/README_subsystem1.md` (command `map ...`, topic `robot/se18/barcode_map`).
