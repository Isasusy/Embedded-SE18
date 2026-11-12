
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
on again after `VC_BENCH_CLEAR_MS` of clear path. Adjusting the servo: `S5_SERVO_TEST 1` sweeps it at power-up and prints
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

## Start / stop button (11 Nov, v24)

`VC_START_BUTTON 1` (vc_config.h): after power-up the car waits in IDLE and prints
`[S5] ready: press a button on the Robo Pico (GP20 or GP21) to start`. Press and release either on-board button: it
starts `VC_BUTTON_DELAY_MS` (1 s) later, so your hand is clear. Press while it runs: it stops (`button: STOP`).
Press again: it resets and starts again. The buttons pull the pin to GND; the pads get internal pull-ups.
The ultrasonic pin search no longer touches GP18 (RGB LED), GP20/21 (buttons) or GP22 (buzzer).
`VC_START_BUTTON 0` brings back the old 5 s automatic start (`VC_AUTOSTART_MS`). An MQTT `start` still works too.
