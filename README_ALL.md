# SE18 robot car: firmware for all five subsystems (course kernel)

Written from the SE18 Design Review (Week 6 freeze) and the course `TEAM.md`. Each subsystem is one folder in the template layout
(`app_program/subsystemN/`, `testing/subsystemN/`), replacing the template's example code. Paste `TEAM_ALL.md` into `TEAM.md`.
Per-subsystem detail for S2 and S4 is in `docs/`.

| # | Subsystem | Owner (design review) | What is in it |
|---|---|---|---|
| 1 | WiFi + MQTT telemetry | Aristotle Cister Ben | `telemetry.c` (JSON + command parser), `subsystem1.c` (status buffer + 2 hooks), `PATCH_mqtt_utk.md` |
| 2 | Motion Control | Muhammad Irshad | `motion_ctrl.c` (statechart 6.2), `motion_task.c`, motor + encoder drivers |
| 3 | Line follow + barcode | Muhammad Isa | `line_algo.c`, `barcode_algo.c`, `line_task.c`, `barcode_task.c`, ADC-device driver + polled GP28 |
| 4 | IMU + terrain | Raymond | `imu_algo.c` (statechart 6.3), `imu_task.c`, MPU-6050 I2C driver |
| 5 | Vehicle Controller + obstacle scan | Sew Jing Wen | `vc_logic.c` (system statechart), `vc_task.c`, `scan_algo.c`, `scan_task.c`, servo + ultrasonic driver |

## How it fits together
`vc_task` (S5) is the only task that decides anything. It reads `imu_evt_t` (S4), `nav_cmd_t` and `line_evt_t` (S3), remote commands (S1) and the
READY / ALIVE flags of S2, S3, S4, and drives the mission: INIT -> IDLE -> LINE_FOLLOW -> BARCODE_ACTION / HUMP_CROSSING / OBSTACLE_AVOID / RECOVER ->
COMPLETE, FAULT from anywhere. Only `motion_task` (S2) writes the motors. Impact, remote stop, motion fault, a task silent for 100 ms: the VC sets the STOP
bit and S2 cuts the motors in its next 10 ms loop. Remote RESET clears STOP, resets S2 and the IMU's running maximum, and re-runs the self-test.
All cross-subsystem calls use `TMO_POL`: nothing waits for another subsystem.

## Install
1. Replace the five `app_program/subsystemN/` and `testing/subsystemN/` folders. Paste `TEAM_ALL.md` into `TEAM.md`.
2. `app_main.c` should already call `subsystemN_init()` for 1..5 and then `subsystemN_start()`. **Call all five `init`s before any `start`** (each task uses the others' objects).
3. Make the shared-config edits in `SHARED_CONFIG_CHANGES.md` (I2C1 pins, ADC pins). No Pico SDK libraries are needed: the drivers use the template's board library (`<bsp/libbsp.h>`) and the kernel's I2C / ADC devices.
4. Subsystem 1 only works after the `mqtt_utk.c` patch in `app_program/subsystem1/PATCH_mqtt_utk.md` (needs the instructor's agreement and `NET=lwip MQTT=1`).
5. If the compiler complains, look at the `sN_port.h` file of that subsystem first (console print, microsecond timer).

## Bring-up order (wheels OFF the ground)
1. **S4 alone / S2 alone** as in `docs/README_subsystem4.md` and `docs/README_subsystem2.md`: signs, `MC_TICKS_PER_REV`, pitch sign.
2. **S3**: watch the 2 Hz `[S3]` line. Calibration needs both sensors to see white AND black in the first 2 s (rock the robot over the line).
   Check `offset` sign: line under the right sensor is positive and `steer` negative.
3. **S5 scan**: put a box 30 cm ahead, trigger a scan, read `[S5] scan:` (width within 20 % is the NFR4 target).
4. All five: `[S5] state ->` lines show INIT -> IDLE. Send `start` to `robot/se18/cmd` (or build a test that sends `REMOTE_START` to `mbf_remote_cmd`).
5. Set `S2_BENCH_SCRIPT 0`, all `*_DEBUG_PRINT 0`, `VC_WATCHDOG_ENABLE 1` at integration.

## Things you must settle with the team (I had to assume them)
* **Barcode format**: the design review does not say it. `line_config.h` assumes 4 black bars, bar 1 narrow (reference), bars 2-4 wide = 1 / narrow = 0; codes 0 left, 1 right,
  2 straight, 3 U-turn, 7 stop. Change `bd_decode` / the code table when you have the real briefing.
* **Pins**: Figure 4.1 vs the pin table disagree (IMU, ultrasonic, servo). I used the figure. The Fritzing picture seems to put the IMU on GP0/GP1, which the debug probe uses with `CRASH_REPORT=1`.
* **Priorities**: the TEAM.md bands make S1 highest and S5 lowest (the opposite of the design doc, where Motion Control is highest). It works because every task sleeps between periods, but a
  stop takes IMU 10 ms + VC tick 20 ms + S2 10 ms, up to about 40 ms against the 50 ms NFR6. If you can, give `vc_task` a better priority by team agreement.
* **Steering sign**: `line_status_t.steering_corr` in mm/s of wheel-speed difference, + = steer left (S2 and S3 both follow it).
* **Obstacle bypass** uses a fixed obstacle depth (`VC_OBST_DEPTH_MM`, the ultrasonic sensor cannot measure it) and a 7-step turn / move sequence; tune the dimensions on the track.
* **Hump slow-down**: the IMU reports a hump only after it ends, so the VC slows when the IMU says CLIMBING and speeds up on the HUMP event.
* **Line lost**: RECOVER drives slowly straight; there is no sweep search (add one in `vc_logic.c` if the track needs it).
* **Link lost** (statechart 6.5 LINK_LOST) is not implemented: the robot keeps running its mission when WiFi drops, as NFR5 requires, but the VC is never told.
* The IMU yaw cross-check for U-turns is not implemented (S4 publishes turn rate, not yaw).

## Not verified
* Pure logic is tested on a PC: S1 21 checks, S2 36, S3 29, S4 (its own suite), S5 47 (state machine, bypass sequence, watchdog, scan geometry).
* All five link together and run on a PC against a fake kernel and fake hardware: boot, self-test, MQTT start, a motor stall becoming a FAULT, and RESET back to IDLE
  (the PC has no encoders, so the stall is expected).
* **Never built for the Pico, against the course template, or run on a robot.** I never saw `app.h`, `app_main.c`, the build files or `mqtt_utk.c`. Every gain, threshold and dimension is a
  placeholder to tune on the real car.
