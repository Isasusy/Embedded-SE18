
## Barcode then junction (from the project write-up, 6 Nov)

A decoded barcode is only **remembered** (`nav_has`). The car keeps line following. At the next JUNCTION event it
drives `VC_JUNCTION_ADVANCE_MM` forward (so the axle is at the junction centre) and then turns: LEFT +90, RIGHT -90,
UTURN 180; STRAIGHT drives on; STOP completes the mission at once. A junction with no barcode pending is ignored, and an
unused command expires after `VC_NAV_EXPIRE_MS`.

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

## START button on GP20, power-up in STOP (11 Nov)

The car powers up **stopped** and starts only when the GP20 button is pressed. Nothing else is needed on the bench: there
is no autostart and no MQTT command.

**Wiring** (no external parts): one side of the button to **GP20**, the other side to **GND**. The RP2040's internal
pull-up holds GP20 high while the button is open, so a press reads low (active low) and an unplugged or broken button
reads "not pressed" — the car cannot start by itself.

**Where it lives.** `subsystem5/drv_button.c` / `.h` (pin `PIN_START_BTN`, pull-up and debounce) plus a few lines in
`vc_task.c`. Nothing else needs rewriting: the button produces the same `remote_cmd_t` an MQTT command produces
(IF-15), so the state machine in `vc_logic.c` is untouched, and no other subsystem's code or shared kernel/config file
changes. The one extra edit is in S5's own `drv_scan.c`: GP20 is removed from the ultrasonic autodetect list, which
would otherwise have driven the button's pin during its wiring search. The pin is claimed in `TEAM.md` section 2; the
owning task is `vc_task`.

**What a press does, by state:**

| State when pressed | Result |
|---|---|
| `INIT` (self-test, < `VC_SELFTEST_MS`) | ignored; press again once the console shows `state -> IDLE` |
| `IDLE` | **starts the run**: sends `MOTION_CMD_GO` at `VC_CRUISE_SPEED`, state -> `LINE_FOLLOW` |
| `LINE_FOLLOW` and every other running state | ignored — brushing the button does not stop a run |
| `FAULT` / `COMPLETE` | **reset first** (same as `REMOTE_RESET`): back to `INIT` -> `IDLE`, stopped. The next press starts |

So after a fault the car never jumps straight back into driving: one press clears the fault, a second press starts.

**Why the run starts in STOP.**

- `motion_task` calls `motor_off()` at power-up, so PWM is 0 before the first loop;
- `vc_task` sends one `MOTION_CMD_STOP` before its first tick;
- `vc_logic.c` starts in `VC_INIT`, and `VC_INIT -> VC_IDLE` only reports the state — it never commands motion;
- `VC_AUTOSTART_MS` is now **0**, so nothing starts the mission on its own.

The wheels stay off until a `REMOTE_START` (button or MQTT) arrives in `IDLE`.

**Settings** (`vc_config.h`): `VC_START_BTN` 1 = button fitted, 0 = no button (MQTT / `VC_AUTOSTART_MS` only);
`VC_START_BTN_ACTIVE_LOW` 1 = button to GND with the pull-up, 0 = button to 3V3 with a pull-down;
`VC_START_BTN_DEBOUNCE_MS` 25 (a new level must be stable that long; `vc_task` samples every `VC_TICK_MS` = 20 ms, so a
press is seen in about 40 ms). Change the pin in `drv_button.h` **and** `TEAM.md`.

**Testing.** `make TEST=1` checks the resting level at power-up (`S5-U01`): a stuck button, a button wired to 3V3, the
wrong pin, or a missing pull-up (a floating pin reads 0 = pressed) fails that test instead of silently breaking the
start. Manual check: flash, wheels off the ground, wait 10 s — the car must not move; press GP20 — it drives; release;
press again while driving — nothing happens.

**Making it start/stop instead.** In `vc_task.c`, change the press handler to send `REMOTE_STOP` when the state is one of
the running states (`LINE_FOLLOW`, `BARCODE_ACTION`, `HUMP_CROSSING`, `OBSTACLE_AVOID`, `RECOVER`); `REMOTE_STOP` puts
the VC in `FAULT` with `FAULT_REMOTE_STOP`, which the next press clears.
