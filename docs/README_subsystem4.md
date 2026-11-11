# Subsystem 4 - IMU and terrain monitoring (course kernel)

Replaces the template's example `app_program/subsystem4/` (Monitor). Student ID 2501101.
Written from the SE18 Design Review: FR4, FR8, FR9, FR10, NFR3, NFR10, statechart 6.3, messages 6.6.
Pins follow Figure 4.1 (the task interaction diagram): **I2C0, GP0 = SDA, GP1 = SCL**.

This package is subsystem 4 only. It builds, links and runs by itself; the other subsystems are not needed until integration.

## Files

```
app_program/subsystem4/
  subsystem4.h / .c   interface + init/start (creates event flag flg_imu, starts imu_task at priority 18)
  imu_task.c          the task: 10 ms loop, calibration, publishing events, optional console output
  imu_algo.h / .c     the IMU logic: pitch filter, motion classes, hump height, impact (no kernel calls)
  imu_config.h        every threshold and the two bench switches - tune here
  imu_msgs.h          imu_evt_t and motion_status_t (Section 6.6)
  drv_imu.h / .c      I2C driver for MPU-6050-family chips (called only by imu_task)
  s4_port.h           the only places that depend on things the template did not show me
  s4_fmt.h            float -> text so the kernel's printf needs no "%f"
testing/subsystem4/pc/   test_imu_algo.c + Makefile: runs on a PC, no Pico (make run)
TEAM_subsystem4.md       rows to paste into TEAM.md (pins, peripheral, priority, interfaces, budget)
```

## Install

1. Replace the template's `app_program/subsystem4/` folder with this one (and `testing/subsystem4/` with ours).
2. Paste the rows in `TEAM_subsystem4.md` into `TEAM.md`.
3. Build the way the template's README says. `app_main.c` already calls `subsystem4_init()` and `subsystem4_start()`.
4. The IMU is read through the kernel's I2C device `iicb` (I2C1). Its pins are set in `device/i2c/sysdepend/rp2040/i2c_cnf_sysdep.h`: change `DEVCNF_I2C0_SDA_PIN` / `_SCL_PIN` from 8/9 to **0/1** (see `SHARED_CONFIG_CHANGES.md`).
5. If the **compiler** complains, the fix is almost certainly in `s4_port.h` (console print, microsecond timer).

## Running it alone

Subsystem 4 needs two objects that other subsystems will own later (`mbf_imu_evt` from the Vehicle Controller,
`mtx_motion` + `app_motion` from Motion Control). `subsystem4.c` defines **weak** defaults of 0 for them, so it links without them.
While they are 0 the task:
* prints each event to the console instead of sending it, and
* pretends the robot drives at `S4_FAKE_SPEED_MM_S` (100 mm/s, in `imu_config.h`) so hump heights can be tested by hand.
  Set it to 0 to test the STATIONARY class.

When the real subsystems define the real objects, the linker picks theirs and the task switches over by itself.
Set `S4_DEBUG_PRINT` to 0 at integration to keep the console quiet.

Bench test (robot on the table, USB console open). Wiring: VCC to 3V3, GND to GND, **SDA to GP0, SCL to GP1**.

1. Boot: `found chip, WHO_AM_I = 0x68` (or 0x70 / 0x71 / 0x73), then calibration. Keep the board still and flat for 2 s.
   If `|a|` is not about 1.000 g, the chip or register map is not what I assumed.
2. **Pitch sign**: tilt the nose (+X side) up: `pitch` must go **positive** and stay. If it swings negative first, set
   `IMU_PITCH_RATE_SIGN` to `+1.0f` in `imu_config.h`. If nose-up is not your board's X axis, edit the `IMU_MAP_*` macros.
3. **Hump**: hold the board nose-up about 15 degrees for 1 s, lay it flat, nose-down about 15 degrees for 1 s, lay it flat:
   `>> HUMP peak = ~25 mm` (100 mm x sin 15 deg). A smaller hump keeps the MAX; a bigger one raises it.
4. **Turning**: rotate the board about its vertical axis: class `TURNING`.
5. **Impact**: tap the board sharply: `>> IMPACT`. Tune `IMU_SHOCK_G`.
6. The status line prints `exec<=` and `jitter<=` in microseconds: screenshot these for assumption A6.
   Expect jitter up to about 1 ms because the kernel tick is 1 ms.

## What the other subsystems must do (at integration)

* **Vehicle Controller**: create `mbf_imu_evt` (use `S4_IMU_EVT_MAXMSZ` / `S4_IMU_EVT_BUFSZ`) and receive `imu_evt_t` from it.
  On `IMU_EVT_IMPACT` set the STOP flag; on `IMU_EVT_HUMP` keep `max_hump_mm`; on `IMU_EVT_SENSOR_FAULT` go to FAULT.
  On a mission RESET `tk_set_flg(flg_imu, FLG_IMU_RESET)`. In INIT wait for `FLG_IMU_READY`. Before feeding the watchdog check
  `FLG_IMU_ALIVE` then `tk_clr_flg(flg_imu, ~FLG_IMU_ALIVE)`.
* **Motion Control**: define `app_motion` (`motion_status_t`) and `mtx_motion`; update it every 10 ms under the mutex.
  `distance_mm` is a cumulative odometer.

## What I assumed or added (tell the team)

* **Chip**: MPU-6050 family (the doc only says a 4-pin I2C module). The driver probes 0x68 / 0x69, prints WHO_AM_I, and
  calibration rejects the board if gravity is not about 1 g.
* `imu_evt_t` gained `turn_rate_dps`, `pitch_deg`, `accel_g`. A MOTION_CLASS event is sent on every class change and every
  500 ms (`IMU_PERIODIC_REPORT_MS`, 0 = off).
* Full message buffer: IMPACT and SENSOR_FAULT discard the oldest queued event to get in; other events are dropped.
  The IMU loop never blocks on another subsystem.
* Impact uses total acceleration magnitude, as in statechart 6.3. If crossing a hump edge triggers it, raise `IMU_SHOCK_G`.
* Hump height is the approximate pitch x distance estimate (assumption A4): check it against a hump of known height.
* **Priority**: your band is 18-19. In this kernel 1 is the highest priority, so S1-S3 (bands 12-17) outrank the IMU task.
  The design doc had Motion Control above the IMU, so it is worth checking the other bands follow the same order.
* **Stack size**: `app_start_task()` did not show a stack size, so I could not set one. `imu_task` uses a few hundred bytes of
  locals; if the template's default is very small, raise it.

## Not verified

* The IMU logic passes its 42 PC checks (hump height, running max, impact, calibration, turning).
* Every source file type-checks against the real T-Kernel (`mtkernel_3`) function signatures for the calls I use.
* The whole subsystem links on its own and runs on a PC against a fake kernel and a fake still IMU: calibration, the 10 ms loop,
  event printing and the status line all work.
* **Not built against the course template and not run on a Pico**: I only had `subsystem4.c` / `.h` from the template, not
  `app.h`, `app_main.c` or the build files. Thresholds in `imu_config.h` are starting values.
