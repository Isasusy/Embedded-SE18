# Subsystem 2 - Motion Control (course kernel)

Replaces the template's example `app_program/subsystem2/` (Producer). Written from the SE18 Design Review:
FR3, FR8, NFR2, NFR6, NFR8, NFR10, statechart 6.2, messages 6.6, pins from Figure 4.1 / Section 2.
Pins: encoders GP2-GP5, motors GP8-GP11 (see `drv_motor.h`).

## Files
```
app_program/subsystem2/
  subsystem2.h / .c   interface + init/start (mbf_motion_cmd, flg_motion, mtx_motion; starts enc_task priority 14 and motion_task priority 15, both core 0)
  motion_task.c       the 10 ms loop: encoders, flags, command, line steering, control, motors, status, alive
  motion_ctrl.h / .c  the control logic (statechart 6.2): no kernel or hardware calls, tested on a PC
  motion_config.h     every tunable number and the two bench switches
  motion_msgs.h       motion_cmd_t, motion_status_t, line_status_t (Section 6.6)
  drv_motor.h / .c    Robo Pico PWM, 10 kHz
  drv_encoder.h / .c  quadrature x4 decode in a GPIO edge interrupt (started by enc_task.c; polling fallback)
  s2_port.h           console print, microsecond timer, ms clock
testing/subsystem2/pc/   test_motion_ctrl.c + Makefile (make run)
TEAM_subsystem2.md       rows to paste into TEAM.md
```

## Behaviour
* **CRUISE** `GO speed`: PI per wheel (feed-forward + P + I) every 10 ms. Line Follow's `steering_corr` (mm/s, + = left) is
  split across the wheels. If the line is lost or `mtx_line` does not exist, it holds the heading it had.
* **TURNING / U_TURN**: spin on the spot using encoder distance, slow down near the target, stop 2 degrees early (coasting finishes).
* **DISTANCE_MOVE**: drive `distance_mm` (negative = reverse), heading held, ramp down over the last 40 mm.
* After a TURN / UTURN / MOVE: `cmd_done` latches TRUE and the state returns to CRUISE **if it was cruising**, otherwise to STOPPED
  (small deviation from the chart: a turn from STOPPED does not suddenly drive off). The next command clears `cmd_done`.
* **STOP bit** (`FLG_MOTION_STOP`) is polled first every loop: PWM is 0 in the same loop (NFR6), state FAULT_STOP.
  Leave it by clearing STOP, then setting `FLG_MOTION_RESET`. Commands are ignored in FAULT_STOP.
* **Stall** (duty >= 200 with no encoder edge for 300 ms) and **command timeout** (8 s) enter FAULT_STOP and set `FLG_MOTION_FAULT`.
* `FLG_MOTION_ALIVE` is set every loop.

## Install
1. Replace the template's `app_program/subsystem2/` and `testing/subsystem2/` with these. Paste `TEAM_subsystem2.md` into `TEAM.md`.
2. **The template's example Consumer (subsystem 3) imports `sem_slots` / `mpf_samples` from the old subsystem 2.** Those are gone, so
   the example subsystem 3 will not link until its owner replaces it (Line Follow will).
3. The drivers use the template's board library (`<bsp/libbsp.h>`: `pwm_*`, `gpio_*`); no Pico SDK libraries are needed.
4. If the **compiler** complains, look at `s2_port.h` first.

## Bench test (wheels OFF the ground)
1. Check wiring signs: `MC_MOTOR_L_SIGN` / `MC_MOTOR_R_SIGN` so a positive duty drives the wheel forward, `MC_ENC_L_SIGN` / `MC_ENC_R_SIGN`
   so `tick` increases when the wheel turns forward (watch the 4 Hz `[S2]` line while turning a wheel by hand).
2. **Calibrate** `MC_TICKS_PER_REV`: turn one wheel exactly one revolution by hand and read `tick`. Then measure `MC_WHEEL_DIAM_MM` and `MC_TRACK_MM`.
3. Set `S2_BENCH_SCRIPT` to 1: GO 150 mm/s for 3 s, TURN +90, MOVE 300 mm, UTURN, STOP. Check `vL` / `vR` settle near `tgt`.
4. Tune `MC_KP`, `MC_KI`, `MC_FF_*` if speed oscillates or lags. Tune `MC_KH` if it weaves while holding heading.
5. On the floor: measure the real turn angle and the 300 mm move. The U-turn target is +-10 degrees (FR3).
6. The status line shows `exec<=` and `late<=` in microseconds: late must stay under 2000 for NFR2 (assumption A2).
7. Set `S2_BENCH_SCRIPT` 0 and `S2_DEBUG_PRINT` 0 at integration.

## What the other subsystems must do
* **Vehicle Controller**: send `motion_cmd_t` with `tk_snd_mbf(mbf_motion_cmd, &cmd, sizeof cmd, TMO_POL)`; set `FLG_MOTION_STOP` on impact / fault /
  remote stop and clear it when safe; then set `FLG_MOTION_RESET`; check and clear `FLG_MOTION_ALIVE` before feeding the watchdog; react to `FLG_MOTION_FAULT`.
* **Line Follow**: define `app_line` / `mtx_line` and update them every 5 ms. `steering_corr` is mm/s of wheel speed difference, + = steer left.
* **IMU (subsystem 4)**: its weak `app_motion` / `mtx_motion` defaults are replaced by ours at link time. Its `motion_status_t` is identical.

## Not done / assumptions
* **IMU yaw cross-check for U-turns** (design doc 6.2) is not implemented: the IMU subsystem publishes turn rate, not yaw. Encoders alone control the turn.
* Geometry and gains are placeholders. `MC_TICKS_PER_REV = 860` is measured by hand turning (about 830-890 per revolution); confirm with a 1 m push test (dist= in the status line). Encoder signs: left encoder inverted (`MC_ENC_L_SIGN -1`), the others +1.
* Encoders are **polled every 1 ms** by `enc_task` (the board library has no GPIO interrupt call). Edge rates above ~500/s per channel can be missed; check `tick` against a hand-turned revolution.
  The motor PWM block is released from reset in `motor_init()` (harmless if the kernel already did it).
* The `late<=` jitter on the PC test run is the PC's, not the Pico's.

## Not verified
* The control logic passes 36 PC checks against a simulated motor and wheel model: speed settling, straightness with a weaker motor, line steering sign,
  turns +-90 and U-turn within +-10 degrees with a 3-5% geometry error, moves forward and in reverse, STOP-bit behaviour and reset, stall, timeout.
* All files compile warning-free against stub headers copying the real kernel signatures, and the subsystem links and runs alone on a PC against a fake kernel.
* **Never built for the Pico or against the course template, never driven on a real robot.** I only had `subsystem2.c` / `.h` from the template, not `app.h`,
  `app_main.c` or the build files. The PC motor model is mine, so the real gains will need tuning.
