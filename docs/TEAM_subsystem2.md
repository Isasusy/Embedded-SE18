# Rows to add to TEAM.md for subsystem 2 (Motion Control)

Paste each block into the matching table of `TEAM.md`. The interface rows still need the other owners to agree.

## 1. Members and subsystems
| # | Subsystem (what it does) | Student ID | GitHub username | Code | Tests |
|---|---|---|---|---|---|
| 2 | Motion Control: motor PWM, wheel encoders, speed PI, straight-line hold, turns, U-turn, move | 2500297 | *(owner's username)* | `app_program/subsystem2/` | `testing/subsystem2/` |

(Replaces the template's example "Producer" row.)

## 2. Pins
| Pin | Owner | Used for |
|---|---|---|
| GP2 / GP3 | S2 | left wheel encoder A / B |
| GP4 / GP5 | S2 | right wheel encoder A / B |
| GP8 / GP9 | S2 | left motor M1 (Robo Pico IN1A / IN1B) (PWM) |
| GP10 / GP11 | S2 | right motor M2 (Robo Pico IN2A / IN2B) (PWM) |

## 3. Peripherals
| Peripheral | Owner subsystem | Owning task | Pins |
|---|---|---|---|
| PWM slice 4 (GP8, GP9) and slice 5 (GP10, GP11) | S2 | `motion_task` | GP8-11 |
| GPIO bank edge interrupt 13 (encoders) | S2 | `enc_task` (core 0) | GP2-5 |

Drivers: `drv_motor.c`, `drv_encoder.c` in `app_program/subsystem2/`, `drv_motor.c` called only from `motion_task`; `drv_encoder.c` polled by `enc_task`, read by `motion_task`.

## 4. Task priorities
| Subsystem | Priority band | Tasks (name - priority - core) |
|---|---|---|
| 2 | 14-15 | `enc_task` - 14 - core 0 (TP_PRC1); `motion_task` - 15 - core 0 (TP_PRC1) |

## 5. Interfaces between subsystems
| ID | From -> To | Mechanism | Object ID name | Data / meaning | Agreed by |
|---|---|---|---|---|---|
| IF-09 | Vehicle Controller -> S2 | message buffer (owned by S2), depth 4 | `mbf_motion_cmd` | `motion_cmd_t` (GO, STOP, TURN, UTURN, MOVE; speed_mm_s, angle_deg, distance_mm) | |
| IF-10 | Vehicle Controller <-> S2 | event flag (owned by S2) | `flg_motion` | VC: STOP (level, VC clears), RESET. S2: READY, ALIVE (every 10 ms), FAULT (stall / timeout) | |
| IF-08 | S2 -> Vehicle Controller, IMU (S4) | shared data + mutex (owned by S2) | `app_motion`, `mtx_motion` | `motion_status_t`: speed_l/r, distance_mm (cumulative), heading_deg, cmd_done | |
| IF-11 | Line Follow (S3) -> S2 | shared data + mutex (owned by S3, **proposed names**) | `app_line`, `mtx_line` | `line_status_t`: `steering_corr` in mm/s of wheel-speed difference, **+ = steer left**; `on_line` | |

## 6. Kernel object budget (subsystem 2 column)
| Object | S2 |
|---|---|
| Task | 1 |
| Event flag | 1 |
| Message buffer | 1 |
| Mutex | 1 |
