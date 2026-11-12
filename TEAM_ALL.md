# Rows to paste into TEAM.md (all five subsystems)

Pins follow Figure 4.1 (the task interaction diagram), not the pin table, as you asked.

## 1. Members and subsystems
| # | Subsystem (what it does) | Student ID | GitHub username | Code | Tests |
|---|---|---|---|---|---|
| 1 | WiFi + MQTT telemetry: status, heartbeat, remote commands (runs inside the kernel's WiFi task) | 2503802 | | `app_program/subsystem1/` | `testing/subsystem1/` |
| 2 | Motion Control: motor PWM, encoders, speed PI, straight hold, turns, U-turn, move | 2500297 | | `app_program/subsystem2/` | `testing/subsystem2/` |
| 3 | Line following + barcode decoding | 2503719 | | `app_program/subsystem3/` | `testing/subsystem3/` |
| 4 | IMU and terrain monitoring | 2501101 | | `app_program/subsystem4/` | `testing/subsystem4/` |
| 5 | Vehicle Controller + obstacle scan | 2503464 | | `app_program/subsystem5/` | `testing/subsystem5/` |

## 2. Pins
| Pin | Owner | Used for |
|---|---|---|
| GP2 / GP3 | S2 | left encoder A / B |
| GP4 / GP5 | S2 | right encoder A / B |
| GP8 / GP9 | S2 | left motor M1 (Robo Pico IN1A / IN1B) |
| GP10 / GP11 | S2 | right motor M2 (Robo Pico IN2A / IN2B) |
| GP0 / GP1 | S4 | IMU I2C0 SDA / SCL |
| GP15 | S5 | servo signal (PWM) |
| GP17 | S5 | ultrasonic TRIG |
| GP16 | S5 | ultrasonic ECHO |
| GP26 | S3 | line IR left (ADC0) |
| GP27 | S3 | line IR right (ADC1) |
| GP28 | S3 | barcode IR (digital input, polled every 1 ms) |

## 3. Peripherals
| Peripheral | Owner subsystem | Owning task | Pins |
|---|---|---|---|
| I2C0 | S4 | `imu_task` | GP0, GP1 |
| ADC | S3 | `line_task` | GP26, GP27 |
| PWM slices 4, 5 | S2 | `motion_task` | GP8-11 |
| PWM slice 7 (channel B) | S5 | `scan_task` | GP15 |
| GPIO polling, encoders (1 ms) | S2 | `enc_task` | GP2-5 |
| GPIO polling, barcode (1 ms) | S3 | `barcode_task` | GP28 |
| WiFi radio, TCP/IP, MQTT | *kernel* | `cyw43_utk_task` (S1's hooks run inside it) | GP23-25, GP29 |

No interrupts are used by S2/S3: both inputs are polled every 1 ms (the board library has no GPIO interrupt call). I2C1 and the ADC go through the kernel's device drivers (`iicb`, `adca`); this needs the small shared-config edits in `SHARED_CONFIG_CHANGES.md`. Nothing uses the Pico SDK `hardware/*` headers.

## 4. Task priorities (1 = highest)
| Subsystem | Band | Tasks (name - priority - core) |
|---|---|---|
| 1 | 12-13 | none (hooks inside `cyw43_utk_task`) |
| 2 | 14-15 | `enc_task` - 14 - core 0; `motion_task` - 15 - core 0 |
| 3 | 16-17 | `line_task` - 16 - any; `barcode_task` - 17 - core 0 |
| 4 | 18-19 | `imu_task` - 18 - any |
| 5 | 20-21 | `vc_task` - 20 - any; `scan_task` - 21 - any |

## 5. Interfaces (the provider creates the object and publishes its ID in its subsystemN.h)
| ID | From -> To | Mechanism | Object ID name | Data / meaning | Agreed by |
|---|---|---|---|---|---|
| IF-06 | S4 -> S5 | message buffer (owned by S5) | `mbf_imu_evt` | `imu_evt_t` (imu_msgs.h): HUMP, IMPACT, MOTION_CLASS, SENSOR_FAULT | |
| IF-07 | S5 <-> S4 | event flag (owned by S4) | `flg_imu` | VC sets RESET; S4 sets READY, ALIVE | |
| IF-08 | S2 -> S5, S4 | shared data + mutex (owned by S2) | `app_motion`, `mtx_motion` | `motion_status_t` | |
| IF-09 | S5 -> S2 | message buffer (owned by S2) | `mbf_motion_cmd` | `motion_cmd_t` (GO, STOP, TURN, UTURN, MOVE) | |
| IF-10 | S5 <-> S2 | event flag (owned by S2) | `flg_motion` | VC: STOP (level), RESET. S2: READY, ALIVE, FAULT | |
| IF-11 | S3 -> S2 | shared data + mutex (owned by S3) | `app_line`, `mtx_line` | `line_status_t`: `steering_corr` mm/s, **+ = steer left** | |
| IF-12 | S3 -> S5 | message buffer (owned by S5) | `mbf_nav_cmd` | `nav_cmd_t` (vc_msgs.h) | |
| IF-13 | S3 -> S5 | message buffer (owned by S5) | `mbf_line_evt` | `line_evt_t`: JUNCTION, LINE_LOST, LINE_FOUND | |
| IF-14 | S5 -> S1 | message buffer (owned by S1) | `mbf_status` | `robot_status_t`, every 500 ms + on state change | |
| IF-15 | S1 -> S5 | message buffer (owned by S5) | `mbf_remote_cmd` | `remote_cmd_t`: START, STOP, RESET | |
| IF-16 | S5 <-> S3 | event flag (owned by S3) | `flg_line` | S3 sets READY, ALIVE | |

Internal to S5: `flg_scan`, `mbf_obstacle`. Shared structs live in `subsystem5/vc_msgs.h` (S1, S3, S5), `subsystem2/motion_msgs.h` (S2, S3, S5) and `subsystem4/imu_msgs.h` (S4, S5).

## 6. Kernel object budget
| Object | S1 | S2 | S3 | S4 | S5 | Total |
|---|---|---|---|---|---|---|
| Task | 0 | 2 | 2 | 1 | 2 | 7 (limit 32) |
| Event flag | | 1 | 1 | 1 | 1 | 4 (16) |
| Message buffer | 1 | 1 | | | 5 | 7 (12; the WiFi/USB tasks use a few) |
| Mutex | | 1 | 1 | | | 2 (12) |
