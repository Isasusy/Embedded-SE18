# Rows to add to TEAM.md for subsystem 4 (IMU and terrain monitoring)

Paste each block into the matching table of `TEAM.md`. Anyone may claim empty rows (pins, peripheral, interface IDs);
the interface rows still need the other owners to agree on the names and data.

## 1. Members and subsystems

| # | Subsystem (what it does) | Student ID | GitHub username | Code | Tests |
|---|---|---|---|---|---|
| 4 | IMU and terrain monitoring: calibration, motion class, hump height + running max, impact | 2501101 | *(your username)* | `app_program/subsystem4/` | `testing/subsystem4/` |

(This replaces the template's example "Monitor" row.)

## 2. Pins

| Pin | Owner | Used for |
|---|---|---|
| GP0 | S4 | IMU I2C0 SDA |
| GP1 | S4 | IMU I2C0 SCL |

These follow Figure 4.1 (task interaction diagram). If the robot is physically wired like the Fritzing picture
(GP0 / GP1), the pins change and GP0/GP1 clash with `CRASH_REPORT=1`: settle this before wiring.

## 3. Peripherals

| Peripheral | Owner subsystem | Owning task | Pins |
|---|---|---|---|
| I2C0 | S4 | `imu_task` | GP0, GP1 |

The driver is `drv_imu.c` / `drv_imu.h` inside `app_program/subsystem4/`, called only from `imu_task`.

## 4. Task priorities

| Subsystem | Priority band | Tasks (name – priority – core) |
|---|---|---|
| 4 | 18–19 | `imu_task` – 18 – any |

## 5. Interfaces between subsystems

| ID | From → To | Mechanism | Object ID name | Data / meaning | Agreed by |
|---|---|---|---|---|---|
| IF-06 | S4 → Vehicle Controller | message buffer (owned by the Vehicle Controller) | `mbf_imu_evt` | `imu_evt_t` in `imu_msgs.h`: HUMP (peak + running max), IMPACT, MOTION_CLASS, SENSOR_FAULT. VC sets STOP on IMPACT | |
| IF-07 | Vehicle Controller ↔ S4 | event flag (owned by S4) | `flg_imu` | VC sets `FLG_IMU_RESET` (mission reset clears max hump); S4 sets `FLG_IMU_READY` (self-test) and `FLG_IMU_ALIVE` every 10 ms (VC checks and clears it before feeding the watchdog) | |
| IF-08 | Motion Control → S4 | shared data under a mutex (owned by Motion Control) | `app_motion`, `mtx_motion` | `motion_status_t`: speed_l, speed_r, distance_mm (cumulative). S4 reads with `TMO_POL` | |

## 6. Kernel object budget (subsystem 4 column)

| Object | S4 |
|---|---|
| Task | 1 |
| Event flag | 1 |
