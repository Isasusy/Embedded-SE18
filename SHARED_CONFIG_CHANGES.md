# Shared configuration edits (agree these with the team / instructor)

The drivers use the kernel's own device drivers, which are configured in files the whole team shares.
Pins follow the Fritzing wiring diagram.

## 1. I2C0 pins for the IMU (Subsystem 4): GP0 = SDA, GP1 = SCL
File: `device/i2c/sysdepend/rp2040/i2c_cnf_sysdep.h`

    #define DEVCNF_I2C0_SDA_PIN   0       /* was 8: GP8/GP9 are the left motor, must NOT be claimed by I2C */
    #define DEVCNF_I2C0_SCL_PIN   1       /* was 9 */
    #define DEVCNF_I2C1_SDA_PIN   6       /* leave at the original values: 14/15 would claim GP15 (servo) */
    #define DEVCNF_I2C1_SCL_PIN   7

## 2. ADC pins for the line sensors (Subsystem 3)
File: `device/adc/sysdepend/rp2040/adc_cnf_sysdep.h`

    #define DEVCONF_ADC_PIN_INIT_0   TRUE     /* GP26, left line sensor  */
    #define DEVCONF_ADC_PIN_INIT_1   TRUE     /* GP27, right line sensor */

Leave `_2` (GP28) FALSE (barcode is read as a digital input) and `_3` FALSE (GP29 is the Pico W radio clock).

## 3. Enable the I2C device driver
File: `config/config_device.h`

    #define DEVCNF_USE_IIC   1

## Notes
* GP0/GP1 are also the crash-report UART: do not build with `CRASH_REPORT=1` while the IMU is connected.
* The PWM block is released from reset by `motor_init()` / `servo_init()` (harmless if the kernel already did it).
* The hardware watchdog (only if `VC_WATCHDOG_ENABLE 1`) is programmed by register in `subsystem5/s5_port.h`; untested.

## Bench settings of this build (Raymond's robot, 5 Nov)

| File | Setting | Bench value | At integration |
|---|---|---|---|
| subsystem2/motion_config.h | MC_ENC_L_SIGN | -1 (left encoder counts backwards) | keep |
| subsystem2/motion_config.h | MC_TICKS_PER_REV | 860 (hand-measured) | confirm with 1 m push test |
| subsystem2/motion_config.h | S2_MOTOR_TEST | 0 | 0 |
| subsystem2/motion_config.h | S2_DEBUG_PRINT | 1 | 0 |
| subsystem5/vc_config.h | VC_AUTOSTART_MS | 5000 (starts without MQTT) | 0 |
| subsystem5/vc_config.h | VC_REQUIRE_IMU | 0 (IMU does not answer on the bench) | 1 |
| subsystem5/vc_config.h | VC_WATCHDOG_ENABLE | 0 | 1 |
| subsystem5/vc_config.h | VC_BENCH_LINE_ONLY | 1 (creep at 60 mm/s, ignore barcodes/obstacles/humps) | 0 |
| subsystem5/vc_config.h | SCAN_CLOSE_MM / SCAN_WATCH_DEG / SERVO_TRIM_DEG / SERVO_REVERSE | 200 / 0 / 0 / 0 (adjust on the robot) | keep adjusted values |
| subsystem5/vc_config.h | SCAN_WATCH_SWEEP_DEG / STEP / SETTLE_MS / SCAN_PATH_HALF_W_MM | 40 / 5 / 45 / 120 (continuous servo sweep while driving) | keep |
| subsystem5/vc_config.h | S5_SERVO_TEST, S5_ULTRA_PRINT | 0, 1 | 0, 0 |
| subsystem3/line_config.h | LINE_STEER_GAIN / LINE_STEER_MIN / LINE_STEER_HOLD_MS | 240 / 110 / 200 (stronger steering on curves) | keep, tune on the track |
| subsystem2/motion_config.h | MC_STALL_CYCLES / MC_STALL_MIN_TGT | 100 (1 s) / 30 mm/s (fewer false stall faults) | keep |
| subsystem5/vc_config.h | VC_JUNCTION_PAUSE_MS / VC_JUNCTION_PAUSE_ALWAYS | 1000 / 1 (stop at every junction, then act) | keep or set ALWAYS 0 |
| subsystem5/vc_config.h | VC_BENCH_BARCODES | 1 (bench still turns at a junction after a barcode) | (bench only) |
| subsystem3/line_config.h | BAR_ADC_THRESH / BAR_ADC_HYST | 1000 / 150 (GP28 read by the ADC) | tune from the `barcode=` readings |
| subsystem5/vc_config.h | VC_BENCH_RETRY_MS | 1500 (bench: reset and drive on after a stall) | (bench only) |
| subsystem2/motion_config.h | MC_STEER_MAX | 200 (inner wheel may slow to 0 or reverse on a tight curve) | keep |
| app_program/app_console.h | APP_CONSOLE | 1 (0 = no printing, for battery runs without the USB cable) | 0 at integration unless a console is wanted |
| subsystem3/line_config.h | LINE_STEER_TOWARD_BLACK | 1 (one line between the sensors) | flip to 0 if it steers the wrong way |
