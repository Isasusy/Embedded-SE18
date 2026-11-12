/*
 * imu_config.h - every tunable number for the IMU task in one place.
 * Section 9 of the design review says thresholds "will be tuned": do it here.
 * IMU_MAP_*, IMU_PITCH_RATE_SIGN, S4_DEBUG_PRINT and S4_FAKE_SPEED_MM_S are #ifndef-guarded and can also be
 * overridden on the compiler command line (-D...); the other values are edited here.
 */
#ifndef IMU_CONFIG_H
#define IMU_CONFIG_H

/* ------------------------------------------------------------------ timing / bus */
#define IMU_PERIOD_MS            10        /* NFR3: 100 Hz                                   */
/* Bus speed and per-transfer timeout belong to the kernel's I2C driver, not to this file:
 * device/i2c/sysdepend/rp2040/i2c_cnf_sysdep.h, DEVCNF_I2C0_TMO (10 ms, so a dead bus cannot hang the 10 ms loop). */
#define IMU_MAX_READ_FAILS       5        /* consecutive failed reads -> SENSOR_FAULT       */
#define IMU_RETRY_PERIOD_MS      500       /* re-init attempts while in fault                */

/* ------------------------------------------------------------------ chip scale factors
 * Configured in drv_imu.c: accel +-8 g, gyro +-500 dps, DLPF ~94 Hz (MPU6050/6500 family). */
#define IMU_ACCEL_LSB_PER_G      4096.0f
#define IMU_GYRO_LSB_PER_DPS     65.5f

/* ------------------------------------------------------------------ mounting orientation
 * Robot frame: X = forward, Y = left, Z = up.
 * Edit these three lines so they turn *sensor* axes (x,y,z) into robot axes. The same
 * mapping is applied to accelerometer and gyro. Default = board mounted flat, X pointing forward.
 * Example, board rotated 90 deg so sensor +Y points forward and sensor -X points left:
 *     IMU_MAP_X(x,y,z) (y)      IMU_MAP_Y(x,y,z) (-(x))      IMU_MAP_Z(x,y,z) (z)
 */
#ifndef IMU_MAP_X
#define IMU_MAP_X(x,y,z)         (x)
#define IMU_MAP_Y(x,y,z)         (y)
#define IMU_MAP_Z(x,y,z)         (z)
#endif

/* Right-hand rotation about robot +Y (left axis) points the nose DOWN, so nose-up rate = -gyro_y.
 * If, when you tilt the nose up quickly, pitch swings negative before settling, flip this to +1. */
#ifndef IMU_PITCH_RATE_SIGN
#define IMU_PITCH_RATE_SIGN      (-1.0f)
#endif

/* ------------------------------------------------------------------ CALIBRATE state */
#define IMU_CAL_SAMPLES          200       /* 2 s at 100 Hz                                  */
#define IMU_CAL_MAX_TRIES        5
#define IMU_CAL_MAX_GYRO_STD_DPS 1.5f      /* above this the robot was moved during calibration */
#define IMU_CAL_G_MIN            0.85f     /* sanity: stationary |accel| must be ~1 g        */
#define IMU_CAL_G_MAX            1.15f

/* ------------------------------------------------------------------ pitch filter */
#define IMU_COMP_ALPHA           0.98f     /* complementary filter: gyro weight per step     */
#define IMU_ACC_TRUST_BAND_G     0.25f     /* ignore accel for pitch when | |a| - 1 g | > this */

/* ------------------------------------------------------------------ Region A: motion classification */
#define IMU_MOVE_START_MM_S      20.0f     /* speed above this = moving                      */
#define IMU_MOVE_STOP_MM_S       10.0f     /* speed below this = stopped (hysteresis)        */
#define IMU_QUIET_GYRO_DPS       3.0f      /* fallback "stopped" test when no motion_status  */
#define IMU_QUIET_ACC_G          0.04f
#define IMU_QUIET_MS             500
#define IMU_TURN_ENTER_DPS       30.0f     /* |yaw rate| to enter TURNING                    */
#define IMU_TURN_EXIT_DPS        15.0f
#define IMU_CLIMB_ENTER_DEG      8.0f      /* pitch above +this = CLIMBING                   */
#define IMU_CLIMB_EXIT_DEG       4.0f
#define IMU_DESCEND_ENTER_DEG    8.0f      /* pitch below -this = DESCENDING                 */
#define IMU_DESCEND_EXIT_DEG     4.0f
#define IMU_ACCEL_ENTER_G        0.10f     /* forward accel above this = ACCELERATING        */
#define IMU_ACCEL_EXIT_G         0.04f
#define IMU_ACCEL_LPF            0.10f     /* low-pass gain for forward accel (per 10 ms)    */
#define IMU_CLASS_DEBOUNCE       3         /* samples a new class must persist (30 ms)       */
#define IMU_PERIODIC_REPORT_MS   500       /* periodic MOTION_CLASS status event, 0 = off    */

/* ------------------------------------------------------------------ SUDDEN_IMPACT */
#define IMU_SHOCK_G              2.0f      /* accel magnitude above this = collision         */
#define IMU_SHOCK_ARM_MS         500       /* ignore shocks this long after start-up         */
#define IMU_IMPACT_MIN_HOLD_MS   200       /* stay in SUDDEN_IMPACT at least this long (one event per hit) */
#define IMU_IMPACT_HOLD_MS       1500      /* leave SUDDEN_IMPACT after this even if moving  */

/* ------------------------------------------------------------------ Region B: hump measurement */
#define IMU_HUMP_START_DEG       6.0f      /* pitch above this begins a climb                */
#define IMU_HUMP_TOP_DEG         3.0f      /* pitch back below this = top reached            */
#define IMU_HUMP_DESC_DEG        4.0f      /* pitch below -this = descending                 */
#define IMU_HUMP_END_DEG         2.0f      /* pitch above -this after descent = hump over    */
#define IMU_HUMP_DEBOUNCE        3         /* samples                                         */
#define IMU_HUMP_MIN_DIST_MM     10.0f     /* must travel this far uphill to count as a hump */
#define IMU_HUMP_MIN_HEIGHT_MM   3.0f      /* smaller peaks are treated as noise             */
#define IMU_HUMP_TIMEOUT_MS      6000      /* give up if the hump never ends (e.g. a ramp)   */
#define IMU_HUMP_LOCKOUT_MS      300       /* "reset window" after a hump                    */
#define IMU_MAX_STEP_MM          30.0f     /* larger odometer jumps are treated as resync    */

/* task stack: TASK_STACK_SIZE in app.h (shared by every task) */

/* ------------------------------------------------------------------ bench helpers (subsystem 4 only) */
#ifndef S4_DEBUG_PRINT
#define S4_DEBUG_PRINT		1	/* 1 = print a status line 4x/s and (until the Vehicle Controller exists)
					       the events. Set 0 at integration to keep the 10 ms loop quiet. */
#endif
#ifndef S4_FAKE_SPEED_MM_S
#define S4_FAKE_SPEED_MM_S	100.0f	/* speed assumed while Motion Control does not exist yet.
					       0 = robot standing still (no hump heights). */
#endif

#endif /* IMU_CONFIG_H */
