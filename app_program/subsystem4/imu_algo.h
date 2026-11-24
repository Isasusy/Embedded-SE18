/*
 * imu_algo.h - hardware-free IMU logic: calibration maths, pitch filter,
 * Region A (motion classification) and Region B (hump measurement) of statechart 6.3.
 *
 * No kernel, no hardware: it is unit-tested on a PC (testing/subsystem4/pc/test_imu_algo.c).
 * All inputs are already in the ROBOT frame (X forward, Y left, Z up) and in g / deg/s.
 */
#ifndef IMU_ALGO_H
#define IMU_ALGO_H

#include <stdint.h>
#include <stdbool.h>
#include "imu_msgs.h"

typedef struct {
    uint32_t n;
    float    sum_a[3], sum_g[3], sumsq_g[3];
} imu_cal_t;

typedef struct {
    float gyro_bias_dps[3];
    float pitch_offset_deg;   /* mounting tilt so that "level on the floor" reads 0 deg */
    float accel_mag_g;
    float gyro_std_dps[3];
    float accel_mean_g[3];    /* gravity at rest = the car's "level" direction (for tilt detection) */
} imu_cal_result_t;

typedef enum { IMU_CAL_OK, IMU_CAL_MOVED, IMU_CAL_BAD_GRAVITY, IMU_CAL_NO_DATA } imu_cal_status_t;

void             imu_cal_reset(imu_cal_t *c);
void             imu_cal_add(imu_cal_t *c, float ax, float ay, float az, float gx, float gy, float gz);
imu_cal_status_t imu_cal_finish(const imu_cal_t *c, imu_cal_result_t *res);

typedef enum {
    HUMP_NORMAL,
    HUMP_DETECTED,        /* climb started                    */
    HUMP_MEASURING_PEAK,  /* integrate sin(pitch) x distance  */
    HUMP_UPDATE_MAX       /* momentary: max_hump = max(max_hump, peak), publish */
} hump_state_t;

typedef struct {
    float    ax_g, ay_g, az_g;          /* robot frame                                  */
    float    gx_dps, gy_dps, gz_dps;    /* robot frame, gyro bias already removed       */
    float    dt_s;
    uint32_t now_ms;
    bool     speed_valid;               /* motion_status_t available?                   */
    float    speed_mm_s;                /* (speed_l + speed_r) / 2                      */
    bool     dist_valid;
    float    distance_mm;               /* cumulative odometer from Motion Control      */
} imu_algo_in_t;

#define IMU_ALGO_MAX_EVTS 4
typedef struct {
    imu_evt_t evt[IMU_ALGO_MAX_EVTS];
    uint8_t   n;
} imu_algo_out_t;

typedef struct {
    /* ---- observable (read these for debug / telemetry) ---- */
    motion_class_t cls;
    hump_state_t   hump_state;
    float          pitch_deg;           /* + = nose up, level = 0 */
    float          turn_rate_dps;       /* + = left               */
    float          accel_mag_g;
    float          a_fwd_g;
    float          hump_h_mm;           /* running height of the hump being measured */
    float          hump_peak_mm;
    float          max_hump_mm;         /* FR9: cleared only by imu_algo_reset_mission() */
    float          tilt_deg;            /* angle away from level, any direction (0..180) */
    bool           tilted;              /* a TILT was sent; cleared when level again     */

    /* ---- internal ---- */
    float    pitch_offset_deg;
    bool     pitch_init;
    bool     moving;
    float    a_fwd_lpf;
    float    quiet_s;
    motion_class_t cand;
    uint8_t  cand_cnt;
    uint32_t start_ms, last_report_ms;
    bool     impact_latched;
    uint32_t impact_ms;
    bool     have_dist;
    float    last_dist;
    uint8_t  climb_samples, end_samples;
    float    climb_dist;
    bool     seen_descent;
    uint32_t hump_t0_ms;
    bool     lockout_active;
    uint32_t lockout_ms;
    float    level_g[3];                /* unit vector of gravity when level (calibration) */
    float    grav_lpf[3];               /* low-passed accelerometer = gravity direction    */
    bool     grav_init;
    uint32_t tilt_since_ms;             /* first sample above IMU_TILT_STOP_DEG            */
    bool     tilt_counting;
    bool     no_gyro;                   /* accelerometer-only chip: pitch from accel alone */
} imu_algo_t;

void imu_algo_init(imu_algo_t *s, float pitch_offset_deg, uint32_t now_ms);
/* Optional, after imu_algo_init: the calibrated gravity vector (imu_cal_result_t.accel_mean_g) = "level" for tilt
   detection. Without it, level = robot +Z straight up. */
void imu_algo_set_level(imu_algo_t *s, const float accel_mean_g[3]);
/* Optional, after imu_algo_init: the chip has no gyroscope (LSM303). Pitch then comes from the accelerometer only
   (IMU_ACC_ONLY_ALPHA) and turn rate stays 0, so TURNING is never reported. Tilt, impact and humps still work. */
void imu_algo_set_no_gyro(imu_algo_t *s);
void imu_algo_reset_mission(imu_algo_t *s);   /* mission RESET: clears max_hump (FR9) */
void imu_algo_step(imu_algo_t *s, const imu_algo_in_t *in, imu_algo_out_t *out);

const char *motion_class_name(motion_class_t c);
const char *hump_state_name(hump_state_t h);

#endif /* IMU_ALGO_H */
