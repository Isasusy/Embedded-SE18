/*
 * PC unit test for imu_algo.c (no Pico needed).
 *   cd test && make run
 *
 * The simulator builds sensor readings that are physically consistent with a robot
 * (X forward, Y left, Z up) pitching nose-up by `pitch` degrees:
 *     ax = sin(pitch), az = cos(pitch), gyro_y = -d(pitch)/dt   (nose-up = negative rotation about +Y)
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "imu_algo.h"
#include "imu_config.h"

static int g_fail = 0;
#define CHECK(cond, ...) do { if (!(cond)) { g_fail++; printf("  FAIL: "); printf(__VA_ARGS__); printf("\n"); } \
                              else { printf("  ok:   "); printf(__VA_ARGS__); printf("\n"); } } while (0)

typedef struct {
    imu_algo_t s;
    uint32_t   t_ms;
    float      pitch_true, dist;
    int        n_hump, n_impact, n_class;
    imu_evt_t  last_hump, last_impact;
} sim_t;

static void sim_init(sim_t *m)
{
    memset(m, 0, sizeof *m);
    imu_algo_init(&m->s, 0.0f, 0);
}

/* advance 10 ms. pitch_target is slew-limited to 60 deg/s so gyro rates stay physical */
static void sim_step(sim_t *m, float pitch_target, float yaw_dps, float a_fwd_g,
                     float speed_mm_s, bool speed_valid, float az_override)
{
    const float dt = 0.010f;
    float d = pitch_target - m->pitch_true;
    const float max_d = 60.0f * dt;
    if (d >  max_d) d =  max_d;
    if (d < -max_d) d = -max_d;
    m->pitch_true += d;
    const float rate_up = d / dt;

    m->dist += speed_mm_s * dt;
    m->t_ms += 10;

    imu_algo_in_t in = {
        .ax_g = sinf(m->pitch_true * 0.017453293f) + a_fwd_g,
        .ay_g = 0.0f,
        .az_g = (az_override > 0.0f) ? az_override : cosf(m->pitch_true * 0.017453293f),
        .gx_dps = 0.0f, .gy_dps = -rate_up, .gz_dps = yaw_dps,
        .dt_s = dt, .now_ms = m->t_ms,
        .speed_valid = speed_valid, .speed_mm_s = speed_mm_s,
        .dist_valid = speed_valid, .distance_mm = m->dist,
    };
    imu_algo_out_t out;
    imu_algo_step(&m->s, &in, &out);
    for (int i = 0; i < out.n; i++) {
        switch (out.evt[i].type) {
        case IMU_EVT_HUMP:         m->n_hump++;   m->last_hump   = out.evt[i]; break;
        case IMU_EVT_IMPACT:       m->n_impact++; m->last_impact = out.evt[i]; break;
        case IMU_EVT_MOTION_CLASS: m->n_class++;  break;
        default: break;
        }
    }
}

static void run_flat(sim_t *m, float seconds, float yaw, float afwd, float speed)
{
    for (int i = 0; i < (int)(seconds * 100); i++) sim_step(m, 0.0f, yaw, afwd, speed, true, 0.0f);
}

/* one hump: climb at +up_deg for climb_mm, crest for crest_mm, descend at -up_deg for climb_mm, then level */
static void drive_hump(sim_t *m, float up_deg, float climb_mm, float crest_mm, float speed)
{
    float x = 0.0f;
    while (x < climb_mm)               { sim_step(m,  up_deg, 0, 0, speed, true, 0); x += speed * 0.01f; }
    x = 0.0f;
    while (x < crest_mm)               { sim_step(m,  0.0f,   0, 0, speed, true, 0); x += speed * 0.01f; }
    x = 0.0f;
    while (x < climb_mm)               { sim_step(m, -up_deg, 0, 0, speed, true, 0); x += speed * 0.01f; }
    for (int i = 0; i < 100; i++)        sim_step(m,  0.0f,   0, 0, speed, true, 0);
}

static void test_stationary_then_start(void)
{
    printf("[1] stationary -> accelerating -> cruising\n");
    sim_t m; sim_init(&m);
    run_flat(&m, 1.0f, 0, 0, 0.0f);
    CHECK(m.s.cls == MOTION_CLASS_STATIONARY, "stationary at speed 0 (got %s)", motion_class_name(m.s.cls));
    CHECK(fabsf(m.s.pitch_deg) < 0.5f, "level pitch ~0 (got %.2f)", m.s.pitch_deg);
    run_flat(&m, 0.5f, 0, 0.20f, 100.0f);
    CHECK(m.s.cls == MOTION_CLASS_ACCELERATING, "accelerating while a_fwd = 0.2 g (got %s)", motion_class_name(m.s.cls));
    run_flat(&m, 1.0f, 0, 0.0f, 100.0f);
    CHECK(m.s.cls == MOTION_CLASS_CRUISING, "cruising once accel ~0 (got %s)", motion_class_name(m.s.cls));
    run_flat(&m, 1.0f, 0, 0.0f, 0.0f);
    CHECK(m.s.cls == MOTION_CLASS_STATIONARY, "back to stationary when speed 0 (got %s)", motion_class_name(m.s.cls));
}

static void test_turning(void)
{
    printf("[2] turning\n");
    sim_t m; sim_init(&m);
    run_flat(&m, 1.0f, 0, 0, 100.0f);
    run_flat(&m, 1.0f, 60.0f, 0, 100.0f);
    CHECK(m.s.cls == MOTION_CLASS_TURNING, "TURNING at 60 dps (got %s)", motion_class_name(m.s.cls));
    CHECK(fabsf(m.s.turn_rate_dps - 60.0f) < 0.1f, "turn rate reported = %.1f dps", m.s.turn_rate_dps);
    run_flat(&m, 1.0f, 0, 0, 100.0f);
    CHECK(m.s.cls == MOTION_CLASS_CRUISING, "cruising again after turn (got %s)", motion_class_name(m.s.cls));
}

static void test_climb_descend_classes_and_sign(void)
{
    printf("[3] climbing / descending classes + pitch sign convention\n");
    sim_t m; sim_init(&m);
    run_flat(&m, 1.0f, 0, 0, 100.0f);
    for (int i = 0; i < 100; i++) sim_step(&m, 12.0f, 0, 0, 100.0f, true, 0);
    CHECK(m.s.pitch_deg > 10.0f, "nose-up gives POSITIVE pitch (got %.2f)", m.s.pitch_deg);
    CHECK(m.s.cls == MOTION_CLASS_CLIMBING, "CLIMBING at +12 deg (got %s)", motion_class_name(m.s.cls));
    for (int i = 0; i < 200; i++) sim_step(&m, -12.0f, 0, 0, 100.0f, true, 0);
    CHECK(m.s.pitch_deg < -10.0f, "nose-down gives NEGATIVE pitch (got %.2f)", m.s.pitch_deg);
    CHECK(m.s.cls == MOTION_CLASS_DESCENDING, "DESCENDING at -12 deg (got %s)", motion_class_name(m.s.cls));
}

static void test_hump_and_running_max(void)
{
    printf("[4] hump height + running maximum (FR4, FR9)\n");
    sim_t m; sim_init(&m);
    run_flat(&m, 1.0f, 0, 0, 100.0f);

    /* hump 1: 12 deg x 150 mm  -> 150*sin(12) = 31.2 mm */
    drive_hump(&m, 12.0f, 150.0f, 100.0f, 100.0f);
    CHECK(m.n_hump == 1, "one hump event (got %d)", m.n_hump);
    CHECK(m.last_hump.peak_mm > 28.0f && m.last_hump.peak_mm < 34.0f,
          "hump 1 peak ~31.2 mm (got %.1f)", m.last_hump.peak_mm);
    CHECK(fabsf(m.last_hump.max_hump_mm - m.last_hump.peak_mm) < 0.01f, "max == peak after first hump");
    const float max1 = m.s.max_hump_mm;

    /* hump 2: smaller, 8 deg x 100 mm -> 13.9 mm; max must NOT drop */
    drive_hump(&m, 8.0f, 100.0f, 0.0f, 100.0f);
    CHECK(m.n_hump == 2, "second hump event (got %d)", m.n_hump);
    CHECK(m.last_hump.peak_mm < max1 - 10.0f, "hump 2 peak is smaller (%.1f mm)", m.last_hump.peak_mm);
    CHECK(fabsf(m.s.max_hump_mm - max1) < 0.01f, "running max unchanged by smaller hump (%.1f)", m.s.max_hump_mm);
    CHECK(fabsf(m.last_hump.max_hump_mm - max1) < 0.01f, "event carries the running max (%.1f)", m.last_hump.max_hump_mm);

    /* hump 3: taller, 15 deg x 200 mm -> 51.8 mm; max must rise */
    drive_hump(&m, 15.0f, 200.0f, 50.0f, 100.0f);
    CHECK(m.n_hump == 3, "third hump event (got %d)", m.n_hump);
    CHECK(m.s.max_hump_mm > max1 + 15.0f && m.s.max_hump_mm > 47.0f && m.s.max_hump_mm < 56.0f,
          "running max rose to ~51.8 mm (got %.1f)", m.s.max_hump_mm);

    /* mission reset clears the maximum */
    imu_algo_reset_mission(&m.s);
    CHECK(m.s.max_hump_mm == 0.0f, "mission reset clears max_hump");
}

static void test_no_hump_when_stopped_or_noise(void)
{
    printf("[5] no false humps\n");
    sim_t m; sim_init(&m);
    run_flat(&m, 1.0f, 0, 0, 0.0f);
    /* tilted while NOT moving (someone lifts the robot): no distance -> no hump */
    for (int i = 0; i < 200; i++) sim_step(&m, 15.0f, 0, 0, 0.0f, true, 0);
    for (int i = 0; i < 200; i++) sim_step(&m, 0.0f, 0, 0, 0.0f, true, 0);
    CHECK(m.n_hump == 0, "lifting a stationary robot is not a hump (got %d)", m.n_hump);

    /* tiny bump: 7 deg over 12 mm -> ~1.5 mm, below IMU_HUMP_MIN_HEIGHT_MM */
    drive_hump(&m, 7.0f, 12.0f, 0.0f, 100.0f);
    CHECK(m.n_hump == 0, "1.5 mm bump ignored (got %d)", m.n_hump);

    /* long ramp that never comes down: timeout, no event, no max change */
    sim_t r; sim_init(&r);
    run_flat(&r, 1.0f, 0, 0, 100.0f);
    for (int i = 0; i < 1000; i++) sim_step(&r, 10.0f, 0, 0, 100.0f, true, 0);
    for (int i = 0; i < 1000; i++) sim_step(&r, 0.0f, 0, 0, 100.0f, true, 0);
    CHECK(r.n_hump == 0 && r.s.max_hump_mm == 0.0f, "a ramp is not reported as a hump");
}

static void test_impact(void)
{
    printf("[6] sudden impact (FR8)\n");
    sim_t m; sim_init(&m);
    run_flat(&m, 1.0f, 0, 0, 100.0f);
    sim_step(&m, 0.0f, 0, 0, 100.0f, true, 3.2f);          /* one-sample 3.2 g spike */
    CHECK(m.n_impact == 1, "exactly one IMPACT event (got %d)", m.n_impact);
    CHECK(m.last_impact.accel_g > 3.0f, "impact magnitude reported (%.2f g)", m.last_impact.accel_g);
    CHECK(m.last_impact.motion_class == MOTION_CLASS_SUDDEN_IMPACT, "class = SUDDEN_IMPACT");
    sim_step(&m, 0.0f, 0, 0, 100.0f, true, 3.0f);           /* still ringing: must not re-trigger */
    sim_step(&m, 0.0f, 0, 0, 100.0f, true, 2.5f);
    CHECK(m.n_impact == 1, "ringing does not re-trigger (got %d)", m.n_impact);
    run_flat(&m, 0.5f, 0, 0, 0.0f);                          /* robot stopped (STOP bit took effect) */
    CHECK(m.s.cls == MOTION_CLASS_STATIONARY, "back to STATIONARY once stopped (got %s)", motion_class_name(m.s.cls));
    CHECK(m.n_hump == 0, "impact produced no hump");

    /* impact in the middle of a hump must not corrupt the running max */
    sim_t h; sim_init(&h);
    run_flat(&h, 1.0f, 0, 0, 100.0f);
    for (int i = 0; i < 100; i++) sim_step(&h, 12.0f, 0, 0, 100.0f, true, 0);
    sim_step(&h, 12.0f, 0, 0, 100.0f, true, 3.5f);
    for (int i = 0; i < 300; i++) sim_step(&h, 0.0f, 0, 0, 0.0f, true, 0);
    CHECK(h.s.max_hump_mm == 0.0f && h.n_hump == 0, "impact mid-hump aborts the measurement");

    /* shocks right after boot are ignored */
    sim_t b; sim_init(&b);
    sim_step(&b, 0.0f, 0, 0, 0.0f, true, 3.0f);
    CHECK(b.n_impact == 0, "start-up shock ignored during arming time");
}

static void test_no_motion_status_fallback(void)
{
    printf("[7] no motion_status available (IMU-only fallback)\n");
    sim_t m; sim_init(&m);
    for (int i = 0; i < 100; i++) sim_step(&m, 0.0f, 0, 0, 0.0f, false, 0);
    CHECK(m.s.cls == MOTION_CLASS_STATIONARY, "quiet IMU -> STATIONARY (got %s)", motion_class_name(m.s.cls));
}

static void test_calibration(void)
{
    printf("[8] calibration\n");
    imu_cal_t c; imu_cal_result_t r;

    imu_cal_reset(&c);
    srand(1);
    for (int i = 0; i < 200; i++) {
        float n = ((rand() % 100) - 50) / 100.0f * 0.1f;               /* +-0.05 dps noise */
        imu_cal_add(&c, 0.0f, 0.0f, 1.0f, 1.2f + n, -0.8f + n, 0.5f + n);
    }
    CHECK(imu_cal_finish(&c, &r) == IMU_CAL_OK, "still data accepted");
    CHECK(fabsf(r.gyro_bias_dps[0] - 1.2f) < 0.05f && fabsf(r.gyro_bias_dps[1] + 0.8f) < 0.05f &&
          fabsf(r.gyro_bias_dps[2] - 0.5f) < 0.05f, "gyro bias recovered (%.2f %.2f %.2f)",
          r.gyro_bias_dps[0], r.gyro_bias_dps[1], r.gyro_bias_dps[2]);

    imu_cal_reset(&c);
    for (int i = 0; i < 200; i++)                                         /* mounted 5 deg nose-up */
        imu_cal_add(&c, sinf(5.0f * 0.017453293f), 0.0f, cosf(5.0f * 0.017453293f), 0, 0, 0);
    imu_cal_finish(&c, &r);
    CHECK(fabsf(r.pitch_offset_deg - 5.0f) < 0.1f, "mounting tilt measured (%.2f deg)", r.pitch_offset_deg);

    imu_cal_reset(&c);
    for (int i = 0; i < 200; i++) imu_cal_add(&c, 0, 0, 1.0f, (i % 2) ? 40.0f : -40.0f, 0, 0);
    CHECK(imu_cal_finish(&c, &r) == IMU_CAL_MOVED, "moving robot rejected");

    imu_cal_reset(&c);
    for (int i = 0; i < 200; i++) imu_cal_add(&c, 0, 0, 0.3f, 0, 0, 0);
    CHECK(imu_cal_finish(&c, &r) == IMU_CAL_BAD_GRAVITY, "|a| != 1 g rejected (wrong chip / not stationary)");

    /* level reading after calibration with a mounting tilt: pitch must read 0 */
    sim_t m; imu_algo_init(&m.s, 5.0f, 0); m.t_ms = 0; m.pitch_true = 5.0f; m.dist = 0;
    for (int i = 0; i < 100; i++) sim_step(&m, 5.0f, 0, 0, 0.0f, true, 0);
    CHECK(fabsf(m.s.pitch_deg) < 0.3f, "pitch offset removed (got %.2f)", m.s.pitch_deg);
}

static void test_bench_procedure(void)
{
    printf("[9] the hand-tilt bench procedure from the README (fake 100 mm/s)\n");
    sim_t m; sim_init(&m);
    run_flat(&m, 1.0f, 0, 0, 100.0f);
    for (int i = 0; i < 100; i++) sim_step(&m, 15.0f, 0, 0, 100.0f, true, 0);   /* nose-up 15 deg, 1 s */
    for (int i = 0; i <  50; i++) sim_step(&m,  0.0f, 0, 0, 100.0f, true, 0);   /* flat 0.5 s          */
    for (int i = 0; i < 100; i++) sim_step(&m,-15.0f, 0, 0, 100.0f, true, 0);   /* nose-down 15 deg, 1 s */
    for (int i = 0; i <  50; i++) sim_step(&m,  0.0f, 0, 0, 100.0f, true, 0);   /* flat 0.5 s          */
    CHECK(m.n_hump == 1, "one HUMP event (got %d)", m.n_hump);
    CHECK(m.last_hump.peak_mm > 20.0f && m.last_hump.peak_mm < 27.0f,
          "peak ~ 20-27 mm (got %.1f)", m.last_hump.peak_mm);
}

int main(void)
{
    test_stationary_then_start();
    test_turning();
    test_climb_descend_classes_and_sign();
    test_hump_and_running_max();
    test_no_hump_when_stopped_or_noise();
    test_impact();
    test_no_motion_status_fallback();
    test_calibration();
    test_bench_procedure();
    printf("\n%s (%d failed checks)\n", g_fail ? "FAILED" : "ALL PASSED", g_fail);
    return g_fail ? 1 : 0;
}
