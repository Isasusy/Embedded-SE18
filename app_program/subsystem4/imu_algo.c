/*
 * imu_algo.c - see imu_algo.h. Hardware-free; builds with plain gcc for the PC unit test.
 *
 * Statechart 6.3 mapping
 *   CALIBRATE ............ imu_cal_*()
 *   Region A (10 ms) ..... region_a():  STATIONARY / ACCELERATING / CRUISING / TURNING /
 *                                       CLIMBING / DESCENDING / SUDDEN_IMPACT
 *   Region B (hump) ...... region_b():  NORMAL -> HUMP_DETECTED -> MEASURING_PEAK -> UPDATE_MAX -> NORMAL
 */
#include "imu_algo.h"
#include "imu_config.h"
#include "s4_math.h"
#include <string.h>

#define RAD2DEG 57.29577951f
#define DEG2RAD 0.017453293f

/* ================================================================ calibration */
void imu_cal_reset(imu_cal_t *c) { memset(c, 0, sizeof *c); }

void imu_cal_add(imu_cal_t *c, float ax, float ay, float az, float gx, float gy, float gz)
{
    c->sum_a[0] += ax;  c->sum_a[1] += ay;  c->sum_a[2] += az;
    c->sum_g[0] += gx;  c->sum_g[1] += gy;  c->sum_g[2] += gz;
    c->sumsq_g[0] += gx * gx;  c->sumsq_g[1] += gy * gy;  c->sumsq_g[2] += gz * gz;
    c->n++;
}

imu_cal_status_t imu_cal_finish(const imu_cal_t *c, imu_cal_result_t *r)
{
    memset(r, 0, sizeof *r);
    if (c->n < 2) return IMU_CAL_NO_DATA;

    const float n = (float)c->n;
    float ma[3], worst_std = 0.0f;
    for (int i = 0; i < 3; i++) {
        ma[i] = c->sum_a[i] / n;
        const float mg  = c->sum_g[i] / n;
        float var = c->sumsq_g[i] / n - mg * mg;
        if (var < 0.0f) var = 0.0f;
        r->gyro_bias_dps[i] = mg;
        r->gyro_std_dps[i]  = mm_sqrtf(var);
        if (r->gyro_std_dps[i] > worst_std) worst_std = r->gyro_std_dps[i];
    }
    r->accel_mag_g       = mm_sqrtf(ma[0] * ma[0] + ma[1] * ma[1] + ma[2] * ma[2]);
    r->pitch_offset_deg  = mm_atan2f(ma[0], mm_sqrtf(ma[1] * ma[1] + ma[2] * ma[2])) * RAD2DEG;

    if (worst_std > IMU_CAL_MAX_GYRO_STD_DPS)                              return IMU_CAL_MOVED;
    if (r->accel_mag_g < IMU_CAL_G_MIN || r->accel_mag_g > IMU_CAL_G_MAX)  return IMU_CAL_BAD_GRAVITY;
    return IMU_CAL_OK;
}

/* ================================================================ helpers */
static void emit(imu_algo_out_t *o, const imu_algo_t *s, imu_evt_type_t type,
                 float peak_mm, float accel_g, uint32_t now_ms)
{
    if (o->n >= IMU_ALGO_MAX_EVTS) return;
    imu_evt_t *e = &o->evt[o->n++];
    e->type          = type;
    e->peak_mm       = peak_mm;
    e->max_hump_mm   = s->max_hump_mm;
    e->motion_class  = s->cls;
    e->turn_rate_dps = s->turn_rate_dps;
    e->pitch_deg     = s->pitch_deg;
    e->accel_g       = accel_g;
    e->timestamp_ms  = now_ms;
}

static void hump_clear(imu_algo_t *s)
{
    s->hump_state    = HUMP_NORMAL;
    s->hump_h_mm     = 0.0f;
    s->hump_peak_mm  = 0.0f;
    s->climb_samples = 0;
    s->climb_dist    = 0.0f;
    s->end_samples   = 0;
    s->seen_descent  = false;
}

static void hump_clear_with_lockout(imu_algo_t *s, uint32_t now_ms)
{
    hump_clear(s);
    s->lockout_active = true;
    s->lockout_ms     = now_ms;
}

const char *motion_class_name(motion_class_t c)
{
    switch (c) {
    case MOTION_CLASS_STATIONARY:    return "STATIONARY";
    case MOTION_CLASS_ACCELERATING:  return "ACCELERATING";
    case MOTION_CLASS_CRUISING:      return "CRUISING";
    case MOTION_CLASS_TURNING:       return "TURNING";
    case MOTION_CLASS_CLIMBING:      return "CLIMBING";
    case MOTION_CLASS_DESCENDING:    return "DESCENDING";
    case MOTION_CLASS_SUDDEN_IMPACT: return "SUDDEN_IMPACT";
    }
    return "?";
}

const char *hump_state_name(hump_state_t h)
{
    switch (h) {
    case HUMP_NORMAL:         return "NORMAL";
    case HUMP_DETECTED:       return "HUMP_DETECTED";
    case HUMP_MEASURING_PEAK: return "MEASURING_PEAK";
    case HUMP_UPDATE_MAX:     return "UPDATE_MAX";
    }
    return "?";
}

/* ================================================================ init / reset */
void imu_algo_init(imu_algo_t *s, float pitch_offset_deg, uint32_t now_ms)
{
    memset(s, 0, sizeof *s);
    s->pitch_offset_deg = pitch_offset_deg;
    s->cls              = MOTION_CLASS_STATIONARY;
    s->cand             = MOTION_CLASS_STATIONARY;
    s->start_ms         = now_ms;
    s->last_report_ms   = now_ms;
    hump_clear(s);
}

void imu_algo_reset_mission(imu_algo_t *s)
{
    s->max_hump_mm    = 0.0f;       /* FR9: the ONLY place the running maximum is cleared */
    s->impact_latched = false;
    s->cls            = MOTION_CLASS_STATIONARY;
    s->cand           = MOTION_CLASS_STATIONARY;
    s->cand_cnt       = 0;
    s->lockout_active = false;
    hump_clear(s);
}

/* ================================================================ Region A */
static void region_a(imu_algo_t *s, const imu_algo_in_t *in, imu_algo_out_t *out)
{
    const float gz_abs = mm_fabsf(in->gz_dps);

    const bool turning =
        gz_abs > IMU_TURN_ENTER_DPS ||
        (s->cls == MOTION_CLASS_TURNING && gz_abs > IMU_TURN_EXIT_DPS);

    const bool climbing =
        s->pitch_deg > IMU_CLIMB_ENTER_DEG ||
        (s->cls == MOTION_CLASS_CLIMBING && s->pitch_deg > IMU_CLIMB_EXIT_DEG);

    const bool descending =
        s->pitch_deg < -IMU_DESCEND_ENTER_DEG ||
        (s->cls == MOTION_CLASS_DESCENDING && s->pitch_deg < -IMU_DESCEND_EXIT_DEG);

    /* forward acceleration with the gravity component of the tilted board removed */
    const float tilt_rad = (s->pitch_deg + s->pitch_offset_deg) * DEG2RAD;
    s->a_fwd_g   = in->ax_g - mm_sinf(tilt_rad);
    s->a_fwd_lpf += IMU_ACCEL_LPF * (s->a_fwd_g - s->a_fwd_lpf);

    const bool accel =
        s->a_fwd_lpf > IMU_ACCEL_ENTER_G ||
        (s->cls == MOTION_CLASS_ACCELERATING && s->a_fwd_lpf > IMU_ACCEL_EXIT_G);

    motion_class_t want;
    if (!s->moving)                              want = MOTION_CLASS_STATIONARY;
    else if (s->cls == MOTION_CLASS_STATIONARY)  want = MOTION_CLASS_ACCELERATING; /* "speed up" */
    else if (turning)                            want = MOTION_CLASS_TURNING;
    else if (climbing)                           want = MOTION_CLASS_CLIMBING;
    else if (descending)                         want = MOTION_CLASS_DESCENDING;
    else if (accel)                              want = MOTION_CLASS_ACCELERATING;
    else                                         want = MOTION_CLASS_CRUISING;

    if (want == s->cls) {
        s->cand = want;
        s->cand_cnt = 0;
    } else {
        if (want == s->cand) s->cand_cnt++;
        else { s->cand = want; s->cand_cnt = 1; }

        if (s->cand_cnt >= IMU_CLASS_DEBOUNCE) {
            s->cls = want;
            s->cand_cnt = 0;
            s->last_report_ms = in->now_ms;
            emit(out, s, IMU_EVT_MOTION_CLASS, 0.0f, s->accel_mag_g, in->now_ms);
        }
    }

#if IMU_PERIODIC_REPORT_MS > 0
    if ((in->now_ms - s->last_report_ms) >= IMU_PERIODIC_REPORT_MS) {
        s->last_report_ms = in->now_ms;
        emit(out, s, IMU_EVT_MOTION_CLASS, 0.0f, s->accel_mag_g, in->now_ms);
    }
#endif
}

/* ================================================================ Region B */
static void region_b(imu_algo_t *s, const imu_algo_in_t *in, float dd_mm, imu_algo_out_t *out)
{
    const uint32_t now = in->now_ms;
    const float step_mm = dd_mm * mm_sinf(s->pitch_deg * DEG2RAD);   /* height gained this step */

    switch (s->hump_state) {

    case HUMP_NORMAL:
        if (s->lockout_active) {
            if ((now - s->lockout_ms) < IMU_HUMP_LOCKOUT_MS) break;      /* "reset window" */
            s->lockout_active = false;
        }
        if (s->pitch_deg > IMU_HUMP_START_DEG) {
            /* integrate from the very first sample so the debounce time is not lost */
            s->hump_h_mm  += step_mm;
            s->climb_dist += dd_mm;
            s->climb_samples++;
            if (s->climb_samples >= IMU_HUMP_DEBOUNCE && s->climb_dist >= IMU_HUMP_MIN_DIST_MM) {
                s->hump_state   = HUMP_DETECTED;
                s->hump_peak_mm = s->hump_h_mm;
                s->hump_t0_ms   = now;
            }
        } else {
            s->hump_h_mm = 0.0f;  s->climb_samples = 0;  s->climb_dist = 0.0f;
        }
        break;

    case HUMP_DETECTED:                       /* climb started: keep integrating until the top */
        s->hump_h_mm += step_mm;
        if (s->hump_h_mm > s->hump_peak_mm) s->hump_peak_mm = s->hump_h_mm;
        if (s->pitch_deg < IMU_HUMP_TOP_DEG) {
            s->hump_state   = HUMP_MEASURING_PEAK;      /* top reached */
            s->seen_descent = false;
            s->end_samples  = 0;
        } else if ((now - s->hump_t0_ms) > IMU_HUMP_TIMEOUT_MS) {
            hump_clear_with_lockout(s, now);            /* never levelled out: it is a ramp, not a hump */
        }
        break;

    case HUMP_MEASURING_PEAK:                 /* over the crest: wait for the descent to finish */
        s->hump_h_mm += step_mm;
        if (s->hump_h_mm > s->hump_peak_mm) s->hump_peak_mm = s->hump_h_mm;

        if (s->pitch_deg < -IMU_HUMP_DESC_DEG) {
            s->seen_descent = true;
            s->end_samples  = 0;
        } else if (s->seen_descent && s->pitch_deg > -IMU_HUMP_END_DEG) {
            if (++s->end_samples >= IMU_HUMP_DEBOUNCE) s->hump_state = HUMP_UPDATE_MAX;  /* descent complete */
        } else {
            s->end_samples = 0;
        }
        if (s->hump_state == HUMP_MEASURING_PEAK && (now - s->hump_t0_ms) > IMU_HUMP_TIMEOUT_MS)
            hump_clear_with_lockout(s, now);
        break;

    case HUMP_UPDATE_MAX:
        break;  /* handled right below, in the same step */
    }

    if (s->hump_state == HUMP_UPDATE_MAX) {
        const float peak = s->hump_peak_mm;
        if (peak >= IMU_HUMP_MIN_HEIGHT_MM) {
            if (peak > s->max_hump_mm) s->max_hump_mm = peak;           /* running maximum (FR9) */
            emit(out, s, IMU_EVT_HUMP, peak, s->accel_mag_g, now);
        }
        hump_clear_with_lockout(s, now);
    }
}

/* ================================================================ one 10 ms step */
void imu_algo_step(imu_algo_t *s, const imu_algo_in_t *in, imu_algo_out_t *out)
{
    out->n = 0;
    const float    dt  = in->dt_s;
    const uint32_t now = in->now_ms;

    /* ---- magnitudes ---- */
    const float a_h   = mm_sqrtf(in->ay_g * in->ay_g + in->az_g * in->az_g);
    const float a_mag = mm_sqrtf(in->ax_g * in->ax_g + a_h * a_h);
    s->accel_mag_g    = a_mag;
    s->turn_rate_dps  = in->gz_dps;

    /* ---- pitch: complementary filter (gyro for dynamics, accel for drift) ---- */
    const float pitch_acc = mm_atan2f(in->ax_g, a_h) * RAD2DEG - s->pitch_offset_deg;
    if (!s->pitch_init) {
        s->pitch_deg  = pitch_acc;
        s->pitch_init = true;
    } else {
        float p = s->pitch_deg + (IMU_PITCH_RATE_SIGN * in->gy_dps) * dt;
        if (mm_fabsf(a_mag - 1.0f) < IMU_ACC_TRUST_BAND_G)       /* accel only trusted near 1 g */
            p = IMU_COMP_ALPHA * p + (1.0f - IMU_COMP_ALPHA) * pitch_acc;
        s->pitch_deg = p;
    }

    /* ---- moving / stopped ---- */
    const bool quiet = mm_fabsf(in->gx_dps) < IMU_QUIET_GYRO_DPS &&
                       mm_fabsf(in->gy_dps) < IMU_QUIET_GYRO_DPS &&
                       mm_fabsf(in->gz_dps) < IMU_QUIET_GYRO_DPS &&
                       mm_fabsf(a_mag - 1.0f) < IMU_QUIET_ACC_G;
    s->quiet_s = quiet ? (s->quiet_s + dt) : 0.0f;

    if (in->speed_valid) {
        const float v = mm_fabsf(in->speed_mm_s);
        if (s->moving) { if (v < IMU_MOVE_STOP_MM_S)  s->moving = false; }
        else           { if (v > IMU_MOVE_START_MM_S) s->moving = true;  }
    } else {
        /* no motion_status yet: fall back to "IMU has been quiet for 0.5 s" */
        s->moving = s->quiet_s < (IMU_QUIET_MS / 1000.0f);
    }

    /* ---- SUDDEN_IMPACT (highest priority) ---- */
    const bool armed = (now - s->start_ms) >= IMU_SHOCK_ARM_MS;
    if (armed && !s->impact_latched && a_mag > IMU_SHOCK_G) {
        s->impact_latched = true;
        s->impact_ms      = now;
        s->cls            = MOTION_CLASS_SUDDEN_IMPACT;
        s->cand           = MOTION_CLASS_SUDDEN_IMPACT;
        s->cand_cnt       = 0;
        hump_clear_with_lockout(s, now);        /* an impact must never corrupt the hump maximum (A4) */
        emit(out, s, IMU_EVT_IMPACT, 0.0f, a_mag, now);
    }
    if (s->impact_latched) {
        const uint32_t held = now - s->impact_ms;
        if ((!s->moving && held >= IMU_IMPACT_MIN_HOLD_MS) || held >= IMU_IMPACT_HOLD_MS) {
            s->impact_latched = false;          /* robot stopped -> back to STATIONARY            */
            s->cls  = MOTION_CLASS_STATIONARY;
            s->cand = MOTION_CLASS_STATIONARY;
        }
    }

    /* ---- distance travelled since the last step ---- */
    float dd = 0.0f;
    if (in->dist_valid) {
        if (s->have_dist) {
            const float d = in->distance_mm - s->last_dist;
            if (d > 0.0f && d <= IMU_MAX_STEP_MM) dd = d;     /* backwards / huge jumps = resync */
        }
        s->have_dist = true;
        s->last_dist = in->distance_mm;
    } else if (in->speed_valid) {
        dd = mm_fabsf(in->speed_mm_s) * dt;
    }

    /* ---- the two parallel regions ---- */
    if (!s->impact_latched) {
        region_a(s, in, out);
        region_b(s, in, dd, out);
    }
}
