#include "slip_observer.h"

#include <math.h>
#include <string.h>

/*
 * Thresholds intentionally live only in this translation unit for now.
 * The observer is confidence logic, not a Kalman update.
 *
 * Design intent:
 * - near zero: wider residual deadband avoids encoder quantization false alarms;
 * - moving: residual allowance grows modestly with speed for acceleration/braking;
 * - NIS/yaw/lateral terms corroborate wheel residual instead of triggering alone;
 * - EWMA + persistence + hysteresis prevent one-sample escalation.
 */
#define SLIP_EWMA_TAU_S                 0.35f
#define SLIP_NEAR_ZERO_SPEED_MPS        0.15f
#define SLIP_ZERO_RESIDUAL_DB_MPS       0.18f
#define SLIP_MOVING_RESIDUAL_DB_MPS     0.28f
#define SLIP_SPEED_DB_GAIN              0.08f
#define SLIP_FORWARD_FULL_MPS           1.20f

#define SLIP_NIS_DB                     2.0f
#define SLIP_NIS_FULL                  18.0f

#define SLIP_YAW_DB_RAD_S               0.10f
#define SLIP_YAW_EXPECTED_GAIN          0.25f
#define SLIP_YAW_FULL_RAD_S             0.70f

#define SLIP_LATERAL_DB_MPS2            1.50f
#define SLIP_LATERAL_FULL_MPS2          4.00f

#define SLIP_SUSPECT_SCORE              0.32f
#define SLIP_SUSPECT_HOLD_S             0.35f
#define SLIP_SLIP_SCORE                 0.58f
#define SLIP_SLIP_HOLD_S                0.65f
#define SLIP_SEVERE_SCORE               0.82f
#define SLIP_SEVERE_HOLD_S              0.90f

#define SLIP_RECOVER_ENTER_SCORE        0.30f
#define SLIP_RECOVER_ENTER_HOLD_S       1.20f
#define SLIP_RECOVER_NORMAL_SCORE       0.18f
#define SLIP_RECOVER_NORMAL_HOLD_S      2.20f

static float clamp01(float x)
{
    if (x < 0.0f) return 0.0f;
    if (x > 1.0f) return 1.0f;
    return x;
}

static float ewma(float previous, float sample, float dt)
{
    float alpha = dt / (SLIP_EWMA_TAU_S + dt);
    return previous + alpha * (sample - previous);
}

static void clear_escalation_timers(SlipObserver *o)
{
    o->suspect_time_s = 0.0f;
    o->slip_time_s = 0.0f;
    o->severe_time_s = 0.0f;
}

void slip_observer_init(SlipObserver *o)
{
    if (!o) return;
    memset(o, 0, sizeof(*o));
    o->state = SLIP_STATE_NORMAL;
    o->initialized = 1U;
}

void slip_observer_update(SlipObserver *o, const SlipObserverInput *in)
{
    if (!o || !in || !o->initialized) return;
    if (!isfinite(in->dt_s) || in->dt_s <= 0.0f) return;
    if (!isfinite(in->wheel_speed_mps) ||
        !isfinite(in->predicted_forward_mps) ||
        !isfinite(in->wheel_nis) ||
        !isfinite(in->gyro_z_rads) ||
        !isfinite(in->lateral_accel_mps2)) return;
    if (in->expected_yaw_rate_valid && !isfinite(in->expected_yaw_rate_rads)) return;

    float dt = in->dt_s;
    if (dt > 0.20f) dt = 0.20f;

    const float abs_wheel = fabsf(in->wheel_speed_mps);
    const float abs_pred = fabsf(in->predicted_forward_mps);
    const float speed_ref = abs_wheel > abs_pred ? abs_wheel : abs_pred;
    const float forward_residual = fabsf(in->wheel_speed_mps - in->predicted_forward_mps);

    float residual_db;
    if (speed_ref < SLIP_NEAR_ZERO_SPEED_MPS) {
        residual_db = SLIP_ZERO_RESIDUAL_DB_MPS;
    } else {
        residual_db = SLIP_MOVING_RESIDUAL_DB_MPS + SLIP_SPEED_DB_GAIN * speed_ref;
    }

    const float forward_evidence =
        clamp01((forward_residual - residual_db) / SLIP_FORWARD_FULL_MPS);
    const float nis_evidence =
        clamp01((in->wheel_nis - SLIP_NIS_DB) / (SLIP_NIS_FULL - SLIP_NIS_DB));

    float yaw_residual = 0.0f;
    float yaw_evidence = 0.0f;
    if (in->expected_yaw_rate_valid) {
        yaw_residual = fabsf(in->gyro_z_rads - in->expected_yaw_rate_rads);
        const float yaw_db =
            SLIP_YAW_DB_RAD_S + SLIP_YAW_EXPECTED_GAIN * fabsf(in->expected_yaw_rate_rads);
        yaw_evidence = clamp01((yaw_residual - yaw_db) / SLIP_YAW_FULL_RAD_S);
    }

    const float lateral_evidence =
        clamp01((fabsf(in->lateral_accel_mps2) - SLIP_LATERAL_DB_MPS2) /
                SLIP_LATERAL_FULL_MPS2);

    o->forward_residual_ewma_mps =
        ewma(o->forward_residual_ewma_mps, forward_residual, dt);
    o->wheel_nis_ewma = ewma(o->wheel_nis_ewma, in->wheel_nis, dt);
    o->yaw_residual_ewma_rads =
        ewma(o->yaw_residual_ewma_rads, yaw_residual, dt);
    o->lateral_evidence = ewma(o->lateral_evidence, lateral_evidence, dt);
    o->forward_evidence = ewma(o->forward_evidence, forward_evidence, dt);
    o->yaw_evidence = ewma(o->yaw_evidence, yaw_evidence, dt);

    /*
     * Wheel residual is primary evidence. NIS provides covariance-aware support.
     * Yaw mismatch is useful during turning, while lateral acceleration only
     * amplifies an already inconsistent wheel/yaw condition.
     */
    float raw_score = 0.58f * forward_evidence +
                      0.27f * nis_evidence +
                      0.15f * yaw_evidence;
    raw_score += 0.18f * lateral_evidence *
                 fmaxf(forward_evidence, yaw_evidence);
    raw_score = clamp01(raw_score);
    o->score = ewma(o->score, raw_score, dt);

    if (o->score >= SLIP_SUSPECT_SCORE) o->suspect_time_s += dt;
    else o->suspect_time_s = 0.0f;

    if (o->score >= SLIP_SLIP_SCORE) o->slip_time_s += dt;
    else o->slip_time_s = 0.0f;

    const int severe_corroborated =
        (o->forward_evidence > 0.65f) ||
        (o->yaw_evidence > 0.55f && o->wheel_nis_ewma > 6.0f);
    if (o->score >= SLIP_SEVERE_SCORE && severe_corroborated) {
        o->severe_time_s += dt;
    } else {
        o->severe_time_s = 0.0f;
    }

    if (o->score <= SLIP_RECOVER_ENTER_SCORE) o->stable_time_s += dt;
    else o->stable_time_s = 0.0f;

    switch (o->state) {
    case SLIP_STATE_NORMAL:
        if (o->suspect_time_s >= SLIP_SUSPECT_HOLD_S) {
            o->state = SLIP_STATE_SUSPECT;
            o->stable_time_s = 0.0f;
        }
        break;

    case SLIP_STATE_SUSPECT:
        if (o->slip_time_s >= SLIP_SLIP_HOLD_S) {
            o->state = SLIP_STATE_SLIP;
            o->stable_time_s = 0.0f;
        } else if (o->stable_time_s >= 0.90f) {
            o->state = SLIP_STATE_NORMAL;
            clear_escalation_timers(o);
        }
        break;

    case SLIP_STATE_SLIP:
        if (o->severe_time_s >= SLIP_SEVERE_HOLD_S) {
            o->state = SLIP_STATE_SEVERE;
            o->stable_time_s = 0.0f;
        } else if (o->stable_time_s >= SLIP_RECOVER_ENTER_HOLD_S) {
            o->state = SLIP_STATE_RECOVERY;
            clear_escalation_timers(o);
            o->stable_time_s = 0.0f;
        }
        break;

    case SLIP_STATE_SEVERE:
        if (o->stable_time_s >= SLIP_RECOVER_ENTER_HOLD_S) {
            o->state = SLIP_STATE_RECOVERY;
            clear_escalation_timers(o);
            o->stable_time_s = 0.0f;
        }
        break;

    case SLIP_STATE_RECOVERY:
        if (o->score >= SLIP_SLIP_SCORE && o->slip_time_s >= 0.35f) {
            o->state = SLIP_STATE_SLIP;
            o->stable_time_s = 0.0f;
        } else if (o->score >= SLIP_SUSPECT_SCORE &&
                   o->suspect_time_s >= SLIP_SUSPECT_HOLD_S) {
            o->state = SLIP_STATE_SUSPECT;
            o->stable_time_s = 0.0f;
        } else if (o->score <= SLIP_RECOVER_NORMAL_SCORE) {
            if (o->stable_time_s >= SLIP_RECOVER_NORMAL_HOLD_S) {
                o->state = SLIP_STATE_NORMAL;
                clear_escalation_timers(o);
                o->stable_time_s = 0.0f;
            }
        } else {
            o->stable_time_s = 0.0f;
        }
        break;

    default:
        slip_observer_init(o);
        break;
    }
}

SlipState slip_observer_get_state(const SlipObserver *o)
{
    return o ? o->state : SLIP_STATE_NORMAL;
}

float slip_observer_get_score(const SlipObserver *o)
{
    return o ? o->score : 0.0f;
}

float slip_observer_wheel_sigma_scale(const SlipObserver *o)
{
    if (!o) return 1.0f;
    switch (o->state) {
    case SLIP_STATE_SUSPECT: return 2.5f;
    case SLIP_STATE_SLIP: return 7.0f;
    case SLIP_STATE_SEVERE: return 10.0f;
    case SLIP_STATE_RECOVERY: return 3.0f;
    case SLIP_STATE_NORMAL:
    default: return 1.0f;
    }
}

float slip_observer_nhc_sigma_scale(const SlipObserver *o)
{
    if (!o) return 1.0f;
    switch (o->state) {
    case SLIP_STATE_SUSPECT:
        return 1.0f;
    case SLIP_STATE_SLIP:
        return o->lateral_evidence > 0.30f ? 4.0f : 2.0f;
    case SLIP_STATE_SEVERE:
        return o->lateral_evidence > 0.30f ? 8.0f : 3.0f;
    case SLIP_STATE_RECOVERY:
        return 1.5f;
    case SLIP_STATE_NORMAL:
    default:
        return 1.0f;
    }
}

int slip_observer_reject_wheel(const SlipObserver *o)
{
    return o && o->state == SLIP_STATE_SEVERE;
}
