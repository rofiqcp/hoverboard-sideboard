#include "imu_preintegrator.h"

#include <math.h>
#include <string.h>

#define PREINT_SMALL_ANGLE2 0.0025f

static int vec3_is_finite(const float v[3])
{
    return isfinite(v[0]) && isfinite(v[1]) && isfinite(v[2]);
}

static void cross3(const float a[3], const float b[3], float out[3])
{
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}

static void quat_normalize(float q[4])
{
    float n2 = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];
    if (!isfinite(n2) || n2 < 1.0e-12f) {
        q[0] = 1.0f;
        q[1] = q[2] = q[3] = 0.0f;
        return;
    }
    float inv_n = 1.0f / sqrtf(n2);
    for (int i = 0; i < 4; i++) q[i] *= inv_n;
}

static void quat_from_rotvec(const float dtheta[3], float q[4])
{
    float a2 = dtheta[0]*dtheta[0] + dtheta[1]*dtheta[1] + dtheta[2]*dtheta[2];
    if (a2 < PREINT_SMALL_ANGLE2) {
        float a4 = a2*a2;
        float s = 0.5f - a2*(1.0f/48.0f) + a4*(1.0f/3840.0f);
        q[0] = 1.0f - a2*(1.0f/8.0f) + a4*(1.0f/384.0f);
        q[1] = s*dtheta[0];
        q[2] = s*dtheta[1];
        q[3] = s*dtheta[2];
    } else {
        float a = sqrtf(a2);
        float s = sinf(0.5f*a) / a;
        q[0] = cosf(0.5f*a);
        q[1] = s*dtheta[0];
        q[2] = s*dtheta[1];
        q[3] = s*dtheta[2];
    }
    quat_normalize(q);
}

static void quat_multiply(const float a[4], const float b[4], float out[4])
{
    float q[4];
    q[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
    q[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
    q[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
    q[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
    memcpy(out, q, sizeof(q));
}

static void quat_rotate(const float q[4], const float v[3], float out[3])
{
    const float qv[3] = {q[1], q[2], q[3]};
    float c1[3], c2[3];
    cross3(qv, v, c1);
    cross3(qv, c1, c2);
    for (int i = 0; i < 3; i++)
        out[i] = v[i] + 2.0f*(q[0]*c1[i] + c2[i]);
}

static void quat_to_rotvec(const float q_in[4], float dtheta[3])
{
    float q[4];
    memcpy(q, q_in, sizeof(q));
    quat_normalize(q);
    if (q[0] < 0.0f)
        for (int i = 0; i < 4; i++) q[i] = -q[i];

    float vn = sqrtf(q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    float scale = 2.0f;
    if (vn > 1.0e-7f)
        scale = 2.0f * atan2f(vn, q[0]) / vn;
    dtheta[0] = scale*q[1];
    dtheta[1] = scale*q[2];
    dtheta[2] = scale*q[3];
}

static void quat_half(const float q_in[4], float half[4])
{
    float q[4];
    memcpy(q, q_in, sizeof(q));
    quat_normalize(q);
    if (q[0] < 0.0f)
        for (int i = 0; i < 4; i++) q[i] = -q[i];

    half[0] = sqrtf(fmaxf(0.0f, 0.5f*(1.0f + q[0])));
    if (half[0] > 1.0e-6f) {
        float s = 0.5f / half[0];
        half[1] = q[1]*s;
        half[2] = q[2]*s;
        half[3] = q[3]*s;
    } else {
        float dtheta[3];
        quat_to_rotvec(q, dtheta);
        for (int i = 0; i < 3; i++) dtheta[i] *= 0.5f;
        quat_from_rotvec(dtheta, half);
    }
    quat_normalize(half);
}

static void clear_accumulator(ImuPreintegrator *p)
{
    p->delta_quat[0] = 1.0f;
    p->delta_quat[1] = p->delta_quat[2] = p->delta_quat[3] = 0.0f;
    memset(p->delta_velocity_start, 0, sizeof(p->delta_velocity_start));
    p->dt_sum = 0.0f;
    p->sample_count = 0U;
}

void imu_preintegrator_init(ImuPreintegrator *p)
{
    if (!p) return;
    memset(p, 0, sizeof(*p));
    p->delta_quat[0] = 1.0f;
}

void imu_preintegrator_reset(ImuPreintegrator *p)
{
    if (!p) return;
    float last_g[3], last_a[3];
    uint8_t have = p->have_previous;
    memcpy(last_g, p->last_gyro, sizeof(last_g));
    memcpy(last_a, p->last_accel, sizeof(last_a));
    memset(p, 0, sizeof(*p));
    p->delta_quat[0] = 1.0f;
    p->have_previous = have;
    memcpy(p->last_gyro, last_g, sizeof(last_g));
    memcpy(p->last_accel, last_a, sizeof(last_a));
}

void imu_preintegrator_push(ImuPreintegrator *p,
                            const float gyro[3], const float accel[3], float dt)
{
    if (!p || !gyro || !accel || !isfinite(dt) || dt <= 0.0f || dt > 0.05f ||
        !vec3_is_finite(gyro) || !vec3_is_finite(accel)) {
        return;
    }

    if (!p->have_previous) {
        memcpy(p->last_gyro, gyro, sizeof(p->last_gyro));
        memcpy(p->last_accel, accel, sizeof(p->last_accel));
        p->have_previous = 1U;
        return;
    }

    float da[3];
    for (int i = 0; i < 3; i++)
        da[i] = 0.5f*(p->last_gyro[i] + gyro[i])*dt;

    /*
     * Quaternion composition already contributes the BCH 1/2 cross-term
     * between accumulated and current rotations. For trapezoidal rate samples,
     * the classic streaming coning correction has one additional 1/12 term
     * from the immediately preceding delta-angle. Add only that residual here
     * to avoid double-counting the non-commutative rotation already represented
     * by quaternion multiplication.
     */
    float coning_extra[3], da_corrected[3];
    cross3(p->last_delta_angle, da, coning_extra);
    for (int i = 0; i < 3; i++)
        da_corrected[i] = da[i] + coning_extra[i]*(1.0f/12.0f);

    float dq[4], q_next[4];
    quat_from_rotvec(da_corrected, dq);
    quat_multiply(p->delta_quat, dq, q_next);
    quat_normalize(q_next);

    /*
     * Integrasikan specific-force pada body frame awal window. Acceleration
     * endpoint lama diputar dengan attitude relatif lama, endpoint baru dengan
     * attitude relatif baru; trapezoid setelah transform menangani sculling
     * akibat body rotation tanpa mengasumsikan komponen accel berada pada frame
     * yang sama.
     */
    float a0_start[3], a1_start[3];
    quat_rotate(p->delta_quat, p->last_accel, a0_start);
    quat_rotate(q_next, accel, a1_start);
    for (int i = 0; i < 3; i++)
        p->delta_velocity_start[i] += 0.5f*(a0_start[i] + a1_start[i])*dt;

    memcpy(p->delta_quat, q_next, sizeof(p->delta_quat));
    memcpy(p->last_delta_angle, da, sizeof(p->last_delta_angle));
    p->dt_sum += dt;
    if (p->sample_count < UINT16_MAX) p->sample_count++;

    memcpy(p->last_gyro, gyro, sizeof(p->last_gyro));
    memcpy(p->last_accel, accel, sizeof(p->last_accel));
}

int imu_preintegrator_take(ImuPreintegrator *p, ImuDelta *out)
{
    if (!p || !out || p->sample_count == 0U || !isfinite(p->dt_sum) ||
        p->dt_sum <= 0.0f) {
        return 0;
    }

    quat_to_rotvec(p->delta_quat, out->delta_angle);

    /*
     * ESKF memutar delta_velocity memakai attitude midpoint. Ubah integral yang
     * saat ini berada pada body awal ke body midpoint:
     * dv_mid = R(start<-mid)^T * dv_start.
     */
    float q_half[4], q_half_conj[4];
    quat_half(p->delta_quat, q_half);
    q_half_conj[0] = q_half[0];
    q_half_conj[1] = -q_half[1];
    q_half_conj[2] = -q_half[2];
    q_half_conj[3] = -q_half[3];
    quat_rotate(q_half_conj, p->delta_velocity_start, out->delta_velocity);

    out->dt = p->dt_sum;
    out->sample_count = p->sample_count;

    clear_accumulator(p);
    return 1;
}
