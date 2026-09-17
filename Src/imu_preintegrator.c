#include "imu_preintegrator.h"
#include <string.h>

static void cross3(const float a[3], const float b[3], float out[3])
{
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}

void imu_preintegrator_init(ImuPreintegrator *p)
{
    if (p) memset(p, 0, sizeof(*p));
}

void imu_preintegrator_reset(ImuPreintegrator *p)
{
    if (!p) return;
    float last_g[3], last_a[3];
    uint8_t have = p->have_previous;
    memcpy(last_g, p->last_gyro, sizeof(last_g));
    memcpy(last_a, p->last_accel, sizeof(last_a));
    memset(p, 0, sizeof(*p));
    p->have_previous = have;
    memcpy(p->last_gyro, last_g, sizeof(last_g));
    memcpy(p->last_accel, last_a, sizeof(last_a));
}

void imu_preintegrator_push(ImuPreintegrator *p,
                            const float gyro[3], const float accel[3], float dt)
{
    if (!p || !gyro || !accel || dt <= 0.0f || dt > 0.05f) return;
    if (!p->have_previous) {
        memcpy(p->last_gyro, gyro, sizeof(p->last_gyro));
        memcpy(p->last_accel, accel, sizeof(p->last_accel));
        p->have_previous = 1U;
        return;
    }

    float da[3], dv[3], base[3], coning[3];
    for (int i = 0; i < 3; i++) {
        da[i] = 0.5f * (p->last_gyro[i] + gyro[i]) * dt;
        dv[i] = 0.5f * (p->last_accel[i] + accel[i]) * dt;
        base[i] = p->delta_angle[i] + p->last_delta_angle[i] * (1.0f / 6.0f);
    }

    /* Koreksi coning streaming mengikuti bentuk yang dipakai backend ArduPilot. */
    cross3(base, da, coning);
    for (int i = 0; i < 3; i++) {
        p->delta_angle[i] += da[i] + 0.5f * coning[i];
        p->delta_velocity[i] += dv[i];
        p->last_delta_angle[i] = da[i];
    }
    p->dt_sum += dt;
    p->sample_count++;
    memcpy(p->last_gyro, gyro, sizeof(p->last_gyro));
    memcpy(p->last_accel, accel, sizeof(p->last_accel));
}

int imu_preintegrator_take(ImuPreintegrator *p, ImuDelta *out)
{
    if (!p || !out || p->sample_count == 0U || p->dt_sum <= 0.0f) return 0;

    memcpy(out->delta_angle, p->delta_angle, sizeof(out->delta_angle));
    memcpy(out->delta_velocity, p->delta_velocity, sizeof(out->delta_velocity));
    out->dt = p->dt_sum;
    out->sample_count = p->sample_count;

    memset(p->delta_angle, 0, sizeof(p->delta_angle));
    memset(p->delta_velocity, 0, sizeof(p->delta_velocity));
    /* last_delta_angle sengaja dipertahankan lintas output 100 Hz.
     * Koreksi coning membutuhkan delta sampel sebelumnya walau accumulator
     * utama baru saja dipublish. History hanya dibuang saat reset/gap. */
    p->dt_sum = 0.0f;
    p->sample_count = 0U;
    return 1;
}
