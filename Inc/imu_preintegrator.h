#ifndef IMU_PREINTEGRATOR_H
#define IMU_PREINTEGRATOR_H

#include <stdint.h>

typedef struct {
    float delta_angle[3];
    float delta_velocity[3];
    float last_delta_angle[3];
    float last_gyro[3];
    float last_accel[3];
    float dt_sum;
    uint16_t sample_count;
    uint8_t have_previous;
} ImuPreintegrator;

typedef struct {
    float delta_angle[3];
    float delta_velocity[3];
    float dt;
    uint16_t sample_count;
} ImuDelta;

void imu_preintegrator_init(ImuPreintegrator *p);
void imu_preintegrator_reset(ImuPreintegrator *p);
void imu_preintegrator_push(ImuPreintegrator *p,
                            const float gyro_rads[3],
                            const float accel_mps2[3],
                            float dt);
int imu_preintegrator_take(ImuPreintegrator *p, ImuDelta *out);

#endif
