#ifndef IMU_PREINTEGRATOR_FIXED_H
#define IMU_PREINTEGRATOR_FIXED_H
#include <stdint.h>
#include "imu_mpu6xxx.h"
#include "eeprom_flash.h"
#include "imu_preintegrator.h"

typedef struct {
    int32_t gyro_bias_q20[3];
    int32_t accel_offset_q16[3];
    int32_t accel_gain_q24[3];
    int32_t last_gyro_q20[3];
    int32_t last_accel_q16[3];
    int32_t delta_angle_q20[3];
    int32_t delta_velocity_q16[3];
    int32_t last_delta_angle_q20[3];
    uint16_t sample_count;
    uint8_t have_previous;
} ImuPreintegratorFixed;

void imu_preintegrator_fixed_init(ImuPreintegratorFixed *p,
                                  const PersistedSettings *settings);
void imu_preintegrator_fixed_push(ImuPreintegratorFixed *p,
                                  const ImuSample *raw);
int imu_preintegrator_fixed_take(ImuPreintegratorFixed *p, ImuDelta *out);
#endif
