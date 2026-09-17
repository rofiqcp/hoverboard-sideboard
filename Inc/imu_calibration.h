#ifndef IMU_CALIBRATION_H
#define IMU_CALIBRATION_H

#include <stdint.h>
#include "imu_mpu6xxx.h"
#include "eeprom_flash.h"

typedef enum {
    IMU_CAL_IDLE=0,
    IMU_CAL_STILL=1,
    IMU_CAL_ROTATE=2,
    IMU_CAL_DONE=3,
    IMU_CAL_FAILED=4
} ImuCalState;

typedef struct {
    ImuCalState state;
    uint8_t coverage;
    uint16_t progress;
    uint32_t samples;
    uint32_t total_seen;
    float sum_g[3], sum_g2[3];
    float sum_a[3], sum_a2[3];
    float sum_temp;
    float face_sum[6][3];
    uint16_t face_count[6];
    uint8_t event_saved_needed;
    uint8_t error_code;
} ImuCalibration;

void imu_calibration_init(ImuCalibration *c);
void imu_calibration_start_still(ImuCalibration *c);
void imu_calibration_start_rotate(ImuCalibration *c);
int imu_calibration_finish_rotate(ImuCalibration *c, PersistedSettings *s);
void imu_calibration_cancel(ImuCalibration *c);
void imu_calibration_update(ImuCalibration *c, const ImuSample *raw, PersistedSettings *s);
void imu_apply_static_calibration(const ImuSample *raw, const PersistedSettings *s,
                                  float accel_mps2[3], float gyro_rads[3]);

#endif
