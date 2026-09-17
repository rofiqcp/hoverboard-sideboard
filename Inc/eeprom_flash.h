#ifndef EEPROM_FLASH_H
#define EEPROM_FLASH_H

#include <stdint.h>

#define CAL_FLAG_STILL_VALID   (1UL << 0)
#define CAL_FLAG_ROTATE_VALID  (1UL << 1)
#define CAL_FLAG_MOUNT_VALID   (1UL << 2)
#define CAL_FLAG_THERMAL_VALID (1UL << 3)

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t length;
    float gyro_bias[3];
    float accel_offset[3];
    float accel_transform[9];       /* row-major: a_corr = T * (a_raw-offset). */
    float sensor_to_body_q[4];      /* quaternion sensor -> body, wxyz. */
    float gyro_temp_slope[3];       /* rad/s/degC relative calibration_temp_c. */
    float accel_temp_slope[3];      /* m/s^2/degC before accel transform. */
    float imu_position_body[3];     /* IMU lever arm from body origin, meter. */
    float gyro_noise;
    float accel_process_noise;
    float gyro_bias_walk;
    float accel_bias_walk;
    float accel_dir_noise;
    float gyro_std[3];
    float accel_std[3];
    float calibration_temp_c;
    float still_gyro_std_max_dps;
    float still_accel_std_max_g;
    float rotate_residual_rms_g;
    float rotate_residual_max_g;
    uint32_t calibration_flags;
    uint32_t still_cal_count;
    uint32_t rotate_cal_count;
    uint32_t save_count;
    uint16_t crc16;
    uint16_t reserved;
} PersistedSettings;

void eeprom_settings_defaults(PersistedSettings *s);
int eeprom_settings_load(PersistedSettings *s);
int eeprom_settings_save(PersistedSettings *s);

#endif
