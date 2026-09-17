#ifndef IMU_MPU6XXX_H
#define IMU_MPU6XXX_H

#include <stdint.h>

typedef enum {
    IMU_CLASS_UNKNOWN = 0,
    IMU_CLASS_MPU6050 = 1,
    IMU_CLASS_MPU65XX_COMPAT = 2
} ImuDeviceClass;

typedef struct {
    uint8_t whoami;
    ImuDeviceClass device_class;
    uint16_t sample_hz;
    uint8_t fifo_packet_bytes;
    uint8_t accel_config2_supported;
    float temperature_lsb_per_c;
    float temperature_offset_c;
} ImuDeviceInfo;

typedef struct {
    int16_t accel_raw[3];
    int16_t temp_raw;
    int16_t gyro_raw[3];
    float accel_mps2[3];
    float gyro_rads[3];
    float temperature_c;
    uint32_t sample_time_us;
    uint32_t sample_index;
} ImuSample;

typedef struct {
    uint32_t fifo_read_count;
    uint32_t fifo_sample_count;
    uint32_t fifo_empty_count;
    uint32_t fifo_overflow_count;
    uint32_t fifo_resync_count;
    uint16_t fifo_max_bytes;
    uint32_t rate_window_start_us;
    uint32_t rate_window_samples;
    float observed_sample_hz;
} ImuFifoStats;

int imu_mpu6xxx_init(void);
int imu_mpu6xxx_read(ImuSample *sample);
int imu_mpu6xxx_read_fifo(ImuSample *samples, uint8_t max_samples, uint8_t *out_count);
int imu_mpu6xxx_fifo_reset(void);
uint8_t imu_mpu6xxx_whoami(void);
const ImuDeviceInfo *imu_mpu6xxx_get_info(void);
const ImuFifoStats *imu_mpu6xxx_get_fifo_stats(void);

#endif
