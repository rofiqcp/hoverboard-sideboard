#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdint.h>

/* Peta flash STM32F103C8 64 KB.
 * 0x08000000..0x080017FF : bootloader 6 KB
 * 0x08001800..0x0800FBFF : aplikasi 57 KB
 * 0x0800FC00..0x0800FFFF : EEPROM emulasi 1 KB
 */
#define APP_FLASH_START             0x08001800UL
#define EEPROM_FLASH_ADDR           0x0800FC00UL
#define FLASH_END_ADDR              0x08010000UL
#define EEPROM_FLASH_PAGE_SIZE      1024UL

/* Konfigurasi komunikasi serial utama ke CH340/NUC. */
#define APP_UART_BAUD               921600UL
#define IMU_LOOP_RATE_HZ            100UL
#define IMU_LOOP_PERIOD_US          (1000000UL / IMU_LOOP_RATE_HZ)
#define TELEMETRY_RATE_HZ            50UL
#define TELEMETRY_DIVIDER           (IMU_LOOP_RATE_HZ / TELEMETRY_RATE_HZ)

/* Konfigurasi MPU6xxx. Sensor internal 200 Hz masuk FIFO, ESKF tetap 100 Hz.
 * HAL STM32 memakai alamat I2C 8-bit. */
#define MPU6XXX_I2C_ADDR            (0x68U << 1)
#define MPU6XXX_SAMPLE_HZ           200UL
#define MPU6XXX_ACCEL_LSB_PER_G      8192.0f
#define MPU6XXX_GYRO_LSB_PER_DPS       65.5f
#define IMU_REINIT_ERROR_COUNT          20U
#define GRAVITY_MPS2                   9.80665f

/* Batas koreksi accelerometer. Data di luar batas dianggap sedang mendapat
 * percepatan linear/vibrasi kuat sehingga tidak dipakai untuk koreksi gravitasi. */
#define ESKF_ACCEL_GATE_MIN_G          0.85f
#define ESKF_ACCEL_GATE_MAX_G          1.15f
#define ESKF_MOVING_ACCEL_GATE_MIN_G   0.94f
#define ESKF_MOVING_ACCEL_GATE_MAX_G   1.06f
#define ESKF_MOVING_GRAVITY_NOISE_SCALE 6.0f
#define ESKF_GYRO_NOISE_RAD            0.018f
#define ESKF_ACCEL_PROCESS_NOISE        0.18f
#define ESKF_GYRO_BIAS_WALK_RAD        0.0010f
#define ESKF_ACCEL_BIAS_WALK           0.010f
#define ESKF_ACCEL_DIR_NOISE           0.060f
#define ESKF_ACCEL_NIS_GATE             16.0f
#define ESKF_GRAVITY_SCALAR_NIS_GATE      9.0f
#define ESKF_COVARIANCE_DIVIDER           2U

/* Kalibrasi awal 2 detik pada 100 Hz. Jika board diam, bias gyro hasil rata-rata
 * dipakai pada sesi ini. Jika EEPROM belum valid, hasil ini juga disimpan sekali. */
#define STARTUP_CALIB_SECONDS            2U
#define STARTUP_CALIB_SAMPLES          (MPU6XXX_SAMPLE_HZ * STARTUP_CALIB_SECONDS)
#define STARTUP_STILL_GYRO_DPS           3.0f
#define STARTUP_STILL_ACCEL_ERR_G         0.15f
#define STARTUP_STILL_GYRO_STD_DPS         0.50f
#define STARTUP_STILL_ACCEL_STD_G           0.06f
#define MASTER_STILL_CAL_SAMPLES          600U
#define STILL_CAL_GYRO_STD_MAX_DPS          0.35f
#define STILL_CAL_ACCEL_STD_MAX_G            0.04f
#define STILL_CAL_ACCEL_NORM_ERR_G           0.08f
#define ROTATE_FACE_MIN_SAMPLES              100U
#define ROTATE_FACE_AXIS_MIN_G                0.80f
#define ROTATE_FACE_OTHER_MAX_G               0.45f
#define ROTATE_CAL_MIN_SPAN_G                 1.50f
#define ROTATE_CAL_MAX_OFFSET_G               0.35f
#define ROTATE_CAL_SCALE_MIN                  0.80f
#define ROTATE_CAL_SCALE_MAX                  1.20f
#define ROTATE_CAL_RMS_MAX_G                  0.06f
#define ROTATE_CAL_MAX_ERROR_G                0.10f
#define ZUPT_SIGMA_MPS                      0.03f
#define ZERO_RATE_SIGMA_RAD                 0.008f

/* ID payload privat di dalam framing VESC. Framing dan CRC tetap kompatibel VESC. */
#define COMM_SIDEBOARD_IMU             0xF0U
#define COMM_SIDEBOARD_BOOTLOADER      0xF1U
#define COMM_SIDEBOARD_CALIBRATION     0xF2U
#define COMM_GET_IMU_DATA              65U
#define BOOTLOADER_REQUEST_MAGIC       0xB007U
#define IMU_PROTOCOL_VERSION           2U

/* Flag telemetry supaya host tahu kualitas data. */
#define IMU_FLAG_SENSOR_OK             (1U << 0)
#define IMU_FLAG_ESKF_OK               (1U << 1)
#define IMU_FLAG_ACCEL_FUSED           (1U << 2)
#define IMU_FLAG_EEPROM_VALID          (1U << 3)
#define IMU_FLAG_STARTUP_STILL         (1U << 4)
/* Bit 5 sengaja selalu 0: yaw MPU6xxx tidak absolut tanpa magnetometer/GNSS. */
#define IMU_FLAG_YAW_ABSOLUTE          (1U << 5)
#define IMU_FLAG_STILL_CAL_VALID       (1U << 6)
#define IMU_FLAG_ROTATE_CAL_VALID      (1U << 7)
#define IMU_FLAG_CAL_ACTIVE            (1U << 8)
#define IMU_FLAG_ZUPT_APPLIED          (1U << 9)
#define IMU_FLAG_MASTER_STATIONARY     (1U << 10)

#endif
