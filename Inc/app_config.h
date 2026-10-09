#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdint.h>

/* Peta flash STM32F103C8 64 KB.
 * 0x08000000..0x080017FF : bootloader 6 KB
 * 0x08001800..0x0800F3FF : aplikasi 55 KB
 * 0x0800F400..0x0800F7FF : settings journal A 1 KB
 * 0x0800F800..0x0800FBFF : manifest validitas aplikasi 1 KB (bootloader)
 * 0x0800FC00..0x0800FFFF : settings journal B / legacy EEPROM 1 KB
 */
#define APP_FLASH_START             0x08001800UL
#define EEPROM_FLASH_ADDR_A         0x0800F400UL
#define EEPROM_FLASH_ADDR_B         0x0800FC00UL
#define EEPROM_FLASH_ADDR           EEPROM_FLASH_ADDR_B /* legacy schema location */
#define FLASH_END_ADDR              0x08010000UL
#define EEPROM_FLASH_PAGE_SIZE      1024UL

/* Konfigurasi komunikasi serial utama ke CH340/NUC. */
#define APP_UART_BAUD               921600UL
/* 0=validated 500Hz baseline, 1=1kHz/200Hz/100Hz hardware-test candidate.
 * Do not activate the 1kHz build on a vehicle until deadline/FIFO/I2C
 * counters pass the real-board gate. */
#ifndef IMU_PROFILE_1KHZ
#define IMU_PROFILE_1KHZ             0U
#endif
#if IMU_PROFILE_1KHZ
#define IMU_LOOP_RATE_HZ            200UL
#define TELEMETRY_RATE_HZ           100UL
#define IMU_FIFO_BATCH_CAPACITY      16U
#else
#define IMU_LOOP_RATE_HZ            100UL
#define TELEMETRY_RATE_HZ            50UL
#define IMU_FIFO_BATCH_CAPACITY       8U
#endif
#define IMU_LOOP_PERIOD_US          (1000000UL / IMU_LOOP_RATE_HZ)
#define TELEMETRY_DIVIDER           (IMU_LOOP_RATE_HZ / TELEMETRY_RATE_HZ)
#define IMU_GRAVITY_DIVIDER         (IMU_LOOP_RATE_HZ / 50UL)
#define IMU_ZUPT_DIVIDER            (IMU_LOOP_RATE_HZ / 20UL)
#define IMU_TEMP_READ_DIVIDER       (IMU_LOOP_RATE_HZ / 10UL)
#if (1000000UL % IMU_LOOP_RATE_HZ) != 0 || (IMU_LOOP_RATE_HZ % TELEMETRY_RATE_HZ) != 0 || (IMU_LOOP_RATE_HZ % 50UL) != 0 || (IMU_LOOP_RATE_HZ % 20UL) != 0
#error "IMU scheduler, telemetry and estimator correction rates must divide exactly"
#endif

/* Konfigurasi MPU6xxx. Semua sampel FIFO masuk preintegrasi, lalu ESKF
 * pada frekuensi service 100/200 Hz. DLPF aktif: output base 1 kHz, divider
 * 1 menghasilkan 500 Hz atau divider 0 menghasilkan 1000 Hz.
 * HAL STM32 menggunakan alamat I2C 8-bit. */
#define MPU6XXX_I2C_ADDR            (0x68U << 1)
#define MPU6XXX_HIGHSPEED_PROFILE       1U
#if MPU6XXX_HIGHSPEED_PROFILE
#if IMU_PROFILE_1KHZ
#define MPU6XXX_SAMPLE_HZ          1000UL
#else
#define MPU6XXX_SAMPLE_HZ           500UL
#endif
#define MPU6XXX_ACCEL_LSB_PER_G      4096.0f  /* +/-8 g */
#define MPU6XXX_GYRO_LSB_PER_DPS       32.8f  /* +/-1000 deg/s */
#define MPU6XXX_DLPF_CFG                2U    /* ~92-98 Hz class bandwidth */
#define MPU6XXX_GYRO_CONFIG_VALUE    0x10U
#define MPU6XXX_ACCEL_CONFIG_VALUE   0x10U
#else
#define MPU6XXX_SAMPLE_HZ           200UL
#define MPU6XXX_ACCEL_LSB_PER_G      8192.0f  /* +/-4 g */
#define MPU6XXX_GYRO_LSB_PER_DPS       65.5f  /* +/-500 deg/s */
#define MPU6XXX_DLPF_CFG                3U    /* ~42-44 Hz class bandwidth */
#define MPU6XXX_GYRO_CONFIG_VALUE    0x08U
#define MPU6XXX_ACCEL_CONFIG_VALUE   0x08U
#endif
#if IMU_PROFILE_1KHZ && !MPU6XXX_HIGHSPEED_PROFILE
#error "1kHz requires the high-speed MPU6xxx range and DLPF profile"
#endif
#if (1000UL % MPU6XXX_SAMPLE_HZ) != 0 || (MPU6XXX_SAMPLE_HZ % IMU_LOOP_RATE_HZ) != 0
#error "MPU6xxx FIFO rate must divide the DLPF base rate and service frequency"
#endif
#if IMU_FIFO_BATCH_CAPACITY < (MPU6XXX_SAMPLE_HZ / IMU_LOOP_RATE_HZ)
#error "FIFO buffer cannot accommodate one nominal service interval"
#endif
#define IMU_REINIT_ERROR_COUNT          20U
#define GRAVITY_MPS2                   9.80665f

/* Batas koreksi accelerometer. Data di luar batas dianggap sedang mendapat
 * percepatan linear/vibrasi kuat sehingga tidak dipakai untuk koreksi gravitasi. */
#define ESKF_ACCEL_GATE_MIN_G          0.85f
#define ESKF_ACCEL_GATE_MAX_G          1.15f
#define ESKF_GYRO_NOISE_RAD            0.018f
#define ESKF_ACCEL_PROCESS_NOISE        0.18f
#define ESKF_GYRO_BIAS_WALK_RAD        0.0010f
#define ESKF_ACCEL_BIAS_WALK           0.010f
#define ESKF_ACCEL_DIR_NOISE           0.060f
#define ESKF_NIS_GATE_1D                  9.0f
#define ESKF_NIS_GATE_2D                 11.829007f
#define ESKF_NIS_GATE_3D                 14.156253f
#define ESKF_GRAVITY_NIS_GATE_2D         ESKF_NIS_GATE_2D
#define ESKF_GRAVITY_DIR_COS_STILL          0.99756405f /* 4 deg */
#define ESKF_COVARIANCE_DIVIDER           1U

/* Kalibrasi awal 2 detik pada 100 Hz. Jika board diam, bias gyro hasil rata-rata
 * dipakai pada sesi ini. Jika EEPROM belum valid, hasil ini juga disimpan sekali. */
#define STARTUP_CALIB_SECONDS            2U
#define STARTUP_CALIB_SAMPLES          (MPU6XXX_SAMPLE_HZ * STARTUP_CALIB_SECONDS)
#define STARTUP_STILL_GYRO_DPS           3.0f
#define STARTUP_STILL_ACCEL_ERR_G         0.15f
#define STARTUP_STILL_GYRO_STD_DPS         0.50f
#define STARTUP_STILL_ACCEL_STD_G           0.06f
#define MASTER_STILL_CAL_SAMPLES          (MPU6XXX_SAMPLE_HZ * 10U) /* 10 s */
#define STILL_CAL_GYRO_STD_MAX_DPS          0.35f
#define STILL_CAL_ACCEL_STD_MAX_G            0.04f
#define STILL_CAL_ACCEL_NORM_ERR_G           0.08f
#define ROTATE_FACE_MIN_SAMPLES           (MPU6XXX_SAMPLE_HZ * 1U)  /* 1 s/face */
#define ROTATE_FACE_STABLE_SAMPLES        (MPU6XXX_SAMPLE_HZ / 5U)  /* 0.2 s dwell */
#define ROTATE_FACE_AXIS_MIN_G                0.80f
#define ROTATE_FACE_OTHER_MAX_G               0.45f
#define ROTATE_CAL_MIN_SPAN_G                 1.50f
#define ROTATE_CAL_MAX_OFFSET_G               0.35f
#define ROTATE_CAL_SCALE_MIN                  0.80f
#define ROTATE_CAL_SCALE_MAX                  1.20f
#define ROTATE_CAL_RMS_MAX_G                  0.06f
#define ROTATE_CAL_MAX_ERROR_G                0.10f
#define ROTATE_CAL_COND_FRO_MAX                 6.0f
#define ZUPT_SIGMA_MPS                      0.03f
#define ZERO_NAV_POSITION_SIGMA_M             0.01f
/* ZUPT startup bersifat one-shot. Setelah gerak pertama, re-arm hanya melalui
 * master STATIONARY_ON/ZUPT/zero-nav karena IMU 6-axis tidak dapat membedakan
 * diam dari gerak konstan tanpa referensi kecepatan eksternal. */
#define MOTION_AID_RELEASE_SPEED_MPS           0.05f
#define MASTER_STATIONARY_LEASE_US          2000000UL /* 2 s; renew only when still */
#define ZERO_RATE_SIGMA_RAD                 0.008f
#define NHC_SIGMA_MPS                         0.05f
/* Local MCU ESKF does not rewind history. Freshness is therefore source-specific;
 * delayed/global position belongs on the NUC map->odom estimator (or a future
 * history-buffer estimator), not by inflating sigma on an old MCU sample. */
#define AID_MAX_AGE_US                      200000UL /* legacy compatibility only */
#define AID_WHEEL_MAX_AGE_US                 15000UL
#define AID_WHEEL_HISTORY_SAMPLES                 3U
#define AID_WHEEL_COMP_MIN_AGE_US              2000UL
#define AID_WHEEL_HISTORY_MAX_GAP_US          25000UL
#define AID_YAW_MAX_AGE_US                   50000UL
#define AID_WORLD_VEL_MAX_AGE_US             50000UL
#define AID_WORLD_POS_MAX_AGE_US             40000UL
#define AID_STATUS_TIMEOUT_US               500000UL
#define AID_REACQUIRE_COUNT                      3U
#define AID_REACQUIRE_POS_FLOOR_M             0.75f
#define AID_REACQUIRE_POS_SIGMA_MULT           4.0f
#define AID_REACQUIRE_YAW_FLOOR_RAD          0.17453293f
#define AID_REACQUIRE_YAW_SIGMA_MULT           4.0f
#define ESKF_PSD_CHECK_INTERVAL_US           100000UL
#define ESKF_IMU_GAP_NOISE_SCALE                10.0f
#define ESKF_IMU_GAP_MIN_US                    20000UL
#define ESKF_CLIP_NOISE_SCALE                     25.0f
#define DIAG_STREAM_DIVIDER                     10U
#define AID_WHEEL_MAX_MPS                      15.0f
#define AID_WORLD_VEL_MAX_MPS                  30.0f
#define AID_WORLD_POS_MAX_M                 100000.0f
#define AID_SIGMA_VEL_MIN_MPS                   0.01f
#define AID_SIGMA_VEL_MAX_MPS                   10.0f
#define AID_SIGMA_POS_MIN_M                     0.02f
#define AID_SIGMA_POS_MAX_M                    100.0f
#define AID_SIGMA_YAW_MIN_RAD                   0.005f
#define AID_SIGMA_YAW_MAX_RAD                   1.0f
#define AID_WHEEL_INNOV_MAX_MPS                 3.0f
#define AID_WORLD_VEL_INNOV_MAX_MPS            10.0f
#define AID_WORLD_POS_INNOV_MAX_M              50.0f
#define AID_YAW_INNOV_MAX_RAD                   1.04719755f
#define AID_YAW_NIS_GATE                         36.0f
#define ESKF_GYRO_BIAS_LIMIT_RAD                0.05f
#define ESKF_ACCEL_BIAS_LIMIT_MPS2              1.5f
#define ESKF_GYRO_BIAS_STEP_MAX_RAD             0.003f
#define ESKF_ACCEL_BIAS_STEP_MAX_MPS2           0.10f
#define ESKF_ATTITUDE_STEP_MAX_RAD               0.25f
#define ESKF_VELOCITY_STEP_MAX_MPS               5.0f
#define ESKF_POSITION_STEP_MAX_M                25.0f
#define ESKF_HEALTH_VELOCITY_MAX_MPS           100.0f
#define ESKF_HEALTH_POSITION_MAX_M          1000000.0f

/* ID payload privat di dalam framing VESC. Framing dan CRC tetap kompatibel VESC. */
#define COMM_SIDEBOARD_IMU             0xF0U
#define COMM_SIDEBOARD_BOOTLOADER      0xF1U
#define COMM_SIDEBOARD_CALIBRATION     0xF2U
#define COMM_SIDEBOARD_CONFIG          0xF3U
#define COMM_SIDEBOARD_AIDING          0xF4U
#define COMM_SIDEBOARD_DIAG            0xF5U
#define COMM_SIDEBOARD_FAST_NAV        0xF6U /* reserved; optional stream not enabled */
#define COMM_GET_IMU_DATA              65U
#define BOOTLOADER_REQUEST_MAGIC       0xB007U
#define IMU_PROTOCOL_VERSION           6U

/* Flag telemetry supaya host tahu kualitas data. */
#define IMU_FLAG_SENSOR_OK             (1U << 0)
#define IMU_FLAG_ESKF_OK               (1U << 1)
#define IMU_FLAG_ACCEL_FUSED           (1U << 2)
#define IMU_FLAG_EEPROM_VALID          (1U << 3)
#define IMU_FLAG_STARTUP_STILL         (1U << 4)
/* Bit 5 aktif hanya saat external yaw aid fresh; MPU6xxx sendiri tidak memberi yaw absolut. */
#define IMU_FLAG_YAW_ABSOLUTE          (1U << 5)
#define IMU_FLAG_STILL_CAL_VALID       (1U << 6)
#define IMU_FLAG_ROTATE_CAL_VALID      (1U << 7)
#define IMU_FLAG_CAL_ACTIVE            (1U << 8)
#define IMU_FLAG_ZUPT_APPLIED          (1U << 9)
#define IMU_FLAG_MASTER_STATIONARY     (1U << 10)
#define IMU_FLAG_WHEEL_AID              (1U << 11)
#define IMU_FLAG_NHC_AID                (1U << 12)
#define IMU_FLAG_YAW_AID                (1U << 13)
#define IMU_FLAG_VEL_AID                (1U << 14)
#define IMU_FLAG_POS_AID                (1U << 15)

#endif