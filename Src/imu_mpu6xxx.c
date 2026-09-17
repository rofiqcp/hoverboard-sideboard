#include "imu_mpu6xxx.h"
#include "app_config.h"
#include "board.h"
#include <string.h>

#define REG_SMPLRT_DIV       0x19U
#define REG_CONFIG           0x1AU
#define REG_GYRO_CONFIG      0x1BU
#define REG_ACCEL_CONFIG     0x1CU
#define REG_ACCEL_CONFIG2    0x1DU
#define REG_FIFO_EN          0x23U
#define REG_INT_STATUS       0x3AU
#define REG_ACCEL_XOUT_H     0x3BU
#define REG_TEMP_OUT_H       0x41U
#define REG_USER_CTRL        0x6AU
#define REG_PWR_MGMT_1       0x6BU
#define REG_PWR_MGMT_2       0x6CU
#define REG_FIFO_COUNT_H     0x72U
#define REG_FIFO_R_W         0x74U
#define REG_WHO_AM_I         0x75U

#define FIFO_PACKET_BYTES    12U
#define FIFO_ENABLE_AG       0x78U
#define USER_FIFO_ENABLE     0x40U
#define USER_FIFO_RESET      0x04U
#define INT_FIFO_OVERFLOW    0x10U
#define FIFO_CAPACITY_BYTES  512U

static ImuDeviceInfo info;
static ImuFifoStats fifo_stats;
static uint32_t sample_index;
static int16_t temperature_raw_cache;
static uint8_t temperature_read_divider;
static int write_reg(uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(&hi2c1, MPU6XXX_I2C_ADDR, reg,
                             I2C_MEMADD_SIZE_8BIT, &value, 1U, 20U) == HAL_OK;
}

static int read_bytes(uint8_t reg, uint8_t *data, uint16_t len)
{
    return HAL_I2C_Mem_Read(&hi2c1, MPU6XXX_I2C_ADDR, reg,
                            I2C_MEMADD_SIZE_8BIT, data, len, 20U) == HAL_OK;
}

static int read_reg(uint8_t reg, uint8_t *value)
{
    return value && read_bytes(reg, value, 1U);
}

static int16_t be_i16(const uint8_t *p)
{
    return (int16_t)(((uint16_t)p[0] << 8) | p[1]);
}

uint8_t imu_mpu6xxx_whoami(void)
{
    uint8_t value = 0U;
    return read_reg(REG_WHO_AM_I, &value) ? value : 0U;
}

const ImuDeviceInfo *imu_mpu6xxx_get_info(void) { return &info; }
const ImuFifoStats *imu_mpu6xxx_get_fifo_stats(void) { return &fifo_stats; }
static int configure_descriptor(uint8_t who)
{
    memset(&info, 0, sizeof(info));
    info.whoami = who;
    info.sample_hz = MPU6XXX_SAMPLE_HZ;
    info.fifo_packet_bytes = FIFO_PACKET_BYTES;

    if (who == 0x68U) {
        info.device_class = IMU_CLASS_MPU6050;
        info.temperature_lsb_per_c = 340.0f;
        info.temperature_offset_c = 36.53f;
        return 1;
    }

    if (who == 0x70U || who == 0x71U || who == 0x72U ||
        who == 0x73U || who == 0x74U) {
        /* 0x72 pada board ini diperlakukan sebagai kompatibel register MPU65xx.
         * Identitas persis clone tidak diasumsikan; fitur tambahan diprobe. */
        info.device_class = IMU_CLASS_MPU65XX_COMPAT;
        info.temperature_lsb_per_c = 333.87f;
        info.temperature_offset_c = 21.0f;
        return 1;
    }
    return 0;
}

static float temperature_from_raw(int16_t raw)
{
    if (info.temperature_lsb_per_c <= 0.0f) return 0.0f;
    return (float)raw / info.temperature_lsb_per_c + info.temperature_offset_c;
}

static void convert_sample(ImuSample *s)
{
    const float a_scale = GRAVITY_MPS2 / MPU6XXX_ACCEL_LSB_PER_G;
    const float g_scale = 0.01745329251994329577f / MPU6XXX_GYRO_LSB_PER_DPS;
    for (int i = 0; i < 3; i++) {
        s->accel_mps2[i] = (float)s->accel_raw[i] * a_scale;
        s->gyro_rads[i] = (float)s->gyro_raw[i] * g_scale;
    }
    s->temperature_c = temperature_from_raw(s->temp_raw);
}
int imu_mpu6xxx_fifo_reset(void)
{
    if (!write_reg(REG_FIFO_EN, 0x00U)) return 0;
    if (!write_reg(REG_USER_CTRL, USER_FIFO_RESET)) return 0;
    HAL_Delay(2U);
    if (!write_reg(REG_USER_CTRL, USER_FIFO_ENABLE)) return 0;
    if (!write_reg(REG_FIFO_EN, FIFO_ENABLE_AG)) return 0;
    uint8_t ignored = 0U;
    (void)read_reg(REG_INT_STATUS, &ignored);
    return 1;
}

static int verify_base_config(uint8_t divider)
{
    uint8_t v = 0U;
    if (!read_reg(REG_SMPLRT_DIV, &v) || v != divider) return 0;
    if (!read_reg(REG_CONFIG, &v) || (v & 0x07U) != 0x03U) return 0;
    if (!read_reg(REG_GYRO_CONFIG, &v) || (v & 0x18U) != 0x08U) return 0;
    if (!read_reg(REG_ACCEL_CONFIG, &v) || (v & 0x18U) != 0x08U) return 0;
    if (!read_reg(REG_FIFO_EN, &v) || (v & FIFO_ENABLE_AG) != FIFO_ENABLE_AG) return 0;
    if (!read_reg(REG_USER_CTRL, &v) || (v & USER_FIFO_ENABLE) == 0U) return 0;
    return 1;
}

int imu_mpu6xxx_init(void)
{
    uint8_t who = imu_mpu6xxx_whoami();
    if (!configure_descriptor(who)) return 0;
    memset(&fifo_stats, 0, sizeof(fifo_stats));
    fifo_stats.observed_sample_hz = (float)MPU6XXX_SAMPLE_HZ;
    sample_index = 0U;
    temperature_raw_cache = 0;
    temperature_read_divider = 0U;

    if (!write_reg(REG_PWR_MGMT_1, 0x80U)) return 0;
    HAL_Delay(100U);
    if (!write_reg(REG_PWR_MGMT_1, 0x01U)) return 0;
    if (!write_reg(REG_PWR_MGMT_2, 0x00U)) return 0;
    const uint8_t divider = (uint8_t)(1000U / MPU6XXX_SAMPLE_HZ - 1U);
    if (!write_reg(REG_SMPLRT_DIV, divider)) return 0;
    if (!write_reg(REG_CONFIG, 0x03U)) return 0;
    if (!write_reg(REG_GYRO_CONFIG, 0x08U)) return 0;
    if (!write_reg(REG_ACCEL_CONFIG, 0x08U)) return 0;

    if (info.device_class == IMU_CLASS_MPU65XX_COMPAT) {
        uint8_t probe = 0U;
        if (write_reg(REG_ACCEL_CONFIG2, 0x03U) &&
            read_reg(REG_ACCEL_CONFIG2, &probe) && (probe & 0x07U) == 0x03U) {
            info.accel_config2_supported = 1U;
        } else {
            /* WHO_AM_I clone/varian tidak cukup untuk menentukan register map.
             * Jika ACCEL_CONFIG2 tidak nyata, gunakan karakteristik temperatur
             * MPU60xx legacy yang cocok dengan register filter tunggal CONFIG. */
            info.device_class = IMU_CLASS_MPU6050;
            info.temperature_lsb_per_c = 340.0f;
            info.temperature_offset_c = 36.53f;
        }
    }

    if (!imu_mpu6xxx_fifo_reset()) return 0;
    HAL_Delay(20U);
    return verify_base_config(divider) && imu_mpu6xxx_whoami() == who;
}

int imu_mpu6xxx_read(ImuSample *sample)
{
    uint8_t b[14];
    if (!sample || !read_bytes(REG_ACCEL_XOUT_H, b, sizeof(b))) return 0;

    sample->accel_raw[0] = be_i16(&b[0]);
    sample->accel_raw[1] = be_i16(&b[2]);
    sample->accel_raw[2] = be_i16(&b[4]);
    sample->temp_raw = be_i16(&b[6]);
    sample->gyro_raw[0] = be_i16(&b[8]);
    sample->gyro_raw[1] = be_i16(&b[10]);
    sample->gyro_raw[2] = be_i16(&b[12]);
    sample->sample_time_us = board_micros();
    sample->sample_index = sample_index++;
    convert_sample(sample);
    return 1;
}
int imu_mpu6xxx_read_fifo(ImuSample *samples, uint8_t max_samples, uint8_t *out_count)
{
    uint8_t count_b[2];
    uint8_t status = 0U;
    uint8_t temp_b[2] = {0U, 0U};
    uint8_t fifo_b[12U * 8U];
    if (!samples || !out_count || max_samples == 0U || max_samples > 8U) return 0;
    *out_count = 0U;

    if (!read_bytes(REG_FIFO_COUNT_H, count_b, 2U)) return 0;
    uint16_t fifo_bytes = ((uint16_t)count_b[0] << 8) | count_b[1];
    fifo_stats.fifo_read_count++;
    if (fifo_bytes > fifo_stats.fifo_max_bytes) fifo_stats.fifo_max_bytes = fifo_bytes;

    /* INT_STATUS tidak perlu dibaca pada setiap service. Saat FIFO masih jauh
     * dari penuh, overflow tidak mungkin. Ini menghemat satu transaksi I2C. */
    if (fifo_bytes >= (FIFO_CAPACITY_BYTES / 2U)) {
        (void)read_reg(REG_INT_STATUS, &status);
    }
    if ((status & INT_FIFO_OVERFLOW) || fifo_bytes >= FIFO_CAPACITY_BYTES - FIFO_PACKET_BYTES) {
        fifo_stats.fifo_overflow_count++;
        fifo_stats.fifo_resync_count++;
        return imu_mpu6xxx_fifo_reset();
    }

    if ((fifo_bytes % FIFO_PACKET_BYTES) != 0U) {
        /* FIFO accel+gyro harus selalu kelipatan 12 byte. Sisa byte berarti
         * alignment tidak dapat dipercaya, jadi sinkronkan ulang. */
        fifo_stats.fifo_resync_count++;
        return imu_mpu6xxx_fifo_reset();
    }

    uint8_t packet_count = (uint8_t)(fifo_bytes / FIFO_PACKET_BYTES);
    if (packet_count == 0U) {
        fifo_stats.fifo_empty_count++;
        return 1;
    }
    if (packet_count > max_samples) {
        /* Backlog besar berarti estimator terlalu lama tidak melayani IMU.
         * Reset lebih aman daripada mengintegrasikan data tua dengan latency besar. */
        fifo_stats.fifo_resync_count++;
        return imu_mpu6xxx_fifo_reset();
    }

    uint16_t read_len = (uint16_t)packet_count * FIFO_PACKET_BYTES;
    if (!read_bytes(REG_FIFO_R_W, fifo_b, read_len)) return 0;

    /* Temperatur berubah lambat. Baca sekitar 10 Hz, bukan 100 Hz, agar bus
     * I2C punya margin lebih besar untuk FIFO 200 Hz. */
    if (++temperature_read_divider >= 10U || temperature_raw_cache == 0) {
        temperature_read_divider = 0U;
        if (read_bytes(REG_TEMP_OUT_H, temp_b, 2U)) {
            temperature_raw_cache = be_i16(temp_b);
        }
    }
    int16_t temp_raw = temperature_raw_cache;

    uint32_t now_us = board_micros();

    /* Sensor FIFO punya oscillator sendiri. Ukur rate aktual seperti pola
     * backend FIFO ArduPilot. Batch pertama hanya membuka window agar tidak
     * menghitung sampel yang terjadi sebelum timestamp awal. */
    if (fifo_stats.rate_window_start_us == 0U) {
        fifo_stats.rate_window_start_us = now_us;
        fifo_stats.rate_window_samples = 0U;
    } else {
        fifo_stats.rate_window_samples += packet_count;
        uint32_t elapsed_us = now_us - fifo_stats.rate_window_start_us;
        if (elapsed_us >= 1000000U) {
            float measured = (float)fifo_stats.rate_window_samples * 1000000.0f /
                             (float)elapsed_us;
            float nominal = (float)MPU6XXX_SAMPLE_HZ;
            if (measured < nominal * 0.95f) measured = nominal * 0.95f;
            if (measured > nominal * 1.05f) measured = nominal * 1.05f;
            fifo_stats.observed_sample_hz = 0.9f * fifo_stats.observed_sample_hz +
                                            0.1f * measured;
            fifo_stats.rate_window_start_us = now_us;
            fifo_stats.rate_window_samples = 0U;
        }
    }

    float timestamp_hz = fifo_stats.observed_sample_hz;
    if (!(timestamp_hz > 100.0f && timestamp_hz < 300.0f)) {
        timestamp_hz = (float)info.sample_hz;
    }
    uint32_t sample_period_us = (uint32_t)(1000000.0f / timestamp_hz + 0.5f);
    for (uint8_t n = 0U; n < packet_count; n++) {
        const uint8_t *b = &fifo_b[(uint16_t)n * FIFO_PACKET_BYTES];
        ImuSample *s = &samples[n];
        memset(s, 0, sizeof(*s));
        s->accel_raw[0] = be_i16(&b[0]);
        s->accel_raw[1] = be_i16(&b[2]);
        s->accel_raw[2] = be_i16(&b[4]);
        s->gyro_raw[0] = be_i16(&b[6]);
        s->gyro_raw[1] = be_i16(&b[8]);
        s->gyro_raw[2] = be_i16(&b[10]);
        s->temp_raw = temp_raw;
        s->sample_time_us = now_us - (uint32_t)(packet_count - 1U - n) * sample_period_us;
        s->sample_index = sample_index++;
        convert_sample(s);
    }

    fifo_stats.fifo_sample_count += packet_count;
    *out_count = packet_count;
    return 1;
}
