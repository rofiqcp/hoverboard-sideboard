#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "imu_mpu6xxx.h"
#include "board.h"

#define REG_GYRO_CONFIG   0x1BU
#define REG_FIFO_COUNT_H  0x72U
#define REG_FIFO_R_W      0x74U
#define REG_TEMP_OUT_H    0x41U
#define REG_WHO_AM_I      0x75U

I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart2;

static uint8_t regs[256];
static uint8_t fifo_data[96];
static uint16_t fifo_len;
static uint32_t fake_now_us = 1000U;
static uint32_t read_transactions;
static int fail_next_read;
static int fail_next_write;

static void put_i16_be(uint8_t *p, int16_t v)
{
    uint16_t u = (uint16_t)v;
    p[0] = (uint8_t)(u >> 8);
    p[1] = (uint8_t)u;
}
static void set_packet(uint8_t index,
                       int16_t ax, int16_t ay, int16_t az,
                       int16_t gx, int16_t gy, int16_t gz)
{
    uint8_t *p = &fifo_data[(uint16_t)index * 12U];
    put_i16_be(&p[0], ax); put_i16_be(&p[2], ay); put_i16_be(&p[4], az);
    put_i16_be(&p[6], gx); put_i16_be(&p[8], gy); put_i16_be(&p[10], gz);
}

static void queue_packets(uint8_t count)
{
    fifo_len = (uint16_t)count * 12U;
}

HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *h, uint16_t addr,
                                    uint16_t reg, uint16_t mem_size,
                                    uint8_t *data, uint16_t len, uint32_t timeout)
{
    (void)h; (void)addr; (void)mem_size; (void)timeout;
    if (fail_next_write) { fail_next_write = 0; return HAL_ERROR; }
    if (len != 1U) return HAL_ERROR;
    regs[(uint8_t)reg] = data[0];
    return HAL_OK;
}
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *h, uint16_t addr,
                                   uint16_t reg, uint16_t mem_size,
                                   uint8_t *data, uint16_t len, uint32_t timeout)
{
    (void)h; (void)addr; (void)mem_size; (void)timeout;
    read_transactions++;
    if (fail_next_read) { fail_next_read = 0; return HAL_ERROR; }
    if ((uint8_t)reg == REG_FIFO_COUNT_H && len == 2U) {
        data[0] = (uint8_t)(fifo_len >> 8); data[1] = (uint8_t)fifo_len; return HAL_OK;
    }
    if ((uint8_t)reg == REG_FIFO_R_W) {
        if (len != fifo_len) return HAL_ERROR;
        memcpy(data, fifo_data, len); fifo_len = 0U; return HAL_OK;
    }
    if ((uint8_t)reg == REG_TEMP_OUT_H && len == 2U) {
        data[0] = 0x01U; data[1] = 0x00U; return HAL_OK;
    }
    if (len == 1U) { data[0] = regs[(uint8_t)reg]; return HAL_OK; }
    return HAL_ERROR;
}

void HAL_Delay(uint32_t ms) { fake_now_us += ms * 1000U; }
uint32_t board_micros(void) { return fake_now_us; }

static void read_one(int16_t ax, int16_t ay, int16_t az,
                     int16_t gx, int16_t gy, int16_t gz)
{
    ImuSample samples[2];
    uint8_t count = 0U;
    set_packet(0U, ax, ay, az, gx, gy, gz);
    queue_packets(1U);
    fake_now_us += 10000U;
    assert(imu_mpu6xxx_read_fifo(samples, 2U, &count) == 1);
    assert(count == 1U);
}

int main(void)
{
    const ImuRuntimeHealth *health = imu_mpu6xxx_get_runtime_health();
    assert(health != NULL);
    assert(health->i2c_error_count == 0U && health->init_count == 0U);
    assert(health->last_accel_clip_mask == 0U && health->last_gyro_clip_mask == 0U);

    volatile BoardRuntimeStats stats = {0};
    board_runtime_account_imu_pending(&stats, 1U);
    assert(stats.imu_deadline_miss_count == 0U && stats.imu_max_pending_ticks == 1U);
    board_runtime_account_imu_pending(&stats, 3U);
    assert(stats.imu_deadline_miss_count == 2U && stats.imu_max_pending_ticks == 3U);
    board_runtime_account_imu_pending(&stats, 2U);
    assert(stats.imu_deadline_miss_count == 3U && stats.imu_max_pending_ticks == 3U);

    memset(regs, 0, sizeof(regs));
    regs[REG_WHO_AM_I] = 0x68U;
    assert(imu_mpu6xxx_init() == 1);
    health = imu_mpu6xxx_get_runtime_health();
    assert(health->init_count == 1U && health->reinit_count == 0U && health->config_ok == 1U);
    assert(imu_mpu6xxx_get_info()->whoami == 0x68U);
    assert(imu_mpu6xxx_get_info()->fifo_packet_bytes == 12U);

    read_one(32767, 0, 0, 0, 0, 0);
    assert(health->accel_clip_count[0] == 1U && health->last_accel_clip_mask == 0x01U);
    read_one(0, -32768, 0, 0, 0, 0);
    assert(health->accel_clip_count[1] == 1U && health->last_accel_clip_mask == 0x02U);
    read_one(0, 0, 0, 0, 0, 32767);
    assert(health->gyro_clip_count[2] == 1U && health->last_gyro_clip_mask == 0x04U);
    read_one(32111, -32111, 1000, 32000, -32000, 0);
    assert(health->last_accel_clip_mask == 0U && health->last_gyro_clip_mask == 0U);

    ImuSample batch[2];
    uint8_t count = 0U;
    set_packet(0U, 0, 0, 32767, -32768, 0, 0);
    set_packet(1U, -32768, 0, 0, 0, 32767, 0);
    queue_packets(2U);
    fake_now_us += 10000U;
    assert(imu_mpu6xxx_read_fifo(batch, 2U, &count) == 1 && count == 2U);
    assert(health->last_accel_clip_mask == 0x05U);
    assert(health->last_gyro_clip_mask == 0x03U);
    assert(health->accel_clip_count[0] == 2U && health->accel_clip_count[1] == 1U &&
           health->accel_clip_count[2] == 1U);
    assert(health->gyro_clip_count[0] == 1U && health->gyro_clip_count[1] == 1U &&
           health->gyro_clip_count[2] == 1U);

    uint32_t t = health->last_config_check_us + 1500000U;
    assert(imu_mpu6xxx_periodic_verify(t) == 1);
    uint32_t mismatches = health->config_mismatch_count;
    regs[REG_GYRO_CONFIG] = 0U;
    t += 1500000U;
    assert(imu_mpu6xxx_periodic_verify(t) == 0);
    assert(health->config_ok == 0U && health->config_mismatch_count == mismatches + 1U);

    regs[REG_GYRO_CONFIG] = 0x08U;
    uint32_t reads_before = read_transactions;
    assert(imu_mpu6xxx_periodic_verify(t + 100000U) == 0);
    assert(read_transactions == reads_before);
    t += 1500000U;
    assert(imu_mpu6xxx_periodic_verify(t) == 1 && health->config_ok == 1U);

    uint32_t i2c_errors = health->i2c_error_count;
    mismatches = health->config_mismatch_count;
    fail_next_read = 1;
    t += 1500000U;
    assert(imu_mpu6xxx_periodic_verify(t) == 0);
    assert(health->i2c_error_count == i2c_errors + 1U);
    assert(health->config_mismatch_count == mismatches);
    assert(health->config_ok == 0U);
    fail_next_write = 1;
    assert(imu_mpu6xxx_fifo_reset() == 0);
    assert(health->i2c_error_count == i2c_errors + 2U);

    puts("agent1 imu runtime health tests: PASS");
    return 0;
}
