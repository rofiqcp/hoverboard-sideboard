#ifndef VESC_PACKET_H
#define VESC_PACKET_H

#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "imu_mpu6xxx.h"

#define CAL_CMD_STILL_START   1U
#define CAL_CMD_ROTATE_START  2U
#define CAL_CMD_ROTATE_FINISH 3U
#define CAL_CMD_CANCEL        4U
#define CAL_CMD_ZERO_NAV      5U
#define CAL_CMD_ZUPT_ONCE     6U
#define CAL_CMD_STATUS        7U
#define CAL_CMD_STATIONARY_ON 8U
#define CAL_CMD_STATIONARY_OFF 9U

typedef struct {
    uint32_t sequence;
    uint32_t time_us;
    uint16_t flags;
    ImuSample raw;
    float roll_rad, pitch_rad, yaw_rad;
    float q[4];
    float accel_cal[3];       /* m/s^2 */
    float gyro_cal[3];        /* rad/s */
    float linear_accel_world[3];
    float velocity[3];
    float position[3];
    uint8_t cal_state;
    uint8_t cal_coverage;
    uint8_t cal_error;
    uint16_t cal_progress;
} VescImuState;

typedef enum {
    VESC_ACTION_NONE=0,
    VESC_ACTION_BOOTLOADER,
    VESC_ACTION_CAL_STILL,
    VESC_ACTION_CAL_ROTATE_START,
    VESC_ACTION_CAL_ROTATE_FINISH,
    VESC_ACTION_CAL_CANCEL,
    VESC_ACTION_ZERO_NAV,
    VESC_ACTION_ZUPT,
    VESC_ACTION_STATIONARY_ON,
    VESC_ACTION_STATIONARY_OFF
} VescAction;

uint16_t vesc_crc16(const uint8_t *data, uint16_t len);
int vesc_send_extended_imu(UART_HandleTypeDef *uart, const VescImuState *s);
int vesc_send_calibration_status(UART_HandleTypeDef *uart, uint8_t subcmd,
                                 uint8_t status, const VescImuState *s);
VescAction vesc_process_rx(UART_HandleTypeDef *uart, const VescImuState *latest);

#endif
