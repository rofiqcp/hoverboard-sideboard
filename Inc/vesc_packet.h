#ifndef VESC_PACKET_H
#define VESC_PACKET_H

#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "imu_mpu6xxx.h"
#include "eeprom_flash.h"

#define CAL_CMD_STILL_START   1U
#define CAL_CMD_ROTATE_START  2U
#define CAL_CMD_ROTATE_FINISH 3U
#define CAL_CMD_CANCEL        4U
#define CAL_CMD_ZERO_NAV      5U
#define CAL_CMD_ZUPT_ONCE     6U
#define CAL_CMD_STATUS        7U
#define CAL_CMD_STATIONARY_ON 8U
#define CAL_CMD_STATIONARY_OFF 9U

#define CFG_CMD_GET             1U
#define CFG_CMD_SET_MOUNT_RPY   2U
#define CFG_CMD_SET_THERMAL     3U
#define CFG_CMD_SET_LEVER_ARM   4U
#define CFG_CMD_CLEAR_THERMAL   5U
#define CFG_CMD_RESET_MOUNT     6U

#define AID_CMD_WHEEL_BODY_X    1U
#define AID_CMD_WORLD_VELOCITY  2U
#define AID_CMD_WORLD_POSITION  3U
#define AID_CMD_YAW             4U
#define AID_FLAG_NHC            (1U << 0)

#define NAV_STATUS_ATT_VALID          (1U << 0)
#define NAV_STATUS_VEL_AIDED          (1U << 1)
#define NAV_STATUS_POS_AIDED          (1U << 2)
#define NAV_STATUS_YAW_AIDED          (1U << 3)
#define NAV_STATUS_DEAD_RECKONING     (1U << 4)
#define NAV_STATUS_COV_VALID          (1U << 5)
#define NAV_STATUS_STATIONARY_BOUND   (1U << 6)
#define NAV_STATUS_HEALTH_RECOVERED   (1U << 7)

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
    uint16_t aid_age_ms;
    uint16_t aid_reject_count;
    float attitude_std_rad[3];
    float velocity_std_mps[3];
    float position_std_m[3];
    uint8_t nav_status;
    uint16_t health_reset_count;
} VescImuState;

typedef struct {
    uint8_t subcmd;
    float value[6];
} VescConfigRequest;

typedef struct {
    uint8_t type;
    uint8_t flags;
    uint32_t time_us;
    float value[3];
    float sigma;
} VescAidingRequest;

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
    VESC_ACTION_STATIONARY_OFF,
    VESC_ACTION_CONFIG,
    VESC_ACTION_AIDING
} VescAction;

uint16_t vesc_crc16(const uint8_t *data, uint16_t len);
int vesc_send_extended_imu(UART_HandleTypeDef *uart, const VescImuState *s);
int vesc_send_calibration_status(UART_HandleTypeDef *uart, uint8_t subcmd,
                                 uint8_t status, const VescImuState *s);
int vesc_send_config_status(UART_HandleTypeDef *uart, uint8_t subcmd, uint8_t status,
                            const PersistedSettings *s);
int vesc_send_aiding_status(UART_HandleTypeDef *uart, uint8_t type, uint8_t status, uint16_t age_ms);
int vesc_take_config_request(VescConfigRequest *out);
int vesc_take_aiding_request(VescAidingRequest *out);
VescAction vesc_process_rx(UART_HandleTypeDef *uart, const VescImuState *latest);

#endif
