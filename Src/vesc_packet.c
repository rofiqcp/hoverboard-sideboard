#include "vesc_packet.h"
#include "app_config.h"
#include "board.h"
#include <math.h>
#include <string.h>

uint16_t vesc_crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0U;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t b = 0; b < 8U; b++) {
            crc = (crc & 0x8000U) ?
                  (uint16_t)((crc << 1) ^ 0x1021U) :
                  (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static void put_u16(uint8_t *b, uint16_t *i, uint16_t v)
{
    b[(*i)++] = (uint8_t)(v >> 8);
    b[(*i)++] = (uint8_t)v;
}

static void put_i16(uint8_t *b, uint16_t *i, int16_t v)
{
    put_u16(b, i, (uint16_t)v);
}

static void put_u32(uint8_t *b, uint16_t *i, uint32_t v)
{
    b[(*i)++] = (uint8_t)(v >> 24);
    b[(*i)++] = (uint8_t)(v >> 16);
    b[(*i)++] = (uint8_t)(v >> 8);
    b[(*i)++] = (uint8_t)v;
}

static void put_i32(uint8_t *b, uint16_t *i, int32_t v)
{
    put_u32(b, i, (uint32_t)v);
}

static uint16_t get_u16(const uint8_t *b)
{
    return ((uint16_t)b[0] << 8) | b[1];
}

static uint32_t get_u32(const uint8_t *b)
{
    return ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) |
           ((uint32_t)b[2] << 8) | b[3];
}

static int32_t get_i32(const uint8_t *b)
{
    return (int32_t)get_u32(b);
}

static int32_t sat_i32(float x)
{
    if (x > 2147483000.0f) return 2147483000;
    if (x < -2147483000.0f) return -2147483000;
    return (int32_t)lrintf(x);
}

static uint16_t sat_u16(float x)
{
    if (x <= 0.0f) return 0U;
    if (x >= 65535.0f) return 65535U;
    return (uint16_t)lrintf(x);
}

/* Format yang sama dengan buffer_append_float32_auto milik VESC. */
static void put_float_auto(uint8_t *b, uint16_t *i, float number)
{
    if (fabsf(number) < 1.5e-38f) {
        number = 0.0f;
    }

    int exponent = 0;
    float sig = frexpf(number, &exponent);
    float sig_abs = fabsf(sig);
    uint32_t sig_i = 0U;

    if (sig_abs >= 0.5f) {
        sig_i = (uint32_t)((sig_abs - 0.5f) * 2.0f * 8388608.0f);
        exponent += 126;
    }

    uint32_t encoded = ((uint32_t)(exponent & 0xFF) << 23) |
                       (sig_i & 0x7FFFFFU);
    if (sig < 0.0f) {
        encoded |= 1UL << 31;
    }
    put_u32(b, i, encoded);
}

static int send_payload(UART_HandleTypeDef *uart,
                        const uint8_t *payload,
                        uint16_t len)
{
    if (!uart || !payload || len == 0U || len > 255U) {
        return 0;
    }

    (void)board_uart_tx_wait_idle(3000U);

    uint8_t frame[272];
    uint16_t f = 0U;
    uint16_t crc = vesc_crc16(payload, len);

    frame[f++] = 2U;
    frame[f++] = (uint8_t)len;
    memcpy(&frame[f], payload, len);
    f += len;
    frame[f++] = (uint8_t)(crc >> 8);
    frame[f++] = (uint8_t)crc;
    frame[f++] = 3U;

    return HAL_UART_Transmit(uart, frame, f, 10U) == HAL_OK;
}

static int send_payload_async(const uint8_t *payload, uint16_t len)
{
    if (!payload || len == 0U || len > 120U) {
        return 0;
    }
    uint8_t frame[128];
    uint16_t f = 0U;
    uint16_t crc = vesc_crc16(payload, len);
    frame[f++] = 2U;
    frame[f++] = (uint8_t)len;
    memcpy(&frame[f], payload, len);
    f += len;
    frame[f++] = (uint8_t)(crc >> 8);
    frame[f++] = (uint8_t)crc;
    frame[f++] = 3U;
    return board_uart_tx_async(frame, f);
}

int vesc_send_extended_imu(UART_HandleTypeDef *uart, const VescImuState *s)
{
    uint8_t payload[112];
    uint16_t i = 0U;

    payload[i++] = COMM_SIDEBOARD_IMU;
    payload[i++] = IMU_PROTOCOL_VERSION;
    put_u16(payload, &i, s->flags);
    put_u32(payload, &i, s->sequence);
    put_u32(payload, &i, s->time_us);

    for (int k = 0; k < 3; k++) {
        put_i16(payload, &i, s->raw.accel_raw[k]);
    }
    put_i16(payload, &i, s->raw.temp_raw);
    for (int k = 0; k < 3; k++) {
        put_i16(payload, &i, s->raw.gyro_raw[k]);
    }

    const float rad_to_mdeg = 57295.77951308232f;
    put_i32(payload, &i, sat_i32(s->roll_rad * rad_to_mdeg));
    put_i32(payload, &i, sat_i32(s->pitch_rad * rad_to_mdeg));
    put_i32(payload, &i, sat_i32(s->yaw_rad * rad_to_mdeg));

    for (int k = 0; k < 3; k++) {
        put_i32(payload, &i, sat_i32(s->velocity[k] * 1000.0f));
    }
    for (int k = 0; k < 3; k++) {
        put_i32(payload, &i, sat_i32(s->position[k] * 1000.0f));
    }
    for (int k = 0; k < 3; k++) {
        put_i32(payload, &i, sat_i32(s->linear_accel_world[k] * 1000.0f));
    }

    payload[i++] = s->cal_state;
    payload[i++] = s->cal_coverage;
    payload[i++] = s->cal_error;
    put_u16(payload, &i, s->cal_progress);
    put_u16(payload, &i, s->aid_age_ms);
    put_u16(payload, &i, s->aid_reject_count);
    for (int k=0;k<3;k++) put_u16(payload,&i,sat_u16(s->attitude_std_rad[k]*57295.7795f));
    for (int k=0;k<3;k++) put_u16(payload,&i,sat_u16(s->velocity_std_mps[k]*1000.0f));
    for (int k=0;k<3;k++) put_u16(payload,&i,sat_u16(s->position_std_m[k]*1000.0f));
    payload[i++]=s->nav_status;
    put_u16(payload,&i,s->health_reset_count);

    (void)uart;
    return send_payload_async(payload, i);
}

static int send_vesc_imu(UART_HandleTypeDef *uart,
                         const VescImuState *s,
                         uint16_t mask)
{
    uint8_t payload[80];
    uint16_t i = 0U;
    payload[i++] = COMM_GET_IMU_DATA;
    put_u16(payload, &i, mask);

    float values[16] = {
        s->roll_rad,
        s->pitch_rad,
        s->yaw_rad,
        s->accel_cal[0] / GRAVITY_MPS2,
        s->accel_cal[1] / GRAVITY_MPS2,
        s->accel_cal[2] / GRAVITY_MPS2,
        s->gyro_cal[0] * 57.29577951308232f,
        s->gyro_cal[1] * 57.29577951308232f,
        s->gyro_cal[2] * 57.29577951308232f,
        0.0f, 0.0f, 0.0f,
        s->q[0], s->q[1], s->q[2], s->q[3]
    };

    for (int bit = 0; bit < 16; bit++) {
        if (mask & (1U << bit)) {
            put_float_auto(payload, &i, values[bit]);
        }
    }

    return send_payload(uart, payload, i);
}

int vesc_send_calibration_status(UART_HandleTypeDef *uart,
                                 uint8_t subcmd,
                                 uint8_t status,
                                 const VescImuState *s)
{
    uint8_t payload[9];
    uint16_t i = 0U;

    payload[i++] = COMM_SIDEBOARD_CALIBRATION;
    payload[i++] = subcmd;
    payload[i++] = status;
    payload[i++] = s ? s->cal_state : 0U;
    payload[i++] = s ? s->cal_coverage : 0U;
    payload[i++] = s ? s->cal_error : 0U;
    put_u16(payload, &i, s ? s->cal_progress : 0U);

    return send_payload(uart, payload, i);
}


static VescConfigRequest pending_config;
static VescAidingRequest pending_aiding;
static uint8_t config_pending=0U;
static uint8_t aiding_pending=0U;

int vesc_take_config_request(VescConfigRequest *out)
{
    if(!out||!config_pending)return 0;
    *out=pending_config; config_pending=0U; return 1;
}

int vesc_take_aiding_request(VescAidingRequest *out)
{
    if(!out||!aiding_pending)return 0;
    *out=pending_aiding; aiding_pending=0U; return 1;
}

int vesc_send_config_status(UART_HandleTypeDef *uart,uint8_t subcmd,uint8_t status,
                            const PersistedSettings *s)
{
    uint8_t payload[64]; uint16_t i=0U;
    payload[i++]=COMM_SIDEBOARD_CONFIG; payload[i++]=subcmd; payload[i++]=status;
    if(s){
        for(int k=0;k<4;k++)put_i32(payload,&i,sat_i32(s->sensor_to_body_q[k]*1000000.0f));
        for(int k=0;k<3;k++)put_i32(payload,&i,sat_i32(s->gyro_temp_slope[k]*1000000.0f));
        for(int k=0;k<3;k++)put_i32(payload,&i,sat_i32(s->accel_temp_slope[k]*1000000.0f));
        for(int k=0;k<3;k++)put_i32(payload,&i,sat_i32(s->imu_position_body[k]*1000.0f));
        put_u32(payload,&i,s->calibration_flags);
    }
    return send_payload(uart,payload,i);
}

int vesc_send_aiding_status(UART_HandleTypeDef *uart,uint8_t type,uint8_t status,uint16_t age_ms)
{
    uint8_t payload[5]; uint16_t i=0U;
    payload[i++]=COMM_SIDEBOARD_AIDING; payload[i++]=type; payload[i++]=status;
    put_u16(payload,&i,age_ms);
    return send_payload(uart,payload,i);
}

typedef struct {
    uint8_t state;
    uint8_t len;
    uint8_t index;
    uint8_t payload[40];
    uint16_t crc;
} RxState;

static RxState rx;
static uint32_t rx_last_byte_us = 0U;
static uint32_t rx_seen_overflow = 0U;

static void rx_reset(void)
{
    rx.state = 0U; rx.len = 0U; rx.index = 0U; rx.crc = 0U;
}

static int rx_feed(uint8_t b)
{
    switch (rx.state) {
    case 0:
        if (b == 2U) rx.state = 1U;
        break;
    case 1:
        if (b == 0U || b > sizeof(rx.payload)) {
            rx.state = (b == 2U) ? 1U : 0U;
        } else {
            rx.len = b;
            rx.index = 0U;
            rx.state = 2U;
        }
        break;
    case 2:
        rx.payload[rx.index++] = b;
        if (rx.index >= rx.len) rx.state = 3U;
        break;
    case 3:
        rx.crc = (uint16_t)b << 8;
        rx.state = 4U;
        break;
    case 4:
        rx.crc |= b;
        rx.state = 5U;
        break;
    case 5: {
        int ok = b == 3U && rx.crc == vesc_crc16(rx.payload, rx.len);
        rx.state = (!ok && b == 2U) ? 1U : 0U;
        return ok;
    }
    default:
        rx.state = 0U;
        break;
    }
    return 0;
}

VescAction vesc_process_rx(UART_HandleTypeDef *uart,
                           const VescImuState *latest)
{
    uint8_t byte;
    uint32_t now_us = board_micros();
    uint32_t overflow = board_uart_rx_overflow_count();
    if (overflow != rx_seen_overflow ||
        (rx.state != 0U && (uint32_t)(now_us - rx_last_byte_us) > 50000U)) {
        rx_reset();
        rx_seen_overflow = overflow;
    }
    while (board_uart_rx_pop(&byte)) {
        uint32_t overflow_now = board_uart_rx_overflow_count();
        if (overflow_now != rx_seen_overflow) {
            rx_reset();
            rx_seen_overflow = overflow_now;
        }
        now_us = board_micros();
        if (rx.state != 0U && (uint32_t)(now_us - rx_last_byte_us) > 20000U) rx_reset();
        rx_last_byte_us = now_us;
        if (!rx_feed(byte)) {
            continue;
        }

        if (rx.len == 1U && rx.payload[0] == COMM_SIDEBOARD_BOOTLOADER) {
            uint8_t ack[2] = {COMM_SIDEBOARD_BOOTLOADER, 0U};
            (void)send_payload(uart, ack, sizeof(ack));
            return VESC_ACTION_BOOTLOADER;
        }

        if (rx.len >= 3U && rx.payload[0] == COMM_GET_IMU_DATA) {
            /* Jangan kirim state nol saat estimator belum siap pada fase startup. */
            if (latest) {
                uint16_t mask = ((uint16_t)rx.payload[1] << 8) | rx.payload[2];
                (void)send_vesc_imu(uart, latest, mask);
            }
            continue;
        }

        if (rx.len >= 2U && rx.payload[0] == COMM_SIDEBOARD_CONFIG) {
            memset(&pending_config,0,sizeof(pending_config));
            pending_config.subcmd=rx.payload[1];
            if (pending_config.subcmd==CFG_CMD_GET ||
                pending_config.subcmd==CFG_CMD_CLEAR_THERMAL ||
                pending_config.subcmd==CFG_CMD_RESET_MOUNT) {
                if(rx.len!=2U){(void)vesc_send_config_status(uart,pending_config.subcmd,2U,0);continue;}
            } else if (pending_config.subcmd==CFG_CMD_SET_MOUNT_RPY && rx.len==14U) {
                for(int k=0;k<3;k++) pending_config.value[k]=(float)get_i32(&rx.payload[2+4*k])*0.001f*0.0174532925199433f;
            } else if (pending_config.subcmd==CFG_CMD_SET_THERMAL && rx.len==26U) {
                for(int k=0;k<6;k++) pending_config.value[k]=(float)get_i32(&rx.payload[2+4*k])*1.0e-6f;
            } else if (pending_config.subcmd==CFG_CMD_SET_LEVER_ARM && rx.len==14U) {
                for(int k=0;k<3;k++) pending_config.value[k]=(float)get_i32(&rx.payload[2+4*k])*0.001f;
            } else {
                (void)vesc_send_config_status(uart,pending_config.subcmd,2U,0); continue;
            }
            config_pending=1U; return VESC_ACTION_CONFIG;
        }

        if (rx.len >= 2U && rx.payload[0] == COMM_SIDEBOARD_AIDING) {
            memset(&pending_aiding,0,sizeof(pending_aiding));
            pending_aiding.type=rx.payload[1];
            if(pending_aiding.type==AID_CMD_WHEEL_BODY_X && rx.len==13U){
                pending_aiding.time_us=get_u32(&rx.payload[2]);
                pending_aiding.value[0]=(float)get_i32(&rx.payload[6])*0.001f;
                pending_aiding.sigma=(float)get_u16(&rx.payload[10])*0.001f;
                pending_aiding.flags=rx.payload[12];
            }else if((pending_aiding.type==AID_CMD_WORLD_VELOCITY || pending_aiding.type==AID_CMD_WORLD_POSITION) && rx.len==20U){
                pending_aiding.time_us=get_u32(&rx.payload[2]);
                for(int k=0;k<3;k++)pending_aiding.value[k]=(float)get_i32(&rx.payload[6+4*k])*0.001f;
                pending_aiding.sigma=(float)get_u16(&rx.payload[18])*0.001f;
            }else if(pending_aiding.type==AID_CMD_YAW && rx.len==12U){
                pending_aiding.time_us=get_u32(&rx.payload[2]);
                pending_aiding.value[0]=(float)get_i32(&rx.payload[6])*0.001f*0.0174532925199433f;
                pending_aiding.sigma=(float)get_u16(&rx.payload[10])*0.001f*0.0174532925199433f;
            }else{
                (void)vesc_send_aiding_status(uart,pending_aiding.type,2U,0U); continue;
            }
            if(pending_aiding.sigma<=0.0f){(void)vesc_send_aiding_status(uart,pending_aiding.type,3U,0U);continue;}
            aiding_pending=1U; return VESC_ACTION_AIDING;
        }

        if (rx.len >= 2U && rx.payload[0] == COMM_SIDEBOARD_CALIBRATION) {
            if (rx.len != 2U) {
                (void)vesc_send_calibration_status(uart, rx.payload[1], 2U, latest);
                continue;
            }
            switch (rx.payload[1]) {
            case CAL_CMD_STILL_START:   return VESC_ACTION_CAL_STILL;
            case CAL_CMD_ROTATE_START:  return VESC_ACTION_CAL_ROTATE_START;
            case CAL_CMD_ROTATE_FINISH: return VESC_ACTION_CAL_ROTATE_FINISH;
            case CAL_CMD_CANCEL:        return VESC_ACTION_CAL_CANCEL;
            case CAL_CMD_ZERO_NAV:      return VESC_ACTION_ZERO_NAV;
            case CAL_CMD_ZUPT_ONCE:     return VESC_ACTION_ZUPT;
            case CAL_CMD_STATUS:
                (void)vesc_send_calibration_status(uart, CAL_CMD_STATUS, 0U, latest);
                break;
            case CAL_CMD_STATIONARY_ON:
                return VESC_ACTION_STATIONARY_ON;
            case CAL_CMD_STATIONARY_OFF:
                return VESC_ACTION_STATIONARY_OFF;
            default:
                (void)vesc_send_calibration_status(uart, rx.payload[1], 2U, latest);
                break;
            }
        }
    }

    return VESC_ACTION_NONE;
}
