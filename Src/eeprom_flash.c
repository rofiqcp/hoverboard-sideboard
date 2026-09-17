#include "eeprom_flash.h"
#include "app_config.h"
#include "vesc_packet.h"
#include "stm32f1xx_hal.h"
#include <string.h>

#define SETTINGS_MAGIC   0x494D5533UL  /* ASCII IMU3 */
#define SETTINGS_VERSION 3U

static uint16_t settings_crc(PersistedSettings *s)
{
    uint16_t old_crc = s->crc16;
    s->crc16 = 0U;
    uint16_t crc = vesc_crc16((const uint8_t *)s, (uint16_t)sizeof(*s));
    s->crc16 = old_crc;
    return crc;
}

void eeprom_settings_defaults(PersistedSettings *s)
{
    memset(s, 0, sizeof(*s));
    s->magic = SETTINGS_MAGIC;
    s->version = SETTINGS_VERSION;
    s->length = (uint16_t)sizeof(*s);

    for (int i = 0; i < 3; i++) {
        s->accel_scale[i] = 1.0f;
    }

    s->gyro_noise = ESKF_GYRO_NOISE_RAD;
    s->accel_process_noise = ESKF_ACCEL_PROCESS_NOISE;
    s->gyro_bias_walk = ESKF_GYRO_BIAS_WALK_RAD;
    s->accel_bias_walk = ESKF_ACCEL_BIAS_WALK;
    s->accel_dir_noise = ESKF_ACCEL_DIR_NOISE;
}

int eeprom_settings_load(PersistedSettings *s)
{
    if (!s) {
        return 0;
    }

    memcpy(s, (const void *)EEPROM_FLASH_ADDR, sizeof(*s));

    if (s->magic != SETTINGS_MAGIC ||
        s->version != SETTINGS_VERSION ||
        s->length != sizeof(*s)) {
        return 0;
    }

    if (s->crc16 != settings_crc(s)) {
        return 0;
    }

    for (int i = 0; i < 3; i++) {
        if (!(s->accel_scale[i] > 0.5f && s->accel_scale[i] < 1.5f)) {
            return 0;
        }
    }
    return 1;
}

int eeprom_settings_save(PersistedSettings *s)
{
    if (!s) {
        return 0;
    }

    s->magic = SETTINGS_MAGIC;
    s->version = SETTINGS_VERSION;
    s->length = (uint16_t)sizeof(*s);
    s->save_count++;
    s->crc16 = 0U;
    s->crc16 = settings_crc(s);

    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef erase = {0};
    uint32_t page_error = 0U;
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.PageAddress = EEPROM_FLASH_ADDR;
    erase.NbPages = 1U;

    if (HAL_FLASHEx_Erase(&erase, &page_error) != HAL_OK) {
        HAL_FLASH_Lock();
        return 0;
    }

    const uint8_t *raw = (const uint8_t *)s;
    for (uint32_t off = 0U; off < sizeof(*s); off += 2U) {
        uint16_t halfword = raw[off];
        if (off + 1U < sizeof(*s)) {
            halfword |= (uint16_t)raw[off + 1U] << 8;
        }

        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
                              EEPROM_FLASH_ADDR + off,
                              halfword) != HAL_OK) {
            HAL_FLASH_Lock();
            return 0;
        }
    }

    HAL_FLASH_Lock();

    PersistedSettings verify;
    return eeprom_settings_load(&verify) &&
           memcmp(&verify, s, sizeof(*s)) == 0;
}
