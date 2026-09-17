#include "app_config.h"
#include "eeprom_flash.h"
#include "vesc_packet.h"
#include "stm32f1xx_hal.h"
#include <math.h>
#include <string.h>

#define SETTINGS_MAGIC   0x494D5534UL  /* ASCII IMU4 */
#define SETTINGS_VERSION 4U
#define SETTINGS_V3_MAGIC 0x494D5533UL /* Migrasi schema Tahap 1. */

typedef struct {
    uint32_t magic; uint16_t version; uint16_t length;
    float gyro_bias[3]; float accel_offset[3]; float accel_scale[3];
    float gyro_noise; float accel_process_noise; float gyro_bias_walk;
    float accel_bias_walk; float accel_dir_noise;
    float gyro_std[3]; float accel_std[3]; float calibration_temp_c;
    float still_gyro_std_max_dps; float still_accel_std_max_g;
    float rotate_residual_rms_g; float rotate_residual_max_g;
    uint32_t calibration_flags; uint32_t still_cal_count;
    uint32_t rotate_cal_count; uint32_t save_count;
    uint16_t crc16; uint16_t reserved;
} PersistedSettingsV3;

static uint16_t crc_struct(void *ptr, uint16_t len, uint16_t crc_offset)
{
    uint8_t tmp[256];
    if (!ptr || len > sizeof(tmp) || crc_offset + 1U >= len) return 0U;
    memcpy(tmp, ptr, len); tmp[crc_offset]=0U; tmp[crc_offset+1U]=0U;
    return vesc_crc16(tmp, len);
}

static uint16_t settings_crc(PersistedSettings *s)
{
    return crc_struct(s, (uint16_t)sizeof(*s), (uint16_t)((uint8_t *)&s->crc16-(uint8_t *)s));
}

static uint16_t settings_v3_crc(PersistedSettingsV3 *s)
{
    return crc_struct(s, (uint16_t)sizeof(*s), (uint16_t)((uint8_t *)&s->crc16-(uint8_t *)s));
}

void eeprom_settings_defaults(PersistedSettings *s)
{
    memset(s,0,sizeof(*s));
    s->magic=SETTINGS_MAGIC; s->version=SETTINGS_VERSION; s->length=(uint16_t)sizeof(*s);
    s->accel_transform[0]=1.0f; s->accel_transform[4]=1.0f; s->accel_transform[8]=1.0f;
    s->sensor_to_body_q[0]=1.0f;
    s->gyro_noise=ESKF_GYRO_NOISE_RAD; s->accel_process_noise=ESKF_ACCEL_PROCESS_NOISE;
    s->gyro_bias_walk=ESKF_GYRO_BIAS_WALK_RAD; s->accel_bias_walk=ESKF_ACCEL_BIAS_WALK;
    s->accel_dir_noise=ESKF_ACCEL_DIR_NOISE;
}

static int migrate_v3(PersistedSettings *out)
{
    PersistedSettingsV3 old;
    memcpy(&old,(const void *)EEPROM_FLASH_ADDR,sizeof(old));
    if (old.magic!=SETTINGS_V3_MAGIC || old.version!=3U || old.length!=sizeof(old) ||
        old.crc16!=settings_v3_crc(&old)) return 0;
    eeprom_settings_defaults(out);
    memcpy(out->gyro_bias,old.gyro_bias,sizeof(old.gyro_bias));
    memcpy(out->accel_offset,old.accel_offset,sizeof(old.accel_offset));
    out->accel_transform[0]=old.accel_scale[0];
    out->accel_transform[4]=old.accel_scale[1];
    out->accel_transform[8]=old.accel_scale[2];
    out->gyro_noise=old.gyro_noise; out->accel_process_noise=old.accel_process_noise;
    out->gyro_bias_walk=old.gyro_bias_walk; out->accel_bias_walk=old.accel_bias_walk;
    out->accel_dir_noise=old.accel_dir_noise;
    memcpy(out->gyro_std,old.gyro_std,sizeof(old.gyro_std));
    memcpy(out->accel_std,old.accel_std,sizeof(old.accel_std));
    out->calibration_temp_c=old.calibration_temp_c;
    out->still_gyro_std_max_dps=old.still_gyro_std_max_dps;
    out->still_accel_std_max_g=old.still_accel_std_max_g;
    out->rotate_residual_rms_g=old.rotate_residual_rms_g;
    out->rotate_residual_max_g=old.rotate_residual_max_g;
    out->calibration_flags=old.calibration_flags;
    out->still_cal_count=old.still_cal_count; out->rotate_cal_count=old.rotate_cal_count;
    out->save_count=old.save_count;
    return 1;
}

int eeprom_settings_load(PersistedSettings *s)
{
    if(!s) return 0;
    memcpy(s,(const void *)EEPROM_FLASH_ADDR,sizeof(*s));
    if(s->magic==SETTINGS_MAGIC && s->version==SETTINGS_VERSION && s->length==sizeof(*s) &&
       s->crc16==settings_crc(s)) {
        float qn=0.0f; for(int i=0;i<4;i++){if(!isfinite(s->sensor_to_body_q[i]))return 0;qn+=s->sensor_to_body_q[i]*s->sensor_to_body_q[i];}
        if(qn<0.5f || qn>1.5f) return 0;
        float qi=1.0f/sqrtf(qn); for(int i=0;i<4;i++)s->sensor_to_body_q[i]*=qi;
        for(int i=0;i<9;i++)if(!isfinite(s->accel_transform[i]))return 0;
        float *T=s->accel_transform;
        float det=T[0]*(T[4]*T[8]-T[5]*T[7])-T[1]*(T[3]*T[8]-T[5]*T[6])+T[2]*(T[3]*T[7]-T[4]*T[6]);
        if(!isfinite(det)||fabsf(det)<0.20f||fabsf(det)>5.0f)return 0;
        for(int i=0;i<3;i++){
            if(!isfinite(s->gyro_bias[i])||fabsf(s->gyro_bias[i])>2.0f)return 0;
            if(!isfinite(s->accel_offset[i])||fabsf(s->accel_offset[i])>5.0f*GRAVITY_MPS2)return 0;
            if(!isfinite(s->gyro_temp_slope[i])||fabsf(s->gyro_temp_slope[i])>0.05f)return 0;
            if(!isfinite(s->accel_temp_slope[i])||fabsf(s->accel_temp_slope[i])>1.0f)return 0;
            if(!isfinite(s->imu_position_body[i])||fabsf(s->imu_position_body[i])>5.0f)return 0;
            if(!isfinite(s->gyro_std[i])||s->gyro_std[i]<0.0f||s->gyro_std[i]>2.0f)return 0;
            if(!isfinite(s->accel_std[i])||s->accel_std[i]<0.0f||s->accel_std[i]>5.0f*GRAVITY_MPS2)return 0;
        }
        if(!isfinite(s->gyro_noise)||s->gyro_noise<=0.0f||s->gyro_noise>1.0f)return 0;
        if(!isfinite(s->accel_process_noise)||s->accel_process_noise<=0.0f||s->accel_process_noise>20.0f)return 0;
        if(!isfinite(s->gyro_bias_walk)||s->gyro_bias_walk<0.0f||s->gyro_bias_walk>0.5f)return 0;
        if(!isfinite(s->accel_bias_walk)||s->accel_bias_walk<0.0f||s->accel_bias_walk>5.0f)return 0;
        if(!isfinite(s->accel_dir_noise)||s->accel_dir_noise<=0.0f||s->accel_dir_noise>1.0f)return 0;
        if(!isfinite(s->calibration_temp_c)||s->calibration_temp_c<-80.0f||s->calibration_temp_c>150.0f)return 0;
        if(!isfinite(s->still_gyro_std_max_dps)||s->still_gyro_std_max_dps<0.0f||s->still_gyro_std_max_dps>20.0f)return 0;
        if(!isfinite(s->still_accel_std_max_g)||s->still_accel_std_max_g<0.0f||s->still_accel_std_max_g>2.0f)return 0;
        if(!isfinite(s->rotate_residual_rms_g)||s->rotate_residual_rms_g<0.0f||s->rotate_residual_rms_g>2.0f)return 0;
        if(!isfinite(s->rotate_residual_max_g)||s->rotate_residual_max_g<0.0f||s->rotate_residual_max_g>4.0f)return 0;
        return 1;
    }
    return migrate_v3(s) ? 2 : 0;
}

int eeprom_settings_save(PersistedSettings *s)
{
    if(!s || sizeof(*s)>EEPROM_FLASH_PAGE_SIZE) return 0;
    s->magic=SETTINGS_MAGIC; s->version=SETTINGS_VERSION; s->length=(uint16_t)sizeof(*s);
    s->save_count++; s->crc16=0U; s->crc16=settings_crc(s);
    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef erase={0}; uint32_t page_error=0U;
    erase.TypeErase=FLASH_TYPEERASE_PAGES; erase.PageAddress=EEPROM_FLASH_ADDR; erase.NbPages=1U;
    if(HAL_FLASHEx_Erase(&erase,&page_error)!=HAL_OK){HAL_FLASH_Lock();return 0;}
    const uint8_t *raw=(const uint8_t *)s;
    for(uint32_t off=0U;off<sizeof(*s);off+=2U){
        uint16_t hw=raw[off]; if(off+1U<sizeof(*s)) hw|=(uint16_t)raw[off+1U]<<8;
        if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,EEPROM_FLASH_ADDR+off,hw)!=HAL_OK){HAL_FLASH_Lock();return 0;}
    }
    HAL_FLASH_Lock();
    /* Verifikasi bytes mentah. eeprom_settings_load() boleh menormalisasi quaternion
     * pada copy hasil baca sehingga tidak cocok dipakai sebagai byte-for-byte verify. */
    PersistedSettings verify;
    memcpy(&verify,(const void *)EEPROM_FLASH_ADDR,sizeof(verify));
    return verify.magic==SETTINGS_MAGIC && verify.version==SETTINGS_VERSION &&
           verify.length==sizeof(verify) && verify.crc16==settings_crc(&verify) &&
           memcmp(&verify,s,sizeof(*s))==0;
}
