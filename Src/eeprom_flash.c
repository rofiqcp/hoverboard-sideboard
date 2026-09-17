#include "app_config.h"
#include "eeprom_flash.h"
#include "vesc_packet.h"
#include "stm32f1xx_hal.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

#define SETTINGS_MAGIC       0x494D5534UL  /* ASCII IMU4 */
#define SETTINGS_VERSION     4U
#define SETTINGS_V3_MAGIC    0x494D5533UL
#define JOURNAL_MAGIC        0x4A4D5534UL  /* JMU4 */
#define JOURNAL_COMMIT       0xA55AU

typedef struct {
    uint32_t magic;
    uint32_t generation;
    uint32_t generation_inv;
    uint16_t payload_len;
    uint16_t payload_crc;
} JournalHeader;

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

#define JOURNAL_PAYLOAD_OFFSET ((uint32_t)sizeof(JournalHeader))
#define JOURNAL_COMMIT_OFFSET  (JOURNAL_PAYLOAD_OFFSET + (uint32_t)sizeof(PersistedSettings))

typedef char journal_fits_page[(JOURNAL_COMMIT_OFFSET + 2U <= EEPROM_FLASH_PAGE_SIZE) ? 1 : -1];

static uint16_t crc_struct(const void *ptr, uint16_t len, uint16_t crc_offset)
{
    uint8_t tmp[256];
    if (!ptr || len > sizeof(tmp) || crc_offset + 1U >= len) return 0U;
    memcpy(tmp, ptr, len);
    tmp[crc_offset] = 0U; tmp[crc_offset + 1U] = 0U;
    return vesc_crc16(tmp, len);
}

static uint16_t settings_crc(PersistedSettings *s)
{
    return crc_struct(s, (uint16_t)sizeof(*s),
                      (uint16_t)((uint8_t *)&s->crc16 - (uint8_t *)s));
}

static uint16_t settings_v3_crc(PersistedSettingsV3 *s)
{
    return crc_struct(s, (uint16_t)sizeof(*s),
                      (uint16_t)((uint8_t *)&s->crc16 - (uint8_t *)s));
}

void eeprom_settings_defaults(PersistedSettings *s)
{
    memset(s, 0, sizeof(*s));
    s->magic = SETTINGS_MAGIC; s->version = SETTINGS_VERSION; s->length = (uint16_t)sizeof(*s);
    s->accel_transform[0] = 1.0f; s->accel_transform[4] = 1.0f; s->accel_transform[8] = 1.0f;
    s->sensor_to_body_q[0] = 1.0f;
    s->gyro_noise = ESKF_GYRO_NOISE_RAD; s->accel_process_noise = ESKF_ACCEL_PROCESS_NOISE;
    s->gyro_bias_walk = ESKF_GYRO_BIAS_WALK_RAD; s->accel_bias_walk = ESKF_ACCEL_BIAS_WALK;
    s->accel_dir_noise = ESKF_ACCEL_DIR_NOISE;
}

static int settings_validate(PersistedSettings *s, int normalize_q)
{
    if (!s || s->magic != SETTINGS_MAGIC || s->version != SETTINGS_VERSION ||
        s->length != sizeof(*s) || s->crc16 != settings_crc(s)) return 0;

    float qn = 0.0f;
    for (int i=0;i<4;i++) { if (!isfinite(s->sensor_to_body_q[i])) return 0; qn += s->sensor_to_body_q[i]*s->sensor_to_body_q[i]; }
    if (qn < 0.5f || qn > 1.5f) return 0;
    if (normalize_q) { float qi=1.0f/sqrtf(qn); for(int i=0;i<4;i++) s->sensor_to_body_q[i]*=qi; }

    for(int i=0;i<9;i++) if(!isfinite(s->accel_transform[i])) return 0;
    float *T=s->accel_transform;
    float det=T[0]*(T[4]*T[8]-T[5]*T[7])-T[1]*(T[3]*T[8]-T[5]*T[6])+T[2]*(T[3]*T[7]-T[4]*T[6]);
    if(!isfinite(det)||fabsf(det)<0.20f||fabsf(det)>5.0f) return 0;

    for(int i=0;i<3;i++) {
        if(!isfinite(s->gyro_bias[i])||fabsf(s->gyro_bias[i])>2.0f) return 0;
        if(!isfinite(s->accel_offset[i])||fabsf(s->accel_offset[i])>5.0f*GRAVITY_MPS2) return 0;
        if(!isfinite(s->gyro_temp_slope[i])||fabsf(s->gyro_temp_slope[i])>0.05f) return 0;
        if(!isfinite(s->accel_temp_slope[i])||fabsf(s->accel_temp_slope[i])>1.0f) return 0;
        if(!isfinite(s->imu_position_body[i])||fabsf(s->imu_position_body[i])>5.0f) return 0;
        if(!isfinite(s->gyro_std[i])||s->gyro_std[i]<0.0f||s->gyro_std[i]>2.0f) return 0;
        if(!isfinite(s->accel_std[i])||s->accel_std[i]<0.0f||s->accel_std[i]>5.0f*GRAVITY_MPS2) return 0;
    }
    if(!isfinite(s->gyro_noise)||s->gyro_noise<=0.0f||s->gyro_noise>1.0f) return 0;
    if(!isfinite(s->accel_process_noise)||s->accel_process_noise<=0.0f||s->accel_process_noise>20.0f) return 0;
    if(!isfinite(s->gyro_bias_walk)||s->gyro_bias_walk<0.0f||s->gyro_bias_walk>0.5f) return 0;
    if(!isfinite(s->accel_bias_walk)||s->accel_bias_walk<0.0f||s->accel_bias_walk>5.0f) return 0;
    if(!isfinite(s->accel_dir_noise)||s->accel_dir_noise<=0.0f||s->accel_dir_noise>1.0f) return 0;
    if(!isfinite(s->calibration_temp_c)||s->calibration_temp_c<-80.0f||s->calibration_temp_c>150.0f) return 0;
    if(!isfinite(s->still_gyro_std_max_dps)||s->still_gyro_std_max_dps<0.0f||s->still_gyro_std_max_dps>20.0f) return 0;
    if(!isfinite(s->still_accel_std_max_g)||s->still_accel_std_max_g<0.0f||s->still_accel_std_max_g>2.0f) return 0;
    if(!isfinite(s->rotate_residual_rms_g)||s->rotate_residual_rms_g<0.0f||s->rotate_residual_rms_g>2.0f) return 0;
    if(!isfinite(s->rotate_residual_max_g)||s->rotate_residual_max_g<0.0f||s->rotate_residual_max_g>4.0f) return 0;
    return 1;
}

static int generation_newer(uint32_t a, uint32_t b)
{
    return (int32_t)(a - b) > 0;
}

static int journal_read(uint32_t addr, PersistedSettings *out, uint32_t *generation)
{
    JournalHeader h;
    memcpy(&h, (const void *)addr, sizeof(h));
    if (h.magic != JOURNAL_MAGIC || h.generation_inv != ~h.generation ||
        h.payload_len != sizeof(PersistedSettings)) return 0;
    if (*(const volatile uint16_t *)(addr + JOURNAL_COMMIT_OFFSET) != JOURNAL_COMMIT) return 0;

    PersistedSettings s;
    memcpy(&s, (const void *)(addr + JOURNAL_PAYLOAD_OFFSET), sizeof(s));
    if (h.payload_crc != s.crc16 || !settings_validate(&s, 1)) return 0;
    if (out) *out = s;
    if (generation) *generation = h.generation;
    return 1;
}

static int load_legacy_v4(PersistedSettings *out)
{
    PersistedSettings s;
    memcpy(&s, (const void *)EEPROM_FLASH_ADDR_B, sizeof(s));
    if (!settings_validate(&s, 1)) return 0;
    *out = s; return 1;
}

static int migrate_v3(PersistedSettings *out)
{
    PersistedSettingsV3 old;
    memcpy(&old,(const void *)EEPROM_FLASH_ADDR_B,sizeof(old));
    if (old.magic!=SETTINGS_V3_MAGIC || old.version!=3U || old.length!=sizeof(old) ||
        old.crc16!=settings_v3_crc(&old)) return 0;
    eeprom_settings_defaults(out);
    memcpy(out->gyro_bias,old.gyro_bias,sizeof(old.gyro_bias));
    memcpy(out->accel_offset,old.accel_offset,sizeof(old.accel_offset));
    out->accel_transform[0]=old.accel_scale[0]; out->accel_transform[4]=old.accel_scale[1]; out->accel_transform[8]=old.accel_scale[2];
    out->gyro_noise=old.gyro_noise; out->accel_process_noise=old.accel_process_noise;
    out->gyro_bias_walk=old.gyro_bias_walk; out->accel_bias_walk=old.accel_bias_walk; out->accel_dir_noise=old.accel_dir_noise;
    memcpy(out->gyro_std,old.gyro_std,sizeof(old.gyro_std)); memcpy(out->accel_std,old.accel_std,sizeof(old.accel_std));
    out->calibration_temp_c=old.calibration_temp_c;
    out->still_gyro_std_max_dps=old.still_gyro_std_max_dps; out->still_accel_std_max_g=old.still_accel_std_max_g;
    out->rotate_residual_rms_g=old.rotate_residual_rms_g; out->rotate_residual_max_g=old.rotate_residual_max_g;
    out->calibration_flags=old.calibration_flags; out->still_cal_count=old.still_cal_count;
    out->rotate_cal_count=old.rotate_cal_count; out->save_count=old.save_count;
    out->crc16=0U; out->crc16=settings_crc(out);
    return 1;
}

int eeprom_settings_load(PersistedSettings *s)
{
    if (!s) return 0;
    PersistedSettings a,b; uint32_t ga=0U,gb=0U;
    int va=journal_read(EEPROM_FLASH_ADDR_A,&a,&ga);
    int vb=journal_read(EEPROM_FLASH_ADDR_B,&b,&gb);
    if (va && vb) { *s = generation_newer(ga,gb) ? a : b; return 1; }
    if (va) { *s=a; return 1; }
    if (vb) { *s=b; return 1; }
    if (load_legacy_v4(s)) return 3; /* single-page IMU4 -> journal */
    return migrate_v3(s) ? 2 : 0;
}

static int erase_page_unlocked(uint32_t addr)
{
    FLASH_EraseInitTypeDef erase={0}; uint32_t page_error=0U;
    erase.TypeErase=FLASH_TYPEERASE_PAGES; erase.PageAddress=addr; erase.NbPages=1U;
    return HAL_FLASHEx_Erase(&erase,&page_error)==HAL_OK;
}

static int program_bytes_unlocked(uint32_t addr,const void *ptr,uint32_t len)
{
    const uint8_t *raw=(const uint8_t *)ptr;
    for(uint32_t off=0U;off<len;off+=2U) {
        uint16_t hw=raw[off]; if(off+1U<len) hw|=(uint16_t)raw[off+1U]<<8; else hw|=0xFF00U;
        if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,addr+off,hw)!=HAL_OK) return 0;
        if(*(const volatile uint16_t *)(addr+off)!=hw) return 0;
    }
    return 1;
}

int eeprom_settings_save(PersistedSettings *s)
{
    if(!s || sizeof(*s)>EEPROM_FLASH_PAGE_SIZE || JOURNAL_COMMIT_OFFSET+2U>EEPROM_FLASH_PAGE_SIZE) return 0;

    PersistedSettings tmp; uint32_t ga=0U,gb=0U;
    int va=journal_read(EEPROM_FLASH_ADDR_A,&tmp,&ga);
    int vb=journal_read(EEPROM_FLASH_ADDR_B,&tmp,&gb);
    uint32_t newest=0U,target=EEPROM_FLASH_ADDR_A;
    if(va && vb) {
        if(generation_newer(ga,gb)){newest=ga;target=EEPROM_FLASH_ADDR_B;}
        else {newest=gb;target=EEPROM_FLASH_ADDR_A;}
    } else if(va) {newest=ga;target=EEPROM_FLASH_ADDR_B;}
    else if(vb) {newest=gb;target=EEPROM_FLASH_ADDR_A;}
    /* Jika hanya legacy di B yang valid, target tetap A sehingga legacy tidak disentuh. */

    PersistedSettings candidate=*s;
    candidate.magic=SETTINGS_MAGIC; candidate.version=SETTINGS_VERSION; candidate.length=(uint16_t)sizeof(candidate);
    candidate.save_count=s->save_count+1U; candidate.crc16=0U; candidate.crc16=settings_crc(&candidate);
    if(!settings_validate(&candidate,0)) return 0;

    JournalHeader h={JOURNAL_MAGIC,newest+1U,~(newest+1U),(uint16_t)sizeof(candidate),candidate.crc16};
    HAL_FLASH_Unlock();
    int ok=erase_page_unlocked(target) &&
           program_bytes_unlocked(target,&h,sizeof(h)) &&
           program_bytes_unlocked(target+JOURNAL_PAYLOAD_OFFSET,&candidate,sizeof(candidate));
    if(ok) {
        JournalHeader vh; PersistedSettings vs;
        memcpy(&vh,(const void *)target,sizeof(vh));
        memcpy(&vs,(const void *)(target+JOURNAL_PAYLOAD_OFFSET),sizeof(vs));
        ok=memcmp(&vh,&h,sizeof(h))==0 && memcmp(&vs,&candidate,sizeof(vs))==0;
    }
    if(ok) {
        uint16_t commit=JOURNAL_COMMIT;
        ok=program_bytes_unlocked(target+JOURNAL_COMMIT_OFFSET,&commit,sizeof(commit));
    }
    HAL_FLASH_Lock();
    if(!ok) return 0;

    PersistedSettings verify; uint32_t vg=0U;
    if(!journal_read(target,&verify,&vg) || vg!=h.generation) return 0;
    *s=candidate; /* RAM baru ikut berubah setelah flash record benar-benar committed. */
    return 1;
}
