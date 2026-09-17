/*
 * Firmware sideboard IMU.
 *
 * Alur utama:
 * raw MPU6xxx -> kalibrasi tersimpan -> ESKF 15-state -> protokol VESC.
 */
#include "app_config.h"
#include "board.h"
#include "imu_mpu6xxx.h"
#include "imu_preintegrator.h"
#include "imu_calibration.h"
#include "eskf_nav.h"
#include "eeprom_flash.h"
#include "vesc_packet.h"
#include <math.h>
#include <string.h>

typedef struct {
    float accel_avg[3];
    float gyro_avg[3];
    float accel_sq_avg[3];
    float gyro_sq_avg[3];
    float temperature_avg;
    uint16_t valid;
    uint8_t stationary;
    uint8_t have_last;
    ImuSample last_sample;
} StartupMeasurement;

typedef struct {
    uint32_t last_any_us, last_wheel_us, last_nhc_us;
    uint32_t last_yaw_us, last_vel_us, last_pos_us;
    uint32_t reject_count;
} ExternalAidStatus;

static volatile uint32_t filter_health_reset_count = 0U;

static void rpy_to_quat(float roll,float pitch,float yaw,float q[4])
{
    float cr=cosf(0.5f*roll),sr=sinf(0.5f*roll);
    float cp=cosf(0.5f*pitch),sp=sinf(0.5f*pitch);
    float cy=cosf(0.5f*yaw),sy=sinf(0.5f*yaw);
    q[0]=cr*cp*cy+sr*sp*sy; q[1]=sr*cp*cy-cr*sp*sy;
    q[2]=cr*sp*cy+sr*cp*sy; q[3]=cr*cp*sy-sr*sp*cy;
    float n=sqrtf(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
    if(n>1e-6f)for(int i=0;i<4;i++)q[i]/=n;
}

static uint32_t aid_age_us(uint32_t now_us,uint32_t stamp_us)
{
    return stamp_us==0U ? 0U : (uint32_t)(now_us-stamp_us);
}

static int aid_recent(uint32_t now_us,uint32_t last_us)
{
    return last_us!=0U && (uint32_t)(now_us-last_us)<=AID_STATUS_TIMEOUT_US;
}

static void service_bootloader_while_starting(void)
{
    VescImuState empty_state;
    memset(&empty_state, 0, sizeof(empty_state));

    VescAction action = vesc_process_rx(&huart2, NULL);
    if (action == VESC_ACTION_BOOTLOADER) {
        HAL_Delay(5U);
        board_reboot_to_bootloader();
    }
}

static void wait_for_imu(void)
{
    while (!imu_mpu6xxx_init()) {
        /* Update firmware harus tetap mungkin walaupun IMU/I2C sedang bermasalah. */
        service_bootloader_while_starting();
        board_i2c_recover();
        for (uint8_t i = 0U; i < 25U; i++) {
            HAL_Delay(10U);
            service_bootloader_while_starting();
        }
    }
}

static void startup_measure(StartupMeasurement *m)
{
    memset(m, 0, sizeof(*m));
    (void)imu_mpu6xxx_fifo_reset();

    uint32_t start_us = board_micros();
    uint8_t error_streak = 0U;
    while (m->valid < STARTUP_CALIB_SAMPLES &&
           (uint32_t)(board_micros() - start_us) < 3500000U) {
        ImuSample samples[8];
        uint8_t count = 0U;
        if (!imu_mpu6xxx_read_fifo(samples, 8U, &count)) {
            if (++error_streak >= IMU_REINIT_ERROR_COUNT) {
                board_i2c_recover();
                (void)imu_mpu6xxx_init();
                error_streak = 0U;
            }
        } else {
            error_streak = 0U;
            for (uint8_t n=0U; n<count && m->valid<STARTUP_CALIB_SAMPLES; n++) {
                const ImuSample *sample=&samples[n];
                for (int i=0;i<3;i++) {
                    m->accel_avg[i]+=sample->accel_mps2[i];
                    m->gyro_avg[i]+=sample->gyro_rads[i];
                    m->accel_sq_avg[i]+=sample->accel_mps2[i]*sample->accel_mps2[i];
                    m->gyro_sq_avg[i]+=sample->gyro_rads[i]*sample->gyro_rads[i];
                }
                m->temperature_avg+=sample->temperature_c;
                m->last_sample=*sample;
                m->have_last=1U;
                m->valid++;
            }
        }
        service_bootloader_while_starting();
        HAL_Delay(4U);
    }

    if (m->valid == 0U) return;

    float inv_n=1.0f/(float)m->valid;
    float max_gyro_std_dps=0.0f, max_accel_std_g=0.0f;
    for (int i=0;i<3;i++) {
        m->accel_avg[i]*=inv_n;
        m->gyro_avg[i]*=inv_n;
        m->accel_sq_avg[i]*=inv_n;
        m->gyro_sq_avg[i]*=inv_n;
        float gv=fmaxf(0.0f,m->gyro_sq_avg[i]-m->gyro_avg[i]*m->gyro_avg[i]);
        float av=fmaxf(0.0f,m->accel_sq_avg[i]-m->accel_avg[i]*m->accel_avg[i]);
        float gdps=sqrtf(gv)*57.2957795f;
        float ag=sqrtf(av)/GRAVITY_MPS2;
        if (gdps>max_gyro_std_dps) max_gyro_std_dps=gdps;
        if (ag>max_accel_std_g) max_accel_std_g=ag;
    }
    m->temperature_avg*=inv_n;

    float an=sqrtf(m->accel_avg[0]*m->accel_avg[0]+
                   m->accel_avg[1]*m->accel_avg[1]+
                   m->accel_avg[2]*m->accel_avg[2]);
    float accel_error_g=fabsf(an/GRAVITY_MPS2-1.0f);
    float gyro_mean_dps=sqrtf(m->gyro_avg[0]*m->gyro_avg[0]+
                              m->gyro_avg[1]*m->gyro_avg[1]+
                              m->gyro_avg[2]*m->gyro_avg[2])*57.2957795f;

    m->stationary = m->valid > (STARTUP_CALIB_SAMPLES*9U/10U) &&
                    gyro_mean_dps < STARTUP_STILL_GYRO_DPS &&
                    max_gyro_std_dps < STARTUP_STILL_GYRO_STD_DPS &&
                    accel_error_g < STARTUP_STILL_ACCEL_ERR_G &&
                    max_accel_std_g < STARTUP_STILL_ACCEL_STD_G;
}

static void init_filter(EskfNav *eskf,
                        const ImuSample *raw,
                        const PersistedSettings *settings)
{
    float accel[3];
    float gyro[3];
    imu_apply_static_calibration(raw, settings, accel, gyro);

    eskf_nav_init(eskf, accel);
    eskf->gyro_noise = settings->gyro_noise;
    eskf->accel_noise = settings->accel_process_noise;
    eskf->gyro_bias_walk = settings->gyro_bias_walk;
    eskf->accel_bias_walk = settings->accel_bias_walk;
    eskf->accel_dir_noise = settings->accel_dir_noise;
}

static int recover_filter_if_unhealthy(EskfNav *eskf,
                                      const ImuSample *raw,
                                      const PersistedSettings *settings,
                                      ImuPreintegrator *preintegrator,
                                      uint32_t *last_fifo_resync,
                                      uint8_t sample_is_still,
                                      uint8_t *startup_zupt)
{
    if (eskf_nav_is_healthy(eskf)) return 0;
    filter_health_reset_count++;
    init_filter(eskf,raw,settings);
    imu_preintegrator_init(preintegrator);
    (void)imu_mpu6xxx_fifo_reset();
    if(last_fifo_resync)*last_fifo_resync=imu_mpu6xxx_get_fifo_stats()->fifo_resync_count;
    if(startup_zupt)*startup_zupt=sample_is_still?1U:0U;
    return 1;
}

typedef struct {
    uint16_t enter_count;
    uint8_t exit_count;
    uint8_t still;
} StillnessDetector;

static int stillness_update(StillnessDetector *d, const float accel[3], const float gyro[3])
{
    /* Bandingkan norm kuadrat agar loop 100 Hz tidak membayar dua sqrtf. */
    const float dps_to_rad = 0.01745329251994329577f;
    float gyro_n2=gyro[0]*gyro[0]+gyro[1]*gyro[1]+gyro[2]*gyro[2];
    float accel_n2=accel[0]*accel[0]+accel[1]*accel[1]+accel[2]*accel[2];
    float g2=GRAVITY_MPS2*GRAVITY_MPS2;
    float enter_g_lo=0.96f*0.96f*g2, enter_g_hi=1.04f*1.04f*g2;
    float exit_g_lo=0.92f*0.92f*g2, exit_g_hi=1.08f*1.08f*g2;
    float enter_w2=dps_to_rad*dps_to_rad;
    float exit_w2=4.0f*dps_to_rad*dps_to_rad;
    int enter_ok=gyro_n2<enter_w2 && accel_n2>enter_g_lo && accel_n2<enter_g_hi;
    int exit_bad=gyro_n2>exit_w2 || accel_n2<exit_g_lo || accel_n2>exit_g_hi;

    if (!d->still) {
        if (enter_ok) {
            if (d->enter_count<1000U) d->enter_count++;
            if (d->enter_count>=20U) { d->still=1U; d->exit_count=0U; }
        } else d->enter_count=0U;
    } else {
        if (exit_bad) {
            if (d->exit_count<50U) d->exit_count++;
            /* Noise/spike singkat tidak boleh melepas stationary startup.
             * Butuh motion evidence persisten 200 ms (20 tick @100 Hz). */
            if (d->exit_count>=20U) { d->still=0U; d->enter_count=0U; }
        } else d->exit_count=0U;
    }
    return d->still != 0U;
}

static void sync_calibration_status(VescImuState *out, const ImuCalibration *cal)
{
    if (!out || !cal) return;
    out->cal_state = (uint8_t)cal->state;
    out->cal_coverage = cal->coverage;
    out->cal_error = cal->error_code;
    out->cal_progress = cal->progress;
    out->flags &= (uint16_t)~IMU_FLAG_CAL_ACTIVE;
    if (cal->state == IMU_CAL_STILL || cal->state == IMU_CAL_ROTATE) {
        out->flags |= IMU_FLAG_CAL_ACTIVE;
    }
}

static float zero_rate_sigma_from_settings(const PersistedSettings *settings)
{
    /*
     * Noise zero-rate mengikuti noise gyro hasil kalibrasi diam.
     * 1.5 sigma memberi margin terhadap variasi temperatur dan kuantisasi.
     */
    float sigma = ZERO_RATE_SIGMA_RAD;
    if (settings &&
        (settings->calibration_flags & CAL_FLAG_STILL_VALID) &&
        settings->still_gyro_std_max_dps > 0.0f) {
        sigma = settings->still_gyro_std_max_dps * 0.01745329252f * 1.5f;
        if (sigma < 0.0025f) sigma = 0.0025f;
        if (sigma > 0.0100f) sigma = 0.0100f;
    }
    return sigma;
}

static void fill_telemetry(VescImuState *out,
                           const ImuSample *raw,
                           const float accel[3],
                           const float gyro[3],
                           const EskfNav *eskf,
                           const ImuCalibration *cal,
                           const PersistedSettings *settings,
                           uint32_t sequence,
                           uint32_t time_us,
                           int gravity_ok,
                           int eeprom_valid,
                           int startup_stationary,
                           int zupt_applied,
                           int master_stationary,
                           const ExternalAidStatus *aid)
{
    memset(out, 0, sizeof(*out));
    out->sequence = sequence;
    out->time_us = time_us;
    out->raw = *raw;

    eskf_nav_get_euler_rad(eskf, &out->roll_rad, &out->pitch_rad, &out->yaw_rad);
    memcpy(out->q, eskf->q, sizeof(out->q));
    for (int i = 0; i < 3; i++) {
        /* Native VESC menerima sensor yang sudah dikoreksi bias residual ESKF. */
        out->accel_cal[i] = accel[i] - eskf->accel_bias[i];
        out->gyro_cal[i] = gyro[i] - eskf->gyro_bias[i];
    }
    memcpy(out->velocity, eskf->velocity, sizeof(out->velocity));
    memcpy(out->position, eskf->position, sizeof(out->position));
    eskf_nav_linear_accel_world(eskf, accel, out->linear_accel_world);

    out->cal_state = (uint8_t)cal->state;
    out->cal_coverage = cal->coverage;
    out->cal_error = cal->error_code;
    out->cal_progress = cal->progress;

    out->flags = IMU_FLAG_SENSOR_OK | IMU_FLAG_ESKF_OK;
    if (gravity_ok) {
        out->flags |= IMU_FLAG_ACCEL_FUSED;
    }
    if (eeprom_valid) {
        out->flags |= IMU_FLAG_EEPROM_VALID;
    }
    if (startup_stationary) {
        out->flags |= IMU_FLAG_STARTUP_STILL;
    }
    if (settings->calibration_flags & CAL_FLAG_STILL_VALID) {
        out->flags |= IMU_FLAG_STILL_CAL_VALID;
    }
    if (settings->calibration_flags & CAL_FLAG_ROTATE_VALID) {
        out->flags |= IMU_FLAG_ROTATE_CAL_VALID;
    }
    if (cal->state == IMU_CAL_STILL || cal->state == IMU_CAL_ROTATE) {
        out->flags |= IMU_FLAG_CAL_ACTIVE;
    }
    if (zupt_applied) {
        out->flags |= IMU_FLAG_ZUPT_APPLIED;
    }
    if (master_stationary) {
        out->flags |= IMU_FLAG_MASTER_STATIONARY;
    }

    eskf_nav_get_std(eskf,out->attitude_std_rad,out->velocity_std_mps,out->position_std_m);
    out->aid_age_ms=65535U;
    if(aid){
        uint32_t best=0xFFFFFFFFUL;
        if(aid_recent(time_us,aid->last_wheel_us)){out->flags|=IMU_FLAG_WHEEL_AID;uint32_t a=time_us-aid->last_wheel_us;if(a<best)best=a;}
        if(aid_recent(time_us,aid->last_nhc_us)){out->flags|=IMU_FLAG_NHC_AID;uint32_t a=time_us-aid->last_nhc_us;if(a<best)best=a;}
        if(aid_recent(time_us,aid->last_yaw_us)){out->flags|=IMU_FLAG_YAW_AID|IMU_FLAG_YAW_ABSOLUTE;uint32_t a=time_us-aid->last_yaw_us;if(a<best)best=a;}
        if(aid_recent(time_us,aid->last_vel_us)){out->flags|=IMU_FLAG_VEL_AID;uint32_t a=time_us-aid->last_vel_us;if(a<best)best=a;}
        if(aid_recent(time_us,aid->last_pos_us)){out->flags|=IMU_FLAG_POS_AID;uint32_t a=time_us-aid->last_pos_us;if(a<best)best=a;}
        if(best!=0xFFFFFFFFUL)out->aid_age_ms=(uint16_t)((best/1000U)>65535U?65535U:(best/1000U));
        out->aid_reject_count=(uint16_t)(aid->reject_count>65535U?65535U:aid->reject_count);
    }
    out->nav_status=NAV_STATUS_ATT_VALID|NAV_STATUS_COV_VALID;
    if(aid && (aid_recent(time_us,aid->last_wheel_us)||aid_recent(time_us,aid->last_vel_us)))
        out->nav_status|=NAV_STATUS_VEL_AIDED;
    if(aid && aid_recent(time_us,aid->last_pos_us))out->nav_status|=NAV_STATUS_POS_AIDED;
    if(aid && aid_recent(time_us,aid->last_yaw_us))out->nav_status|=NAV_STATUS_YAW_AIDED;
    if(!(out->nav_status&NAV_STATUS_POS_AIDED))out->nav_status|=NAV_STATUS_DEAD_RECKONING;
    if(master_stationary)out->nav_status|=NAV_STATUS_STATIONARY_BOUND;
    if(filter_health_reset_count>0U)out->nav_status|=NAV_STATUS_HEALTH_RECOVERED;
    out->health_reset_count=(uint16_t)(filter_health_reset_count>65535U?65535U:filter_health_reset_count);
}

int main(void)
{
    board_init();
    wait_for_imu();

    PersistedSettings settings;
    int settings_load_status = eeprom_settings_load(&settings);
    int eeprom_valid = settings_load_status != 0;
    if (!eeprom_valid) {
        eeprom_settings_defaults(&settings);
    } else if (settings_load_status == 2) {
        /* Migrasi IMU3 -> IMU4 dipersist sekali. Semua bias/noise/counter lama
         * dipertahankan, field Tahap 2 mendapat default identity/zero. */
        eeprom_valid = eeprom_settings_save(&settings);
    }

    StartupMeasurement startup;
    startup_measure(&startup);

    /* Saat boot benar-benar diam, pakai bias gyro terbaru untuk sesi ini.
     * Jika EEPROM schema v2 belum pernah valid, simpan hasil pertama ini. */
    if (startup.stationary) {
        float old_ref_temp=settings.calibration_temp_c;
        float new_ref_temp=startup.temperature_avg;
        float ref_shift=new_ref_temp-old_ref_temp;
        for (int i = 0; i < 3; i++) {
            settings.gyro_bias[i] = startup.gyro_avg[i];
            /* Jika slope thermal pernah dikonfigurasi, pindahkan reference
             * accel tanpa mengubah model koreksinya. */
            settings.accel_offset[i] += settings.accel_temp_slope[i]*ref_shift;
        }
        settings.calibration_temp_c=new_ref_temp;
        settings.calibration_flags |= CAL_FLAG_STILL_VALID;

        if (!eeprom_valid) {
            for (int i = 0; i < 3; i++) {
                float gyro_var = fmaxf(0.0f, startup.gyro_sq_avg[i] -
                                       startup.gyro_avg[i] * startup.gyro_avg[i]);
                float accel_var = fmaxf(0.0f, startup.accel_sq_avg[i] -
                                        startup.accel_avg[i] * startup.accel_avg[i]);
                settings.gyro_std[i] = sqrtf(gyro_var);
                settings.accel_std[i] = sqrtf(accel_var);
            }
            settings.calibration_temp_c = startup.temperature_avg;
            float max_g_std=0.0f, max_a_std=0.0f;
            for (int i=0;i<3;i++) {
                float gdps=settings.gyro_std[i]*57.2957795f;
                float ag=settings.accel_std[i]/GRAVITY_MPS2;
                if (gdps>max_g_std) max_g_std=gdps;
                if (ag>max_a_std) max_a_std=ag;
            }
            settings.still_gyro_std_max_dps=max_g_std;
            settings.still_accel_std_max_g=max_a_std;
            settings.still_cal_count++;
            eeprom_valid = eeprom_settings_save(&settings);
        }
    }

    ImuSample raw;
    if (startup.have_last) {
        raw=startup.last_sample;
    } else {
        while (!imu_mpu6xxx_read(&raw)) {
            service_bootloader_while_starting();
            board_i2c_recover();
            (void)imu_mpu6xxx_init();
            HAL_Delay(20U);
        }
    }

    EskfNav eskf;
    init_filter(&eskf, &raw, &settings);

    /* Buang seluruh FIFO yang terkumpul selama kalibrasi startup. Data lama tidak
     * boleh ikut diintegrasikan sebagai gerakan setelah ESKF mulai. */
    if (!imu_mpu6xxx_fifo_reset()) {
        board_i2c_recover();
        if (!imu_mpu6xxx_init()) board_panic();
    }

    ImuPreintegrator preintegrator;
    imu_preintegrator_init(&preintegrator);

    ImuCalibration calibration;
    imu_calibration_init(&calibration);
    /* Hasil startup 2 detik adalah evidence diam yang lebih kuat daripada
     * menunggu window detector dari nol. Seed ini mencegah ZUPT mati 10 ms
     * setelah boot hanya karena hysteresis detector belum penuh. */
    StillnessDetector stillness = {0};
    if (startup.stationary) {
        stillness.still = 1U;
        stillness.enter_count = 20U;
    }
    float zero_rate_sigma = zero_rate_sigma_from_settings(&settings);

    board_imu_scheduler_reset();
    uint32_t sequence = 0U;
    uint32_t imu_error_streak = 0U;
    uint32_t last_fifo_resync = imu_mpu6xxx_get_fifo_stats()->fifo_resync_count;

    /* ZUPT otomatis hanya diizinkan dari keadaan boot diam sampai gerak pertama.
     * Sesudah AGV mulai bergerak, ZUPT hanya dilakukan atas perintah master. */
    uint8_t startup_zupt = startup.stationary ? 1U : 0U;
    uint8_t master_stationary = 0U;
    uint8_t zupt_divider = 0U;
    uint8_t zupt_phase = 0U;
    uint8_t gravity_divider = 1U;
    uint8_t gravity_ok = 0U;
    uint8_t zupt_applied = 0U;
    uint8_t telemetry_divider = 0U;
    uint8_t telemetry_valid = 0U;
    VescImuState telemetry;
    memset(&telemetry, 0, sizeof(telemetry));
    ExternalAidStatus aid_status;
    memset(&aid_status,0,sizeof(aid_status));

    for (;;) {
        /* TIM2 menentukan cadence service. Jika CPU pernah terlambat, pending>1
         * hanya menjadi diagnostik; FIFO tetap membawa semua sampel sensor. */
        uint32_t pending_ticks = board_wait_imu_tick();
        (void)pending_ticks;
        uint32_t now_us = board_micros();

        ImuSample fifo_samples[8];
        uint8_t fifo_count = 0U;
        if (!imu_mpu6xxx_read_fifo(fifo_samples, 8U, &fifo_count)) {
            service_bootloader_while_starting();
            if (++imu_error_streak >= IMU_REINIT_ERROR_COUNT) {
                board_i2c_recover();
                (void)imu_mpu6xxx_init();
                imu_preintegrator_init(&preintegrator);
                last_fifo_resync = imu_mpu6xxx_get_fifo_stats()->fifo_resync_count;
                imu_error_streak = 0U;
            }
            continue;
        }
        imu_error_streak = 0U;

        const ImuFifoStats *fifo_stats = imu_mpu6xxx_get_fifo_stats();
        if (fifo_stats->fifo_resync_count != last_fifo_resync) {
            /* Jangan menghubungkan dua sisi gap/overflow sebagai satu interval IMU. */
            imu_preintegrator_init(&preintegrator);
            last_fifo_resync = fifo_stats->fifo_resync_count;
        }
        if (fifo_count == 0U) {
            service_bootloader_while_starting();
            continue;
        }

        float accel[3] = {0.0f, 0.0f, 0.0f};
        float gyro[3] = {0.0f, 0.0f, 0.0f};
        float sensor_hz=fifo_stats->observed_sample_hz;
        if (!(sensor_hz>100.0f && sensor_hz<300.0f)) sensor_hz=(float)MPU6XXX_SAMPLE_HZ;
        const float sensor_dt=1.0f/sensor_hz;

        /* Sama seperti backend INS production: semua sampel FIFO dipakai untuk
         * membentuk delta-angle/delta-velocity. State ESKF tidak membaca raw
         * register secara polling satu-per-satu. */
        for (uint8_t n=0U;n<fifo_count;n++) {
            raw=fifo_samples[n];
            imu_apply_static_calibration(&raw,&settings,accel,gyro);
            imu_preintegrator_push(&preintegrator,gyro,accel,sensor_dt);
            imu_calibration_update(&calibration,&raw,&settings);
        }
        imu_apply_static_calibration(&raw,&settings,accel,gyro);

        ImuDelta delta;
        if (!imu_preintegrator_take(&preintegrator,&delta)) {
            service_bootloader_while_starting();
            continue;
        }
        eskf_nav_predict_delta(&eskf, delta.delta_angle, delta.delta_velocity, delta.dt);

        int sample_is_still = stillness_update(&stillness, accel, gyro);
        if(recover_filter_if_unhealthy(&eskf,&raw,&settings,&preintegrator,
                                      &last_fifo_resync,(uint8_t)sample_is_still,&startup_zupt)) {
            aid_status.reject_count++;
            continue;
        }
        /* Gravity correction 50 Hz. Saat bergerak gate dibuat jauh lebih ketat
         * dan measurement-noise diperbesar agar percepatan AGV tidak dianggap tilt. */
        if (++gravity_divider >= 2U) {
            gravity_divider = 0U;
            gravity_ok = (uint8_t)eskf_nav_correct_gravity(&eskf, accel, sample_is_still);
        }

        zupt_applied = 0U;
        if (startup_zupt && !sample_is_still) {
            /* Gerak pertama mematikan ZUPT otomatis agar gerak konstan tidak dianggap diam. */
            startup_zupt = 0U;
        }

        int allow_zupt = sample_is_still &&
                         (startup_zupt || master_stationary ||
                          calibration.state == IMU_CAL_STILL);
        if (allow_zupt) {
            /*
             * Satu measurement update per 5 sampel, diselang-seling:
             * velocity 10 Hz dan zero-rate 10 Hz. Information rate tetap sama,
             * tetapi dua update matriks tidak menumpuk pada satu siklus CPU.
             */
            if (++zupt_divider >= 5U) {
                zupt_divider = 0U;
                if (zupt_phase == 0U) {
                    (void)eskf_nav_fuse_zero_velocity(&eskf, ZUPT_SIGMA_MPS);
                    zupt_phase = 1U;
                } else {
                    (void)eskf_nav_fuse_zero_rate(&eskf, gyro, zero_rate_sigma);
                    zupt_phase = 0U;
                }
                zupt_applied = 1U;
            }
        } else {
            zupt_divider = 0U;
            zupt_phase = 0U;
        }

        if (calibration.event_saved_needed) {
            eeprom_valid = eeprom_settings_save(&settings);
            calibration.event_saved_needed = 0U;
            imu_apply_static_calibration(&raw, &settings, accel, gyro);
            init_filter(&eskf, &raw, &settings);
            zero_rate_sigma = zero_rate_sigma_from_settings(&settings);
            imu_preintegrator_init(&preintegrator);
            (void)imu_mpu6xxx_fifo_reset();
            last_fifo_resync = imu_mpu6xxx_get_fifo_stats()->fifo_resync_count;
            startup_zupt = 1U;
        }

        /* Konversi Euler (atan2/asin) dan linear-accel cukup dihitung pada
         * cadence telemetry 50 Hz. Quaternion/state ESKF tetap 100 Hz. */
        uint8_t telemetry_due = 0U;
        if (++telemetry_divider >= TELEMETRY_DIVIDER) {
            telemetry_divider = 0U;
            telemetry_due = 1U;
            fill_telemetry(&telemetry, &raw, accel, gyro, &eskf, &calibration,
                           &settings, sequence, now_us, gravity_ok, eeprom_valid,
                           startup.stationary, zupt_applied, master_stationary,&aid_status);
            telemetry_valid = 1U;
        }

        /* Field status kalibrasi murah untuk diperbarui setiap 10 ms, supaya
         * command master tidak menerima state satu frame lama. */
        if (telemetry_valid) sync_calibration_status(&telemetry, &calibration);
        VescAction action = vesc_process_rx(&huart2, telemetry_valid ? &telemetry : NULL);

        if (action == VESC_ACTION_BOOTLOADER) {
            HAL_Delay(5U);
            board_reboot_to_bootloader();
        } else if (action == VESC_ACTION_CAL_STILL) {
            imu_calibration_start_still(&calibration);
            sync_calibration_status(&telemetry, &calibration);
            (void)vesc_send_calibration_status(&huart2, CAL_CMD_STILL_START, 0U, &telemetry);
        } else if (action == VESC_ACTION_CAL_ROTATE_START) {
            imu_calibration_start_rotate(&calibration);
            sync_calibration_status(&telemetry, &calibration);
            (void)vesc_send_calibration_status(&huart2, CAL_CMD_ROTATE_START, 0U, &telemetry);
        } else if (action == VESC_ACTION_CAL_ROTATE_FINISH) {
            int ok = imu_calibration_finish_rotate(&calibration, &settings);
            if (ok && calibration.event_saved_needed) {
                eeprom_valid = eeprom_settings_save(&settings);
                calibration.event_saved_needed = 0U;
                init_filter(&eskf, &raw, &settings);
                zero_rate_sigma = zero_rate_sigma_from_settings(&settings);
                imu_preintegrator_init(&preintegrator);
                (void)imu_mpu6xxx_fifo_reset();
                last_fifo_resync = imu_mpu6xxx_get_fifo_stats()->fifo_resync_count;
                startup_zupt = 1U;
            }
            sync_calibration_status(&telemetry, &calibration);
            (void)vesc_send_calibration_status(&huart2, CAL_CMD_ROTATE_FINISH,
                                               ok ? 0U : 1U, &telemetry);
        } else if (action == VESC_ACTION_CAL_CANCEL) {
            imu_calibration_cancel(&calibration);
            sync_calibration_status(&telemetry, &calibration);
            (void)vesc_send_calibration_status(&huart2, CAL_CMD_CANCEL, 0U, &telemetry);
        } else if (action == VESC_ACTION_ZERO_NAV) {
            eskf_nav_reset_motion(&eskf);
            startup_zupt = 1U;
            (void)vesc_send_calibration_status(&huart2, CAL_CMD_ZERO_NAV, 0U, &telemetry);
        } else if (action == VESC_ACTION_ZUPT) {
            if (sample_is_still) {
                (void)eskf_nav_fuse_zero_velocity(&eskf, ZUPT_SIGMA_MPS);
                (void)eskf_nav_fuse_zero_rate(&eskf, gyro, zero_rate_sigma);
                startup_zupt = 1U;
                (void)vesc_send_calibration_status(&huart2, CAL_CMD_ZUPT_ONCE, 0U, &telemetry);
            } else {
                (void)vesc_send_calibration_status(&huart2, CAL_CMD_ZUPT_ONCE, 1U, &telemetry);
            }
        } else if (action == VESC_ACTION_STATIONARY_ON) {
            master_stationary = 1U;
            (void)vesc_send_calibration_status(&huart2, CAL_CMD_STATIONARY_ON, 0U, &telemetry);
        } else if (action == VESC_ACTION_STATIONARY_OFF) {
            /* Master mengetahui kendaraan akan bergerak; jangan menunggu IMU
             * membuktikan motion karena gerak konstan tidak observable oleh IMU. */
            master_stationary = 0U;
            startup_zupt = 0U;
            (void)vesc_send_calibration_status(&huart2, CAL_CMD_STATIONARY_OFF, 0U, &telemetry);
        } else if (action == VESC_ACTION_CONFIG) {
            VescConfigRequest req;
            if(vesc_take_config_request(&req)){
                int changed=0,reinit=0;
                if(req.subcmd==CFG_CMD_SET_MOUNT_RPY){
                    rpy_to_quat(req.value[0],req.value[1],req.value[2],settings.sensor_to_body_q);
                    settings.calibration_flags|=CAL_FLAG_MOUNT_VALID; changed=1; reinit=1;
                }else if(req.subcmd==CFG_CMD_RESET_MOUNT){
                    settings.sensor_to_body_q[0]=1.0f; settings.sensor_to_body_q[1]=0.0f;
                    settings.sensor_to_body_q[2]=0.0f; settings.sensor_to_body_q[3]=0.0f;
                    settings.calibration_flags&=~CAL_FLAG_MOUNT_VALID; changed=1; reinit=1;
                }else if(req.subcmd==CFG_CMD_SET_THERMAL){
                    int valid=1; for(int i=0;i<3;i++)if(fabsf(req.value[i])>0.05f||fabsf(req.value[i+3])>1.0f)valid=0;
                    if(valid){for(int i=0;i<3;i++){settings.gyro_temp_slope[i]=req.value[i];settings.accel_temp_slope[i]=req.value[i+3];}
                        settings.calibration_flags|=CAL_FLAG_THERMAL_VALID; changed=1; reinit=1;}
                    else {(void)vesc_send_config_status(&huart2,req.subcmd,2U,&settings);continue;}
                }else if(req.subcmd==CFG_CMD_CLEAR_THERMAL){
                    memset(settings.gyro_temp_slope,0,sizeof(settings.gyro_temp_slope));
                    memset(settings.accel_temp_slope,0,sizeof(settings.accel_temp_slope));
                    settings.calibration_flags&=~CAL_FLAG_THERMAL_VALID; changed=1; reinit=1;
                }else if(req.subcmd==CFG_CMD_SET_LEVER_ARM){
                    int valid=1;for(int i=0;i<3;i++)if(fabsf(req.value[i])>5.0f)valid=0;
                    if(valid){memcpy(settings.imu_position_body,req.value,sizeof(settings.imu_position_body));changed=1;}
                    else {(void)vesc_send_config_status(&huart2,req.subcmd,2U,&settings);continue;}
                }
                uint8_t status=0U;
                if(changed){eeprom_valid=eeprom_settings_save(&settings);if(!eeprom_valid)status=1U;}
                if(reinit && status==0U){
                    imu_apply_static_calibration(&raw,&settings,accel,gyro); init_filter(&eskf,&raw,&settings);
                    imu_preintegrator_init(&preintegrator); (void)imu_mpu6xxx_fifo_reset();
                    last_fifo_resync=imu_mpu6xxx_get_fifo_stats()->fifo_resync_count;
                    startup_zupt=sample_is_still?1U:0U;
                }
                (void)vesc_send_config_status(&huart2,req.subcmd,status,&settings);
            }
        } else if (action == VESC_ACTION_AIDING) {
            VescAidingRequest req;
            if(vesc_take_aiding_request(&req)){
                uint32_t age=aid_age_us(now_us,req.time_us);
                uint16_t age_ms=(uint16_t)((age/1000U)>65535U?65535U:(age/1000U));
                uint8_t status=0U; int ok=0;
                if(age>AID_MAX_AGE_US){status=4U;aid_status.reject_count++;}
                else{
                    float sigma=req.sigma*(1.0f+(float)age/(float)AID_MAX_AGE_US);
                    if(req.type==AID_CMD_WHEEL_BODY_X){
                        float omega[3],lever_cross[3],target[3]={req.value[0],0.0f,0.0f};
                        if(!isfinite(req.value[0])||fabsf(req.value[0])>AID_WHEEL_MAX_MPS ||
                           !isfinite(sigma)||sigma<AID_SIGMA_VEL_MIN_MPS||sigma>AID_SIGMA_VEL_MAX_MPS){
                            ok=0;
                        }else{
                            for(int i=0;i<3;i++)omega[i]=gyro[i]-eskf.gyro_bias[i];
                            lever_cross[0]=omega[1]*settings.imu_position_body[2]-omega[2]*settings.imu_position_body[1];
                            lever_cross[1]=omega[2]*settings.imu_position_body[0]-omega[0]*settings.imu_position_body[2];
                            lever_cross[2]=omega[0]*settings.imu_position_body[1]-omega[1]*settings.imu_position_body[0];
                            for(int i=0;i<3;i++)target[i]+=lever_cross[i];
                            if(!aid_recent(now_us,aid_status.last_wheel_us))
                                eskf_nav_inflate_velocity_uncertainty(&eskf,1.0f);
                            ok=eskf_nav_fuse_body_velocity(&eskf,target,0x01U,sigma);
                            if(ok){aid_status.last_wheel_us=now_us;aid_status.last_any_us=now_us;}
                            if(ok && (req.flags&AID_FLAG_NHC)){
                                float nhc_sigma=NHC_SIGMA_MPS*(1.0f+(float)age/(float)AID_MAX_AGE_US);
                                if(eskf_nav_fuse_body_velocity(&eskf,target,0x06U,nhc_sigma))aid_status.last_nhc_us=now_us;
                            }
                        }
                    }else if(req.type==AID_CMD_WORLD_VELOCITY){
                        if(!aid_recent(now_us,aid_status.last_vel_us))
                            ok=eskf_nav_reset_world_velocity(&eskf,req.value,sigma);
                        else ok=eskf_nav_fuse_world_velocity(&eskf,req.value,sigma);
                        if(ok){aid_status.last_vel_us=now_us;aid_status.last_any_us=now_us;}
                    }else if(req.type==AID_CMD_WORLD_POSITION){
                        if(!aid_recent(now_us,aid_status.last_pos_us))
                            ok=eskf_nav_reset_world_position(&eskf,req.value,sigma);
                        else ok=eskf_nav_fuse_world_position(&eskf,req.value,sigma);
                        if(ok){aid_status.last_pos_us=now_us;aid_status.last_any_us=now_us;}
                    }else if(req.type==AID_CMD_YAW){
                        if(!aid_recent(now_us,aid_status.last_yaw_us))
                            ok=eskf_nav_reset_yaw(&eskf,req.value[0],sigma);
                        else ok=eskf_nav_fuse_yaw(&eskf,req.value[0],sigma);
                        if(ok){aid_status.last_yaw_us=now_us;aid_status.last_any_us=now_us;}
                    }
                    if(!ok){status=1U;aid_status.reject_count++;}
                    if(!eskf_nav_is_healthy(&eskf)) {
                        (void)recover_filter_if_unhealthy(&eskf,&raw,&settings,&preintegrator,
                                                         &last_fifo_resync,(uint8_t)sample_is_still,&startup_zupt);
                        status=5U; aid_status.reject_count++;
                    }
                }
                (void)vesc_send_aiding_status(&huart2,req.type,status,age_ms);
            }
        }

        if (telemetry_due) {
            (void)vesc_send_extended_imu(&huart2, &telemetry);
            sequence++;
        }
    }
}
