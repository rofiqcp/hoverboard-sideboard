#include "imu_calibration.h"
#include "app_config.h"
#include <math.h>
#include <string.h>

static void reset_common(ImuCalibration *c)
{
    memset(c, 0, sizeof(*c));
}

void imu_calibration_init(ImuCalibration *c) { reset_common(c); }
void imu_calibration_start_still(ImuCalibration *c) { reset_common(c); c->state=IMU_CAL_STILL; }
void imu_calibration_start_rotate(ImuCalibration *c) { reset_common(c); c->state=IMU_CAL_ROTATE; }
void imu_calibration_cancel(ImuCalibration *c) { reset_common(c); c->state=IMU_CAL_IDLE; }

void imu_apply_static_calibration(const ImuSample *raw, const PersistedSettings *s,
                                  float accel[3], float gyro[3])
{
    for (int i=0; i<3; i++) {
        accel[i]=(raw->accel_mps2[i]-s->accel_offset[i])*s->accel_scale[i];
        gyro[i]=raw->gyro_rads[i]-s->gyro_bias[i];
    }
}

static int sample_still(const ImuSample *s)
{
    float gyro_norm=sqrtf(s->gyro_rads[0]*s->gyro_rads[0] +
                          s->gyro_rads[1]*s->gyro_rads[1] +
                          s->gyro_rads[2]*s->gyro_rads[2])*57.2957795f;
    float accel_norm=sqrtf(s->accel_mps2[0]*s->accel_mps2[0] +
                           s->accel_mps2[1]*s->accel_mps2[1] +
                           s->accel_mps2[2]*s->accel_mps2[2]);
    return gyro_norm < STARTUP_STILL_GYRO_DPS &&
           fabsf(accel_norm/GRAVITY_MPS2-1.0f) < STARTUP_STILL_ACCEL_ERR_G;
}

static void clear_still_accumulator(ImuCalibration *c)
{
    c->samples=0U;
    c->progress=0U;
    memset(c->sum_g,0,sizeof(c->sum_g));
    memset(c->sum_g2,0,sizeof(c->sum_g2));
    memset(c->sum_a,0,sizeof(c->sum_a));
    memset(c->sum_a2,0,sizeof(c->sum_a2));
    c->sum_temp=0.0f;
}

static int finish_still(ImuCalibration *c, PersistedSettings *set)
{
    float n=(float)c->samples;
    float accel_mean[3], gyro_mean[3], gyro_std[3], accel_std[3];
    float max_gyro_std_dps=0.0f, max_accel_std_g=0.0f;

    for (int i=0;i<3;i++) {
        gyro_mean[i]=c->sum_g[i]/n;
        accel_mean[i]=c->sum_a[i]/n;
        float gv=fmaxf(0.0f,c->sum_g2[i]/n-gyro_mean[i]*gyro_mean[i]);
        float av=fmaxf(0.0f,c->sum_a2[i]/n-accel_mean[i]*accel_mean[i]);
        gyro_std[i]=sqrtf(gv);
        accel_std[i]=sqrtf(av);
        float gdps=gyro_std[i]*57.2957795f;
        float ag=accel_std[i]/GRAVITY_MPS2;
        if (gdps>max_gyro_std_dps) max_gyro_std_dps=gdps;
        if (ag>max_accel_std_g) max_accel_std_g=ag;
    }

    float an=sqrtf(accel_mean[0]*accel_mean[0]+accel_mean[1]*accel_mean[1]+accel_mean[2]*accel_mean[2]);
    float norm_error_g=fabsf(an/GRAVITY_MPS2-1.0f);
    if (max_gyro_std_dps>STILL_CAL_GYRO_STD_MAX_DPS ||
        max_accel_std_g>STILL_CAL_ACCEL_STD_MAX_G ||
        norm_error_g>STILL_CAL_ACCEL_NORM_ERR_G) {
        c->state=IMU_CAL_FAILED; c->error_code=4U; return 0;
    }

    /* Commit atomik ke settings hanya sesudah seluruh quality gate lolos. */
    PersistedSettings candidate=*set;
    for (int i=0;i<3;i++) {
        candidate.gyro_bias[i]=gyro_mean[i];
        candidate.gyro_std[i]=gyro_std[i];
        candidate.accel_std[i]=accel_std[i];
    }
    candidate.calibration_temp_c=c->sum_temp/n;
    candidate.still_gyro_std_max_dps=max_gyro_std_dps;
    candidate.still_accel_std_max_g=max_accel_std_g;
    candidate.calibration_flags|=CAL_FLAG_STILL_VALID;
    candidate.still_cal_count++;
    *set=candidate;

    c->state=IMU_CAL_DONE; c->progress=1000U; c->event_saved_needed=1U;
    return 1;
}

static int select_face(const ImuSample *s)
{
    if (!sample_still(s)) return -1;
    int axis=0;
    if (fabsf(s->accel_mps2[1])>fabsf(s->accel_mps2[axis])) axis=1;
    if (fabsf(s->accel_mps2[2])>fabsf(s->accel_mps2[axis])) axis=2;
    if (fabsf(s->accel_mps2[axis]) < ROTATE_FACE_AXIS_MIN_G*GRAVITY_MPS2) return -1;
    for (int i=0;i<3;i++) {
        if (i!=axis && fabsf(s->accel_mps2[i]) > ROTATE_FACE_OTHER_MAX_G*GRAVITY_MPS2) return -1;
    }
    return axis*2 + (s->accel_mps2[axis] < 0.0f ? 1 : 0);
}

static void update_rotate(ImuCalibration *c,const ImuSample *s)
{
    int face=select_face(s);
    if (face<0) return;
    if (c->face_count[face] < 60000U) {
        for (int i=0;i<3;i++) c->face_sum[face][i]+=s->accel_mps2[i];
        c->face_count[face]++;
        c->samples++;
    }
    if (c->face_count[face]>=ROTATE_FACE_MIN_SAMPLES) c->coverage|=(uint8_t)(1U<<face);

    uint32_t accepted=0U;
    for (int f=0;f<6;f++) {
        accepted += c->face_count[f] < ROTATE_FACE_MIN_SAMPLES ? c->face_count[f] : ROTATE_FACE_MIN_SAMPLES;
    }
    c->progress=(uint16_t)((accepted*1000U)/(6U*ROTATE_FACE_MIN_SAMPLES));
}

void imu_calibration_update(ImuCalibration *c,const ImuSample *s,PersistedSettings *set)
{
    if (!c || !s || !set) return;
    c->total_seen++;

    if (c->state==IMU_CAL_STILL) {
        if (!sample_still(s)) {
            clear_still_accumulator(c);
            if (c->total_seen > MASTER_STILL_CAL_SAMPLES*10U) {
                c->state=IMU_CAL_FAILED;
                c->error_code=1U;
            }
            return;
        }
        for (int i=0;i<3;i++) {
            float g=s->gyro_rads[i], a=s->accel_mps2[i];
            c->sum_g[i]+=g; c->sum_g2[i]+=g*g;
            c->sum_a[i]+=a; c->sum_a2[i]+=a*a;
        }
        c->sum_temp+=s->temperature_c;
        c->samples++;
        c->progress=(uint16_t)((c->samples*1000U)/MASTER_STILL_CAL_SAMPLES);
        if (c->samples>=MASTER_STILL_CAL_SAMPLES) (void)finish_still(c,set);
    } else if (c->state==IMU_CAL_ROTATE) {
        update_rotate(c,s);
    }
}

int imu_calibration_finish_rotate(ImuCalibration *c,PersistedSettings *s)
{
    if (!c || !s || c->state!=IMU_CAL_ROTATE) return 0;
    if (c->coverage!=0x3FU) {
        c->state=IMU_CAL_FAILED; c->error_code=2U; return 0;
    }

    float mean[6][3];
    for (int f=0;f<6;f++) {
        if (c->face_count[f]<ROTATE_FACE_MIN_SAMPLES) {
            c->state=IMU_CAL_FAILED; c->error_code=2U; return 0;
        }
        float inv=1.0f/(float)c->face_count[f];
        for (int i=0;i<3;i++) mean[f][i]=c->face_sum[f][i]*inv;
    }

    PersistedSettings candidate=*s;
    for (int axis=0;axis<3;axis++) {
        float pos=mean[axis*2][axis];
        float neg=mean[axis*2+1][axis];
        float span=pos-neg;
        float offset=0.5f*(pos+neg);
        if (span<ROTATE_CAL_MIN_SPAN_G*GRAVITY_MPS2 ||
            fabsf(offset)>ROTATE_CAL_MAX_OFFSET_G*GRAVITY_MPS2) {
            c->state=IMU_CAL_FAILED; c->error_code=3U; return 0;
        }
        float scale=(2.0f*GRAVITY_MPS2)/span;
        if (scale<ROTATE_CAL_SCALE_MIN || scale>ROTATE_CAL_SCALE_MAX) {
            c->state=IMU_CAL_FAILED; c->error_code=3U; return 0;
        }
        candidate.accel_offset[axis]=offset;
        candidate.accel_scale[axis]=scale;
    }

    float sum_sq=0.0f,max_error=0.0f;
    for (int f=0;f<6;f++) {
        float corrected[3];
        for (int i=0;i<3;i++)
            corrected[i]=(mean[f][i]-candidate.accel_offset[i])*candidate.accel_scale[i];
        float norm=sqrtf(corrected[0]*corrected[0]+corrected[1]*corrected[1]+corrected[2]*corrected[2]);
        float err=fabsf(norm/GRAVITY_MPS2-1.0f);
        sum_sq+=err*err; if (err>max_error) max_error=err;
    }
    float rms=sqrtf(sum_sq/6.0f);
    if (rms>ROTATE_CAL_RMS_MAX_G || max_error>ROTATE_CAL_MAX_ERROR_G) {
        c->state=IMU_CAL_FAILED; c->error_code=5U; return 0;
    }

    candidate.rotate_residual_rms_g=rms;
    candidate.rotate_residual_max_g=max_error;
    candidate.calibration_flags|=CAL_FLAG_ROTATE_VALID;
    candidate.rotate_cal_count++;
    *s=candidate;

    c->state=IMU_CAL_DONE; c->progress=1000U; c->event_saved_needed=1U;
    return 1;
}
