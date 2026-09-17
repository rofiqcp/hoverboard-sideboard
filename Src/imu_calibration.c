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

static void rotate_sensor_to_body(const float q[4], const float in[3], float out[3])
{
    /* v' = v + 2*qw*(qv x v) + 2*qv x (qv x v), q sensor->body. */
    float cx=q[2]*in[2]-q[3]*in[1];
    float cy=q[3]*in[0]-q[1]*in[2];
    float cz=q[1]*in[1]-q[2]*in[0];
    float c2x=q[2]*cz-q[3]*cy;
    float c2y=q[3]*cx-q[1]*cz;
    float c2z=q[1]*cy-q[2]*cx;
    out[0]=in[0]+2.0f*(q[0]*cx+c2x);
    out[1]=in[1]+2.0f*(q[0]*cy+c2y);
    out[2]=in[2]+2.0f*(q[0]*cz+c2z);
}

void imu_apply_static_calibration(const ImuSample *raw, const PersistedSettings *s,
                                  float accel[3], float gyro[3])
{
    float dtemp=raw->temperature_c-s->calibration_temp_c;
    if (dtemp>60.0f) dtemp=60.0f;
    if (dtemp<-60.0f) dtemp=-60.0f;

    float a_sensor_raw[3],a_sensor_cal[3],g_sensor[3];
    for(int i=0;i<3;i++) {
        a_sensor_raw[i]=raw->accel_mps2[i]-s->accel_offset[i]-s->accel_temp_slope[i]*dtemp;
        g_sensor[i]=raw->gyro_rads[i]-s->gyro_bias[i]-s->gyro_temp_slope[i]*dtemp;
    }
    for(int r=0;r<3;r++) {
        a_sensor_cal[r]=0.0f;
        for(int c=0;c<3;c++) a_sensor_cal[r]+=s->accel_transform[r*3+c]*a_sensor_raw[c];
    }
    rotate_sensor_to_body(s->sensor_to_body_q,a_sensor_cal,accel);
    rotate_sensor_to_body(s->sensor_to_body_q,g_sensor,gyro);
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
    float new_ref_temp=c->sum_temp/n;
    float ref_shift=new_ref_temp-candidate.calibration_temp_c;
    for (int i=0;i<3;i++) {
        /* Rebase offset accel agar model thermal tetap kontinu saat reference
         * temperature dipindah oleh kalibrasi diam baru. */
        candidate.accel_offset[i]+=candidate.accel_temp_slope[i]*ref_shift;
    }
    candidate.calibration_temp_c=new_ref_temp;
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

static int inverse3(const float A[3][3], float inv[3][3])
{
    float det=A[0][0]*(A[1][1]*A[2][2]-A[1][2]*A[2][1])
             -A[0][1]*(A[1][0]*A[2][2]-A[1][2]*A[2][0])
             +A[0][2]*(A[1][0]*A[2][1]-A[1][1]*A[2][0]);
    if (fabsf(det)<0.10f) return 0;
    float id=1.0f/det;
    inv[0][0]=(A[1][1]*A[2][2]-A[1][2]*A[2][1])*id;
    inv[0][1]=(A[0][2]*A[2][1]-A[0][1]*A[2][2])*id;
    inv[0][2]=(A[0][1]*A[1][2]-A[0][2]*A[1][1])*id;
    inv[1][0]=(A[1][2]*A[2][0]-A[1][0]*A[2][2])*id;
    inv[1][1]=(A[0][0]*A[2][2]-A[0][2]*A[2][0])*id;
    inv[1][2]=(A[0][2]*A[1][0]-A[0][0]*A[1][2])*id;
    inv[2][0]=(A[1][0]*A[2][1]-A[1][1]*A[2][0])*id;
    inv[2][1]=(A[0][1]*A[2][0]-A[0][0]*A[2][1])*id;
    inv[2][2]=(A[0][0]*A[1][1]-A[0][1]*A[1][0])*id;
    return 1;
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
        float invn=1.0f/(float)c->face_count[f];
        for (int i=0;i<3;i++) mean[f][i]=c->face_sum[f][i]*invn;
    }

    PersistedSettings candidate=*s;
    float offset[3]={0.0f,0.0f,0.0f};
    for (int axis=0;axis<3;axis++) {
        for (int i=0;i<3;i++)
            offset[i]+=0.5f*(mean[axis*2][i]+mean[axis*2+1][i])/3.0f;
    }
    for (int i=0;i<3;i++) {
        if (fabsf(offset[i])>ROTATE_CAL_MAX_OFFSET_G*GRAVITY_MPS2) {
            c->state=IMU_CAL_FAILED; c->error_code=3U; return 0;
        }
        candidate.accel_offset[i]=offset[i];
    }

    /* measured = offset + A * true. Tiap kolom A didapat langsung dari
     * pasangan centroid +axis/-axis. Runtime memakai T=A^-1. */
    float A[3][3],T[3][3];
    for (int axis=0;axis<3;axis++) {
        for (int row=0;row<3;row++)
            A[row][axis]=(mean[axis*2][row]-mean[axis*2+1][row])/(2.0f*GRAVITY_MPS2);
        float cn=sqrtf(A[0][axis]*A[0][axis]+A[1][axis]*A[1][axis]+A[2][axis]*A[2][axis]);
        if (cn<ROTATE_CAL_SCALE_MIN || cn>ROTATE_CAL_SCALE_MAX) {
            c->state=IMU_CAL_FAILED; c->error_code=3U; return 0;
        }
    }
    if (!inverse3(A,T)) { c->state=IMU_CAL_FAILED; c->error_code=6U; return 0; }
    for (int r=0;r<3;r++) for (int col=0;col<3;col++)
        candidate.accel_transform[r*3+col]=T[r][col];

    float sum_sq=0.0f,max_error=0.0f;
    for (int f=0;f<6;f++) {
        float corrected[3]={0.0f,0.0f,0.0f};
        for (int r=0;r<3;r++) for (int col=0;col<3;col++)
            corrected[r]+=T[r][col]*(mean[f][col]-offset[col]);
        int axis=f/2; float sign=(f&1)?-1.0f:1.0f;
        float e2=0.0f;
        for (int i=0;i<3;i++) {
            float target=(i==axis)?sign*GRAVITY_MPS2:0.0f;
            float e=(corrected[i]-target)/GRAVITY_MPS2;
            e2+=e*e;
        }
        float err=sqrtf(e2);
        sum_sq+=err*err; if(err>max_error)max_error=err;
    }
    float rms=sqrtf(sum_sq/6.0f);
    if(rms>ROTATE_CAL_RMS_MAX_G || max_error>ROTATE_CAL_MAX_ERROR_G) {
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
