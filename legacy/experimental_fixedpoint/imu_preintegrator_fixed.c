#include "imu_preintegrator_fixed.h"
#include "app_config.h"
#include <string.h>

#define Q20_SCALE 1048576.0f
#define Q16_SCALE 65536.0f
#define Q24_SCALE 16777216.0f
#define Q28_SCALE 268435456.0f
#define GYRO_RAW_TO_RAD (0.01745329251994329577f / MPU6XXX_GYRO_LSB_PER_DPS)
#define ACCEL_RAW_TO_MPS2 (GRAVITY_MPS2 / MPU6XXX_ACCEL_LSB_PER_G)

static int32_t f_to_q(float x, float scale)
{
    return (int32_t)(x * scale + (x >= 0.0f ? 0.5f : -0.5f));
}

void imu_preintegrator_fixed_init(ImuPreintegratorFixed *p,
                                  const PersistedSettings *s)
{
    if (!p || !s) return;
    memset(p, 0, sizeof(*p));
    for (int i=0;i<3;i++) {
        p->gyro_bias_q20[i]=f_to_q(s->gyro_bias[i],Q20_SCALE);
        p->accel_offset_q16[i]=f_to_q(s->accel_offset[i]*s->accel_scale[i],Q16_SCALE);
        p->accel_gain_q24[i]=f_to_q(ACCEL_RAW_TO_MPS2*s->accel_scale[i],Q24_SCALE);
    }
}

static int32_t gyro_q20(int16_t raw, int32_t bias)
{
    const int32_t gain_q28=(int32_t)(GYRO_RAW_TO_RAD*Q28_SCALE+0.5f);
    return (int32_t)(((int64_t)raw*gain_q28)>>8)-bias;
}

static int32_t accel_q16(int16_t raw, int32_t gain_q24, int32_t offset)
{
    return (int32_t)(((int64_t)raw*gain_q24)>>8)-offset;
}

static int32_t mul_q20(int32_t a,int32_t b)
{
    return (int32_t)(((int64_t)a*(int64_t)b)>>20);
}

void imu_preintegrator_fixed_push(ImuPreintegratorFixed *p,const ImuSample *raw)
{
    if (!p || !raw) return;
    int32_t g[3],a[3];
    for(int i=0;i<3;i++) {
        g[i]=gyro_q20(raw->gyro_raw[i],p->gyro_bias_q20[i]);
        a[i]=accel_q16(raw->accel_raw[i],p->accel_gain_q24[i],p->accel_offset_q16[i]);
    }
    if(!p->have_previous) {
        memcpy(p->last_gyro_q20,g,sizeof(g)); memcpy(p->last_accel_q16,a,sizeof(a));
        p->have_previous=1U; return;
    }
    int32_t da[3],dv[3],base[3],cross[3];
    for(int i=0;i<3;i++) {
        /* Trapezoid, dt=1/200 s. Pembagian konstanta dioptimalkan compiler. */
        da[i]=(p->last_gyro_q20[i]+g[i])/400;
        dv[i]=(p->last_accel_q16[i]+a[i])/400;
        base[i]=p->delta_angle_q20[i]+p->last_delta_angle_q20[i]/6;
    }
    cross[0]=mul_q20(base[1],da[2])-mul_q20(base[2],da[1]);
    cross[1]=mul_q20(base[2],da[0])-mul_q20(base[0],da[2]);
    cross[2]=mul_q20(base[0],da[1])-mul_q20(base[1],da[0]);
    for(int i=0;i<3;i++) {
        p->delta_angle_q20[i]+=da[i]+cross[i]/2;
        p->delta_velocity_q16[i]+=dv[i];
        p->last_delta_angle_q20[i]=da[i];
    }
    p->sample_count++;
    memcpy(p->last_gyro_q20,g,sizeof(g)); memcpy(p->last_accel_q16,a,sizeof(a));
}

int imu_preintegrator_fixed_take(ImuPreintegratorFixed *p,ImuDelta *out)
{
    if(!p || !out || p->sample_count==0U) return 0;
    for(int i=0;i<3;i++) {
        out->delta_angle[i]=(float)p->delta_angle_q20[i]/Q20_SCALE;
        out->delta_velocity[i]=(float)p->delta_velocity_q16[i]/Q16_SCALE;
        p->delta_angle_q20[i]=0; p->delta_velocity_q16[i]=0;
    }
    out->sample_count=p->sample_count;
    out->dt=(float)p->sample_count/(float)MPU6XXX_SAMPLE_HZ;
    p->sample_count=0U;
    return 1;
}
