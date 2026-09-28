#include <math.h>
#include <stdio.h>
#include <string.h>

#include "app_config.h"
#include "eskf_nav.h"
#include "imu_preintegrator.h"

#define PI_D 3.14159265358979323846

static void truth_qmul(const double a[4], const double b[4], double out[4])
{
    out[0]=a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3];
    out[1]=a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2];
    out[2]=a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1];
    out[3]=a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0];
}

static void truth_qstep(double q[4], const float w[3], double dt)
{
    double wx=(double)w[0],wy=(double)w[1],wz=(double)w[2];
    double wn=sqrt(wx*wx+wy*wy+wz*wz);
    double dq[4]={1.0,0.0,0.0,0.0}, out[4];
    if(wn>1e-14){
        double angle=wn*dt, scale=sin(0.5*angle)/wn;
        dq[0]=cos(0.5*angle);
        dq[1]=scale*wx; dq[2]=scale*wy; dq[3]=scale*wz;
    }
    truth_qmul(q,dq,out);
    memcpy(q,out,sizeof(out));
}

static void planar_sensor(double t, double yaw_rate, float gyro[3], float accel[3])
{
    double yaw=yaw_rate*t, c=cos(yaw), s=sin(yaw);
    double awx=1.5*sin(0.4*t), awy=cos(0.6*t);
    gyro[0]=0.0f; gyro[1]=0.0f; gyro[2]=(float)yaw_rate;
    accel[0]=(float)( c*awx+s*awy);
    accel[1]=(float)(-s*awx+c*awy);
    accel[2]=GRAVITY_MPS2;
}

static int run_planar_accuracy(void)
{
    const double sensor_dt=0.005;
    const int intervals=24000;
    const double yaw_rate=1.2;
    float a0[3]={0.0f,0.0f,GRAVITY_MPS2};
    EskfNav f; eskf_nav_init(&f,a0);
    f.gyro_noise=1e-6f; f.accel_noise=1e-6f;
    f.gyro_bias_walk=0.0f; f.accel_bias_walk=0.0f;

    ImuPreintegrator p; imu_preintegrator_init(&p);
    float gyro[3],accel[3]; planar_sensor(0.0,yaw_rate,gyro,accel);
    imu_preintegrator_push(&p,gyro,accel,(float)sensor_dt);
    double vx=0.0,vy=0.0,px=0.0,py=0.0;
    double prev_ax=0.0,prev_ay=1.0;
    for(int k=1;k<=intervals;k++){
        double t=(double)k*sensor_dt;
        planar_sensor(t,yaw_rate,gyro,accel);
        imu_preintegrator_push(&p,gyro,accel,(float)sensor_dt);

        double ax=1.5*sin(0.4*t), ay=cos(0.6*t);
        double nvx=vx+0.5*(prev_ax+ax)*sensor_dt;
        double nvy=vy+0.5*(prev_ay+ay)*sensor_dt;
        px+=0.5*(vx+nvx)*sensor_dt;
        py+=0.5*(vy+nvy)*sensor_dt;
        vx=nvx; vy=nvy; prev_ax=ax; prev_ay=ay;

        if((k&1)==0){
            ImuDelta d;
            if(!imu_preintegrator_take(&p,&d) ||
               !eskf_nav_predict_delta(&f,d.delta_angle,d.delta_velocity,d.dt))return 0;
        }
    }
    double velocity_error=hypot((double)f.velocity[0]-vx,(double)f.velocity[1]-vy);
    double position_error=hypot((double)f.position[0]-px,(double)f.position[1]-py);
    printf("planar preintegration: velocity_error=%.9g m/s position_error=%.9g m\n",
           velocity_error,position_error);
    return velocity_error<0.001 && position_error<0.02;
}

static void coning_rate(double t, float w[3])
{
    const double amplitude=1.5, frequency=3.0;
    w[0]=(float)(amplitude*cos(frequency*t));
    w[1]=(float)(amplitude*sin(frequency*t));
    w[2]=(float)(0.2*sin(0.7*t));
}

static int run_coning_accuracy(void)
{
    const double sensor_dt=0.005;
    const int intervals=12000, truth_substeps=20;
    float a0[3]={0.0f,0.0f,GRAVITY_MPS2};
    EskfNav f; eskf_nav_init(&f,a0);
    f.gyro_noise=1e-6f; f.accel_noise=1e-6f;
    f.gyro_bias_walk=0.0f; f.accel_bias_walk=0.0f;
    ImuPreintegrator p; imu_preintegrator_init(&p);

    double truth_q[4]={1.0,0.0,0.0,0.0};
    float gyro[3],zero_accel[3]={0.0f,0.0f,0.0f};
    coning_rate(0.0,gyro);
    imu_preintegrator_push(&p,gyro,zero_accel,(float)sensor_dt);

    for(int k=1;k<=intervals;k++){
        double t0=(double)(k-1)*sensor_dt;
        for(int j=0;j<truth_substeps;j++){
            double tm=t0+((double)j+0.5)*sensor_dt/(double)truth_substeps;
            float wmid[3]; coning_rate(tm,wmid);
            truth_qstep(truth_q,wmid,(double)sensor_dt/(double)truth_substeps);
        }
        coning_rate((double)k*sensor_dt,gyro);
        imu_preintegrator_push(&p,gyro,zero_accel,(float)sensor_dt);
        if((k&1)==0){
            ImuDelta d;
            if(!imu_preintegrator_take(&p,&d) ||
               !eskf_nav_predict_delta(&f,d.delta_angle,d.delta_velocity,d.dt))return 0;
        }
    }

    double truth_conj[4]={truth_q[0],-truth_q[1],-truth_q[2],-truth_q[3]};
    double estimate[4]={f.q[0],f.q[1],f.q[2],f.q[3]}, qerr[4];
    truth_qmul(truth_conj,estimate,qerr);
    double vec_norm=sqrt(qerr[1]*qerr[1]+qerr[2]*qerr[2]+qerr[3]*qerr[3]);
    double angle_deg=2.0*atan2(vec_norm,fabs(qerr[0]))*180.0/PI_D;
    printf("coning preintegration: attitude_error=%.9g deg\n",angle_deg);
    return angle_deg<0.10;
}

int main(void)
{
    int ok_planar=run_planar_accuracy();
    int ok_coning=run_coning_accuracy();
    printf("%s: 200->100 Hz planar preintegration accuracy\n",ok_planar?"PASS":"FAIL");
    printf("%s: coning compensation accuracy\n",ok_coning?"PASS":"FAIL");
    return (ok_planar&&ok_coning)?0:1;
}
