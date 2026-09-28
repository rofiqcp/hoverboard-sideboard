#include <math.h>
#include <stdio.h>
#include <string.h>

#include "app_config.h"
#include "eskf_nav.h"
#include "imu_preintegrator.h"

static int failures = 0;

static void check(int ok, const char *msg)
{
    printf("%s: %s\n", ok ? "PASS" : "FAIL", msg);
    if (!ok) failures++;
}

static float norm3(const float v[3])
{
    return sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

static void rotz(float yaw, const float in[3], float out[3])
{
    float c = cosf(yaw), s = sinf(yaw);
    out[0] = c*in[0] - s*in[1];
    out[1] = s*in[0] + c*in[1];
    out[2] = in[2];
}

static void world_to_body_z(float yaw, const float world[3], float body[3])
{
    float c = cosf(yaw), s = sinf(yaw);
    body[0] = c*world[0] + s*world[1];
    body[1] = -s*world[0] + c*world[1];
    body[2] = world[2];
}

static void qmul(const float a[4], const float b[4], float out[4])
{
    float q[4];
    q[0]=a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3];
    q[1]=a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2];
    q[2]=a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1];
    q[3]=a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0];
    memcpy(out,q,sizeof(q));
}

static void q_from_rotvec(const float r[3], float q[4])
{
    float a=sqrtf(r[0]*r[0]+r[1]*r[1]+r[2]*r[2]);
    if(a<1e-8f){q[0]=1.0f;q[1]=0.5f*r[0];q[2]=0.5f*r[1];q[3]=0.5f*r[2];return;}
    float s=sinf(0.5f*a)/a;
    q[0]=cosf(0.5f*a);q[1]=s*r[0];q[2]=s*r[1];q[3]=s*r[2];
}

static float q_abs_dot(const float a[4], const float b[4])
{
    float d=a[0]*b[0]+a[1]*b[1]+a[2]*b[2]+a[3]*b[3];
    return fabsf(d);
}

static void test_no_rotation(void)
{
    ImuPreintegrator p;
    imu_preintegrator_init(&p);
    const float g[3]={0.0f,0.0f,0.0f};
    const float a[3]={1.25f,-0.50f,9.20f};
    const float dt=0.005f;
    imu_preintegrator_push(&p,g,a,dt);
    imu_preintegrator_push(&p,g,a,dt);
    imu_preintegrator_push(&p,g,a,dt);

    ImuDelta d;
    check(imu_preintegrator_take(&p,&d),"constant no-rotation preintegration produces output");
    check(fabsf(d.dt-0.010f)<1e-7f && d.sample_count==2U,
          "preintegration duration matches two physical intervals");
    check(norm3(d.delta_angle)<1e-7f,"zero gyro produces zero delta angle");
    float err[3]={d.delta_velocity[0]-a[0]*d.dt,
                  d.delta_velocity[1]-a[1]*d.dt,
                  d.delta_velocity[2]-a[2]*d.dt};
    check(norm3(err)<1e-6f,"constant acceleration integrates exactly without rotation");
}

static void test_rotating_world_force(void)
{
    ImuPreintegrator p;
    imu_preintegrator_init(&p);
    const float dt=0.005f;
    const float omega=8.0f;
    const float world_force[3]={3.0f,-1.5f,4.0f};
    float gyro[3]={0.0f,0.0f,omega};
    float body[3];

    for(int k=0;k<=2;k++){
        world_to_body_z(omega*dt*(float)k,world_force,body);
        imu_preintegrator_push(&p,gyro,body,dt);
    }

    ImuDelta d;
    check(imu_preintegrator_take(&p,&d),"rotating-frame preintegration produces output");
    check(fabsf(d.delta_angle[2]-omega*d.dt)<2e-6f &&
          fabsf(d.delta_angle[0])<1e-7f && fabsf(d.delta_angle[1])<1e-7f,
          "constant yaw rate integrates to the expected rotation");

    float reconstructed_start[3];
    rotz(0.5f*d.delta_angle[2],d.delta_velocity,reconstructed_start);
    float truth[3]={world_force[0]*d.dt,world_force[1]*d.dt,world_force[2]*d.dt};
    float err[3]={reconstructed_start[0]-truth[0],
                  reconstructed_start[1]-truth[1],
                  reconstructed_start[2]-truth[2]};
    check(norm3(err)<2e-6f,
          "sculling-aware delta velocity reconstructs constant world force during rotation");

    float b0[3],b1[3],b2[3],legacy_body[3],legacy_start[3];
    world_to_body_z(0.0f,world_force,b0);
    world_to_body_z(omega*dt,world_force,b1);
    world_to_body_z(2.0f*omega*dt,world_force,b2);
    for(int i=0;i<3;i++)
        legacy_body[i]=0.5f*(b0[i]+2.0f*b1[i]+b2[i])*dt;
    rotz(omega*dt,legacy_body,legacy_start);
    float legacy_err[3]={legacy_start[0]-truth[0],
                         legacy_start[1]-truth[1],
                         legacy_start[2]-truth[2]};
    check(norm3(err)<0.20f*norm3(legacy_err),
          "frame-consistent accumulation materially reduces legacy sculling error");
}

static void test_noncommuting_rotation(void)
{
    ImuPreintegrator p;
    imu_preintegrator_init(&p);
    const float dt=0.005f;
    const float a[3]={0.0f,0.0f,9.80665f};
    const float gyro[][3]={{2.0f,0.0f,0.0f},
                           {2.0f,3.0f,0.0f},
                           {0.0f,3.0f,4.0f},
                           {0.0f,0.0f,4.0f}};
    float qref[4]={1.0f,0.0f,0.0f,0.0f};

    imu_preintegrator_push(&p,gyro[0],a,dt);
    for(int k=1;k<4;k++){
        float da[3],dq[4],qnext[4];
        for(int i=0;i<3;i++)da[i]=0.5f*(gyro[k-1][i]+gyro[k][i])*dt;
        q_from_rotvec(da,dq);
        qmul(qref,dq,qnext);
        memcpy(qref,qnext,sizeof(qref));
        imu_preintegrator_push(&p,gyro[k],a,dt);
    }

    ImuDelta d;
    check(imu_preintegrator_take(&p,&d),"noncommuting rotation preintegration produces output");
    float qout[4];
    q_from_rotvec(d.delta_angle,qout);
    check(q_abs_dot(qout,qref)>0.999999f,
          "quaternion accumulation preserves noncommuting coning rotation");
}

static void test_window_continuity(void)
{
    ImuPreintegrator p;
    imu_preintegrator_init(&p);
    const float dt=0.005f;
    const float gyro[3]={0.0f,0.0f,1.0f};
    const float accel[3]={1.0f,0.0f,9.0f};

    imu_preintegrator_push(&p,gyro,accel,dt);
    imu_preintegrator_push(&p,gyro,accel,dt);
    ImuDelta first;
    check(imu_preintegrator_take(&p,&first),"first preintegration window available");

    imu_preintegrator_push(&p,gyro,accel,dt);
    ImuDelta second;
    check(imu_preintegrator_take(&p,&second),
          "take preserves previous physical sample for the next interval");
    check(fabsf(first.dt-dt)<1e-7f && fabsf(second.dt-dt)<1e-7f,
          "adjacent windows account for each sample interval exactly once");

    imu_preintegrator_push(&p,gyro,accel,dt);
    imu_preintegrator_reset(&p);
    imu_preintegrator_push(&p,gyro,accel,dt);
    ImuDelta after_reset;
    check(imu_preintegrator_take(&p,&after_reset) && fabsf(after_reset.dt-dt)<1e-7f,
          "soft reset clears accumulated motion but preserves sample continuity");
}


static void test_preintegrator_to_eskf_rotating_gravity(void)
{
    ImuPreintegrator p;
    imu_preintegrator_init(&p);
    const float dt=0.005f;
    const float omega=8.0f;
    const float world_gravity[3]={0.0f,0.0f,GRAVITY_MPS2};
    const float gyro[3]={0.0f,omega,0.0f};

    for(int k=0;k<=2;k++){
        float theta=omega*dt*(float)k;
        float body_gravity[3]={-sinf(theta)*GRAVITY_MPS2,
                               0.0f,
                               cosf(theta)*GRAVITY_MPS2};
        imu_preintegrator_push(&p,gyro,body_gravity,dt);
    }

    ImuDelta d;
    check(imu_preintegrator_take(&p,&d),"rotating-gravity IMU window available");

    EskfNav f;
    eskf_nav_init(&f,world_gravity);
    check(eskf_nav_predict_delta(&f,d.delta_angle,d.delta_velocity,d.dt),
          "ESKF accepts sculling-compensated rotating-gravity delta");
    check(norm3(f.velocity)<2.0e-5f,
          "pure rotation in gravity does not create false translational velocity");
    check(eskf_nav_covariance_psd_check(&f),
          "sculling-compensated ESKF propagation preserves PSD covariance");
}

static void test_invalid_input_does_not_poison_state(void)
{
    ImuPreintegrator p;
    imu_preintegrator_init(&p);
    const float dt=0.005f;
    const float gyro[3]={0.0f,0.0f,0.0f};
    const float accel[3]={0.0f,0.0f,9.80665f};
    float bad_gyro[3]={NAN,0.0f,0.0f};

    imu_preintegrator_push(&p,gyro,accel,dt);
    imu_preintegrator_push(&p,bad_gyro,accel,dt);
    imu_preintegrator_push(&p,gyro,accel,dt);

    ImuDelta d;
    check(imu_preintegrator_take(&p,&d),"valid sample after invalid input remains usable");
    check(fabsf(d.dt-dt)<1e-7f && d.sample_count==1U,
          "invalid input is ignored without advancing integration time");
    check(isfinite(d.delta_velocity[0]) && isfinite(d.delta_velocity[1]) &&
          isfinite(d.delta_velocity[2]),"invalid input cannot inject NaN into preintegrator");
}

int main(void)
{
    test_no_rotation();
    test_rotating_world_force();
    test_noncommuting_rotation();
    test_window_continuity();
    test_preintegrator_to_eskf_rotating_gravity();
    test_invalid_input_does_not_poison_state();
    return failures ? 1 : 0;
}
