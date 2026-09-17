#include <stdio.h>
#include <string.h>
#include <math.h>
#include "app_config.h"
#include "eskf_nav.h"
#include "imu_calibration.h"

static int fail=0;
static void ck(int cond,const char *msg){printf("%s: %s\n",cond?"PASS":"FAIL",msg);if(!cond)fail++;}

int main(void){
    const float g=GRAVITY_MPS2;
    EskfNav f; float a0[3]={0,0,g}; eskf_nav_init(&f,a0);
    for(int k=0;k<1000;k++){
        float da[3]={0,0,0}, dv[3]={0,0,g*0.01f};
        eskf_nav_predict_delta(&f,da,dv,0.01f);
        if((k%2)==0) (void)eskf_nav_correct_gravity(&f,a0,1);
        if((k%10)==0) (void)eskf_nav_fuse_zero_velocity(&f,0.03f);
    }
    float vn=sqrtf(f.velocity[0]*f.velocity[0]+f.velocity[1]*f.velocity[1]+f.velocity[2]*f.velocity[2]);
    float pn=sqrtf(f.position[0]*f.position[0]+f.position[1]*f.position[1]+f.position[2]*f.position[2]);
    ck(eskf_nav_is_healthy(&f),"ESKF healthy after 10s stationary");
    ck(vn<0.01f,"stationary velocity bounded <1 cm/s");
    ck(pn<0.05f,"stationary position bounded <5 cm");

    ck(eskf_nav_reset_yaw(&f,170.0f*(float)M_PI/180.0f,2.0f*(float)M_PI/180.0f),"first absolute yaw reset 170 deg");
    float yaw=0; eskf_nav_get_euler_rad(&f,0,0,&yaw);
    ck(fabsf(yaw-170.0f*(float)M_PI/180.0f)<0.01f,"yaw reset exact");
    ck(!eskf_nav_fuse_yaw(&f,0.0f,2.0f*(float)M_PI/180.0f),"large in-lock yaw innovation rejected");
    ck(eskf_nav_reset_yaw(&f,0.0f,2.0f*(float)M_PI/180.0f),"yaw reacquisition reset after source timeout");

    float vv[3]={1.0f,0.0f,0.0f};
    ck(eskf_nav_reset_world_velocity(&f,vv,0.1f),"world velocity source reset accepted");
    ck(fabsf(f.velocity[0]-1.0f)<1e-6f,"velocity reset exact");
    float badv[3]={100.0f,0,0};
    ck(!eskf_nav_reset_world_velocity(&f,badv,0.1f),"absurd velocity rejected");
    float pp[3]={12.3f,-4.5f,0.2f};
    ck(eskf_nav_reset_world_position(&f,pp,0.25f),"world position source reset accepted");
    float badp[3]={200000.0f,0,0};
    ck(!eskf_nav_reset_world_position(&f,badp,0.25f),"absurd position rejected");

    PersistedSettings s; memset(&s,0,sizeof(s));
    s.accel_transform[0]=s.accel_transform[4]=s.accel_transform[8]=1.0f; s.sensor_to_body_q[0]=1.0f;
    ImuCalibration c; imu_calibration_start_rotate(&c);
    const float A[3][3]={{1.03f,0.015f,-0.010f},{0.012f,0.98f,0.008f},{-0.006f,0.011f,1.02f}};
    const float off[3]={0.12f,-0.08f,0.05f};
    for(int axis=0;axis<3;axis++) for(int signi=0;signi<2;signi++){
        int face=axis*2+signi; float sign=signi?-1.0f:1.0f;
        float truth[3]={0,0,0};truth[axis]=sign*g;
        float meas[3]={off[0],off[1],off[2]};
        for(int r=0;r<3;r++)for(int j=0;j<3;j++)meas[r]+=A[r][j]*truth[j];
        c.face_count[face]=ROTATE_FACE_MIN_SAMPLES; c.coverage|=(1u<<face);
        for(int r=0;r<3;r++)c.face_sum[face][r]=meas[r]*(float)c.face_count[face];
    }
    ck(imu_calibration_finish_rotate(&c,&s),"synthetic full 3x3 six-face calibration succeeds");
    ck(c.state==IMU_CAL_DONE && c.event_saved_needed,"calibration transactional DONE/save-needed");
    float maxerr=0;
    for(int axis=0;axis<3;axis++)for(int signi=0;signi<2;signi++){
        float truth[3]={0,0,0};truth[axis]=(signi?-1.0f:1.0f)*g;
        ImuSample raw={0};
        for(int r=0;r<3;r++){raw.accel_mps2[r]=off[r];for(int j=0;j<3;j++)raw.accel_mps2[r]+=A[r][j]*truth[j];}
        raw.temperature_c=s.calibration_temp_c;
        float ac[3],gy[3]; imu_apply_static_calibration(&raw,&s,ac,gy);
        for(int i=0;i<3;i++){float e=fabsf(ac[i]-truth[i]);if(e>maxerr)maxerr=e;}
    }
    printf("3x3 max reconstruction error = %.9f m/s2\n",maxerr);
    ck(maxerr<1e-4f,"full 3x3 calibration reconstructs six axes");

    ImuCalibration bad; imu_calibration_start_rotate(&bad); PersistedSettings before=s;
    ck(!imu_calibration_finish_rotate(&bad,&s),"incomplete rotate calibration rejected");
    ck(memcmp(&before,&s,sizeof(s))==0,"failed calibration leaves settings byte-identical");
    return fail?1:0;
}
