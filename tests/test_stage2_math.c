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

    /* FIFO can legally deliver up to ~40 ms of preintegrated data. The estimator
     * must use the same dt as the delta increments, never clamp dt independently. */
    eskf_nav_init(&f,a0);
    {
        float da40[3]={0,0,0}, dv40[3]={0,0,g*0.04f};
        ck(eskf_nav_predict_delta(&f,da40,dv40,0.04f),"40 ms preintegrated delta accepted");
        ck(fabsf(f.velocity[2])<1e-6f,"40 ms stationary delta has no false vertical velocity");
    }

    /* Moderate unaided motion previously drove P indefinite after the diagonal
     * upper clamp saturated. FPF^T propagation must remain numerically healthy. */
    eskf_nav_init(&f,a0);
    for(int k=1;k<=12000;k++){
        float t=k*0.01f;
        float w[3]={0.05f*sinf(0.2f*t),0.04f*cosf(0.17f*t),0.35f*sinf(0.11f*t)};
        float ac[3]={0.8f*sinf(0.13f*t),0.4f*cosf(0.19f*t),g};
        float da[3],dv[3];
        for(int i=0;i<3;i++){da[i]=w[i]*0.01f;dv[i]=ac[i]*0.01f;}
        if(!eskf_nav_predict_delta(&f,da,dv,0.01f)){printf("FAIL: moderate-motion delta rejected\n");fail++;break;}
    }
    int cov_ok=1;
    for(int r=0;r<ESKF_NAV_DIM;r++){
        if(!isfinite(f.P[r][r]) || f.P[r][r]<=0.0f) cov_ok=0;
        for(int c=r+1;c<ESKF_NAV_DIM;c++){
            float x=f.P[r][c];
            float bound=f.P[r][r]*f.P[c][c]*1.02f+1e-12f;
            if(!isfinite(x) || x*x>bound) cov_ok=0;
        }
    }
    ck(cov_ok,"covariance invariants hold after 120s moderate unaided motion");
    {float q_before[4],v_before[3],p_before[3];memcpy(q_before,f.q,sizeof(q_before));memcpy(v_before,f.velocity,sizeof(v_before));memcpy(p_before,f.position,sizeof(p_before));
     eskf_nav_reset_covariance(&f);
     ck(memcmp(q_before,f.q,sizeof(q_before))==0 && memcmp(v_before,f.velocity,sizeof(v_before))==0 && memcmp(p_before,f.position,sizeof(p_before))==0,
        "covariance recovery preserves nominal attitude/velocity/position");}

    /* Sustained 0.1 g horizontal acceleration has magnitude ~1.005 g and must
     * not be mistaken for a 5.7 degree gravity tilt while moving. */
    eskf_nav_init(&f,a0);
    for(int k=0;k<500;k++){
        float da[3]={0,0,0},dv[3]={0.1f*g*0.01f,0,g*0.01f};
        (void)eskf_nav_predict_delta(&f,da,dv,0.01f);
        if((k&1)==0){float am[3]={0.1f*g,0,g};(void)eskf_nav_correct_gravity(&f,am,0);}
    }
    {float rr=0,pp=0,yy=0;eskf_nav_get_euler_deg(&f,&rr,&pp,&yy);
     ck(fabsf(pp)<1.0f,"moving 0.1g acceleration is not fused as false pitch");
     ck(f.velocity[0]>4.5f,"moving 0.1g acceleration remains real forward velocity");}

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
