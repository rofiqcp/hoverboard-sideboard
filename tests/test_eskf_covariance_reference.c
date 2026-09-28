#include <math.h>
#include <stdio.h>
#include <string.h>

#include "app_config.h"
#include "eskf_nav.h"

static void skew3(const float v[3], float s[3][3])
{
    s[0][0]=0.0f; s[0][1]=-v[2]; s[0][2]=v[1];
    s[1][0]=v[2]; s[1][1]=0.0f; s[1][2]=-v[0];
    s[2][0]=-v[1]; s[2][1]=v[0]; s[2][2]=0.0f;
}

static void qmul(const float a[4], const float b[4], float out[4])
{
    out[0]=a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3];
    out[1]=a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2];
    out[2]=a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1];
    out[3]=a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0];
}

static void qright(float q[4], const float dtheta[3])
{
    float a=sqrtf(dtheta[0]*dtheta[0]+dtheta[1]*dtheta[1]+dtheta[2]*dtheta[2]);
    float dq[4]={1.0f,0.0f,0.0f,0.0f}, out[4];
    if(a>1e-9f){
        float scale=sinf(0.5f*a)/a;
        dq[0]=cosf(0.5f*a);
        for(int i=0;i<3;i++)dq[1+i]=scale*dtheta[i];
    }
    qmul(q,dq,out);
    memcpy(q,out,sizeof(out));
}

static void make_test_covariance(float P[ESKF_NAV_DIM][ESKF_NAV_DIM])
{
    float L[ESKF_NAV_DIM][ESKF_NAV_DIM]={{0.0f}};
    memset(P,0,sizeof(float)*ESKF_NAV_DIM*ESKF_NAV_DIM);
    for(int i=0;i<ESKF_NAV_DIM;i++){
        L[i][i]=0.02f+0.003f*(float)i;
        for(int j=0;j<i;j++)L[i][j]=0.001f*(float)(((i+2*j)%5)-2);
    }
    for(int r=0;r<ESKF_NAV_DIM;r++)
        for(int c=0;c<ESKF_NAV_DIM;c++)
            for(int k=0;k<ESKF_NAV_DIM;k++)
                P[r][c]+=L[r][k]*L[c][k];
}

static void build_error_dynamics(const float w[3], const float fb[3],
                                 const float R[3][3],
                                 float A[ESKF_NAV_DIM][ESKF_NAV_DIM])
{
    float sw[3][3], sf[3][3];
    memset(A,0,sizeof(float)*ESKF_NAV_DIM*ESKF_NAV_DIM);
    skew3(w,sw); skew3(fb,sf);
    for(int r=0;r<3;r++){
        for(int c=0;c<3;c++){
            A[r][c]=-sw[r][c];
            for(int k=0;k<3;k++)A[3+r][c]-=R[r][k]*sf[k][c];
            A[3+r][12+c]=-R[r][c];
        }
        A[r][9+r]=-1.0f;
        A[6+r][3+r]=1.0f;
    }
}

static void reference_predict(const EskfNav *f, const float P0[ESKF_NAV_DIM][ESKF_NAV_DIM],
                              const float w[3], const float fb[3], const float R[3][3],
                              float dt, float out[ESKF_NAV_DIM][ESKF_NAV_DIM])
{
    float A[ESKF_NAV_DIM][ESKF_NAV_DIM], F[ESKF_NAV_DIM][ESKF_NAV_DIM]={{0.0f}};
    float FP[ESKF_NAV_DIM][ESKF_NAV_DIM]={{0.0f}};
    build_error_dynamics(w,fb,R,A);
    for(int r=0;r<ESKF_NAV_DIM;r++)
        for(int c=0;c<ESKF_NAV_DIM;c++)
            F[r][c]=(r==c?1.0f:0.0f)+A[r][c]*dt;
    memset(out,0,sizeof(float)*ESKF_NAV_DIM*ESKF_NAV_DIM);
    for(int r=0;r<ESKF_NAV_DIM;r++)
        for(int c=0;c<ESKF_NAV_DIM;c++)
            for(int k=0;k<ESKF_NAV_DIM;k++)
                FP[r][c]+=F[r][k]*P0[k][c];
    for(int r=0;r<ESKF_NAV_DIM;r++)
        for(int c=0;c<ESKF_NAV_DIM;c++)
            for(int k=0;k<ESKF_NAV_DIM;k++)
                out[r][c]+=FP[r][k]*F[c][k];
    float qg=f->gyro_noise*f->gyro_noise*dt;
    float qa=f->accel_noise*f->accel_noise*dt;
    float qbg=f->gyro_bias_walk*f->gyro_bias_walk*dt;
    float qba=f->accel_bias_walk*f->accel_bias_walk*dt;
    float qpv=f->accel_noise*f->accel_noise*dt*dt*0.5f;
    float qp=f->accel_noise*f->accel_noise*dt*dt*dt/3.0f;
    for(int i=0;i<3;i++){
        out[i][i]+=qg;
        out[3+i][3+i]+=qa;
        out[3+i][6+i]+=qpv; out[6+i][3+i]+=qpv;
        out[6+i][6+i]+=qp;
        out[9+i][9+i]+=qbg;
        out[12+i][12+i]+=qba;
    }
}

int main(void)
{
    const float g=GRAVITY_MPS2, dt=0.013f;
    float a0[3]={0.0f,0.0f,g};
    float da[3]={0.004f,-0.002f,0.006f};
    float dv[3]={0.012f,-0.008f,g*dt+0.004f};
    EskfNav f; eskf_nav_init(&f,a0);
    if(!eskf_nav_reset_yaw(&f,0.37f,0.1f))return 2;
    make_test_covariance(f.P);
    float P0[ESKF_NAV_DIM][ESKF_NAV_DIM], qmid[4], half[3], dtheta[3], dvel[3];
    memcpy(P0,f.P,sizeof(P0)); memcpy(qmid,f.q,sizeof(qmid));
    for(int i=0;i<3;i++){
        dtheta[i]=da[i]-f.gyro_bias[i]*dt;
        dvel[i]=dv[i]-f.accel_bias[i]*dt;
        half[i]=0.5f*dtheta[i];
    }
    qright(qmid,half);
    EskfNav midpoint=f; memcpy(midpoint.q,qmid,sizeof(qmid));
    float R[3][3]; eskf_nav_rotation_matrix(&midpoint,R);
    float w[3],fb[3];
    for(int i=0;i<3;i++){w[i]=dtheta[i]/dt;fb[i]=dvel[i]/dt;}

    float expected[ESKF_NAV_DIM][ESKF_NAV_DIM];
    reference_predict(&f,P0,w,fb,R,dt,expected);
    if(!eskf_nav_predict_delta(&f,da,dv,dt))return 3;

    float max_abs=0.0f;
    for(int r=0;r<ESKF_NAV_DIM;r++)for(int c=0;c<ESKF_NAV_DIM;c++){
        float e=fabsf(f.P[r][c]-expected[r][c]);
        if(e>max_abs)max_abs=e;
    }
    printf("covariance reference max_abs_error=%.9g\n",(double)max_abs);
    if(max_abs>2e-6f){
        fprintf(stderr,"FAIL: sparse covariance propagation differs from full FPF^T reference\n");
        return 1;
    }
    puts("PASS: sparse covariance propagation matches full matrix reference");
    return 0;
}
