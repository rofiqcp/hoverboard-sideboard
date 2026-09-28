#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "app_config.h"
#include "eskf_nav.h"

static int failures=0;
static void check(int ok,const char *msg){printf("%s: %s\n",ok?"PASS":"FAIL",msg);if(!ok)failures++;}
static uint32_t rng_state=0x12345678U;
static float rand_uniform(void){
 rng_state=1664525U*rng_state+1013904223U;
 return ((float)(rng_state>>8)+1.0f)/16777217.0f;
}
static float rand_gaussian(void){
 float u1=rand_uniform(),u2=rand_uniform();
 return sqrtf(-2.0f*logf(u1))*cosf(6.28318530717958647692f*u2);
}
static float qdot(const float a[4],const float b[4]){return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]+a[3]*b[3];}
static void qconj(const float q[4],float o[4]){o[0]=q[0];o[1]=-q[1];o[2]=-q[2];o[3]=-q[3];}
static void qmul(const float a[4],const float b[4],float o[4]){
 o[0]=a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3];
 o[1]=a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2];
 o[2]=a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1];
 o[3]=a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0];
}
static void qright_axis(float q[4],int axis,float angle){
 float dq[4]={cosf(0.5f*angle),0.0f,0.0f,0.0f},out[4];
 dq[axis+1]=sinf(0.5f*angle); qmul(q,dq,out); memcpy(q,out,sizeof(out));
}
static float body_x_measurement(const EskfNav *f){
 float R[3][3]; eskf_nav_rotation_matrix(f,R); float bx=0.0f;
 for(int w=0;w<3;w++)bx+=R[w][0]*f->velocity[w];
 return bx;
}

static void test_covariance_phase_invariance(void){
 const float g=GRAVITY_MPS2; EskfNav a,b; float a0[3]={0,0,g}; eskf_nav_init(&a,a0); eskf_nav_init(&b,a0);
 int propagation_ok=1;
 for(int k=0;k<1000;k++){
  float wa=((k&1)==0)?2.0f:0.0f, wb=((k&1)==0)?0.0f:2.0f;
  float daa[3]={0,0,wa*0.01f}, dab[3]={0,0,wb*0.01f}, dv[3]={0,0,g*0.01f};
  propagation_ok &= eskf_nav_predict_delta(&a,daa,dv,0.01f);
  propagation_ok &= eskf_nav_predict_delta(&b,dab,dv,0.01f);
 }
 check(propagation_ok,"alternating-motion propagation accepted");
 float max_rel=0.0f;
 for(int i=0;i<ESKF_NAV_DIM;i++){
  float den=fmaxf(fmaxf(fabsf(a.P[i][i]),fabsf(b.P[i][i])),1e-6f);
  float rel=fabsf(a.P[i][i]-b.P[i][i])/den; if(rel>max_rel)max_rel=rel;
 }
 check(max_rel<0.08f,"covariance is not scheduler-phase dependent for equivalent alternating motion");
}

static void test_atomic_vector_updates(void){
 const float g=GRAVITY_MPS2; float a0[3]={0,0,g}; EskfNav f,before;
 eskf_nav_init(&f,a0); before=f; float vel[3]={0.10f,1.0f,0.0f};
 check(!eskf_nav_fuse_world_velocity(&f,vel,0.01f),"joint world velocity rejects inconsistent 3D measurement");
 check(memcmp(&f.q,&before.q,sizeof(f.q))==0 && memcmp(f.velocity,before.velocity,sizeof(f.velocity))==0 &&
       memcmp(f.position,before.position,sizeof(f.position))==0 && memcmp(f.P,before.P,sizeof(f.P))==0,
       "rejected world velocity leaves nominal state and covariance unchanged");
 eskf_nav_init(&f,a0); before=f; float pos[3]={0.10f,1.0f,0.0f};
 check(!eskf_nav_fuse_world_position(&f,pos,0.02f),"joint world position rejects inconsistent 3D measurement");
 check(memcmp(f.position,before.position,sizeof(f.position))==0 && memcmp(f.P,before.P,sizeof(f.P))==0,
       "rejected world position is atomic");
 eskf_nav_init(&f,a0); before=f; float body[3]={0.10f,1.0f,0.0f};
 check(!eskf_nav_fuse_body_velocity(&f,body,0x03U,0.01f),"joint body velocity rejects inconsistent requested axes");
 check(memcmp(f.velocity,before.velocity,sizeof(f.velocity))==0 && memcmp(f.P,before.P,sizeof(f.P))==0,
       "rejected body velocity is atomic");
}

static void test_body_velocity_jacobian(void){
 const float deg=0.01745329251994329577f,r=12.0f*deg,p=-8.0f*deg,y=35.0f*deg,g=GRAVITY_MPS2;
 float a0[3]={-sinf(p)*g,sinf(r)*cosf(p)*g,cosf(r)*cosf(p)*g}; EskfNav f;
 eskf_nav_init(&f,a0); check(eskf_nav_reset_yaw(&f,y,0.05f),"body-velocity test yaw initialized");
 f.velocity[0]=2.0f; f.velocity[1]=-0.7f; f.velocity[2]=0.2f;
 memset(f.P,0,sizeof(f.P)); for(int i=0;i<ESKF_NAV_DIM;i++)f.P[i][i]=1e-9f;
 for(int i=0;i<3;i++){f.P[i][i]=0.01f;f.P[3+i][3+i]=0.04f;}
 EskfNav before=f; const float eps=1e-4f,sigma=0.10f,innov=0.08f;
 float H[6]={0.0f};
 for(int axis=0;axis<3;axis++){
  EskfNav plus=before,minus=before; qright_axis(plus.q,axis,eps);qright_axis(minus.q,axis,-eps);
  H[axis]=(body_x_measurement(&plus)-body_x_measurement(&minus))/(2.0f*eps);
 }
 for(int axis=0;axis<3;axis++){
  EskfNav plus=before,minus=before;plus.velocity[axis]+=eps;minus.velocity[axis]-=eps;
  H[3+axis]=(body_x_measurement(&plus)-body_x_measurement(&minus))/(2.0f*eps);
 }
 float S=sigma*sigma;for(int i=0;i<3;i++){S+=0.01f*H[i]*H[i];S+=0.04f*H[3+i]*H[3+i];}
 float measurement[3]={body_x_measurement(&before)+innov,0.0f,0.0f};
 check(eskf_nav_fuse_body_velocity(&f,measurement,0x01U,sigma),"body-x velocity update accepted at tilted pose");
 float qc[4],dq[4];qconj(before.q,qc);qmul(qc,f.q,dq);if(dq[0]<0.0f)for(int i=0;i<4;i++)dq[i]=-dq[i];
 float actual_th[3]={2.0f*dq[1],2.0f*dq[2],2.0f*dq[3]};
 int ok=1;for(int i=0;i<3;i++){
  float eth=0.01f*H[i]*innov/S,ev=0.04f*H[3+i]*innov/S;
  if(fabsf(actual_th[i]-eth)>0.0025f||fabsf((f.velocity[i]-before.velocity[i])-ev)>0.0025f)ok=0;
 }
 check(ok,"body-velocity update matches finite-difference right-error Jacobian");
 check(eskf_nav_covariance_psd_check(&f),"body-velocity update preserves PSD covariance");
}

static void test_gravity_tangent_update(void){
 const float tilt=2.0f*0.01745329251994329577f,g=GRAVITY_MPS2;
 float ainit[3]={0.0f,sinf(tilt)*g,cosf(tilt)*g}, level[3]={0.0f,0.0f,g}; EskfNav f;
 eskf_nav_init(&f,ainit); float r0=0,p0=0,y0=0,r1=0,p1=0,y1=0;
 eskf_nav_get_euler_deg(&f,&r0,&p0,&y0);
 check(eskf_nav_correct_gravity(&f,level,1),"2-DOF gravity tangent update accepts small tilt residual");
 eskf_nav_get_euler_deg(&f,&r1,&p1,&y1);
 check(fabsf(r1)<fabsf(r0) && fabsf(p1)<=fabsf(p0)+1e-5f,"gravity tangent update reduces tilt error");
 check(eskf_nav_covariance_psd_check(&f),"gravity tangent update preserves PSD covariance");
}

static void test_process_noise_nees_consistency(void){
 const int runs=256,steps=100; const float dt=0.01f,g=GRAVITY_MPS2,sigma_a=0.18f;
 double nees_sum=0.0,err2_sum=0.0,p_sum=0.0; rng_state=0x12345678U;
 for(int run=0;run<runs;run++){
  float a0[3]={0.0f,0.0f,g}; EskfNav f; eskf_nav_init(&f,a0);
  memset(f.P,0,sizeof(f.P)); for(int i=0;i<ESKF_NAV_DIM;i++)f.P[i][i]=1e-9f;
  f.gyro_noise=1e-6f; f.accel_noise=sigma_a; f.gyro_bias_walk=0.0f; f.accel_bias_walk=0.0f;
  for(int k=0;k<steps;k++){
   float accel_noise=sigma_a/sqrtf(dt)*rand_gaussian();
   float da[3]={0.0f,0.0f,0.0f},dv[3]={accel_noise*dt,0.0f,g*dt};
   if(!eskf_nav_predict_delta(&f,da,dv,dt)){failures++;return;}
  }
  double err=f.velocity[0],var=f.P[3][3];
  nees_sum+=err*err/var; err2_sum+=err*err; p_sum+=var;
 }
 double nees=nees_sum/(double)runs,ratio=err2_sum/p_sum;
 check(nees>0.80 && nees<1.20,"Monte-Carlo velocity NEES matches configured accel process noise");
 check(ratio>0.80 && ratio<1.20,"empirical velocity error variance matches propagated covariance");
}

static void test_fast_covariance_health_check(void){
 const float g=GRAVITY_MPS2; float a0[3]={0.0f,0.0f,g}; EskfNav f; eskf_nav_init(&f,a0);
 for(int i=0;i<3;i++)for(int j=0;j<3;j++)f.P[i][j]=(i==j)?1.0f:-0.9f;
 check(!eskf_nav_is_healthy(&f),"fast health check rejects indefinite 3x3 attitude covariance");
 check(!eskf_nav_covariance_psd_check(&f),"full PSD check agrees with fast block rejection");
}

static void test_bias_bound_covariance_consistency(void){
 const float g=GRAVITY_MPS2; float a0[3]={0.0f,0.0f,g}; EskfNav f; eskf_nav_init(&f,a0);
 f.gyro_bias[0]=ESKF_GYRO_BIAS_LIMIT_RAD-0.001f;
 float gyro[3]={0.20f,0.0f,0.0f};
 check(eskf_nav_fuse_zero_rate(&f,gyro,0.008f),"zero-rate update near gyro-bias bound accepted");
 check(f.gyro_bias[0]<=ESKF_GYRO_BIAS_LIMIT_RAD+1e-7f &&
       f.gyro_bias[0]>=ESKF_GYRO_BIAS_LIMIT_RAD-2e-6f,
       "gyro-bias correction respects configured hard bound");
 check(f.diagnostics.bias_saturation_count>=1U,"bias-bound gain limiting is diagnosed");
 check(eskf_nav_covariance_psd_check(&f),"bias-bound gain limiting preserves PSD covariance");
 float p_after=f.P[9][9];
 check(eskf_nav_fuse_zero_rate(&f,gyro,0.008f),"repeated outward bias update remains numerically valid");
 check(f.P[9][9]>=p_after*0.999f,"saturated bias covariance does not collapse from unapplied correction");
}

static void test_clipping_noise_scaling(void){
 const float g=GRAVITY_MPS2,dt=0.01f; float a0[3]={0.0f,0.0f,g}; EskfNav normal,clipped;
 eskf_nav_init(&normal,a0); clipped=normal;
 float da[3]={0.002f,-0.001f,0.003f},dv[3]={0.01f,-0.005f,g*dt};
 check(eskf_nav_predict_delta(&normal,da,dv,dt),"normal-noise propagation accepted");
 check(eskf_nav_predict_delta_scaled(&clipped,da,dv,dt,ESKF_CLIP_NOISE_SCALE,ESKF_CLIP_NOISE_SCALE),
       "clip-scaled propagation accepted");
 check(fabsf(normal.q[0]-clipped.q[0])<1e-7f && fabsf(normal.q[1]-clipped.q[1])<1e-7f &&
       fabsf(normal.q[2]-clipped.q[2])<1e-7f && fabsf(normal.q[3]-clipped.q[3])<1e-7f &&
       fabsf(normal.velocity[0]-clipped.velocity[0])<1e-7f &&
       fabsf(normal.velocity[1]-clipped.velocity[1])<1e-7f &&
       fabsf(normal.velocity[2]-clipped.velocity[2])<1e-7f,
       "noise scaling changes covariance but not nominal propagation");
 check(clipped.P[0][0]>normal.P[0][0] && clipped.P[3][3]>normal.P[3][3] && clipped.P[6][6]>normal.P[6][6],
       "clip-scaled propagation increases attitude/velocity/position uncertainty");
 check(eskf_nav_covariance_psd_check(&clipped),"clip-scaled covariance remains PSD");
 check(!eskf_nav_predict_delta_scaled(&clipped,da,dv,dt,0.5f,1.0f),
       "invalid sub-unity noise scale is rejected");
}

static void test_imu_gap_covariance_inflation(void){
 const float g=GRAVITY_MPS2,gap=0.05f; float a0[3]={0.0f,0.0f,g}; EskfNav f; eskf_nav_init(&f,a0);
 float pth=f.P[0][0],pv=f.P[3][3],pp=f.P[6][6],pvp=f.P[3][6];
 float sg=f.gyro_noise*ESKF_IMU_GAP_NOISE_SCALE,sa=f.accel_noise*ESKF_IMU_GAP_NOISE_SCALE;
 float qg=sg*sg*gap,qa=sa*sa*gap,qp=sa*sa*gap*gap*gap/3.0f,qpv=sa*sa*gap*gap*0.5f;
 check(eskf_nav_inflate_for_imu_gap(&f,gap),"IMU gap covariance inflation accepted");
 check(fabsf((f.P[0][0]-pth)-qg)<1e-6f,"IMU gap attitude variance follows integrated gyro noise");
 check(fabsf((f.P[3][3]-pv)-qa)<1e-6f,"IMU gap velocity variance follows integrated accel noise");
 check(fabsf((f.P[6][6]-pp)-qp)<1e-6f && fabsf((f.P[3][6]-pvp)-qpv)<1e-6f,
       "IMU gap position and velocity-position covariance use continuous-noise discretization");
 check(f.diagnostics.imu_gap_count==1U && fabsf(f.diagnostics.last_imu_gap_s-gap)<1e-7f &&
       fabsf(f.diagnostics.max_imu_gap_s-gap)<1e-7f,"IMU gap diagnostics track count/last/max");
 check(eskf_nav_covariance_psd_check(&f),"IMU gap inflation preserves PSD covariance");
 unsigned before=f.diagnostics.imu_gap_count;
 check(!eskf_nav_inflate_for_imu_gap(&f,-0.1f) && f.diagnostics.imu_gap_count==before,
       "invalid IMU gap is rejected without mutating diagnostics");
}

static void test_tilted_yaw_jacobian(void){
 const float r=20.0f*0.01745329251994329577f, p=15.0f*0.01745329251994329577f, g=GRAVITY_MPS2;
 float a0[3]={-sinf(p)*g,sinf(r)*cosf(p)*g,cosf(r)*cosf(p)*g}; EskfNav f; eskf_nav_init(&f,a0);
 memset(f.P,0,sizeof(f.P)); for(int i=0;i<3;i++)f.P[i][i]=0.01f;
 for(int i=3;i<ESKF_NAV_DIM;i++)f.P[i][i]=1e-6f;
 float q0[4]; memcpy(q0,f.q,sizeof(q0)); const float sigma=0.05f, innov=0.10f;
 check(eskf_nav_fuse_yaw(&f,innov,sigma),"tilted yaw observation accepted");
 float qc[4],dq[4]; qconj(q0,qc); qmul(qc,f.q,dq); if(dq[0]<0){for(int i=0;i<4;i++)dq[i]=-dq[i];}
 float dtheta[3]={2.0f*dq[1],2.0f*dq[2],2.0f*dq[3]};
 float H[3]={0.0f,sinf(r)/cosf(p),cosf(r)/cosf(p)};
 float h2=H[0]*H[0]+H[1]*H[1]+H[2]*H[2]; float gain=0.01f/(0.01f*h2+sigma*sigma);
 float expected[3]={gain*H[0]*innov,gain*H[1]*innov,gain*H[2]*innov};
 check(fabsf(dtheta[0]-expected[0])<0.003f && fabsf(dtheta[1]-expected[1])<0.003f && fabsf(dtheta[2]-expected[2])<0.003f,
       "yaw correction uses correct right-error Jacobian at nonzero roll/pitch");
 check(qdot(f.q,f.q)>0.999f && qdot(f.q,f.q)<1.001f,"yaw update keeps quaternion normalized");
}

int main(void){
 test_covariance_phase_invariance();
 test_atomic_vector_updates();
 test_body_velocity_jacobian();
 test_gravity_tangent_update();
 test_process_noise_nees_consistency();
 test_fast_covariance_health_check();
 test_bias_bound_covariance_consistency();
 test_clipping_noise_scaling();
 test_imu_gap_covariance_inflation();
 test_tilted_yaw_jacobian();
 return failures?1:0;
}
