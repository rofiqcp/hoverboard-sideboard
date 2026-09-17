#include "eskf_nav.h"
#include "app_config.h"
#include <math.h>
#include <string.h>

#define PI_F 3.14159265358979323846f
#define IDX_TH 0
#define IDX_V  3
#define IDX_P  6
#define IDX_BG 9
#define IDX_BA 12

/* Scratch global: menghindari stack besar pada STM32F103 20 KB RAM. */
static float Tm[ESKF_NAV_DIM][ESKF_NAV_DIM];
static float HPm[3][ESKF_NAV_DIM];
static float dxm[ESKF_NAV_DIM];

static float clampf_local(float x, float lo, float hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}

static void quat_normalize(float q[4]) {
    float n2=q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3];
    if (n2 < 1e-12f) { q[0]=1.0f; q[1]=q[2]=q[3]=0.0f; return; }
    float s=1.0f/sqrtf(n2);
    for (int i=0;i<4;i++) q[i]*=s;
}

static void quat_inject(float q[4], const float d[3]) {
    float a2=d[0]*d[0]+d[1]*d[1]+d[2]*d[2];
    float dq[4];
    if (a2 < 0.0025f) {
        /* Untuk |dtheta| < 0,05 rad gunakan Taylor orde-4. Error sangat kecil,
         * tetapi menghindari sqrt/sin/cos pada hampir semua sampel AGV. */
        float a4=a2*a2;
        float s=0.5f-a2*(1.0f/48.0f)+a4*(1.0f/3840.0f);
        dq[0]=1.0f-a2*(1.0f/8.0f)+a4*(1.0f/384.0f);
        dq[1]=s*d[0]; dq[2]=s*d[1]; dq[3]=s*d[2];
    } else {
        float a=sqrtf(a2);
        float s=sinf(0.5f*a)/a;
        dq[0]=cosf(0.5f*a); dq[1]=s*d[0]; dq[2]=s*d[1]; dq[3]=s*d[2];
    }
    float o[4];
    o[0]=q[0]*dq[0]-q[1]*dq[1]-q[2]*dq[2]-q[3]*dq[3];
    o[1]=q[0]*dq[1]+q[1]*dq[0]+q[2]*dq[3]-q[3]*dq[2];
    o[2]=q[0]*dq[2]-q[1]*dq[3]+q[2]*dq[0]+q[3]*dq[1];
    o[3]=q[0]*dq[3]+q[1]*dq[2]-q[2]*dq[1]+q[3]*dq[0];
    memcpy(q,o,sizeof(o)); quat_normalize(q);
}

static void euler_to_quat(float roll,float pitch,float yaw,float q[4]) {
    float cr=cosf(roll*.5f), sr=sinf(roll*.5f);
    float cp=cosf(pitch*.5f), sp=sinf(pitch*.5f);
    float cy=cosf(yaw*.5f), sy=sinf(yaw*.5f);
    q[0]=cr*cp*cy+sr*sp*sy; q[1]=sr*cp*cy-cr*sp*sy;
    q[2]=cr*sp*cy+sr*cp*sy; q[3]=cr*cp*sy-sr*sp*cy;
    quat_normalize(q);
}

static void skew(const float v[3], float s[3][3]) {
    s[0][0]=0; s[0][1]=-v[2]; s[0][2]=v[1];
    s[1][0]=v[2]; s[1][1]=0; s[1][2]=-v[0];
    s[2][0]=-v[1]; s[2][1]=v[0]; s[2][2]=0;
}

static void rotation_matrix_from_q(const float q[4], float R[3][3])
{
    float w=q[0], x=q[1], y=q[2], z=q[3];
    R[0][0]=1.0f-2.0f*(y*y+z*z);
    R[0][1]=2.0f*(x*y-w*z);
    R[0][2]=2.0f*(x*z+w*y);
    R[1][0]=2.0f*(x*y+w*z);
    R[1][1]=1.0f-2.0f*(x*x+z*z);
    R[1][2]=2.0f*(y*z-w*x);
    R[2][0]=2.0f*(x*z-w*y);
    R[2][1]=2.0f*(y*z+w*x);
    R[2][2]=1.0f-2.0f*(x*x+y*y);
}

void eskf_nav_rotation_matrix(const EskfNav *f, float R[3][3]) {
    rotation_matrix_from_q(f->q, R);
}

static void gravity_body(const EskfNav *f,float h[3]) {
    float R[3][3]; eskf_nav_rotation_matrix(f,R);
    h[0]=R[2][0]; h[1]=R[2][1]; h[2]=R[2][2];
}

static void inject_state(EskfNav *f,const float dx[ESKF_NAV_DIM]) {
    quat_inject(f->q,&dx[IDX_TH]);
    for(int i=0;i<3;i++) {
        f->velocity[i]+=dx[IDX_V+i]; f->position[i]+=dx[IDX_P+i];
        f->gyro_bias[i]=clampf_local(f->gyro_bias[i]+dx[IDX_BG+i],
                                      -ESKF_GYRO_BIAS_LIMIT_RAD,ESKF_GYRO_BIAS_LIMIT_RAD);
        f->accel_bias[i]=clampf_local(f->accel_bias[i]+dx[IDX_BA+i],
                                      -ESKF_ACCEL_BIAS_LIMIT_MPS2,ESKF_ACCEL_BIAS_LIMIT_MPS2);
    }
}

static void reset_covariance_attitude(EskfNav *f, const float dtheta[3]) {
    /* Reset Jacobian ESKF setelah error attitude diinjeksi ke quaternion nominal. */
    float sd[3][3], G[3][3];
    skew(dtheta, sd);
    for (int r=0;r<3;r++) for (int c=0;c<3;c++) {
        G[r][c] = (r==c ? 1.0f : 0.0f) - 0.5f*sd[r][c];
    }
    for (int r=0;r<3;r++) for (int c=0;c<ESKF_NAV_DIM;c++) {
        HPm[r][c]=0.0f;
        for (int k=0;k<3;k++) HPm[r][c]+=G[r][k]*f->P[IDX_TH+k][c];
    }
    for (int r=0;r<3;r++) for (int c=0;c<ESKF_NAV_DIM;c++) f->P[IDX_TH+r][c]=HPm[r][c];
    for (int r=0;r<ESKF_NAV_DIM;r++) {
        float old[3]={f->P[r][IDX_TH],f->P[r][IDX_TH+1],f->P[r][IDX_TH+2]};
        float out[3]={0};
        for (int c=0;c<3;c++) for (int k=0;k<3;k++) out[c]+=old[k]*G[c][k];
        for (int c=0;c<3;c++) f->P[r][IDX_TH+c]=out[c];
    }
    for (int r=0;r<ESKF_NAV_DIM;r++) for (int c=r;c<ESKF_NAV_DIM;c++) {
        float v=0.5f*(f->P[r][c]+f->P[c][r]);
        f->P[r][c]=v; f->P[c][r]=v;
    }
}

static int update1_sparse(EskfNav *f,
                          const uint8_t *h_index, const float *h_value, uint8_t h_count,
                          float innov, float variance, float nis_gate,
                          const float gravity_axis[3], uint8_t accel_bias_mode,
                          uint8_t gyro_bias_mode)
{
    if (!f || !h_index || !h_value || h_count==0U || h_count>ESKF_NAV_DIM ||
        !isfinite(innov) || !isfinite(variance) || !(variance>0.0f)) return 0;
    float hp[ESKF_NAV_DIM];
    for (int c=0;c<ESKF_NAV_DIM;c++) {
        float v=0.0f;
        for (uint8_t j=0U;j<h_count;j++) v+=h_value[j]*f->P[h_index[j]][c];
        hp[c]=v;
    }

    float S=variance;
    for (uint8_t j=0U;j<h_count;j++) S+=hp[h_index[j]]*h_value[j];
    if (!isfinite(S) || !(S>1e-12f)) return 0;
    if (nis_gate>0.0f && innov*innov/S>nis_gate) return 0;

    float K[ESKF_NAV_DIM];
    float inv_s=1.0f/S;
    for (int r=0;r<ESKF_NAV_DIM;r++) K[r]=hp[r]*inv_s;

    if (gravity_axis) {
        float along_th=0.0f, along_bg=0.0f, along_ba=0.0f;
        for (int i=0;i<3;i++) {
            along_th+=gravity_axis[i]*K[IDX_TH+i];
            along_bg+=gravity_axis[i]*K[IDX_BG+i];
            along_ba+=gravity_axis[i]*K[IDX_BA+i];
        }
        for (int i=0;i<3;i++) {
            K[IDX_TH+i]-=gravity_axis[i]*along_th;
            K[IDX_BG+i]-=gravity_axis[i]*along_bg;
            if (accel_bias_mode==0U) K[IDX_BA+i]=0.0f;
            else if (accel_bias_mode==1U) K[IDX_BA+i]=gravity_axis[i]*along_ba;
        }
    } else if (accel_bias_mode==0U) {
        for (int i=0;i<3;i++) K[IDX_BA+i]=0.0f;
    }
    if (gyro_bias_mode==0U) {
        for (int i=0;i<3;i++) K[IDX_BG+i]=0.0f;
    }

    /* Batasi satu correction step dengan menskalakan gain, bukan sekadar dx.
     * Karena covariance update memakai K yang sama, konsistensi Joseph-form tetap terjaga. */
    float gain_scale=1.0f;
    const float limits[5]={ESKF_ATTITUDE_STEP_MAX_RAD,ESKF_VELOCITY_STEP_MAX_MPS,
                           ESKF_POSITION_STEP_MAX_M,ESKF_GYRO_BIAS_STEP_MAX_RAD,
                           ESKF_ACCEL_BIAS_STEP_MAX_MPS2};
    const int starts[5]={IDX_TH,IDX_V,IDX_P,IDX_BG,IDX_BA};
    for(int group=0;group<5;group++) for(int i=0;i<3;i++) {
        float correction=fabsf(K[starts[group]+i]*innov);
        if(!isfinite(correction)) return 0;
        if(correction>limits[group] && correction>1e-12f) {
            float a=limits[group]/correction;
            if(a<gain_scale) gain_scale=a;
        }
    }
    if(gain_scale<1.0f) for(int r=0;r<ESKF_NAV_DIM;r++) K[r]*=gain_scale;
    for (int r=0;r<ESKF_NAV_DIM;r++) dxm[r]=K[r]*innov;

    /* Joseph-equivalent scalar update tetap valid walau K diproyeksikan/diskalakan. */
    for (int r=0;r<ESKF_NAV_DIM;r++) for (int c=r;c<ESKF_NAV_DIM;c++) {
        float v=f->P[r][c]-K[r]*hp[c]-hp[r]*K[c]+K[r]*S*K[c];
        Tm[r][c]=v; Tm[c][r]=v;
    }
    memcpy(f->P,Tm,sizeof(f->P));
    for (int i=0;i<ESKF_NAV_DIM;i++) if (f->P[i][i] < 1e-9f) f->P[i][i]=1e-9f;
    inject_state(f,dxm);
    reset_covariance_attitude(f,&dxm[IDX_TH]);
    return 1;
}

static void reset_covariance_defaults(EskfNav *f)
{
    memset(f->P,0,sizeof(f->P));
    for(int i=0;i<3;i++) {
        f->P[IDX_TH+i][IDX_TH+i]=0.08f*0.08f;
        f->P[IDX_V+i][IDX_V+i]=0.20f*0.20f;
        f->P[IDX_P+i][IDX_P+i]=0.10f*0.10f;
        f->P[IDX_BG+i][IDX_BG+i]=0.03f*0.03f;
        f->P[IDX_BA+i][IDX_BA+i]=0.20f*0.20f;
    }
    f->covariance_dt_accum=0.0f;
    f->covariance_divider=0U;
}

void eskf_nav_reset_covariance(EskfNav *f)
{
    if(!f) return;
    reset_covariance_defaults(f);
}

void eskf_nav_init(EskfNav *f,const float accel[3]) {
    memset(f,0,sizeof(*f));
    float roll=atan2f(accel[1],accel[2]);
    float pitch=atan2f(-accel[0],sqrtf(accel[1]*accel[1]+accel[2]*accel[2]));
    euler_to_quat(roll,pitch,0.0f,f->q);
    f->gyro_noise=ESKF_GYRO_NOISE_RAD;
    f->accel_noise=ESKF_ACCEL_PROCESS_NOISE;
    f->gyro_bias_walk=ESKF_GYRO_BIAS_WALK_RAD;
    f->accel_bias_walk=ESKF_ACCEL_BIAS_WALK;
    f->accel_dir_noise=ESKF_ACCEL_DIR_NOISE;
    reset_covariance_defaults(f);
    f->initialized=1U;
}

void eskf_nav_reset_motion(EskfNav *f) {
    if(!f) return;
    for(int i=0;i<3;i++){f->velocity[i]=0;f->position[i]=0;}
}

void eskf_nav_linear_accel_world(const EskfNav *f,const float accel[3],float out[3]) {
    float R[3][3],fb[3]; eskf_nav_rotation_matrix(f,R);
    for(int i=0;i<3;i++) fb[i]=accel[i]-f->accel_bias[i];
    for(int r=0;r<3;r++){out[r]=0;for(int c=0;c<3;c++)out[r]+=R[r][c]*fb[c];}
    out[2]-=GRAVITY_MPS2;
}

static void covariance_predict(EskfNav *f, const float w[3], const float fb[3],
                               const float R[3][3], float dt)
{
    f->predict_count++;
    f->covariance_dt_accum += dt;
    if (++f->covariance_divider < ESKF_COVARIANCE_DIVIDER) return;
    f->covariance_divider = 0U;

    float cov_dt = clampf_local(f->covariance_dt_accum, 0.001f, 0.05f);
    f->covariance_dt_accum = 0.0f;
    memset(Tm, 0, sizeof(Tm));

    float skew_w[3][3], skew_f[3][3], vel_theta[3][3];
    skew(w, skew_w);
    skew(fb, skew_f);
    for (int r=0;r<3;r++) for (int c=0;c<3;c++) {
        vel_theta[r][c]=0.0f;
        for (int k=0;k<3;k++) vel_theta[r][c]-=R[r][k]*skew_f[k][c];
    }

    for (int c=0;c<ESKF_NAV_DIM;c++) {
        for (int r=0;r<3;r++) {
            float th=-f->P[IDX_BG+r][c];
            float vv=0.0f;
            for (int k=0;k<3;k++) {
                th-=skew_w[r][k]*f->P[IDX_TH+k][c];
                vv+=vel_theta[r][k]*f->P[IDX_TH+k][c];
                vv-=R[r][k]*f->P[IDX_BA+k][c];
            }
            Tm[IDX_TH+r][c]=th;
            Tm[IDX_V+r][c]=vv;
            Tm[IDX_P+r][c]=f->P[IDX_V+r][c];
        }
    }

    /* Discretisasi F ~= I + A*dt: P = F*P*F^T + Q.
     * Term dt^2*A*P*A^T wajib dipertahankan agar PSD tidak rusak.
     * A hanya punya row nonzero untuk theta/velocity/position, sehingga
     * extra term dihitung sparse tanpa generic 15x15 matrix multiply. */
    for (int r=0;r<ESKF_NAV_DIM;r++) for (int c=r;c<ESKF_NAV_DIM;c++) {
        float second=0.0f;
        if (r<IDX_BG && c<IDX_BG) {
            float brc=0.0f,bcr=0.0f;
            if (c<IDX_V) {
                for(int k=0;k<3;k++) brc += Tm[r][IDX_TH+k]*(-skew_w[c][k]);
                brc -= Tm[r][IDX_BG+c];
            } else if (c<IDX_P) {
                int a=c-IDX_V;
                for(int k=0;k<3;k++) { brc += Tm[r][IDX_TH+k]*vel_theta[a][k]; brc -= Tm[r][IDX_BA+k]*R[a][k]; }
            } else { brc = Tm[r][IDX_V+(c-IDX_P)]; }
            if (r<IDX_V) {
                for(int k=0;k<3;k++) bcr += Tm[c][IDX_TH+k]*(-skew_w[r][k]);
                bcr -= Tm[c][IDX_BG+r];
            } else if (r<IDX_P) {
                int a=r-IDX_V;
                for(int k=0;k<3;k++) { bcr += Tm[c][IDX_TH+k]*vel_theta[a][k]; bcr -= Tm[c][IDX_BA+k]*R[a][k]; }
            } else { bcr = Tm[c][IDX_V+(r-IDX_P)]; }
            second=0.5f*(brc+bcr);
        }
        float v=f->P[r][c] + cov_dt*(Tm[r][c]+Tm[c][r]) + cov_dt*cov_dt*second;
        f->P[r][c]=v; f->P[c][r]=v;
    }

    float qg=f->gyro_noise*f->gyro_noise*cov_dt;
    float qa=f->accel_noise*f->accel_noise*cov_dt;
    float qbg=f->gyro_bias_walk*f->gyro_bias_walk*cov_dt;
    float qba=f->accel_bias_walk*f->accel_bias_walk*cov_dt;
    for (int i=0;i<3;i++) {
        f->P[IDX_TH+i][IDX_TH+i]+=qg;
        f->P[IDX_V+i][IDX_V+i]+=qa;
        /* White acceleration noise also contributes to position and v-p cross covariance. */
        float qpv=f->accel_noise*f->accel_noise*cov_dt*cov_dt*0.5f;
        float qp=f->accel_noise*f->accel_noise*cov_dt*cov_dt*cov_dt*(1.0f/3.0f);
        f->P[IDX_V+i][IDX_P+i]+=qpv; f->P[IDX_P+i][IDX_V+i]=f->P[IDX_V+i][IDX_P+i];
        f->P[IDX_P+i][IDX_P+i]+=qp;
        f->P[IDX_BG+i][IDX_BG+i]+=qbg;
        f->P[IDX_BA+i][IDX_BA+i]+=qba;
    }
    for (int i=0;i<ESKF_NAV_DIM;i++) if (f->P[i][i] < 1e-9f) f->P[i][i]=1e-9f;
}

int eskf_nav_predict_delta(EskfNav *f, const float delta_angle[3],
                            const float delta_velocity[3], float dt)
{
    if (!f || !f->initialized || !delta_angle || !delta_velocity ||
        !isfinite(dt) || dt < 0.001f || dt > 0.050f) return 0;

    float dtheta[3], dvel_body[3], half_theta[3];
    for (int i=0;i<3;i++) {
        dtheta[i]=delta_angle[i]-f->gyro_bias[i]*dt;
        dvel_body[i]=delta_velocity[i]-f->accel_bias[i]*dt;
        half_theta[i]=0.5f*dtheta[i];
    }

    /* Delta-velocity diputar memakai attitude tengah interval. Ini mengurangi
     * error integrasi saat kendaraan berotasi dibanding memakai attitude akhir. */
    float q_mid[4]; memcpy(q_mid,f->q,sizeof(q_mid));
    quat_inject(q_mid,half_theta);
    float Rmid[3][3]; rotation_matrix_from_q(q_mid,Rmid);

    float dv_world[3]={0.0f,0.0f,0.0f};
    for (int r=0;r<3;r++) for (int c=0;c<3;c++)
        dv_world[r]+=Rmid[r][c]*dvel_body[c];
    dv_world[2]-=GRAVITY_MPS2*dt;

    float v_old[3]={f->velocity[0],f->velocity[1],f->velocity[2]};
    for (int i=0;i<3;i++) f->velocity[i]+=dv_world[i];
    for (int i=0;i<3;i++)
        f->position[i]+=0.5f*(v_old[i]+f->velocity[i])*dt;

    quat_inject(f->q,dtheta);

    float inv_dt=1.0f/dt, w[3], fb[3];
    for (int i=0;i<3;i++) { w[i]=dtheta[i]*inv_dt; fb[i]=dvel_body[i]*inv_dt; }
    covariance_predict(f,w,fb,Rmid,dt);
    return 1;
}

void eskf_nav_predict(EskfNav *f, const float gyro[3], const float accel[3], float dt)
{
    if (!gyro || !accel || dt<=0.0f) return;
    float da[3],dv[3];
    for (int i=0;i<3;i++) { da[i]=gyro[i]*dt; dv[i]=accel[i]*dt; }
    (void)eskf_nav_predict_delta(f,da,dv,dt);
}

int eskf_nav_correct_gravity(EskfNav *f, const float accel[3], int stationary)
{
    float a[3];
    for (int i=0;i<3;i++) a[i]=accel[i]-f->accel_bias[i];
    float n=sqrtf(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
    if (n<1e-4f) { f->gravity_reject_count++; return 0; }

    float gr=n/GRAVITY_MPS2;
    float gate_min=stationary ? ESKF_ACCEL_GATE_MIN_G : ESKF_MOVING_ACCEL_GATE_MIN_G;
    float gate_max=stationary ? ESKF_ACCEL_GATE_MAX_G : ESKF_MOVING_ACCEL_GATE_MAX_G;
    if (gr<gate_min || gr>gate_max) { f->gravity_reject_count++; return 0; }

    float z[3]={a[0]/n,a[1]/n,a[2]/n};
    float h_gate[3]; gravity_body(f,h_gate);
    float dir_dot=z[0]*h_gate[0]+z[1]*h_gate[1]+z[2]*h_gate[2];
    float dir_gate=stationary ? ESKF_GRAVITY_DIR_COS_STILL : ESKF_GRAVITY_DIR_COS_MOVING;
    if(!isfinite(dir_dot) || dir_dot<dir_gate) { f->gravity_reject_count++; return 0; }

    float sigma=f->accel_dir_noise*(stationary ? 1.0f : ESKF_MOVING_GRAVITY_NOISE_SCALE);
    sigma*=1.0f+8.0f*fabsf(gr-1.0f);

    int fused=0;
    for (int axis=0;axis<3;axis++) {
        float h[3], sh[3][3];
        gravity_body(f,h);
        float innov=z[axis]-h[axis];
        skew(h,sh);
        const uint8_t h_index[3]={IDX_TH,IDX_TH+1,IDX_TH+2};
        float h_value[3]={sh[axis][0],sh[axis][1],sh[axis][2]};

        if (!update1_sparse(f,h_index,h_value,3U,innov,sigma*sigma,
                            ESKF_GRAVITY_SCALAR_NIS_GATE,h,0U,2U)) {
            f->gravity_reject_count++;
            continue;
        }
        fused++;
    }

    if (fused==3) { f->gravity_fuse_count++; return 1; }
    return 0;
}

int eskf_nav_fuse_zero_velocity(EskfNav *f,float sigma)
{
    float gravity_axis[3]; gravity_body(f,gravity_axis);
    int fused=0;
    for (int axis=0;axis<3;axis++) {
        const uint8_t h_index[3]={(uint8_t)(IDX_V+axis),0U,0U};
        const float h_value[3]={1.0f,0.0f,0.0f};
        if (update1_sparse(f,h_index,h_value,1U,-f->velocity[axis],sigma*sigma,0.0f,
                           gravity_axis,1U,2U)) fused++;
    }
    if (fused==3) { f->zupt_count++; return 1; }
    return 0;
}

int eskf_nav_fuse_zero_rate(EskfNav *f,const float gyro[3],float sigma)
{
    int fused=0;
    for (int axis=0;axis<3;axis++) {
        const uint8_t h_index[3]={(uint8_t)(IDX_BG+axis),0U,0U};
        const float h_value[3]={1.0f,0.0f,0.0f};
        float innov=gyro[axis]-f->gyro_bias[axis];
        if (update1_sparse(f,h_index,h_value,1U,innov,sigma*sigma,0.0f,0,0U,2U)) fused++;
    }
    if (fused==3) { f->zero_rate_count++; return 1; }
    return 0;
}



static float wrap_pi(float x)
{
    while (x > PI_F) x -= 2.0f*PI_F;
    while (x < -PI_F) x += 2.0f*PI_F;
    return x;
}

int eskf_nav_fuse_world_velocity(EskfNav *f,const float velocity[3],float sigma)
{
    if(!f||!velocity||!isfinite(sigma)||sigma<AID_SIGMA_VEL_MIN_MPS||sigma>AID_SIGMA_VEL_MAX_MPS)return 0;
    for(int axis=0;axis<3;axis++) {
        if(!isfinite(velocity[axis])||fabsf(velocity[axis])>AID_WORLD_VEL_MAX_MPS ||
           fabsf(velocity[axis]-f->velocity[axis])>AID_WORLD_VEL_INNOV_MAX_MPS) return 0;
    }
    int fused=0;
    for(int axis=0;axis<3;axis++){
        const uint8_t hi[1]={(uint8_t)(IDX_V+axis)};
        const float hv[1]={1.0f};
        if(update1_sparse(f,hi,hv,1U,velocity[axis]-f->velocity[axis],sigma*sigma,9.0f,0,2U,2U))fused++;
    }
    return fused==3;
}

int eskf_nav_fuse_world_position(EskfNav *f,const float position[3],float sigma)
{
    if(!f||!position||!isfinite(sigma)||sigma<AID_SIGMA_POS_MIN_M||sigma>AID_SIGMA_POS_MAX_M)return 0;
    for(int axis=0;axis<3;axis++) {
        if(!isfinite(position[axis])||fabsf(position[axis])>AID_WORLD_POS_MAX_M ||
           fabsf(position[axis]-f->position[axis])>AID_WORLD_POS_INNOV_MAX_M) return 0;
    }
    int fused=0;
    for(int axis=0;axis<3;axis++){
        const uint8_t hi[1]={(uint8_t)(IDX_P+axis)};
        const float hv[1]={1.0f};
        if(update1_sparse(f,hi,hv,1U,position[axis]-f->position[axis],sigma*sigma,9.0f,0,2U,2U))fused++;
    }
    return fused==3;
}

int eskf_nav_fuse_body_velocity(EskfNav *f,const float velocity_body[3],uint8_t axis_mask,float sigma)
{
    if(!f||!velocity_body||axis_mask==0U||!isfinite(sigma)||
       sigma<AID_SIGMA_VEL_MIN_MPS||sigma>AID_SIGMA_VEL_MAX_MPS)return 0;
    float R[3][3]; eskf_nav_rotation_matrix(f,R);
    float pred_body[3]={0.0f,0.0f,0.0f};
    for(int axis=0;axis<3;axis++) for(int w=0;w<3;w++) pred_body[axis]+=R[w][axis]*f->velocity[w];
    for(int axis=0;axis<3;axis++) if(axis_mask&(1U<<axis)) {
        if(!isfinite(velocity_body[axis])||fabsf(velocity_body[axis])>AID_WORLD_VEL_MAX_MPS ||
           fabsf(velocity_body[axis]-pred_body[axis])>AID_WHEEL_INNOV_MAX_MPS) return 0;
    }

    int requested=0,fused=0;
    for(int axis=0;axis<3;axis++){
        if((axis_mask&(1U<<axis))==0U)continue;
        requested++;
        /* Right-error q_true=q_nom⊗dq:
         * δ(R^T v) = skew(v_body) δtheta + R^T δv. */
        float sv[3][3]; skew(pred_body,sv);
        uint8_t hi[6]={IDX_TH,IDX_TH+1,IDX_TH+2,IDX_V,IDX_V+1,IDX_V+2};
        float hv[6]={sv[axis][0],sv[axis][1],sv[axis][2],
                     R[0][axis],R[1][axis],R[2][axis]};
        if(update1_sparse(f,hi,hv,6U,velocity_body[axis]-pred_body[axis],
                          sigma*sigma,9.0f,0,2U,2U))fused++;
    }
    return requested>0 && fused==requested;
}

int eskf_nav_fuse_yaw(EskfNav *f,float yaw_rad,float sigma)
{
    if(!f||!isfinite(yaw_rad)||!isfinite(sigma)||
       sigma<AID_SIGMA_YAW_MIN_RAD||sigma>AID_SIGMA_YAW_MAX_RAD)return 0;
    float current=0.0f; eskf_nav_get_euler_rad(f,0,0,&current);
    float gbody[3]; gravity_body(f,gbody);
    const uint8_t hi[3]={IDX_TH,IDX_TH+1,IDX_TH+2};
    float innov=wrap_pi(yaw_rad-current);
    if(fabsf(innov)>AID_YAW_INNOV_MAX_RAD)return 0;
    return update1_sparse(f,hi,gbody,3U,innov,sigma*sigma,AID_YAW_NIS_GATE,0,0U,0U);
}

static void reset_state_covariance_block(EskfNav *f,int start,float variance)
{
    variance=clampf_local(variance,1e-8f,1e4f);
    for(int i=0;i<3;i++){
        int idx=start+i;
        for(int j=0;j<ESKF_NAV_DIM;j++){f->P[idx][j]=0.0f;f->P[j][idx]=0.0f;}
        f->P[idx][idx]=variance;
    }
}

void eskf_nav_inflate_velocity_uncertainty(EskfNav *f,float sigma_prior)
{
    if(!f||!isfinite(sigma_prior)||sigma_prior<=0.0f)return;
    float v=sigma_prior*sigma_prior;
    for(int i=0;i<3;i++)if(f->P[IDX_V+i][IDX_V+i]<v)f->P[IDX_V+i][IDX_V+i]=v;
}

int eskf_nav_reset_world_velocity(EskfNav *f,const float velocity[3],float sigma)
{
    if(!f||!velocity||!isfinite(sigma)||sigma<AID_SIGMA_VEL_MIN_MPS||sigma>AID_SIGMA_VEL_MAX_MPS)return 0;
    for(int i=0;i<3;i++)if(!isfinite(velocity[i])||fabsf(velocity[i])>AID_WORLD_VEL_MAX_MPS)return 0;
    memcpy(f->velocity,velocity,sizeof(f->velocity));
    reset_state_covariance_block(f,IDX_V,sigma*sigma);
    return 1;
}

int eskf_nav_reset_world_position(EskfNav *f,const float position[3],float sigma)
{
    if(!f||!position||!isfinite(sigma)||sigma<AID_SIGMA_POS_MIN_M||sigma>AID_SIGMA_POS_MAX_M)return 0;
    for(int i=0;i<3;i++)if(!isfinite(position[i])||fabsf(position[i])>AID_WORLD_POS_MAX_M)return 0;
    memcpy(f->position,position,sizeof(f->position));
    reset_state_covariance_block(f,IDX_P,sigma*sigma);
    return 1;
}

int eskf_nav_reset_yaw(EskfNav *f,float yaw,float sigma)
{
    if(!f||!isfinite(yaw)||!isfinite(sigma)||sigma<AID_SIGMA_YAW_MIN_RAD||sigma>AID_SIGMA_YAW_MAX_RAD)return 0;
    float roll=0.0f,pitch=0.0f;eskf_nav_get_euler_rad(f,&roll,&pitch,0);
    euler_to_quat(roll,pitch,wrap_pi(yaw),f->q);
    /* Heading reset adalah source-initialization. Putuskan korelasi attitude lama
     * agar covariance/bias historis tidak menyeret reset heading ke state lain. */
    float old0=f->P[IDX_TH][IDX_TH],old1=f->P[IDX_TH+1][IDX_TH+1];
    reset_state_covariance_block(f,IDX_TH,sigma*sigma);
    if(old0>f->P[IDX_TH][IDX_TH])f->P[IDX_TH][IDX_TH]=old0;
    if(old1>f->P[IDX_TH+1][IDX_TH+1])f->P[IDX_TH+1][IDX_TH+1]=old1;
    return 1;
}

void eskf_nav_get_std(const EskfNav *f,float att[3],float vel[3],float pos[3])
{
    if(!f)return;
    for(int i=0;i<3;i++){
        if(att)att[i]=sqrtf(fmaxf(f->P[IDX_TH+i][IDX_TH+i],0.0f));
        if(vel)vel[i]=sqrtf(fmaxf(f->P[IDX_V+i][IDX_V+i],0.0f));
        if(pos)pos[i]=sqrtf(fmaxf(f->P[IDX_P+i][IDX_P+i],0.0f));
    }
}

int eskf_nav_nominal_is_healthy(const EskfNav *f)
{
    if(!f||!f->initialized)return 0;
    float qn=0.0f;
    for(int i=0;i<4;i++){if(!isfinite(f->q[i]))return 0;qn+=f->q[i]*f->q[i];}
    if(qn<0.8f||qn>1.2f)return 0;
    for(int i=0;i<3;i++){
        if(!isfinite(f->velocity[i])||fabsf(f->velocity[i])>ESKF_HEALTH_VELOCITY_MAX_MPS)return 0;
        if(!isfinite(f->position[i])||fabsf(f->position[i])>ESKF_HEALTH_POSITION_MAX_M)return 0;
        if(!isfinite(f->gyro_bias[i])||fabsf(f->gyro_bias[i])>ESKF_GYRO_BIAS_LIMIT_RAD*1.01f)return 0;
        if(!isfinite(f->accel_bias[i])||fabsf(f->accel_bias[i])>ESKF_ACCEL_BIAS_LIMIT_MPS2*1.01f)return 0;
    }
    return 1;
}

int eskf_nav_is_healthy(const EskfNav *f)
{
    if(!eskf_nav_nominal_is_healthy(f))return 0;
    for(int i=0;i<ESKF_NAV_DIM;i++) {
        float d=f->P[i][i];
        if(!isfinite(d) || d<1e-10f || d>1e18f) return 0;
    }
    /* P selalu ditulis simetris, jadi cukup scan upper triangle. */
    for(int r=0;r<ESKF_NAV_DIM;r++) for(int c=r+1;c<ESKF_NAV_DIM;c++) {
        float x=f->P[r][c];
        if(!isfinite(x)) return 0;
        float bound=f->P[r][r]*f->P[c][c]*1.02f + 1e-12f;
        if(x*x>bound) return 0;
    }
    return 1;
}

void eskf_nav_get_euler_rad(const EskfNav *f,float *roll,float *pitch,float *yaw) {
    float w=f->q[0],x=f->q[1],y=f->q[2],z=f->q[3];
    float sr=2*(w*x+y*z),cr=1-2*(x*x+y*y),sp=clampf_local(2*(w*y-z*x),-1,1);
    float sy=2*(w*z+x*y),cy=1-2*(y*y+z*z);
    if (roll) *roll = atan2f(sr, cr);
    if (pitch) *pitch = asinf(sp);
    if (yaw) *yaw = atan2f(sy, cy);
}
void eskf_nav_get_euler_deg(const EskfNav *f,float *r,float *p,float *y) {
    float rr,pp,yy;eskf_nav_get_euler_rad(f,&rr,&pp,&yy);float k=180.0f/PI_F;
    if (r) *r = rr * k;
    if (p) *p = pp * k;
    if (y) *y = yy * k;
}
