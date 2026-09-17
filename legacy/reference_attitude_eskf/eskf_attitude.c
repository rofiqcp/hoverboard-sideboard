#include "eskf_attitude.h"
#include "app_config.h"
#include <math.h>
#include <string.h>

#define PI_F 3.14159265358979323846f

static float clampf_local(float x, float lo, float hi)
{
    return (x < lo) ? lo : ((x > hi) ? hi : x);
}

static void quat_normalize(float q[4])
{
    float n2 = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];
    if (n2 < 1.0e-12f) {
        q[0] = 1.0f; q[1] = q[2] = q[3] = 0.0f;
        return;
    }
    float inv = 1.0f / sqrtf(n2);
    for (int i = 0; i < 4; i++) q[i] *= inv;
}

static void quat_right_small_angle(float q[4], const float dtheta[3])
{
    /* Injeksi error ESKF dengan pendekatan exponential map. */
    float a = sqrtf(dtheta[0]*dtheta[0] + dtheta[1]*dtheta[1] + dtheta[2]*dtheta[2]);
    float dq[4];
    if (a < 1.0e-6f) {
        dq[0] = 1.0f;
        dq[1] = 0.5f * dtheta[0];
        dq[2] = 0.5f * dtheta[1];
        dq[3] = 0.5f * dtheta[2];
    } else {
        float s = sinf(0.5f * a) / a;
        dq[0] = cosf(0.5f * a);
        dq[1] = s * dtheta[0]; dq[2] = s * dtheta[1]; dq[3] = s * dtheta[2];
    }

    float out[4];
    out[0] = q[0]*dq[0] - q[1]*dq[1] - q[2]*dq[2] - q[3]*dq[3];
    out[1] = q[0]*dq[1] + q[1]*dq[0] + q[2]*dq[3] - q[3]*dq[2];
    out[2] = q[0]*dq[2] - q[1]*dq[3] + q[2]*dq[0] + q[3]*dq[1];
    out[3] = q[0]*dq[3] + q[1]*dq[2] - q[2]*dq[1] + q[3]*dq[0];
    memcpy(q, out, sizeof(out));
    quat_normalize(q);
}

static void euler_to_quat(float roll, float pitch, float yaw, float q[4])
{
    float cr = cosf(roll * 0.5f), sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
    float cy = cosf(yaw * 0.5f), sy = sinf(yaw * 0.5f);
    q[0] = cr*cp*cy + sr*sp*sy;
    q[1] = sr*cp*cy - cr*sp*sy;
    q[2] = cr*sp*cy + sr*cp*sy;
    q[3] = cr*cp*sy - sr*sp*cy;
    quat_normalize(q);
}

static void gravity_body(const float q[4], float h[3])
{
    /* R(body->world)^T * [0,0,1]. Ini prediksi arah gravitasi pada frame IMU. */
    float w=q[0], x=q[1], y=q[2], z=q[3];
    h[0] = 2.0f * (x*z - w*y);
    h[1] = 2.0f * (y*z + w*x);
    h[2] = 1.0f - 2.0f * (x*x + y*y);
}

static void skew3(const float v[3], float s[3][3])
{
    s[0][0]=0.0f;  s[0][1]=-v[2]; s[0][2]= v[1];
    s[1][0]=v[2];  s[1][1]=0.0f;  s[1][2]=-v[0];
    s[2][0]=-v[1]; s[2][1]=v[0];  s[2][2]=0.0f;
}

static int inv3(const float a[3][3], float inv[3][3])
{
    float det = a[0][0]*(a[1][1]*a[2][2]-a[1][2]*a[2][1])
              - a[0][1]*(a[1][0]*a[2][2]-a[1][2]*a[2][0])
              + a[0][2]*(a[1][0]*a[2][1]-a[1][1]*a[2][0]);
    if (fabsf(det) < 1.0e-12f) return 0;
    float d = 1.0f / det;
    inv[0][0]=(a[1][1]*a[2][2]-a[1][2]*a[2][1])*d;
    inv[0][1]=(a[0][2]*a[2][1]-a[0][1]*a[2][2])*d;
    inv[0][2]=(a[0][1]*a[1][2]-a[0][2]*a[1][1])*d;
    inv[1][0]=(a[1][2]*a[2][0]-a[1][0]*a[2][2])*d;
    inv[1][1]=(a[0][0]*a[2][2]-a[0][2]*a[2][0])*d;
    inv[1][2]=(a[0][2]*a[1][0]-a[0][0]*a[1][2])*d;
    inv[2][0]=(a[1][0]*a[2][1]-a[1][1]*a[2][0])*d;
    inv[2][1]=(a[0][1]*a[2][0]-a[0][0]*a[2][1])*d;
    inv[2][2]=(a[0][0]*a[1][1]-a[0][1]*a[1][0])*d;
    return 1;
}

void eskf_attitude_init(EskfAttitude *f, const float accel[3], const float bias[3])
{
    memset(f, 0, sizeof(*f));
    float ay = accel[1], az = accel[2], ax = accel[0];
    float roll = atan2f(ay, az);
    float pitch = atan2f(-ax, sqrtf(ay*ay + az*az));
    euler_to_quat(roll, pitch, 0.0f, f->q);
    for (int i=0; i<3; i++) f->gyro_bias[i] = bias ? bias[i] : 0.0f;
    f->gyro_noise = ESKF_GYRO_NOISE_RAD;
    f->bias_walk = ESKF_GYRO_BIAS_WALK_RAD;
    f->accel_noise = ESKF_ACCEL_DIR_NOISE;
    for (int i=0; i<3; i++) f->P[i][i] = 0.08f * 0.08f;
    for (int i=3; i<6; i++) f->P[i][i] = 0.03f * 0.03f;
    f->initialized = 1U;
}

void eskf_attitude_predict(EskfAttitude *f, const float gyro[3], float dt)
{
    if (!f || !f->initialized || dt <= 0.0f) return;
    dt = clampf_local(dt, 0.001f, 0.020f);

    float w[3] = {
        gyro[0] - f->gyro_bias[0],
        gyro[1] - f->gyro_bias[1],
        gyro[2] - f->gyro_bias[2]
    };
    float dtheta[3] = {w[0]*dt, w[1]*dt, w[2]*dt};
    quat_right_small_angle(f->q, dtheta);

    /* Error-state: dtheta_dot = -skew(w)dtheta - dbias - noise_gyro. */
    float sw[3][3];
    skew3(w, sw);
    float F[6][6] = {{0}};
    for (int i=0; i<6; i++) F[i][i] = 1.0f;
    for (int r=0; r<3; r++) {
        for (int c=0; c<3; c++) F[r][c] -= sw[r][c] * dt;
        F[r][r+3] = -dt;
    }

    float FP[6][6] = {{0}}, Pn[6][6] = {{0}};
    for (int r=0; r<6; r++)
        for (int c=0; c<6; c++)
            for (int k=0; k<6; k++) FP[r][c] += F[r][k] * f->P[k][c];
    for (int r=0; r<6; r++)
        for (int c=0; c<6; c++)
            for (int k=0; k<6; k++) Pn[r][c] += FP[r][k] * F[c][k];

    float q_angle = f->gyro_noise * f->gyro_noise * dt;
    float q_bias = f->bias_walk * f->bias_walk * dt;
    for (int i=0; i<3; i++) Pn[i][i] += q_angle;
    for (int i=3; i<6; i++) Pn[i][i] += q_bias;
    memcpy(f->P, Pn, sizeof(Pn));
    f->predict_count++;
}

int eskf_attitude_correct_accel(EskfAttitude *f, const float accel[3])
{
    if (!f || !f->initialized) return 0;
    float norm = sqrtf(accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2]);
    float g_ratio = norm / GRAVITY_MPS2;
    if (norm < 1.0e-4f || g_ratio < ESKF_ACCEL_GATE_MIN_G || g_ratio > ESKF_ACCEL_GATE_MAX_G) {
        f->accel_reject_count++;
        return 0;
    }

    float z[3] = {accel[0]/norm, accel[1]/norm, accel[2]/norm};
    float h[3];
    gravity_body(f->q, h);
    float innov[3] = {z[0]-h[0], z[1]-h[1], z[2]-h[2]};

    /* H = [skew(h), 0]. Bentuk ini sesuai right-error quaternion body->world. */
    float Htheta[3][3];
    skew3(h, Htheta);
    float HP[3][6] = {{0}}, PHt[6][3] = {{0}};
    for (int r=0; r<3; r++)
        for (int c=0; c<6; c++)
            for (int k=0; k<3; k++) HP[r][c] += Htheta[r][k] * f->P[k][c];
    for (int r=0; r<6; r++)
        for (int c=0; c<3; c++)
            for (int k=0; k<3; k++) PHt[r][c] += f->P[r][k] * Htheta[c][k];

    float motion = fabsf(g_ratio - 1.0f);
    float sigma = f->accel_noise * (1.0f + 8.0f * motion);
    float S[3][3] = {{0}};
    for (int r=0; r<3; r++) {
        for (int c=0; c<3; c++) {
            for (int k=0; k<6; k++) S[r][c] += HP[r][k] * ((k < 3) ? Htheta[c][k] : 0.0f);
        }
        S[r][r] += sigma * sigma;
    }

    float Sinv[3][3];
    if (!inv3(S, Sinv)) {
        f->accel_reject_count++;
        return 0;
    }

    /* Gate inovasi: menolak arah percepatan yang tidak konsisten dengan gravitasi. */
    float nis = 0.0f;
    for (int r=0; r<3; r++)
        for (int c=0; c<3; c++) nis += innov[r] * Sinv[r][c] * innov[c];
    if (nis > ESKF_ACCEL_NIS_GATE) {
        f->accel_reject_count++;
        return 0;
    }

    float K[6][3] = {{0}};
    for (int r=0; r<6; r++)
        for (int c=0; c<3; c++)
            for (int k=0; k<3; k++) K[r][c] += PHt[r][k] * Sinv[k][c];

    float dx[6] = {0};
    for (int r=0; r<6; r++)
        for (int k=0; k<3; k++) dx[r] += K[r][k] * innov[k];

    /* Gravitasi tidak mengobservasi rotasi terhadap sumbunya sendiri.
     * Hilangkan komponen tak-teramati agar accel tidak menggeser yaw/bias-yaw. */
    float along_theta = dx[0]*h[0] + dx[1]*h[1] + dx[2]*h[2];
    float along_bias = dx[3]*h[0] + dx[4]*h[1] + dx[5]*h[2];
    for (int i=0; i<3; i++) {
        dx[i] -= along_theta * h[i];
        dx[i+3] -= along_bias * h[i];
    }

    /* Cegah satu sampel buruk membuat koreksi orientasi terlalu besar. */
    float da = sqrtf(dx[0]*dx[0] + dx[1]*dx[1] + dx[2]*dx[2]);
    if (da > 0.25f) {
        float scale = 0.25f / da;
        dx[0] *= scale; dx[1] *= scale; dx[2] *= scale;
    }
    quat_right_small_angle(f->q, dx);
    for (int i=0; i<3; i++) f->gyro_bias[i] = clampf_local(f->gyro_bias[i] + dx[i+3], -0.5f, 0.5f);

    /* Joseph form: P=(I-KH)P(I-KH)^T + K R K^T.
     * Lebih stabil terhadap round-off float32 dibanding P-KHP. */
    float A[6][6] = {{0}};
    for (int r=0; r<6; r++) {
        for (int c=0; c<6; c++) {
            float kh = 0.0f;
            if (c < 3) {
                for (int k=0; k<3; k++) kh += K[r][k] * Htheta[k][c];
            }
            A[r][c] = ((r == c) ? 1.0f : 0.0f) - kh;
        }
    }

    float AP[6][6] = {{0}}, Pupd[6][6] = {{0}};
    for (int r=0; r<6; r++)
        for (int c=0; c<6; c++)
            for (int k=0; k<6; k++) AP[r][c] += A[r][k] * f->P[k][c];
    for (int r=0; r<6; r++) {
        for (int c=0; c<6; c++) {
            for (int k=0; k<6; k++) Pupd[r][c] += AP[r][k] * A[c][k];
            for (int k=0; k<3; k++) Pupd[r][c] += K[r][k] * (sigma*sigma) * K[c][k];
        }
    }

    /* Reset Jacobian ESKF setelah dtheta diinjeksi ke quaternion nominal.
     * G_theta ~= I - 0.5*skew(dtheta). */
    float sd[3][3];
    float dth[3] = {dx[0], dx[1], dx[2]};
    skew3(dth, sd);
    float G[6][6] = {{0}};
    for (int i=0; i<6; i++) G[i][i] = 1.0f;
    for (int r=0; r<3; r++)
        for (int c=0; c<3; c++) G[r][c] -= 0.5f * sd[r][c];

    float GP[6][6] = {{0}}, Preset[6][6] = {{0}};
    for (int r=0; r<6; r++)
        for (int c=0; c<6; c++)
            for (int k=0; k<6; k++) GP[r][c] += G[r][k] * Pupd[k][c];
    for (int r=0; r<6; r++)
        for (int c=0; c<6; c++)
            for (int k=0; k<6; k++) Preset[r][c] += GP[r][k] * G[c][k];

    /* Jaga covariance simetris dan diagonal positif untuk stabilitas numerik F103. */
    for (int r=0; r<6; r++) {
        for (int c=r; c<6; c++) {
            float sym = 0.5f * (Preset[r][c] + Preset[c][r]);
            f->P[r][c] = sym;
            f->P[c][r] = sym;
        }
        f->P[r][r] = clampf_local(f->P[r][r], 1.0e-9f, 10.0f);
    }
    f->accel_fuse_count++;
    return 1;
}

void eskf_attitude_get_euler_deg(const EskfAttitude *f, float *roll, float *pitch, float *yaw)
{
    float w=f->q[0], x=f->q[1], y=f->q[2], z=f->q[3];
    float sinr = 2.0f * (w*x + y*z);
    float cosr = 1.0f - 2.0f * (x*x + y*y);
    float sinp = 2.0f * (w*y - z*x);
    sinp = clampf_local(sinp, -1.0f, 1.0f);
    float siny = 2.0f * (w*z + x*y);
    float cosy = 1.0f - 2.0f * (y*y + z*z);
    const float rad_to_deg = 180.0f / PI_F;
    if (roll)  *roll  = atan2f(sinr, cosr) * rad_to_deg;
    if (pitch) *pitch = asinf(sinp) * rad_to_deg;
    if (yaw)   *yaw   = atan2f(siny, cosy) * rad_to_deg;
}
