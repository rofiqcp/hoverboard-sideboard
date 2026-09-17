#ifndef ESKF_ATTITUDE_H
#define ESKF_ATTITUDE_H

#include <stdint.h>

typedef struct {
    float q[4];              /* Quaternion body -> world: w, x, y, z. */
    float gyro_bias[3];      /* Bias gyro yang dipelajari, satuan rad/s. */
    float P[6][6];           /* Covariance error: dtheta(3), dbias(3). */
    float gyro_noise;
    float bias_walk;
    float accel_noise;
    uint32_t predict_count;
    uint32_t accel_fuse_count;
    uint32_t accel_reject_count;
    uint8_t initialized;
} EskfAttitude;

/* Inisialisasi roll/pitch dari gravitasi; yaw awal = 0 karena tanpa referensi heading. */
void eskf_attitude_init(EskfAttitude *f, const float accel_mps2[3],
                        const float gyro_bias_rads[3]);

/* Prediksi quaternion dan covariance dari gyro. */
void eskf_attitude_predict(EskfAttitude *f, const float gyro_rads[3], float dt);

/* Koreksi gravitasi dari accelerometer. Return 1 jika data diterima, 0 jika ditolak. */
int eskf_attitude_correct_accel(EskfAttitude *f, const float accel_mps2[3]);

/* Menghasilkan roll, pitch, yaw dalam derajat. */
void eskf_attitude_get_euler_deg(const EskfAttitude *f, float *roll, float *pitch, float *yaw);

#endif
