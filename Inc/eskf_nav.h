#ifndef ESKF_NAV_H
#define ESKF_NAV_H

#include <stdint.h>

#define ESKF_NAV_DIM 15

typedef enum {
    ESKF_DIAG_REASON_NONE = 0,
    ESKF_DIAG_REASON_INVALID_INPUT,
    ESKF_DIAG_REASON_INNOVATION_LIMIT,
    ESKF_DIAG_REASON_NIS_GATE,
    ESKF_DIAG_REASON_NUMERICAL
} EskfNavDiagReason;

typedef struct {
    float last_wheel_innovation_mps;
    float last_wheel_nis;
    float last_yaw_innovation_rad;
    float last_yaw_nis;
    float last_velocity_innovation_norm_mps;
    float last_velocity_nis;
    float last_position_innovation_norm_m;
    float last_position_nis;

    uint32_t wheel_accept_count;
    uint32_t wheel_reject_count;
    uint32_t yaw_accept_count;
    uint32_t yaw_reject_count;
    uint32_t velocity_accept_count;
    uint32_t velocity_reject_count;
    uint32_t position_accept_count;
    uint32_t position_reject_count;
    uint32_t covariance_repair_count;
    uint32_t imu_gap_count;
    uint32_t bias_saturation_count;
    float last_imu_gap_s;
    float max_imu_gap_s;

    uint8_t covariance_psd_ok;
    uint8_t last_wheel_reason;
    uint8_t last_yaw_reason;
    uint8_t last_velocity_reason;
    uint8_t last_position_reason;
} EskfNavDiagnostics;

typedef struct {
    float q[4];                 /* Quaternion body -> world: w,x,y,z. */
    float velocity[3];          /* Kecepatan lokal world frame, m/s. */
    float position[3];          /* Posisi lokal world frame, m. */
    float gyro_bias[3];         /* Bias gyro residual, rad/s. */
    float accel_bias[3];        /* Bias accel residual, m/s^2. */
    float P[ESKF_NAV_DIM][ESKF_NAV_DIM];
    float gyro_noise;
    float accel_noise;
    float gyro_bias_walk;
    float accel_bias_walk;
    float accel_dir_noise;
    float covariance_dt_accum;
    uint32_t predict_count;
    uint8_t covariance_divider;
    uint32_t gravity_fuse_count;
    uint32_t gravity_reject_count;
    uint32_t zupt_count;
    uint32_t zero_rate_count;
    EskfNavDiagnostics diagnostics;
    uint8_t initialized;
} EskfNav;

/* State error: dtheta,dvel,dpos,dbg,dba = 15 state. */
void eskf_nav_init(EskfNav *f, const float accel_mps2[3]);
void eskf_nav_reset_motion(EskfNav *f);
void eskf_nav_reset_covariance(EskfNav *f);
int eskf_nav_nominal_is_healthy(const EskfNav *f);
int eskf_nav_predict_delta(EskfNav *f, const float delta_angle[3],
                           const float delta_velocity[3], float dt);
int eskf_nav_predict_delta_scaled(EskfNav *f, const float delta_angle[3],
                                  const float delta_velocity[3], float dt,
                                  float gyro_noise_scale, float accel_noise_scale);
int eskf_nav_correct_gravity(EskfNav *f, const float accel_mps2[3], int stationary);
int eskf_nav_fuse_zero_velocity(EskfNav *f, float sigma_mps);
int eskf_nav_fuse_zero_rate(EskfNav *f, const float gyro_rads[3], float sigma_rads);
int eskf_nav_fuse_world_velocity(EskfNav *f, const float velocity_mps[3], float sigma_mps);
int eskf_nav_fuse_world_position(EskfNav *f, const float position_m[3], float sigma_m);
int eskf_nav_fuse_body_velocity(EskfNav *f, const float velocity_body_mps[3],
                                uint8_t axis_mask, float sigma_mps);
int eskf_nav_fuse_yaw(EskfNav *f, float yaw_rad, float sigma_rad);
int eskf_nav_reset_world_velocity(EskfNav *f, const float velocity_mps[3], float sigma_mps);
int eskf_nav_reset_world_position(EskfNav *f, const float position_m[3], float sigma_m);
int eskf_nav_reset_yaw(EskfNav *f, float yaw_rad, float sigma_rad);
void eskf_nav_inflate_velocity_uncertainty(EskfNav *f, float sigma_prior_mps);
int eskf_nav_inflate_for_imu_gap(EskfNav *f, float gap_s);
void eskf_nav_get_std(const EskfNav *f, float attitude_rad[3],
                      float velocity_mps[3], float position_m[3]);
int eskf_nav_is_healthy(const EskfNav *f);
void eskf_nav_get_euler_rad(const EskfNav *f, float *roll, float *pitch, float *yaw);
void eskf_nav_get_euler_deg(const EskfNav *f, float *roll, float *pitch, float *yaw);
void eskf_nav_rotation_matrix(const EskfNav *f, float R[3][3]);
void eskf_nav_linear_accel_world(const EskfNav *f, const float accel_mps2[3], float out[3]);

/* Diagnostics do not alter filter state. covariance_psd_ok is evaluated on demand. */
void eskf_nav_get_diagnostics(const EskfNav *f, EskfNavDiagnostics *out);

/* Returns world velocity expressed in body axes: +X forward, +Y left, +Z up. */
int eskf_nav_get_body_velocity(const EskfNav *f, float velocity_body[3]);

/* Low-rate full covariance check. Tolerates only float roundoff; never repairs P. */
int eskf_nav_covariance_psd_check(const EskfNav *f);

#endif
