#ifndef ESKF_NAV_H
#define ESKF_NAV_H

#include <stdint.h>

#define ESKF_NAV_DIM 15

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
    uint8_t initialized;
} EskfNav;

/* State error: dtheta,dvel,dpos,dbg,dba = 15 state. */
void eskf_nav_init(EskfNav *f, const float accel_mps2[3]);
void eskf_nav_reset_motion(EskfNav *f);
void eskf_nav_reset_covariance(EskfNav *f);
int eskf_nav_nominal_is_healthy(const EskfNav *f);
void eskf_nav_predict(EskfNav *f, const float gyro_rads[3],
                      const float accel_mps2[3], float dt);
int eskf_nav_predict_delta(EskfNav *f, const float delta_angle[3],
                           const float delta_velocity[3], float dt);
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
void eskf_nav_get_std(const EskfNav *f, float attitude_rad[3],
                      float velocity_mps[3], float position_m[3]);
int eskf_nav_is_healthy(const EskfNav *f);
void eskf_nav_get_euler_rad(const EskfNav *f, float *roll, float *pitch, float *yaw);
void eskf_nav_get_euler_deg(const EskfNav *f, float *roll, float *pitch, float *yaw);
void eskf_nav_rotation_matrix(const EskfNav *f, float R[3][3]);
void eskf_nav_linear_accel_world(const EskfNav *f, const float accel_mps2[3], float out[3]);

#endif
