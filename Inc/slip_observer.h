#ifndef SLIP_OBSERVER_H
#define SLIP_OBSERVER_H

#include <stdint.h>

typedef enum {
    SLIP_STATE_NORMAL = 0,
    SLIP_STATE_SUSPECT,
    SLIP_STATE_SLIP,
    SLIP_STATE_SEVERE,
    SLIP_STATE_RECOVERY
} SlipState;

typedef struct {
    float wheel_speed_mps;
    float predicted_forward_mps;
    float wheel_nis;
    float gyro_z_rads;
    float expected_yaw_rate_rads;
    uint8_t expected_yaw_rate_valid;
    float lateral_accel_mps2;
    float dt_s;
} SlipObserverInput;

typedef struct {
    SlipState state;
    float score;
    float forward_residual_ewma_mps;
    float yaw_residual_ewma_rads;
    float wheel_nis_ewma;
    float lateral_evidence;
    float forward_evidence;
    float yaw_evidence;
    float suspect_time_s;
    float slip_time_s;
    float severe_time_s;
    float stable_time_s;
    uint8_t initialized;
} SlipObserver;

void slip_observer_init(SlipObserver *observer);
void slip_observer_update(SlipObserver *observer, const SlipObserverInput *input);
SlipState slip_observer_get_state(const SlipObserver *observer);
float slip_observer_get_score(const SlipObserver *observer);
float slip_observer_wheel_sigma_scale(const SlipObserver *observer);
float slip_observer_nhc_sigma_scale(const SlipObserver *observer);
int slip_observer_reject_wheel(const SlipObserver *observer);

#endif
