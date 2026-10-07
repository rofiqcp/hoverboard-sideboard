"""Pure-math P2 fusion core.

This module is intentionally ROS-free so estimator/calibration behavior can be
regression-tested on the host. It performs:
- online wheel speed scale estimation;
- online steering zero/wheelbase calibration;
- wheel acceleration consistency checks;
- adaptive wheel covariance from slip probability;
- a compact disturbance-observer-inspired slip velocity estimate.
"""
from __future__ import annotations

from dataclasses import dataclass, asdict
import math
from typing import Optional, Dict, Any


def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def ewma(previous: float, sample: float, dt: float, tau: float) -> float:
    if not math.isfinite(sample):
        return previous
    if tau <= 0.0:
        return sample
    alpha = clamp(dt / (tau + dt), 0.0, 1.0)
    return previous + alpha * (sample - previous)


def transform_planar_pose(
    x_body_m: float,
    y_body_m: float,
    yaw_body_rad: float,
    body_to_base_x_m: float,
    body_to_base_y_m: float,
    body_to_base_yaw_rad: float,
):
    """Compose world->LIO-body with a planar LIO-body->base transform."""
    c = math.cos(yaw_body_rad)
    s = math.sin(yaw_body_rad)
    x_base = x_body_m + c * body_to_base_x_m - s * body_to_base_y_m
    y_base = y_body_m + s * body_to_base_x_m + c * body_to_base_y_m
    yaw_base = wrap_pi(yaw_body_rad + body_to_base_yaw_rad)
    return x_base, y_base, yaw_base


@dataclass
class P2Config:
    wheel_scale_initial: float = 1.0
    wheel_scale_min: float = 0.75
    wheel_scale_max: float = 1.25
    wheel_scale_forgetting: float = 0.999
    wheel_scale_cov_initial: float = 25.0

    wheelbase_initial_m: float = 1.0
    wheelbase_min_m: float = 0.35
    wheelbase_max_m: float = 4.0
    steering_zero_initial_rad: float = 0.0
    steering_zero_limit_rad: float = 0.25
    steering_offset_learning_rate: float = 0.0015
    wheelbase_learning_rate: float = 0.006

    reference_timeout_s: float = 0.20
    scale_min_speed_mps: float = 0.60
    scale_max_yaw_rate_rads: float = 0.35
    geometry_min_speed_mps: float = 0.80
    geometry_min_steer_rad: float = 0.035
    geometry_max_steer_rad: float = 0.70
    calibration_slip_probability_max: float = 0.35
    calibration_accel_abs_max_mps2: float = 3.5
    scale_calibration_ratio_max: float = 0.12
    geometry_speed_ratio_max: float = 0.08

    speed_ratio_deadband: float = 0.035
    speed_ratio_full: float = 0.20
    accel_mismatch_deadband_mps2: float = 0.75
    accel_mismatch_full_mps2: float = 4.0
    yaw_mismatch_deadband_rads: float = 0.08
    yaw_mismatch_full_rads: float = 0.65
    external_slip_ratio_full: float = 0.20
    slip_probability_tau_s: float = 0.20

    dob_bandwidth_hz: float = 3.0
    dob_max_slip_velocity_mps: float = 8.0
    wheel_sigma_base_mps: float = 0.05
    wheel_sigma_max_mps: float = 5.0
    wheel_sigma_slip_gain: float = 25.0
    wheel_sigma_dob_gain: float = 0.40
    severe_slip_probability: float = 0.92

    derivative_min_dt_s: float = 0.005
    derivative_max_dt_s: float = 0.25


@dataclass
class ReferenceState:
    stamp_s: float = 0.0
    speed_mps: float = 0.0
    yaw_rate_rads: float = 0.0
    accel_mps2: float = 0.0
    valid: bool = False


@dataclass
class FusionOutput:
    corrected_wheel_speed_mps: float
    expected_yaw_rate_rads: float
    wheel_sigma_mps: float
    slip_probability: float
    slip_velocity_mps: float
    signed_slip_ratio: float
    speed_residual_mps: float
    acceleration_mismatch_mps2: float
    yaw_mismatch_rads: float
    wheel_scale: float
    steering_zero_rad: float
    wheelbase_m: float
    reference_valid: bool
    severe_slip: bool
    calibration_updated: bool
    steering_valid: bool


class ScalarRLS:
    """Bounded scalar recursive least-squares for y = theta*x."""

    def __init__(self, theta: float, covariance: float, forgetting: float):
        self.theta = float(theta)
        self.covariance = max(float(covariance), 1.0e-6)
        self.forgetting = clamp(float(forgetting), 0.90, 1.0)

    def update(self, x: float, y: float) -> float:
        if not (math.isfinite(x) and math.isfinite(y)) or abs(x) < 1.0e-5:
            return self.theta
        denom = self.forgetting + x * x * self.covariance
        if denom <= 1.0e-12:
            return self.theta
        gain = self.covariance * x / denom
        self.theta += gain * (y - x * self.theta)
        self.covariance = max(
            (self.covariance - gain * x * self.covariance) / self.forgetting,
            1.0e-7,
        )
        return self.theta


class P2FusionCore:
    def __init__(self, config: Optional[P2Config] = None):
        self.cfg = config or P2Config()
        self.scale_rls = ScalarRLS(
            self.cfg.wheel_scale_initial,
            self.cfg.wheel_scale_cov_initial,
            self.cfg.wheel_scale_forgetting,
        )
        self.wheel_scale = clamp(
            self.cfg.wheel_scale_initial,
            self.cfg.wheel_scale_min,
            self.cfg.wheel_scale_max,
        )
        self.steering_zero_rad = clamp(
            self.cfg.steering_zero_initial_rad,
            -self.cfg.steering_zero_limit_rad,
            self.cfg.steering_zero_limit_rad,
        )
        self.wheelbase_m = clamp(
            self.cfg.wheelbase_initial_m,
            self.cfg.wheelbase_min_m,
            self.cfg.wheelbase_max_m,
        )

        self.reference = ReferenceState()
        self.slip_probability = 0.0
        self.slip_velocity_mps = 0.0
        self.external_slip_ratio = 0.0
        self.external_slip_valid = False

        self.last_wheel_stamp_s = 0.0
        self.last_corrected_wheel_mps = 0.0
        self.last_reference_speed_mps = 0.0
        self.last_reference_stamp_s = 0.0
        self.last_wheel_accel_mps2 = 0.0
        self.last_ref_accel_mps2 = 0.0

        self.scale_update_count = 0
        self.geometry_update_count = 0

    def restore(self, state: Dict[str, Any]) -> None:
        if not isinstance(state, dict):
            return
        if "wheel_scale" in state and math.isfinite(float(state["wheel_scale"])):
            self.wheel_scale = clamp(
                float(state["wheel_scale"]),
                self.cfg.wheel_scale_min,
                self.cfg.wheel_scale_max,
            )
            self.scale_rls.theta = self.wheel_scale
        if "wheel_scale_covariance" in state:
            try:
                self.scale_rls.covariance = clamp(
                    float(state["wheel_scale_covariance"]), 1.0e-7, 1.0e6
                )
            except (TypeError, ValueError):
                pass
        if "steering_zero_rad" in state:
            try:
                self.steering_zero_rad = clamp(
                    float(state["steering_zero_rad"]),
                    -self.cfg.steering_zero_limit_rad,
                    self.cfg.steering_zero_limit_rad,
                )
            except (TypeError, ValueError):
                pass
        if "wheelbase_m" in state:
            try:
                self.wheelbase_m = clamp(
                    float(state["wheelbase_m"]),
                    self.cfg.wheelbase_min_m,
                    self.cfg.wheelbase_max_m,
                )
            except (TypeError, ValueError):
                pass

    def snapshot(self) -> Dict[str, Any]:
        return {
            "wheel_scale": self.wheel_scale,
            "wheel_scale_covariance": self.scale_rls.covariance,
            "steering_zero_rad": self.steering_zero_rad,
            "wheelbase_m": self.wheelbase_m,
            "scale_update_count": self.scale_update_count,
            "geometry_update_count": self.geometry_update_count,
            "config": asdict(self.cfg),
        }

    def set_external_slip_ratio(self, ratio: float, valid: bool = True) -> None:
        if valid and math.isfinite(ratio):
            self.external_slip_ratio = clamp(ratio, -2.0, 2.0)
            self.external_slip_valid = True
        else:
            self.external_slip_valid = False

    def update_reference_velocity(
        self, stamp_s: float, speed_mps: float, yaw_rate_rads: float
    ) -> None:
        if not all(math.isfinite(v) for v in (stamp_s, speed_mps, yaw_rate_rads)):
            return
        accel = 0.0
        if self.reference.valid:
            dt = stamp_s - self.reference.stamp_s
            if self.cfg.derivative_min_dt_s <= dt <= self.cfg.derivative_max_dt_s:
                raw_accel = (speed_mps - self.reference.speed_mps) / dt
                accel = ewma(self.reference.accel_mps2, raw_accel, dt, 0.08)
            else:
                accel = self.reference.accel_mps2
        self.reference = ReferenceState(
            stamp_s=stamp_s,
            speed_mps=speed_mps,
            yaw_rate_rads=yaw_rate_rads,
            accel_mps2=accel,
            valid=True,
        )

    def _reference_is_fresh(self, stamp_s: float) -> bool:
        return (
            self.reference.valid
            and stamp_s >= self.reference.stamp_s
            and (stamp_s - self.reference.stamp_s) <= self.cfg.reference_timeout_s
        )

    def _update_scale(
        self,
        raw_wheel_speed_mps: float,
        corrected_speed_mps: float,
        wheel_accel_mps2: float,
    ) -> bool:
        ref = self.reference
        if abs(ref.speed_mps) < self.cfg.scale_min_speed_mps:
            return False
        if abs(raw_wheel_speed_mps) < self.cfg.scale_min_speed_mps:
            return False
        if abs(ref.yaw_rate_rads) > self.cfg.scale_max_yaw_rate_rads:
            return False
        if abs(wheel_accel_mps2) > self.cfg.calibration_accel_abs_max_mps2:
            return False
        if abs(ref.accel_mps2) > self.cfg.calibration_accel_abs_max_mps2:
            return False
        if self.slip_probability > self.cfg.calibration_slip_probability_max:
            return False
        scale_ratio = abs(corrected_speed_mps - ref.speed_mps) / max(
            abs(corrected_speed_mps), abs(ref.speed_mps), self.cfg.scale_min_speed_mps
        )
        if scale_ratio > self.cfg.scale_calibration_ratio_max:
            return False
        if self.external_slip_valid and abs(self.external_slip_ratio) > 0.08:
            return False

        previous = self.wheel_scale
        theta = self.scale_rls.update(raw_wheel_speed_mps, ref.speed_mps)
        self.scale_rls.theta = clamp(
            theta, self.cfg.wheel_scale_min, self.cfg.wheel_scale_max
        )
        self.wheel_scale = self.scale_rls.theta
        if abs(self.wheel_scale - previous) > 1.0e-9:
            self.scale_update_count += 1
            return True
        return False

    def _update_geometry(
        self,
        corrected_speed_mps: float,
        steering_angle_rad: float,
        wheel_accel_mps2: float,
    ) -> bool:
        ref = self.reference
        delta = steering_angle_rad - self.steering_zero_rad
        if abs(corrected_speed_mps) < self.cfg.geometry_min_speed_mps:
            return False
        if not (
            self.cfg.geometry_min_steer_rad
            <= abs(delta)
            <= self.cfg.geometry_max_steer_rad
        ):
            return False
        if self.slip_probability > self.cfg.calibration_slip_probability_max:
            return False
        speed_ratio = abs(corrected_speed_mps - ref.speed_mps) / max(
            abs(corrected_speed_mps), abs(ref.speed_mps), self.cfg.scale_min_speed_mps
        )
        if speed_ratio > self.cfg.geometry_speed_ratio_max:
            return False
        if abs(wheel_accel_mps2) > self.cfg.calibration_accel_abs_max_mps2:
            return False
        if abs(ref.accel_mps2) > self.cfg.calibration_accel_abs_max_mps2:
            return False
        if self.external_slip_valid and abs(self.external_slip_ratio) > 0.08:
            return False

        cos_delta = math.cos(delta)
        if abs(cos_delta) < 0.25:
            return False

        pred = corrected_speed_mps / self.wheelbase_m * math.tan(delta)
        error = clamp(ref.yaw_rate_rads - pred, -0.50, 0.50)
        sec2 = 1.0 / (cos_delta * cos_delta)

        d_pred_d_offset = -corrected_speed_mps / self.wheelbase_m * sec2
        d_pred_d_wheelbase = (
            -corrected_speed_mps
            * math.tan(delta)
            / (self.wheelbase_m * self.wheelbase_m)
        )

        old_offset = self.steering_zero_rad
        old_wheelbase = self.wheelbase_m

        self.steering_zero_rad += (
            self.cfg.steering_offset_learning_rate * error * d_pred_d_offset
        )
        self.wheelbase_m += (
            self.cfg.wheelbase_learning_rate * error * d_pred_d_wheelbase
        )
        self.steering_zero_rad = clamp(
            self.steering_zero_rad,
            -self.cfg.steering_zero_limit_rad,
            self.cfg.steering_zero_limit_rad,
        )
        self.wheelbase_m = clamp(
            self.wheelbase_m,
            self.cfg.wheelbase_min_m,
            self.cfg.wheelbase_max_m,
        )

        changed = (
            abs(self.steering_zero_rad - old_offset) > 1.0e-10
            or abs(self.wheelbase_m - old_wheelbase) > 1.0e-10
        )
        if changed:
            self.geometry_update_count += 1
        return changed

    def _expected_yaw_rate(
        self, corrected_speed_mps: float, steering_angle_rad: float
    ) -> float:
        delta = clamp(
            steering_angle_rad - self.steering_zero_rad,
            -self.cfg.geometry_max_steer_rad,
            self.cfg.geometry_max_steer_rad,
        )
        return corrected_speed_mps / max(self.wheelbase_m, 1.0e-3) * math.tan(delta)

    def update_wheel(
        self,
        stamp_s: float,
        raw_wheel_speed_mps: float,
        steering_angle_rad: float,
        steering_valid: bool = True,
    ) -> FusionOutput:
        if not all(
            math.isfinite(v)
            for v in (stamp_s, raw_wheel_speed_mps, steering_angle_rad)
        ):
            raise ValueError("non-finite P2 wheel input")

        corrected = raw_wheel_speed_mps * self.wheel_scale
        dt = 0.02
        wheel_accel = self.last_wheel_accel_mps2
        if self.last_wheel_stamp_s > 0.0:
            candidate_dt = stamp_s - self.last_wheel_stamp_s
            if self.cfg.derivative_min_dt_s <= candidate_dt <= self.cfg.derivative_max_dt_s:
                dt = candidate_dt
                raw_accel = (
                    corrected - self.last_corrected_wheel_mps
                ) / candidate_dt
                wheel_accel = ewma(
                    self.last_wheel_accel_mps2, raw_accel, candidate_dt, 0.06
                )

        fresh = self._reference_is_fresh(stamp_s)
        expected_yaw = (
            self._expected_yaw_rate(corrected, steering_angle_rad)
            if steering_valid
            else 0.0
        )

        speed_residual = 0.0
        signed_ratio = 0.0
        accel_mismatch = 0.0
        yaw_mismatch = 0.0
        raw_probability = 0.0

        if fresh:
            ref = self.reference
            speed_residual = corrected - ref.speed_mps
            speed_den = max(
                abs(corrected), abs(ref.speed_mps), self.cfg.scale_min_speed_mps
            )
            signed_ratio = speed_residual / speed_den

            ratio_evidence = clamp(
                (abs(signed_ratio) - self.cfg.speed_ratio_deadband)
                / max(
                    self.cfg.speed_ratio_full - self.cfg.speed_ratio_deadband,
                    1.0e-6,
                ),
                0.0,
                1.0,
            )

            accel_mismatch = wheel_accel - ref.accel_mps2
            accel_evidence = clamp(
                (
                    abs(accel_mismatch)
                    - self.cfg.accel_mismatch_deadband_mps2
                )
                / max(
                    self.cfg.accel_mismatch_full_mps2
                    - self.cfg.accel_mismatch_deadband_mps2,
                    1.0e-6,
                ),
                0.0,
                1.0,
            )

            yaw_evidence = 0.0
            if steering_valid:
                yaw_mismatch = expected_yaw - ref.yaw_rate_rads
                yaw_evidence = clamp(
                    (abs(yaw_mismatch) - self.cfg.yaw_mismatch_deadband_rads)
                    / max(
                        self.cfg.yaw_mismatch_full_rads
                        - self.cfg.yaw_mismatch_deadband_rads,
                        1.0e-6,
                    ),
                    0.0,
                    1.0,
                )

            ext_evidence = 0.0
            if self.external_slip_valid:
                ext_evidence = clamp(
                    abs(self.external_slip_ratio)
                    / max(self.cfg.external_slip_ratio_full, 1.0e-6),
                    0.0,
                    1.0,
                )

            # Any one strong physical inconsistency must be able to reach 1.0.
            # This prevents pure longitudinal wheel-spin from being capped below
            # the severe threshold merely because yaw/accel corroboration is quiet.
            raw_probability = max(
                ratio_evidence,
                0.80 * accel_evidence,
                0.80 * yaw_evidence,
                ext_evidence,
            )
            raw_probability = clamp(raw_probability, 0.0, 1.0)

            self.slip_probability = ewma(
                self.slip_probability,
                raw_probability,
                dt,
                self.cfg.slip_probability_tau_s,
            )

            # Disturbance-observer-inspired residual state.
            tau_dob = 1.0 / max(2.0 * math.pi * self.cfg.dob_bandwidth_hz, 1.0e-3)
            self.slip_velocity_mps = ewma(
                self.slip_velocity_mps, speed_residual, dt, tau_dob
            )
            self.slip_velocity_mps = clamp(
                self.slip_velocity_mps,
                -self.cfg.dob_max_slip_velocity_mps,
                self.cfg.dob_max_slip_velocity_mps,
            )
        else:
            self.slip_probability = ewma(
                self.slip_probability, 0.0, dt, 0.60
            )
            self.slip_velocity_mps = ewma(
                self.slip_velocity_mps, 0.0, dt, 0.60
            )

        calibration_updated = False
        if fresh:
            if self._update_scale(
                raw_wheel_speed_mps, corrected, wheel_accel
            ):
                corrected = raw_wheel_speed_mps * self.wheel_scale
                expected_yaw = self._expected_yaw_rate(
                    corrected, steering_angle_rad
                )
                calibration_updated = True
            if steering_valid and self._update_geometry(
                corrected, steering_angle_rad, wheel_accel
            ):
                expected_yaw = self._expected_yaw_rate(
                    corrected, steering_angle_rad
                )
                calibration_updated = True

        sigma = self.cfg.wheel_sigma_base_mps * (
            1.0 + self.cfg.wheel_sigma_slip_gain * self.slip_probability ** 2
        )
        sigma += self.cfg.wheel_sigma_dob_gain * abs(self.slip_velocity_mps)
        sigma = clamp(
            sigma,
            self.cfg.wheel_sigma_base_mps,
            self.cfg.wheel_sigma_max_mps,
        )

        severe = self.slip_probability >= self.cfg.severe_slip_probability
        if severe:
            sigma = self.cfg.wheel_sigma_max_mps

        self.last_wheel_stamp_s = stamp_s
        self.last_corrected_wheel_mps = corrected
        self.last_wheel_accel_mps2 = wheel_accel

        return FusionOutput(
            corrected_wheel_speed_mps=corrected,
            expected_yaw_rate_rads=expected_yaw,
            wheel_sigma_mps=sigma,
            slip_probability=self.slip_probability,
            slip_velocity_mps=self.slip_velocity_mps,
            signed_slip_ratio=signed_ratio,
            speed_residual_mps=speed_residual,
            acceleration_mismatch_mps2=accel_mismatch,
            yaw_mismatch_rads=yaw_mismatch,
            wheel_scale=self.wheel_scale,
            steering_zero_rad=self.steering_zero_rad,
            wheelbase_m=self.wheelbase_m,
            reference_valid=fresh,
            severe_slip=severe,
            calibration_updated=calibration_updated,
            steering_valid=steering_valid,
        )
