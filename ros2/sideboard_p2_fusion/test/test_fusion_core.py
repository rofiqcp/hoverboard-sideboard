import math

from sideboard_p2_fusion.fusion_core import P2Config, P2FusionCore, transform_planar_pose


def feed(core, seconds, raw_speed, ref_speed, steer=0.0, ref_yaw=0.0, dt=0.02, steering_valid=True):
    t = core.last_wheel_stamp_s
    out = None
    for _ in range(round(seconds / dt)):
        t += dt
        core.update_reference_velocity(t, ref_speed, ref_yaw)
        out = core.update_wheel(t, raw_speed, steer, steering_valid=steering_valid)
    return out


def test_online_wheel_scale_converges_without_absorbing_normal_slip():
    core = P2FusionCore()
    out = feed(core, 12.0, raw_speed=2.5, ref_speed=2.3)
    assert abs(core.wheel_scale - 0.92) < 0.005
    assert out.slip_probability < 0.05
    learned = core.wheel_scale

    # 40% wheel-spin must be classified as slip, not re-learned as tire scale.
    out = feed(core, 1.6, raw_speed=3.3, ref_speed=2.3)
    assert out.severe_slip
    assert out.slip_probability > 0.90
    assert out.wheel_sigma_mps == core.cfg.wheel_sigma_max_mps
    assert abs(core.wheel_scale - learned) < 0.002


def test_geometry_estimator_converges_to_wheelbase_and_steering_zero():
    truth_wheelbase = 1.20
    truth_offset = 0.04
    speed = 2.0
    core = P2FusionCore(
        P2Config(wheelbase_initial_m=1.0, steering_zero_initial_rad=0.0)
    )
    t = 0.0
    for k in range(7000):
        t += 0.02
        steer = 0.20 if (k // 100) % 2 == 0 else -0.18
        yaw_rate = speed / truth_wheelbase * math.tan(steer - truth_offset)
        core.update_reference_velocity(t, speed, yaw_rate)
        out = core.update_wheel(t, speed, steer)

    assert abs(core.steering_zero_rad - truth_offset) < 0.008
    assert abs(core.wheelbase_m - truth_wheelbase) < 0.03
    assert out.slip_probability < 0.05


def test_longitudinal_slip_probability_and_dob_velocity_are_signed():
    core = P2FusionCore()
    feed(core, 2.0, raw_speed=5.0, ref_speed=5.0)

    spin = feed(core, 1.6, raw_speed=7.0, ref_speed=5.0)
    assert spin.signed_slip_ratio > 0.25
    assert spin.slip_velocity_mps > 1.5
    assert spin.slip_probability > 0.90
    assert spin.severe_slip

    brake = P2FusionCore()
    feed(brake, 2.0, raw_speed=5.0, ref_speed=5.0)
    skid = feed(brake, 1.6, raw_speed=2.5, ref_speed=5.0)
    assert skid.signed_slip_ratio < -0.40
    assert skid.slip_velocity_mps < -2.0
    assert skid.slip_probability > 0.90


def test_acceleration_mismatch_inflates_covariance():
    core = P2FusionCore()
    out = feed(core, 1.0, raw_speed=2.0, ref_speed=2.0)
    baseline_sigma = out.wheel_sigma_mps

    t = core.last_wheel_stamp_s
    for k in range(30):
        t += 0.02
        # Reference stays steady while wheel speed ramps quickly.
        raw = 2.0 + 0.08 * k
        core.update_reference_velocity(t, 2.0, 0.0)
        out = core.update_wheel(t, raw, 0.0)

    assert out.acceleration_mismatch_mps2 > 1.0
    assert out.wheel_sigma_mps > baseline_sigma


def test_stale_reference_pauses_calibration_and_slip_comparison():
    core = P2FusionCore()
    out = feed(core, 2.0, raw_speed=2.0, ref_speed=2.0)
    count = core.scale_update_count
    t = core.last_wheel_stamp_s

    # No new reference for longer than reference_timeout_s.
    for _ in range(30):
        t += 0.02
        out = core.update_wheel(t, 3.0, 0.0)

    assert not out.reference_valid
    assert core.scale_update_count == count
    assert out.slip_probability < 0.5


def test_stale_steering_disables_yaw_evidence_and_geometry_learning():
    core = P2FusionCore()
    t = 0.0
    for _ in range(200):
        t += 0.02
        core.update_reference_velocity(t, 3.0, 0.7)
        out = core.update_wheel(t, 3.0, 0.25, steering_valid=False)

    assert not out.steering_valid
    assert out.expected_yaw_rate_rads == 0.0
    assert core.geometry_update_count == 0
    assert out.slip_probability < 0.05


def test_snapshot_restore_preserves_online_geometry_state():
    core = P2FusionCore()
    core.wheel_scale = 0.973
    core.scale_rls.theta = 0.973
    core.scale_rls.covariance = 0.012
    core.steering_zero_rad = -0.031
    core.wheelbase_m = 1.37

    restored = P2FusionCore()
    restored.restore(core.snapshot())

    assert abs(restored.wheel_scale - 0.973) < 1e-12
    assert abs(restored.scale_rls.covariance - 0.012) < 1e-12
    assert abs(restored.steering_zero_rad + 0.031) < 1e-12
    assert abs(restored.wheelbase_m - 1.37) < 1e-12


def test_high_slip_probability_freezes_scale_and_geometry_learning():
    core = P2FusionCore()
    core.slip_probability = 0.8
    core.update_reference_velocity(1.0, 2.0, 0.35)
    scale_updates = core.scale_update_count
    geom_updates = core.geometry_update_count
    scale = core.wheel_scale
    wheelbase = core.wheelbase_m
    offset = core.steering_zero_rad

    # Measurement itself is close enough to pass ratio gates, so the explicit
    # probability gate is what must freeze both online calibration paths.
    out = core.update_wheel(1.0, 2.02, 0.18)

    assert core.scale_update_count == scale_updates
    assert core.geometry_update_count == geom_updates
    assert core.wheel_scale == scale
    assert core.wheelbase_m == wheelbase
    assert core.steering_zero_rad == offset
    assert out.slip_probability > core.cfg.calibration_slip_probability_max


def test_planar_lio_body_to_base_transform():
    x, y, yaw = transform_planar_pose(
        10.0, 5.0, math.pi / 2.0,
        body_to_base_x_m=1.0,
        body_to_base_y_m=0.25,
        body_to_base_yaw_rad=0.10,
    )
    assert abs(x - 9.75) < 1e-9
    assert abs(y - 6.0) < 1e-9
    assert abs(yaw - (math.pi / 2.0 + 0.10)) < 1e-9
