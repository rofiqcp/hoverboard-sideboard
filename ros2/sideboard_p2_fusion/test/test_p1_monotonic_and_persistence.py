"""P1 regression: bad persistence and out-of-order ROS sensor timestamps."""
import math
from types import SimpleNamespace

import pytest
from nav_msgs.msg import Odometry

from sideboard_p2_fusion.fusion_core import P2FusionCore
from sideboard_p2_fusion.p2_fusion_node import P2FusionNode


def test_corrupted_yaml_floats_do_not_poison_calibration_or_rls():
    core = P2FusionCore()
    original = (core.wheel_scale, core.scale_rls.covariance,
                core.steering_zero_rad, core.wheelbase_m)
    core.restore({
        "wheel_scale": ".nan",
        "wheel_scale_covariance": ".inf",
        "steering_zero_rad": float("nan"),
        "wheelbase_m": "invalid",
    })
    assert (core.wheel_scale, core.scale_rls.covariance,
            core.steering_zero_rad, core.wheelbase_m) == original
    core.restore({
        "wheel_scale": .93, "wheel_scale_covariance": .01,
        "steering_zero_rad": -.025, "wheelbase_m": 1.25,
    })
    assert abs(core.wheel_scale - .93) < 1.e-12
    assert abs(core.scale_rls.covariance - .01) < 1.e-12
    assert abs(core.steering_zero_rad + .025) < 1.e-12
    assert abs(core.wheelbase_m - 1.25) < 1.e-12
    core.restore({"wheel_scale_covariance": float("nan"),
                  "steering_zero_rad": float("inf"), "wheelbase_m": float("-inf")})
    assert math.isfinite(core.scale_rls.covariance)
    assert math.isfinite(core.steering_zero_rad)
    assert math.isfinite(core.wheelbase_m)


def test_wheel_duplicate_and_backwards_timestamps_cannot_relearn_wheel_scale():
    core = P2FusionCore()
    core.update_reference_velocity(1.00, 2.0, 0.0)
    core.update_wheel(1.00, 2.0, 0.0)
    baseline = (core.wheel_scale, core.scale_update_count, core.geometry_update_count,
                core.last_wheel_stamp_s)
    with pytest.raises(ValueError, match="non-monotonic"):
        core.update_wheel(1.00, 50.0, 0.5)
    with pytest.raises(ValueError, match="non-monotonic"):
        core.update_wheel(.90, -50.0, -0.5)
    assert (core.wheel_scale, core.scale_update_count, core.geometry_update_count,
            core.last_wheel_stamp_s) == baseline
    core.update_wheel(1.02, 2.0, 0.0)


def test_lio_reference_out_of_order_is_ignored_not_used_for_velocity_derivative():
    core = P2FusionCore()
    core.update_reference_velocity(8.0, 2.0, 0.1)
    core.update_reference_velocity(7.95, -50.0, -5.0)
    core.update_reference_velocity(8.0, 200.0, 2.0)
    assert core.reference.stamp_s == 8.0
    assert core.reference.speed_mps == 2.0
    core.update_reference_velocity(8.02, 2.1, 0.1)
    assert 0.0 < core.reference.accel_mps2 < 10.0


def test_external_slip_lease_cannot_be_renewed_by_older_sample():
    core = P2FusionCore()
    core.set_external_slip_ratio(.9, True, stamp_s=5.0)
    core.set_external_slip_ratio(0.0, True, stamp_s=4.90)
    assert core.external_slip_valid
    assert core.external_slip_stamp_s == 5.0
    assert core.external_slip_ratio == .9
    core.update_wheel(5.6, 3.0, 0.0)
    assert not core.external_slip_valid


def test_lio_node_does_not_publish_out_of_order_pose():
    # Unbound method on a tiny fake: the early guard must return before it
    # touches any ROS publishers, node params, or calibration state.
    fake = SimpleNamespace(
        last_lio_pose=(10.0, 0.0, 0.0, 0.0),
        _message_time=lambda msg: 9.5,
    )
    msg = Odometry()
    msg.pose.pose.orientation.w = 1.0
    P2FusionNode._lio_cb(fake, msg)
    assert fake.last_lio_pose == (10.0, 0.0, 0.0, 0.0)
