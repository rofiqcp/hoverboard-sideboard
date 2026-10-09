"""Long replay P2: bounded estimator state under dropouts, slip, and stale ROS frames."""
import math
import random
from sideboard_p2_fusion.fusion_core import P2FusionCore

def test_p2_400_second_deterministic_fault_replay():
    rng = random.Random(20261008)
    core = P2FusionCore()
    period = 0.02
    severe_hits = 0
    stale_hits = 0
    # 20,000 samples = 400 s at 50 Hz.
    for k in range(20000):
        t = 10.0 + k * period
        truth = 2.0 + .25 * math.sin(.013 * k)
        raw = truth / .97 + rng.gauss(0.0, .012)
        dropout = (2500 <= k < 3000) or (12000 <= k < 12400)
        slip = (4500 <= k < 4600) or (13300 <= k < 13400)
        if slip:
            raw += 1.5
        if not dropout:
            core.update_reference_velocity(t, truth, 0.0)
            if k % 97 == 0:
                # Old ROS delivery must not rewrite reference or produce
                # a huge velocity derivative on the next good sample.
                core.update_reference_velocity(t - .05, -40.0, 2.0)
        out = core.update_wheel(t, raw, 0.0, steering_valid=True)
        if dropout and k % 10 == 0 and k > 2520 and k < 2950:
            stale_hits += 1
            assert not out.reference_valid
            assert out.wheel_sigma_mps >= core.cfg.stale_reference_sigma_floor_mps
        if slip and out.reference_valid and out.severe_slip:
            severe_hits += 1
            assert out.wheel_sigma_mps == core.cfg.wheel_sigma_max_mps

        for v in (out.wheel_scale, out.wheelbase_m, out.steering_zero_rad,
                  out.wheel_sigma_mps, out.slip_probability, out.slip_velocity_mps,
                  core.scale_rls.covariance):
            assert math.isfinite(v)
        assert 0 <= out.slip_probability <= 1
        assert core.cfg.wheel_scale_min <= core.wheel_scale <= core.cfg.wheel_scale_max
        assert core.cfg.wheelbase_min_m <= core.wheelbase_m <= core.cfg.wheelbase_max_m

    assert severe_hits >= 50
    assert stale_hits >= 30
    assert abs(core.wheel_scale - .97) < .07
