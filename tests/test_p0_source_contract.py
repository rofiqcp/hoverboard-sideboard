"""Source-level safety invariants for the MCU HAL integration.
Not a substitute for real-board lease expiration and hard-deadline tests.
"""
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]

def test_master_stationary_has_expiring_permission_and_fails_closed():
    cfg=(ROOT/"Inc/app_config.h").read_text()
    source=(ROOT/"Src/main.c").read_text()
    assert "#define MASTER_STATIONARY_LEASE_US" in cfg
    assert "now_us-master_stationary_since_us" in source
    assert "master_stationary=0U" in source
    assert "startup_zupt=0U" in source
    assert "safe=sample_is_still && v2<0.0225f" in source
    assert "CAL_CMD_STATIONARY_ON, safe?0U:3U" in source

def test_gravity_correction_requires_sensor_confirmed_rest():
    source=(ROOT/"Src/main.c").read_text()
    assert "imu_health->last_accel_clip_mask==0U) && sample_is_still" in source
    assert "gravity_stationary = sample_is_still || master_stationary" not in source

def test_imu_both_profiles_enforce_fifo_capacity():
    cfg=(ROOT/"Inc/app_config.h").read_text()
    assert "IMU_FIFO_BATCH_CAPACITY < (MPU6XXX_SAMPLE_HZ / IMU_LOOP_RATE_HZ)" in cfg
    assert "#define IMU_LOOP_RATE_HZ            200UL" in cfg
    assert "#define IMU_LOOP_RATE_HZ            100UL" in cfg
