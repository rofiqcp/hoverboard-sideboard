from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]


def load(name):
    return yaml.safe_load((ROOT / "config" / name).read_text())


def test_local_ekf_enables_lagged_measurement_replay():
    cfg = load("ekf_local.yaml")["ekf_local"]["ros__parameters"]
    assert cfg["smooth_lagged_data"] is True
    assert cfg["history_length"] >= 1.0
    assert cfg["permit_corrected_publication"] is True
    assert cfg["odom0"] == "/p2/wheel/odometry"
    assert cfg["odom1"] == "/p2/lio/odometry"


def test_global_ekf_has_gnss_metric_anchor():
    cfg = load("ekf_global.yaml")["ekf_global"]["ros__parameters"]
    assert cfg["world_frame"] == "map"
    assert cfg["odom1"] == "/odometry/gps"
    assert cfg["odom1_config"][0] is True
    assert cfg["odom1_config"][1] is True


def test_navsat_requires_world_referenced_heading_by_default():
    cfg = load("navsat_transform.yaml")["navsat_transform"]["ros__parameters"]
    assert cfg["use_odometry_yaw"] is False
    assert cfg["wait_for_datum"] is False


def test_gnss_launch_defaults_to_absolute_enu_heading_topic():
    launch = (ROOT / "launch" / "p2_fusion.launch.py").read_text()
    assert 'default_value="/imu/enu_absolute"' in launch


def test_p2_canonicalizes_fast_lio_frames_before_local_ekf():
    cfg = load("p2_fusion.yaml")["sideboard_p2_fusion"]["ros__parameters"]
    assert cfg["lio_odom_topic"] == "/Odometry"
    assert cfg["lio_canonical_topic"] == "/p2/lio/odometry"
    assert cfg["lio_output_frame"] == "odom"
    assert cfg["base_frame"] == "base_link"
