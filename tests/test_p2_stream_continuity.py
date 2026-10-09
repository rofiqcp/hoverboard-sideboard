"""End-to-end CSV acceptance: failures in the middle cannot average away.

These are synthetic fault injections, not physical accuracy measurements.
"""
import csv
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
ANALYZER = ROOT / "tools" / "imu" / "analyze_nav_log.py"
HEALTH = {
    "diag_fresh": 1, "diag_age_ms": 20, "imu_config_ok": 1,
    "covariance_psd_ok": 1, "accel_clip_mask": 0, "gyro_clip_mask": 0,
    "i2c_error_count": 0, "imu_reinit_count": 0,
    "fifo_overflow_count": 0, "fifo_resync_count": 0,
    "scheduler_miss_count": 0, "uart_rx_overflow_count": 0,
    "uart_tx_timeout_count": 0, "filter_health_reset_count": 0,
    "imu_gap_count": 0,
}


def data():
    return [
        dict(
            host_s=round(n * .02, 5), seq=n,
            board_us=n * 20000, sensor_lag_us=2000,
            vx_mps=0.0, vy_mps=0.0, vz_mps=0.0,
            px_m=0.0, py_m=0.0, pz_m=0.0,
            lax_mps2=0.0, lay_mps2=0.0, laz_mps2=0.0,
            **HEALTH,
        ) for n in range(501)
    ]


def evaluate(rows):
    with tempfile.TemporaryDirectory() as d:
        file = Path(d) / "test.csv"
        with file.open("w", newline="") as fh:
            wr = csv.DictWriter(fh, fieldnames=list(rows[0]))
            wr.writeheader()
            wr.writerows(rows)
        return subprocess.run(
            [sys.executable, str(ANALYZER), str(file), "--stationary"],
            capture_output=True, text=True,
        )


def expect_fail(rows, key):
    result = evaluate(rows)
    assert result.returncode == 1, result.stdout + result.stderr
    assert "FAIL " + key in result.stdout, result.stdout


def test_clean_continuous_csv_passes():
    result = evaluate(data())
    assert result.returncode == 0, result.stdout + result.stderr


def test_seq_duplicate_with_same_endpoints_fails():
    rows = data()
    rows[250]["seq"] = rows[249]["seq"]
    expect_fail(rows, "sequence continuity")


def test_mcu_timestamp_rollback_fails_even_if_final_epoch_unchanged():
    rows = data()
    rows[250]["board_us"] = rows[249]["board_us"] - 500
    expect_fail(rows, "MCU epoch continuity")


def test_host_time_rollback_does_not_disappear_from_statistics():
    rows = data()
    rows[250]["host_s"] = rows[249]["host_s"] - .001
    expect_fail(rows, "host time monotonic")


def test_short_host_stall_fails_peak_even_when_p95_is_good():
    rows = data()
    for n in range(250, len(rows)):
        rows[n]["host_s"] += .30
    expect_fail(rows, "host max gap")


def test_diagnostic_blackout_fails_even_with_more_than_90pct_coverage():
    rows = data()
    for row in rows[245:276]:
        row["diag_fresh"] = 0
        row["diag_age_ms"] = 1001
    expect_fail(rows, "F5 blackout duration")


def test_counter_spike_then_reset_does_not_average_to_zero():
    rows = data()
    for row in rows[220:242]:
        row["i2c_error_count"] = 1
    expect_fail(rows, "runtime health")


def test_position_excursion_then_return_to_origin_fails():
    rows = data()
    rows[250]["px_m"] = .080
    expect_fail(rows, "position excursion")


def test_single_very_stale_sensor_frame_fails_peak_lag():
    rows = data()
    rows[250]["sensor_lag_us"] = 70000
    expect_fail(rows, "sensor lag peak")


def test_u32_rollover_is_not_a_false_reset():
    rows = data()
    for n, row in enumerate(rows):
        row["seq"] = (0xFFFFFF00 + n) & 0xffffffff
        row["board_us"] = (0xFFF00000 + n * 20000) & 0xffffffff
    result = evaluate(rows)
    assert result.returncode == 0, result.stdout + result.stderr
