#!/usr/bin/env python3
"""Regression: a CSV without actual F5 health evidence MUST NOT pass."""
import csv
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
ANALYZER=ROOT/"tools"/"imu"/"analyze_nav_log.py"
DIAG={
    "diag_fresh":1, "diag_age_ms":30, "imu_config_ok":1,
    "covariance_psd_ok":1,"accel_clip_mask":0,"gyro_clip_mask":0,
    "i2c_error_count":0,"imu_reinit_count":0,
    "fifo_overflow_count":0,"fifo_resync_count":0,
    "scheduler_miss_count":0, "uart_rx_overflow_count":0,
    "uart_tx_timeout_count":0,"filter_health_reset_count":0,
    "imu_gap_count":0,
}

def run_log(rows):
    with tempfile.TemporaryDirectory() as temp:
        p=Path(temp)/"imu.csv"
        cols=list(dict.fromkeys(k for r in rows for k in r.keys()))
        with p.open("w",newline="") as f:
            w=csv.DictWriter(f,fieldnames=cols)
            w.writeheader();w.writerows(rows)
        return subprocess.run([sys.executable,str(ANALYZER),str(p),"--stationary"],
                              capture_output=True,text=True)

def sample(n,diag=True):
    row={"host_s":n*.02,"seq":n,"board_us":n*20000,"sensor_lag_us":2000,
         "vx_mps":0,"vy_mps":0,"vz_mps":0,
         "px_m":0,"py_m":0,"pz_m":0,
         "lax_mps2":0,"lay_mps2":0,"laz_mps2":0}
    if diag: row.update(DIAG)
    return row

def test_stationary_rejects_missing_f5_entirely():
    result=run_log([sample(n,False) for n in range(101)])
    assert result.returncode==1
    assert "FAIL F5 diagnostic columns" in result.stdout

def test_stationary_accepts_valid_f5_evidence():
    result=run_log([sample(n) for n in range(101)])
    assert result.returncode==0, result.stdout+result.stderr
    assert "RESULT: PASS" in result.stdout

def test_stationary_rejects_stale_f5_on_last_frame():
    rows=[sample(n) for n in range(101)]
    for row in rows[-30:]:
        row["diag_fresh"]=0
        row["diag_age_ms"]=1500
    result=run_log(rows)
    assert result.returncode==1
    assert "FAIL F5 last frame fresh" in result.stdout

def test_stationary_rejects_nonfinite_nav_values():
    rows=[sample(n) for n in range(101)]
    rows[50]["vx_mps"]=float("nan")
    result=run_log(rows)
    assert result.returncode==1
    assert "FAIL finite navigation outputs" in result.stdout
