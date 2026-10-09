#!/usr/bin/env python3
"""Stationary hardware gate for STM32 sideboard IMU/ESKF telemetry.

Run with the board/vehicle completely still. The script validates the real
serial path, timing, ESKF zero-velocity behavior, short-term XYZ drift, and
runtime health without ROS or any external aiding.
"""
from __future__ import annotations

import argparse
import math
import statistics
import sys
import time

import serial

from read_imu import (
    decode_diag,
    decode_imu,
    open_telemetry_port,
    read_frame,
)


def pctl(values, q):
    if not values:
        return float("nan")
    s = sorted(values)
    if len(s) == 1:
        return float(s[0])
    x = (len(s) - 1) * q
    lo = int(math.floor(x))
    hi = int(math.ceil(x))
    if lo == hi:
        return float(s[lo])
    a = x - lo
    return float(s[lo] * (1.0 - a) + s[hi] * a)


def norm3(v):
    return math.sqrt(sum(float(x) * float(x) for x in v))


def delta_u32(a, b):
    return (int(a) - int(b)) & 0xFFFFFFFF


def main():
    ap = argparse.ArgumentParser(
        description="Validasi stationary STM32+IMU+ESKF+Python tanpa ROS"
    )
    ap.add_argument("--port", default="auto")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--duration", type=float, default=60.0)
    ap.add_argument("--min-stream-hz", type=float, default=48.0)
    ap.add_argument("--min-sensor-hz", type=float, default=475.0)
    ap.add_argument("--max-sensor-hz", type=float, default=525.0)
    ap.add_argument("--max-host-interval-p95-ms", type=float, default=30.0)
    ap.add_argument("--max-host-gap-ms", type=float, default=250.0)
    ap.add_argument("--max-board-gap-ms", type=float, default=150.0)
    ap.add_argument("--max-lag-peak-us", type=float, default=25000.0)
    ap.add_argument("--max-lost", type=int, default=0)
    ap.add_argument("--min-diag-hz", type=float, default=3.0)
    ap.add_argument("--max-diag-gap-ms", type=float, default=750.0)
    ap.add_argument("--max-lag-p95-us", type=float, default=8000.0)
    ap.add_argument("--max-speed-mps", type=float, default=0.020)
    ap.add_argument("--max-pos-drift-m", type=float, default=0.050)
    ap.add_argument("--max-rp-span-deg", type=float, default=1.0)
    ap.add_argument("--max-yaw-drift-deg", type=float, default=3.0)
    args = ap.parse_args()

    print("STATIONARY TEST: board/vehicle harus benar-benar diam selama pengujian.")
    port = open_telemetry_port(
        args.port, args.baud, timeout=0.04, attempts=10, delay=0.20
    )
    print(f"Port terverifikasi: {port.port} @ {args.baud}")

    frames = []
    arrival_times = []
    lags = []
    first_diag = last_diag = None
    diag_history = []
    last_diag_arrival = None
    diag_frames = 0
    diag_arrivals = []
    crc_errors = other_errors = 0
    start = time.monotonic()
    deadline = start + max(args.duration, 1.0)

    try:
        while time.monotonic() < deadline:
            payload, err = read_frame(port)
            if err:
                if err == "crc":
                    crc_errors += 1
                elif err != "timeout":
                    other_errors += 1
                continue
            d = decode_diag(payload)
            if d is not None:
                if first_diag is None:
                    first_diag = d.copy()
                last_diag = d.copy()
                diag_history.append(d.copy())
                last_diag_arrival = time.monotonic()
                diag_arrivals.append(last_diag_arrival)
                diag_frames += 1
                continue
            d = decode_imu(payload)
            if d is None:
                continue
            frames.append(d)
            arrival_times.append(time.monotonic())
            if d.get("version", 0) >= 6:
                lag = d.get("sensor_lag_us")
                if lag is not None and abs(lag) < 1_000_000:
                    lags.append(float(lag))
    except (serial.SerialException, OSError) as exc:
        print(f"FAIL: serial putus: {exc}", file=sys.stderr)
        return 2
    finally:
        try:
            port.close()
        except Exception:
            pass

    if len(frames) < 2:
        print("FAIL: telemetry F0 tidak cukup untuk dianalisis.", file=sys.stderr)
        return 2

    first = frames[0]
    last = frames[-1]
    seq_span = delta_u32(last["seq"], first["seq"])
    time_span_us = delta_u32(last["time_us"], first["time_us"])
    stream_hz = seq_span / (time_span_us * 1e-6) if time_span_us else 0.0
    lost = max(0, seq_span - (len(frames) - 1))
    seq_steps=[delta_u32(b["seq"],a["seq"]) for a,b in zip(frames,frames[1:])]
    board_steps=[delta_u32(b["time_us"],a["time_us"]) for a,b in zip(frames,frames[1:])]
    seq_bad=sum(d!=1 for d in seq_steps)
    board_bad=sum(d==0 or d>args.max_board_gap_ms*1000.0 for d in board_steps)
    host_steps=[(b-a)*1000.0 for a,b in zip(arrival_times,arrival_times[1:])]
    host_bad=sum(d<=0 for d in host_steps)

    speeds = [norm3(f["vel"]) for f in frames]
    pos0 = first["pos"]
    pos1 = last["pos"]
    pos_d = tuple(pos1[i] - pos0[i] for i in range(3))
    pos_drift = norm3(pos_d)

    roll = [f["roll"] for f in frames]
    pitch = [f["pitch"] for f in frames]
    yaw0 = first["yaw"]
    yaw_drift = last["yaw"] - yaw0
    while yaw_drift > 180.0:
        yaw_drift -= 360.0
    while yaw_drift < -180.0:
        yaw_drift += 360.0
    roll_span = max(roll) - min(roll)
    pitch_span = max(pitch) - min(pitch)

    lin_rms = []
    for axis in range(3):
        vals = [f["linacc"][axis] for f in frames]
        lin_rms.append(math.sqrt(sum(v * v for v in vals) / len(vals)))

    lag_abs = [abs(x) for x in lags]
    lag_avg = statistics.fmean(lags) if lags else float("nan")
    lag_p95 = pctl(lag_abs, 0.95)
    lag_max = max(lag_abs) if lags else float("nan")
    host_intervals_ms = [
        (arrival_times[i] - arrival_times[i-1]) * 1000.0
        for i in range(1, len(arrival_times))
        if arrival_times[i] >= arrival_times[i-1]
    ]
    host_interval_p95 = pctl(host_intervals_ms, 0.95)

    health_reset_delta = max(
        0, int(last.get("health_resets", 0)) - int(first.get("health_resets", 0))
    )

    diag_delta = {}
    diag_changed = {}
    diag_bad_frames = 0
    if first_diag is not None and last_diag is not None:
        diag_bad_frames=sum(
            int(d.get("version",0)<2 or not d["imu_config_ok"] or
                not d["covariance_psd_ok"] or d["accel_clip_mask"]!=0 or
                d["gyro_clip_mask"]!=0)
            for d in diag_history
        )
        for key in (
            "accel_clip_count",
            "gyro_clip_count",
            "imu_config_mismatch_count",
            "i2c_error_count",
            "imu_reinit_count",
            "fifo_overflow_count",
            "fifo_resync_count",
            "scheduler_miss_count",
            "uart_rx_overflow_count",
            "uart_tx_timeout_count",
            "filter_health_reset_count",
            "imu_gap_count",
        ):
            diag_delta[key] = delta_u32(last_diag[key], first_diag[key])
            diag_changed[key] = sum(
                delta_u32(b[key],a[key]) != 0
                for a,b in zip(diag_history,diag_history[1:])
            )

    checks = []
    observed_duration=max(arrival_times[-1] - arrival_times[0], 1.0)
    diag_rate=diag_frames/observed_duration
    diag_gaps_ms=[(b-a)*1000.0 for a,b in zip(diag_arrivals,diag_arrivals[1:])]
    max_diag_gap=max(diag_gaps_ms,default=float("inf"))
    checks.append(("diagnostic F5 present (>=2)", diag_frames >= 2, str(diag_frames)))
    checks.append(("diagnostic F5 rate",diag_rate>=args.min_diag_hz,
                   f"{diag_rate:.2f} Hz >= {args.min_diag_hz:.2f} Hz"))
    checks.append(("diagnostic F5 max gap",max_diag_gap<=args.max_diag_gap_ms,
                   f"{max_diag_gap:.1f} ms <= {args.max_diag_gap_ms:.1f} ms"))
    diag_age_s = (time.monotonic()-last_diag_arrival) if last_diag_arrival is not None else float("inf")
    checks.append(("diagnostic F5 fresh", diag_age_s <= 1.0, f"{diag_age_s:.2f} s <=1.0"))
    checks.append(("v6 sensor epoch present", len(lags) >= max(1,int(0.95*len(frames))),
                   f"{len(lags)}/{len(frames)} F0 frames"))
    checks.append(("protocol v6", int(last.get("version", 0)) >= 6,
                   f"v{int(last.get('version', 0))}"))
    sensor_hz = float(last.get("observed_sample_hz", 0.0))
    checks.append(("IMU sample rate",
                   args.min_sensor_hz <= sensor_hz <= args.max_sensor_hz,
                   f"{sensor_hz:.2f} Hz in [{args.min_sensor_hz:.1f}, {args.max_sensor_hz:.1f}]"))
    checks.append(("stream rate", stream_hz >= args.min_stream_hz,
                   f"{stream_hz:.2f} Hz >= {args.min_stream_hz:.2f}"))
    if host_intervals_ms:
        checks.append(("Python arrival P95",
                       host_interval_p95 <= args.max_host_interval_p95_ms,
                       f"{host_interval_p95:.2f} ms <= {args.max_host_interval_p95_ms:.2f}"))
    checks.append(("sequence loss", lost <= args.max_lost,
                   f"{lost} <= {args.max_lost}"))
    checks.append(("sequence continuity",seq_bad==0,str(seq_bad)))
    checks.append(("MCU epoch continuity",board_bad==0,str(board_bad)))
    checks.append(("host monotonic",host_bad==0,str(host_bad)))
    checks.append(("host max gap",bool(host_steps) and max(host_steps)<=args.max_host_gap_ms,
                   f"{max(host_steps):.1f} ms <= {args.max_host_gap_ms:.1f} ms" if host_steps else "unknown"))
    checks.append(("CRC errors", crc_errors == 0, f"{crc_errors}"))
    if lags:
        checks.append(("sensor lag P95", abs(lag_p95) <= args.max_lag_p95_us,
                       f"{lag_p95:.0f} us <= {args.max_lag_p95_us:.0f}"))
    checks.append(("sensor lag peak",len(lags)>=max(1,int(.95*len(frames)))
                   and lag_max<=args.max_lag_peak_us,
                   f"{lag_max:.0f} us <= {args.max_lag_peak_us:.0f} us"))
    checks.append(("stationary speed max", max(speeds) <= args.max_speed_mps,
                   f"{max(speeds):.6f} m/s <= {args.max_speed_mps:.6f}"))
    checks.append(("position drift", pos_drift <= args.max_pos_drift_m,
                   f"{pos_drift:.6f} m <= {args.max_pos_drift_m:.6f}"))
    checks.append(("roll span", roll_span <= args.max_rp_span_deg,
                   f"{roll_span:.4f} deg <= {args.max_rp_span_deg:.4f}"))
    checks.append(("pitch span", pitch_span <= args.max_rp_span_deg,
                   f"{pitch_span:.4f} deg <= {args.max_rp_span_deg:.4f}"))
    checks.append(("yaw drift", abs(yaw_drift) <= args.max_yaw_drift_deg,
                   f"{yaw_drift:+.4f} deg <= +/-{args.max_yaw_drift_deg:.4f}"))
    checks.append(("ESKF health resets", health_reset_delta == 0,
                   f"delta={health_reset_delta}"))

    if last_diag is not None:
        checks.append(("F5 v2 throughout",all(d.get("version",0)>=2 for d in diag_history),
                       f"{len(diag_history)} frames"))
        checks.append(("no transient F5 health fault",diag_bad_frames==0,
                       f"{diag_bad_frames} unhealthy diagnostic frames"))
        checks.append(("IMU config", bool(last_diag["imu_config_ok"]),
                       "ok" if last_diag["imu_config_ok"] else "BAD"))
        checks.append(("accel clip mask", int(last_diag["accel_clip_mask"]) == 0,
                       f"0x{int(last_diag['accel_clip_mask']):02X}"))
        checks.append(("gyro clip mask", int(last_diag["gyro_clip_mask"]) == 0,
                       f"0x{int(last_diag['gyro_clip_mask']):02X}"))
        checks.append(("covariance health", bool(last_diag["covariance_psd_ok"]),
                       "ok" if last_diag["covariance_psd_ok"] else "BAD"))
        for key, value in diag_delta.items():
            checks.append((key, value == 0 and diag_changed[key] == 0,
                           f"delta={value} intermediate_transitions={diag_changed[key]}"))

    print("\n--- METRICS ---")
    print(f"protocol=v{last.get('version',0)} imu_rate={last.get('observed_sample_hz',0.0):.2f}Hz")
    print(f"frames={len(frames)} stream={stream_hz:.3f}Hz lost={lost} "
          f"crc={crc_errors} other_err={other_errors}")
    print(f"continuity seq_bad={seq_bad} board_bad={board_bad} host_bad={host_bad} "
          f"F5_unhealthy={diag_bad_frames}")
    if host_intervals_ms:
        print(f"python_interval p95={host_interval_p95:.2f}ms "
              f"max={max(host_intervals_ms):.2f}ms")
    if lags:
        print(f"sensor_lag avg={lag_avg:.0f}us p95={lag_p95:.0f}us max={lag_max:.0f}us")
    print(f"velocity max={max(speeds):.6f}m/s rms={math.sqrt(sum(v*v for v in speeds)/len(speeds)):.6f}m/s")
    print(f"position delta=({pos_d[0]:+.6f},{pos_d[1]:+.6f},{pos_d[2]:+.6f})m norm={pos_drift:.6f}m")
    print(f"RP span=({roll_span:.4f},{pitch_span:.4f})deg yaw_drift={yaw_drift:+.4f}deg")
    print(f"linear_accel RMS=({lin_rms[0]:.5f},{lin_rms[1]:.5f},{lin_rms[2]:.5f})m/s2")
    print(f"final stdV={last['vel_std']} m/s final stdP={last['pos_std']} m")

    print("\n--- GATE ---")
    failed = 0
    for name, ok, detail in checks:
        print(f"{'PASS' if ok else 'FAIL'} {name}: {detail}")
        failed += 0 if ok else 1

    if failed:
        print(f"\nRESULT: FAIL ({failed} gate)")
        return 1
    print("\nRESULT: PASS")
    return 0


if __name__ == "__main__":
    from run_csv import run_logged
    raise SystemExit(run_logged(main, __file__))
