#!/usr/bin/env python3
"""Analyze structured STM32 IMU/ESKF CSV logs without ROS."""
from __future__ import annotations

import argparse
import csv
import math
import statistics
from pathlib import Path

from tool_paths import log_input, display_path


def pctl(values, q):
    if not values:
        return float("nan")
    s=sorted(float(x) for x in values)
    if len(s)==1:
        return s[0]
    x=(len(s)-1)*q
    lo=int(math.floor(x)); hi=int(math.ceil(x))
    if lo==hi:
        return s[lo]
    a=x-lo
    return s[lo]*(1-a)+s[hi]*a


def mean_std(values):
    v=[float(x) for x in values]
    if not v:
        return float("nan"),float("nan")
    return statistics.fmean(v), statistics.pstdev(v) if len(v)>1 else 0.0


def rms(values):
    v=[float(x) for x in values]
    return math.sqrt(sum(x*x for x in v)/len(v)) if v else float("nan")


def u32_delta(a,b):
    return (int(a)-int(b)) & 0xffffffff


def f(row,key,default=0.0):
    try:
        return float(row.get(key,default))
    except (TypeError,ValueError):
        return float(default)


def i(row,key,default=0):
    try:
        return int(float(row.get(key,default)))
    except (TypeError,ValueError):
        return int(default)


def main():
    ap=argparse.ArgumentParser(description="Analisis CSV STM32 IMU/ESKF xyz/vxyz")
    ap.add_argument("file",help="CSV relatif di tools/logs/ atau path absolut")
    ap.add_argument("--stationary",action="store_true",help="aktifkan stationary acceptance gate")
    ap.add_argument("--max-speed-mps",type=float,default=0.020)
    ap.add_argument("--max-pos-drift-m",type=float,default=0.050)
    ap.add_argument("--max-lag-p95-us",type=float,default=8000.0)
    ap.add_argument("--min-stream-hz",type=float,default=48.0)
    ap.add_argument("--max-host-p95-ms",type=float,default=35.0,
                    help="P95 interval paket NAV di host saat stationary")
    ap.add_argument("--max-host-gap-ms",type=float,default=250.0,
                    help="jeda tunggal host maksimum sebelum tes gagal")
    ap.add_argument("--max-board-gap-ms",type=float,default=150.0,
                    help="jeda snapshot MCU maksimum (menerima wrap u32)")
    ap.add_argument("--max-diag-blind-s",type=float,default=0.50,
                    help="batas durasi kontigu tanpa bukti diagnostik sehat")
    ap.add_argument("--max-lag-peak-us",type=float,default=25000.0)
    args=ap.parse_args()

    raw=Path(args.file)
    if raw.is_absolute():
        path=raw
    else:
        try:
            path=log_input(args.file)
        except (ValueError,FileNotFoundError) as exc:
            ap.error(str(exc))

    with open(path,newline="",encoding="utf-8") as fh:
        rows=list(csv.DictReader(fh))
    if len(rows)<2:
        raise SystemExit("ERROR: log butuh minimal 2 frame")

    nav_keys=("vx_mps","vy_mps","vz_mps","px_m","py_m","pz_m",
              "lax_mps2","lay_mps2","laz_mps2","host_s","board_us","seq")
    for n,row in enumerate(rows):
        if not all(math.isfinite(f(row,k,float("nan"))) for k in nav_keys):
            print(f"FAIL finite navigation outputs: missing/NaN/Inf at row {n+1}")
            print("RESULT: FAIL (invalid navigation data)")
            return 1

    first,last=rows[0],rows[-1]
    board=[i(r,"board_us") & 0xffffffff for r in rows]
    seq=[i(r,"seq") & 0xffffffff for r in rows]
    host=[f(r,"host_s") for r in rows]
    board_span=u32_delta(board[-1],board[0])
    seq_span=u32_delta(seq[-1],seq[0])
    stream_hz=seq_span/(board_span*1e-6) if board_span else 0.0
    lost=max(0,seq_span-(len(rows)-1))

    # Reject local time/sequence discontinuities even if first and last
    # values appear consistent. Unsigned u32 subtraction preserves genuine
    # wraps (2**32-1 -> 0) while exposing out-of-order/restarted packets.
    seq_steps=[u32_delta(b,a) for a,b in zip(seq,seq[1:])]
    board_steps_us=[u32_delta(b,a) for a,b in zip(board,board[1:])]
    host_steps_ms=[(b-a)*1000.0 for a,b in zip(host,host[1:])]
    seq_discontinuities=sum(d != 1 for d in seq_steps)
    board_discontinuities=sum(d == 0 or d > args.max_board_gap_ms*1000.0
                              for d in board_steps_us)
    host_discontinuities=sum(d <= 0 for d in host_steps_ms)
    host_dt=[d for d in host_steps_ms if d >= 0]
    lag=[abs(f(r,"sensor_lag_us")) for r in rows if
         math.isfinite(f(r,"sensor_lag_us",float("nan")))
         and abs(f(r,"sensor_lag_us")) < 1e6]

    axes=("x","y","z")
    vcols=("vx_mps","vy_mps","vz_mps")
    pcols=("px_m","py_m","pz_m")
    acols=("lax_mps2","lay_mps2","laz_mps2")
    vel={a:[f(r,c) for r in rows] for a,c in zip(axes,vcols)}
    pos={a:[f(r,c) for r in rows] for a,c in zip(axes,pcols)}
    lin={a:[f(r,c) for r in rows] for a,c in zip(axes,acols)}
    speeds=[math.sqrt(sum(vel[a][k]**2 for a in axes)) for k in range(len(rows))]
    pd=[pos[a][-1]-pos[a][0] for a in axes]
    pnorm=math.sqrt(sum(x*x for x in pd))
    pexcursion=max(math.sqrt(sum((pos[a][k]-pos[a][0])**2 for a in axes))
                   for k in range(len(rows)))

    print(f"file={display_path(path)} rows={len(rows)}")
    print(f"continuity: seq_discontinuities={seq_discontinuities} "
          f"board_discontinuities={board_discontinuities} "
          f"host_nonmonotonic={host_discontinuities}")
    print(f"stream={stream_hz:.3f} Hz lost_seq={lost} duration={board_span*1e-6:.3f}s")
    if host_dt:
        print(f"host_interval_ms p50={pctl(host_dt,.50):.3f} p95={pctl(host_dt,.95):.3f} "
              f"p99={pctl(host_dt,.99):.3f} max={max(host_dt):.3f}")
    if lag:
        print(f"sensor_lag_us mean={statistics.fmean(lag):.1f} p95={pctl(lag,.95):.1f} max={max(lag):.1f}")

    print("\nVELOCITY")
    for a in axes:
        m,s=mean_std(vel[a])
        print(f"v{a}: mean={m:+.6f} std={s:.6f} rms={rms(vel[a]):.6f} "
              f"maxabs={max(abs(x) for x in vel[a]):.6f} m/s")
    print(f"speed: rms={rms(speeds):.6f} max={max(speeds):.6f} m/s")

    print("\nPOSITION")
    for idx,a in enumerate(axes):
        span=max(pos[a])-min(pos[a])
        print(f"{a}: delta={pd[idx]:+.6f} span={span:.6f} final={pos[a][-1]:+.6f} m")
    print(f"position_delta_norm={pnorm:.6f} m; max_excursion_norm={pexcursion:.6f} m")

    print("\nLINEAR ACCELERATION")
    for a in axes:
        m,s=mean_std(lin[a])
        print(f"a{a}: mean={m:+.6f} std={s:.6f} rms={rms(lin[a]):.6f} m/s2")

    for name,col in (("roll","roll_deg"),("pitch","pitch_deg"),("yaw","yaw_deg")):
        vals=[f(r,col) for r in rows]
        print(f"{name}: start={vals[0]:+.4f} end={vals[-1]:+.4f} "
              f"delta={vals[-1]-vals[0]:+.4f} span={max(vals)-min(vals):.4f} deg")

    print("\nUNCERTAINTY FINAL")
    print("stdV=({:.5f},{:.5f},{:.5f}) m/s".format(
        f(last,"std_vx"),f(last,"std_vy"),f(last,"std_vz")))
    print("stdP=({:.5f},{:.5f},{:.5f}) m".format(
        f(last,"std_px"),f(last,"std_py"),f(last,"std_pz")))

    print("\nHEALTH DELTAS")
    health_keys=(
        "i2c_error_count","imu_reinit_count","fifo_overflow_count","fifo_resync_count",
        "scheduler_miss_count","uart_rx_overflow_count","uart_tx_timeout_count",
        "filter_health_reset_count","imu_gap_count"
    )
    required_diag_columns = health_keys + (
        "imu_config_ok", "covariance_psd_ok", "accel_clip_mask",
        "gyro_clip_mask", "diag_fresh", "diag_age_ms",
    )
    required_diag_present = all(k in first for k in required_diag_columns)
    def good_diag_row(row):
        if not required_diag_present or str(row.get("diag_fresh","")) != "1":
            return False
        try:
            values = [float(row[k]) for k in required_diag_columns if k != "diag_fresh"]
            return all(math.isfinite(v) for v in values) and 0 <= float(row["diag_age_ms"]) <= 1000.0
        except (TypeError, ValueError, KeyError):
            return False
    healthy_diag_rows = [r for r in rows if good_diag_row(r)]
    coverage = len(healthy_diag_rows)/len(rows)
    last_diag_fresh = good_diag_row(last)
    # Continuous F5 blackout is dangerous even if overall coverage >=90%.
    # Sample times are elapsed host-monotonic seconds, not 32-bit board time.
    blind_start=None
    max_blind_s=0.0
    for n,row in enumerate(rows):
        if good_diag_row(row):
            if blind_start is not None:
                max_blind_s=max(max_blind_s,host[n]-blind_start)
                blind_start=None
        elif blind_start is None:
            blind_start=host[n]
    if blind_start is not None:
        max_blind_s=max(max_blind_s,host[-1]-blind_start)

    health_bad = 0
    if healthy_diag_rows:
        first_health,last_health=healthy_diag_rows[0],healthy_diag_rows[-1]
        for key in health_keys:
            readings=[i(r,key) for r in healthy_diag_rows]
            # Per-snapshot checks catch 0->1->0 counter glitches/reboots
            # that first-to-last differences would falsely accept.
            changed=sum(u32_delta(b,a) != 0 for a,b in zip(readings,readings[1:]))
            d=u32_delta(readings[-1],readings[0])
            health_bad += int(changed != 0)
            print(f"{key}: delta={d} transitions={changed}")
    else:
        health_bad = len(health_keys)
        print("UNKNOWN: no valid F5 diagnostic snapshots in log")
    health_bad += sum(int(i(r,"covariance_psd_ok") != 1 or
                          i(r,"imu_config_ok") != 1 or
                          i(r,"accel_clip_mask") != 0 or
                          i(r,"gyro_clip_mask") != 0)
                      for r in healthy_diag_rows)
    print(f"F5 diagnostic coverage={coverage*100:.1f}% "
          f"last_fresh={last_diag_fresh} max_blind={max_blind_s:.3f}s "
          "(health must be OBSERVED, not guessed)")
    print(f"final covariance_psd_ok={last.get('covariance_psd_ok','UNKNOWN')} "
          f"imu_config_ok={last.get('imu_config_ok','UNKNOWN')} "
          f"clipA={last.get('accel_clip_mask','UNKNOWN')} "
          f"clipG={last.get('gyro_clip_mask','UNKNOWN')}")

    if args.stationary:
        gates=[
            ("F5 diagnostic columns",required_diag_present,str(required_diag_present)),
            ("F5 diagnostic coverage",coverage >= 0.90,f"{coverage*100:.1f}% >=90%"),
            ("F5 last frame fresh",last_diag_fresh,str(last_diag_fresh)),
            ("F5 blackout duration",max_blind_s<=args.max_diag_blind_s,
             f"{max_blind_s:.3f}<={args.max_diag_blind_s:.3f} s"),
            ("sequence continuity",seq_discontinuities==0,str(seq_discontinuities)),
            ("MCU epoch continuity",board_discontinuities==0,str(board_discontinuities)),
            ("host time monotonic",host_discontinuities==0,str(host_discontinuities)),
            ("host P95 jitter",bool(host_dt) and pctl(host_dt,.95)<=args.max_host_p95_ms,
             f"{pctl(host_dt,.95):.2f}<={args.max_host_p95_ms:.2f} ms"),
            ("host max gap",bool(host_dt) and max(host_dt)<=args.max_host_gap_ms,
             f"{max(host_dt):.2f}<={args.max_host_gap_ms:.2f} ms" if host_dt else "unknown"),
            ("sensor lag peak",len(lag)==len(rows) and max(lag)<=args.max_lag_peak_us,
             f"{max(lag):.0f}<={args.max_lag_peak_us:.0f} us" if lag else "missing"),
            ("finite navigation outputs",
             all(math.isfinite(v) for a in axes for v in (vel[a]+pos[a]+lin[a]))
             and all(math.isfinite(v) for v in (speeds+[stream_hz,pnorm])),
             "xyz/vxyz and rate finite"),
            ("stream rate",stream_hz>=args.min_stream_hz,
             f"{stream_hz:.2f}>={args.min_stream_hz:.2f} Hz"),
            ("sequence loss",lost==0,f"{lost}"),
            ("speed max",max(speeds)<=args.max_speed_mps,
             f"{max(speeds):.6f}<={args.max_speed_mps:.6f} m/s"),
            ("position drift",pnorm<=args.max_pos_drift_m,
             f"{pnorm:.6f}<={args.max_pos_drift_m:.6f} m"),
            ("position excursion",pexcursion<=args.max_pos_drift_m,
             f"{pexcursion:.6f}<={args.max_pos_drift_m:.6f} m"),
            ("sensor lag P95",len(lag) >= max(1,int(.95*len(rows)))
             and pctl(lag,.95)<=args.max_lag_p95_us,
             f"{pctl(lag,.95):.0f}<={args.max_lag_p95_us:.0f} us" if lag else "missing sensor age"),
            ("runtime health",health_bad==0,f"{health_bad} counters changed"),
            ("covariance PSD",last_diag_fresh and i(last,"covariance_psd_ok")==1,str(last.get("covariance_psd_ok","UNKNOWN"))),
            ("IMU config",last_diag_fresh and i(last,"imu_config_ok")==1,str(last.get("imu_config_ok","UNKNOWN"))),
            ("no clipping",last_diag_fresh and i(last,"accel_clip_mask")==0 and i(last,"gyro_clip_mask")==0,
             f"A=0x{i(last,'accel_clip_mask'):02X} G=0x{i(last,'gyro_clip_mask'):02X}"),
        ]
        print("\nSTATIONARY GATE")
        failed=0
        for name,ok,detail in gates:
            print(f"{'PASS' if ok else 'FAIL'} {name}: {detail}")
            failed += int(not ok)
        print(f"RESULT: {'PASS' if failed==0 else 'FAIL'} ({failed} failed)")
        return 0 if failed==0 else 1
    return 0


if __name__=="__main__":
    raise SystemExit(main())
