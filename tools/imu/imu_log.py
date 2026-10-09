#!/usr/bin/env python3
"""Log raw IMU + firmware temperature; tahan USB disconnect/reconnect."""
import argparse,csv,math,time
import serial
from read_imu import read_frame,decode_imu,decode_diag,open_telemetry_port

# Compatibility name retained for tests/scripts; implementation is identity-probed.
open_sideboard_port = open_telemetry_port
from tool_paths import log_output,display_path
G=9.80665

def main():
    ap=argparse.ArgumentParser(description="Log IMU/ESKF v5+ CSV; v6 menambah sensor epoch/lag presisi")
    ap.add_argument("--duration",type=float,default=60.0)
    ap.add_argument("--output",default="auto",help="nama CSV relatif; default auto -> tools/logs/imu_YYYYMMDD_HHMMSS.csv")
    ap.add_argument("--port",default="auto"); ap.add_argument("--baud",type=int,default=921600)
    a=ap.parse_args()
    try: out=log_output(a.output,"imu")
    except ValueError as e: ap.error(str(e))
    fields=["host_s","seq","board_us","sample_us","sensor_lag_us","temp_c",
            "accel_lsb_per_g","gyro_lsb_per_dps",
            "ax_mps2","ay_mps2","az_mps2","gx_rads","gy_rads","gz_rads",
            "roll_deg","pitch_deg","yaw_deg","vx_mps","vy_mps","vz_mps","px_m","py_m","pz_m",
            "lax_mps2","lay_mps2","laz_mps2",
            "std_vx","std_vy","std_vz","std_px","std_py","std_pz",
            "flags","nav_status","cal_state","cal_coverage","cal_error",
            "cal_progress","aid_age_ms","aid_reject","imu_whoami","imu_class","observed_sample_hz",
            "accel_clip_mask","gyro_clip_mask","imu_config_ok","i2c_error_count","imu_reinit_count",
            "fifo_overflow_count","fifo_resync_count","fifo_max_bytes","scheduler_miss_count",
            "max_pending_ticks","uart_rx_overflow_count","uart_tx_timeout_count",
            "covariance_psd_ok","filter_health_reset_count","imu_gap_count","last_imu_gap_ms","max_imu_gap_ms",
            "diag_fresh","diag_age_ms"]
    n=moving=reconnects=0; t0=time.monotonic(); port=None; last_diag={}; last_diag_at=None
    with open(out,"w",newline="",buffering=1) as f:
        w=csv.DictWriter(f,fieldnames=fields); w.writeheader()
        while time.monotonic()-t0<a.duration:
            try:
                if port is None or not port.is_open:
                    port=open_sideboard_port(a.port,a.baud,timeout=.05,attempts=5,delay=.20)
                    port.reset_input_buffer()
                    last_diag={}; last_diag_at=None  # reject stale cache after reconnect
                    if reconnects: print(f"Serial reconnect: {port.port}")
                payload,_=read_frame(port)
                if payload:
                    diag=decode_diag(payload)
                    if diag is not None:
                        last_diag=diag
                        last_diag_at=time.monotonic()
                        continue
                d=decode_imu(payload) if payload else None
                if not d: continue
                if d["version"]<5:
                    raise RuntimeError("Firmware protocol v5+ diperlukan agar temperature/range tidak ambigu")
                highspeed=float(d.get("observed_sample_hz",0.0))>=300.0
                accel_scale=float(d.get("accel_lsb_per_g",4096.0 if highspeed else 8192.0))
                gyro_scale=float(d.get("gyro_lsb_per_dps",32.8 if highspeed else 65.5))
                ax,ay,az=(x/accel_scale*G for x in d["accel_raw"])
                gx,gy,gz=(x/gyro_scale*math.pi/180.0 for x in d["gyro_raw"])
                an=math.sqrt(ax*ax+ay*ay+az*az); gn=math.sqrt(gx*gx+gy*gy+gz*gz)*180/math.pi
                if abs(an/G-1)>0.12 or gn>3.0: moving+=1
                cs,cc,ce,cp=d["cal"]
                now_host=time.monotonic()
                diag_age_ms=(now_host-last_diag_at)*1000.0 if last_diag_at is not None else None
                current_diag=last_diag if diag_age_ms is not None and diag_age_ms <= 1000.0 else {}
                w.writerow(dict(host_s=now_host-t0,seq=d["seq"],board_us=d["time_us"],
                                sample_us=d.get("sample_time_us",d["time_us"]),sensor_lag_us=d.get("sensor_lag_us",0),temp_c=d["temp_c"],
                                accel_lsb_per_g=accel_scale,gyro_lsb_per_dps=gyro_scale,
                                ax_mps2=ax,ay_mps2=ay,az_mps2=az,gx_rads=gx,gy_rads=gy,gz_rads=gz,
                                roll_deg=d["roll"],pitch_deg=d["pitch"],yaw_deg=d["yaw"],
                                vx_mps=d["vel"][0],vy_mps=d["vel"][1],vz_mps=d["vel"][2],
                                px_m=d["pos"][0],py_m=d["pos"][1],pz_m=d["pos"][2],
                                lax_mps2=d["linacc"][0],lay_mps2=d["linacc"][1],laz_mps2=d["linacc"][2],
                                std_vx=d.get("vel_std",(0.0,0.0,0.0))[0],
                                std_vy=d.get("vel_std",(0.0,0.0,0.0))[1],
                                std_vz=d.get("vel_std",(0.0,0.0,0.0))[2],
                                std_px=d.get("pos_std",(0.0,0.0,0.0))[0],
                                std_py=d.get("pos_std",(0.0,0.0,0.0))[1],
                                std_pz=d.get("pos_std",(0.0,0.0,0.0))[2],
                                flags=d["flags"],nav_status=d["nav_status"],cal_state=cs,cal_coverage=cc,cal_error=ce,
                                cal_progress=cp,aid_age_ms=d["aid_age_ms"],aid_reject=d["aid_reject"],
                                imu_whoami=d["imu_whoami"],imu_class=d["imu_class"],observed_sample_hz=d["observed_sample_hz"],
                                accel_clip_mask=current_diag.get("accel_clip_mask",""),
                                gyro_clip_mask=current_diag.get("gyro_clip_mask",""),
                                imu_config_ok=(int(bool(current_diag["imu_config_ok"])) if "imu_config_ok" in current_diag else ""),
                                i2c_error_count=current_diag.get("i2c_error_count",""),
                                imu_reinit_count=current_diag.get("imu_reinit_count",""),
                                fifo_overflow_count=current_diag.get("fifo_overflow_count",""),
                                fifo_resync_count=current_diag.get("fifo_resync_count",""),
                                fifo_max_bytes=current_diag.get("fifo_max_bytes",""),
                                scheduler_miss_count=current_diag.get("scheduler_miss_count",""),
                                max_pending_ticks=current_diag.get("max_pending_ticks",""),
                                uart_rx_overflow_count=current_diag.get("uart_rx_overflow_count",""),
                                uart_tx_timeout_count=current_diag.get("uart_tx_timeout_count",""),
                                covariance_psd_ok=(int(bool(current_diag["covariance_psd_ok"])) if "covariance_psd_ok" in current_diag else ""),
                                filter_health_reset_count=current_diag.get("filter_health_reset_count",""),
                                imu_gap_count=current_diag.get("imu_gap_count",""),
                                last_imu_gap_ms=current_diag.get("last_imu_gap_ms",""),
                                max_imu_gap_ms=current_diag.get("max_imu_gap_ms",""),
                                diag_fresh=int(bool(current_diag)),
                                diag_age_ms=(round(diag_age_ms,2) if diag_age_ms is not None else "" )))
                n+=1
            except (serial.SerialException,OSError,FileNotFoundError) as exc:
                reconnects+=1
                if port is not None:
                    try: port.close()
                    except Exception: pass
                port=None; print(f"Serial terputus ({exc}); menunggu reconnect ...")
                time.sleep(.20)
        if port is not None:
            try: port.close()
            except Exception: pass
    frac=100.0*moving/max(n,1)
    print(f"saved={display_path(out)} samples={n} moving_or_bad={moving} ({frac:.2f}%) reconnects={reconnects}")
    return 0 if n>10 and frac<5.0 else 2
if __name__=="__main__":
    from run_csv import run_logged
    raise SystemExit(run_logged(main,__file__))
