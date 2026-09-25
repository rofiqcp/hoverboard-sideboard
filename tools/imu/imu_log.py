#!/usr/bin/env python3
"""Log raw IMU + firmware temperature; tahan USB disconnect/reconnect."""
import argparse,csv,math,time
import serial
from read_imu import read_frame,decode_imu
from serial_common import open_sideboard_port
from tool_paths import log_output,display_path
G=9.80665

def main():
    ap=argparse.ArgumentParser(description="Log IMU v5 CSV; semua output disimpan relatif di tools/logs/")
    ap.add_argument("--duration",type=float,default=60.0)
    ap.add_argument("--output",default="auto",help="nama CSV relatif; default auto -> tools/logs/imu_YYYYMMDD_HHMMSS.csv")
    ap.add_argument("--port",default="auto"); ap.add_argument("--baud",type=int,default=921600)
    a=ap.parse_args()
    try: out=log_output(a.output,"imu")
    except ValueError as e: ap.error(str(e))
    fields=["host_s","seq","board_us","temp_c","ax_mps2","ay_mps2","az_mps2","gx_rads","gy_rads","gz_rads",
            "roll_deg","pitch_deg","yaw_deg","vx_mps","vy_mps","vz_mps","px_m","py_m","pz_m",
            "lax_mps2","lay_mps2","laz_mps2","flags","nav_status","cal_state","cal_coverage","cal_error",
            "cal_progress","aid_age_ms","aid_reject","imu_whoami","imu_class","observed_sample_hz"]
    n=moving=reconnects=0; t0=time.monotonic(); port=None
    with open(out,"w",newline="",buffering=1) as f:
        w=csv.DictWriter(f,fieldnames=fields); w.writeheader()
        while time.monotonic()-t0<a.duration:
            try:
                if port is None or not port.is_open:
                    port=open_sideboard_port(a.port,a.baud,timeout=.05,attempts=5,delay=.20)
                    port.reset_input_buffer()
                    if reconnects: print(f"Serial reconnect: {port.port}")
                payload,_=read_frame(port); d=decode_imu(payload) if payload else None
                if not d: continue
                if d["version"]<5: raise RuntimeError("Firmware protocol v5 diperlukan agar temperature_c tidak ambigu")
                ax,ay,az=(x/8192.0*G for x in d["accel_raw"])
                gx,gy,gz=(x/65.5*math.pi/180.0 for x in d["gyro_raw"])
                an=math.sqrt(ax*ax+ay*ay+az*az); gn=math.sqrt(gx*gx+gy*gy+gz*gz)*180/math.pi
                if abs(an/G-1)>0.12 or gn>3.0: moving+=1
                cs,cc,ce,cp=d["cal"]
                w.writerow(dict(host_s=time.monotonic()-t0,seq=d["seq"],board_us=d["time_us"],temp_c=d["temp_c"],
                                ax_mps2=ax,ay_mps2=ay,az_mps2=az,gx_rads=gx,gy_rads=gy,gz_rads=gz,
                                roll_deg=d["roll"],pitch_deg=d["pitch"],yaw_deg=d["yaw"],
                                vx_mps=d["vel"][0],vy_mps=d["vel"][1],vz_mps=d["vel"][2],
                                px_m=d["pos"][0],py_m=d["pos"][1],pz_m=d["pos"][2],
                                lax_mps2=d["linacc"][0],lay_mps2=d["linacc"][1],laz_mps2=d["linacc"][2],
                                flags=d["flags"],nav_status=d["nav_status"],cal_state=cs,cal_coverage=cc,cal_error=ce,
                                cal_progress=cp,aid_age_ms=d["aid_age_ms"],aid_reject=d["aid_reject"],
                                imu_whoami=d["imu_whoami"],imu_class=d["imu_class"],observed_sample_hz=d["observed_sample_hz"]))
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
