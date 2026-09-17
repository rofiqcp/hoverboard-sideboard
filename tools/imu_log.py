#!/usr/bin/env python3
"""Log raw IMU + firmware temperature for thermal/Allan calibration."""
import argparse,csv,math,time
from read_imu import read_frame,decode_imu
from serial_common import find_sideboard_port,open_sideboard_port
G=9.80665

def main():
    ap=argparse.ArgumentParser(description="Log IMU v5 CSV; board harus diam untuk calibration/noise")
    ap.add_argument("--duration",type=float,default=60.0); ap.add_argument("--output",required=True)
    ap.add_argument("--port",default="auto"); ap.add_argument("--baud",type=int,default=921600)
    a=ap.parse_args(); name=find_sideboard_port(a.port)
    fields=["host_s","seq","board_us","temp_c","ax_mps2","ay_mps2","az_mps2","gx_rads","gy_rads","gz_rads","roll_deg","pitch_deg","yaw_deg"]
    n=moving=0; t0=time.monotonic()
    with open(a.output,"w",newline="") as f, open_sideboard_port(name,a.baud,timeout=.05) as port:
        w=csv.DictWriter(f,fieldnames=fields); w.writeheader(); port.reset_input_buffer()
        while time.monotonic()-t0<a.duration:
            payload,_=read_frame(port); d=decode_imu(payload) if payload else None
            if not d: continue
            if d["version"]<5: raise RuntimeError("Firmware protocol v5 diperlukan agar temperature_c tidak ambigu")
            ax,ay,az=(x/8192.0*G for x in d["accel_raw"])
            gx,gy,gz=(x/65.5*math.pi/180.0 for x in d["gyro_raw"])
            an=math.sqrt(ax*ax+ay*ay+az*az); gn=math.sqrt(gx*gx+gy*gy+gz*gz)*180/math.pi
            if abs(an/G-1)>0.12 or gn>3.0: moving+=1
            w.writerow(dict(host_s=time.monotonic()-t0,seq=d["seq"],board_us=d["time_us"],temp_c=d["temp_c"],
                            ax_mps2=ax,ay_mps2=ay,az_mps2=az,gx_rads=gx,gy_rads=gy,gz_rads=gz,
                            roll_deg=d["roll"],pitch_deg=d["pitch"],yaw_deg=d["yaw"]))
            n+=1
    frac=100.0*moving/max(n,1)
    print(f"saved={a.output} samples={n} moving_or_bad={moving} ({frac:.2f}%)")
    return 0 if n>10 and frac<5.0 else 2
if __name__=="__main__": raise SystemExit(main())
