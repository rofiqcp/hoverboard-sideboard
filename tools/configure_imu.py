#!/usr/bin/env python3
import argparse,math,struct,time,serial
from read_imu import crc16,read_frame
from serial_common import find_sideboard_port,open_sideboard_port
COMM=0xF3; BAUD=921600
SUB={"get":1,"mount-rpy":2,"thermal":3,"lever":4,"clear-thermal":5,"reset-mount":6,"noise":7,"accel-cal":8}

def packet(p):
    c=crc16(p); return bytes((2,len(p)))+p+bytes((c>>8,c&255,3))

def decode(p,sub):
    if not p or p[0]!=COMM or len(p)<3 or p[1]!=sub:return None
    if len(p)==3:return {"status":p[2]}
    if len(p) not in (59,79,127):return None
    vals=struct.unpack(">BBB4i3i3i3iI",p[:59]); out={"status":vals[2]}
    out["q"]=tuple(x/1e6 for x in vals[3:7]); out["gs"]=tuple(x/1e6 for x in vals[7:10])
    out["a"]=tuple(x/1e6 for x in vals[10:13]); out["lever"]=tuple(x/1000 for x in vals[13:16]); out["flags"]=vals[16]
    out["noise"]=tuple(x/1e6 for x in struct.unpack(">5i",p[59:79])) if len(p)>=79 else None
    out["accel_offset"]=tuple(x/1e6 for x in struct.unpack(">3i",p[79:91])) if len(p)==127 else None
    out["accel_matrix"]=tuple(x/1e6 for x in struct.unpack(">9i",p[91:127])) if len(p)==127 else None
    return out

def request(req,sub,portname,baud,overall=8.0):
    deadline=time.monotonic()+overall; last=None
    while time.monotonic()<deadline:
        try:
            with open_sideboard_port(portname,baud,timeout=.05) as port:
                port.reset_input_buffer();port.write(packet(req));port.flush();end=min(deadline,time.monotonic()+1.5)
                while time.monotonic()<end:
                    p,_=read_frame(port); d=decode(p,sub)
                    if d:
                        if d["status"]==7: break
                        return d
        except (serial.SerialException,OSError,FileNotFoundError) as e:last=e
        time.sleep(.20)
    raise TimeoutError(f"timeout/retry konfigurasi: {last}")

def main():
    ap=argparse.ArgumentParser(description="Konfigurasi mounting/thermal/lever/noise IMU dengan retry startup")
    ap.add_argument("command",nargs="?",choices=SUB,default="get");ap.add_argument("values",nargs="*",type=float)
    ap.add_argument("--port",default="auto");ap.add_argument("--baud",type=int,default=BAUD);a=ap.parse_args();v=a.values;sub=SUB[a.command]
    req=bytes((COMM,sub))
    if a.command=="mount-rpy":
        if len(v)!=3:ap.error("mount-rpy butuh roll pitch yaw derajat")
        req+=struct.pack(">3i",*(round(x*1000) for x in v))
    elif a.command=="thermal":
        if len(v)!=6:ap.error("thermal butuh gx gy gz [deg/s/C] ax ay az [m/s2/C]")
        req+=struct.pack(">6i",*(round(x*math.pi/180*1e6) for x in v[:3]),*(round(x*1e6) for x in v[3:]))
    elif a.command=="lever":
        if len(v)!=3:ap.error("lever butuh x y z meter")
        req+=struct.pack(">3i",*(round(x*1000) for x in v))
    elif a.command=="noise":
        if len(v)!=5:ap.error("noise butuh gyro_noise accel_noise gyro_bias_walk accel_bias_walk accel_dir_noise (SI)")
        req+=struct.pack(">5i",*(round(x*1e6) for x in v))
    elif a.command=="accel-cal":
        if len(v)!=12:ap.error("accel-cal butuh offset xyz lalu matrix 3x3 row-major (12 nilai)")
        req+=struct.pack(">12i",*(round(x*1e6) for x in v))
    elif v:ap.error("command ini tidak menerima values")
    port=find_sideboard_port(a.port);print(f"Sideboard: {port} @ {a.baud}")
    try:d=request(req,sub,port,a.baud)
    except TimeoutError as e:print("ERROR:",e);return 3
    print(f"status={d['status']}")
    if "q" in d:
        print(f"flags=0x{d['flags']:08X} q_sensor_to_body={d['q']}")
        print("gyro_temp_slope deg/s/C=",tuple(x*180/math.pi for x in d["gs"]))
        print("accel_temp_slope m/s2/C=",d["a"],"lever_arm_m=",d["lever"])
        if d["noise"]:print("noise [gyro,accel,gyro_bias_walk,accel_bias_walk,accel_dir]=",d["noise"])
        if d.get("accel_offset"):
            print("accel_offset_mps2=",d["accel_offset"]); m=d["accel_matrix"]; print("accel_transform=")
            for r in range(3): print(" ",m[3*r:3*r+3])
    return 0 if d["status"]==0 else 2
if __name__=="__main__":raise SystemExit(main())
