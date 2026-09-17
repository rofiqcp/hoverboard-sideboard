#!/usr/bin/env python3
import argparse, math, struct, serial, time
from read_imu import crc16, read_frame
from serial_common import find_sideboard_port, open_sideboard_port
COMM=0xF3; BAUD=921600
SUB={"get":1,"mount-rpy":2,"thermal":3,"lever":4,"clear-thermal":5,"reset-mount":6}
def packet(p):
    c=crc16(p); return bytes((2,len(p)))+p+bytes((c>>8,c&255,3))
def wait(port,sub,timeout=1.0):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        p,e=read_frame(port)
        if p and len(p)==59 and p[0]==COMM and p[1]==sub:
            vals=struct.unpack(">BBB4i3i3i3iI",p)
            q=tuple(x/1e6 for x in vals[3:7]); gs=tuple(x/1e6 for x in vals[7:10])
            a=tuple(x/1e6 for x in vals[10:13]); lever=tuple(x/1000 for x in vals[13:16])
            return vals[2],q,gs,a,lever,vals[16]
    raise TimeoutError("timeout menunggu ACK konfigurasi")
def main():
    ap=argparse.ArgumentParser(description="Konfigurasi mounting/thermal/lever-arm IMU")
    ap.add_argument("command",nargs="?",choices=SUB,default="get"); ap.add_argument("values",nargs="*",type=float)
    ap.add_argument("--port",default="auto"); ap.add_argument("--baud",type=int,default=BAUD); args=ap.parse_args()
    sub=SUB[args.command]; p=bytes((COMM,sub)); v=args.values
    if args.command=="mount-rpy":
        if len(v)!=3: ap.error("mount-rpy butuh roll pitch yaw dalam derajat")
        p+=struct.pack(">3i",*(round(x*1000) for x in v))
    elif args.command=="thermal":
        if len(v)!=6: ap.error("thermal butuh gx gy gz [deg/s/C] ax ay az [m/s2/C]")
        p+=struct.pack(">6i",*(round(x*math.pi/180*1e6) for x in v[:3]),*(round(x*1e6) for x in v[3:]))
    elif args.command=="lever":
        if len(v)!=3: ap.error("lever butuh x y z meter")
        p+=struct.pack(">3i",*(round(x*1000) for x in v))
    elif v: ap.error("command ini tidak menerima values")
    portname=find_sideboard_port(args.port); print(f"Sideboard: {portname} @ {args.baud}")
    with open_sideboard_port(portname,args.baud,timeout=.05) as port:
        port.reset_input_buffer(); port.write(packet(p)); port.flush()
        try: status,q,gs,a,lever,flags=wait(port,sub)
        except TimeoutError as exc:
            print(f"ERROR: {exc}")
            return 3
    print(f"status={status} flags=0x{flags:08X} q_sensor_to_body={q}")
    print("gyro_temp_slope deg/s/C=",tuple(x*180/math.pi for x in gs))
    print("accel_temp_slope m/s2/C=",a,"lever_arm_m=",lever)
    return 0 if status==0 else 2
if __name__=="__main__": raise SystemExit(main())
