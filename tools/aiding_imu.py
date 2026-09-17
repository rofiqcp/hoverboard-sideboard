#!/usr/bin/env python3
import argparse, math, struct, serial, time
from read_imu import crc16, read_frame
from serial_common import find_sideboard_port, open_sideboard_port
COMM=0xF4; BAUD=921600
TYPES={"wheel":1,"world-vel":2,"world-pos":3,"yaw":4}
def packet(p):
    c=crc16(p); return bytes((2,len(p)))+p+bytes((c>>8,c&255,3))
def main():
    ap=argparse.ArgumentParser(description="Kirim measurement aiding timestamped ke ESKF sideboard")
    ap.add_argument("command",choices=TYPES); ap.add_argument("values",nargs="+",type=float)
    ap.add_argument("--sigma",type=float,default=None); ap.add_argument("--timestamp",type=int,default=0)
    ap.add_argument("--nhc",action="store_true"); ap.add_argument("--port",default="auto"); ap.add_argument("--baud",type=int,default=BAUD)
    a=ap.parse_args(); typ=TYPES[a.command]; p=bytes((COMM,typ))+struct.pack(">I",a.timestamp&0xffffffff)
    if a.command=="wheel":
        if len(a.values)!=1: ap.error("wheel butuh velocity m/s")
        sigma=.05 if a.sigma is None else a.sigma
        p+=struct.pack(">iHB",round(a.values[0]*1000),round(sigma*1000),1 if a.nhc else 0)
    elif a.command in ("world-vel","world-pos"):
        if len(a.values)!=3: ap.error(f"{a.command} butuh x y z")
        sigma=(.10 if a.command=="world-vel" else .25) if a.sigma is None else a.sigma
        p+=struct.pack(">3iH",*(round(x*1000) for x in a.values),round(sigma*1000))
    else:
        if len(a.values)!=1: ap.error("yaw butuh derajat")
        sigma=3.0 if a.sigma is None else a.sigma
        p+=struct.pack(">iH",round(a.values[0]*1000),round(sigma*1000))
    portname=find_sideboard_port(a.port); print(f"Sideboard: {portname} @ {a.baud}")
    with open_sideboard_port(portname,a.baud,timeout=.05) as port:
        port.reset_input_buffer(); port.write(packet(p)); port.flush()
        deadline=time.monotonic()+1.0
        while time.monotonic()<deadline:
            r,e=read_frame(port)
            if r and len(r)==5 and r[0]==COMM and r[1]==typ:
                status=r[2]; age=(r[3]<<8)|r[4]; print(f"aid status={status} age={age} ms")
                return 0 if status==0 else 2
        print("ERROR: timeout menunggu ACK aiding", flush=True)
        return 3
if __name__=="__main__": raise SystemExit(main())
