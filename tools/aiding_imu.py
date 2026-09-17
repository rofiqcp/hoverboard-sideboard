#!/usr/bin/env python3
import argparse, struct, time
from read_imu import crc16, read_frame
from serial_common import find_sideboard_port, open_sideboard_port

COMM=0xF4; BAUD=921600
TYPES={"wheel":1,"world-vel":2,"world-pos":3,"yaw":4}
TIMING={"now":0,"board":1,"age":2}
FRAMES={"local":0,"enu":1,"body":2}
STATUS={0:"accepted",1:"innovation/range rejected",2:"bad packet",3:"bad sigma",4:"stale/bad timing",5:"ESKF recovered",6:"bad/un-aligned frame"}

def packet(p):
    c=crc16(p); return bytes((2,len(p)))+p+bytes((c>>8,c&255,3))

def main():
    ap=argparse.ArgumentParser(description="Kirim external aiding ke ESKF dengan frame + timing eksplisit")
    ap.add_argument("command",choices=TYPES); ap.add_argument("values",nargs="+",type=float)
    ap.add_argument("--sigma",type=float,default=None)
    tg=ap.add_mutually_exclusive_group()
    tg.add_argument("--age-ms",type=float,default=None,help="umur measurement saat dikirim (recommended untuk ROS)")
    tg.add_argument("--board-timestamp",type=int,default=None,help="timestamp board_micros(), hanya jika clock sudah sinkron")
    ap.add_argument("--frame",choices=("local","enu"),default="local",help="world frame untuk world-vel/world-pos/yaw")
    ap.add_argument("--nhc",action="store_true"); ap.add_argument("--port",default="auto"); ap.add_argument("--baud",type=int,default=BAUD)
    a=ap.parse_args(); typ=TYPES[a.command]

    if a.age_ms is not None:
        if a.age_ms < 0 or a.age_ms > 4294967.0: ap.error("--age-ms di luar range")
        timing=TIMING["age"]; timing_value=round(a.age_ms*1000.0)
    elif a.board_timestamp is not None:
        timing=TIMING["board"]; timing_value=a.board_timestamp & 0xffffffff
    else:
        timing=TIMING["age"]; timing_value=0  # explicit zero-age, tidak bergantung clock host

    frame=FRAMES["body"] if a.command=="wheel" else FRAMES[a.frame]
    p=bytes((COMM,typ,timing,frame))+struct.pack(">I",timing_value)
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

    portname=find_sideboard_port(a.port); print(f"Sideboard: {portname} @ {a.baud} | timing={timing} frame={frame}")
    with open_sideboard_port(portname,a.baud,timeout=.05) as port:
        port.reset_input_buffer(); port.write(packet(p)); port.flush()
        deadline=time.monotonic()+1.5
        while time.monotonic()<deadline:
            r,e=read_frame(port)
            if r and len(r)==5 and r[0]==COMM and r[1]==typ:
                status=r[2]; age=(r[3]<<8)|r[4]
                print(f"aid status={status} ({STATUS.get(status,'unknown')}) age={age} ms")
                return 0 if status==0 else 2
    print("ERROR: timeout menunggu ACK aiding",flush=True); return 3

if __name__=="__main__": raise SystemExit(main())
