#!/usr/bin/env python3
"""Kirim external aiding F4 dengan request-id/retry idempotent."""
import argparse, struct, time
import serial
from read_imu import crc16, read_frame
from serial_common import find_sideboard_port, open_sideboard_port

COMM=0xF4; BAUD=921600
TYPES={"wheel":1,"world-vel":2,"world-pos":3,"yaw":4}
TIMING={"now":0,"board":1,"age":2}; FRAMES={"local":0,"enu":1,"body":2}
STATUS={0:"accepted",1:"innovation/range rejected",2:"bad packet",3:"bad sigma",4:"stale/bad timing",5:"ESKF recovered",6:"bad/un-aligned frame",7:"starting/busy"}

def packet(p):
    c=crc16(p); return bytes((2,len(p)))+p+bytes((c>>8,c&255,3))

def build_payload(a,typ,request_id):
    if a.age_ms is not None:
        if a.age_ms < 0 or a.age_ms > 4294967.0: raise ValueError("--age-ms di luar range")
        timing=TIMING["age"]; timing_value=round(a.age_ms*1000.0)
    elif a.board_timestamp is not None:
        timing=TIMING["board"]; timing_value=a.board_timestamp & 0xffffffff
    else:
        timing=TIMING["age"]; timing_value=0
    frame=FRAMES["body"] if a.command=="wheel" else FRAMES[a.frame]
    # v2: F4,type,timing,frame,request_id,u32 time/age,...
    p=bytes((COMM,typ,timing,frame))+struct.pack(">H",request_id)+struct.pack(">I",timing_value)
    if a.command=="wheel":
        if len(a.values)!=1: raise ValueError("wheel butuh velocity m/s")
        sigma=.05 if a.sigma is None else a.sigma
        p+=struct.pack(">iHB",round(a.values[0]*1000),round(sigma*1000),1 if a.nhc else 0)
    elif a.command in ("world-vel","world-pos"):
        if len(a.values)!=3: raise ValueError(f"{a.command} butuh x y z")
        sigma=(.10 if a.command=="world-vel" else .25) if a.sigma is None else a.sigma
        p+=struct.pack(">3iH",*(round(x*1000) for x in a.values),round(sigma*1000))
    else:
        if len(a.values)!=1: raise ValueError("yaw butuh derajat")
        sigma=3.0 if a.sigma is None else a.sigma
        p+=struct.pack(">iH",round(a.values[0]*1000),round(sigma*1000))
    return p,timing,frame

def transact(port,pkt,typ,request_id,retries=4):
    for attempt in range(retries):
        port.write(pkt); port.flush(); deadline=time.monotonic()+0.20
        while time.monotonic()<deadline:
            r,_=read_frame(port)
            if not r or r[0]!=COMM or len(r)<5 or r[1]!=typ: continue
            if len(r)>=7:
                rid=(r[5]<<8)|r[6]
                if rid!=request_id: continue
            else:
                # Firmware lama; masih bisa dibaca, tetapi retry idempotent tidak tersedia.
                if attempt>0: continue
            return r
        time.sleep(.010)
    return None

def main():
    ap=argparse.ArgumentParser(description="Kirim external aiding ke ESKF dengan retry idempotent")
    ap.add_argument("command",choices=TYPES); ap.add_argument("values",nargs="+",type=float)
    ap.add_argument("--sigma",type=float,default=None)
    tg=ap.add_mutually_exclusive_group(); tg.add_argument("--age-ms",type=float,default=None); tg.add_argument("--board-timestamp",type=int,default=None)
    ap.add_argument("--frame",choices=("local","enu"),default="local"); ap.add_argument("--nhc",action="store_true")
    ap.add_argument("--port",default="auto"); ap.add_argument("--baud",type=int,default=BAUD); ap.add_argument("--retries",type=int,default=4)
    a=ap.parse_args(); typ=TYPES[a.command]
    request_id=(time.monotonic_ns() ^ (time.time_ns()>>16)) & 0xffff
    if request_id==0: request_id=1
    try: payload,timing,frame=build_payload(a,typ,request_id)
    except ValueError as e: ap.error(str(e))
    pkt=packet(payload); portname=find_sideboard_port(a.port)
    print(f"Sideboard: {portname} @ {a.baud} | timing={timing} frame={frame} req={request_id}")
    last_error=None
    for session in range(3):
        try:
            with open_sideboard_port(portname,a.baud,timeout=.03,attempts=5,delay=.08) as port:
                time.sleep(.025); port.reset_input_buffer()
                r=transact(port,pkt,typ,request_id,max(1,a.retries))
                if r:
                    status=r[2]; age=(r[3]<<8)|r[4]
                    print(f"aid status={status} ({STATUS.get(status,'unknown')}) age={age} ms")
                    return 0 if status==0 else 2
        except (serial.SerialException,OSError,FileNotFoundError) as e:
            last_error=e
        time.sleep(.08)
    print(f"ERROR: timeout menunggu ACK aiding req={request_id} {last_error or ''}",flush=True); return 3

if __name__=="__main__": raise SystemExit(main())
