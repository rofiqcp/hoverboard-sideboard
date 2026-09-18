#!/usr/bin/env python3
"""Uji kompatibilitas COMM_GET_IMU_DATA=65 seperti VESC Tool, robust terhadap USB re-open."""
import argparse, math, struct, time
import serial
from read_imu import crc16, read_frame
from serial_common import find_sideboard_port, open_sideboard_port

PORT='auto'

def packet(p):
    c=crc16(p); return bytes((2,len(p)))+p+bytes((c>>8,c&255,3))

def fauto(b): return struct.unpack('>f',b)[0]

def query_once(requested, baud, per_try_timeout=0.45):
    mask=0xFFFF; req=bytes((65,mask>>8,mask&255)); port=find_sideboard_port(requested)
    with open_sideboard_port(port,baud,timeout=.03,attempts=8,delay=.10) as s:
        # PL2303/CH340 dapat butuh jeda sangat singkat sesudah open. Jangan toggle DTR/RTS.
        time.sleep(.025); s.reset_input_buffer()
        for _ in range(2):
            s.write(packet(req)); s.flush(); end=time.monotonic()+per_try_timeout
            while time.monotonic()<end:
                p,_=read_frame(s)
                if p and p[0]==65 and len(p)>=3:
                    m=(p[1]<<8)|p[2]; vals=[]; i=3
                    for bit in range(16):
                        if m&(1<<bit):
                            if i+4>len(p): return None
                            vals.append(fauto(p[i:i+4])); i+=4
                        else: vals.append(None)
                    return m,vals
            # ACK dapat hilang tepat saat port baru dibuka; retry command pada sesi yang sama.
            time.sleep(.02)
    return None

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--port',default=PORT); ap.add_argument('--baud',type=int,default=921600); ap.add_argument('--attempts',type=int,default=4); a=ap.parse_args()
    last=None
    for attempt in range(max(1,a.attempts)):
        try: last=query_once(a.port,a.baud)
        except (serial.SerialException,OSError,FileNotFoundError): last=None
        if last: break
        if attempt+1<a.attempts: time.sleep(.10)
    if not last: raise SystemExit('Tidak ada response VESC IMU setelah retry')
    m,vals=last
    print('COMM_GET_IMU_DATA OK mask=0x%04X'%m)
    print('RPY rad=',vals[0:3],'deg=',[v*180/math.pi for v in vals[0:3]])
    print('ACC g=',vals[3:6],'GYRO dps=',vals[6:9],'MAG=',vals[9:12],'Q=',vals[12:16])

if __name__=='__main__': main()
