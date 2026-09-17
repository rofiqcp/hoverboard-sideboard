#!/usr/bin/env python3
"""Uji kompatibilitas response COMM_GET_IMU_DATA=65 seperti VESC Tool."""
import argparse, math, serial, struct, time
from read_imu import crc16, read_frame
PORT='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'
def packet(p):
    c=crc16(p);return bytes([2,len(p)])+p+bytes([c>>8,c&255,3])
def fauto(b): return struct.unpack('>f',b)[0]
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--port',default=PORT);ap.add_argument('--baud',type=int,default=921600);a=ap.parse_args()
    mask=0xFFFF;req=bytes([65,mask>>8,mask&255])
    with serial.Serial(a.port,a.baud,timeout=.05) as s:
        s.reset_input_buffer();s.write(packet(req));s.flush();end=time.monotonic()+1
        while time.monotonic()<end:
            p,e=read_frame(s)
            if p and p[0]==65:
                m=(p[1]<<8)|p[2];vals=[];i=3
                for bit in range(16):
                    if m&(1<<bit):vals.append(fauto(p[i:i+4]));i+=4
                    else:vals.append(None)
                print('COMM_GET_IMU_DATA OK mask=0x%04X'%m)
                print('RPY rad=',vals[0:3],'deg=',[v*180/math.pi for v in vals[0:3]])
                print('ACC g=',vals[3:6],'GYRO dps=',vals[6:9],'MAG=',vals[9:12],'Q=',vals[12:16]);return
    raise SystemExit('Tidak ada response VESC IMU')
if __name__=='__main__':main()
