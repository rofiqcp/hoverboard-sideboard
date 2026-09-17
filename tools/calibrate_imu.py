#!/usr/bin/env python3
"""Kalibrasi/monitor IMU dengan auto-reconnect dan resume state firmware."""
import argparse
import select
import sys
import time
import serial
from read_imu import crc16, read_frame, decode_imu, flag_text
from serial_common import find_sideboard_port, open_sideboard_port

BAUD_DEFAULT=921600; COMM_CAL=0xF2
CMDS={"still":1,"rotate-start":2,"rotate-finish":3,"cancel":4,"zero-nav":5,
      "zupt":6,"status":7,"stationary-on":8,"stationary-off":9}
FACE_NAMES=["+X","-X","+Y","-Y","+Z","-Z"]
STATE_IDLE=0; STATE_STILL=1; STATE_ROTATE=2; STATE_DONE=3; STATE_FAILED=4

def packet(payload):
    c=crc16(payload); return bytes((2,len(payload)))+payload+bytes((c>>8,c&255,3))

def parse_cal_status(payload):
    if not payload or len(payload)<8 or payload[0]!=COMM_CAL:return None
    return {"sub":payload[1],"status":payload[2],"state":payload[3],"coverage":payload[4],
            "error":payload[5],"progress":((payload[6]<<8)|payload[7])/10.0}

def coverage_text(mask):
    done=[FACE_NAMES[i] for i in range(6) if mask&(1<<i)]
    missing=[FACE_NAMES[i] for i in range(6) if not mask&(1<<i)]
    return f"selesai={','.join(done) or '-'} | kurang={','.join(missing) or '-'}"

def print_status(st):
    names={0:"IDLE",1:"STILL",2:"ROTATE",3:"DONE",4:"FAILED"}
    print(f"CAL {names.get(st['state'],st['state'])} | status={st['status']} progress={st['progress']:.1f}% "
          f"coverage=0x{st['coverage']:02X} error={st['error']} | {coverage_text(st['coverage'])}")

class CalLink:
    def __init__(self, requested="auto", baud=BAUD_DEFAULT):
        self.requested=requested; self.baud=baud; self.port=None; self.last_name=None
    def close(self):
        if self.port is not None:
            try:self.port.close()
            except Exception:pass
        self.port=None
    def connect(self, announce=True):
        self.close()
        self.port=open_sideboard_port(self.requested,self.baud,timeout=.04,attempts=150,delay=.20)
        self.last_name=self.port.port
        try:self.port.reset_input_buffer()
        except Exception:pass
        if announce: print(f"Sideboard tersambung: {self.last_name} @ {self.baud}")
        return self.port
    def ensure(self):
        if self.port is None or not self.port.is_open:self.connect()
        return self.port
    def one_request(self, sub, timeout=1.2):
        p=self.ensure(); p.write(packet(bytes((COMM_CAL,sub)))); p.flush()
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            payload,_=read_frame(p); st=parse_cal_status(payload)
            if st and st["sub"]==sub:return st
        raise TimeoutError(f"Tidak ada ACK command kalibrasi {sub}")
    def status(self, overall=30.0):
        end=time.monotonic()+overall; last=None
        while time.monotonic()<end:
            try:return self.one_request(CMDS["status"],1.0)
            except (TimeoutError,serial.SerialException,OSError,FileNotFoundError) as exc:
                last=exc; self.close(); print("Serial terputus/ACK hilang; menunggu reconnect ...",file=sys.stderr)
                time.sleep(.20)
        raise TimeoutError(f"STATUS gagal setelah reconnect: {last}")
    def command_retry(self, sub, overall=15.0):
        """Untuk command idempotent; retry setelah reconnect."""
        end=time.monotonic()+overall; last=None
        while time.monotonic()<end:
            try:return self.one_request(sub,1.2)
            except (TimeoutError,serial.SerialException,OSError,FileNotFoundError) as exc:
                last=exc; self.close(); time.sleep(.20)
        raise TimeoutError(f"Command {sub} gagal setelah reconnect: {last}")

def start_or_resume(link, sub, target_state):
    try:
        old=link.status(10.0)
        if old["state"]==target_state:
            print("Firmware masih menjalankan kalibrasi sebelumnya; RESUME tanpa reset progress.")
            return old
    except TimeoutError: pass
    # START tidak diulang sebelum kita tahu state, agar ACK yang hilang tidak mereset progress.
    try:return link.one_request(sub,1.5)
    except (TimeoutError,serial.SerialException,OSError,FileNotFoundError):
        link.close(); print("ACK START hilang; cek state firmware setelah reconnect ...",file=sys.stderr)
    end=time.monotonic()+30.0
    while time.monotonic()<end:
        st=link.status(10.0)
        if st["state"] in (target_state,STATE_DONE,STATE_FAILED):return st
        # State terkonfirmasi belum mulai, baru aman kirim START lagi.
        try:return link.one_request(sub,1.5)
        except (TimeoutError,serial.SerialException,OSError,FileNotFoundError):link.close()
    raise TimeoutError("Tidak bisa memastikan START kalibrasi")

def finish_rotate_robust(link):
    try:return link.one_request(CMDS["rotate-finish"],3.0)
    except (TimeoutError,serial.SerialException,OSError,FileNotFoundError):
        link.close(); print("ACK FINISH hilang; verifikasi state setelah reconnect ...",file=sys.stderr)
        st=link.status(30.0)
        if st["state"] in (STATE_DONE,STATE_FAILED):return st
        if st["state"]==STATE_ROTATE and st["coverage"]==0x3F:
            return link.command_retry(CMDS["rotate-finish"],20.0)
        return st

def run_still(link):
    print("\nKalibrasi diam: jangan sentuh atau gerakkan board sampai selesai.")
    st=start_or_resume(link,CMDS["still"],STATE_STILL); print_status(st)
    deadline=time.monotonic()+120.0; last=-1
    while time.monotonic()<deadline:
        if st["state"]==STATE_DONE:
            print("Kalibrasi diam selesai dan firmware telah menyimpan hasil ke EEPROM.");return 0
        if st["state"]==STATE_FAILED:
            print("Kalibrasi diam gagal. Pastikan board benar-benar diam.",file=sys.stderr);return 2
        time.sleep(.30); st=link.status(30.0)
        if int(st["progress"])!=last or st["state"] in (STATE_DONE,STATE_FAILED):
            print_status(st);last=int(st["progress"])
    print("Timeout menunggu kalibrasi diam.",file=sys.stderr);return 3

def run_rotate(link):
    print("\nKalibrasi 6 sisi accelerometer.")
    print("Putar perlahan lalu TAHAN board pada +X, -X, +Y, -Y, +Z, -Z.")
    print("Jika USB putus, JANGAN restart tool berulang; firmware tetap mengumpulkan sisi dan tool akan reconnect.")
    st=start_or_resume(link,CMDS["rotate-start"],STATE_ROTATE);print_status(st)
    deadline=time.monotonic()+900.0; last=-1
    while time.monotonic()<deadline:
        if st["state"]==STATE_FAILED:print("Kalibrasi rotate gagal.",file=sys.stderr);return 2
        if st["state"]==STATE_DONE:print("Kalibrasi accelerometer sudah DONE/tersimpan.");return 0
        if st["coverage"]==0x3F:
            print("Enam sisi lengkap. Validasi dan simpan EEPROM ...")
            result=finish_rotate_robust(link);print_status(result)
            return 0 if result["state"]==STATE_DONE and result["status"]==0 else 2
        time.sleep(.30); st=link.status(30.0)
        if st["coverage"]!=last:print_status(st);last=st["coverage"]
    print("Timeout: enam sisi belum lengkap.",file=sys.stderr);return 3

def show_live(data):
    ax,ay,az=data["accel_raw"];gx,gy,gz=data["gyro_raw"]
    print(f"RPY=({data['roll']:+7.3f},{data['pitch']:+7.3f},{data['yaw']:+7.3f})deg "
          f"V=({data['vel'][0]:+.3f},{data['vel'][1]:+.3f},{data['vel'][2]:+.3f})m/s "
          f"RAW A=({ax},{ay},{az}) G=({gx},{gy},{gz}) [{flag_text(data['flags'])}]")

def interactive(link,every=20):
    print("\nLIVE: still | rotate-start | rotate-finish | status | zero-nav | zupt | stationary-on | stationary-off | cancel | q")
    valid=0
    while True:
        try:
            p=link.ensure()
            if select.select([sys.stdin],[],[],0)[0]:
                cmd=sys.stdin.readline().strip().lower()
                if cmd in ("q","quit","exit"):return 0
                if cmd=="still":run_still(link);continue
                if cmd=="rotate-start":run_rotate(link);continue
                if cmd in CMDS:print_status(link.command_retry(CMDS[cmd]));continue
            payload,_=read_frame(p);data=decode_imu(payload) if payload else None
            if data:
                valid+=1
                if valid==1 or valid%max(every,1)==0:show_live(data)
        except (serial.SerialException,OSError,FileNotFoundError):
            link.close();print("Serial terputus; monitor reconnect otomatis ...",file=sys.stderr);time.sleep(.2)

def main():
    ap=argparse.ArgumentParser(description="Kalibrasi IMU robust: auto reconnect + resume state MCU")
    ap.add_argument("command",nargs="?",choices=list(CMDS)+["monitor"],default="monitor")
    ap.add_argument("--port",default="auto");ap.add_argument("--baud",type=int,default=BAUD_DEFAULT)
    ap.add_argument("--every",type=int,default=20);args=ap.parse_args()
    link=CalLink(args.port,args.baud)
    try:
        link.connect()
        if args.command=="monitor":return interactive(link,args.every)
        if args.command=="still":return run_still(link)
        if args.command=="rotate-start":return run_rotate(link)
        print_status(link.command_retry(CMDS[args.command]));return 0
    finally:link.close()
if __name__=="__main__":raise SystemExit(main())
