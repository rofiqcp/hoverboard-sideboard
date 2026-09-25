#!/usr/bin/env python3
"""Kalibrasi/monitor IMU dengan auto-reconnect dan resume state firmware."""
import argparse
import queue
import sys
import threading
import time
import serial
from read_imu import crc16, read_frame, decode_imu, flag_text
from serial_common import find_sideboard_port, open_sideboard_port
from flash_uart import catch_bootloader,parse_info,transact,CMD_GO

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

def _telemetry_status(data, sub=CMDS["status"]):
    if not data:return None
    state,coverage,error,progress=data["cal"]
    return {"sub":sub,"status":0,"state":state,"coverage":coverage,
            "error":error,"progress":progress/10.0,"source":"telemetry"}

class CalLink:
    """Satu ownership serial persistent.

    Timeout protocol TIDAK menutup COM. Port hanya ditutup saat pyserial memberi
    error I/O nyata (unplug/reset USB). Kalibrasi dipantau terutama dari telemetry
    50 Hz sehingga ACK command yang terlambat/hilang tidak memicu reconnect storm.
    """
    def __init__(self, requested="auto", baud=BAUD_DEFAULT):
        self.requested=requested; self.baud=baud; self.port=None; self.last_name=None
        self.last_rx=time.monotonic(); self.last_notice=0.0; self.last_boot_probe=0.0
    def close(self):
        if self.port is not None:
            try:self.port.close()
            except Exception:pass
        self.port=None
    def connect(self, announce=True):
        self.close()
        self.port=open_sideboard_port(self.requested,self.baud,timeout=.025,attempts=3,delay=.20)
        self.last_name=self.port.port
        try:self.port.reset_input_buffer()
        except Exception:pass
        self.last_rx=time.monotonic()
        if announce: print(f"Port {self.last_name} terbuka @ {self.baud}; sinkronisasi STM32 ...")
        return self.port
    def ensure(self):
        if self.port is None or not self.port.is_open:self.connect()
        return self.port
    def _io_fault(self, exc):
        self.close()
        now=time.monotonic()
        if now-self.last_notice>1.0:
            print(f"USB/serial benar-benar terputus ({exc}); menunggu hotplug ...",file=sys.stderr)
            self.last_notice=now
        time.sleep(.20)
    def _recover_from_bootloader_if_needed(self):
        """Saat link silent, deteksi bootloader dan GO ke aplikasi bila image valid."""
        if self.port is None or not self.port.is_open:return False
        now=time.monotonic()
        if now-self.last_rx<2.0 or now-self.last_boot_probe<3.0:return False
        self.last_boot_probe=now
        try:
            info=catch_bootloader(self.port,.45)
            version,start,end,page,chunk,app_valid=parse_info(info)
            if not app_valid:
                print(f"Bootloader v{version} terdeteksi tetapi app tidak valid; tidak GO.",file=sys.stderr)
                return False
            print(f"Bootloader v{version} terdeteksi; app valid. GO otomatis ke aplikasi ...",file=sys.stderr)
            transact(self.port,bytes((CMD_GO,)),CMD_GO,timeout=.8)
            self.close()
            time.sleep(.45)
            return True
        except TimeoutError:
            return False
        except (serial.SerialException,OSError,FileNotFoundError) as exc:
            self._io_fault(exc)
            return False
    def _read_status_frame(self, deadline, ack_sub=None, accepted_states=None):
        """Baca ACK atau telemetry kalibrasi sampai deadline pada sesi yang sama."""
        while time.monotonic()<deadline:
            try:
                p=self.ensure()
                payload,err=read_frame(p)
            except (serial.SerialException,OSError,FileNotFoundError) as exc:
                self._io_fault(exc); continue
            if not payload:
                if self._recover_from_bootloader_if_needed():
                    continue
                continue
            self.last_rx=time.monotonic()
            st=parse_cal_status(payload)
            if st:
                st["source"]="ack"
                if ack_sub is None or st["sub"]==ack_sub:return st
                continue
            data=decode_imu(payload)
            if data:
                st=_telemetry_status(data,ack_sub or CMDS["status"])
                # Passive status may come directly from telemetry. For a real
                # command (ZERO_NAV/STATIONARY/...) telemetry is NOT an ACK;
                # only START calibration may be confirmed by authoritative state.
                if ack_sub is None:return st
                if accepted_states is not None and st["state"] in accepted_states:return st
        return None
    def one_request(self, sub, timeout=1.2, accepted_states=None):
        """Kirim sekali. ACK boleh digantikan bukti state dari telemetry."""
        try:
            p=self.ensure()
            p.write(packet(bytes((COMM_CAL,sub)))); p.flush()
        except (serial.SerialException,OSError,FileNotFoundError) as exc:
            self._io_fault(exc)
            raise
        st=self._read_status_frame(time.monotonic()+timeout,sub,accepted_states)
        if st:return st
        raise TimeoutError(f"Belum ada ACK/state untuk command kalibrasi {sub}")
    def status(self, overall=10.0):
        """Utamakan telemetry pasif; STATUS request hanya fallback.

        status=7 berarti STM32 hidup tetapi estimator belum READY. Tetap gunakan
        sesi serial yang sama dan tunggu; jangan close/reopen COM.
        """
        end=time.monotonic()+overall; last=None; next_request=time.monotonic()+0.35
        while time.monotonic()<end:
            # Coba konsumsi telemetry yang sudah mengalir tanpa command.
            slice_end=min(end,next_request)
            st=self._read_status_frame(slice_end)
            if st and st.get("status",0)!=7:return st
            if time.monotonic()>=next_request:
                try:
                    st=self.one_request(CMDS["status"],0.70)
                    if st["status"]!=7:return st
                    last=RuntimeError("STM32 hidup, estimator belum READY (BUSY=7)")
                except TimeoutError as exc:
                    # Timeout protokol bukan disconnect: pertahankan handle.
                    last=exc
                except (serial.SerialException,OSError,FileNotFoundError) as exc:
                    last=exc
                next_request=time.monotonic()+0.35
                now=time.monotonic()
                if now-self.last_notice>2.0:
                    age=now-self.last_rx
                    print(f"Menunggu respons STM32 pada {self.last_name} (silent {age:.1f}s); COM tetap dibuka ...",
                          file=sys.stderr)
                    self.last_notice=now
        raise TimeoutError(f"STM32 belum memberi status dalam {overall:.1f}s: {last}")
    def command_retry(self, sub, overall=15.0, accepted_states=None):
        """Retry idempotent pada sesi sama; reconnect hanya jika USB benar-benar error."""
        end=time.monotonic()+overall; last=None
        while time.monotonic()<end:
            try:
                st=self.one_request(sub,min(1.5,max(.2,end-time.monotonic())),accepted_states)
                if st.get("status",0)==7:
                    last=RuntimeError("STM32 BUSY/booting")
                else:
                    return st
            except TimeoutError as exc:
                last=exc
            except (serial.SerialException,OSError,FileNotFoundError) as exc:
                last=exc
            time.sleep(.12)
        raise TimeoutError(f"Command {sub} belum dikonfirmasi: {last}")

def start_or_resume(link, sub, target_state):
    """Mulai/resume secara dinamis; tunggu sampai STM32 benar-benar mengonfirmasi.

    Tidak ada deadline keras. Jika USB/STM32 mati sementara, tool tetap hidup dan
    akan lanjut saat komunikasi kembali. User selalu bisa Ctrl+C untuk membatalkan.
    """
    last_start=0.0
    announced=False
    while True:
        try:
            st=link.status(3.0)
            if st["state"]==target_state:
                if not announced:
                    print("Firmware sudah menjalankan kalibrasi; RESUME progress yang ada.")
                return st

            # IDLE/DONE/FAILED = belum menjalankan sesi yang diminta.
            if st["state"] in (STATE_IDLE,STATE_DONE,STATE_FAILED) and time.monotonic()-last_start>=1.0:
                last_start=time.monotonic()
                try:
                    started=link.one_request(sub,2.0,accepted_states={target_state})
                    if started["state"]==target_state:return started
                except TimeoutError:
                    print("ACK START belum terlihat; menunggu telemetry state tanpa menutup COM ...",
                          file=sys.stderr)
                except (serial.SerialException,OSError,FileNotFoundError):
                    pass
        except TimeoutError:
            if not announced:
                print("COM akan dipertahankan/reconnect otomatis sampai STM32 merespons. Ctrl+C untuk batal.",
                      file=sys.stderr)
                announced=True
            # Saat STM32 silent, START periodik tetap aman; setelah MCU hidup command
            # pertama yang diterima akan mengubah state dan telemetry menjadi bukti.
            if time.monotonic()-last_start>=2.0:
                last_start=time.monotonic()
                try:
                    started=link.one_request(sub,1.2,accepted_states={target_state})
                    if started["state"]==target_state:return started
                except (TimeoutError,serial.SerialException,OSError,FileNotFoundError):
                    pass

def finish_rotate_robust(link):
    accepted={STATE_DONE,STATE_FAILED}
    last_send=0.0
    while True:
        try:
            st=link.status(3.0)
            if st["state"] in accepted:return st
            if st["state"]==STATE_ROTATE and st["coverage"]==0x3F and time.monotonic()-last_send>=1.0:
                last_send=time.monotonic()
                try:
                    done=link.one_request(CMDS["rotate-finish"],2.0,accepted_states=accepted)
                    if done["state"] in accepted:return done
                except (TimeoutError,serial.SerialException,OSError,FileNotFoundError):
                    print("ACK FINISH belum terlihat; tetap menunggu state firmware ...",file=sys.stderr)
        except TimeoutError:
            # USB/MCU boleh hilang sementara; CalLink akan reconnect / bootloader-GO.
            pass

def run_still(link):
    print("\nKalibrasi diam: jangan sentuh atau gerakkan board sampai selesai.")
    st=start_or_resume(link,CMDS["still"],STATE_STILL); print_status(st)
    last=-1
    while True:
        if st["state"]==STATE_DONE:
            print("Kalibrasi diam selesai dan firmware telah menyimpan hasil ke EEPROM.");return 0
        if st["state"]==STATE_FAILED:
            print("Kalibrasi diam gagal. Pastikan board benar-benar diam.",file=sys.stderr);return 2
        time.sleep(.20)
        try:
            st=link.status(5.0)
        except TimeoutError:
            print("STM32 sementara belum merespons; progress dipertahankan, menunggu reconnect ...",
                  file=sys.stderr)
            continue
        if int(st["progress"])!=last or st["state"] in (STATE_DONE,STATE_FAILED):
            print_status(st);last=int(st["progress"])

def run_rotate(link):
    print("\nKalibrasi 6 sisi accelerometer.")
    print("Putar perlahan lalu TAHAN board pada +X, -X, +Y, -Y, +Z, -Z.")
    print("Jika USB/STM reset, tool tetap hidup dan akan resume state firmware saat kembali.")
    st=start_or_resume(link,CMDS["rotate-start"],STATE_ROTATE);print_status(st)
    last=-1
    while True:
        if st["state"]==STATE_FAILED:print("Kalibrasi rotate gagal.",file=sys.stderr);return 2
        if st["state"]==STATE_DONE:print("Kalibrasi accelerometer sudah DONE/tersimpan.");return 0
        if st["coverage"]==0x3F:
            print("Enam sisi lengkap. Validasi dan simpan EEPROM ...")
            result=finish_rotate_robust(link);print_status(result)
            return 0 if result["state"]==STATE_DONE and result["status"]==0 else 2
        time.sleep(.20)
        try:
            st=link.status(5.0)
        except TimeoutError:
            print("STM32 sementara belum merespons; coverage dipertahankan, menunggu reconnect ...",
                  file=sys.stderr)
            continue
        if st["coverage"]!=last:print_status(st);last=st["coverage"]

def show_live(data):
    ax,ay,az=data["accel_raw"];gx,gy,gz=data["gyro_raw"]
    print(f"RPY=({data['roll']:+7.3f},{data['pitch']:+7.3f},{data['yaw']:+7.3f})deg "
          f"V=({data['vel'][0]:+.3f},{data['vel'][1]:+.3f},{data['vel'][2]:+.3f})m/s "
          f"RAW A=({ax},{ay},{az}) G=({gx},{gy},{gz}) [{flag_text(data['flags'])}]")

_INPUT_QUEUE = queue.Queue()
_STDIN_THREAD = None

def _stdin_worker():
    """Blocking reader hanya aktif pada mode monitor interaktif."""
    while True:
        line=sys.stdin.readline()
        if line=="": return
        _INPUT_QUEUE.put(line.strip().lower())

def _ensure_stdin_thread():
    global _STDIN_THREAD
    if _STDIN_THREAD is None or not _STDIN_THREAD.is_alive():
        _STDIN_THREAD=threading.Thread(target=_stdin_worker,name="cal-console-input",daemon=True)
        _STDIN_THREAD.start()

def _poll_command():
    try:return _INPUT_QUEUE.get_nowait()
    except queue.Empty:return None

def interactive(link,every=20):
    _ensure_stdin_thread()
    print("\nLIVE: still | rotate-start | rotate-finish | status | zero-nav | zupt | stationary-on | stationary-off | cancel | q")
    valid=0
    while True:
        try:
            p=link.ensure()
            cmd=_poll_command()
            if cmd:
                if cmd in ("q","quit","exit"):return 0
                if cmd=="still":run_still(link);continue
                if cmd=="rotate-start":run_rotate(link);continue
                if cmd in CMDS:print_status(link.command_retry(CMDS[cmd]));continue
                print(f"Command tidak dikenal: {cmd}",file=sys.stderr)
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
        print(f"Target serial: {args.port} @ {args.baud} (hotplug/reconnect otomatis)")
        if args.command=="monitor":return interactive(link,args.every)
        if args.command=="still":return run_still(link)
        if args.command=="rotate-start":return run_rotate(link)
        print_status(link.command_retry(CMDS[args.command]));return 0
    finally:link.close()
if __name__=="__main__":
    from run_csv import run_logged
    raise SystemExit(run_logged(main,__file__))