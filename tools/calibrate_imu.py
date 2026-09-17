#!/usr/bin/env python3
"""Console kalibrasi + monitor sideboard IMU.

Tanpa argumen program langsung mencari CH340, membuka telemetry, dan siap menerima
command kalibrasi dari keyboard. Semua command memakai framing VESC + CRC16.
"""
import argparse
import select
import sys
import time

import serial
from read_imu import crc16, read_frame, decode_imu, flag_text
from serial_common import find_sideboard_port, open_sideboard_port

BAUD_DEFAULT = 921600
COMM_CAL = 0xF2
CMDS = {
    "still": 1,
    "rotate-start": 2,
    "rotate-finish": 3,
    "cancel": 4,
    "zero-nav": 5,
    "zupt": 6,
    "status": 7,
    "stationary-on": 8,
    "stationary-off": 9,
}
FACE_NAMES = ["+X", "-X", "+Y", "-Y", "+Z", "-Z"]


def packet(payload: bytes) -> bytes:
    c = crc16(payload)
    return bytes((2, len(payload))) + payload + bytes((c >> 8, c & 0xFF, 3))


def parse_cal_status(payload):
    if not payload or len(payload) < 8 or payload[0] != COMM_CAL:
        return None
    progress = ((payload[6] << 8) | payload[7]) / 10.0
    return {
        "sub": payload[1], "status": payload[2], "state": payload[3],
        "coverage": payload[4], "error": payload[5], "progress": progress,
    }


def coverage_text(mask: int) -> str:
    done = [FACE_NAMES[i] for i in range(6) if mask & (1 << i)]
    missing = [FACE_NAMES[i] for i in range(6) if not mask & (1 << i)]
    return f"selesai={','.join(done) or '-'} | kurang={','.join(missing) or '-'}"


def print_status(st):
    states = {0: "IDLE", 1: "STILL", 2: "ROTATE", 3: "DONE", 4: "FAILED"}
    print(f"CAL {states.get(st['state'], st['state'])} | status={st['status']} "
          f"progress={st['progress']:.1f}% coverage=0x{st['coverage']:02X} "
          f"error={st['error']} | {coverage_text(st['coverage'])}")


def send_command(port, sub: int, timeout: float = 2.0):
    port.write(packet(bytes((COMM_CAL, sub))))
    port.flush()
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        payload, _ = read_frame(port)
        st = parse_cal_status(payload)
        if st and st["sub"] == sub:
            return st
    raise TimeoutError(f"Tidak ada ACK command kalibrasi {sub}")


def get_status(port):
    return send_command(port, CMDS["status"], 1.0)


def run_still(port):
    print("\nKalibrasi diam: jangan sentuh atau gerakkan board sampai selesai.")
    print_status(send_command(port, CMDS["still"]))
    deadline = time.monotonic() + 20.0
    last_progress = -1
    while time.monotonic() < deadline:
        time.sleep(0.35)
        st = get_status(port)
        if int(st["progress"]) != last_progress or st["state"] in (3, 4):
            print_status(st)
            last_progress = int(st["progress"])
        if st["state"] == 3:
            print("Kalibrasi diam selesai dan firmware telah menyimpan hasil ke EEPROM.")
            return 0
        if st["state"] == 4:
            print("Kalibrasi diam gagal. Pastikan board benar-benar diam.", file=sys.stderr)
            return 2
    print("Timeout menunggu kalibrasi diam.", file=sys.stderr)
    return 3


def run_rotate(port):
    print("\nKalibrasi 6 sisi accelerometer.")
    print("Putar perlahan lalu TAHAN board pada +X, -X, +Y, -Y, +Z, -Z.")
    print("Coverage baru dianggap selesai setelah tiap sisi stabil cukup lama.")
    print_status(send_command(port, CMDS["rotate-start"]))
    deadline = time.monotonic() + 180.0
    last_mask = -1
    while time.monotonic() < deadline:
        time.sleep(0.35)
        st = get_status(port)
        if st["coverage"] != last_mask:
            print_status(st)
            last_mask = st["coverage"]
        if st["state"] == 4:
            print("Kalibrasi rotate gagal sebelum selesai.", file=sys.stderr)
            return 2
        if st["coverage"] == 0x3F:
            print("Enam sisi lengkap. Validasi parameter dan simpan EEPROM ...")
            result = send_command(port, CMDS["rotate-finish"], 3.0)
            print_status(result)
            if result["status"] == 0:
                print("Kalibrasi accelerometer selesai dan tersimpan ke EEPROM.")
                return 0
            return 2
    print("Timeout: enam sisi belum lengkap.", file=sys.stderr)
    return 3


def show_live(data):
    ax, ay, az = data["accel_raw"]
    gx, gy, gz = data["gyro_raw"]
    print(f"RPY=({data['roll']:+7.3f},{data['pitch']:+7.3f},{data['yaw']:+7.3f})deg "
          f"V=({data['vel'][0]:+.3f},{data['vel'][1]:+.3f},{data['vel'][2]:+.3f})m/s "
          f"P=({data['pos'][0]:+.3f},{data['pos'][1]:+.3f},{data['pos'][2]:+.3f})m "
          f"RAW A=({ax},{ay},{az}) G=({gx},{gy},{gz}) "
          f"[{flag_text(data['flags'])}]")


def interactive(port, every=20):
    print("\nLIVE IMU siap. Ketik command lalu Enter kapan saja:")
    print(" still | rotate-start | rotate-finish | status | zero-nav | zupt")
    print(" stationary-on | stationary-off | cancel | q")
    valid = 0
    while True:
        if select.select([sys.stdin], [], [], 0)[0]:
            cmd = sys.stdin.readline().strip().lower()
            if cmd in ("q", "quit", "exit"):
                return 0
            if cmd == "still":
                run_still(port)
                continue
            if cmd == "rotate-start":
                run_rotate(port)
                continue
            if cmd in CMDS:
                try:
                    print_status(send_command(port, CMDS[cmd]))
                except TimeoutError as exc:
                    print(exc, file=sys.stderr)
                continue
            if cmd:
                print(f"Command tidak dikenal: {cmd}")

        payload, _ = read_frame(port)
        data = decode_imu(payload) if payload else None
        if data:
            valid += 1
            if valid == 1 or valid % max(every, 1) == 0:
                show_live(data)


def main():
    ap = argparse.ArgumentParser(description="Kalibrasi dan monitor ESKF sideboard IMU")
    ap.add_argument("command", nargs="?", choices=list(CMDS) + ["monitor"], default="monitor")
    ap.add_argument("--port", default="auto", help="default auto: by-id lalu ttyUSB0")
    ap.add_argument("--baud", type=int, default=BAUD_DEFAULT)
    ap.add_argument("--every", type=int, default=20, help="interval tampilan frame pada mode monitor")
    args = ap.parse_args()

    port_name = find_sideboard_port(args.port)
    print(f"Sideboard: {port_name} @ {args.baud} baud")
    with open_sideboard_port(port_name, args.baud, timeout=0.03) as port:
        port.reset_input_buffer()
        if args.command == "monitor":
            return interactive(port, args.every)
        if args.command == "still":
            return run_still(port)
        if args.command == "rotate-start":
            return run_rotate(port)
        print_status(send_command(port, CMDS[args.command]))
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
