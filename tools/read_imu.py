#!/usr/bin/env python3
"""Pembaca telemetry IMU sideboard.

Program ini membaca frame pendek bergaya VESC dari serial, memeriksa CRC,
lalu menampilkan data mentah MPU6xxx dan hasil attitude ESKF.
"""

import argparse
import struct
import time
import serial
from serial_common import find_sideboard_port

PORT_DEFAULT = "auto"
BAUD_DEFAULT = 921600
COMM_SIDEBOARD_IMU = 0xF0


def crc16(data: bytes) -> int:
    """CRC16-CCITT 0x1021 initial 0, sama dengan firmware VESC."""
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def read_exact(port: serial.Serial, count: int) -> bytes:
    """Baca tepat sejumlah byte atau hasil kosong saat timeout."""
    data = bytearray()
    deadline = time.monotonic() + 0.25
    while len(data) < count and time.monotonic() < deadline:
        data.extend(port.read(count - len(data)))
    return bytes(data)

def read_frame(port: serial.Serial):
    """Cari dan validasi satu frame VESC pendek: 2,len,payload,crc_hi,crc_lo,3."""
    while True:
        first = port.read(1)
        if not first:
            return None, "timeout"
        if first[0] != 2:
            continue

        length_b = read_exact(port, 1)
        if len(length_b) != 1:
            return None, "timeout"
        length = length_b[0]
        if length == 0 or length > 192:
            continue

        rest = read_exact(port, length + 3)
        if len(rest) != length + 3:
            return None, "timeout"
        payload = rest[:length]
        crc_rx = (rest[length] << 8) | rest[length + 1]
        stop = rest[length + 2]
        if stop != 3:
            return None, "stop"
        if crc16(payload) != crc_rx:
            return None, "crc"
        return payload, None


def decode_imu(payload: bytes):
    """Ubah payload binary menjadi dictionary yang mudah dibaca manusia."""
    if len(payload) != 79 or payload[0] != COMM_SIDEBOARD_IMU:
        return None
    fields = struct.unpack(">BBHII7h12iBBBH", payload)
    command, version, flags, seq, time_us = fields[:5]
    ax, ay, az, temp, gx, gy, gz = fields[5:12]
    roll_md, pitch_md, yaw_md = fields[12:15]
    vx,vy,vz = fields[15:18]
    px,py,pz = fields[18:21]
    lax,lay,laz = fields[21:24]
    cal_state,coverage,cal_error,cal_progress = fields[24:28]
    return {
        "command": command, "version": version, "flags": flags,
        "seq": seq, "time_us": time_us,
        "accel_raw": (ax, ay, az), "temp_raw": temp, "gyro_raw": (gx, gy, gz),
        "roll": roll_md / 1000.0, "pitch": pitch_md / 1000.0, "yaw": yaw_md / 1000.0,
        "vel": (vx/1000.0,vy/1000.0,vz/1000.0), "pos": (px/1000.0,py/1000.0,pz/1000.0),
        "linacc": (lax/1000.0,lay/1000.0,laz/1000.0), "cal": (cal_state,coverage,cal_error,cal_progress),
    }

def flag_text(flags: int) -> str:
    """Terjemahkan bit status firmware ke tulisan ringkas bahasa Indonesia."""
    names = []
    if flags & (1 << 0): names.append("sensor_ok")
    if flags & (1 << 1): names.append("eskf_ok")
    if flags & (1 << 2): names.append("accel_fused")
    if flags & (1 << 3): names.append("eeprom_valid")
    if flags & (1 << 4): names.append("startup_diam")
    if flags & (1 << 5): names.append("yaw_absolut")
    else: names.append("yaw_drift")
    if flags & (1 << 6): names.append("still_cal_valid")
    if flags & (1 << 7): names.append("rotate_cal_valid")
    if flags & (1 << 8): names.append("cal_active")
    if flags & (1 << 9): names.append("zupt")
    if flags & (1 << 10): names.append("master_stationary")
    return ",".join(names)


def main():
    parser = argparse.ArgumentParser(description="Baca raw MPU6xxx-compatible + roll/pitch/yaw ESKF")
    parser.add_argument("--port", default=PORT_DEFAULT)
    parser.add_argument("--baud", type=int, default=BAUD_DEFAULT)
    parser.add_argument("--every", type=int, default=5,
                        help="Tampilkan tiap N frame; default 5 = sekitar 10 Hz dari stream 50 Hz")
    parser.add_argument("--count", type=int, default=0,
                        help="Berhenti setelah jumlah frame valid ini; 0 berarti terus")
    args = parser.parse_args()

    valid = crc_errors = other_errors = 0
    last_seq = None
    lost = 0
    first_time_us = None
    last_time_us = None
    started = time.monotonic()

    selected_port = find_sideboard_port(args.port)
    with serial.Serial(selected_port, args.baud, timeout=0.05) as port:
        port.reset_input_buffer()
        print(f"Membaca {selected_port} @ {args.baud} baud ...")
        try:
            while args.count == 0 or valid < args.count:
                payload, error = read_frame(port)
                if error:
                    if error == "crc": crc_errors += 1
                    elif error != "timeout": other_errors += 1
                    continue
                data = decode_imu(payload)
                if data is None:
                    continue

                if last_seq is not None:
                    delta = (data["seq"] - last_seq) & 0xFFFFFFFF
                    if delta > 1:
                        lost += delta - 1
                last_seq = data["seq"]
                if first_time_us is None:
                    first_time_us = data["time_us"]
                last_time_us = data["time_us"]
                valid += 1

                if valid == 1 or valid % max(args.every, 1) == 0:
                    ax, ay, az = data["accel_raw"]
                    gx, gy, gz = data["gyro_raw"]
                    ag = (ax / 8192.0, ay / 8192.0, az / 8192.0)
                    gd = (gx / 65.5, gy / 65.5, gz / 65.5)
                    temp_c = data["temp_raw"] / 340.0 + 36.53
                    print(
                        f"seq={data['seq']:8d} t={data['time_us']:10d}us "
                        f"RPY=({data['roll']:8.3f},{data['pitch']:8.3f},{data['yaw']:8.3f})deg "
                        f"Araw=({ax:6d},{ay:6d},{az:6d}) "
                        f"Graw=({gx:6d},{gy:6d},{gz:6d}) Traw={data['temp_raw']:6d} "
                        f"A[g]=({ag[0]:+.3f},{ag[1]:+.3f},{ag[2]:+.3f}) "
                        f"G[dps]=({gd[0]:+.2f},{gd[1]:+.2f},{gd[2]:+.2f}) "
                        f"V=({data['vel'][0]:+.3f},{data['vel'][1]:+.3f},{data['vel'][2]:+.3f})m/s "
                        f"P=({data['pos'][0]:+.3f},{data['pos'][1]:+.3f},{data['pos'][2]:+.3f})m "
                        f"T={temp_c:.2f}C flags=0x{data['flags']:04X} [{flag_text(data['flags'])}]"
                    )
        except KeyboardInterrupt:
            pass

    elapsed = max(time.monotonic() - started, 1e-6)
    if valid > 1 and first_time_us is not None and last_time_us is not None:
        sensor_elapsed = ((last_time_us - first_time_us) & 0xFFFFFFFF) * 1.0e-6
        stream_hz = (valid - 1) / max(sensor_elapsed, 1e-6)
    else:
        stream_hz = 0.0
    print(f"Ringkasan: valid={valid}, hilang_seq={lost}, crc_error={crc_errors}, "
          f"error_lain={other_errors}, stream={stream_hz:.1f} Hz, "
          f"laju_host_total={valid/elapsed:.1f} frame/s")


if __name__ == "__main__":
    main()
