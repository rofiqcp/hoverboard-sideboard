#!/usr/bin/env python3
"""Pembaca telemetry IMU sideboard.

Program ini membaca frame pendek bergaya VESC dari serial, memeriksa CRC,
lalu menampilkan data mentah MPU6xxx dan hasil attitude ESKF.
"""

import argparse
import struct
import time
import serial
import weakref
from serial_common import find_sideboard_port, open_sideboard_port

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
    """Compatibility helper; read_frame memakai sliding buffer yang lebih robust."""
    data = bytearray()
    deadline = time.monotonic() + 0.25
    while len(data) < count and time.monotonic() < deadline:
        data.extend(port.read(count - len(data)))
    return bytes(data)


_RX_BUFFERS = weakref.WeakKeyDictionary()

def read_frame(port: serial.Serial):
    """Sliding VESC frame parser yang tahan buka-port di tengah stream/byte corrupt.

    Kandidat start palsu (0x02 di dalam payload) hanya membuang satu byte lalu
    parser scan ulang. Ini mencegah satu false start membuang frame valid berikutnya.
    """
    buf = _RX_BUFFERS.get(port)
    if buf is None:
        buf = bytearray(); _RX_BUFFERS[port] = buf
    deadline = time.monotonic() + 0.30
    last_error = "timeout"
    while time.monotonic() < deadline:
        while True:
            try:
                start = buf.index(2)
            except ValueError:
                buf.clear(); break
            if start:
                del buf[:start]
            if len(buf) < 2:
                break
            n = buf[1]
            if n == 0 or n > 192:
                del buf[0]; last_error = "length"; continue
            total = 2 + n + 3
            if len(buf) < total:
                break
            payload = bytes(buf[2:2+n])
            crc_rx = (buf[2+n] << 8) | buf[3+n]
            stop = buf[4+n]
            if stop == 3 and crc16(payload) == crc_rx:
                del buf[:total]
                return payload, None
            last_error = "crc" if stop == 3 else "stop"
            # Jangan buang total kandidat: start ini mungkin berasal dari payload
            # frame parsial. Geser satu byte agar start asli berikutnya tetap ditemukan.
            del buf[0]
        chunk = port.read(256)
        if chunk:
            buf.extend(chunk)
            if len(buf) > 1024:
                del buf[:-512]
    return None, last_error


def decode_imu(payload: bytes):
    """Decode protocol v2 (79), v3 (101), dan v4 validity (104 byte)."""
    if not payload or payload[0] != COMM_SIDEBOARD_IMU or len(payload) not in (79, 101, 104, 114):
        return None
    if len(payload) == 79:
        fields = struct.unpack(">BBHII7h12iBBBH", payload)
        extra = None
    elif len(payload) == 101:
        fields = struct.unpack(">BBHII7h12iBBB12H", payload)
        extra = fields[28:39]
    else:
        fields = struct.unpack(">BBHII7h12iBBB12HBH", payload[:104])
        extra = fields[28:39]
    command, version, flags, seq, time_us = fields[:5]
    ax, ay, az, temp, gx, gy, gz = fields[5:12]
    roll_md, pitch_md, yaw_md = fields[12:15]
    vx,vy,vz = fields[15:18]
    px,py,pz = fields[18:21]
    lax,lay,laz = fields[21:24]
    cal_state,coverage,cal_error,cal_progress = fields[24:28]
    data = {
        "command": command, "version": version, "flags": flags,
        "seq": seq, "time_us": time_us,
        "accel_raw": (ax, ay, az), "temp_raw": temp, "gyro_raw": (gx, gy, gz),
        "roll": roll_md / 1000.0, "pitch": pitch_md / 1000.0, "yaw": yaw_md / 1000.0,
        "vel": (vx/1000.0,vy/1000.0,vz/1000.0), "pos": (px/1000.0,py/1000.0,pz/1000.0),
        "linacc": (lax/1000.0,lay/1000.0,laz/1000.0), "cal": (cal_state,coverage,cal_error,cal_progress),
        "aid_age_ms": 65535, "aid_reject": 0,
        "att_std_deg": (0.0,0.0,0.0), "vel_std": (0.0,0.0,0.0), "pos_std": (0.0,0.0,0.0),
        "nav_status": 0, "health_resets": 0,
        "temp_c": temp / 340.0 + 36.53, "imu_whoami": 0, "imu_class": 0, "observed_sample_hz": 0.0,
    }
    if extra:
        data["aid_age_ms"], data["aid_reject"] = extra[:2]
        data["att_std_deg"] = tuple(x/1000.0 for x in extra[2:5])
        data["vel_std"] = tuple(x/1000.0 for x in extra[5:8])
        data["pos_std"] = tuple(x/1000.0 for x in extra[8:11])
    if len(payload) in (104,114):
        data["nav_status"] = fields[39]
        data["health_resets"] = fields[40]
    if len(payload) == 114:
        temp_md,who,klass,rate_mhz=struct.unpack(">iBBI",payload[104:114])
        data["temp_c"]=temp_md/1000.0; data["imu_whoami"]=who; data["imu_class"]=klass; data["observed_sample_hz"]=rate_mhz/1000.0
    return data

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
    if flags & (1 << 11): names.append("wheel_aid")
    if flags & (1 << 12): names.append("nhc")
    if flags & (1 << 13): names.append("yaw_aid")
    if flags & (1 << 14): names.append("vel_aid")
    if flags & (1 << 15): names.append("pos_aid")
    return ",".join(names)


def nav_text(status: int) -> str:
    names=[]
    if status & 1: names.append("att_valid")
    if status & 2: names.append("vel_aided")
    if status & 4: names.append("pos_aided")
    if status & 8: names.append("yaw_aided")
    if status & 16: names.append("dead_reckoning")
    if status & 32: names.append("cov_valid")
    if status & 64: names.append("stationary_bound")
    if status & 128: names.append("health_recovered")
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
    with open_sideboard_port(selected_port, args.baud, timeout=0.05) as port:
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
                    temp_c = data["temp_c"]
                    print(
                        f"seq={data['seq']:8d} t={data['time_us']:10d}us "
                        f"RPY=({data['roll']:8.3f},{data['pitch']:8.3f},{data['yaw']:8.3f})deg "
                        f"Araw=({ax:6d},{ay:6d},{az:6d}) "
                        f"Graw=({gx:6d},{gy:6d},{gz:6d}) Traw={data['temp_raw']:6d} "
                        f"A[g]=({ag[0]:+.3f},{ag[1]:+.3f},{ag[2]:+.3f}) "
                        f"G[dps]=({gd[0]:+.2f},{gd[1]:+.2f},{gd[2]:+.2f}) "
                        f"V=({data['vel'][0]:+.3f},{data['vel'][1]:+.3f},{data['vel'][2]:+.3f})m/s "
                        f"P=({data['pos'][0]:+.3f},{data['pos'][1]:+.3f},{data['pos'][2]:+.3f})m "
                        f"stdV=({data['vel_std'][0]:.3f},{data['vel_std'][1]:.3f},{data['vel_std'][2]:.3f}) "
                        f"aid_age={data['aid_age_ms']}ms rej={data['aid_reject']} "
                        f"T={temp_c:.2f}C imu=0x{data['imu_whoami']:02X}/c{data['imu_class']} rate={data['observed_sample_hz']:.2f}Hz "
                        f"flags=0x{data['flags']:04X} [{flag_text(data['flags'])}] "
                        f"nav=0x{data['nav_status']:02X}[{nav_text(data['nav_status'])}] resets={data['health_resets']}"
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
