#!/usr/bin/env python3
"""Pembaca telemetry IMU sideboard.

Program ini membaca frame pendek bergaya VESC dari serial, memeriksa CRC,
lalu menampilkan data mentah MPU6xxx dan hasil attitude ESKF.
"""

import argparse
import struct
import sys
import time
import serial
import weakref
from serial_common import open_sideboard_port, sideboard_candidates
from flash_uart import catch_bootloader,parse_info,transact,CMD_GO

PORT_DEFAULT = "auto"
BAUD_DEFAULT = 921600

ACCEL_LSB_PER_G_BY_FS = {0: 16384.0, 1: 8192.0, 2: 4096.0, 3: 2048.0}
GYRO_LSB_PER_DPS_BY_FS = {0: 131.0, 1: 65.5, 2: 32.8, 3: 16.4}
COMM_SIDEBOARD_IMU = 0xF0
COMM_SIDEBOARD_DIAG = 0xF5


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


def _nested_frame_start(buf):
    """Locate CRC-valid frame behind a truncated hotplug prefix."""
    for pos in range(1, max(1, len(buf) - 4)):
        if buf[pos] != 2 or pos + 5 > len(buf):
            continue
        n = buf[pos+1]
        if not 0 < n <= 192 or pos+n+5 > len(buf):
            continue
        q = pos+2+n
        if buf[q+2] == 3 and crc16(bytes(buf[pos+2:q])) == (
                (buf[q] << 8) | buf[q+1]):
            return pos
    return None


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
                nested = _nested_frame_start(buf)
                if nested is not None:
                    del buf[:nested]
                    continue
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


def open_telemetry_port(requested="auto", baud=BAUD_DEFAULT, timeout=0.05,
                        attempts=5, delay=0.20):
    """Buka port yang benar-benar teridentifikasi sebagai sideboard.

    Untuk port eksplisit, hormati pilihan user. Untuk auto, setiap kandidat
    harus mengirim telemetry F0/F5 valid secara pasif. Ini mencegah
    ESC/Android/serial lain menerima probe aktif atau terpilih keliru.
    """
    if requested not in ("auto", "AUTO", None):
        return open_sideboard_port(requested, baud, timeout=timeout,
                                   attempts=attempts, delay=delay)
    last_error = None
    for outer in range(max(attempts, 1)):
        candidates = sideboard_candidates()
        for candidate in candidates:
            port = None
            try:
                port = open_sideboard_port(candidate, baud, timeout=timeout,
                                           attempts=1, delay=delay)
                port.reset_input_buffer()
                payload, _ = read_frame(port)
                if decode_imu(payload) is not None or decode_diag(payload) is not None:
                    accepted = port
                    port = None
                    return accepted

            except (FileNotFoundError, serial.SerialException, OSError, RuntimeError) as exc:
                last_error = exc
            finally:
                try:
                    if port is not None and port.is_open:
                        port.close()
                except Exception:
                    pass
        if outer + 1 < attempts:
            time.sleep(delay)
    if last_error is not None:
        raise last_error
    raise FileNotFoundError("Sideboard tidak ditemukan: tidak ada kandidat yang lolos probe F0/F5/bootloader.")


def decode_imu(payload: bytes):
    """Decode telemetry v2..v6; v6 menambahkan epoch sampel IMU."""
    if not payload or payload[0] != COMM_SIDEBOARD_IMU or len(payload) not in (79, 101, 104, 114, 118):
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
    nav_scale = 10000.0 if version >= 6 else 1000.0
    data = {
        "command": command, "version": version, "flags": flags,
        "seq": seq, "time_us": time_us,
        "accel_raw": (ax, ay, az), "temp_raw": temp, "gyro_raw": (gx, gy, gz),
        "roll": roll_md / 1000.0, "pitch": pitch_md / 1000.0, "yaw": yaw_md / 1000.0,
        "vel": (vx/nav_scale,vy/nav_scale,vz/nav_scale), "pos": (px/nav_scale,py/nav_scale,pz/nav_scale),
        "linacc": (lax/nav_scale,lay/nav_scale,laz/nav_scale), "cal": (cal_state,coverage,cal_error,cal_progress),
        "aid_age_ms": 65535, "aid_reject": 0,
        "att_std_deg": (0.0,0.0,0.0), "vel_std": (0.0,0.0,0.0), "pos_std": (0.0,0.0,0.0),
        "nav_status": 0, "health_resets": 0,
        "temp_c": temp / 340.0 + 36.53, "imu_whoami": 0, "imu_class": 0, "observed_sample_hz": 0.0,
        "sample_time_us": time_us, "sensor_lag_us": 0,
        "accel_fs_sel": None, "gyro_fs_sel": None,
        "accel_lsb_per_g": 8192.0, "gyro_lsb_per_dps": 65.5,
    }
    if extra:
        data["aid_age_ms"], data["aid_reject"] = extra[:2]
        data["att_std_deg"] = tuple(x/1000.0 for x in extra[2:5])
        data["vel_std"] = tuple(x/1000.0 for x in extra[5:8])
        data["pos_std"] = tuple(x/1000.0 for x in extra[8:11])
    if len(payload) in (104,114,118):
        data["nav_status"] = fields[39]
        data["health_resets"] = fields[40]
    if len(payload) in (114,118):
        temp_md,who,klass_meta,rate_mhz=struct.unpack(">iBBI",payload[104:114])
        data["temp_c"]=temp_md/1000.0
        data["imu_whoami"]=who
        data["observed_sample_hz"]=rate_mhz/1000.0
        if len(payload) == 118:
            data["imu_class"] = klass_meta & 0x03
            accel_fs = (klass_meta >> 2) & 0x03
            gyro_fs = (klass_meta >> 4) & 0x03
            data["accel_fs_sel"] = accel_fs
            data["gyro_fs_sel"] = gyro_fs
            data["accel_lsb_per_g"] = ACCEL_LSB_PER_G_BY_FS[accel_fs]
            data["gyro_lsb_per_dps"] = GYRO_LSB_PER_DPS_BY_FS[gyro_fs]
            sample_time_us = struct.unpack(">I", payload[114:118])[0]
            data["sample_time_us"] = sample_time_us
            lag = (time_us - sample_time_us) & 0xFFFFFFFF
            data["sensor_lag_us"] = lag if lag < 0x80000000 else lag - 0x100000000
        else:
            data["imu_class"] = klass_meta
            # Legacy v5 tidak membawa FS_SEL; pertahankan compatibility profile.
            if data["observed_sample_hz"] >= 300.0:
                data["accel_lsb_per_g"] = 4096.0
                data["gyro_lsb_per_dps"] = 32.8
    return data

def decode_diag(payload: bytes):
    """Decode fixed-point diagnostic packet. v2 appends IMU-gap fields to the v1 prefix."""
    if not payload or payload[0] != COMM_SIDEBOARD_DIAG or len(payload) not in (69, 77):
        return None
    f = struct.unpack(">BBhHHHBBBB7IH4IBII", payload[:69])
    data = {
        "version": f[1],
        "wheel_innovation_mps": f[2] / 1000.0,
        "wheel_nis": f[3] / 100.0,
        "effective_wheel_sigma_mps": f[4] / 1000.0,
        "slip_score": f[5] / 1000.0,
        "slip_state": f[6],
        "accel_clip_mask": f[7],
        "gyro_clip_mask": f[8],
        "imu_config_ok": bool(f[9]),
        "accel_clip_count": f[10],
        "gyro_clip_count": f[11],
        "imu_config_mismatch_count": f[12],
        "i2c_error_count": f[13],
        "imu_reinit_count": f[14],
        "fifo_overflow_count": f[15],
        "fifo_resync_count": f[16],
        "fifo_max_bytes": f[17],
        "scheduler_miss_count": f[18],
        "max_pending_ticks": f[19],
        "uart_rx_overflow_count": f[20],
        "uart_tx_timeout_count": f[21],
        "covariance_psd_ok": bool(f[22]),
        "aiding_reject_count": f[23],
        "filter_health_reset_count": f[24],
        "imu_gap_count": 0,
        "last_imu_gap_ms": 0,
        "max_imu_gap_ms": 0,
    }
    if len(payload) == 77:
        if data["version"] < 2:
            return None
        gap_count, last_ms, max_ms = struct.unpack(">IHH", payload[69:77])
        data.update(imu_gap_count=gap_count, last_imu_gap_ms=last_ms, max_imu_gap_ms=max_ms)
    return data


def format_diag(d: dict) -> str:
    return (
        f"DIAG slip={d['slip_state']} score={d['slip_score']:.3f} "
        f"wheelInnov={d['wheel_innovation_mps']:+.3f}m/s NIS={d['wheel_nis']:.2f} "
        f"sigmaEff={d['effective_wheel_sigma_mps']:.3f}m/s "
        f"clipA=0x{d['accel_clip_mask']:02X}/{d['accel_clip_count']} "
        f"clipG=0x{d['gyro_clip_mask']:02X}/{d['gyro_clip_count']} "
        f"cfg={'ok' if d['imu_config_ok'] else 'BAD'} mismatch={d['imu_config_mismatch_count']} "
        f"i2c={d['i2c_error_count']} reinit={d['imu_reinit_count']} "
        f"fifo={d['fifo_overflow_count']}/{d['fifo_resync_count']} max={d['fifo_max_bytes']} "
        f"schedMiss={d['scheduler_miss_count']} pendingMax={d['max_pending_ticks']} "
        f"uartRxOv={d['uart_rx_overflow_count']} uartTxTO={d['uart_tx_timeout_count']} "
        f"PSD={'ok' if d['covariance_psd_ok'] else 'BAD'} "
        f"aidReject={d['aiding_reject_count']} healthReset={d['filter_health_reset_count']} "
        f"imuGap={d['imu_gap_count']} last={d['last_imu_gap_ms']}ms max={d['max_imu_gap_ms']}ms"
    )


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
    parser.add_argument("--mode", choices=("raw","ready","nav","all","diag"), default="all",
                        help="raw=sensor mentah, ready=data siap pakai, nav=x/y/z+vx/vy/vz realtime, all=diagnostik lengkap")
    args = parser.parse_args()

    valid = crc_errors = other_errors = lost = reconnects = reboots = 0
    lag_count = lag_sum_us = lag_max_us = 0
    diag_frames = 0
    last_diag = None
    last_seq = last_time_us = None
    sensor_elapsed_us = seq_advance_total = 0
    started = time.monotonic()
    interrupted = False
    last_rx_host = time.monotonic()
    last_notice = 0.0
    last_boot_probe = 0.0
    port = None
    try:
        while args.count == 0 or valid < args.count:
            try:
                if port is None or not port.is_open:
                    port = open_telemetry_port(args.port, args.baud, timeout=0.05,
                                                attempts=5, delay=0.20)
                    port.reset_input_buffer()
                    print(f"Membaca {port.port} @ {args.baud} baud ..."
                          + (" (reconnect)" if reconnects else ""))
                payload, error = read_frame(port)
                if error:
                    if error == "crc": crc_errors += 1
                    elif error != "timeout": other_errors += 1
                    if error == "timeout":
                        now=time.monotonic()
                        if now-last_rx_host > 2.0 and now-last_notice > 2.0:
                            print(f"COM {port.port} terbuka, tetapi STM32 belum mengirim telemetry; tetap menunggu ...",
                                  file=sys.stderr)
                            last_notice=now
                        if now-last_rx_host > 2.0 and now-last_boot_probe > 3.0:
                            last_boot_probe=now
                            try:
                                info=catch_bootloader(port,.40)
                                version,start,end,page,chunk,app_valid=parse_info(info)
                                if app_valid:
                                    print(f"Bootloader v{version} terdeteksi; GO otomatis ke aplikasi ...",
                                          file=sys.stderr)
                                    transact(port,bytes((CMD_GO,)),CMD_GO,timeout=.8)
                                    port.close(); port=None
                                    time.sleep(.45)
                            except TimeoutError:
                                pass
                    continue
                diag = decode_diag(payload)
                if diag is not None:
                    last_diag = diag
                    diag_frames += 1
                    last_rx_host = time.monotonic()
                    if args.mode == "diag":
                        valid += 1
                        if valid == 1 or valid % max(args.every, 1) == 0:
                            print(format_diag(diag))
                    continue
                data = decode_imu(payload)
                if data is None:
                    continue
                if args.mode == "diag":
                    continue
                last_rx_host=time.monotonic()

                if last_seq is not None and last_time_us is not None:
                    dseq = (data["seq"] - last_seq) & 0xFFFFFFFF
                    dt_us = (data["time_us"] - last_time_us) & 0xFFFFFFFF
                    if 0 < dseq < 0x80000000 and 0 < dt_us < 0x80000000:
                        if dseq > 1: lost += dseq - 1
                        seq_advance_total += dseq
                        sensor_elapsed_us += dt_us
                    else:
                        reboots += 1
                last_seq = data["seq"]
                last_time_us = data["time_us"]
                sample_time_us = data.get("sample_time_us", data["time_us"])
                sensor_lag_us = data.get("sensor_lag_us", 0)
                if data.get("version", 0) >= 6 and abs(sensor_lag_us) < 1000000:
                    lag_count += 1
                    lag_sum_us += sensor_lag_us
                    lag_max_us = max(lag_max_us, abs(sensor_lag_us))
                valid += 1

                if valid == 1 or valid % max(args.every, 1) == 0:
                    ax, ay, az = data["accel_raw"]
                    gx, gy, gz = data["gyro_raw"]
                    highspeed = data.get("observed_sample_hz", 0.0) >= 300.0
                    accel_scale = data.get("accel_lsb_per_g", 4096.0 if highspeed else 8192.0)
                    gyro_scale = data.get("gyro_lsb_per_dps", 32.8 if highspeed else 65.5)
                    ag = (ax / accel_scale, ay / accel_scale, az / accel_scale)
                    gd = (gx / gyro_scale, gy / gyro_scale, gz / gyro_scale)
                    temp_c = data["temp_c"]
                    if args.mode == "raw":
                        print(
                            f"RAW seq={data['seq']:8d} t={data['time_us']:10d}us "
                            f"sample_t={sample_time_us:10d}us lag={sensor_lag_us:5d}us "
                            f"Araw=({ax:6d},{ay:6d},{az:6d}) "
                            f"Graw=({gx:6d},{gy:6d},{gz:6d}) Traw={data['temp_raw']:6d} "
                            f"A[g]=({ag[0]:+.4f},{ag[1]:+.4f},{ag[2]:+.4f}) "
                            f"G[dps]=({gd[0]:+.3f},{gd[1]:+.3f},{gd[2]:+.3f}) "
                            f"T={temp_c:.2f}C sample={data['observed_sample_hz']:.2f}Hz"
                        )
                    elif args.mode == "ready":
                        print(
                            f"READY seq={data['seq']:8d} "
                            f"RPY=({data['roll']:+8.3f},{data['pitch']:+8.3f},{data['yaw']:+8.3f})deg "
                            f"LinAcc=({data['linacc'][0]:+.3f},{data['linacc'][1]:+.3f},{data['linacc'][2]:+.3f})m/s2 "
                            f"V=({data['vel'][0]:+.3f},{data['vel'][1]:+.3f},{data['vel'][2]:+.3f})m/s "
                            f"P=({data['pos'][0]:+.3f},{data['pos'][1]:+.3f},{data['pos'][2]:+.3f})m "
                            f"stdV=({data['vel_std'][0]:.3f},{data['vel_std'][1]:.3f},{data['vel_std'][2]:.3f}) "
                            f"lag={sensor_lag_us}us T={temp_c:.2f}C flags=[{flag_text(data['flags'])}] "
                            f"nav=[{nav_text(data['nav_status'])}] resets={data['health_resets']}"
                        )
                    elif args.mode == "nav":
                        px, py, pz = data["pos"]
                        vx, vy, vz = data["vel"]
                        sx, sy, sz = data["pos_std"]
                        svx, svy, svz = data["vel_std"]
                        print(
                            f"NAV seq={data['seq']:8d} sample_t={sample_time_us:10d}us lag={sensor_lag_us:5d}us "
                            f"x={px:+.4f} y={py:+.4f} z={pz:+.4f}m "
                            f"vx={vx:+.4f} vy={vy:+.4f} vz={vz:+.4f}m/s "
                            f"stdP=({sx:.4f},{sy:.4f},{sz:.4f})m "
                            f"stdV=({svx:.4f},{svy:.4f},{svz:.4f})m/s "
                            f"nav=[{nav_text(data['nav_status'])}]"
                        )
                    else:
                        print(
                            f"seq={data['seq']:8d} t={data['time_us']:10d}us "
                            f"RPY=({data['roll']:8.3f},{data['pitch']:8.3f},{data['yaw']:8.3f})deg "
                            f"Araw=({ax:6d},{ay:6d},{az:6d}) "
                            f"Graw=({gx:6d},{gy:6d},{gz:6d}) Traw={data['temp_raw']:6d} "
                            f"A[g]=({ag[0]:+.3f},{ag[1]:+.3f},{ag[2]:+.3f}) "
                            f"G[dps]=({gd[0]:+.2f},{gd[1]:+.2f},{gd[2]:+.2f}) "
                            f"LinAcc=({data['linacc'][0]:+.3f},{data['linacc'][1]:+.3f},{data['linacc'][2]:+.3f})m/s2 "
                            f"V=({data['vel'][0]:+.3f},{data['vel'][1]:+.3f},{data['vel'][2]:+.3f})m/s "
                            f"P=({data['pos'][0]:+.3f},{data['pos'][1]:+.3f},{data['pos'][2]:+.3f})m "
                            f"stdV=({data['vel_std'][0]:.3f},{data['vel_std'][1]:.3f},{data['vel_std'][2]:.3f}) "
                            f"aid_age={data['aid_age_ms']}ms rej={data['aid_reject']} "
                            f"T={temp_c:.2f}C imu=0x{data['imu_whoami']:02X}/c{data['imu_class']} rate={data['observed_sample_hz']:.2f}Hz "
                            f"flags=0x{data['flags']:04X} [{flag_text(data['flags'])}] "
                            f"nav=0x{data['nav_status']:02X}[{nav_text(data['nav_status'])}] resets={data['health_resets']}"
                            + (f" | {format_diag(last_diag)}" if last_diag else "")
                        )
            except (serial.SerialException, OSError, FileNotFoundError) as exc:
                reconnects += 1
                if port is not None:
                    try: port.close()
                    except Exception: pass
                port = None
                print(f"Serial terputus ({exc}); menunggu COM kembali ...", file=sys.stderr)
                time.sleep(0.20)
    except KeyboardInterrupt:
        interrupted = True
    finally:
        if port is not None:
            try: port.close()
            except Exception: pass

    elapsed = max(time.monotonic() - started, 1e-6)
    stream_hz = seq_advance_total / max(sensor_elapsed_us * 1.0e-6, 1e-6) if sensor_elapsed_us else 0.0
    lag_text = (f", sensor_lag_avg={lag_sum_us/lag_count:.0f}us max={lag_max_us}us" if lag_count else "")
    print(f"Ringkasan: valid={valid}, hilang_seq={lost}, crc_error={crc_errors}, "
          f"error_lain={other_errors}, reconnect={reconnects}, reboot/reset={reboots}, "
          f"diag={diag_frames}, stream={stream_hz:.1f} Hz, laju_host_total={valid/elapsed:.1f} frame/s{lag_text}")
    if interrupted:
        raise KeyboardInterrupt
    return 0


if __name__ == "__main__":
    from run_csv import run_logged
    raise SystemExit(run_logged(main,__file__))
