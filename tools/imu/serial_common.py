#!/usr/bin/env python3
"""Pemilihan/pembukaan serial sideboard yang tahan USB re-enumeration."""
from pathlib import Path
import glob
import os
import time
import serial
from serial.tools import list_ports

PREFERRED_LINUX = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
PREFERRED_WINDOWS = "COM18"
PREFERRED_USB_VIDS = {0x1A86, 0x067B, 0x0403}  # CH340/CH341, PL2303, FTDI


def _windows_ports():
    """Auto hanya memilih USB-UART yang memang mungkin dipakai sideboard.

    Port lain tetap bisa dipakai bila user memberikannya eksplisit dengan --port.
    Ini mencegah auto-mode mengirim frame biner ke modem/Android/COM lain.
    """
    ports = list(list_ports.comports())
    preferred = [p.device for p in ports if p.vid in PREFERRED_USB_VIDS]
    preferred.sort(key=lambda d: (d.lower() != PREFERRED_WINDOWS.lower(), d.lower()))
    return preferred


def _candidates():
    if os.name == "nt":
        return _windows_ports()
    out = [PREFERRED_LINUX]
    out += sorted(glob.glob("/dev/serial/by-id/*USB*Serial*"))
    out += sorted(glob.glob("/dev/serial/by-id/*USB-Serial*"))
    # Jangan fallback buta ke ttyUSB*/ttyACM*: pada robot port tersebut juga
    # dapat dimiliki IMU CP210x, ESC, Android, dsb. Auto hanya menerima VID
    # USB-UART yang memang didukung sideboard; port lain harus eksplisit.
    out += [p.device for p in list_ports.comports() if p.vid in PREFERRED_USB_VIDS]
    seen = set()
    return [p for p in out if _port_exists(p) and not (p in seen or seen.add(p))]


def sideboard_candidates():
    """Kandidat auto yang lolos filter USB-UART; identity tetap diverifikasi caller."""
    return list(_candidates())


def _port_exists(port):
    if os.name == "nt":
        return any(p.device.lower() == str(port).lower() for p in list_ports.comports())
    return Path(port).exists()


def find_sideboard_port(requested=None):
    """Pilih kandidat USB-UART sideboard; mode auto belum membuka/probe port."""
    if requested and requested not in ("auto", "AUTO"):
        if _port_exists(requested):
            return requested
        raise FileNotFoundError(f"Port serial tidak ditemukan: {requested}")
    for port in _candidates():
        if _port_exists(port):
            return port
    raise FileNotFoundError("Sideboard serial belum terdeteksi pada kandidat USB-UART yang didukung.")


def _crc16(data):
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def _vesc_packet(payload):
    c = _crc16(payload)
    return bytes((2, len(payload))) + payload + bytes((c >> 8, c & 0xFF, 3))


def _valid_sideboard_telemetry(payload):
    """Validated F0/F5 version and exact layout, after outer-frame CRC.

    A CRC-valid VESC packet with a coincidentally reused command ID is NOT
    sideboard identity unless its application schema matches.
    """
    if len(payload) < 2:
        return False
    if payload[0] == 0xF0:
        return (payload[1], len(payload)) in (
            (2, 79), (3, 101), (4, 104), (5, 114), (6, 118)
        )
    if payload[0] == 0xF5:
        return (payload[1], len(payload)) in ((1, 69), (2, 77))
    return False


def _nested_sideboard_frame_start(buf):
    """Find a full validated F0/F5 frame after a false hotplug prefix."""
    for pos in range(1, max(1, len(buf)-4)):
        if buf[pos] != 2 or pos+5 > len(buf):
            continue
        n = buf[pos+1]
        if not 0 < n <= 192 or pos+n+5 > len(buf):
            continue
        end = pos+n+2
        payload = bytes(buf[pos+2:end])
        if (buf[end+2] == 3
            and _crc16(payload) == ((buf[end] << 8)|buf[end+1])
            and _valid_sideboard_telemetry(payload)):
            return pos
    return None


def _probe_sideboard_identity(port, seconds=0.45):
    """Passive identity probe: hanya menerima telemetry aplikasi F0/F5 valid.

    Auto-detection sengaja tidak menulis command apa pun ke kandidat serial,
    sehingga ESC/IMU/Android yang memakai adapter USB-UART serupa tidak pernah
    menerima probe aktif. Bootloader-only harus direacquire lewat port yang
    sudah diketahui dari aplikasi atau dipilih eksplisit oleh user.
    """
    try:
        port.reset_input_buffer()
    except (serial.SerialException, OSError):
        return False
    buf = bytearray()
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        try:
            chunk = port.read(256)
        except (serial.SerialException, OSError):
            return False
        if chunk:
            buf.extend(chunk)
            if len(buf) > 512:
                del buf[:-256]
        while True:
            try:
                start = buf.index(2)
            except ValueError:
                buf.clear()
                break
            if start:
                del buf[:start]
            if len(buf) < 2:
                break
            n = buf[1]
            if n == 0 or n > 192:
                del buf[0]
                continue
            total = 2 + n + 3
            if len(buf) < total:
                nested = _nested_sideboard_frame_start(buf)
                if nested is not None:
                    del buf[:nested]
                    continue
                break
            payload = bytes(buf[2:2+n])
            crc_rx = (buf[2+n] << 8) | buf[3+n]
            if buf[4+n] == 3 and _crc16(payload) == crc_rx:
                del buf[:total]
                if _valid_sideboard_telemetry(payload):
                    return True
            else:
                del buf[0]
    return False


def open_sideboard_port(requested="auto", baud=921600, timeout=0.05,
                        attempts=20, delay=0.20):
    """Buka serial dengan retry, identity probe untuk auto, dan tanpa toggle DTR/RTS.

    Auto-mode mencoba semua kandidat secara pasif dan hanya menerima aplikasi
    F0/F5 valid. Port eksplisit tetap dihormati tanpa probe.
    """
    last = None
    auto = requested in (None, "auto", "AUTO")
    for attempt in range(max(attempts, 1)):
        try:
            candidates = sideboard_candidates() if auto else [find_sideboard_port(requested)]
        except FileNotFoundError as exc:
            candidates = []
            last = exc
        if not candidates:
            last = last or FileNotFoundError("Sideboard serial belum terdeteksi")
        for name in candidates:
            s_port = None
            try:
                s_port = serial.Serial(port=None, baudrate=baud, timeout=timeout,
                                       write_timeout=max(timeout, 0.20),
                                       rtscts=False, dsrdtr=False, exclusive=True)
                s_port.dtr = False
                s_port.rts = False
                s_port.port = name
                s_port.open()
                if not auto or _probe_sideboard_identity(s_port):
                    accepted = s_port
                    s_port = None
                    return accepted
                last = FileNotFoundError(f"{name} bukan sideboard (probe F0/F5/F8 gagal)")
            except (FileNotFoundError, serial.SerialException, OSError) as exc:
                last = exc
            finally:
                if s_port is not None and s_port.is_open:
                    try:
                        s_port.close()
                    except Exception:
                        pass
        if attempt + 1 < attempts:
            time.sleep(delay)
    raise last if last else FileNotFoundError("Sideboard serial tidak tersedia")


def main():
    print("Serial candidates:")
    for p in _candidates():
        print(" ", p, "FOUND" if _port_exists(p) else "-")
    port = None
    try:
        port = open_sideboard_port("auto", attempts=1)
        print("Verified sideboard:", port.port)
    except (FileNotFoundError, serial.SerialException, OSError) as e:
        print("Verified sideboard: none -", e)
    finally:
        try:
            if port is not None:
                port.close()
        except Exception:
            pass
    return 0

if __name__=="__main__":
    from run_csv import run_logged
    raise SystemExit(run_logged(main,__file__))
