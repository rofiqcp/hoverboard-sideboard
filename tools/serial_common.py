#!/usr/bin/env python3
"""Pemilihan/pembukaan serial sideboard yang tahan USB re-enumeration."""
from pathlib import Path
import glob
import time
import serial

PREFERRED = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"


def _candidates():
    out = [PREFERRED]
    out += sorted(glob.glob("/dev/serial/by-id/*USB*Serial*"))
    out += sorted(glob.glob("/dev/serial/by-id/*USB-Serial*"))
    out += ["/dev/ttyUSB0"]
    out += sorted(glob.glob("/dev/ttyUSB*"))
    out += sorted(glob.glob("/dev/ttyACM*"))
    seen = set()
    return [p for p in out if not (p in seen or seen.add(p))]


def find_sideboard_port(requested=None):
    """Pilih by-id bila ada, fallback ttyUSB/ttyACM."""
    if requested and requested not in ("auto", "AUTO"):
        if Path(requested).exists():
            return requested
        raise FileNotFoundError(f"Port serial tidak ditemukan: {requested}")
    for port in _candidates():
        if Path(port).exists():
            return port
    raise FileNotFoundError("Sideboard serial belum terdeteksi (by-id/ttyUSB/ttyACM tidak ada).")


def open_sideboard_port(requested="auto", baud=921600, timeout=0.05,
                        attempts=100, delay=0.20):
    """Tunggu/retry open hingga ~20 s; cocok saat USB adapter re-enumerate."""
    last = None
    for attempt in range(max(attempts, 1)):
        try:
            port = find_sideboard_port(requested)
            s = serial.Serial(port, baud, timeout=timeout, write_timeout=max(timeout, 0.20))
            return s
        except (FileNotFoundError, serial.SerialException, OSError) as exc:
            last = exc
            if attempt + 1 < attempts:
                time.sleep(delay)
    raise last if last else FileNotFoundError("Sideboard serial tidak tersedia")
