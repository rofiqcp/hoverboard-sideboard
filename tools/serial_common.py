#!/usr/bin/env python3
"""Helper pemilihan dan pembukaan port serial sideboard yang robust."""
from pathlib import Path
import glob
import time
import serial

PREFERRED = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"

def find_sideboard_port(requested=None):
    """Pilih port stabil by-id, lalu fallback ttyUSB/ttyACM."""
    if requested and requested not in ("auto", "AUTO"):
        if Path(requested).exists():
            return requested
        raise FileNotFoundError(f"Port serial tidak ditemukan: {requested}")
    candidates=[PREFERRED,"/dev/ttyUSB0"]
    candidates+=sorted(glob.glob("/dev/serial/by-id/*USB*Serial*"))
    candidates+=sorted(glob.glob("/dev/ttyUSB*"))+sorted(glob.glob("/dev/ttyACM*"))
    seen=set()
    for port in candidates:
        if port not in seen and Path(port).exists():
            return port
        seen.add(port)
    raise FileNotFoundError("Sideboard tidak ditemukan. Cek CH340 dan /dev/ttyUSB0.")

def open_sideboard_port(requested, baud, timeout=0.05, attempts=5, delay=0.20):
    """Retry open untuk transien CH340/hub; tetap gagal cepat jika perangkat benar-benar hilang."""
    last=None
    for attempt in range(attempts):
        try:
            port=find_sideboard_port(requested)
            return serial.Serial(port, baud, timeout=timeout)
        except (FileNotFoundError, serial.SerialException, OSError) as exc:
            last=exc
            if attempt+1<attempts:
                time.sleep(delay)
    raise last
