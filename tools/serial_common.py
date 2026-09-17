#!/usr/bin/env python3
"""Helper pemilihan port serial sideboard secara otomatis."""
from pathlib import Path
import glob

PREFERRED = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"


def find_sideboard_port(requested=None):
    """Pilih port stabil by-id, lalu fallback ttyUSB/ttyACM."""
    if requested and requested not in ("auto", "AUTO"):
        if Path(requested).exists():
            return requested
        raise FileNotFoundError(f"Port serial tidak ditemukan: {requested}")

    candidates = [PREFERRED, "/dev/ttyUSB0"]
    candidates += sorted(glob.glob("/dev/serial/by-id/*USB*Serial*"))
    candidates += sorted(glob.glob("/dev/ttyUSB*"))
    candidates += sorted(glob.glob("/dev/ttyACM*"))

    seen = set()
    for port in candidates:
        if port in seen:
            continue
        seen.add(port)
        if Path(port).exists():
            return port
    raise FileNotFoundError("Sideboard tidak ditemukan. Cek CH340 dan /dev/ttyUSB0.")
