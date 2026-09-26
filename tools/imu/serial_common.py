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
PREFERRED_USB_VIDS = {0x1A86, 0x067B}  # CH340/CH341 dan Prolific PL2303


def _windows_ports():
    """Daftar COM Windows, memprioritaskan COM18 lalu USB-UART CH34x/PL2303."""
    ports = list(list_ports.comports())
    preferred = [p.device for p in ports if p.vid in PREFERRED_USB_VIDS]
    all_ports = [p.device for p in ports]
    out = [PREFERRED_WINDOWS] + preferred + all_ports
    seen = set()
    return [p for p in out if not (p.lower() in seen or seen.add(p.lower()))]


def _candidates():
    if os.name == "nt":
        return _windows_ports()
    out = [PREFERRED_LINUX]
    out += sorted(glob.glob("/dev/serial/by-id/*USB*Serial*"))
    out += sorted(glob.glob("/dev/serial/by-id/*USB-Serial*"))
    out += ["/dev/ttyUSB0"]
    out += sorted(glob.glob("/dev/ttyUSB*"))
    out += sorted(glob.glob("/dev/ttyACM*"))
    seen = set()
    return [p for p in out if not (p in seen or seen.add(p))]


def _port_exists(port):
    if os.name == "nt":
        return any(p.device.lower() == str(port).lower() for p in list_ports.comports())
    return Path(port).exists()


def find_sideboard_port(requested=None):
    """Pilih COM di Windows atau by-id/ttyUSB/ttyACM di Linux."""
    if requested and requested not in ("auto", "AUTO"):
        if _port_exists(requested):
            return requested
        raise FileNotFoundError(f"Port serial tidak ditemukan: {requested}")
    for port in _candidates():
        if _port_exists(port):
            return port
    raise FileNotFoundError("Sideboard serial belum terdeteksi (COM/by-id/ttyUSB/ttyACM tidak ada).")


def open_sideboard_port(requested="auto", baud=921600, timeout=0.05,
                        attempts=20, delay=0.20):
    """Buka serial dengan retry terukur dan tanpa toggle DTR/RTS.

    exclusive=True mencegah dua tool sideboard mencampur frame pada port yang sama.
    DTR/RTS diset sebelum open agar PL2303 tidak ditoggle setiap reconnect.
    """
    last = None
    for attempt in range(max(attempts, 1)):
        s = None
        try:
            port = find_sideboard_port(requested)
            s = serial.Serial(port=None, baudrate=baud, timeout=timeout,
                              write_timeout=max(timeout, 0.20),
                              rtscts=False, dsrdtr=False, exclusive=True)
            s.dtr = False
            s.rts = False
            s.port = port
            s.open()
            return s
        except (FileNotFoundError, serial.SerialException, OSError) as exc:
            last = exc
            try:
                if 's' in locals() and s.is_open: s.close()
            except Exception:
                pass
            if attempt + 1 < attempts:
                time.sleep(delay)
    raise last if last else FileNotFoundError("Sideboard serial tidak tersedia")


def main():
    print("Serial candidates:")
    for p in _candidates(): print(" ",p,"FOUND" if _port_exists(p) else "-")
    try: print("Selected:",find_sideboard_port("auto"))
    except FileNotFoundError as e: print("Selected: none -",e)
    return 0

if __name__=="__main__":
    from run_csv import run_logged
    raise SystemExit(run_logged(main,__file__))
