#!/usr/bin/env python3
"""Flasher UART sederhana untuk bootloader sideboard.

Alur: tangkap bootloader -> INFO -> ERASE -> WRITE per 128 byte -> VERIFY CRC -> GO.
Semua paket memakai framing pendek dan CRC16 yang sama dengan VESC.
"""

import argparse
import struct
import sys
import time
from pathlib import Path

import serial
from serial_common import find_sideboard_port, open_sideboard_port

try:
    sys.stdout.reconfigure(line_buffering=True)
    sys.stderr.reconfigure(line_buffering=True)
except Exception:
    pass

CMD_ENTER_BOOT = 0xF1
CMD_INFO = 0xF8
CMD_ERASE = 0xF9
CMD_WRITE = 0xFA
CMD_GO = 0xFB
CMD_VERIFY = 0xFC
DEFAULT_PORT = "auto"
DEFAULT_BAUD = 921600


def crc16(data: bytes) -> int:
    """Hitung CRC16-CCITT polynomial 0x1021, nilai awal 0 seperti VESC."""
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def make_packet(payload: bytes) -> bytes:
    """Bungkus payload sebagai frame VESC pendek: 2,len,payload,crc,3."""
    if not 0 < len(payload) <= 255:
        raise ValueError("Panjang payload harus 1..255 byte")
    c = crc16(payload)
    return bytes((2, len(payload))) + payload + bytes((c >> 8, c & 0xFF, 3))


def read_exact(port: serial.Serial, size: int, timeout: float) -> bytes:
    """Baca tepat sejumlah byte sampai timeout."""
    out = bytearray()
    end = time.monotonic() + timeout
    while len(out) < size and time.monotonic() < end:
        out.extend(port.read(size - len(out)))
    return bytes(out)


def _rx_buffer(port):
    b = getattr(port, "_vesc_rx_buffer", None)
    if b is None:
        b = bytearray()
        setattr(port, "_vesc_rx_buffer", b)
    return b


def _clear_rx_buffer(port):
    setattr(port, "_vesc_rx_buffer", bytearray())


def read_packet(port: serial.Serial, timeout: float = 1.0):
    """Sliding resynchronizer VESC. Byte rusak hanya membuang 1 byte kandidat."""
    buf = _rx_buffer(port)
    end = time.monotonic() + timeout
    while time.monotonic() < end:
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
            if n == 0:
                del buf[0]; continue
            total = n + 5
            if len(buf) < total:
                break
            payload = bytes(buf[2:2+n])
            crc_rx = (buf[2+n] << 8) | buf[3+n]
            if buf[4+n] == 3 and crc_rx == crc16(payload):
                del buf[:total]
                return payload
            del buf[0]
        remaining = end - time.monotonic()
        if remaining <= 0:
            break
        chunk = port.read(min(256, max(1, getattr(port, "in_waiting", 0) or 1)))
        if chunk:
            buf.extend(chunk)
    return None


def transact(port: serial.Serial, payload: bytes, expect_cmd: int, timeout: float = 1.0) -> bytes:
    """Kirim satu command dan tunggu reply command yang sama."""
    port.write(make_packet(payload))
    port.flush()
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        reply = read_packet(port, min(0.3, max(end - time.monotonic(), 0.01)))
        if reply and reply[0] == expect_cmd:
            return reply
    raise TimeoutError(f"Tidak ada reply command 0x{expect_cmd:02X}")


def transact_retry(port, payload: bytes, expect_cmd: int, timeout: float, attempts: int = 3):
    """Retry command bootloader yang idempotent bila ACK hilang tanpa USB disconnect."""
    last = None
    for _ in range(max(attempts, 1)):
        try:
            return transact(port, payload, expect_cmd, timeout)
        except TimeoutError as exc:
            last = exc
            try: port.reset_input_buffer()
            except Exception: pass
            time.sleep(0.03)
    raise last


def request_bootloader_from_app(port: serial.Serial, seconds: float = 1.5) -> bool:
    """Minta aplikasi aktif reset ke bootloader melalui command VESC 0xF1."""
    end = time.monotonic() + seconds
    packet = make_packet(bytes((CMD_ENTER_BOOT,)))
    while time.monotonic() < end:
        port.write(packet)
        port.flush()
        reply = read_packet(port, 0.20)
        if reply and len(reply) >= 2 and reply[0] == CMD_ENTER_BOOT and reply[1] == 0:
            return True
        time.sleep(0.03)
    return False


def catch_bootloader(port: serial.Serial, seconds: float = 3.0) -> bytes:
    """Kirim INFO periodik; purge hanya sekali agar USB-UART tidak dibombardir ioctl."""
    end = time.monotonic() + seconds
    try:
        port.reset_input_buffer()
    except (serial.SerialException, OSError):
        raise
    _clear_rx_buffer(port)
    while time.monotonic() < end:
        port.write(make_packet(bytes((CMD_INFO,))))
        port.flush()
        reply = read_packet(port, 0.15)
        if reply and reply[0] == CMD_INFO:
            return reply
        time.sleep(0.04)
    raise TimeoutError("Bootloader tidak menjawab INFO dalam deadline")


def parse_info(reply: bytes):
    """Terjemahkan balasan INFO menjadi parameter flash."""
    if len(reply) != 15 or reply[1] != 0:
        raise RuntimeError("Reply INFO tidak valid")
    version = reply[2]
    app_start = struct.unpack_from(">I", reply, 3)[0]
    app_end = struct.unpack_from(">I", reply, 7)[0]
    page = (reply[11] << 8) | reply[12]
    chunk = reply[13]
    app_valid = bool(reply[14])
    return version, app_start, app_end, page, chunk, app_valid


def acquire_bootloader(requested, baud, handshake=4.0, overall=35.0):
    """Dapatkan handle bootloader BARU dan serahkan ownership ke caller.

    Port hanya ditutup pada jalur gagal. Setelah INFO sukses handle wajib tetap
    terbuka untuk ERASE/WRITE/VERIFY/GO.
    """
    deadline = time.monotonic() + overall
    last = None
    attempt = 0
    while time.monotonic() < deadline:
        attempt += 1
        port = None
        try:
            port = open_sideboard_port(requested, baud, timeout=0.03, attempts=2, delay=0.10)
            name = port.port
            try:
                info = catch_bootloader(port, 0.55)
                print(f"Bootloader terdeteksi pada {name} (acquire #{attempt}).")
                return port, info
            except TimeoutError:
                try:
                    ack = request_bootloader_from_app(port, 1.0)
                    if ack:
                        print(f"F1 ACK dari aplikasi pada {name}; reopen serial ...")
                except (serial.SerialException, OSError) as exc:
                    last = exc
                try: port.close()
                except Exception: pass
                port = None
        except (serial.SerialException, OSError, FileNotFoundError, TimeoutError) as exc:
            last = exc
            if port is not None:
                try: port.close()
                except Exception: pass
                port = None
        time.sleep(0.15)
    raise TimeoutError(f"Tidak bisa acquire bootloader setelah reconnect: {last}")


def flash_image(port: serial.Serial, image: bytes, info: bytes):
    """Erase, tulis image, lalu verifikasi CRC keseluruhan."""
    version, start, end, page, suggested_chunk, app_valid = parse_info(info)
    if version < 3:
        raise RuntimeError("Bootloader legacy v%d tidak punya manifest power-loss-safe; flash BOOTLOADER_STLINK v3 dulu" % version)
    if len(image) == 0 or len(image) > end - start:
        raise ValueError(f"Ukuran firmware {len(image)} byte di luar area aplikasi {end-start} byte")

    chunk = min(max(suggested_chunk, 2), 128)
    if chunk & 1:
        chunk -= 1
    print(f"Bootloader v{version} | app 0x{start:08X}..0x{end-1:08X} | "
          f"page={page} | app_lama_valid={app_valid}")
    print("Menghapus area aplikasi ...")
    r = transact_retry(port, bytes((CMD_ERASE,)), CMD_ERASE, timeout=8.0, attempts=2)
    if len(r) < 2 or r[1] != 0:
        raise RuntimeError("Erase gagal")

    total = len(image)
    for off in range(0, total, chunk):
        data = image[off:off + chunk]
        addr = start + off
        payload = bytes((CMD_WRITE,)) + struct.pack(">I", addr) + data
        r = transact_retry(port, payload, CMD_WRITE, timeout=1.5, attempts=3)
        if len(r) < 6 or r[1] != 0 or struct.unpack_from(">I", r, 2)[0] != addr:
            raise RuntimeError(f"Write gagal pada 0x{addr:08X}")
        done = min(off + len(data), total)
        print(f"\rMenulis {done:6d}/{total:6d} byte ({done*100/total:5.1f}%)", end="", flush=True)
    print()

    image_crc = crc16(image)
    verify = bytes((CMD_VERIFY,)) + struct.pack(">I", total) + struct.pack(">H", image_crc)
    r = transact_retry(port, verify, CMD_VERIFY, timeout=2.0, attempts=2)
    if len(r) < 2 or r[1] != 0:
        raise RuntimeError(f"VERIFY CRC gagal (host CRC=0x{image_crc:04X})")
    print(f"VERIFY CRC OK: 0x{image_crc:04X}")

    r = transact(port, bytes((CMD_GO,)), CMD_GO, timeout=1.0)
    if len(r) < 2 or r[1] != 0:
        raise RuntimeError("GO ditolak bootloader")
    print("Firmware valid, aplikasi dijalankan.")


def main():
    parser = argparse.ArgumentParser(description="Flash sideboard melalui bootloader UART VESC-like")
    parser.add_argument("firmware", nargs="?", default=".pio/build/APP_STLINK/firmware.bin")
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--handshake", type=float, default=4.0,
                        help="Lama mencoba menangkap bootloader setelah board di-reset")
    parser.add_argument("--retries", type=int, default=6,
                        help="Ulang sesi penuh bila USB putus saat enter/erase/write/verify")
    parser.add_argument("--overall-timeout", type=float, default=90.0,
                        help="Deadline global seluruh workflow; tidak pernah menunggu tanpa batas")
    args = parser.parse_args()

    path = Path(args.firmware)
    if not path.is_file():
        print(f"Firmware tidak ditemukan: {path}", file=sys.stderr)
        return 2
    image = path.read_bytes()

    last = None
    global_deadline = time.monotonic() + max(args.overall_timeout, 5.0)
    for attempt in range(1, max(args.retries, 1) + 1):
        if time.monotonic() >= global_deadline:
            last = TimeoutError("deadline global flash terlampaui")
            break
        port = None
        try:
            print(f"Acquire bootloader untuk sesi flash {attempt}/{max(args.retries,1)} ...")
            remaining = max(2.0, global_deadline - time.monotonic())
            port, info = acquire_bootloader(args.port, args.baud, args.handshake,
                                            overall=min(12.0, remaining))
            print(f"Sesi flash {attempt} memakai {port.port} @ {args.baud} baud")
            flash_image(port, image, info)
            try: port.close()
            except Exception: pass
            return 0
        except (serial.SerialException, OSError, FileNotFoundError, TimeoutError, RuntimeError) as exc:
            last = exc
            if port is not None:
                try: port.close()
                except Exception: pass
            print(f"Sesi flash {attempt} terputus/gagal: {exc}", file=sys.stderr)
            if attempt < max(args.retries, 1):
                print("Reconnect, acquire bootloader lagi, lalu ulang ERASE+WRITE dari awal ...", file=sys.stderr)
                time.sleep(0.50)
                continue
            break
    print(f"GAGAL setelah {max(args.retries,1)} sesi: {last}", file=sys.stderr)
    return 1



if __name__ == "__main__":
    raise SystemExit(main())
