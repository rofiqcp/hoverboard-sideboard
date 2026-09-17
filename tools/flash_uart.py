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

CMD_ENTER_BOOT = 0xF1
CMD_INFO = 0xF8
CMD_ERASE = 0xF9
CMD_WRITE = 0xFA
CMD_GO = 0xFB
CMD_VERIFY = 0xFC
DEFAULT_PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
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


def read_packet(port: serial.Serial, timeout: float = 1.0):
    """Cari satu frame VESC pendek yang CRC-nya valid."""
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        b = port.read(1)
        if not b or b[0] != 2:
            continue
        lb = read_exact(port, 1, 0.2)
        if len(lb) != 1 or lb[0] == 0:
            continue
        n = lb[0]
        rest = read_exact(port, n + 3, 0.5)
        if len(rest) != n + 3:
            continue
        payload = rest[:n]
        crc_rx = (rest[n] << 8) | rest[n + 1]
        if rest[n + 2] == 3 and crc_rx == crc16(payload):
            return payload
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
    """Kirim INFO berulang agar command tertangkap pada jendela boot 800 ms."""
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        port.reset_input_buffer()
        port.write(make_packet(bytes((CMD_INFO,))))
        port.flush()
        reply = read_packet(port, 0.12)
        if reply and reply[0] == CMD_INFO:
            return reply
        time.sleep(0.03)
    raise TimeoutError("Bootloader tidak tertangkap. Reset/power-cycle board lalu coba lagi.")


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


def flash_image(port: serial.Serial, image: bytes, info: bytes):
    """Erase, tulis image, lalu verifikasi CRC keseluruhan."""
    version, start, end, page, suggested_chunk, app_valid = parse_info(info)
    if len(image) == 0 or len(image) > end - start:
        raise ValueError(f"Ukuran firmware {len(image)} byte di luar area aplikasi {end-start} byte")

    chunk = min(max(suggested_chunk, 2), 128)
    if chunk & 1:
        chunk -= 1
    print(f"Bootloader v{version} | app 0x{start:08X}..0x{end-1:08X} | "
          f"page={page} | app_lama_valid={app_valid}")
    print("Menghapus area aplikasi ...")
    r = transact(port, bytes((CMD_ERASE,)), CMD_ERASE, timeout=8.0)
    if len(r) < 2 or r[1] != 0:
        raise RuntimeError("Erase gagal")

    total = len(image)
    for off in range(0, total, chunk):
        data = image[off:off + chunk]
        addr = start + off
        payload = bytes((CMD_WRITE,)) + struct.pack(">I", addr) + data
        r = transact(port, payload, CMD_WRITE, timeout=1.5)
        if len(r) < 6 or r[1] != 0 or struct.unpack_from(">I", r, 2)[0] != addr:
            raise RuntimeError(f"Write gagal pada 0x{addr:08X}")
        done = min(off + len(data), total)
        print(f"\rMenulis {done:6d}/{total:6d} byte ({done*100/total:5.1f}%)", end="", flush=True)
    print()

    image_crc = crc16(image)
    verify = bytes((CMD_VERIFY,)) + struct.pack(">I", total) + struct.pack(">H", image_crc)
    r = transact(port, verify, CMD_VERIFY, timeout=2.0)
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
    parser.add_argument("--handshake", type=float, default=3.0,
                        help="Lama mencoba menangkap bootloader setelah board di-reset")
    args = parser.parse_args()

    path = Path(args.firmware)
    if not path.is_file():
        print(f"Firmware tidak ditemukan: {path}", file=sys.stderr)
        return 2
    image = path.read_bytes()

    try:
        with serial.Serial(args.port, args.baud, timeout=0.03) as port:
            print(f"Mencari bootloader di {args.port} @ {args.baud} baud ...")
            try:
                info = catch_bootloader(port, 0.5)
                print("Bootloader sudah aktif.")
            except TimeoutError:
                print("Aplikasi aktif; meminta reset ke bootloader lewat USART ...")
                app_ack = request_bootloader_from_app(port, 2.0)
                if app_ack:
                    print("ACK aplikasi diterima; menunggu bootloader ...")
                else:
                    print("ACK aplikasi tidak terlihat; tetap cek apakah reset ke bootloader sudah terjadi ...")
                time.sleep(0.10)
                port.reset_input_buffer()
                try:
                    info = catch_bootloader(port, max(args.handshake, 2.0))
                except TimeoutError:
                    if not app_ack:
                        raise TimeoutError("Aplikasi tidak menjawab ENTER_BOOTLOADER dan bootloader juga tidak tertangkap")
                    raise
                print("Bootloader berhasil dimasuki tanpa ST-LINK.")
            flash_image(port, image, info)
    except Exception as exc:
        print(f"GAGAL: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
