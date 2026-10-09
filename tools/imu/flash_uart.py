#!/usr/bin/env python3
"""Flasher UART sederhana untuk bootloader sideboard.

Alur v4: tangkap bootloader -> INFO -> BEGIN_UPDATE -> ERASE -> WRITE -> VERIFY CRC32 -> GO -> verifikasi aplikasi.
Semua paket memakai framing pendek dan CRC16 yang sama dengan VESC.
"""

import argparse
import os
import struct
import sys
import time
import zlib
from pathlib import Path

import serial
from serial.tools import list_ports
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
CMD_BEGIN_UPDATE = 0xFD
COMM_SIDEBOARD_IMU = 0xF0
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


def crc32_image(data: bytes) -> int:
    """CRC32 IEEE whole-image, sama dengan bootloader v4."""
    return zlib.crc32(data) & 0xFFFFFFFF


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


def _nested_valid_frame_start(buf: bytearray):
    """Find a complete CRC-valid frame after a broken partial prefix.

    A hotplug can expose a fake 0x02/length prefix followed by an entire
    correct packet. Refusing to inspect the interior would stall all INFO/ACK
    until the bogus length is satisfied (or the full timeout expires).
    """
    for pos in range(1, max(1, len(buf) - 4)):
        if buf[pos] != 2 or pos + 5 > len(buf):
            continue
        n = buf[pos+1]
        if not 0 < n <= 192 or pos+n+5 > len(buf):
            continue
        start = pos + 2
        stop = start + n
        if buf[stop+2] == 3 and crc16(bytes(buf[start:stop])) == (
                (buf[stop] << 8) | buf[stop+1]):
            return pos
    return None


def read_packet(port: serial.Serial, timeout: float = 1.0):
    """CRC sliding resynchronizer that survives partial hotplug frames."""
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
            if n == 0 or n > 192:
                del buf[0]; continue
            total = n + 5
            if len(buf) < total:
                # If the leading candidate is truncated or corrupted, a
                # following complete valid frame must not wait for it.
                nested = _nested_valid_frame_start(buf)
                if nested is not None:
                    del buf[:nested]
                    continue
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
            # Keep memory bounded on a noisy/disconnected serial stream.
            # 256 bytes retain one complete maximum-length frame.
            if len(buf) > 512:
                del buf[:-256]
    return None


def transact(port: serial.Serial, payload: bytes, expect_cmd: int, timeout: float = 1.0) -> bytes:
    """Kirim satu command dan tunggu reply command yang sama."""
    packet = make_packet(payload)
    written = port.write(packet)
    if written != len(packet):
        raise serial.SerialTimeoutException(
            f"Partial UART command TX: {written}/{len(packet)} bytes"
        )
    port.flush()
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        reply = read_packet(port, min(0.3, max(end - time.monotonic(), 0.01)))
        if reply and reply[0] == expect_cmd:
            # A delayed WRITE ACK from the preceding chunk is not an ACK for
            # this address. Keep receiving until the matched one arrives.
            if expect_cmd == CMD_WRITE and payload and payload[0] == CMD_WRITE:
                if len(reply) != 6 or reply[2:6] != payload[1:5]:
                    continue
            if expect_cmd == CMD_INFO and len(reply) != 15:
                continue
            if expect_cmd == CMD_VERIFY and len(reply) != 2:
                continue
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
            except (serial.SerialException, OSError): pass
            _clear_rx_buffer(port)
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



class BootGoUncertain(RuntimeError):
    """GO was sent but application runtime status has not yet been verified."""


def _port_descriptor(name):
    """Lookup Linux/Windows USB adapter metadata without opening it."""
    for entry in list_ports.comports():
        if (entry.device == name or
                (os.name != "nt" and os.path.realpath(entry.device) ==
                 os.path.realpath(name))):
            return entry
    return None


def _record_trusted_adapter(trusted: dict, name: str, explicit: bool) -> None:
    """Pin the adapter confirmed by application F0/F5 before first F1 command.

    CH340 commonly lacks a unique serial; USB topology location is then used.
    Fail closed if both identifiers are absent instead of probing any new tty.
    """
    p = _port_descriptor(name)
    trusted.update(device=name, explicit=explicit)
    if p is not None:
        trusted.update(
            vid=p.vid, pid=p.pid, serial_number=p.serial_number,
            location=p.location,
        )


def _trusted_adapter_port(trusted: dict) -> str:
    """Find ONLY the same known USB adapter after /dev/ttyUSB renumbering."""
    serial_number = trusted.get("serial_number")
    location = trusted.get("location")
    vid = trusted.get("vid")
    pid = trusted.get("pid")
    if serial_number or location:
        matches = [
            p for p in list_ports.comports()
            if p.vid == vid and p.pid == pid
            and (not serial_number or p.serial_number == serial_number)
            and (not location or p.location == location)
        ]
        if len(matches) == 1:
            return matches[0].device
        raise FileNotFoundError(
            f"Trusted sideboard adapter disconnected or ambiguous: {len(matches)} matches"
        )
    # A port name alone cannot authenticate a newly attached /dev/ttyUSBn.
    # Exact manual --port is the operator's explicit choice.
    if trusted.get("explicit") and trusted.get("device"):
        return trusted["device"]
    raise FileNotFoundError(
        "Cannot safely rediscover silent bootloader: no stable USB serial/location. "
        "Use a unique /dev/serial/by-id path or --port explicitly."
    )


def _known_application_frame(payload: bytes) -> bool:
    return bool(payload and len(payload) >= 2 and (
        (payload[0] == 0xF0 and
         (payload[1], len(payload)) in ((5, 114), (6, 118)))
        or
        (payload[0] == 0xF5 and
         (payload[1], len(payload)) in ((1, 69), (2, 77)))
    ))


def confirm_go_after_hotplug(trusted: dict, deadline_s: float) -> bool:
    """Read-only verification; never ERASE again after an ambiguous GO."""
    while time.monotonic() < deadline_s:
        port = None
        try:
            name = _trusted_adapter_port(trusted)
            port = open_sideboard_port(name, timeout=0.04, attempts=1)
            until = min(1.5, max(0.1, deadline_s-time.monotonic()))
            result = wait_for_application(port, timeout=until)
            if result is not None and _known_application_frame(result):
                print(f"GO ACK may have been lost, but verified app runs on {name}")
                return True
        except (serial.SerialException, OSError, FileNotFoundError, TimeoutError):
            pass
        finally:
            if port is not None:
                try: port.close()
                except (serial.SerialException, OSError): pass
        time.sleep(0.15)
    return False


def acquire_bootloader(requested, baud, handshake=4.0, overall=35.0, trusted=None):
    """Capture app identity, then re-acquire its SAME adapter in bootloader.

    After the first verified application, serial renumbering is recovered only
    using a stable USB serial/location. Never broadcast INFO/F1 to other UARTs.
    """
    if trusted is None:
        trusted = {}
    deadline = time.monotonic() + overall
    last = None
    attempt = 0
    auto = requested in (None, "auto", "AUTO")
    while time.monotonic() < deadline:
        attempt += 1
        port = None
        try:
            if trusted:
                candidate = _trusted_adapter_port(trusted)
                port = open_sideboard_port(candidate, baud, timeout=0.03,
                                           attempts=1)
            else:
                port = open_sideboard_port(requested, baud, timeout=0.03,
                                           attempts=2, delay=0.10)
                # In auto-mode, open_sideboard_port already passively verified
                # this app. In explicit mode, the operator selected its port.
                _record_trusted_adapter(trusted, port.port, explicit=not auto)
            name = port.port
            try:
                info = catch_bootloader(port, max(0.20, min(0.75, handshake)))
                # INFO must also have a valid bootloader schema and flash map.
                parse_info(info)
                print(f"Bootloader terdeteksi pada {name} (acquire #{attempt}).")
                accepted = port
                port = None  # transfer ownership to caller
                return accepted, info
            except (TimeoutError, RuntimeError):
                # This handle is either a verified app port or the already
                # trusted bootloader; F1 is harmless in the bootloader.
                ack = request_bootloader_from_app(
                    port, max(0.20, min(1.50, handshake))
                )
                if ack:
                    print(f"F1 ACK aplikasi pada {name}; reconnect trusted UART...")
                # Even without ACK, app may already have reset to bootloader.
                # The next iteration talks ONLY to the pinned adapter.
        except (serial.SerialException, OSError, FileNotFoundError,
                TimeoutError) as exc:
            last = exc
        finally:
            # Only successful bootloader acquisition transfers ownership.
            if port is not None and port.is_open:
                try: port.close()
                except (serial.SerialException, OSError): pass
        if time.monotonic() < deadline:
            time.sleep(0.15)
    raise TimeoutError(
        f"Cannot safely acquire/re-acquire sideboard bootloader: {last}"
    )


def wait_for_application(port: serial.Serial, timeout: float = 6.0):
    """Tunggu bukti runtime: extended IMU protocol v5 setelah GO."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        payload = read_packet(port, min(0.35, max(deadline - time.monotonic(), 0.02)))
        if payload and payload[0] == COMM_SIDEBOARD_IMU and _known_application_frame(payload):
            return payload
    return None


def verify_application_boot(port: serial.Serial, timeout: float = 6.0):
    """CRC success is not runtime success; require application telemetry."""
    payload = wait_for_application(port, timeout)
    if payload is not None:
        return payload
    try:
        info = catch_bootloader(port, 1.0)
        version, *_ = parse_info(info)
        raise RuntimeError(
            f"APP_BOOT_FAILED: bootloader v{version} reacquired, aplikasi tidak memberi telemetry"
        )
    except TimeoutError:
        raise RuntimeError(
            "APP_BOOT_FAILED: tidak ada telemetry aplikasi dan bootloader tidak dapat direacquire"
        )



EXPECTED_APP_START = 0x08001800
EXPECTED_APP_END = 0x0800F400
EXPECTED_PAGE_SIZE = 1024


def validate_firmware_image(image: bytes, info: bytes):
    """Prevent destroying an existing app with corrupt/foreign binary.

    This is host preflight; bootloader independently validates the image
    vector table plus CRC32 before committing the boot manifest.
    """
    version, start, end, page, suggested_chunk, app_valid = parse_info(info)
    if version < 3:
        raise RuntimeError("Legacy bootloader v2 is not power-loss-safe")
    if (start, end, page) != (
            EXPECTED_APP_START, EXPECTED_APP_END, EXPECTED_PAGE_SIZE):
        raise RuntimeError(
            f"Foreign/unexpected bootloader flash map: "
            f"0x{start:08X}..0x{end:08X}, page={page}"
        )
    if suggested_chunk <= 0 or suggested_chunk > 128:
        raise RuntimeError(f"Unsupported bootloader chunk size: {suggested_chunk}")
    if not (8 <= len(image) <= end - start):
        raise ValueError(
            f"Application image {len(image)} bytes outside valid range "
            f"8..{end-start}"
        )
    app_sp, app_pc = struct.unpack_from("<II", image)
    if (app_sp < 0x20000000 or app_sp > 0x20005000 or
            app_sp & 0x3 or not (app_pc & 1) or
            not (start <= (app_pc & ~1) < start+len(image))):
        raise ValueError(
            f"Invalid vector table; SP=0x{app_sp:08X}, "
            f"ResetHandler=0x{app_pc:08X}. Refusing ERASE."
        )
    return version, start, end, page, suggested_chunk, app_valid


def flash_image(port: serial.Serial, image: bytes, info: bytes):
    """Erase/write/verify image; v4 uses BEGIN_UPDATE + whole-image CRC32."""
    version, start, end, page, suggested_chunk, app_valid = (
        validate_firmware_image(image, info)
    )
    chunk = min(max(suggested_chunk, 2), 128)
    if chunk & 1:
        chunk -= 1
    print(
        f"Bootloader v{version} | app 0x{start:08X}..0x{end-1:08X} | "
        f"page={page} | app_lama_valid={app_valid}"
    )

    if version >= 4:
        r = transact_retry(
            port, bytes((CMD_BEGIN_UPDATE,)), CMD_BEGIN_UPDATE, timeout=1.0, attempts=3
        )
        if len(r) < 2 or r[1] != 0:
            raise RuntimeError("BEGIN_UPDATE ditolak")
        print("BEGIN_UPDATE OK; recovery latch aktif.")

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
        print(
            f"\rMenulis {done:6d}/{total:6d} byte ({done*100/total:5.1f}%)",
            end="",
            flush=True,
        )
    print()

    if version >= 4:
        image_crc = crc32_image(image)
        verify = bytes((CMD_VERIFY,)) + struct.pack(">I", total) + struct.pack(">I", image_crc)
        label = f"CRC32=0x{image_crc:08X}"
    else:
        image_crc = crc16(image)
        verify = bytes((CMD_VERIFY,)) + struct.pack(">I", total) + struct.pack(">H", image_crc)
        label = f"CRC16=0x{image_crc:04X}"

    r = transact_retry(port, verify, CMD_VERIFY, timeout=2.0, attempts=2)
    if len(r) < 2 or r[1] != 0:
        raise RuntimeError(f"VERIFY gagal ({label})")
    print(f"VERIFY OK: {label}")

    # GO has an important ambiguous failure mode: the MCU may have jumped to
    # a healthy application before USB-UART delivered the ACK. Never repeat
    # ERASE just because the GO ACK was lost. Require runtime evidence.
    try:
        r = transact(port, bytes((CMD_GO,)), CMD_GO, timeout=1.0)
    except (TimeoutError, serial.SerialException, OSError) as exc:
        try:
            app = wait_for_application(port, timeout=3.0)
            if app is not None:
                print("GO ACK lost but application F0 verified; update succeeded.")
                return
        except (serial.SerialException, OSError):
            pass
        raise BootGoUncertain(f"GO status uncertain after lost ACK/USB: {exc}") from exc
    if len(r) < 2 or r[1] != 0:
        raise RuntimeError("GO ditolak bootloader")

    try:
        app = verify_application_boot(port, timeout=6.0)
    except (TimeoutError, serial.SerialException, OSError, RuntimeError) as exc:
        raise BootGoUncertain(f"GO ACK received but app verification interrupted: {exc}") from exc
    print(f"Application runtime verified: protocol v{app[1]} telemetry valid.")


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
    # Independently check vector plausibility before rebooting any application.
    # Final bounds/map verification happens after INFO, before BEGIN/ERASE.
    if len(image) < 8 or len(image) > EXPECTED_APP_END-EXPECTED_APP_START:
        print("Firmware size invalid: refusing to request bootloader", file=sys.stderr)
        return 2
    sp, pc = struct.unpack_from("<II", image)
    if not (0x20000000 <= sp <= 0x20005000 and (sp & 3) == 0 and
            (pc & 1) and EXPECTED_APP_START <= (pc & ~1) <
            EXPECTED_APP_START+len(image)):
        print("Firmware vector table invalid: refusing to enter bootloader",
              file=sys.stderr)
        return 2

    last = None
    trusted_adapter = {}
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
                                            overall=min(12.0, remaining),
                                            trusted=trusted_adapter)
            print(f"Sesi flash {attempt} memakai {port.port} @ {args.baud} baud")
            flash_image(port, image, info)
            try: port.close()
            except Exception: pass
            return 0
        except BootGoUncertain as exc:
            last = exc
            if port is not None:
                try: port.close()
                except (serial.SerialException, OSError): pass
            print(f"{exc}; only passive runtime verification after GO is permitted.",
                  file=sys.stderr)
            if confirm_go_after_hotplug(trusted_adapter, global_deadline):
                print("Verified application after GO/replug. Flash succeeded.")
                return 0
            # IMPORTANT: never automatically re-erase a potentially healthy
            # application following a GO uncertainty. Operator can inspect it.
            print("GO uncertain: NOT erasing again. Inspect application/bootloader "
                  "using the trusted port.", file=sys.stderr)
            return 1
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
    from run_csv import run_logged
    raise SystemExit(run_logged(main,__file__))
