"""UART bootloader hotplug safety: simulated USB detach/renumber/GO loss."""
import os
import struct
import sys
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import pytest
import serial

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools" / "imu"))
import flash_uart as fw


def valid_image(extra=b"12345678"):
    return struct.pack("<II", 0x20005000, 0x08001809) + extra


def descriptor(name, *, location="1-2.3", serial_number=None):
    return SimpleNamespace(device=name, vid=0x1A86, pid=0x7523,
                           location=location, serial_number=serial_number)


class FakePort:
    def __init__(self, name):
        self.port = name
        self.is_open = True
        self.closed = False

    def close(self):
        self.closed = True
        self.is_open = False


def info(version=4):
    return bytes((fw.CMD_INFO, 0, version)) + struct.pack(
        ">IIHBB", 0x08001800, 0x0800F400, 1024, 128, 1
    )


def test_port_renumber_relocates_only_same_verified_usb_adapter():
    trusted = {}
    p_old = descriptor("/dev/ttyUSB0")
    p_new = descriptor("/dev/ttyUSB9")
    with mock.patch.object(fw.list_ports, "comports", return_value=[p_old]):
        fw._record_trusted_adapter(trusted, p_old.device, explicit=False)
    with mock.patch.object(fw.list_ports, "comports",
                           return_value=[descriptor("/dev/ttyUSB1", location="1-4"),
                                         p_new]):
        assert fw._trusted_adapter_port(trusted) == "/dev/ttyUSB9"


def test_ambiguous_or_disconnected_adapter_never_targets_other_uart():
    trusted = {}
    with mock.patch.object(fw.list_ports, "comports",
                           return_value=[descriptor("/dev/ttyUSB0")]):
        fw._record_trusted_adapter(trusted, "/dev/ttyUSB0", explicit=False)
    with mock.patch.object(fw.list_ports, "comports",
                           return_value=[descriptor("/dev/ttyUSB1", location="1-4")]):
        with pytest.raises(FileNotFoundError, match="Trusted"):
            fw._trusted_adapter_port(trusted)
    with mock.patch.object(fw.list_ports, "comports",
                           return_value=[descriptor("/dev/ttyUSB1"),
                                         descriptor("/dev/ttyUSB2")]):
        with pytest.raises(FileNotFoundError, match="ambiguous"):
            fw._trusted_adapter_port(trusted)


def test_auto_silent_bootloader_without_prior_identity_fails_closed():
    trusted = dict(device="/dev/ttyUSB0", explicit=False)
    with pytest.raises(FileNotFoundError, match="Cannot safely rediscover"):
        fw._trusted_adapter_port(trusted)


def test_f1_ack_lost_but_same_adapter_reappears_on_new_tty():
    old = FakePort("/dev/ttyUSB0")
    new = FakePort("/dev/ttyUSB9")
    opened = []
    def connect(port, *args, **kwargs):
        opened.append(port)
        return old if port == "auto" else new
    def catch(port, _seconds):
        if port is old:
            raise TimeoutError("app still running")
        return info()
    trusted = {}
    with mock.patch.object(fw, "open_sideboard_port", side_effect=connect), \
         mock.patch.object(fw, "catch_bootloader", side_effect=catch), \
         mock.patch.object(fw, "request_bootloader_from_app", return_value=False), \
         mock.patch.object(fw.list_ports, "comports",
                           side_effect=[[descriptor("/dev/ttyUSB0")],
                                        [descriptor("/dev/ttyUSB9")]]):
        got, response = fw.acquire_bootloader(
            "auto", 921600, overall=1.0, trusted=trusted
        )
    assert got is new and response == info()
    assert opened == ["auto", "/dev/ttyUSB9"]
    assert old.closed
    assert not new.closed
    assert trusted["location"] == "1-2.3"


def test_boot_runtime_identity_must_match_exact_version_and_length():
    assert not fw._known_application_frame(b"")
    assert not fw._known_application_frame(b"\xf0")
    assert not fw._known_application_frame(b"\xf0\x06")
    assert not fw._known_application_frame(b"\xf0\x06" + bytes(114))
    assert fw._known_application_frame(b"\xf0\x06" + bytes(116))
    assert fw._known_application_frame(b"\xf0\x05" + bytes(112))


def test_go_ack_lost_but_application_telemetry_proves_success():
    chunks = []
    def handle_retry(_port, payload, expect_cmd, timeout, attempts=3):
        chunks.append(payload[0])
        if expect_cmd == fw.CMD_WRITE:
            return bytes((fw.CMD_WRITE, 0)) + payload[1:5]
        return bytes((expect_cmd, 0))
    runtime = bytes((0xF0, 6)) + bytes(116)
    with mock.patch.object(fw, "transact_retry", side_effect=handle_retry), \
         mock.patch.object(fw, "transact", side_effect=TimeoutError("ACK dropped")), \
         mock.patch.object(fw, "wait_for_application", return_value=runtime):
        fw.flash_image(FakePort("/dev/ttyUSB0"), valid_image(), info())
    assert chunks.count(fw.CMD_ERASE) == 1
    assert chunks.count(fw.CMD_VERIFY) == 1


def test_go_ack_lost_and_usb_removed_gives_uncertain_status_not_retry():
    def handle_retry(_port, payload, expect_cmd, timeout, attempts=3):
        if expect_cmd == fw.CMD_WRITE:
            return bytes((fw.CMD_WRITE, 0)) + payload[1:5]
        return bytes((expect_cmd, 0))
    with mock.patch.object(fw, "transact_retry", side_effect=handle_retry), \
         mock.patch.object(fw, "transact",
                           side_effect=serial.SerialException("USB unplugged")), \
         mock.patch.object(fw, "wait_for_application",
                           side_effect=serial.SerialException("USB unplugged")):
        with pytest.raises(fw.BootGoUncertain):
            fw.flash_image(FakePort("/dev/ttyUSB0"), valid_image(), info())


def run_main(tmp_path, image, error, recovered):
    f = tmp_path / "app.bin"
    f.write_bytes(image)
    ports = []
    def acquire(port, baud, handshake, overall, trusted):
        ports.append(trusted)
        return FakePort("/dev/ttyUSB0"), info()
    with mock.patch.object(sys, "argv",
                           ["flash_uart.py", str(f), "--retries", "6",
                            "--overall-timeout", "5"]), \
         mock.patch.object(fw, "acquire_bootloader", side_effect=acquire), \
         mock.patch.object(fw, "flash_image", side_effect=error) as flash, \
         mock.patch.object(fw, "confirm_go_after_hotplug", return_value=recovered) as confirm:
        rc = fw.main()
    assert flash.call_count == 1
    assert confirm.call_count == 1
    assert len(ports) == 1
    return rc


def test_main_never_reerases_when_go_uncertain_and_app_is_verified(tmp_path):
    assert run_main(tmp_path, valid_image(), fw.BootGoUncertain("GO ACK dropped"),
                    recovered=True) == 0


def test_main_never_reerases_when_go_uncertain_and_app_not_verified(tmp_path):
    assert run_main(tmp_path, valid_image(), fw.BootGoUncertain("GO ACK dropped"),
                    recovered=False) == 1


def test_write_retries_preserve_adapter_identity_after_disconnect(tmp_path):
    f = tmp_path / "app.bin"
    f.write_bytes(valid_image())
    identities = []
    attempts = 0
    def acquire(port, baud, handshake, overall, trusted):
        identities.append(trusted)
        trusted.setdefault("location", "1-2.3")
        return FakePort("/dev/ttyUSB0"), info()
    def flash(*_):
        nonlocal attempts
        attempts += 1
        if attempts == 1:
            raise serial.SerialException("USB detached mid-write")
    with mock.patch.object(sys, "argv",
                           ["flash_uart.py", str(f), "--retries", "3",
                            "--overall-timeout", "5"]), \
         mock.patch.object(fw, "acquire_bootloader", side_effect=acquire), \
         mock.patch.object(fw, "flash_image", side_effect=flash), \
         mock.patch.object(fw.time, "sleep"):
        assert fw.main() == 0
    assert attempts == 2
    assert identities[0] is identities[1]


def test_delayed_write_ack_is_ignored_until_current_address_matches():
    class Port:
        def write(self, _): return 12
        def flush(self): pass
    payload = bytes((fw.CMD_WRITE,)) + struct.pack(">I", 0x08002000) + b"xx"
    stale = bytes((fw.CMD_WRITE, 0)) + struct.pack(">I", 0x08001F00)
    current = bytes((fw.CMD_WRITE, 0)) + payload[1:5]
    with mock.patch.object(fw, "read_packet", side_effect=[stale, current]):
        assert fw.transact(Port(), payload, fw.CMD_WRITE, timeout=.2) == current

def test_bad_vectors_rejected_before_begin_update_or_erase():
    with mock.patch.object(fw, "transact_retry") as destructive:
        with pytest.raises(ValueError, match="vector table"):
            fw.flash_image(FakePort("/dev/ttyUSB0"), bytes(16), info())
    destructive.assert_not_called()


def test_foreign_bootloader_map_rejected_before_any_flash_write():
    foreign = bytes((fw.CMD_INFO, 0, 4)) + struct.pack(
        ">IIHBB", 0x08000000, 0x08010000, 1024, 128, 1
    )
    with mock.patch.object(fw, "transact_retry") as destructive:
        with pytest.raises(RuntimeError, match="flash map"):
            fw.flash_image(FakePort("/dev/ttyUSB0"), valid_image(), foreign)
    destructive.assert_not_called()


def test_actual_built_firmware_passes_host_preflight():
    image = (ROOT / ".pio" / "build" / "APP_STLINK" / "firmware.bin").read_bytes()
    assert fw.validate_firmware_image(image, info())[0] == 4
    fast = (ROOT / ".pio" / "build" / "APP_STLINK_1KHZ" / "firmware.bin").read_bytes()
    assert fw.validate_firmware_image(fast, info())[0] == 4


def test_cli_invalid_firmware_never_opens_any_serial_port(tmp_path):
    bad = tmp_path / "corrupt.bin"
    bad.write_bytes(bytes(100))
    with mock.patch.object(sys, "argv", ["flash_uart.py", str(bad)]), \
         mock.patch.object(fw, "acquire_bootloader") as acquire:
        assert fw.main() == 2
    acquire.assert_not_called()
