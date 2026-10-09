"""Prevent CRC-valid packets from unrelated VESC tools passing identity probes."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools" / "imu"))
from serial_common import _valid_sideboard_telemetry


def test_exact_f0_versions_and_lengths():
    for version, length in ((2, 79), (3, 101), (4, 104), (5, 114), (6, 118)):
        payload = bytes((0xF0, version)) + bytes(length - 2)
        assert _valid_sideboard_telemetry(payload)
        assert not _valid_sideboard_telemetry(payload + b"\x00")
        assert not _valid_sideboard_telemetry(payload[:-1])


def test_exact_f5_versions_and_lengths():
    assert _valid_sideboard_telemetry(bytes((0xF5, 1)) + bytes(67))
    assert _valid_sideboard_telemetry(bytes((0xF5, 2)) + bytes(75))
    assert not _valid_sideboard_telemetry(bytes((0xF5, 2)) + bytes(67))
    assert not _valid_sideboard_telemetry(bytes((0xF5, 1)) + bytes(75))
    assert not _valid_sideboard_telemetry(bytes((0xF4, 1)) + bytes(75))
