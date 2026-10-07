import struct

from sideboard_p2_fusion.serial_aiding_bridge import (
    AID_FLAG_EXPECTED_YAW_RATE,
    AID_FLAG_NHC,
    ShortVescParser,
    build_wheel_payload,
    vesc_packet,
)


def test_extended_wheel_payload_matches_firmware_v2_layout():
    p = build_wheel_payload(
        request_id=0x1234,
        age_us=7000,
        velocity_mps=3.25,
        sigma_mps=0.15,
        expected_yaw_rate_rads=0.75,
        nhc=True,
    )
    assert len(p) == 21
    assert p[:4] == bytes((0xF4, 1, 2, 2))
    assert struct.unpack(">H", p[4:6])[0] == 0x1234
    assert struct.unpack(">I", p[6:10])[0] == 7000
    assert struct.unpack(">i", p[10:14])[0] == 3250
    assert struct.unpack(">H", p[14:16])[0] == 150
    assert p[16] == AID_FLAG_NHC | AID_FLAG_EXPECTED_YAW_RATE
    assert struct.unpack(">i", p[17:21])[0] == 750


def test_stale_steering_uses_backward_compatible_17_byte_wheel_payload():
    p = build_wheel_payload(
        request_id=9,
        age_us=0,
        velocity_mps=1.0,
        sigma_mps=0.05,
        expected_yaw_rate_rads=None,
        nhc=True,
    )
    assert len(p) == 17
    assert p[16] == AID_FLAG_NHC


def test_short_vesc_parser_resynchronizes_after_noise_and_bad_candidate():
    payload = build_wheel_payload(5, 1000, 1.5, 0.05, 0.2, True)
    frame = vesc_packet(payload)
    parser = ShortVescParser()

    out = parser.feed(b"\x99\x00\x02\xff" + frame[:7])
    assert out == []
    out = parser.feed(frame[7:])
    assert out == [payload]
