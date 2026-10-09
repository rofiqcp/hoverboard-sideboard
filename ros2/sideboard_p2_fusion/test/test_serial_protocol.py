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


def test_identity_requires_real_sideboard_application_telemetry():
    from sideboard_p2_fusion.serial_aiding_bridge import valid_sideboard_telemetry
    assert valid_sideboard_telemetry(bytes((0xF0, 6)) + bytes(116))
    assert valid_sideboard_telemetry(bytes((0xF5, 2)) + bytes(75))
    assert valid_sideboard_telemetry(bytes((0xF5, 1)) + bytes(67))
    assert not valid_sideboard_telemetry(bytes((0xF5, 2)) + bytes(70))
    assert not valid_sideboard_telemetry(bytes((0xF5, 1)) + bytes(75))
    assert not valid_sideboard_telemetry(bytes((0xF0, 6)) + bytes(117))
    assert not valid_sideboard_telemetry(bytes((0xF0, 5)) + bytes(116))
    assert not valid_sideboard_telemetry(bytes((0xF0, 6)) + bytes(25))
    assert not valid_sideboard_telemetry(bytes((0xF0, 4)) + bytes(116))
    assert not valid_sideboard_telemetry(bytes((0x04, 6)) + bytes(116))


def test_ack_must_match_pending_request_id_and_ignore_legacy_reply():
    from collections import OrderedDict
    from types import SimpleNamespace
    from sideboard_p2_fusion.serial_aiding_bridge import SerialAidingBridge
    fake=SimpleNamespace(
        pending_requests=OrderedDict([(1234,0.0)]),
        ack_count=0,ack_unmatched_count=0,
        ack_reject_count=0,last_ack_monotonic=0.0,
        last_ack_status=-1,last_ack_age_ms=0,
    )
    old_ack=bytes((0xF4,1,0,0,1))  # legacy 5B ACK without correlation
    wrong_ack=bytes((0xF4,1,0,0,1))+struct.pack(">H",1235)
    good_ack=bytes((0xF4,1,0,0,1))+struct.pack(">H",1234)
    SerialAidingBridge._handle_payload(fake,old_ack)
    assert fake.ack_count==0 and len(fake.pending_requests)==1
    SerialAidingBridge._handle_payload(fake,wrong_ack)
    assert fake.ack_count==0 and fake.ack_unmatched_count==1
    SerialAidingBridge._handle_payload(fake,good_ack)
    assert fake.ack_count==1 and not fake.pending_requests
    assert fake.last_ack_status==0 and fake.last_ack_monotonic>0


def test_ack_timeout_is_counted_and_request_dropped():
    from collections import OrderedDict
    from types import SimpleNamespace
    from sideboard_p2_fusion.serial_aiding_bridge import SerialAidingBridge
    from unittest.mock import Mock
    import time
    fake=SimpleNamespace(
        pending_requests=OrderedDict([(7,time.monotonic()-3)]),
        ack_timeout_count=0,
        last_ack_timeout_monotonic=0.0,
        get_parameter=lambda _:SimpleNamespace(value=0.25),
    )
    SerialAidingBridge._expire_acks(fake)
    assert not fake.pending_requests and fake.ack_timeout_count==1
    assert fake.last_ack_timeout_monotonic>0

def test_hotplug_auto_bridge_reconnects_only_original_usb_adapter():
    from types import SimpleNamespace
    from unittest import mock
    from sideboard_p2_fusion.serial_aiding_bridge import SerialAidingBridge

    def port(device, location, serial=None):
        return SimpleNamespace(device=device, vid=0x1A86, pid=0x7523,
                               serial_number=serial, location=location)
    fake = SimpleNamespace(trusted_adapter=dict(
        vid=0x1A86, pid=0x7523, serial=None, location="1-2.4", device="/dev/ttyUSB0"
    ))
    with mock.patch(
        "sideboard_p2_fusion.serial_aiding_bridge.list_ports.comports",
        return_value=[port("/dev/ttyUSB1", "1-2.7"),
                      port("/dev/ttyUSB9", "1-2.4")],
    ):
        candidates = SerialAidingBridge._linux_candidates(fake)
    assert candidates == ["/dev/ttyUSB9"]


def test_hotplug_auto_bridge_does_not_switch_to_other_sideboard():
    from types import SimpleNamespace
    from unittest import mock
    from sideboard_p2_fusion.serial_aiding_bridge import SerialAidingBridge

    trusted = dict(vid=0x1A86, pid=0x7523, serial="SIDEB001",
                   location="1-3", device="/dev/ttyUSB0")
    fake = SimpleNamespace(trusted_adapter=trusted)
    others = [
        SimpleNamespace(device="/dev/ttyUSB2", vid=0x1A86, pid=0x7523,
                        serial_number="SIDEB002", location="1-3"),
        SimpleNamespace(device="/dev/ttyUSB3", vid=0x1A86, pid=0x7523,
                        serial_number="SIDEB001", location="1-4"),
    ]
    with mock.patch(
        "sideboard_p2_fusion.serial_aiding_bridge.list_ports.comports",
        return_value=others,
    ):
        assert SerialAidingBridge._linux_candidates(fake) == []


def test_hotplug_auto_bridge_rejects_ambiguous_duplicate_usb_identity():
    from types import SimpleNamespace
    from unittest import mock
    from sideboard_p2_fusion.serial_aiding_bridge import SerialAidingBridge

    fake = SimpleNamespace(trusted_adapter=dict(
        vid=0x1A86, pid=0x7523, serial="duplicate", location=None,
        device="/dev/ttyUSB0"
    ))
    ports = [
        SimpleNamespace(device="/dev/ttyUSB3", vid=0x1A86, pid=0x7523,
                        serial_number="duplicate", location="1-3"),
        SimpleNamespace(device="/dev/ttyUSB4", vid=0x1A86, pid=0x7523,
                        serial_number="duplicate", location="1-4"),
    ]
    with mock.patch(
        "sideboard_p2_fusion.serial_aiding_bridge.list_ports.comports",
        return_value=ports,
    ):
        assert SerialAidingBridge._linux_candidates(fake) == []

def test_ros_parser_recovers_valid_frame_behind_truncated_header():
    p=ShortVescParser()
    payload=bytes((0xF0,6))+bytes(116)
    frame=vesc_packet(payload)
    assert p.feed(b"\x02\xc0"+frame[:8]) == []
    assert p.feed(frame[8:]) == [payload]


def test_ros_parser_buffer_is_bounded_under_extended_serial_noise():
    p=ShortVescParser()
    for _ in range(200):
        p.feed(b"\x02\xc0"+b"\xaa"*198)
    assert len(p.buffer) <= 512
    good=bytes((0xF5,2))+bytes(75)
    assert p.feed(vesc_packet(good)) == [good]


def test_ros_parser_reconnect_fragmented_telemetry_after_noisy_prefix():
    p=ShortVescParser()
    payload=bytes((0xF0,6))+bytes(116)
    stream=b"\x02\xc0garbage"+vesc_packet(payload)
    found=[]
    for idx in range(0,len(stream),7):
        found.extend(p.feed(stream[idx:idx+7]))
    assert found == [payload]
