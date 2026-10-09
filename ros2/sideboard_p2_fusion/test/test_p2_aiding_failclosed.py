"""P2 aiding must not fabricate measurement age, quality, or steering freshness."""
from collections import OrderedDict
from types import SimpleNamespace
import struct
import time

from nav_msgs.msg import Odometry

from sideboard_p2_fusion.serial_aiding_bridge import SerialAidingBridge


class FakePort:
    is_open = True

    def __init__(self):
        self.written = []

    def write(self, data):
        self.written.append(bytes(data))
        return len(data)

    def flush(self):
        pass


class FakeBridge:
    _stamp_to_sec = staticmethod(SerialAidingBridge._stamp_to_sec)
    _steering_is_fresh = SerialAidingBridge._steering_is_fresh
    _wheel_cb = SerialAidingBridge._wheel_cb
    params = {
        "max_age_us": 14000,
        "max_future_skew_us": 10000,
        "unknown_wheel_sigma_mps": 0.50,
        "steering_valid_timeout_s": 0.25,
        "nhc": True,
    }

    def __init__(self):
        self.port = FakePort()
        self.pending_requests = OrderedDict()
        self.request_id = 1
        self.sent_count = 0
        self.stale_drop_count = 0
        self.last_send_monotonic = 0.0
        self.steering_valid = True
        self.steering_last_update_monotonic = time.monotonic()
        self.now_ns = 10_000_000_000

    def get_parameter(self, name):
        return SimpleNamespace(value=self.params[name])

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=self.now_ns))


def measurement(t=10.0, variance=0.04):
    msg = Odometry()
    sec = int(t)
    msg.header.stamp.sec = sec
    msg.header.stamp.nanosec = round((t-sec)*1e9)
    msg.twist.twist.linear.x = 2.0
    msg.twist.twist.angular.z = 0.7
    msg.twist.covariance[0] = variance
    return msg


def sent_payload(bridge):
    return bridge.port.written[-1][2:-3]


def test_unstamped_wheel_is_dropped_without_sideboard_tx():
    b = FakeBridge()
    b._wheel_cb(measurement(t=0.0))
    assert b.stale_drop_count == 1
    assert not b.port.written


def test_old_and_far_future_messages_are_rejected():
    b = FakeBridge()
    b._wheel_cb(measurement(t=9.98))
    b._wheel_cb(measurement(t=10.02))
    assert b.stale_drop_count == 2
    assert not b.port.written


def test_small_future_skew_is_accepted_but_age_is_never_negative():
    b = FakeBridge()
    b._wheel_cb(measurement(t=10.005))
    assert b.sent_count == 1
    assert struct.unpack(">I", sent_payload(b)[6:10])[0] == 0


def test_valid_wheel_uncertainty_is_preserved():
    b = FakeBridge()
    b._wheel_cb(measurement(variance=.04))
    assert struct.unpack(">H", sent_payload(b)[14:16])[0] == 200


def test_unknown_covariance_never_becomes_001_mps():
    for variance in (0.0, -1.0, float("nan"), float("inf")):
        b = FakeBridge()
        b._wheel_cb(measurement(variance=variance))
        assert b.sent_count == 1
        assert struct.unpack(">H", sent_payload(b)[14:16])[0] == 500


def test_stale_steering_does_not_authorize_yaw_rate_aid():
    b = FakeBridge()
    b.steering_last_update_monotonic = time.monotonic() - 1.0
    b._wheel_cb(measurement())
    assert len(sent_payload(b)) == 17
    assert not sent_payload(b)[16] & (1 << 1)


def test_fresh_steering_restores_optional_yaw_rate_aid():
    b = FakeBridge()
    b._wheel_cb(measurement())
    assert len(sent_payload(b)) == 21
    assert struct.unpack(">i", sent_payload(b)[17:21])[0] == 700
