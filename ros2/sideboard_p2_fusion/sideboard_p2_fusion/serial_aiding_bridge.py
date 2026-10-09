#!/usr/bin/env python3
from __future__ import annotations

import math
import os
import struct
import time
from pathlib import Path
from collections import OrderedDict
from typing import Optional

import serial
from serial.tools import list_ports

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


COMM_SIDEBOARD_AIDING = 0xF4
AID_CMD_WHEEL_BODY_X = 1
AID_TIMING_AGE_US = 2
AID_FRAME_BODY = 2
AID_FLAG_NHC = 1 << 0
AID_FLAG_EXPECTED_YAW_RATE = 1 << 1

PREFERRED_USB_VIDS = {0x1A86, 0x067B, 0x0403}  # CH340, Prolific, FTDI


def crc16(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = (
                ((crc << 1) ^ 0x1021) & 0xFFFF
                if crc & 0x8000
                else (crc << 1) & 0xFFFF
            )
    return crc


def vesc_packet(payload: bytes) -> bytes:
    if len(payload) == 0 or len(payload) > 255:
        raise ValueError("invalid short VESC payload length")
    c = crc16(payload)
    return bytes((2, len(payload))) + payload + bytes((c >> 8, c & 0xFF, 3))


def build_wheel_payload(
    request_id: int,
    age_us: int,
    velocity_mps: float,
    sigma_mps: float,
    expected_yaw_rate_rads,
    nhc: bool,
) -> bytes:
    flags = 0
    if expected_yaw_rate_rads is not None:
        flags |= AID_FLAG_EXPECTED_YAW_RATE
    if nhc:
        flags |= AID_FLAG_NHC
    payload = bytes(
        (
            COMM_SIDEBOARD_AIDING,
            AID_CMD_WHEEL_BODY_X,
            AID_TIMING_AGE_US,
            AID_FRAME_BODY,
        )
    )
    payload += struct.pack(">H", request_id & 0xFFFF)
    payload += struct.pack(">I", max(0, min(int(age_us), 0xFFFFFFFF)))
    payload += struct.pack(
        ">iHB",
        round(velocity_mps * 1000.0),
        max(1, min(round(sigma_mps * 1000.0), 65535)),
        flags,
    )
    if expected_yaw_rate_rads is not None:
        payload += struct.pack(">i", round(expected_yaw_rate_rads * 1000.0))
    return payload


def _nested_vesc_frame_start(buf):
    """Re-sync a truncated candidate to an interior complete CRC-valid frame."""
    for pos in range(1, max(1, len(buf)-4)):
        if buf[pos] != 2 or pos+5 > len(buf):
            continue
        n = buf[pos+1]
        if not 0 < n <= 192 or pos+n+5 > len(buf):
            continue
        end = pos+n+2
        if (buf[end+2] == 3 and
                crc16(bytes(buf[pos+2:end])) == ((buf[end] << 8)|buf[end+1])):
            return pos
    return None


class ShortVescParser:
    def __init__(self) -> None:
        self.buffer = bytearray()

    def feed(self, data: bytes):
        self.buffer.extend(data)
        if len(self.buffer) > 512:
            del self.buffer[:-256]
        out = []
        while True:
            try:
                start = self.buffer.index(2)
            except ValueError:
                self.buffer.clear()
                break
            if start:
                del self.buffer[:start]
            if len(self.buffer) < 2:
                break
            n = self.buffer[1]
            if n == 0 or n > 192:
                del self.buffer[0]
                continue
            total = 2 + n + 3
            if len(self.buffer) < total:
                nested = _nested_vesc_frame_start(self.buffer)
                if nested is not None:
                    del self.buffer[:nested]
                    continue
                break
            payload = bytes(self.buffer[2 : 2 + n])
            crc_rx = (self.buffer[2 + n] << 8) | self.buffer[3 + n]
            stop = self.buffer[4 + n]
            if stop == 3 and crc16(payload) == crc_rx:
                del self.buffer[:total]
                out.append(payload)
            else:
                del self.buffer[0]
        return out


def valid_sideboard_telemetry(payload: bytes) -> bool:
    """Identity is a CRC-validated F0/F5 application frame, never a USB VID alone."""
    if not payload:
        return False
    if len(payload) < 2:
        return False
    if payload[0] == 0xF0:
        return (payload[1], len(payload)) in ((5, 114), (6, 118))
    if payload[0] == 0xF5:
        return (payload[1], len(payload)) in ((1, 69), (2, 77))
    return False


def probe_sideboard_identity(port: serial.Serial, timeout_s: float = 0.8) -> bool:
    """Passive probe. Never write a command to an unverified USB-UART port."""
    parser = ShortVescParser()
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        try:
            waiting = port.in_waiting
            data = port.read(min(max(waiting, 1), 512))
        except (serial.SerialException, OSError):
            return False
        if any(valid_sideboard_telemetry(p) for p in parser.feed(data)):
            return True
        if not data:
            time.sleep(0.005)
    return False


class SerialAidingBridge(Node):
    def __init__(self) -> None:
        super().__init__("sideboard_serial_aiding_bridge")
        self.declare_parameter("input_topic", "/p2/wheel/odometry")
        self.declare_parameter("port", "auto")
        self.declare_parameter("baud", 921600)
        self.declare_parameter("nhc", True)
        self.declare_parameter("max_age_us", 14000)
        self.declare_parameter("max_future_skew_us", 10000)
        self.declare_parameter("unknown_wheel_sigma_mps", 0.50)
        self.declare_parameter("steering_valid_timeout_s", 0.25)
        self.declare_parameter("reconnect_interval_s", 0.5)
        self.declare_parameter("diagnostics_topic", "/p2/serial_bridge_diagnostics")
        self.declare_parameter("steering_valid_topic", "/p2/steering_valid")
        self.declare_parameter("ack_timeout_s", 0.25)
        self.declare_parameter("ack_stale_s", 1.0)

        qos = QoSProfile(
            depth=50,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("input_topic").value),
            self._wheel_cb,
            qos,
        )
        self.diag_pub = self.create_publisher(
            DiagnosticArray,
            str(self.get_parameter("diagnostics_topic").value),
            qos,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("steering_valid_topic").value),
            self._steering_valid_cb,
            qos,
        )

        self.port: Optional[serial.Serial] = None
        self.parser = ShortVescParser()
        self.request_id = 1
        self.sent_count = 0
        self.stale_drop_count = 0
        self.io_error_count = 0
        self.ack_count = 0
        self.ack_reject_count = 0
        self.ack_timeout_count = 0
        self.ack_unmatched_count = 0
        self.pending_requests = OrderedDict()
        self.last_ack_monotonic = 0.0
        self.last_ack_timeout_monotonic = 0.0
        self.last_ack_status = -1
        self.last_ack_age_ms = 0
        self.last_connect_attempt = 0.0
        self.connected_name = ""
        self.trusted_adapter = None  # sticky USB identity survives a disconnect
        self.last_send_monotonic = 0.0
        self.steering_valid = False
        self.steering_last_update_monotonic = 0.0

        self.timer = self.create_timer(0.02, self._service)
        self.diag_timer = self.create_timer(0.5, self._publish_diag)

    def _steering_valid_cb(self, msg: Bool) -> None:
        self.steering_valid = bool(msg.data)
        self.steering_last_update_monotonic = time.monotonic()

    def _steering_is_fresh(self) -> bool:
        return (self.steering_valid
                and self.steering_last_update_monotonic > 0.0
                and (time.monotonic() - self.steering_last_update_monotonic)
                <= float(self.get_parameter("steering_valid_timeout_s").value))

    def _linux_candidates(self):
        # After a verified connection, USB hotplug may renumber ttyUSBn.
        # Restrict reconnection to the SAME adapter serial/topology, never a
        # different sideboard/ESC that happens to share a USB-UART VID.
        ports = [p for p in list_ports.comports() if p.vid in PREFERRED_USB_VIDS]
        trusted = self.trusted_adapter
        if trusted is not None:
            ports = [
                p for p in ports
                if p.vid == trusted["vid"] and p.pid == trusted["pid"]
                and (not trusted["serial"] or p.serial_number == trusted["serial"])
                and (not trusted["location"] or p.location == trusted["location"])
                and (trusted["serial"] or trusted["location"] or
                     p.device == trusted["device"])
            ]
            if len(ports) != 1:
                return []  # detached or ambiguous identity: no writes
        return [p.device for p in sorted(ports, key=lambda p: p.device)]

    @staticmethod
    def _capture_adapter(name):
        for p in list_ports.comports():
            if p.device == name:
                return dict(vid=p.vid, pid=p.pid, serial=p.serial_number,
                            location=p.location, device=name)
        return None

    def _port_candidates(self):
        requested = str(self.get_parameter("port").value)
        if requested.lower() != "auto":
            if not Path(requested).exists():
                raise FileNotFoundError(requested)
            return [requested]
        ports = self._linux_candidates()
        if not ports:
            raise FileNotFoundError("no supported USB-UART sideboard candidates")
        return ports

    def _connect(self) -> None:
        now = time.monotonic()
        interval = float(self.get_parameter("reconnect_interval_s").value)
        if now - self.last_connect_attempt < interval:
            return
        self.last_connect_attempt = now
        for name in self._port_candidates_safe():
            s = None
            try:
                s = serial.Serial(
                    port=None, baudrate=int(self.get_parameter("baud").value),
                    timeout=0.02, write_timeout=0.05,
                    rtscts=False, dsrdtr=False, exclusive=True,
                )
                s.dtr = False
                s.rts = False
                s.port = name
                s.open()
                # Verify even user-selected ports before sending navigation
                # aiding. A bootloader-only port must be explicitly upgraded
                # via the flash utility rather than receiving P2 commands.
                if not probe_sideboard_identity(s):
                    continue
                self.port = s
                s = None
                self.connected_name = name
                if str(self.get_parameter("port").value).lower() == "auto":
                    self.trusted_adapter = self._capture_adapter(name) or dict(
                        vid=None, pid=None, serial=None, location=None,
                        device=name,
                    )
                self.pending_requests.clear()
                self.last_ack_status = -1
                self.last_ack_monotonic = 0.0
                self.last_ack_timeout_monotonic = 0.0
                self.parser = ShortVescParser()
                self.get_logger().info("P2 sideboard identity verified: %s" % name)
                return
            except (serial.SerialException, OSError):
                pass
            finally:
                if s is not None and s.is_open:
                    s.close()

    def _port_candidates_safe(self):
        try:
            return self._port_candidates()
        except FileNotFoundError:
            return []

    def _disconnect(self, reason: str) -> None:
        self.io_error_count += 1
        try:
            if self.port is not None:
                self.port.close()
        except Exception:
            pass
        self.port = None
        self.connected_name = ""
        self.pending_requests.clear()
        self.last_ack_monotonic = 0.0
        self.last_ack_timeout_monotonic = 0.0
        self.last_ack_status = -1
        self.get_logger().warning("P2 serial aiding disconnected: %s" % reason)

    @staticmethod
    def _stamp_to_sec(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9

    def _wheel_cb(self, msg: Odometry) -> None:
        if self.port is None or not self.port.is_open:
            return

        now_s = self.get_clock().now().nanoseconds * 1.0e-9
        stamp_s = self._stamp_to_sec(msg.header.stamp)
        # Missing and future timestamps do NOT authorize zero-age aiding.
        # ROS time and header stamp must have the same time source.
        age_signed_us = (now_s - stamp_s) * 1.0e6
        if not math.isfinite(stamp_s) or stamp_s <= 0.0 or (
            not math.isfinite(age_signed_us)
            or age_signed_us > int(self.get_parameter("max_age_us").value)
            or age_signed_us < -int(self.get_parameter("max_future_skew_us").value)
        ):
            self.stale_drop_count += 1
            return
        age_us = max(0, round(age_signed_us))

        velocity = float(msg.twist.twist.linear.x)
        yaw_rate = float(msg.twist.twist.angular.z)
        variance = float(msg.twist.covariance[0])
        if not all(math.isfinite(v) for v in (velocity, yaw_rate)):
            return
        # ROS covariance 0/negative/nonfinite means UNKNOWN, not near-perfect
        # velocity. Use a deliberately conservative configurable uncertainty.
        sigma = (math.sqrt(variance)
                 if math.isfinite(variance) and variance > 0.0
                 else float(self.get_parameter("unknown_wheel_sigma_mps").value))
        if not math.isfinite(sigma) or sigma <= 0.0:
            return
        sigma = min(max(sigma, 0.01), 5.0)

        payload = build_wheel_payload(
            self.request_id,
            age_us,
            velocity,
            sigma,
            yaw_rate if self._steering_is_fresh() else None,
            bool(self.get_parameter("nhc").value),
        )
        packet = vesc_packet(payload)
        try:
            written = self.port.write(packet)
            if written != len(packet):
                raise serial.SerialTimeoutException("short sideboard packet write")
            self.port.flush()
            self.sent_count += 1
            self.last_send_monotonic = time.monotonic()
            self.pending_requests[self.request_id] = self.last_send_monotonic
            self.request_id = (self.request_id + 1) & 0xFFFF
            if self.request_id == 0:
                self.request_id = 1
        except (serial.SerialException, OSError) as exc:
            self._disconnect(str(exc))

    def _expire_acks(self):
        deadline = time.monotonic() - float(self.get_parameter("ack_timeout_s").value)
        while self.pending_requests:
            oldest_id, timestamp = next(iter(self.pending_requests.items()))
            if timestamp > deadline:
                break
            self.pending_requests.pop(oldest_id)
            self.ack_timeout_count += 1
            self.last_ack_timeout_monotonic = time.monotonic()

    def _service(self) -> None:
        self._expire_acks()
        if self.port is None or not self.port.is_open:
            self._connect()
            return
        try:
            waiting = self.port.in_waiting
            if waiting:
                data = self.port.read(min(waiting, 512))
                for payload in self.parser.feed(data):
                    self._handle_payload(payload)
        except (serial.SerialException, OSError) as exc:
            self._disconnect(str(exc))

    def _handle_payload(self, payload: bytes) -> None:
        if (
            len(payload) != 7
            or payload[0] != COMM_SIDEBOARD_AIDING
            or payload[1] != AID_CMD_WHEEL_BODY_X
        ):
            return
        request_id = struct.unpack(">H", payload[5:7])[0]
        if request_id not in self.pending_requests:
            self.ack_unmatched_count += 1
            return
        self.pending_requests.pop(request_id)
        self.ack_count += 1
        self.last_ack_monotonic = time.monotonic()
        status = int(payload[2])
        self.last_ack_status = status
        self.last_ack_age_ms = (payload[3] << 8) | payload[4]
        if status != 0:
            self.ack_reject_count += 1

    def _publish_diag(self) -> None:
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        st = DiagnosticStatus()
        st.name = "sideboard_p2_serial_aiding"
        st.hardware_id = self.connected_name or "serial-disconnected"

        if self.port is None or not self.port.is_open:
            st.level = DiagnosticStatus.WARN
            st.message = "waiting for sideboard serial"
        elif self.ack_count == 0 or self.last_ack_monotonic == 0.0:
            st.level = DiagnosticStatus.WARN
            st.message = "waiting for matched sideboard ACK"
        elif time.monotonic() - self.last_ack_monotonic > float(
            self.get_parameter("ack_stale_s").value
        ):
            st.level = DiagnosticStatus.WARN
            st.message = "sideboard ACK stale"
        elif self.last_ack_timeout_monotonic > 0.0 and (
            time.monotonic() - self.last_ack_timeout_monotonic
            < float(self.get_parameter("ack_stale_s").value)
        ):
            st.level = DiagnosticStatus.WARN
            st.message = "sideboard ACK timeout detected"
        elif self.last_ack_status != 0:
            st.level = DiagnosticStatus.WARN
            st.message = "sideboard aiding ACK reports rejection"
        else:
            st.level = DiagnosticStatus.OK
            st.message = "serial aiding healthy"

        values = {
            "port": self.connected_name or "none",
            "trusted_usb": self.trusted_adapter or "none",
            "sent_count": self.sent_count,
            "ack_count": self.ack_count,
            "ack_reject_count": self.ack_reject_count,
            "ack_timeout_count": self.ack_timeout_count,
            "ack_unmatched_count": self.ack_unmatched_count,
            "ack_pending": len(self.pending_requests),
            "ack_last_age_s": round(time.monotonic() - self.last_ack_monotonic, 3)
                if self.last_ack_monotonic else "unknown",
            "stale_drop_count": self.stale_drop_count,
            "io_error_count": self.io_error_count,
            "last_ack_status": self.last_ack_status,
            "last_ack_age_ms": self.last_ack_age_ms,
            "steering_valid": int(self._steering_is_fresh()),
        }
        st.values = [
            KeyValue(key=str(k), value=str(v)) for k, v in values.items()
        ]
        arr.status = [st]
        self.diag_pub.publish(arr)

    def destroy_node(self):
        try:
            if self.port is not None:
                self.port.close()
        except Exception:
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SerialAidingBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
