#!/usr/bin/env python3
from __future__ import annotations

import glob
import math
import os
import struct
import time
from pathlib import Path
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

PREFERRED_LINUX = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
PREFERRED_USB_VIDS = {0x1A86, 0x067B}


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


class ShortVescParser:
    def __init__(self) -> None:
        self.buffer = bytearray()

    def feed(self, data: bytes):
        self.buffer.extend(data)
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


class SerialAidingBridge(Node):
    def __init__(self) -> None:
        super().__init__("sideboard_serial_aiding_bridge")
        self.declare_parameter("input_topic", "/p2/wheel/odometry")
        self.declare_parameter("port", "auto")
        self.declare_parameter("baud", 921600)
        self.declare_parameter("nhc", True)
        self.declare_parameter("max_age_us", 14000)
        self.declare_parameter("reconnect_interval_s", 0.5)
        self.declare_parameter("diagnostics_topic", "/p2/serial_bridge_diagnostics")
        self.declare_parameter("steering_valid_topic", "/p2/steering_valid")

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
        self.last_ack_status = -1
        self.last_ack_age_ms = 0
        self.last_connect_attempt = 0.0
        self.connected_name = ""
        self.last_send_monotonic = 0.0
        self.steering_valid = False

        self.timer = self.create_timer(0.02, self._service)
        self.diag_timer = self.create_timer(0.5, self._publish_diag)

    def _steering_valid_cb(self, msg: Bool) -> None:
        self.steering_valid = bool(msg.data)

    @staticmethod
    def _linux_candidates():
        out = [PREFERRED_LINUX]
        out += sorted(glob.glob("/dev/serial/by-id/*USB*Serial*"))
        out += sorted(glob.glob("/dev/serial/by-id/*USB-Serial*"))
        out += sorted(glob.glob("/dev/ttyUSB*"))
        out += sorted(glob.glob("/dev/ttyACM*"))
        seen = set()
        return [p for p in out if not (p in seen or seen.add(p))]

    def _find_port(self) -> str:
        requested = str(self.get_parameter("port").value)
        if requested.lower() != "auto":
            if Path(requested).exists():
                return requested
            raise FileNotFoundError(requested)

        for candidate in self._linux_candidates():
            if Path(candidate).exists():
                return candidate

        # Final pyserial enumeration catches unusual USB-UART names.
        for p in list_ports.comports():
            if p.vid in PREFERRED_USB_VIDS:
                return p.device
        raise FileNotFoundError("sideboard serial not detected")

    def _connect(self) -> None:
        now = time.monotonic()
        interval = float(self.get_parameter("reconnect_interval_s").value)
        if now - self.last_connect_attempt < interval:
            return
        self.last_connect_attempt = now
        try:
            name = self._find_port()
            s = serial.Serial(
                port=None,
                baudrate=int(self.get_parameter("baud").value),
                timeout=0.0,
                write_timeout=0.05,
                rtscts=False,
                dsrdtr=False,
                exclusive=True,
            )
            s.dtr = False
            s.rts = False
            s.port = name
            s.open()
            self.port = s
            self.connected_name = name
            self.parser = ShortVescParser()
            self.get_logger().info("P2 serial aiding connected: %s" % name)
        except Exception:
            self.port = None
            self.connected_name = ""

    def _disconnect(self, reason: str) -> None:
        self.io_error_count += 1
        try:
            if self.port is not None:
                self.port.close()
        except Exception:
            pass
        self.port = None
        self.connected_name = ""
        self.get_logger().warning("P2 serial aiding disconnected: %s" % reason)

    @staticmethod
    def _stamp_to_sec(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9

    def _wheel_cb(self, msg: Odometry) -> None:
        if self.port is None or not self.port.is_open:
            return

        now_s = self.get_clock().now().nanoseconds * 1.0e-9
        stamp_s = self._stamp_to_sec(msg.header.stamp)
        age_us = 0
        if stamp_s > 0.0:
            age_us = max(0, round((now_s - stamp_s) * 1.0e6))
        max_age_us = int(self.get_parameter("max_age_us").value)
        if age_us > max_age_us:
            self.stale_drop_count += 1
            return

        velocity = float(msg.twist.twist.linear.x)
        yaw_rate = float(msg.twist.twist.angular.z)
        variance = float(msg.twist.covariance[0])
        sigma = math.sqrt(max(variance, 1.0e-6))
        if not all(math.isfinite(v) for v in (velocity, yaw_rate, sigma)):
            return
        sigma = min(max(sigma, 0.01), 5.0)

        payload = build_wheel_payload(
            self.request_id,
            age_us,
            velocity,
            sigma,
            yaw_rate if self.steering_valid else None,
            bool(self.get_parameter("nhc").value),
        )
        packet = vesc_packet(payload)
        try:
            self.port.write(packet)
            self.port.flush()
            self.sent_count += 1
            self.last_send_monotonic = time.monotonic()
            self.request_id = (self.request_id + 1) & 0xFFFF
            if self.request_id == 0:
                self.request_id = 1
        except (serial.SerialException, OSError) as exc:
            self._disconnect(str(exc))

    def _service(self) -> None:
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
            len(payload) < 5
            or payload[0] != COMM_SIDEBOARD_AIDING
            or payload[1] != AID_CMD_WHEEL_BODY_X
        ):
            return
        self.ack_count += 1
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
        elif self.last_ack_status not in (-1, 0):
            st.level = DiagnosticStatus.WARN
            st.message = "sideboard aiding ACK reports rejection"
        else:
            st.level = DiagnosticStatus.OK
            st.message = "serial aiding healthy"

        values = {
            "port": self.connected_name or "none",
            "sent_count": self.sent_count,
            "ack_count": self.ack_count,
            "ack_reject_count": self.ack_reject_count,
            "stale_drop_count": self.stale_drop_count,
            "io_error_count": self.io_error_count,
            "last_ack_status": self.last_ack_status,
            "last_ack_age_ms": self.last_ack_age_ms,
            "steering_valid": int(self.steering_valid),
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
