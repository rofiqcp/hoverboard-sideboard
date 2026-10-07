#!/usr/bin/env python3
from __future__ import annotations

import math
import os
import tempfile
import time
from typing import Optional, Tuple

import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32

from .fusion_core import (
    P2Config, P2FusionCore, clamp, ewma, transform_planar_pose, wrap_pi
)


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def quaternion_to_yaw(q) -> float:
    # Standard ROS quaternion (x,y,z,w), ENU yaw about +Z.
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class P2FusionNode(Node):
    def __init__(self) -> None:
        super().__init__("sideboard_p2_fusion")

        self.declare_parameter("wheel_odom_topic", "/wheel/odometry_raw")
        self.declare_parameter("steering_topic", "/steering/angle_rad")
        self.declare_parameter("lio_odom_topic", "/Odometry")
        self.declare_parameter("lio_canonical_topic", "/p2/lio/odometry")
        self.declare_parameter("lio_output_frame", "odom")
        self.declare_parameter("lio_body_to_base_x_m", 0.0)
        self.declare_parameter("lio_body_to_base_y_m", 0.0)
        self.declare_parameter("lio_body_to_base_yaw_rad", 0.0)
        self.declare_parameter("lio_position_sigma_floor_m", 0.03)
        self.declare_parameter("lio_yaw_sigma_floor_rad", 0.02)
        self.declare_parameter("external_slip_ratio_topic", "/sideboard/slip_ratio")
        self.declare_parameter("output_wheel_odom_topic", "/p2/wheel/odometry")
        self.declare_parameter("diagnostics_topic", "/p2/diagnostics")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("wheel_frame", "base_link")
        self.declare_parameter("steering_timeout_s", 0.25)
        self.declare_parameter("lio_velocity_filter_tau_s", 0.08)
        self.declare_parameter("lio_yaw_rate_filter_tau_s", 0.08)
        self.declare_parameter("lio_max_abs_speed_mps", 35.0)
        self.declare_parameter("lio_max_abs_yaw_rate_rads", 5.0)
        self.declare_parameter("wheel_input_abs_max_mps", 30.0)
        self.declare_parameter("steering_input_abs_max_rad", 1.2)
        self.declare_parameter(
            "calibration_file",
            os.path.expanduser("~/.config/hoverboard_sideboard/p2_calibration.yaml"),
        )
        self.declare_parameter("calibration_save_interval_s", 3.0)
        self.declare_parameter("steering_input_scale", 1.0)
        self.declare_parameter("nhc_sigma_base_mps", 0.08)
        self.declare_parameter("yaw_rate_sigma_base_rads", 0.08)

        defaults = P2Config()
        for name, value in defaults.__dict__.items():
            self.declare_parameter(name, value)

        cfg_values = {
            name: self.get_parameter(name).value
            for name in defaults.__dict__.keys()
        }
        self.core = P2FusionCore(P2Config(**cfg_values))

        self.calibration_file = os.path.expanduser(
            str(self.get_parameter("calibration_file").value)
        )
        self.save_interval_s = float(
            self.get_parameter("calibration_save_interval_s").value
        )
        self.last_saved_monotonic = 0.0
        self.last_saved_update_count = -1
        self._load_calibration()

        qos = QoSProfile(
            depth=50,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        qos_sensor = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.wheel_pub = self.create_publisher(
            Odometry,
            str(self.get_parameter("output_wheel_odom_topic").value),
            qos,
        )
        self.lio_pub = self.create_publisher(
            Odometry,
            str(self.get_parameter("lio_canonical_topic").value),
            qos,
        )
        self.diag_pub = self.create_publisher(
            DiagnosticArray,
            str(self.get_parameter("diagnostics_topic").value),
            qos,
        )

        self.slip_prob_pub = self.create_publisher(
            Float32, "/p2/slip_probability", qos
        )
        self.slip_vel_pub = self.create_publisher(
            Float32, "/p2/slip_velocity_mps", qos
        )
        self.slip_ratio_pub = self.create_publisher(
            Float32, "/p2/slip_ratio", qos
        )
        self.scale_pub = self.create_publisher(
            Float32, "/p2/wheel_scale", qos
        )
        self.offset_pub = self.create_publisher(
            Float32, "/p2/steering_zero_rad", qos
        )
        self.wheelbase_pub = self.create_publisher(
            Float32, "/p2/wheelbase_m", qos
        )
        self.expected_yaw_pub = self.create_publisher(
            Float32, "/p2/expected_yaw_rate", qos
        )
        self.steering_valid_pub = self.create_publisher(
            Bool, "/p2/steering_valid", qos
        )
        self.sigma_pub = self.create_publisher(
            Float32, "/p2/wheel_sigma_mps", qos
        )

        self.create_subscription(
            Odometry,
            str(self.get_parameter("wheel_odom_topic").value),
            self._wheel_cb,
            qos_sensor,
        )
        self.create_subscription(
            Float32,
            str(self.get_parameter("steering_topic").value),
            self._steering_cb,
            qos_sensor,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("lio_odom_topic").value),
            self._lio_cb,
            qos_sensor,
        )
        self.create_subscription(
            Float32,
            str(self.get_parameter("external_slip_ratio_topic").value),
            self._external_slip_cb,
            qos_sensor,
        )

        self.steering_angle_rad = 0.0
        self.steering_stamp_s = 0.0

        self.last_lio_pose: Optional[Tuple[float, float, float, float]] = None
        self.filtered_lio_speed = 0.0
        self.filtered_lio_yaw_rate = 0.0
        self.have_filtered_lio = False
        self.last_lio_update_s = 0.0

        self.last_output = None
        self.last_wheel_stamp_s = 0.0
        self.timer = self.create_timer(0.5, self._periodic)

        self.get_logger().info(
            "P2 fusion ready: wheel=%s lio=%s steering=%s"
            % (
                self.get_parameter("wheel_odom_topic").value,
                self.get_parameter("lio_odom_topic").value,
                self.get_parameter("steering_topic").value,
            )
        )

    def _load_calibration(self) -> None:
        if not self.calibration_file:
            return
        try:
            with open(self.calibration_file, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
            self.core.restore(data)
            self.get_logger().info(
                "Loaded P2 calibration: scale=%.6f steering_zero=%.6f wheelbase=%.4f"
                % (
                    self.core.wheel_scale,
                    self.core.steering_zero_rad,
                    self.core.wheelbase_m,
                )
            )
        except FileNotFoundError:
            pass
        except Exception as exc:
            self.get_logger().warning(
                "Ignoring invalid P2 calibration file %s: %s"
                % (self.calibration_file, exc)
            )

    def _save_calibration(self) -> None:
        if not self.calibration_file:
            return
        directory = os.path.dirname(self.calibration_file)
        if directory:
            os.makedirs(directory, exist_ok=True)
        data = self.core.snapshot()
        data["saved_unix_s"] = time.time()
        fd, tmp = tempfile.mkstemp(
            prefix=".p2_calibration.", suffix=".tmp", dir=directory or None
        )
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as f:
                yaml.safe_dump(data, f, sort_keys=True)
                f.flush()
                os.fsync(f.fileno())
            os.replace(tmp, self.calibration_file)
        finally:
            try:
                if os.path.exists(tmp):
                    os.unlink(tmp)
            except OSError:
                pass

    def _message_time(self, msg) -> float:
        t = stamp_to_sec(msg.header.stamp)
        if t <= 0.0:
            return self.get_clock().now().nanoseconds * 1.0e-9
        return t

    def _steering_cb(self, msg: Float32) -> None:
        value = float(msg.data) * float(
            self.get_parameter("steering_input_scale").value
        )
        if math.isfinite(value) and abs(value) <= float(
            self.get_parameter("steering_input_abs_max_rad").value
        ):
            self.steering_angle_rad = value
            self.steering_stamp_s = (
                self.get_clock().now().nanoseconds * 1.0e-9
            )

    def _external_slip_cb(self, msg: Float32) -> None:
        self.core.set_external_slip_ratio(float(msg.data), True)

    def _lio_cb(self, msg: Odometry) -> None:
        stamp_s = self._message_time(msg)
        raw_x = float(msg.pose.pose.position.x)
        raw_y = float(msg.pose.pose.position.y)
        raw_yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        if not all(math.isfinite(v) for v in (stamp_s, raw_x, raw_y, raw_yaw)):
            return

        x, y, yaw = transform_planar_pose(
            raw_x,
            raw_y,
            raw_yaw,
            float(self.get_parameter("lio_body_to_base_x_m").value),
            float(self.get_parameter("lio_body_to_base_y_m").value),
            float(self.get_parameter("lio_body_to_base_yaw_rad").value),
        )

        if self.last_lio_pose is not None:
            t0, x0, y0, yaw0 = self.last_lio_pose
            dt = stamp_s - t0
            if 0.005 <= dt <= 0.50:
                dx = x - x0
                dy = y - y0
                # Project world displacement onto body-forward at the midpoint
                # heading to avoid sign errors during turns.
                yaw_mid = yaw0 + 0.5 * wrap_pi(yaw - yaw0)
                speed = (dx * math.cos(yaw_mid) + dy * math.sin(yaw_mid)) / dt
                yaw_rate = wrap_pi(yaw - yaw0) / dt
                if (
                    abs(speed) > float(self.get_parameter("lio_max_abs_speed_mps").value)
                    or abs(yaw_rate)
                    > float(self.get_parameter("lio_max_abs_yaw_rate_rads").value)
                ):
                    # Relocalization/frame jumps must not poison online scale,
                    # slip probability, or the geometry estimator.
                    self.have_filtered_lio = False
                    self.last_lio_pose = (stamp_s, x, y, yaw)
                    return

                tau_v = float(
                    self.get_parameter("lio_velocity_filter_tau_s").value
                )
                tau_w = float(
                    self.get_parameter("lio_yaw_rate_filter_tau_s").value
                )
                if not self.have_filtered_lio:
                    self.filtered_lio_speed = speed
                    self.filtered_lio_yaw_rate = yaw_rate
                    self.have_filtered_lio = True
                else:
                    self.filtered_lio_speed = ewma(
                        self.filtered_lio_speed, speed, dt, tau_v
                    )
                    self.filtered_lio_yaw_rate = ewma(
                        self.filtered_lio_yaw_rate, yaw_rate, dt, tau_w
                    )
                self.core.update_reference_velocity(
                    stamp_s,
                    self.filtered_lio_speed,
                    self.filtered_lio_yaw_rate,
                )
                self.last_lio_update_s = stamp_s

        self.last_lio_pose = (stamp_s, x, y, yaw)
        self._publish_canonical_lio(msg, x, y, yaw)

    def _publish_canonical_lio(
        self, source: Odometry, x_m: float, y_m: float, yaw_rad: float
    ) -> None:
        out = Odometry()
        out.header.stamp = source.header.stamp
        out.header.frame_id = str(self.get_parameter("lio_output_frame").value)
        out.child_frame_id = str(self.get_parameter("base_frame").value)
        out.pose.pose.position.x = x_m
        out.pose.pose.position.y = y_m
        out.pose.pose.position.z = 0.0
        out.pose.pose.orientation.z = math.sin(0.5 * yaw_rad)
        out.pose.pose.orientation.w = math.cos(0.5 * yaw_rad)

        pos_floor2 = float(
            self.get_parameter("lio_position_sigma_floor_m").value
        ) ** 2
        yaw_floor2 = float(
            self.get_parameter("lio_yaw_sigma_floor_rad").value
        ) ** 2
        src_cov = source.pose.covariance
        cov = [0.0] * 36
        cov[0] = max(abs(float(src_cov[0])), pos_floor2)
        cov[7] = max(abs(float(src_cov[7])), pos_floor2)
        cov[14] = 1.0e6
        cov[21] = 1.0e6
        cov[28] = 1.0e6
        cov[35] = max(abs(float(src_cov[35])), yaw_floor2)
        out.pose.covariance = cov

        twist_cov = [0.0] * 36
        for i in (0, 7, 14, 21, 28, 35):
            twist_cov[i] = 1.0e6
        out.twist.covariance = twist_cov
        self.lio_pub.publish(out)

    def _steering_for_wheel(self, wheel_stamp_s: float):
        now_s = self.get_clock().now().nanoseconds * 1.0e-9
        timeout = float(self.get_parameter("steering_timeout_s").value)
        valid = (
            self.steering_stamp_s > 0.0
            and (now_s - self.steering_stamp_s) <= timeout
        )
        return (
            self.steering_angle_rad if valid else self.core.steering_zero_rad,
            valid,
        )

    def _wheel_cb(self, msg: Odometry) -> None:
        stamp_s = self._message_time(msg)
        raw_speed = float(msg.twist.twist.linear.x)
        if (
            not math.isfinite(raw_speed)
            or abs(raw_speed)
            > float(self.get_parameter("wheel_input_abs_max_mps").value)
        ):
            return
        steering, steering_valid = self._steering_for_wheel(stamp_s)
        try:
            out = self.core.update_wheel(
                stamp_s, raw_speed, steering, steering_valid=steering_valid
            )
        except ValueError as exc:
            self.get_logger().warning("Rejected P2 wheel sample: %s" % exc)
            return

        self.last_output = out
        self.last_wheel_stamp_s = stamp_s

        corrected = Odometry()
        corrected.header = msg.header
        corrected.child_frame_id = (
            msg.child_frame_id
            if msg.child_frame_id
            else str(self.get_parameter("wheel_frame").value)
        )
        corrected.pose.pose = msg.pose.pose
        pose_cov = [0.0] * 36
        for i in (0, 7, 14, 21, 28, 35):
            pose_cov[i] = 1.0e6
        corrected.pose.covariance = pose_cov
        corrected.twist.twist.linear.x = out.corrected_wheel_speed_mps
        corrected.twist.twist.linear.y = 0.0
        corrected.twist.twist.linear.z = 0.0
        corrected.twist.twist.angular.z = out.expected_yaw_rate_rads

        cov = [0.0] * 36
        cov[0] = out.wheel_sigma_mps * out.wheel_sigma_mps
        nhc_sigma = float(self.get_parameter("nhc_sigma_base_mps").value)
        nhc_sigma *= 1.0 + 12.0 * out.slip_probability * out.slip_probability
        cov[7] = nhc_sigma * nhc_sigma
        cov[14] = 1000.0
        cov[21] = 1000.0
        cov[28] = 1000.0
        yaw_sigma = float(
            self.get_parameter("yaw_rate_sigma_base_rads").value
        )
        yaw_sigma *= 1.0 + 8.0 * out.slip_probability * out.slip_probability
        if not out.steering_valid:
            yaw_sigma = max(yaw_sigma, 10.0)
        cov[35] = yaw_sigma * yaw_sigma
        if out.severe_slip:
            cov[0] = max(cov[0], 25.0)
            cov[7] = max(cov[7], 25.0)
            cov[35] = max(cov[35], 4.0)
        corrected.twist.covariance = cov
        self.wheel_pub.publish(corrected)

        self._publish_scalar(self.slip_prob_pub, out.slip_probability)
        self._publish_scalar(self.slip_vel_pub, out.slip_velocity_mps)
        self._publish_scalar(self.slip_ratio_pub, out.signed_slip_ratio)
        self._publish_scalar(self.scale_pub, out.wheel_scale)
        self._publish_scalar(self.offset_pub, out.steering_zero_rad)
        self._publish_scalar(self.wheelbase_pub, out.wheelbase_m)
        self._publish_scalar(self.expected_yaw_pub, out.expected_yaw_rate_rads)
        valid_msg = Bool()
        valid_msg.data = bool(out.steering_valid)
        self.steering_valid_pub.publish(valid_msg)
        self._publish_scalar(self.sigma_pub, out.wheel_sigma_mps)

    @staticmethod
    def _publish_scalar(pub, value: float) -> None:
        m = Float32()
        m.data = float(value)
        pub.publish(m)

    def _periodic(self) -> None:
        self._publish_diagnostics()
        updates = self.core.scale_update_count + self.core.geometry_update_count
        now = time.monotonic()
        if (
            updates != self.last_saved_update_count
            and (now - self.last_saved_monotonic) >= self.save_interval_s
        ):
            try:
                self._save_calibration()
                self.last_saved_update_count = updates
                self.last_saved_monotonic = now
            except Exception as exc:
                self.get_logger().error("Failed saving P2 calibration: %s" % exc)

    def _publish_diagnostics(self) -> None:
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        st = DiagnosticStatus()
        st.name = "sideboard_p2_fusion"
        st.hardware_id = "NUC/P2"

        now_s = self.get_clock().now().nanoseconds * 1.0e-9
        ref_age = (
            now_s - self.last_lio_update_s
            if self.last_lio_update_s > 0.0
            else 1.0e9
        )
        wheel_age = (
            now_s - self.last_wheel_stamp_s
            if self.last_wheel_stamp_s > 0.0
            else 1.0e9
        )

        if self.last_output is None:
            st.level = DiagnosticStatus.WARN
            st.message = "waiting for wheel odometry"
        elif self.last_output.severe_slip:
            st.level = DiagnosticStatus.ERROR
            st.message = "severe wheel slip; covariance maximally inflated"
        elif ref_age > self.core.cfg.reference_timeout_s:
            st.level = DiagnosticStatus.WARN
            st.message = "LIO reference stale; calibration/slip comparison paused"
        elif self.last_output.slip_probability > 0.50:
            st.level = DiagnosticStatus.WARN
            st.message = "wheel slip probability elevated"
        else:
            st.level = DiagnosticStatus.OK
            st.message = "P2 fusion healthy"

        values = {
            "wheel_scale": self.core.wheel_scale,
            "wheel_scale_covariance": self.core.scale_rls.covariance,
            "steering_zero_rad": self.core.steering_zero_rad,
            "wheelbase_m": self.core.wheelbase_m,
            "scale_update_count": self.core.scale_update_count,
            "geometry_update_count": self.core.geometry_update_count,
            "lio_reference_age_s": ref_age,
            "wheel_age_s": wheel_age,
            "slip_probability": (
                self.last_output.slip_probability
                if self.last_output is not None
                else self.core.slip_probability
            ),
            "slip_velocity_mps": self.core.slip_velocity_mps,
        }
        if self.last_output is not None:
            values.update(
                {
                    "wheel_sigma_mps": self.last_output.wheel_sigma_mps,
                    "signed_slip_ratio": self.last_output.signed_slip_ratio,
                    "speed_residual_mps": self.last_output.speed_residual_mps,
                    "acceleration_mismatch_mps2": self.last_output.acceleration_mismatch_mps2,
                    "yaw_mismatch_rads": self.last_output.yaw_mismatch_rads,
                    "reference_valid": int(self.last_output.reference_valid),
                    "severe_slip": int(self.last_output.severe_slip),
                    "steering_valid": int(self.last_output.steering_valid),
                }
            )
        st.values = [
            KeyValue(key=str(k), value=str(v)) for k, v in values.items()
        ]
        arr.status = [st]
        self.diag_pub.publish(arr)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = P2FusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._save_calibration()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
