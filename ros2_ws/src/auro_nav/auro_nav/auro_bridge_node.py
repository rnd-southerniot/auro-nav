#!/usr/bin/env python3
"""
auro_bridge_node — ROS2 <-> Airborne Classic NDJSON serial bridge

Runs on Pi 5. Connects to the RP2040 via UART (GPIO14 TX / GPIO15 RX).
Translates:
  - /cmd_vel (Twist)         -> set_rpm command over NDJSON
  - NDJSON tele messages     -> /odom (Odometry) + TF (odom->base_link)
  - NDJSON tele yaw          -> fused into odometry orientation

Hardware wiring (Pi 5 -> RP2040):
  Pi 5 GPIO14 (TX)  ->  RP2040 GP1 (UART0 RX)
  Pi 5 GPIO15 (RX)  ->  RP2040 GP0 (UART0 TX)
  GND               ->  GND

Usage:
  ros2 run auro_nav auro_bridge_node
  # or launch via nav2_launch.py
"""

import json
import math
import time
import threading
import uuid
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import (
    Twist,
    TransformStamped,
    Quaternion,
    Vector3,
)
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster

import serial


# ── Robot kinematics (must match firmware / HARDWARE_MAP.md) ─────────
WHEEL_DIAMETER_M = 0.054875
WHEEL_RADIUS_M = WHEEL_DIAMETER_M / 2.0
TRACK_WIDTH_M = 0.155
COUNTS_PER_REV_L = 582.0
COUNTS_PER_REV_R = 583.0

# ── Derived constants ────────────────────────────────────────────────
METERS_PER_COUNT_L = (math.pi * WHEEL_DIAMETER_M) / COUNTS_PER_REV_L
METERS_PER_COUNT_R = (math.pi * WHEEL_DIAMETER_M) / COUNTS_PER_REV_R


def yaw_to_quaternion(yaw_rad: float) -> Quaternion:
    """Convert yaw angle (radians) to quaternion."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw_rad / 2.0)
    q.w = math.cos(yaw_rad / 2.0)
    return q


class AuroBridgeNode(Node):
    def __init__(self):
        super().__init__('auro_bridge')

        # ── Parameters ───────────────────────────────────────────────
        self.declare_parameter('serial_port', '/dev/ttyAMA0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('max_linear_vel', 0.5)    # m/s cap
        self.declare_parameter('max_angular_vel', 3.0)    # rad/s cap
        self.declare_parameter('cmd_vel_timeout', 0.5)    # seconds

        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.max_linear = self.get_parameter('max_linear_vel').value
        self.max_angular = self.get_parameter('max_angular_vel').value
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value

        # ── State ────────────────────────────────────────────────────
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0        # radians, from IMU yaw
        self.vx = 0.0
        self.vtheta = 0.0
        self.prev_enc_l = None
        self.prev_enc_r = None
        self.prev_tele_ts = None
        self.last_cmd_vel_time = 0.0
        self.armed = False
        self.ser: Optional[serial.Serial] = None
        self.ser_lock = threading.Lock()

        # ── Publishers ───────────────────────────────────────────────
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ── Subscribers ──────────────────────────────────────────────
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )

        # ── Serial connection ────────────────────────────────────────
        self.connect_serial()

        # ── Timers ───────────────────────────────────────────────────
        # Serial read at high rate
        self.create_timer(0.005, self.serial_read_callback)  # 200 Hz
        # cmd_vel timeout watchdog
        self.create_timer(0.1, self.watchdog_callback)

        self.get_logger().info(
            f'AuroBridge started: {self.serial_port} @ {self.baud_rate}'
        )

    def connect_serial(self):
        """Open serial connection to RP2040."""
        try:
            self.ser = serial.Serial(
                self.serial_port,
                self.baud_rate,
                timeout=0.01,
            )
            self.get_logger().info(f'Serial connected: {self.serial_port}')
            # Arm the robot and calibrate IMU
            time.sleep(0.5)
            self.send_cmd('cal_imu')
            time.sleep(1.5)  # wait for IMU calibration
            self.send_cmd('cal_encoders')
            self.send_cmd('arm')
            self.armed = True
        except serial.SerialException as e:
            self.get_logger().error(f'Serial connect failed: {e}')
            self.ser = None

    def send_cmd(self, cmd: str, **kwargs):
        """Send NDJSON command to RP2040."""
        if not self.ser or not self.ser.is_open:
            return
        msg = {
            'type': 'cmd',
            'id': str(uuid.uuid4())[:8],
            'ts': int(time.time() * 1000),
            'cmd': cmd,
        }
        msg.update(kwargs)
        line = json.dumps(msg, separators=(',', ':')) + '\n'
        with self.ser_lock:
            try:
                self.ser.write(line.encode('ascii'))
                self.ser.flush()
            except serial.SerialException as e:
                self.get_logger().warn(f'Serial write error: {e}')

    def cmd_vel_callback(self, msg: Twist):
        """Convert /cmd_vel to set_rpm command for RP2040."""
        self.last_cmd_vel_time = time.time()

        # Clamp velocities
        linear = max(-self.max_linear, min(self.max_linear, msg.linear.x))
        angular = max(-self.max_angular, min(self.max_angular, msg.angular.z))

        # Differential drive kinematics: v_l = v - omega*L/2, v_r = v + omega*L/2
        v_left = linear - (angular * TRACK_WIDTH_M / 2.0)
        v_right = linear + (angular * TRACK_WIDTH_M / 2.0)

        # Convert m/s to RPM: RPM = (v / (pi * D)) * 60
        circumference = math.pi * WHEEL_DIAMETER_M
        rpm_left = (v_left / circumference) * 60.0 if circumference > 0 else 0.0
        rpm_right = (v_right / circumference) * 60.0 if circumference > 0 else 0.0

        self.send_cmd('set_rpm', left=round(rpm_left, 1), right=round(rpm_right, 1))

    def watchdog_callback(self):
        """Stop robot if no cmd_vel received recently."""
        if self.armed and (time.time() - self.last_cmd_vel_time) > self.cmd_vel_timeout:
            if self.vx != 0.0 or self.vtheta != 0.0:
                self.send_cmd('stop')
                self.vx = 0.0
                self.vtheta = 0.0

    def serial_read_callback(self):
        """Read and process NDJSON lines from RP2040."""
        if not self.ser or not self.ser.is_open:
            return

        with self.ser_lock:
            try:
                while self.ser.in_waiting > 0:
                    raw = self.ser.readline()
                    if not raw:
                        break
                    line = raw.decode('ascii', errors='ignore').strip()
                    if not line or line.startswith('#'):
                        continue
                    self.process_ndjson(line)
            except serial.SerialException as e:
                self.get_logger().warn(f'Serial read error: {e}')

    def process_ndjson(self, line: str):
        """Parse NDJSON telemetry and publish odometry."""
        try:
            msg = json.loads(line)
        except json.JSONDecodeError:
            return

        if msg.get('type') != 'tele':
            return

        enc_l = msg.get('enc_l', 0)
        enc_r = msg.get('enc_r', 0)
        rpm_l = msg.get('rpm_l', 0.0)
        rpm_r = msg.get('rpm_r', 0.0)
        yaw_deg = msg.get('yaw', 0.0)
        ts_ms = msg.get('ts', 0)

        # ── Compute odometry from encoder deltas ─────────────────────
        if self.prev_enc_l is not None and self.prev_tele_ts is not None:
            delta_l = enc_l - self.prev_enc_l
            delta_r = enc_r - self.prev_enc_r
            delta_ts = ts_ms - self.prev_tele_ts

            if delta_ts > 0:
                dt = delta_ts / 1000.0

                # Distance traveled by each wheel
                dist_l = delta_l * METERS_PER_COUNT_L
                dist_r = delta_r * METERS_PER_COUNT_R

                # Robot displacement
                dist_center = (dist_l + dist_r) / 2.0

                # Use IMU yaw (more accurate than encoder-derived heading)
                self.theta = math.radians(yaw_deg)

                # Update position
                self.x += dist_center * math.cos(self.theta)
                self.y += dist_center * math.sin(self.theta)

                # Compute velocities
                self.vx = dist_center / dt
                d_theta_enc = (dist_r - dist_l) / TRACK_WIDTH_M
                self.vtheta = d_theta_enc / dt

        self.prev_enc_l = enc_l
        self.prev_enc_r = enc_r
        self.prev_tele_ts = ts_ms

        # ── Publish odometry ─────────────────────────────────────────
        now = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header = Header()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = yaw_to_quaternion(self.theta)

        # Pose covariance (diagonal, tuned for encoder + IMU fusion)
        odom.pose.covariance[0] = 0.01   # x
        odom.pose.covariance[7] = 0.01   # y
        odom.pose.covariance[35] = 0.03  # yaw

        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.vtheta

        # Twist covariance
        odom.twist.covariance[0] = 0.01   # vx
        odom.twist.covariance[35] = 0.03  # vyaw

        self.odom_pub.publish(odom)

        # ── Broadcast TF: odom -> base_link ──────────────────────────
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = yaw_to_quaternion(self.theta)
        self.tf_broadcaster.sendTransform(t)

    def destroy_node(self):
        """Clean shutdown: disarm and close serial."""
        if self.ser and self.ser.is_open:
            self.send_cmd('disarm')
            self.send_cmd('stop')
            time.sleep(0.1)
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AuroBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
