#!/usr/bin/env python3
"""Handheld remote -> ROS 2 Twist, so the remote can drive a turtle (or show what the robot is doing).

source:=serial (bench, no robot ESP32 needed)
    Reads the remote board's own USB printout (`state=RUN x=+0.50 y=-0.20 ...`, 5 per second) on `port`
    and applies the robot firmware's rules: only RUN drives, same deadband, mixing and speed limit
    (remote_logic.remote_to_wheels). No ramps. Nothing heard for `remote_timeout` -> stop, like the robot.

source:=esp32 (the real chain: remote --ESP-NOW--> robot ESP32 --USB--> esp32_bridge)
    Subscribes esp32/outputs (geometry_msgs/Vector3: x = left, y = right, z = failsafe), published by
    esp32_bridge from the robot ESP32's S line: what the wheels are really doing, whoever drives them
    (stick or Pi), after the kill switch, ramps and limits. Needs esp32_bridge running on the robot ESP32.

Either way the wheel commands become a Twist on `cmd_vel_out` (remap it, e.g. to /turtle1/cmd_vel),
published at `rate_hz` (zeros when stopped or stale), and the remote state on `remote/state` (std_msgs/String:
run | stop | fault | lost, serial source only).

On the real robot nothing here is in the driving path: the robot ESP32 takes the stick directly over ESP-NOW,
even with the Pi off. This node only mirrors it.
"""

import rclpy
import serial
from geometry_msgs.msg import Twist, Vector3
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from honebot_remote_bridge.remote_logic import (FULL_SCALE_MPS, TRACK_WIDTH, parse_remote_line,
                                                remote_to_wheels, wheels_to_twist)


class RemoteBridge(Node):
    def __init__(self):
        super().__init__('remote_bridge')
        self.source = self.declare_parameter('source', 'serial').value
        self.port = self.declare_parameter('port', '/dev/ttyUSB0').value
        self.baud = self.declare_parameter('baud', 115200).value
        self.remote_timeout = self.declare_parameter('remote_timeout', 0.5).value
        rate_hz = self.declare_parameter('rate_hz', 20.0).value
        self.track_width = self.declare_parameter('track_width', TRACK_WIDTH).value
        self.full_scale_mps = self.declare_parameter('full_scale_mps', FULL_SCALE_MPS).value
        self.scale = self.declare_parameter('scale', 1.0).value
        if self.source not in ('serial', 'esp32'):
            self.get_logger().error(f"source must be 'serial' or 'esp32', not {self.source!r}; using 'serial'")
            self.source = 'serial'

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_out', 10)
        self.state_pub = self.create_publisher(String, 'remote/state', 10)

        self.ser = None
        self.buf = b''
        self.latest = None              # (left, right) wheel commands
        self.latest_stamp = None
        self.last_state = None

        if self.source == 'serial':
            self.create_timer(0.01, self._poll)          # read serial at 100 Hz, never blocks
            self.create_timer(1.0, self._ensure_open)
            self._ensure_open()
        else:
            self.create_subscription(Vector3, 'esp32/outputs', self._on_outputs, 10)
        self.create_timer(1.0 / rate_hz, self._publish)
        self.get_logger().info(
            f'source={self.source}' + (f' port={self.port}' if self.source == 'serial' else ' (esp32/outputs)')
            + f', scale={self.scale}')

    # --- serial source ----------------------------------------------------------
    def _ensure_open(self):
        if self.ser is not None:
            return
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0)
            self.buf = b''
            self.get_logger().info(f'Connected to the remote on {self.port}')
        except (serial.SerialException, OSError) as e:
            self.ser = None
            self.get_logger().warn(f'Cannot open {self.port}: {e}', throttle_duration_sec=5.0)

    def _poll(self):
        if self.ser is None:
            return
        try:
            data = self.ser.read(self.ser.in_waiting or 1)
        except (serial.SerialException, OSError) as e:
            self.get_logger().error(f'Serial error ({e}); will reconnect')
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
            return
        if not data:
            return
        self.buf += data
        while b'\n' in self.buf:
            line, self.buf = self.buf.split(b'\n', 1)
            self._handle_line(line.decode('ascii', errors='replace'))
        if len(self.buf) > 512:          # no newline for a long time: junk
            self.buf = b''

    def _handle_line(self, line):
        s = parse_remote_line(line)
        if s is None:
            if line.strip():
                self.get_logger().debug(f'Ignoring line: {line.strip()!r}')
            return
        self.latest = remote_to_wheels(s)
        self.latest_stamp = self.get_clock().now()
        self._set_state(s.state.lower())

    # --- esp32 source -----------------------------------------------------------
    def _on_outputs(self, msg: Vector3):
        self.latest = (float(msg.x), float(msg.y))
        self.latest_stamp = self.get_clock().now()

    # --- output -----------------------------------------------------------------
    def _set_state(self, state):
        if state != self.last_state:
            self.get_logger().info(f'Remote: {state}')
            self.last_state = state

    def _publish(self):
        fresh = False
        if self.latest_stamp is not None:
            age = (self.get_clock().now() - self.latest_stamp).nanoseconds * 1e-9
            fresh = age <= self.remote_timeout
        left, right = self.latest if fresh else (0.0, 0.0)
        if self.source == 'serial':
            if not fresh and self.last_state not in (None, 'lost'):
                self._set_state('lost')
            if self.last_state is not None:
                self.state_pub.publish(String(data=self.last_state))
        linear, angular = wheels_to_twist(left, right, self.track_width, self.full_scale_mps, self.scale)
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RemoteBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.cmd_pub.publish(Twist())      # leave the turtle stopped
        except Exception:
            pass
        if node.ser is not None:
            node.ser.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
