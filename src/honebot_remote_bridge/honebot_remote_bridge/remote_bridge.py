#!/usr/bin/env python3
"""Handheld remote (+ the follow-bot code) -> the Twist that moves the "robot" turtle.

source:=serial (bench: only the remote board, on this machine's USB)
    Reads the remote's own printout (`state=RUN x=+0.50 y=-0.20 ...`, 20 per second) on `port`, and takes the
    follow-bot code's commands on `cmd_vel_in` (Twist). Decides who drives exactly like the robot firmware
    (remote_logic.Arbiter = drive::decide): remote STOP or lost -> nobody; RUN + stick pushed -> the stick
    (same deadband, mixing and 2.4 m/s limit as the firmware); stick released for 0.8 s -> the follower again.
    No ramps, no safety gate (no ultrasonic sensor without the robot ESP32).

source:=esp32 (the real chain: remote --ESP-NOW--> robot ESP32 <--USB-- esp32_bridge <-- safety_gate <-- follower)
    The robot ESP32's firmware makes that decision itself. This node only mirrors the result: it subscribes
    esp32/outputs (geometry_msgs/Vector3: x = left, y = right, z = failsafe) from esp32_bridge, i.e. the wheel
    commands the robot is really producing after kill switch, ramps and limits.

Publishes `cmd_vel_out` (geometry_msgs/Twist at `rate_hz`, zeros when nobody drives; remap it to the turtle),
`remote/state` (never | lost | stop | run) and `remote/source` (none | pi | manual), both std_msgs/String.
`scale` slows the stick's speed for the small turtlesim window; the follower's commands are never scaled.

On the real robot nothing here is in the driving path: the robot ESP32 takes the stick directly over ESP-NOW,
even with the Pi off.
"""

import rclpy
import serial
from geometry_msgs.msg import Twist, Vector3
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String

from honebot_remote_bridge.remote_logic import (FULL_SCALE_MPS, SRC_MANUAL, SRC_NONE, SRC_PI, TRACK_WIDTH,
                                                Arbiter, ArbiterConfig, parse_remote_line, remote_to_wheels,
                                                wheels_to_twist)


class RemoteBridge(Node):
    def __init__(self):
        super().__init__('remote_bridge')
        self.source = self.declare_parameter('source', 'serial').value
        self.port = self.declare_parameter('port', '/dev/ttyUSB0').value
        self.baud = self.declare_parameter('baud', 115200).value
        self.remote_timeout = self.declare_parameter('remote_timeout', 0.5).value
        manual_hold = self.declare_parameter('manual_hold', 0.8).value
        cmd_timeout = self.declare_parameter('cmd_timeout', 0.5).value
        rate_hz = self.declare_parameter('rate_hz', 20.0).value
        self.track_width = self.declare_parameter('track_width', TRACK_WIDTH).value
        self.full_scale_mps = self.declare_parameter('full_scale_mps', FULL_SCALE_MPS).value
        self.scale = self.declare_parameter('scale', 1.0).value
        if self.source not in ('serial', 'esp32'):
            self.get_logger().error(f"source must be 'serial' or 'esp32', not {self.source!r}; using 'serial'")
            self.source = 'serial'

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_out', 10)
        self.state_pub = self.create_publisher(String, 'remote/state', 10)
        self.source_pub = self.create_publisher(String, 'remote/source', 10)

        self.ser = None
        self.buf = b''
        self.remote = None              # serial: last parsed RemoteState
        self.remote_time = None
        self.pi_cmd = None              # serial: last follower Twist
        self.pi_time = None
        self.arbiter = Arbiter(ArbiterConfig(remote_timeout=self.remote_timeout, manual_hold=manual_hold,
                                             cmd_timeout=cmd_timeout))
        self.outputs = None             # esp32: last (left, right)
        self.outputs_time = None
        self.esp32_manual = False
        self.esp32_manual_time = None
        self.esp32_remote = None
        self.esp32_remote_time = None
        self.last_logged = (None, None)

        if self.source == 'serial':
            self.create_subscription(Twist, 'cmd_vel_in', self._on_pi_cmd, 10)
            self.create_timer(0.01, self._poll)          # read serial at 100 Hz, never blocks
            self.create_timer(1.0, self._ensure_open)
            self._ensure_open()
        else:
            self.create_subscription(Vector3, 'esp32/outputs', self._on_outputs, 10)
            self.create_subscription(Bool, 'esp32/manual', self._on_esp32_manual, 10)
            self.create_subscription(String, 'esp32/remote', self._on_esp32_remote, 10)
        self.create_timer(1.0 / rate_hz, self._publish)
        self.get_logger().info(
            f'source={self.source}' + (f' port={self.port}, follower commands on cmd_vel_in'
                                       if self.source == 'serial' else ' (mirroring esp32/outputs)')
            + f', stick scale={self.scale}')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

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
            text = line.decode('ascii', errors='replace')
            s = parse_remote_line(text)
            if s is not None:
                self.remote, self.remote_time = s, self._now()
            elif text.strip():
                self.get_logger().debug(f'Ignoring line: {text.strip()!r}')
        if len(self.buf) > 512:          # no newline for a long time: junk
            self.buf = b''

    def _on_pi_cmd(self, msg: Twist):
        self.pi_cmd, self.pi_time = msg, self._now()

    # --- esp32 source -----------------------------------------------------------
    def _on_outputs(self, msg: Vector3):
        self.outputs, self.outputs_time = (float(msg.x), float(msg.y)), self._now()

    def _on_esp32_manual(self, msg: Bool):
        self.esp32_manual = bool(msg.data)
        self.esp32_manual_time = self._now()

    def _on_esp32_remote(self, msg: String):
        self.esp32_remote = msg.data
        self.esp32_remote_time = self._now()

    # --- output -----------------------------------------------------------------
    def _publish(self):
        now = self._now()
        linear = angular = 0.0
        if self.source == 'serial':
            source, rstate = self.arbiter.decide(now, self.remote, self.remote_time, self.pi_time)
            if source == SRC_MANUAL:
                left, right = remote_to_wheels(self.remote)
                linear, angular = wheels_to_twist(left, right, self.track_width, self.full_scale_mps, self.scale)
            elif source == SRC_PI:
                linear, angular = self.pi_cmd.linear.x, self.pi_cmd.angular.z
        else:
            fresh = self.outputs_time is not None and now - self.outputs_time <= self.remote_timeout
            manual_fresh = (self.esp32_manual_time is not None
                            and now - self.esp32_manual_time <= self.remote_timeout)
            remote_fresh = (self.esp32_remote_time is not None
                            and now - self.esp32_remote_time <= self.remote_timeout)
            manual = fresh and self.esp32_manual and manual_fresh
            if fresh:
                scale = self.scale if manual else 1.0
                linear, angular = wheels_to_twist(*self.outputs, self.track_width, self.full_scale_mps, scale)
            if self.esp32_remote is None:
                rstate = 'never'
            elif fresh and remote_fresh:
                rstate = self.esp32_remote
            else:
                rstate = 'lost'
            moving = abs(linear) > 1e-3 or abs(angular) > 1e-3
            source = SRC_MANUAL if manual else (SRC_PI if (fresh and moving) else SRC_NONE)

        if (source, rstate) != self.last_logged:
            self.get_logger().info(f'Remote: {rstate}, driving: {source}')
            self.last_logged = (source, rstate)
        self.state_pub.publish(String(data=rstate))
        self.source_pub.publish(String(data=source))
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
