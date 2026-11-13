"""ROS 2 <-> ESP32 serial bridge.

ESP32 -> Pi (published):
  R,<m>,<link>   ->  ultrasonic/range   sensor_msgs/Range    (+inf when nothing is in range)
                     esp32/link_ok      std_msgs/Bool        (ESP32 hears our heartbeat)
  S,<fs>,<t>,<s> ->  esp32/failsafe     std_msgs/Bool        (ESP32 is forcing neutral)
  V,<version>    ->  logged             (firmware version banner at boot)

Pi -> ESP32 (sent):
  H                  heartbeat, heartbeat_hz (10)
  C,<thr>,<steer>    normalised -1..1 actuator command, cmd_rate_hz (50), from cmd_vel

Subscribes cmd_vel (geometry_msgs/Twist) and converts it either with
diffdrive.py (drive_mode 'diff': C,<left>,<right>, the custom two-wheel robot)
or ackermann.py (drive_mode 'ackermann': C,<throttle>,<steer>, an RC car).
The ESP32 just outputs two servo/ESC pulse channels either way. If cmd_vel is
older than cmd_timeout the bridge sends neutral. If the bridge dies entirely
the ESP32's own 200 ms command watchdog goes to neutral. Also publishes
esp32/actuators (geometry_msgs/Vector3: x = throttle, y = steer) for tuning.

Reconnects automatically if the USB cable is unplugged and plugged back in.
"""

import math

import rclpy
import serial
from geometry_msgs.msg import Twist, Vector3
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Bool

from honebot_hardware.ackermann import AckermannParams, twist_to_ackermann
from honebot_hardware.diffdrive import DiffDriveParams, twist_to_diffdrive


class Esp32Bridge(Node):
    def __init__(self):
        super().__init__('esp32_bridge')
        self.port = self.declare_parameter('port', '/dev/ttyUSB0').value
        self.baud = self.declare_parameter('baud', 115200).value
        self.frame_id = self.declare_parameter('frame_id', 'ultrasonic_link').value
        heartbeat_hz = self.declare_parameter('heartbeat_hz', 10.0).value
        cmd_rate_hz = self.declare_parameter('cmd_rate_hz', 50.0).value
        self.cmd_timeout = self.declare_parameter('cmd_timeout', 0.3).value
        self.send_commands = self.declare_parameter('send_commands', True).value
        self.drive_mode = self.declare_parameter('drive_mode', 'diff').value
        if self.drive_mode not in ('diff', 'ackermann'):
            self.get_logger().error(f"drive_mode must be 'diff' or 'ackermann', not {self.drive_mode!r}; using 'diff'")
            self.drive_mode = 'diff'
        self.diff = DiffDriveParams(
            track_width=self.declare_parameter('track_width', 0.40).value,
            max_wheel_speed=self.declare_parameter('max_wheel_speed', 7.0).value,
            left_reversed=self.declare_parameter('left_reversed', False).value,
            right_reversed=self.declare_parameter('right_reversed', False).value,
        )
        self.ack = AckermannParams(
            wheelbase=self.declare_parameter('wheelbase', 0.33).value,
            max_steer_angle=self.declare_parameter('max_steer_angle', 0.45).value,
            max_speed=self.declare_parameter('max_speed', 5.0).value,
            min_speed_for_steer=self.declare_parameter('min_speed_for_steer', 0.3).value,
            steer_reversed=self.declare_parameter('steer_reversed', False).value,
            throttle_reversed=self.declare_parameter('throttle_reversed', False).value,
        )

        self.range_pub = self.create_publisher(Range, 'ultrasonic/range', 10)
        self.link_pub = self.create_publisher(Bool, 'esp32/link_ok', 10)
        self.failsafe_pub = self.create_publisher(Bool, 'esp32/failsafe', 10)
        self.act_pub = self.create_publisher(Vector3, 'esp32/actuators', 10)
        self.create_subscription(Twist, 'cmd_vel', self._on_cmd, 10)

        self.ser = None
        self.buf = b''
        self.last_link_ok = None
        self.last_failsafe = None
        self.bad_lines = 0
        self.cmd = None
        self.cmd_stamp = None
        self._warned_stale = False

        self.create_timer(0.01, self._poll)                 # read serial at 100 Hz
        self.create_timer(1.0 / heartbeat_hz, self._heartbeat)
        self.create_timer(1.0 / cmd_rate_hz, self._send_cmd)
        self.create_timer(1.0, self._ensure_open)
        self._ensure_open()
        self.get_logger().info(f'drive_mode = {self.drive_mode} '
                               f'(C line = {"left,right" if self.drive_mode == "diff" else "throttle,steer"})')

    # --- serial connection -----------------------------------------------
    def _ensure_open(self):
        if self.ser is not None:
            return
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0)
            self.buf = b''
            self.get_logger().info(f'Connected to ESP32 on {self.port}')
        except serial.SerialException as e:
            self.ser = None
            self.get_logger().warn(f'Cannot open {self.port}: {e}', throttle_duration_sec=5.0)

    def _drop(self, why):
        self.get_logger().error(f'Serial error ({why}); will reconnect')
        try:
            self.ser.close()
        except Exception:
            pass
        self.ser = None

    def _heartbeat(self):
        if self.ser is None:
            return
        try:
            self.ser.write(b'H\n')
        except (serial.SerialException, OSError) as e:
            self._drop(e)

    # --- actuator commands -----------------------------------------------
    def _on_cmd(self, msg: Twist):
        self.cmd = msg
        self.cmd_stamp = self.get_clock().now()

    def _send_cmd(self):
        if self.ser is None or not self.send_commands:
            return
        fresh = False
        if self.cmd_stamp is not None:
            age = (self.get_clock().now() - self.cmd_stamp).nanoseconds * 1e-9
            fresh = age <= self.cmd_timeout
        if fresh:
            if self.drive_mode == 'diff':
                throttle, steer = twist_to_diffdrive(self.cmd.linear.x, self.cmd.angular.z, self.diff)
            else:
                throttle, steer = twist_to_ackermann(self.cmd.linear.x, self.cmd.angular.z, self.ack)
            if self._warned_stale:
                self.get_logger().info('cmd_vel is back; sending commands')
                self._warned_stale = False
        else:
            throttle, steer = 0.0, 0.0
            if self.cmd_stamp is not None and not self._warned_stale:
                self.get_logger().warn('cmd_vel stale; sending neutral')
                self._warned_stale = True
        try:
            self.ser.write(f'C,{throttle:.3f},{steer:.3f}\n'.encode('ascii'))
        except (serial.SerialException, OSError) as e:
            self._drop(e)
            return
        self.act_pub.publish(Vector3(x=float(throttle), y=float(steer), z=0.0))

    def _poll(self):
        if self.ser is None:
            return
        try:
            data = self.ser.read(self.ser.in_waiting or 1)
        except (serial.SerialException, OSError) as e:
            self._drop(e)
            return
        if not data:
            return
        self.buf += data
        while b'\n' in self.buf:
            line, self.buf = self.buf.split(b'\n', 1)
            self._handle(line.strip().decode('ascii', errors='replace'))
        if len(self.buf) > 256:          # no newline in a long time: junk
            self.buf = b''

    # --- message handling ------------------------------------------------
    def _handle(self, line):
        if not line:
            return
        parts = line.split(',')
        if parts[0] == 'R' and len(parts) == 3:
            try:
                dist = float(parts[1])
                link_ok = parts[2] == '1'
            except ValueError:
                self._bad(line)
                return
            self._publish_range(dist)
            self._publish_link(link_ok)
        elif parts[0] == 'S' and len(parts) >= 2:
            self._publish_failsafe(parts[1] == '1')
        elif parts[0] == 'V' and len(parts) == 2:
            self.get_logger().info(f'ESP32 firmware version {parts[1]}')
        else:
            self._bad(line)

    def _bad(self, line):
        # The ESP32 prints boot messages when the port opens; ignore quietly.
        self.bad_lines += 1
        self.get_logger().debug(f'Ignoring line: {line!r}')

    def _publish_range(self, dist):
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = 0.26        # ~15 degrees for an HC-SR04
        msg.min_range = 0.02
        msg.max_range = 4.0
        # REP 117: +inf means "no object detected within range"
        msg.range = dist if dist >= 0.0 else math.inf
        self.range_pub.publish(msg)

    def _publish_link(self, ok):
        self.link_pub.publish(Bool(data=ok))
        if ok != self.last_link_ok:
            if ok:
                self.get_logger().info('ESP32 reports heartbeat OK')
            else:
                self.get_logger().warn('ESP32 reports NO heartbeat')
            self.last_link_ok = ok

    def _publish_failsafe(self, active):
        self.failsafe_pub.publish(Bool(data=active))
        if active != self.last_failsafe:
            if active:
                self.get_logger().warn('ESP32 FAILSAFE: motors forced to neutral')
            else:
                self.get_logger().info('ESP32 failsafe cleared; motors live')
            self.last_failsafe = active


def main(args=None):
    rclpy.init(args=args)
    node = Esp32Bridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # Ctrl-C or SIGTERM (launch/systemd stopping us): exit cleanly
    finally:
        if node.ser is not None:
            try:
                node.ser.write(b'C,0.000,0.000\n')
            except Exception:
                pass
            node.ser.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
