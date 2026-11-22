"""ROS 2 <-> ESP32 serial bridge.

ESP32 -> Pi (published):
  R,<m>,<link>   ->  ultrasonic/range   sensor_msgs/Range    (+inf when nothing is in range)
                     esp32/link_ok      std_msgs/Bool        (ESP32 hears our heartbeat)
  S,<fs>,<a>,<b> ->  esp32/failsafe     std_msgs/Bool        (ESP32 is forcing neutral)
                     esp32/outputs      geometry_msgs/Vector3 what the ESP32 really outputs, from the stick or
                                        the Pi: x = a (left wheel / throttle), y = b (right wheel / steer),
                                        z = 1.0 in failsafe. 10 Hz.
                     esp32/speed        std_msgs/Float32     own forward speed (m/s) from a, b (diff mode)
  G,<yaw>,<ok>   ->  esp32/yaw_rate     std_msgs/Float32     gyro yaw rate (rad/s, + = left), only while ok = 1
                                        (firmware >= 2.3.0 with YAW_LOOP_ENABLED). Both feed the side shot.
  V,<version>    ->  logged             (firmware version banner at boot)
  K,<req>,<remote>,<source> (firmware >= 2.2.0, handheld remote / kill switch)
                 ->  esp32/remote       std_msgs/String      never | lost | stop | run
                     esp32/manual       std_msgs/Bool        the remote's stick is driving (Pi commands ignored)

Pi -> ESP32 (sent):
  H                  heartbeat, heartbeat_hz (10)
  C,<thr>,<steer>    normalised -1..1 actuator command, cmd_rate_hz (50), from cmd_vel
  P,<pan>            camera pan servo -1..1, pan_rate_hz (20), from camera/pan_cmd (camera_aim)

Subscribes cmd_vel (geometry_msgs/Twist) and converts it either with
diffdrive.py (drive_mode 'diff': C,<left>,<right>, the custom two-wheel robot)
or ackermann.py (drive_mode 'ackermann': C,<throttle>,<steer>, an RC car).
The ESP32 just outputs two servo/ESC pulse channels either way. If cmd_vel is
older than cmd_timeout the bridge sends neutral. If the bridge dies entirely
the ESP32's own 200 ms command watchdog goes to neutral. Also publishes
esp32/actuators (geometry_msgs/Vector3: x = throttle, y = steer, z = pan) for tuning.

The camera pan channel (std_msgs/Float32 on camera/pan_cmd, -1..1) is forwarded as
`P,<pan>` whenever a value has been received; it is not safety-relevant, so no neutral on
timeout (the firmware holds the last pan and centres only at boot). Firmware < 2.1 ignores it.

Reconnects automatically if the USB cable is unplugged and plugged back in.
"""

import math

import rclpy
import serial
from geometry_msgs.msg import Twist, Vector3
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, Float32, String

from honebot_hardware.ackermann import AckermannParams, twist_to_ackermann
from honebot_hardware.diffdrive import DiffDriveParams, twist_to_diffdrive, wheels_to_speed


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
        pan_rate_hz = self.declare_parameter('pan_rate_hz', 20.0).value
        self.drive_mode = self.declare_parameter('drive_mode', 'diff').value
        if self.drive_mode not in ('diff', 'ackermann'):
            self.get_logger().error(f"drive_mode must be 'diff' or 'ackermann', not {self.drive_mode!r}; using 'diff'")
            self.drive_mode = 'diff'
        self.diff = DiffDriveParams(
            track_width=self.declare_parameter('track_width', 0.39).value,
            max_wheel_speed=self.declare_parameter('max_wheel_speed', 9.6).value,
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
        self.add_on_set_parameters_callback(self._on_params)

        self.range_pub = self.create_publisher(Range, 'ultrasonic/range', 10)
        self.link_pub = self.create_publisher(Bool, 'esp32/link_ok', 10)
        self.failsafe_pub = self.create_publisher(Bool, 'esp32/failsafe', 10)
        self.act_pub = self.create_publisher(Vector3, 'esp32/actuators', 10)
        self.outputs_pub = self.create_publisher(Vector3, 'esp32/outputs', 10)
        self.speed_pub = self.create_publisher(Float32, 'esp32/speed', 10)
        self.yaw_pub = self.create_publisher(Float32, 'esp32/yaw_rate', 10)
        self.remote_pub = self.create_publisher(String, 'esp32/remote', 10)
        self.manual_pub = self.create_publisher(Bool, 'esp32/manual', 10)
        self.create_subscription(Twist, 'cmd_vel', self._on_cmd, 10)
        self.create_subscription(Float32, 'camera/pan_cmd', self._on_pan, 10)

        self.ser = None
        self.buf = b''
        self._write_timeouts = 0
        self.last_link_ok = None
        self.last_failsafe = None
        self.last_remote = None
        self.last_manual = None
        self._warned_no_killswitch = False
        self.bad_lines = 0
        self.cmd = None
        self.cmd_stamp = None
        self._warned_stale = False
        self.pan = None               # last camera pan command, None until one arrives
        self.pan_sent = None

        self.create_timer(0.01, self._poll)                 # read serial at 100 Hz
        self.create_timer(1.0 / heartbeat_hz, self._heartbeat)
        self.create_timer(1.0 / cmd_rate_hz, self._send_cmd)
        self.create_timer(1.0 / pan_rate_hz, self._send_pan)
        self.create_timer(1.0, self._ensure_open)
        self._ensure_open()
        self.get_logger().info(f'drive_mode = {self.drive_mode} '
                               f'(C line = {"left,right" if self.drive_mode == "diff" else "throttle,steer"})')

    _MAX_WRITE_TIMEOUTS = 3
    _STARTUP_ONLY_PARAMS = {
        'port', 'baud', 'frame_id', 'heartbeat_hz', 'cmd_rate_hz', 'pan_rate_hz',
        'drive_mode', 'track_width', 'max_wheel_speed', 'left_reversed', 'right_reversed',
        'wheelbase', 'max_steer_angle', 'max_speed', 'min_speed_for_steer',
        'steer_reversed', 'throttle_reversed',
    }

    def _on_params(self, params):
        updates = []
        for p in params:
            if p.name in self._STARTUP_ONLY_PARAMS:
                return SetParametersResult(
                    successful=False, reason=f'{p.name} is fixed at startup; restart the bridge')
            if p.name == 'cmd_timeout':
                val = float(p.value)
                if not math.isfinite(val) or val <= 0.0:
                    return SetParametersResult(successful=False, reason='cmd_timeout must be finite and > 0')
                updates.append((p.name, val))
            elif p.name == 'send_commands':
                updates.append((p.name, bool(p.value)))
        for name, val in updates:
            setattr(self, name, val)
            self.get_logger().info(f'{name} = {val}')
        return SetParametersResult(successful=True)

    # --- serial connection -----------------------------------------------
    def _ensure_open(self):
        if self.ser is not None:
            return
        try:
            # Configure DTR/RTS low before open() so reconnects avoid toggling EN/IO0 on CP2102 boards.
            ser = serial.Serial()
            ser.port = self.port
            ser.baudrate = self.baud
            ser.timeout = 0
            ser.write_timeout = 0.1
            ser.dtr = False
            ser.rts = False
            ser.open()
            self.ser = ser
            self.buf = b''
            self._write_timeouts = 0
            self.get_logger().info(f'Connected to ESP32 on {self.port}')
        except (serial.SerialException, OSError) as e:
            self.ser = None
            self.get_logger().warn(f'Cannot open {self.port}: {e}', throttle_duration_sec=5.0)

    def _drop(self, why):
        self.get_logger().error(f'Serial error ({why}); will reconnect')
        try:
            self.ser.close()
        except Exception:
            pass
        self.ser = None
        self._write_timeouts = 0

    def _write(self, data: bytes) -> bool:
        try:
            self.ser.write(data)
            self._write_timeouts = 0
            return True
        except serial.SerialTimeoutException as e:
            self._write_timeouts += 1
            if self._write_timeouts >= self._MAX_WRITE_TIMEOUTS:
                self._drop(e)
            else:
                self.get_logger().warn(
                    f'Serial write timeout ({self._write_timeouts}/{self._MAX_WRITE_TIMEOUTS}): {e}',
                    throttle_duration_sec=1.0)
            return False
        except (serial.SerialException, OSError) as e:
            self._drop(e)
            return False

    def _heartbeat(self):
        if self.ser is None:
            return
        self._write(b'H\n')

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
        if not self._write(f'C,{throttle:.3f},{steer:.3f}\n'.encode('ascii')):
            return
        self.act_pub.publish(Vector3(x=float(throttle), y=float(steer),
                                     z=float(self.pan_sent) if self.pan_sent is not None else 0.0))

    # --- camera pan servo --------------------------------------------------
    def _on_pan(self, msg: Float32):
        v = float(msg.data)
        if math.isfinite(v):
            self.pan = max(-1.0, min(1.0, v))

    def _send_pan(self):
        if self.ser is None or not self.send_commands or self.pan is None:
            return
        if not self._write(f'P,{self.pan:.3f}\n'.encode('ascii')):
            return
        self.pan_sent = self.pan

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
            if len(parts) == 4:
                try:
                    a, b = float(parts[2]), float(parts[3])
                except ValueError:
                    self._bad(line)
                    return
                if not (math.isfinite(a) and math.isfinite(b)):
                    self._bad(line)
                    return
                # What the ESP32 is really outputting (stick or Pi, after kill switch, ramps and limits).
                self.outputs_pub.publish(Vector3(x=a, y=b, z=1.0 if parts[1] == '1' else 0.0))
                if self.drive_mode == 'diff':     # own speed for the follower's side shot, same scale as C lines
                    self.speed_pub.publish(Float32(data=wheels_to_speed(a, b, self.diff)))
        elif parts[0] == 'G' and len(parts) == 3:
            # gyro yaw rate (rad/s, + = left), printed by firmware >= 2.3.0 when the yaw loop is enabled;
            # published only while the firmware calls it usable (bias measured, loop not tripped)
            try:
                yaw = float(parts[1])
            except ValueError:
                self._bad(line)
                return
            if parts[2] == '1' and math.isfinite(yaw):
                self.yaw_pub.publish(Float32(data=yaw))
        elif parts[0] == 'K' and len(parts) == 4:
            self._publish_remote(parts[1] == '1', parts[2], parts[3])
        elif parts[0] == 'V' and len(parts) == 2:
            self.get_logger().info(f'ESP32 firmware version {parts[1]}')
        else:
            self._bad(line)

    REMOTE_NAMES = {'0': 'never', '1': 'lost', '2': 'stop', '3': 'run'}

    def _publish_remote(self, required, remote, source):
        name = self.REMOTE_NAMES.get(remote)
        if name is None:
            self._bad(f'K,{remote},{source}')
            return
        manual = source == '2'
        self.remote_pub.publish(String(data=name))
        self.manual_pub.publish(Bool(data=manual))
        if not required and not self._warned_no_killswitch:
            self.get_logger().warn('Firmware built with REMOTE_REQUIRED = false: the robot can move WITHOUT the '
                                   'handheld kill switch. Bench use only (wheels off the ground).')
            self._warned_no_killswitch = True
        if name != self.last_remote:
            self.get_logger().info(f'Remote (kill switch): {name}'
                                   + (' - robot will not move' if name != 'run' and required else ''))
            self.last_remote = name
        if manual != self.last_manual:
            if self.last_manual is not None or manual:
                self.get_logger().info('Manual drive from the remote' if manual else 'Manual drive ended')
            self.last_manual = manual

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
        # REP 117: +inf means "no object detected within range" (firmware sends -1.0);
        # NaN or any other negative value is serial corruption -> publish NaN so safety_gate trips RANGE_INVALID.
        if math.isfinite(dist) and dist >= 0.0:
            msg.range = dist
        elif dist == -1.0:
            msg.range = math.inf
        else:
            msg.range = math.nan
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
