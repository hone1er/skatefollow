"""ROS 2 <-> ESP32 serial bridge.

Reads lines like "R,0.532,1" from the ESP32 and publishes:
  /ultrasonic/range   sensor_msgs/Range   (+inf when nothing is in range)
  /esp32/link_ok      std_msgs/Bool       (ESP32 reports it hears our heartbeat)

Sends "H\n" heartbeats to the ESP32 so its watchdog knows the Pi is alive.
Reconnects automatically if the USB cable is unplugged and plugged back in.
"""

import math

import rclpy
import serial
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Bool


class Esp32Bridge(Node):
    def __init__(self):
        super().__init__('esp32_bridge')
        self.port = self.declare_parameter('port', '/dev/ttyUSB0').value
        self.baud = self.declare_parameter('baud', 115200).value
        self.frame_id = self.declare_parameter('frame_id', 'ultrasonic_link').value
        heartbeat_hz = self.declare_parameter('heartbeat_hz', 10.0).value

        self.range_pub = self.create_publisher(Range, 'ultrasonic/range', 10)
        self.link_pub = self.create_publisher(Bool, 'esp32/link_ok', 10)

        self.ser = None
        self.buf = b''
        self.last_link_ok = None
        self.bad_lines = 0

        self.create_timer(0.01, self._poll)                 # read serial at 100 Hz
        self.create_timer(1.0 / heartbeat_hz, self._heartbeat)
        self.create_timer(1.0, self._ensure_open)
        self._ensure_open()

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


def main(args=None):
    rclpy.init(args=args)
    node = Esp32Bridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # Ctrl-C or SIGTERM (launch/systemd stopping us): exit cleanly
    finally:
        if node.ser is not None:
            node.ser.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
