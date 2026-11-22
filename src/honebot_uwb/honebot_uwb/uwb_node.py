"""UWB node: serial ranges from the anchor board -> relative target.

Serial protocol from the UWB board (115200 8N1, ASCII lines), two forms:
    U,<r_left_m>,<r_right_m>        two-anchor ranging: range in metres from each
                                    anchor to the tag, -1 when no fix this cycle
    A,<range_m>,<bearing_deg>       angle-of-arrival anchor (e.g. Makerfabs MaUWB_AOA):
                                    range and bearing (+ve = left) in one line
    anything else                   ignored (boot messages, debug)
Whatever firmware ends up on the UWB board only has to print one of these.

Publishes
    target        geometry_msgs/PointStamped   tag position in base_link (x fwd, y left)
    uwb/ranges    std_msgs/Float32MultiArray   [r_left, r_right] after filtering (debug)

No fix -> nothing published; the follower's target_timeout then stops the robot.
The follower consumes `target` exactly as it does from pose_to_target in turtlesim.
"""

import math

import rclpy
import serial
from geometry_msgs.msg import PointStamped
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from honebot_uwb.localize import RangeFilter, UwbGeometry, aoa_position, two_anchor_position


class UwbNode(Node):
    def __init__(self):
        super().__init__('uwb_node')
        self.port = self.declare_parameter('port', '/dev/ttyUSB1').value
        self.baud = self.declare_parameter('baud', 115200).value
        self.frame_id = self.declare_parameter('frame_id', 'base_link').value
        self.geom = UwbGeometry(
            baseline=self.declare_parameter('baseline', 0.30).value,
            anchor_x=self.declare_parameter('anchor_x', 0.0).value,
        )
        self.max_range = self.declare_parameter('max_range', 30.0).value
        alpha = self.declare_parameter('filter_alpha', 0.5).value
        self.filters = [RangeFilter(alpha=alpha, max_range=self.max_range) for _ in range(2)]
        self.aoa_filter = RangeFilter(alpha=alpha, max_range=self.max_range)
        self.bearing_sign = self.declare_parameter('bearing_sign', 1.0).value   # -1 if the board reports +ve = right
        # Angle-of-arrival anchors are only accurate within a cone in front of the antenna; readings outside
        # it are dropped (the follower then stops on its target timeout rather than steering on a bad angle).
        self.max_bearing_deg = self.declare_parameter('max_bearing_deg', 80.0).value
        self.bearing_filter = RangeFilter(alpha=self.declare_parameter('bearing_alpha', 0.6).value,
                                          min_range=-self.max_bearing_deg, max_range=self.max_bearing_deg)
        self.add_on_set_parameters_callback(self._on_params)

        self.target_pub = self.create_publisher(PointStamped, 'target', 10)
        self.ranges_pub = self.create_publisher(Float32MultiArray, 'uwb/ranges', 10)

        self.ser = None
        self.buf = b''
        self.n_fix = 0
        self.n_reject = 0
        self.create_timer(0.01, self._poll)
        self.create_timer(1.0, self._ensure_open)
        self.create_timer(5.0, self._stats)
        self._ensure_open()

    def _on_params(self, params):
        updates = []
        for prm in params:
            if prm.name in ('baseline', 'max_bearing_deg'):
                val = float(prm.value)
                if not math.isfinite(val) or val <= 0.0:
                    return SetParametersResult(successful=False, reason=f'{prm.name} must be finite and > 0')
                updates.append((prm.name, val))
            elif prm.name == 'max_range':
                val = float(prm.value)
                if not math.isfinite(val) or val <= self.aoa_filter.min_range:
                    return SetParametersResult(successful=False,
                                               reason=f'max_range must be finite and > {self.aoa_filter.min_range}')
                updates.append((prm.name, val))
            elif prm.name in ('filter_alpha', 'bearing_alpha'):
                val = float(prm.value)
                if not math.isfinite(val) or not (0.0 < val <= 1.0):
                    return SetParametersResult(successful=False, reason=f'{prm.name} must be in (0, 1]')
                updates.append((prm.name, val))
            elif prm.name == 'anchor_x':
                val = float(prm.value)
                if not math.isfinite(val):
                    return SetParametersResult(successful=False, reason='anchor_x must be finite')
                updates.append((prm.name, val))
            elif prm.name == 'bearing_sign':
                val = float(prm.value)
                if not math.isfinite(val) or val == 0.0:
                    return SetParametersResult(successful=False, reason='bearing_sign must be finite and non-zero')
                updates.append((prm.name, val))
        for name, val in updates:
            if name == 'baseline':
                self.geom.baseline = val
            elif name == 'anchor_x':
                self.geom.anchor_x = val
            elif name == 'max_range':
                self.max_range = val
                for f in self.filters:
                    f.max_range = val
                self.aoa_filter.max_range = val
            elif name == 'filter_alpha':
                for f in self.filters:
                    f.alpha = val
                self.aoa_filter.alpha = val
            elif name == 'bearing_alpha':
                self.bearing_filter.alpha = val
            elif name == 'max_bearing_deg':
                self.max_bearing_deg = val
                self.bearing_filter.min_range = -val
                self.bearing_filter.max_range = val
            elif name == 'bearing_sign':
                self.bearing_sign = val
            self.get_logger().info(f'{name} = {val}')
        return SetParametersResult(successful=True)

    def _ensure_open(self):
        if self.ser is not None:
            return
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0)
            self.buf = b''
            self.get_logger().info(f'Connected to UWB board on {self.port}')
        except serial.SerialException as e:
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
            self._handle(line.strip().decode('ascii', errors='replace'))
        if len(self.buf) > 256:
            self.buf = b''

    def _handle(self, line):
        parts = line.split(',')
        if len(parts) != 3 or parts[0] not in ('U', 'A'):
            return
        try:
            raw = [float(parts[1]), float(parts[2])]
        except ValueError:
            return
        if parts[0] == 'A':
            self._handle_aoa(raw[0], raw[1])
            return
        filtered = [f.update(r) for f, r in zip(self.filters, raw)]
        if any(v is None for v in filtered):         # no fix on one anchor this cycle: publish nothing
            self.n_reject += 1
            return
        self.ranges_pub.publish(Float32MultiArray(data=[float(v) for v in filtered]))
        pos = two_anchor_position(filtered[0], filtered[1], self.geom)
        if pos is None:
            self.n_reject += 1
            return
        self._publish_target(pos)

    def _handle_aoa(self, rng, bearing_deg):
        if not (math.isfinite(rng) and self.aoa_filter.min_range <= rng <= self.aoa_filter.max_range
                and math.isfinite(bearing_deg) and abs(bearing_deg) <= self.max_bearing_deg):
            self.aoa_filter.update(math.nan)
            self.bearing_filter.update(math.nan)
            self.n_reject += 1
            return
        rng = self.aoa_filter.update(rng)
        bearing_deg = self.bearing_filter.update(bearing_deg)
        if rng is None or bearing_deg is None:       # no fix this cycle: publish nothing
            self.n_reject += 1
            return
        pos = aoa_position(rng, math.radians(bearing_deg) * self.bearing_sign, self.geom, self.max_range)
        if pos is None:
            self.n_reject += 1
            return
        self.ranges_pub.publish(Float32MultiArray(data=[float(rng), float(bearing_deg)]))
        self._publish_target(pos)

    def _publish_target(self, pos):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.point.x, msg.point.y = pos
        self.target_pub.publish(msg)
        self.n_fix += 1

    def _stats(self):
        if self.n_fix or self.n_reject:
            self.get_logger().info(f'UWB: {self.n_fix / 5.0:.1f} fixes/s, {self.n_reject} dropped in 5 s')
            self.n_fix = self.n_reject = 0


def main(args=None):
    rclpy.init(args=args)
    node = UwbNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node.ser is not None:
            node.ser.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
