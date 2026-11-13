"""Safety gate node.

Topics (relative names; remap as needed)
  sub  cmd_vel_raw       geometry_msgs/Twist   what the controller *wants*
  sub  ultrasonic/range  sensor_msgs/Range     forward obstacle distance
  sub  esp32/link_ok     std_msgs/Bool         ESP32 hears the Pi's heartbeat
  pub  cmd_vel           geometry_msgs/Twist   what the motors are *allowed* to do
  pub  safety/state      std_msgs/String       OK / SLOW / OBSTACLE / CMD_STALE / RANGE_STALE / LINK_DOWN ...

Publishes cmd_vel at a fixed rate regardless of input, so downstream always
sees a fresh command (zero when anything is wrong). Logic lives in gate.py.
"""

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, String

from honebot_safety.gate import OK, SLOW, STOP_STATES, GateParams, gate_command


class SafetyGate(Node):
    def __init__(self):
        super().__init__('safety_gate')
        rate_hz = self.declare_parameter('rate_hz', 50.0).value
        self.params = GateParams(
            stop_distance=self.declare_parameter('stop_distance', 0.4).value,
            slow_distance=self.declare_parameter('slow_distance', 1.0).value,
            range_timeout=self.declare_parameter('range_timeout', 0.3).value,
            cmd_timeout=self.declare_parameter('cmd_timeout', 0.5).value,
        )
        self.add_on_set_parameters_callback(self._on_params)

        self.cmd = None
        self.cmd_stamp = None
        self.range_m = None
        self.range_stamp = None
        self.link_ok = False
        self.link_stamp = None
        self.state = None

        self.create_subscription(Twist, 'cmd_vel_raw', self._on_cmd, 10)
        self.create_subscription(Range, 'ultrasonic/range', self._on_range, 10)
        self.create_subscription(Bool, 'esp32/link_ok', self._on_link, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.state_pub = self.create_publisher(String, 'safety/state', 10)

        self.create_timer(1.0 / rate_hz, self._tick)
        self.get_logger().info(
            f'Safety gate: stop < {self.params.stop_distance:.2f} m, '
            f'slow < {self.params.slow_distance:.2f} m, '
            f'range timeout {self.params.range_timeout:.2f} s, cmd timeout {self.params.cmd_timeout:.2f} s')

    def _on_params(self, params):
        for p in params:
            if hasattr(self.params, p.name):
                setattr(self.params, p.name, float(p.value))
                self.get_logger().info(f'{p.name} = {p.value}')
        return SetParametersResult(successful=True)

    # --- inputs ----------------------------------------------------------
    def _on_cmd(self, msg: Twist):
        self.cmd = msg
        self.cmd_stamp = self.get_clock().now()

    def _on_range(self, msg: Range):
        self.range_m = msg.range
        self.range_stamp = self.get_clock().now()

    def _on_link(self, msg: Bool):
        self.link_ok = msg.data
        self.link_stamp = self.get_clock().now()

    def _age(self, stamp):
        if stamp is None:
            return None
        return (self.get_clock().now() - stamp).nanoseconds * 1e-9

    # --- output ----------------------------------------------------------
    def _tick(self):
        want_lin = self.cmd.linear.x if self.cmd is not None else 0.0
        want_ang = self.cmd.angular.z if self.cmd is not None else 0.0

        lin, ang, state = gate_command(
            want_lin, want_ang,
            self.range_m, self._age(self.range_stamp),
            self.link_ok, self._age(self.link_stamp),
            self._age(self.cmd_stamp),
            self.params)

        out = Twist()
        out.linear.x = float(lin)
        out.angular.z = float(ang)
        self.cmd_pub.publish(out)
        self.state_pub.publish(String(data=state))

        if state != self.state:
            self._log_transition(state)
            self.state = state

    def _log_transition(self, state):
        r = f'{self.range_m:.2f} m' if self.range_m is not None else 'no range'
        if state in STOP_STATES:
            self.get_logger().warn(f'STOP ({state}); range {r}')
        elif state == SLOW:
            self.get_logger().info(f'SLOW: obstacle at {r}')
        elif state == OK:
            self.get_logger().info(f'OK: passing commands through; range {r}')
        else:
            self.get_logger().warn(f'{state}: obstacle at {r}; forward motion blocked')

    def stop(self):
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = SafetyGate()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.stop()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
