"""Turtlesim follow-me node.

Spawns a second turtle and drives it to follow the first one, which you steer
with turtle_teleop_key. Later the leader pose will come from UWB and the
output /cmd_vel will go to the ESP32 motor controller, but the control logic
(control.py) stays the same.

Topics
  sub  /<leader>/pose     turtlesim/Pose
  sub  /<follower>/pose   turtlesim/Pose
  pub  /<follower>/cmd_vel geometry_msgs/Twist
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from turtlesim.msg import Pose
from turtlesim.srv import Spawn

from honebot_follow.control import FollowGains, follow_command


class Follower(Node):
    def __init__(self):
        super().__init__('follower')

        self.leader = self.declare_parameter('leader', 'turtle1').value
        self.follower = self.declare_parameter('follower', 'turtle2').value
        self.spawn_follower = self.declare_parameter('spawn_follower', True).value
        rate_hz = self.declare_parameter('rate_hz', 20.0).value
        # Safety habit for the real robot: stop if the leader's position is stale.
        self.leader_timeout = self.declare_parameter('leader_timeout', 0.5).value

        self.gains = FollowGains(
            follow_distance=self.declare_parameter('follow_distance', 1.5).value,
            kp_linear=self.declare_parameter('kp_linear', 1.0).value,
            kp_angular=self.declare_parameter('kp_angular', 4.0).value,
            max_linear=self.declare_parameter('max_linear', 2.0).value,
            max_angular=self.declare_parameter('max_angular', 4.0).value,
        )
        self.add_on_set_parameters_callback(self._on_params)

        self.leader_pose = None
        self.leader_stamp = None
        self.follower_pose = None
        self._warned_stale = False

        self.create_subscription(Pose, f'/{self.leader}/pose', self._on_leader, 10)
        self.create_subscription(Pose, f'/{self.follower}/pose', self._on_follower, 10)
        self.cmd_pub = self.create_publisher(Twist, f'/{self.follower}/cmd_vel', 10)

        if self.spawn_follower:
            self._spawn()

        self.create_timer(1.0 / rate_hz, self._tick)
        self.get_logger().info(
            f'Following /{self.leader} with /{self.follower} '
            f'at {self.gains.follow_distance:.1f} m')

    # --- setup -----------------------------------------------------------
    def _spawn(self):
        client = self.create_client(Spawn, '/spawn')
        if not client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('turtlesim /spawn service not available; is turtlesim running?')
            return
        req = Spawn.Request(x=2.0, y=2.0, theta=0.0, name=self.follower)
        future = client.call_async(req)
        future.add_done_callback(self._on_spawned)

    def _on_spawned(self, future):
        try:
            future.result()
            self.get_logger().info(f'Spawned {self.follower}')
        except Exception as e:  # already exists after a restart, etc.
            self.get_logger().warn(f'Could not spawn {self.follower} ({e}); using existing turtle')

    def _on_params(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if hasattr(self.gains, p.name):
                setattr(self.gains, p.name, float(p.value))
                self.get_logger().info(f'{p.name} = {p.value}')
        return SetParametersResult(successful=True)

    # --- callbacks -------------------------------------------------------
    def _on_leader(self, msg: Pose):
        self.leader_pose = msg
        self.leader_stamp = self.get_clock().now()

    def _on_follower(self, msg: Pose):
        self.follower_pose = msg

    def _tick(self):
        cmd = Twist()
        if self.leader_pose is None or self.follower_pose is None:
            self.cmd_pub.publish(cmd)
            return

        age = (self.get_clock().now() - self.leader_stamp).nanoseconds * 1e-9
        if age > self.leader_timeout:
            if not self._warned_stale:
                self.get_logger().warn(f'Leader pose is {age:.1f}s old; stopping')
                self._warned_stale = True
            self.cmd_pub.publish(cmd)
            return
        self._warned_stale = False

        f, l = self.follower_pose, self.leader_pose
        linear, angular, _ = follow_command(f.x, f.y, f.theta, l.x, l.y, self.gains)
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.cmd_pub.publish(cmd)

    def stop(self):
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = Follower()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # Ctrl-C or SIGTERM: exit cleanly
    finally:
        if rclpy.ok():
            node.stop()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
