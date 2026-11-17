"""Follow-me node.

Two input modes, same control law (control.py):

  turtlesim mode (default, target_topic == ''):
    sub  /<leader>/pose, /<follower>/pose   turtlesim/Pose  -> converted to (distance, bearing)
    pub  /<follower>/cmd_vel                geometry_msgs/Twist   (or cmd_vel_topic if set)
    Spawns the follower turtle via /spawn.

  relative-target mode (target_topic set, e.g. '/target'):
    sub  <target_topic>   geometry_msgs/PointStamped  target in the robot frame
                          (x forward, y left, metres) - what UWB will provide
    pub  cmd_vel_topic    geometry_msgs/Twist          (default 'cmd_vel_raw' -> safety gate)

Either way: if the target is older than target_timeout the robot is commanded to
stop. Set ackermann:=true for the car (no turning in place; creeps instead).
"""

import math

import rclpy
from geometry_msgs.msg import PointStamped, Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from turtlesim.msg import Pose
from turtlesim.srv import Spawn

from honebot_follow.control import FollowGains, follow_command_relative, relative_target


class Follower(Node):
    def __init__(self):
        super().__init__('follower')

        self.target_topic = self.declare_parameter('target_topic', '').value
        cmd_vel_topic = self.declare_parameter('cmd_vel_topic', '').value
        self.leader = self.declare_parameter('leader', 'turtle1').value
        self.follower = self.declare_parameter('follower', 'turtle2').value
        # Spawn the follower turtle in turtlesim. Off by default so the real robot
        # never waits for a /spawn service; turtle_follow.launch.py turns it on.
        self.spawn_follower = self.declare_parameter('spawn_follower', False).value
        rate_hz = self.declare_parameter('rate_hz', 20.0).value
        # Safety habit for the real robot: stop if the target is stale.
        # 'leader_timeout' kept as an alias of the old parameter name.
        self.target_timeout = self.declare_parameter('target_timeout', 0.5).value
        self.target_timeout = self.declare_parameter('leader_timeout', self.target_timeout).value

        self.gains = FollowGains(
            follow_distance=self.declare_parameter('follow_distance', 1.5).value,
            kp_linear=self.declare_parameter('kp_linear', 1.0).value,
            kp_angular=self.declare_parameter('kp_angular', 4.0).value,
            max_linear=self.declare_parameter('max_linear', 2.0).value,
            max_angular=self.declare_parameter('max_angular', 4.0).value,
            ackermann=self.declare_parameter('ackermann', False).value,
            creep_speed=self.declare_parameter('creep_speed', 0.3).value,
            brake_decel=self.declare_parameter('brake_decel', 0.0).value,
            time_headway=self.declare_parameter('time_headway', 0.5).value,
            max_lat_accel=self.declare_parameter('max_lat_accel', 0.0).value,
            min_turn_radius=self.declare_parameter('min_turn_radius', 0.0).value,
            arc_speed=self.declare_parameter('arc_speed', 0.6).value,
            pivot_bearing=self.declare_parameter('pivot_bearing', 2.0).value,
            stopped_turn_deadband=self.declare_parameter('stopped_turn_deadband', 0.0).value,
        )
        self.add_on_set_parameters_callback(self._on_params)

        self.target = None          # (distance, bearing)
        self.target_stamp = None
        self.leader_pose = None
        self.follower_pose = None
        self._warned_stale = False

        self.relative_mode = bool(self.target_topic)
        if self.relative_mode:
            self.create_subscription(PointStamped, self.target_topic, self._on_target, 10)
            cmd_topic = cmd_vel_topic or 'cmd_vel_raw'
            source = self.target_topic
        else:
            self.create_subscription(Pose, f'/{self.leader}/pose', self._on_leader, 10)
            self.create_subscription(Pose, f'/{self.follower}/pose', self._on_follower, 10)
            cmd_topic = cmd_vel_topic or f'/{self.follower}/cmd_vel'
            source = f'/{self.leader} (turtlesim)'
        if self.spawn_follower:
            self._spawn()
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)

        self.create_timer(1.0 / rate_hz, self._tick)
        self.get_logger().info(
            f'Following {source} at {self.gains.follow_distance:.1f} m, publishing {cmd_topic}'
            f'{" (ackermann)" if self.gains.ackermann else ""}')

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
        for p in params:
            if hasattr(self.gains, p.name):
                cur = getattr(self.gains, p.name)
                setattr(self.gains, p.name, bool(p.value) if isinstance(cur, bool) else float(p.value))
                self.get_logger().info(f'{p.name} = {p.value}')
        return SetParametersResult(successful=True)

    # --- inputs ----------------------------------------------------------
    def _on_target(self, msg: PointStamped):
        x, y = msg.point.x, msg.point.y
        self.target = (math.hypot(x, y), math.atan2(y, x))
        self.target_stamp = self.get_clock().now()

    def _on_leader(self, msg: Pose):
        self.leader_pose = msg
        if self.follower_pose is not None:
            f = self.follower_pose
            self.target = relative_target(f.x, f.y, f.theta, msg.x, msg.y)
            self.target_stamp = self.get_clock().now()

    def _on_follower(self, msg: Pose):
        self.follower_pose = msg

    # --- control loop ----------------------------------------------------
    def _tick(self):
        cmd = Twist()
        if self.target is None:
            self.cmd_pub.publish(cmd)
            return

        age = (self.get_clock().now() - self.target_stamp).nanoseconds * 1e-9
        if age > self.target_timeout:
            if not self._warned_stale:
                self.get_logger().warn(f'Target is {age:.1f}s old; stopping')
                self._warned_stale = True
            self.cmd_pub.publish(cmd)
            return
        if self._warned_stale:
            self.get_logger().info('Target is back')
            self._warned_stale = False

        distance, bearing = self.target
        linear, angular, _ = follow_command_relative(distance, bearing, self.gains)
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
