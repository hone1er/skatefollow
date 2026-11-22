"""Simulated UWB: turns two turtlesim poses into a relative target.

Publishes where the leader is *as seen from the follower* (x forward, y left,
metres) as geometry_msgs/PointStamped, which is exactly what the future
uwb_node will publish for the real skater. Lets the relative-target follower be
tested in turtlesim before any UWB hardware exists.

  sub  /<leader>/pose, /<follower>/pose   turtlesim/Pose
  pub  target                             geometry_msgs/PointStamped (frame_id = base_link)
"""

import math

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from turtlesim.msg import Pose


class PoseToTarget(Node):
    def __init__(self):
        super().__init__('pose_to_target')
        leader = self.declare_parameter('leader', 'turtle1').value
        follower = self.declare_parameter('follower', 'turtle2').value
        self.frame_id = self.declare_parameter('frame_id', 'base_link').value
        rate_hz = self.declare_parameter('rate_hz', 20.0).value
        self.pose_timeout = self.declare_parameter('pose_timeout', 0.5).value

        self.leader_pose = None
        self.leader_stamp = None
        self.follower_pose = None
        self.follower_stamp = None
        self.create_subscription(Pose, f'/{leader}/pose', self._on_leader, 10)
        self.create_subscription(Pose, f'/{follower}/pose', self._on_follower, 10)
        self.pub = self.create_publisher(PointStamped, 'target', 10)
        self.create_timer(1.0 / rate_hz, self._tick)
        self.get_logger().info(f'Publishing /{leader} relative to /{follower} on target')

    def _on_leader(self, msg):
        self.leader_pose = msg
        self.leader_stamp = self.get_clock().now()

    def _on_follower(self, msg):
        self.follower_pose = msg
        self.follower_stamp = self.get_clock().now()

    def _tick(self):
        if self.leader_pose is None or self.follower_pose is None:
            return
        now = self.get_clock().now()
        if ((now - self.leader_stamp).nanoseconds * 1e-9 > self.pose_timeout
                or (now - self.follower_stamp).nanoseconds * 1e-9 > self.pose_timeout):
            return
        f, l = self.follower_pose, self.leader_pose
        dx, dy = l.x - f.x, l.y - f.y
        c, s = math.cos(f.theta), math.sin(f.theta)
        msg = PointStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.frame_id
        msg.point.x = c * dx + s * dy      # world -> follower frame
        msg.point.y = -s * dx + c * dy
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PoseToTarget()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
