"""Follower for the real robot: UWB target -> follower (config/robot.yaml) -> /cmd_vel_raw -> safety gate.

Start bringup first (bridge + gate), and uwb_node so /target exists:

    ros2 launch honebot_bringup bringup.launch.py
    ros2 launch honebot_uwb uwb.launch.py
    ros2 launch honebot_follow robot_follow.launch.py
    ros2 launch honebot_follow robot_follow.launch.py max_linear:=5.0

The robot only moves if the handheld remote is in RUN (firmware >= 2.2.0).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([FindPackageShare('honebot_follow'), 'config', 'robot.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('max_linear', default_value='3.0',
                              description='Top follow speed in m/s (start low; 6.7 = 15 mph)'),
        Node(
            package='honebot_follow', executable='follower', name='follower', output='screen',
            parameters=[config, {'max_linear': ParameterValue(LaunchConfiguration('max_linear'), value_type=float)}],
        ),
    ])
