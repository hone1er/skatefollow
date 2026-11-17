"""Start turtlesim plus the follower.

    ros2 launch honebot_follow turtle_follow.launch.py
    ros2 launch honebot_follow turtle_follow.launch.py follow_distance:=2.5

Drive the leader from a second terminal:
    ros2 run turtlesim turtle_teleop_key
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    follow_distance = LaunchConfiguration('follow_distance')
    max_linear = LaunchConfiguration('max_linear')

    return LaunchDescription([
        DeclareLaunchArgument('follow_distance', default_value='1.5',
                              description='Metres to stay behind the leader'),
        DeclareLaunchArgument('max_linear', default_value='2.0',
                              description='Follower top speed (m/s)'),

        Node(package='turtlesim', executable='turtlesim_node', name='sim'),
        Node(
            package='honebot_follow',
            executable='follower',
            name='follower',
            output='screen',
            # value_type=float so "follow_distance:=2" isn't rejected as an int
            parameters=[{
                'follow_distance': ParameterValue(follow_distance, value_type=float),
                'max_linear': ParameterValue(max_linear, value_type=float),
            }],
        ),
    ])
