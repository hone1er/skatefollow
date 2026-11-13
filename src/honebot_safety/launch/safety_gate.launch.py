"""Start the safety gate on its own.

    ros2 launch honebot_safety safety_gate.launch.py
    ros2 launch honebot_safety safety_gate.launch.py stop_distance:=0.6
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('stop_distance', default_value='0.4',
                              description='No forward motion closer than this (m)'),
        DeclareLaunchArgument('slow_distance', default_value='1.0',
                              description='Forward speed tapers below this (m)'),
        Node(
            package='honebot_safety',
            executable='safety_gate',
            name='safety_gate',
            output='screen',
            parameters=[{
                'stop_distance': ParameterValue(LaunchConfiguration('stop_distance'), value_type=float),
                'slow_distance': ParameterValue(LaunchConfiguration('slow_distance'), value_type=float),
            }],
        ),
    ])
