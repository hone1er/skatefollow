"""The real robot's always-on safety layer: ESP32 bridge + safety gate (+ Foxglove), with config/robot.yaml.

    ros2 launch honebot_bringup robot.launch.py
    ros2 launch honebot_bringup robot.launch.py foxglove:=false port:=/dev/ttyUSB0

This is what honebot.service starts at boot. It produces no motion by itself. To follow and film, start
film.launch.py by hand once this is up.  (bringup.launch.py is the bench version with default parameters.)

Foxglove (Mac): File > Open connection > ws://honebot.local:8765
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([FindPackageShare('honebot_bringup'), 'config', 'robot.yaml'])
    default_port = '/dev/esp32' if os.path.exists('/dev/esp32') else '/dev/ttyUSB0'
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value=default_port,
                              description='ESP32 serial device (/dev/esp32 once the udev rule is installed)'),
        DeclareLaunchArgument('foxglove', default_value='true', description='Start foxglove_bridge on port 8765'),
        Node(package='honebot_hardware', executable='esp32_bridge', name='esp32_bridge', output='screen',
             parameters=[config, {'port': LaunchConfiguration('port')}]),
        Node(package='honebot_safety', executable='safety_gate', name='safety_gate', output='screen',
             parameters=[config]),
        Node(package='foxglove_bridge', executable='foxglove_bridge', name='foxglove_bridge', output='screen',
             parameters=[{'port': 8765, 'address': '0.0.0.0'}],
             condition=IfCondition(LaunchConfiguration('foxglove'))),
    ])
