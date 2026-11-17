"""Start the ESP32 bridge.

    ros2 launch honebot_hardware esp32.launch.py
    ros2 launch honebot_hardware esp32.launch.py port:=/dev/ttyACM0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0',
                              description='ESP32 serial device'),
        Node(
            package='honebot_hardware',
            executable='esp32_bridge',
            name='esp32_bridge',
            output='screen',
            parameters=[{'port': LaunchConfiguration('port')}],
        ),
    ])
