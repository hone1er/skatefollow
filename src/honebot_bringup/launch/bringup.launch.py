"""Robot bringup: ESP32 bridge + safety gate (+ Foxglove bridge for remote monitoring).

    ros2 launch honebot_bringup bringup.launch.py
    ros2 launch honebot_bringup bringup.launch.py foxglove:=false
    ros2 launch honebot_bringup bringup.launch.py port:=/dev/ttyACM0 stop_distance:=0.6

Data flow:  <controller> -> /cmd_vel_raw -> safety_gate -> /cmd_vel -> esp32_bridge -> ESP32
The controller (follower, gamepad teleop, ...) is started separately so the
safety layer is always up before anything that can produce motion.

Foxglove (Mac): File > Open connection > ws://honebot.local:8765
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0', description='ESP32 serial device'),
        DeclareLaunchArgument('stop_distance', default_value='0.4',
                              description='Gate: no forward motion closer than this (m)'),
        DeclareLaunchArgument('slow_distance', default_value='1.0',
                              description='Gate: forward speed tapers below this (m)'),
        DeclareLaunchArgument('foxglove', default_value='true',
                              description='Start foxglove_bridge on port 8765'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('honebot_hardware'), 'launch', 'esp32.launch.py'])),
            launch_arguments={'port': LaunchConfiguration('port')}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('honebot_safety'), 'launch', 'safety_gate.launch.py'])),
            launch_arguments={
                'stop_distance': LaunchConfiguration('stop_distance'),
                'slow_distance': LaunchConfiguration('slow_distance'),
            }.items(),
        ),
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
            parameters=[{'port': 8765, 'address': '0.0.0.0'}],
            condition=IfCondition(LaunchConfiguration('foxglove')),
        ),
    ])
