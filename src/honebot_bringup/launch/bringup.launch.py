"""Robot bringup: ESP32 bridge + safety gate (+ Foxglove bridge for remote monitoring).

    ros2 launch honebot_bringup bringup.launch.py
    ros2 launch honebot_bringup bringup.launch.py foxglove:=false
    ros2 launch honebot_bringup bringup.launch.py port:=/dev/ttyACM0 stop_distance:=0.6
    ros2 launch honebot_bringup bringup.launch.py camera:=true [camera_source:=gopro record:=true]

Data flow:  <controller> -> /cmd_vel_raw -> safety_gate -> /cmd_vel -> esp32_bridge -> ESP32
The controller (follower, gamepad teleop, ...) is started separately so the
safety layer is always up before anything that can produce motion.

camera:=true adds camera_node + camera_aim (honebot_camera): GoPro video in, person detection,
and the pan-servo command the bridge forwards to the ESP32 (P lines, GPIO 27).

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
        DeclareLaunchArgument('camera', default_value='false',
                              description='Start camera_node + camera_aim (GoPro / webcam, pan servo)'),
        DeclareLaunchArgument('camera_source', default_value='gopro',
                              description="camera source: 'gopro', '/dev/videoN', URL or file"),
        DeclareLaunchArgument('record', default_value='false',
                              description='camera: tee the raw GoPro stream to ~/videos'),
        DeclareLaunchArgument('model_path', default_value='~/models/yolov8n.onnx',
                              description="camera: YOLO .onnx file (HOG if missing); '' = HOG detector"),

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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('honebot_camera'), 'launch', 'camera.launch.py'])),
            launch_arguments={
                'source': LaunchConfiguration('camera_source'),
                'record': LaunchConfiguration('record'),
                'model_path': LaunchConfiguration('model_path'),
            }.items(),
            condition=IfCondition(LaunchConfiguration('camera')),
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
