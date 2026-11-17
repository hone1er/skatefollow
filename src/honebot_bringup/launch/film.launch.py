"""Follow the skater and film: UWB -> follower (-> safety gate), camera + pan aim, and a log of the run.

Start it by hand, after robot.launch.py (or honebot.service) is running:

    ros2 launch honebot_bringup film.launch.py
    ros2 launch honebot_bringup film.launch.py max_linear:=2.0 follow_bearing:=0.45
    ros2 launch honebot_bringup film.launch.py camera:=false bag:=false        # follow only

The robot moves only while the handheld remote is in RUN. Ctrl-C here stops the follower; the safety gate
then sees no commands and stops the robot.

Arguments
  max_linear      top follow speed, m/s (default: config/robot.yaml, 3.0; the gate's sight limit also applies)
  follow_bearing  where to keep the skater, rad, + = left of the nose (default: config/robot.yaml)
  uwb_port        UWB board serial device
  camera          start the GoPro/webcam detector and the pan servo aim (default true)
  camera_source   'gopro', '/dev/videoN', URL or file
  record          camera: save the GoPro webcam stream to ~/videos (default false)
  bag             record every topic to ~/bags for tuning afterwards (default true)
"""

import os
from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _movers(context):
    """uwb_node + follower. A launch argument overrides config/robot.yaml only when it was actually given."""
    config = os.path.join(FindPackageShare('honebot_bringup').perform(context), 'config', 'robot.yaml')
    overrides = {}
    for name in ('max_linear', 'follow_bearing'):
        value = context.launch_configurations.get(name, '')
        if value != '':
            overrides[name] = float(value)
    return [
        Node(package='honebot_uwb', executable='uwb_node', name='uwb_node', output='screen',
             parameters=[config, {'port': context.launch_configurations['uwb_port']}]),
        Node(package='honebot_follow', executable='follower', name='follower', output='screen',
             parameters=[config, overrides] if overrides else [config]),
    ]


def generate_launch_description():
    bag_dir = os.path.expanduser(f'~/bags/honebot_{datetime.now().strftime("%Y%m%d_%H%M%S")}')
    return LaunchDescription([
        DeclareLaunchArgument('max_linear', default_value='',
                              description='Top follow speed (m/s); empty = value in config/robot.yaml'),
        DeclareLaunchArgument('follow_bearing', default_value='',
                              description='Keep the skater at this bearing (rad, + = left); empty = config value'),
        DeclareLaunchArgument('uwb_port', default_value='/dev/ttyUSB1', description='UWB board serial device'),
        DeclareLaunchArgument('camera', default_value='true', description='Start the camera detector and pan aim'),
        DeclareLaunchArgument('camera_source', default_value='gopro'),
        DeclareLaunchArgument('record', default_value='false', description='Save the GoPro webcam stream'),
        DeclareLaunchArgument('bag', default_value='true', description='Record all topics to ~/bags'),

        OpaqueFunction(function=_movers),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('honebot_camera'), 'launch', 'camera.launch.py'])),
            launch_arguments={'source': LaunchConfiguration('camera_source'),
                              'record': LaunchConfiguration('record')}.items(),
            condition=IfCondition(LaunchConfiguration('camera'))),
        ExecuteProcess(cmd=['ros2', 'bag', 'record', '-a', '-o', bag_dir], output='screen',
                       condition=IfCondition(LaunchConfiguration('bag'))),
    ])
