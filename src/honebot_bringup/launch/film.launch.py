"""Follow the skater and film: UWB -> follower (-> safety gate), camera + pan aim, and a log of the run.

Start it by hand, after robot.launch.py (or honebot.service) is running:

    ros2 launch honebot_bringup film.launch.py
    ros2 launch honebot_bringup film.launch.py max_linear:=2.0 follow_bearing:=0.45
    ros2 launch honebot_bringup film.launch.py camera:=false bag:=false        # follow only
    ros2 launch honebot_bringup film.launch.py side_shot:=true                 # side view, config/side_shot.yaml

The robot moves only while the handheld remote is in RUN. Ctrl-C here stops the follower; the safety gate
then sees no commands and stops the robot.

Arguments
  max_linear      top follow speed, m/s (default: config/robot.yaml, 3.0; the gate's sight limit also applies)
  follow_bearing  where to keep the skater, rad, + = left of the nose (default: config/robot.yaml)
  side_shot       ride 2 m from the skater, 45 deg behind on their right (config/side_shot.yaml; default false)
  uwb_port        UWB board serial device
  camera          start the GoPro/webcam detector and the pan servo aim (default true)
  camera_source   'gopro', '/dev/videoN', URL or file
  record          camera: save the GoPro webcam stream to ~/videos (default false)
  pan_sign        camera: -1.0 if the pan servo turns the wrong way
  bag             record every topic to ~/bags for tuning afterwards (default true)
"""

import os
from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _nodes(context):
    """uwb_node + follower + optional camera + optional bag recording.
    Launch arguments override config/robot.yaml only when explicitly given."""
    lc = context.launch_configurations
    share = FindPackageShare('honebot_bringup').perform(context)
    config = os.path.join(share, 'config', 'robot.yaml')
    side_shot = lc.get('side_shot', 'false').lower() == 'true'

    uwb_params = [config]
    if lc.get('uwb_port', '') != '':
        uwb_params.append({'port': lc['uwb_port']})

    follower_params = [config]
    if side_shot:
        follower_params.append(os.path.join(share, 'config', 'side_shot.yaml'))
    overrides = {}
    for name in ('max_linear', 'follow_bearing'):
        value = lc.get(name, '')
        if value != '':
            overrides[name] = float(value)

    actions = [
        Node(package='honebot_uwb', executable='uwb_node', name='uwb_node', output='screen',
             parameters=uwb_params),
        Node(package='honebot_follow', executable='follower', name='follower', output='screen',
             parameters=follower_params + ([overrides] if overrides else [])),
    ]

    if lc.get('camera', 'true').lower() == 'true':
        cam_share = FindPackageShare('honebot_camera').perform(context)
        gopro_fov = lc.get('gopro_fov', '') or ('0' if side_shot else '4')
        hfov_deg = lc.get('hfov_deg', '') or ('118.0' if side_shot else '87.0')
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(cam_share, 'launch', 'camera.launch.py')),
            launch_arguments={
                'source': lc['camera_source'],
                'record': lc['record'],
                'pan_sign': lc['pan_sign'],
                'gopro_fov': gopro_fov,
                'hfov_deg': hfov_deg,
            }.items()))

    if lc.get('bag', 'true').lower() == 'true':
        bag_dir = os.path.expanduser(f'~/bags/honebot_{datetime.now().strftime("%Y%m%d_%H%M%S")}')
        os.makedirs(os.path.dirname(bag_dir), exist_ok=True)
        actions.append(ExecuteProcess(cmd=['ros2', 'bag', 'record', '-a', '-o', bag_dir], output='screen'))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('max_linear', default_value='',
                              description='Top follow speed (m/s); empty = value in config/robot.yaml'),
        DeclareLaunchArgument('follow_bearing', default_value='',
                              description='Keep the skater at this bearing (rad, + = left); empty = config value'),
        DeclareLaunchArgument('side_shot', default_value='false',
                              description='Side view: 2 m, 45 deg behind on the right (config/side_shot.yaml)'),
        DeclareLaunchArgument('uwb_port', default_value='',
                              description='UWB board serial device; empty = value in config/robot.yaml'),
        DeclareLaunchArgument('camera', default_value='true', description='Start the camera detector and pan aim'),
        DeclareLaunchArgument('camera_source', default_value='gopro'),
        DeclareLaunchArgument('gopro_fov', default_value='',
                              description='GoPro FOV (0 wide, 4 linear); empty = 0 for side_shot, else 4'),
        DeclareLaunchArgument('hfov_deg', default_value='',
                              description='Camera horizontal FOV deg; empty = 118.0 for side_shot, else 87.0'),
        DeclareLaunchArgument('record', default_value='false', description='Save the GoPro webcam stream'),
        DeclareLaunchArgument('pan_sign', default_value='1.0', description='-1.0 if the camera pans the wrong way'),
        DeclareLaunchArgument('bag', default_value='true', description='Record all topics to ~/bags'),

        OpaqueFunction(function=_nodes),
    ])
