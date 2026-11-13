"""Record a rosbag of everything for later tuning/debugging.

    ros2 launch honebot_bringup record.launch.py                 # -> ~/bags/honebot_<timestamp>
    ros2 launch honebot_bringup record.launch.py out:=~/bags/test1

Stop with Ctrl-C; the bag is finalised on exit. Inspect with:
    ros2 bag info ~/bags/<name>
    ros2 bag play ~/bags/<name>          (then open Foxglove, or ros2 topic echo)
"""

import os
from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    default_out = os.path.expanduser(
        f'~/bags/honebot_{datetime.now().strftime("%Y%m%d_%H%M%S")}')
    return LaunchDescription([
        DeclareLaunchArgument('out', default_value=default_out,
                              description='Bag directory to create'),
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-a', '-o', LaunchConfiguration('out')],
            output='screen'),
    ])
