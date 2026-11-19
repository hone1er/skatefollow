"""config/robot.yaml must only use parameters the nodes really declare.

ROS silently ignores a YAML key that no node declares, so a typo here would look like a setting that
"does nothing". This test reads each node's source for its declare_parameter names and compares.
"""
import math
import os
import re

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.abspath(os.path.join(HERE, '..', '..'))
CONFIG = os.path.join(HERE, '..', 'config', 'robot.yaml')
SIDE_SHOT = os.path.join(HERE, '..', 'config', 'side_shot.yaml')
CAMERA_AIM = 'honebot_camera/honebot_camera/camera_aim.py'

NODE_SOURCES = {
    'esp32_bridge': 'honebot_hardware/honebot_hardware/esp32_bridge.py',
    'safety_gate': 'honebot_safety/honebot_safety/safety_gate.py',
    'uwb_node': 'honebot_uwb/honebot_uwb/uwb_node.py',
    'follower': 'honebot_follow/honebot_follow/follower.py',
}


def declared(path):
    text = open(os.path.join(SRC, path)).read()
    return set(re.findall(r"declare_parameter\(\s*'(\w+)'", text))


def load():
    with open(CONFIG) as f:
        return yaml.safe_load(f)


def test_every_node_in_the_config_is_known():
    assert set(load()) == set(NODE_SOURCES)


def test_every_key_is_a_declared_parameter():
    cfg = load()
    for node, path in NODE_SOURCES.items():
        keys = set(cfg[node]['ros__parameters'])
        unknown = keys - declared(path)
        assert not unknown, f'{node}: not parameters of the node: {sorted(unknown)}'


def test_follower_and_gate_agree_on_braking():
    cfg = load()
    assert cfg['follower']['ros__parameters']['brake_decel'] == cfg['safety_gate']['ros__parameters']['brake_decel']


def pan_half_range():
    """camera_aim's half_range_deg default (rad): the angle the pan servo reaches at pan +-1."""
    text = open(os.path.join(SRC, CAMERA_AIM)).read()
    return math.radians(float(re.search(r"p\('half_range_deg', ([\d.]+)\)", text).group(1)))


def load_side_shot():
    with open(SIDE_SHOT) as f:
        return yaml.safe_load(f)


def test_skater_stays_inside_the_camera_pan_range_when_stopped():
    for cfg in (load(), {'follower': {'ros__parameters': dict(load()['follower']['ros__parameters'],
                                                                **load_side_shot()['follower']['ros__parameters'])}}):
        f = cfg['follower']['ros__parameters']
        assert abs(f['follow_bearing']) + f['stopped_turn_deadband'] <= pan_half_range()


def test_side_shot_preset_only_sets_follower_parameters():
    side = load_side_shot()
    assert set(side) == {'follower'}
    unknown = set(side['follower']['ros__parameters']) - declared(NODE_SOURCES['follower'])
    assert not unknown, f'side_shot.yaml: not parameters of the follower: {sorted(unknown)}'


def test_side_shot_camera_can_reach_the_skater():
    """At the side position the skater sits side_angle off the robot's nose: the pan must reach it, with
    room for the robot's heading wobble (10 deg)."""
    f = load_side_shot()['follower']['ros__parameters']
    assert abs(f['side_angle']) + math.radians(10) <= pan_half_range()


def test_follower_output_goes_through_the_gate():
    assert load()['follower']['ros__parameters']['cmd_vel_topic'] == '/cmd_vel_raw'
