"""config/robot.yaml must only use parameters the nodes really declare.

ROS silently ignores a YAML key that no node declares, so a typo here would look like a setting that
"does nothing". This test reads each node's source for its declare_parameter names and compares.
"""
import os
import re

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.abspath(os.path.join(HERE, '..', '..'))
CONFIG = os.path.join(HERE, '..', 'config', 'robot.yaml')

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


def test_skater_stays_inside_the_camera_pan_range_when_stopped():
    f = load()['follower']['ros__parameters']
    assert abs(f['follow_bearing']) + f['stopped_turn_deadband'] <= 0.785


def test_follower_output_goes_through_the_gate():
    assert load()['follower']['ros__parameters']['cmd_vel_topic'] == '/cmd_vel_raw'
