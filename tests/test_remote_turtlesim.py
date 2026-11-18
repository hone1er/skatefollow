#!/usr/bin/env python3
"""End-to-end check on the Pi with a fake remote (no ESP32 needed): fake remote -> remote_bridge -> turtlesim.

Remote only (the remote drives turtle1):
    Terminal 1:  ros2 launch honebot_remote_bridge bridge_launch.py port:=/tmp/honebot_fake_remote
    Terminal 2:  python3 tests/test_remote_turtlesim.py
  1. STOP with the stick pushed forward -> no motion    4. RUN, stick left -> turns counter-clockwise
  2. RUN, stick centred -> no motion                    5. click (STOP) with the stick forward -> stops
  3. RUN, stick forward -> drives forward               6. remote goes silent -> stops after 0.5 s

Follow bot + remote override (turtle1 = skater, turtle2 = robot driven by the follower):
    Terminal 1:  ros2 launch honebot_remote_bridge bridge_launch.py follow:=true port:=/tmp/honebot_fake_remote
    Terminal 2:  python3 tests/test_remote_turtlesim.py --follow
  1. STOP: the follower wants to drive, but the robot stays still (kill switch holds the follow bot too)
  2. RUN, stick centred: the follower closes in on the skater
  3. the skater drives away: the follower keeps up
  4. RUN, stick pulled back: you override the follower, the robot backs away
  5. stick released: after the 0.8 s hold the follower takes over again
  6. click (STOP) while the follower wants to drive: the robot stays still
  7. RUN, then the remote goes silent: the robot stops

(tests/test_esp32_remote.sh fake [follow] starts the launch and this script together.)
Prints PASS/FAIL per check; exit code 0 when all pass. Restart the launch between runs.
"""

import argparse
import math
import os
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String
from turtlesim.msg import Pose

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'firmware', 'tests'))
from sim_remote import DEFAULT_LINK, FakeRemote  # noqa: E402

SETTLE_S = 0.6        # line period 0.2 s + bridge 20 Hz + turtlesim, with margin
STILL_M = 0.05        # "did not move"


class Watcher(Node):
    def __init__(self):
        super().__init__('remote_turtlesim_check')
        self.lock = threading.Lock()
        self.poses = {}
        self.yaw = {}
        self.sources = []
        for name in ('turtle1', 'turtle2'):
            self.create_subscription(Pose, f'/{name}/pose', lambda m, n=name: self._on_pose(n, m), 10)
        self.create_subscription(String, '/remote/source', self._on_source, 10)
        self.skater_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.skater_speed = 0.0
        self.create_timer(0.1, self._drive_skater)

    def _on_pose(self, name, msg):
        with self.lock:
            prev = self.poses.get(name)
            if prev is not None:
                d = msg.theta - prev.theta
                self.yaw[name] = self.yaw.get(name, 0.0) + math.atan2(math.sin(d), math.cos(d))
            self.poses[name] = msg

    def _on_source(self, msg):
        with self.lock:
            if not self.sources or self.sources[-1] != msg.data:
                self.sources.append(msg.data)

    def _drive_skater(self):
        if self.skater_speed:
            t = Twist()
            t.linear.x = self.skater_speed
            self.skater_pub.publish(t)

    def snap(self, name):
        with self.lock:
            p = self.poses.get(name)
            return (p.x, p.y, self.yaw.get(name, 0.0), p.theta) if p else None

    def gap(self):
        a, b = self.snap('turtle1'), self.snap('turtle2')
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def source_log(self):
        with self.lock:
            return list(self.sources)

    def clear_sources(self):
        with self.lock:
            self.sources = self.sources[-1:]


def measure(node, name, seconds, settle=SETTLE_S):
    """Motion of turtle `name` over `seconds` after `settle`: (distance, turn, forward along its heading)."""
    time.sleep(settle)
    x0, y0, yaw0, th0 = node.snap(name)
    time.sleep(seconds)
    x1, y1, yaw1, _ = node.snap(name)
    return (math.hypot(x1 - x0, y1 - y0), yaw1 - yaw0,
            (x1 - x0) * math.cos(th0) + (y1 - y0) * math.sin(th0))


def report(results, ok, name, detail):
    results.append(ok)
    print(f'{"PASS" if ok else "FAIL"}: {name}  ({detail})')


def remote_only(node, remote, results):
    def check_still(name, m):
        ok = m[0] < STILL_M and abs(m[1]) < 0.05
        report(results, ok, name, f'moved {m[0]:.2f} m, turned {math.degrees(m[1]):.0f} deg')

    remote.click()
    remote.stick(1.0, 0.0)
    check_still('STOP + stick forward: does not move', measure(node, 'turtle1', 1.5))

    remote.stick(0.0, 0.0)
    remote.hold_to_run()
    check_still('RUN + stick centred: does not move', measure(node, 'turtle1', 1.0))

    remote.stick(1.0, 0.0)
    m = measure(node, 'turtle1', 1.0)
    report(results, m[2] > 0.5 and abs(m[1]) < 0.1, 'RUN + stick forward: drives forward', f'forward {m[2]:.2f} m')

    remote.stick(0.0, 1.0)
    m = measure(node, 'turtle1', 0.4)
    report(results, m[1] > 0.3, 'RUN + stick left: turns counter-clockwise', f'turned {math.degrees(m[1]):+.0f} deg')

    remote.stick(1.0, 0.0)
    remote.click()
    check_still('click (STOP) with stick forward: stops', measure(node, 'turtle1', 1.0))

    remote.stick(0.0, 0.0)
    remote.hold_to_run()
    remote.stick(-1.0, 0.0)          # backwards, back toward the middle of the window
    time.sleep(1.0)
    remote.set_sending(False)
    check_still('remote silent: stops after the 0.5 s timeout', measure(node, 'turtle1', 1.0, settle=1.0))


def follow(node, remote, results):
    deadline = time.time() + 15
    while node.snap('turtle2') is None and time.time() < deadline:
        time.sleep(0.1)
    if node.snap('turtle2') is None:
        report(results, False, 'turtle2 exists', 'no /turtle2/pose: did you launch with follow:=true?')
        return

    # 1. STOP holds the follow bot
    remote.click()
    g0 = node.gap()
    m = measure(node, 'turtle2', 1.5)
    report(results, m[0] < STILL_M and g0 > 2.0, 'STOP: robot stays still although the follower wants to drive',
           f'gap {g0:.1f} m, robot moved {m[0]:.2f} m')

    # 2. RUN, stick centred: the follower drives
    remote.stick(0.0, 0.0)
    remote.hold_to_run()
    g0 = node.gap()
    time.sleep(3.0)
    g1 = node.gap()
    src = node.source_log()
    report(results, g1 < g0 - 1.0 and 'pi' in src, 'RUN + stick centred: the follower closes in',
           f'gap {g0:.1f} -> {g1:.1f} m, source {src[-1] if src else "?"}')

    # 3. skater drives away, follower keeps up
    node.skater_speed = 1.0
    time.sleep(2.0)
    node.skater_speed = 0.0
    time.sleep(2.0)
    g = node.gap()
    report(results, g < 3.0, 'skater drives 2 m away: the follower keeps up', f'gap {g:.1f} m')

    # 4. override: pull the stick back
    node.clear_sources()
    remote.stick(-0.6, 0.0)
    m = measure(node, 'turtle2', 1.0, settle=0.4)
    src = node.source_log()
    report(results, m[2] < -0.3 and 'manual' in src, 'stick pulled back: you override, the robot backs away',
           f'forward {m[2]:+.2f} m, source {"/".join(src)}')

    # 5. release: hold, then the follower again
    node.clear_sources()
    remote.stick(0.0, 0.0)
    g0 = node.gap()
    time.sleep(3.0)
    g1 = node.gap()
    src = node.source_log()
    report(results, g1 < g0 - 0.3 and src[-1:] == ['pi'], 'stick released: the follower takes over after the hold',
           f'gap {g0:.1f} -> {g1:.1f} m, source {"/".join(src)}')

    # 6. STOP while the follower wants to drive
    remote.stick(-0.6, 0.0)
    time.sleep(1.2)                 # back off so the follower has somewhere to go
    remote.click()
    remote.stick(0.0, 0.0)
    g0 = node.gap()
    m = measure(node, 'turtle2', 1.5)
    report(results, m[0] < STILL_M and g0 > 2.0, 'click (STOP): robot stays still although the follower wants to drive',
           f'gap {g0:.1f} m, robot moved {m[0]:.2f} m')

    # 7. RUN, then the remote goes silent
    remote.hold_to_run()
    time.sleep(0.8)                 # the follower starts moving
    remote.set_sending(False)
    m = measure(node, 'turtle2', 1.0, settle=1.0)
    report(results, m[0] < STILL_M, 'remote silent: robot stops after the 0.5 s timeout', f'moved {m[0]:.2f} m')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--follow', action='store_true', help='test the follow bot (turtle2) with the remote override')
    args = ap.parse_args()

    rclpy.init()
    node = Watcher()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    remote = FakeRemote(DEFAULT_LINK).start()
    launch = (f'ros2 launch honebot_remote_bridge bridge_launch.py {"follow:=true " if args.follow else ""}'
              f'port:={DEFAULT_LINK}')
    print(f'Fake remote on {DEFAULT_LINK}. Waiting for /turtle1/pose (is "{launch}" running?)')

    deadline = time.time() + 30
    while node.snap('turtle1') is None and time.time() < deadline:
        time.sleep(0.1)
    results = []
    if node.snap('turtle1') is None:
        print(f'FAIL: no /turtle1/pose after 30 s. Start:\n  {launch}')
    else:
        time.sleep(2.0)       # the bridge reconnects once a second: let it find the port
        (follow if args.follow else remote_only)(node, remote, results)

    remote.close()
    rclpy.shutdown()
    print(f'\n{sum(results)}/{len(results)} checks passed')
    return 0 if results and all(results) else 1


if __name__ == '__main__':
    sys.exit(main())
