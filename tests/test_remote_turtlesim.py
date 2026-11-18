#!/usr/bin/env python3
"""End-to-end check on the Pi: fake remote -> remote_bridge -> turtlesim, no ESP32 needed.

It plays a fake remote (firmware/tests/sim_remote.py) into a pseudo-terminal, watches /turtle1/pose and
checks that the turtle does what the real robot would:

    1. STOP with the stick pushed forward          -> does not move
    2. RUN with the stick centred                   -> does not move
    3. RUN, stick forward                           -> drives forward
    4. RUN, stick left                              -> turns left (counter-clockwise)
    5. click (STOP) with the stick still forward    -> stops
    6. RUN forward, then the remote goes silent     -> stops within the 0.5 s timeout

Terminal 1 (start the bridge first; it keeps retrying until the fake port exists):
    ros2 launch honebot_remote_bridge bridge_launch.py port:=/tmp/honebot_fake_remote
Terminal 2:
    python3 tests/test_remote_turtlesim.py

Prints PASS/FAIL per check; exit code 0 when all pass. Run it with the turtle near the middle of the window
(restart the launch between runs, or `ros2 service call /reset std_srvs/srv/Empty`).
"""

import math
import os
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'firmware', 'tests'))
from sim_remote import DEFAULT_LINK, FakeRemote  # noqa: E402

SETTLE_S = 0.6        # line period 0.2 s + bridge 20 Hz + turtlesim, with margin
STILL_M = 0.05        # "did not move"


class PoseWatcher(Node):
    def __init__(self):
        super().__init__('remote_turtlesim_check')
        self.lock = threading.Lock()
        self.pose = None
        self.yaw = 0.0               # unwrapped heading
        self.create_subscription(Pose, '/turtle1/pose', self._on_pose, 10)

    def _on_pose(self, msg):
        with self.lock:
            if self.pose is not None:
                d = msg.theta - self.pose.theta
                self.yaw += math.atan2(math.sin(d), math.cos(d))
            self.pose = msg

    def snapshot(self):
        with self.lock:
            return (self.pose.x, self.pose.y, self.yaw) if self.pose else None


def main():
    rclpy.init()
    node = PoseWatcher()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    remote = FakeRemote(DEFAULT_LINK).start()
    print(f'Fake remote on {DEFAULT_LINK}. Waiting for /turtle1/pose (is the bridge launch running?)')

    deadline = time.time() + 30
    while node.snapshot() is None and time.time() < deadline:
        time.sleep(0.1)
    if node.snapshot() is None:
        print('FAIL: no /turtle1/pose after 30 s. Start:\n'
              f'  ros2 launch honebot_remote_bridge bridge_launch.py port:={DEFAULT_LINK}')
        remote.close()
        rclpy.shutdown()
        return 1
    time.sleep(2.0)       # the bridge reconnects once a second: let it find the port

    results = []

    def phase(name, seconds, check, settle=SETTLE_S):
        time.sleep(settle)
        x0, y0, yaw0 = node.snapshot()
        time.sleep(seconds)
        x1, y1, yaw1 = node.snapshot()
        dist = math.hypot(x1 - x0, y1 - y0)
        ok, detail = check(dist, yaw1 - yaw0, (x1 - x0) * math.cos(yaw0) + (y1 - y0) * math.sin(yaw0))
        results.append(ok)
        print(f'{"PASS" if ok else "FAIL"}: {name}  ({detail})')

    def still(dist, dyaw, fwd):
        return dist < STILL_M and abs(dyaw) < 0.05, f'moved {dist:.2f} m, turned {math.degrees(dyaw):.0f} deg'

    remote.click()
    remote.stick(1.0, 0.0)
    phase('STOP + stick forward: does not move', 1.5, still)

    remote.stick(0.0, 0.0)
    remote.hold_to_run()
    phase('RUN + stick centred: does not move', 1.0, still)

    remote.stick(1.0, 0.0)
    phase('RUN + stick forward: drives forward', 1.0,
          lambda d, dyaw, fwd: (fwd > 0.5 and abs(dyaw) < 0.1, f'forward {fwd:.2f} m'))

    remote.stick(0.0, 1.0)
    phase('RUN + stick left: turns counter-clockwise', 0.4,
          lambda d, dyaw, fwd: (dyaw > 0.3, f'turned {math.degrees(dyaw):+.0f} deg'))

    remote.stick(1.0, 0.0)
    remote.click()
    phase('click (STOP) with stick forward: stops', 1.0, still)

    remote.stick(0.0, 0.0)
    remote.hold_to_run()
    remote.stick(-1.0, 0.0)          # backwards, back toward the middle of the window
    time.sleep(1.0)
    remote.set_sending(False)
    phase('remote silent: stops after the 0.5 s timeout', 1.0, still, settle=1.0)

    remote.close()
    rclpy.shutdown()
    print(f'\n{sum(results)}/{len(results)} checks passed')
    return 0 if all(results) else 1


if __name__ == '__main__':
    sys.exit(main())
