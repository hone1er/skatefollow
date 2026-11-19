#!/usr/bin/env python3
"""Fake handheld remote: prints the remote board's serial lines into a pseudo-terminal, so the ROS remote
bridge (honebot_remote_bridge, source:=serial) can be tested with no ESP32 at all.

Same line format and rate as firmware/esp32_remote/esp32_remote.ino (5 lines per second):
    state=RUN x=+1.00 y=+0.00 btn=0 robot=0 pi=0 failsafe=0 manual=0
x = forward (+1 = stick pushed away), y = turn (+1 = stick pushed left). Like the real remote it boots in
STOP, `run` only works with the stick centred, and a click (`stop`) stops at once.

Python 3 standard library only. Linux (the Pi) or macOS.

    python3 tests/sim_remote.py            # type commands, see below
    python3 tests/sim_remote.py --demo     # loops a scripted drive

Then, in another terminal:
    ros2 launch honebot_remote_bridge bridge_launch.py port:=/tmp/honebot_fake_remote

Commands (type, then Enter):
    run          hold the button 1.5 s (needs the stick centred)  -> RUN
    stop         click the button                                  -> STOP
    w / s        stick forward / back          a / d   stick left / right
    c            centre the stick              x <fwd> <turn>      any stick position, e.g. "x 0.5 -0.3"
    off / on     stop / resume sending lines (remote switched off or out of range)
    q            quit
"""

import argparse
import os
import pty
import sys
import threading
import time
import tty

DEFAULT_LINK = '/tmp/honebot_fake_remote'
PRINT_PERIOD_S = 0.2
CENTRE_BAND = 0.15


class FakeRemote:
    """A pseudo-terminal that behaves like the remote board's USB serial port."""

    def __init__(self, link=DEFAULT_LINK):
        self.master, self.slave = pty.openpty()
        tty.setraw(self.slave)
        os.set_blocking(self.master, False)     # never block when nobody is reading
        self.path = os.ttyname(self.slave)
        self.link = link
        if link:
            try:
                os.unlink(link)
            except FileNotFoundError:
                pass
            os.symlink(self.path, link)
        self.state = 'STOP'
        self.x = 0.0
        self.y = 0.0
        self.sending = True
        self._lock = threading.Lock()
        self._stop_evt = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True)

    # --- what a person does with the remote --------------------------------------------------------
    def stick(self, x, y):
        with self._lock:
            self.x = max(-1.0, min(1.0, float(x)))
            self.y = max(-1.0, min(1.0, float(y)))

    def hold_to_run(self):
        """Press and hold the button. Returns False (stays in STOP) if the stick is not centred."""
        with self._lock:
            if abs(self.x) >= CENTRE_BAND or abs(self.y) >= CENTRE_BAND:
                return False
            self.state = 'RUN'
            return True

    def click(self):
        with self._lock:
            self.state = 'STOP'

    def set_sending(self, on):
        with self._lock:
            self.sending = bool(on)

    # --- plumbing -------------------------------------------------------------------------------------
    def line(self):
        with self._lock:
            return (f'state={self.state} x={self.x:+.2f} y={self.y:+.2f} btn=0 '
                    f'robot=0 pi=0 failsafe=0 manual=0\n')

    def start(self):
        self._write('honebot remote 1.0.0  centre x=1850 y=1900  stick=ok radio=ok (simulated)\n')
        self._thread.start()
        return self

    def close(self):
        self._stop_evt.set()
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)
        if self.link:
            try:
                os.unlink(self.link)
            except FileNotFoundError:
                pass
        for fd in (self.master, self.slave):
            try:
                os.close(fd)
            except OSError:
                pass

    def _write(self, text):
        try:
            os.write(self.master, text.encode('ascii'))
        except (BlockingIOError, OSError):
            pass          # buffer full (nobody reading yet): drop the line, like a real serial port

    def _loop(self):
        while not self._stop_evt.wait(PRINT_PERIOD_S):
            with self._lock:
                sending = self.sending
            if sending:
                self._write(self.line())


DEMO = [  # (seconds, action)
    (2.0, 'stop'), (1.0, 'run'), (2.0, 'w'), (1.0, 'a'), (2.0, 'w'), (1.0, 'd'), (1.5, 's'),
    (1.0, 'c'), (1.0, 'stop'),
]


def apply(remote, cmd):
    parts = cmd.split()
    if not parts:
        return True
    c = parts[0].lower()
    moves = {'w': (1.0, 0.0), 's': (-1.0, 0.0), 'a': (0.0, 1.0), 'd': (0.0, -1.0), 'c': (0.0, 0.0)}
    if c in moves:
        remote.stick(*moves[c])
    elif c == 'x' and len(parts) == 3:
        try:
            remote.stick(float(parts[1]), float(parts[2]))
        except ValueError:
            print('usage: x <forward> <turn>, each -1..1')
    elif c == 'run':
        if not remote.hold_to_run():
            print('refused: centre the stick first (c), like the real remote')
    elif c == 'stop':
        remote.click()
    elif c == 'off':
        remote.set_sending(False)
    elif c == 'on':
        remote.set_sending(True)
    elif c == 'q':
        return False
    else:
        print('commands: run stop w a s d c "x <fwd> <turn>" off on q')
    print('  ' + remote.line().strip() + ('' if remote.sending else '   (not sending)'))
    return True


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--link', default=DEFAULT_LINK, help=f'symlink to the fake port (default {DEFAULT_LINK})')
    ap.add_argument('--demo', action='store_true', help='loop a scripted drive instead of reading commands')
    args = ap.parse_args()

    remote = FakeRemote(args.link).start()
    print(f'Fake remote on {remote.path} (also {args.link}). Start the bridge with:')
    print(f'  ros2 launch honebot_remote_bridge bridge_launch.py port:={args.link}')
    try:
        if args.demo:
            while True:
                for seconds, cmd in DEMO:
                    apply(remote, cmd)
                    time.sleep(seconds)
        else:
            print('Commands: run stop w a s d c "x <fwd> <turn>" off on q')
            for line in sys.stdin:
                if not apply(remote, line):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        remote.close()


if __name__ == '__main__':
    main()
