#!/usr/bin/env python3
"""Check the handheld remote's serial output, no ROS needed (Mac or Pi). Needs pyserial.

Guided check of a real remote board on USB (it asks you to move the stick and press the button, and checks
each step from the serial lines):
    python3 tests/test_remote_basic.py /dev/cu.usbserial-0001      # Mac
    python3 tests/test_remote_basic.py /dev/ttyUSB0                # Pi

Just watch the lines, with what the robot would do for each (wheel commands, speed, turn rate):
    python3 tests/test_remote_basic.py /dev/ttyUSB0 --watch

No hardware: runs the same guided check against the fake remote (firmware/tests/sim_remote.py), playing the
person too. Checks this script, the parser and the pseudo-terminal plumbing:
    python3 tests/test_remote_basic.py --sim

Lines are parsed with the same code the ROS bridge uses (src/honebot_remote_bridge/.../remote_logic.py).
Exit code 0 when every step passed.
"""

import argparse
import os
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.join(HERE, '..', 'src', 'honebot_remote_bridge'),
                os.path.join(HERE, '..', 'firmware', 'tests')]
from honebot_remote_bridge.remote_logic import parse_remote_line, remote_to_wheels, wheels_to_twist  # noqa: E402

STEP_TIMEOUT_S = 20.0


class LineReader:
    """Reads remote lines from a serial port in the background; keeps the newest parsed one."""

    def __init__(self, port):
        import serial      # pyserial, only needed here
        self.ser = serial.Serial(port, 115200, timeout=0.2)
        self.latest = None
        self.latest_time = 0.0
        self.count = 0
        self.raw_last = ''
        self._stop_evt = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        buf = b''
        while not self._stop_evt.is_set():
            try:
                buf += self.ser.read(256)
            except Exception:
                time.sleep(0.2)
                continue
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                text = line.decode('ascii', errors='replace').strip()
                self.raw_last = text
                s = parse_remote_line(text)
                if s is not None:
                    self.latest, self.latest_time = s, time.time()
                    self.count += 1

    def close(self):
        self._stop_evt.set()
        self._thread.join(timeout=1.0)
        self.ser.close()


def describe(s):
    left, right = remote_to_wheels(s)
    linear, angular = wheels_to_twist(left, right)
    return (f'state={s.state:5s} x={s.x:+.2f} y={s.y:+.2f} btn={int(s.btn)} robot={int(s.robot)} '
            f'manual={int(s.manual)}  ->  wheels L={left:+.3f} R={right:+.3f}  '
            f'robot {linear:+.2f} m/s {angular:+.2f} rad/s')


def wait_for(reader, prompt, ok, act=None):
    """Show the prompt, (sim: do the action), wait until ok(state) holds. Returns True/False."""
    print(f'\n-> {prompt}')
    if act:
        act()
    deadline = time.time() + STEP_TIMEOUT_S
    while time.time() < deadline:
        s = reader.latest
        if s is not None and time.time() - reader.latest_time < 1.0 and ok(s):
            print(f'   PASS  {describe(s)}')
            return True
        time.sleep(0.05)
    s = reader.latest
    print(f'   FAIL  (nothing matching in {STEP_TIMEOUT_S:.0f} s; last: {describe(s) if s else reader.raw_last!r})')
    return False


def guided(reader, sim=None):
    """The checks from firmware/esp32_remote/README.md, one at a time. `sim` plays the person."""
    act = (lambda f: (lambda: f(sim))) if sim else (lambda f: None)
    steps = [
        ('Let go of the stick (centred), remote in STOP.',
         lambda s: s.state == 'STOP' and abs(s.x) < 0.15 and abs(s.y) < 0.15,
         act(lambda r: (r.click(), r.stick(0, 0)))),
        ('Push the stick AWAY from you, all the way.',
         lambda s: s.x > 0.9 and abs(s.y) < 0.3, act(lambda r: r.stick(1, 0))),
        ('Pull the stick TOWARD you, all the way.',
         lambda s: s.x < -0.9 and abs(s.y) < 0.3, act(lambda r: r.stick(-1, 0))),
        ('Push the stick LEFT, all the way.',
         lambda s: s.y > 0.9 and abs(s.x) < 0.3, act(lambda r: r.stick(0, 1))),
        ('Push the stick RIGHT, all the way.',
         lambda s: s.y < -0.9 and abs(s.x) < 0.3, act(lambda r: r.stick(0, -1))),
        ('STOP drives nothing: push the stick away again (wheels must stay 0).',
         lambda s: s.x > 0.9 and s.state == 'STOP' and remote_to_wheels(s) == (0.0, 0.0),
         act(lambda r: r.stick(1, 0))),
        ('Let go of the stick, then press and HOLD the button for 1.5 s.',
         lambda s: s.state == 'RUN', act(lambda r: (r.stick(0, 0), r.hold_to_run()))),
        ('In RUN, push the stick away: the robot would drive forward.',
         lambda s: s.run and wheels_to_twist(*remote_to_wheels(s))[0] > 2.0, act(lambda r: r.stick(1, 0))),
        ('Click the button once: STOP.',
         lambda s: s.state == 'STOP', act(lambda r: r.click())),
    ]
    if not sim:
        print('Do each step when asked; each waits up to 20 s. Ctrl+C to quit.')
    results = [wait_for(reader, prompt, ok, a) for prompt, ok, a in steps]
    print(f'\n{sum(results)}/{len(results)} steps passed')
    if not all(results) and not sim:
        print('Axes wrong? Set SWAP_XY / X_REVERSED / Y_REVERSED at the top of firmware/esp32_remote/esp32_remote.ino.\n'
              'Never RUN? Hold a full 1.5 s with the stick centred; state=FAULT means the stick was moved or is '
              'loose at power-up: fix and press EN.')
    return all(results)


def watch(reader):
    print('Showing every line (Ctrl+C to quit)...')
    seen = 0
    while True:
        if reader.count != seen and reader.latest is not None:
            seen = reader.count
            print(describe(reader.latest))
        time.sleep(0.02)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('port', nargs='?', help='remote serial port, e.g. /dev/cu.usbserial-0001 or /dev/ttyUSB0')
    ap.add_argument('--watch', action='store_true', help='only print each line with what the robot would do')
    ap.add_argument('--sim', action='store_true', help='no hardware: check against the fake remote')
    args = ap.parse_args()
    if not args.sim and not args.port:
        ap.error('give the remote serial port, or --sim')

    sim = None
    if args.sim:
        from sim_remote import FakeRemote
        sim = FakeRemote(link=None).start()
        args.port = sim.path
    try:
        reader = LineReader(args.port)
    except Exception as e:
        print(f'Cannot open {args.port}: {e}\nFind the port: Mac `ls /dev/cu.usbserial*`, Pi `ls /dev/ttyUSB*`. '
              'Close any serial monitor or ROS launch that has it open.')
        return 1
    print(f'Reading the remote on {args.port}' + (' (fake remote)' if sim else ''))
    try:
        if args.watch:
            watch(reader)
            return 0
        return 0 if guided(reader, sim) else 1
    except KeyboardInterrupt:
        return 1
    finally:
        reader.close()
        if sim:
            sim.close()


if __name__ == '__main__':
    sys.exit(main())
