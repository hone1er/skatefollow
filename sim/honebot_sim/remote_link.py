"""Drive the simulated robot with the real handheld remote (USB) or with the keyboard (viewer).

Real remote: plug the remote ESP32 into the computer with USB. Its sketch prints, over USB, the same thing it
sends by radio:
    state=RUN x=+0.42 y=-0.10 btn=0 robot=0 pi=0 failsafe=1 manual=0
(x = forward, y = turn, both -1..1). RemoteSerial reads those lines and the simulated robot ESP32 uses them
exactly like radio packets: the real drive::decide() picks stick or Pi, STOP (any click) or a remote that goes
quiet for 500 ms stops the robot, RUN = hold the button 1.5 s with the stick centred.
(robot=0 on the remote's own LED is expected: no real robot answers it.)

Keyboard (viewer window, no remote): arrow keys move a virtual stick in steps, Space centres it, R toggles
RUN/STOP. Starts in STOP like the real remote.
"""

import re
import threading
import time

LINE = re.compile(r"state=(RUN|STOP|FAULT)\s+x=([+-]?[\d.]+)\s+y=([+-]?[\d.]+)")


class RemoteSerial:
    def __init__(self, port, baud=115200, timeout=0.12, clock=time.monotonic):
        try:
            import serial
        except ImportError as e:
            raise SystemExit("pyserial is needed for --remote: pip install pyserial") from e
        self.ser = serial.Serial(port, baud, timeout=0.1)
        # "lost" here only means "no fresh line": ~2 missed lines at the 20 Hz print rate. The emulated firmware
        # then applies its own REMOTE_TIMEOUT_MS on top, exactly as it does when radio packets stop.
        self.timeout = timeout
        self.clock = clock              # tests pass a fake clock; live use is wall time (the viewer runs in real time)
        self.state, self.x, self.y = "none", 0.0, 0.0
        self.last = None
        self.lines = 0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self.thread = threading.Thread(target=self._read, daemon=True)
        self.thread.start()

    def _read(self):
        buf = b""
        while not self._stop.is_set():
            try:
                buf += self.ser.read(256)
            except Exception:
                time.sleep(0.1)
                continue
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                m = LINE.search(line.decode("ascii", "replace"))
                if m:
                    run = m.group(1) == "RUN"
                    state = "run" if run else "stop"
                    try:
                        x, y = (float(m.group(2)), float(m.group(3))) if run else (0.0, 0.0)
                    except ValueError:
                        continue
                    now = self.clock()
                    with self._lock:
                        self.state = state
                        self.x, self.y = x, y
                        self.last = now
                        self.lines += 1
            if len(buf) > 512:
                buf = b""

    def poll(self):
        """(state, forward, turn): state run | stop | lost | none."""
        with self._lock:
            last, state, x, y = self.last, self.state, self.x, self.y
        if last is None:
            return "none", 0.0, 0.0
        if self.clock() - last > self.timeout:
            return "lost", x, y                    # the firmware keeps the last packet's stick too
        return state, x, y

    def close(self):
        self._stop.set()
        try:
            self.ser.close()
        except Exception:
            pass


class KeyboardRemote:
    """Virtual remote for the viewer's key callback (GLFW key codes)."""
    UP, DOWN, LEFT, RIGHT, SPACE, R = 265, 264, 263, 262, 32, 82

    def __init__(self, step=0.2):
        self.step = step
        self.run = False
        self.x = self.y = 0.0

    def key(self, code):
        if code == self.R:
            self.run = not self.run
            self.x = self.y = 0.0
            print("remote:", "RUN" if self.run else "STOP")
        elif code == self.SPACE:
            self.x = self.y = 0.0
        elif code == self.UP:
            self.x = min(1.0, self.x + self.step)
        elif code == self.DOWN:
            self.x = max(-1.0, self.x - self.step)
        elif code == self.LEFT:
            self.y = min(1.0, self.y + self.step)
        elif code == self.RIGHT:
            self.y = max(-1.0, self.y - self.step)

    def poll(self):
        return ("run", self.x, self.y) if self.run else ("stop", 0.0, 0.0)
