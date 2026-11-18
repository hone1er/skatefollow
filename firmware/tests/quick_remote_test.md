# Remote + turtlesim: quick checklist

Details, troubleshooting and the "why": `firmware/tests/ESP32_REMOTE_TESTING.md`.

## Mac: flash and check the remote
```bash
firmware/tests/test_remote_turtle.sh /dev/cu.usbserial-0001
```
```bash
arduino-cli monitor -p /dev/cu.usbserial-0001 -c baudrate=115200
```
- [ ] lines like `state=STOP x=+0.00 y=+0.00 btn=0 robot=0 pi=0 failsafe=0 manual=0`, 5 per second
- [ ] stick away from you: `x=+1.00`; stick left: `y=+1.00`
- [ ] hold the button 1.5 s, stick centred: `state=RUN`; click: `state=STOP`

## Pi: build once
```bash
cd ~/honebot_ws && colcon build --symlink-install --packages-select honebot_remote_bridge honebot_hardware
```
```bash
source install/setup.bash
```

## Pi: no hardware (fake remote)
```bash
QT_QPA_PLATFORM=linuxfb ros2 launch honebot_remote_bridge bridge_launch.py port:=/tmp/honebot_fake_remote
```
```bash
python3 tests/test_remote_turtlesim.py
```
- [ ] `6/6 checks passed`

## Pi: remote board on USB
```bash
QT_QPA_PLATFORM=linuxfb ros2 launch honebot_remote_bridge bridge_launch.py port:=/dev/ttyUSB0
```
## Pi: robot ESP32 on USB, remote by radio (the real chain)
```bash
QT_QPA_PLATFORM=linuxfb ros2 launch honebot_remote_bridge bridge_launch.py source:=esp32 port:=/dev/ttyUSB0
```
For either:
- [ ] STOP (power-up): turtle still, even with the stick pushed
- [ ] hold 1.5 s (stick centred) -> RUN; stick away: forward; stick left: turns counter-clockwise
- [ ] click: stops at once
- [ ] remote unplugged / switched off: stops within 0.5 s

Add `follow:=true` to have the follow-bot code (turtle2) chase the turtle you drive.
