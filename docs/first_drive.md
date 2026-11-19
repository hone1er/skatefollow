# From printed parts to the first filmed run

A staged plan. Each stage ends with a check; do not start the next one until it passes. Commands say which
machine they run on. On the Pi they assume `cd ~/honebot_ws` and a built workspace.

**Rules for every stage**
- The handheld remote is in your hand and in STOP before the battery goes in.
- Empty lot, nobody nearby. Public streets and sidewalks may not be legal.
- Wheels off the ground until Stage 4.
- One change at a time; write down what you changed.

## Speed envelope: what limits the robot today

The robot is designed for 12–15 mph. **Today it is limited to about 6–8 mph**, and each limit below has to be
raised deliberately, not all at once.

| Limit | Value now | Where it lives | What raises it |
|---|---|---|---|
| Obstacle sight | 2.7 m/s (6 mph) when the ultrasonic sensor reads the ground ahead; 3.8 m/s (8.5 mph) when it reads nothing | safety gate `sight_distance`, `brake_decel` in `config/robot.yaml` | A longer-range forward sensor (`docs/robot_design_v2.md` §1.6 and §4 list a 12 m ToF and a 2D lidar). Not a bigger number |
| Follower top speed | 3.0 m/s | `max_linear` in `config/robot.yaml`, or `max_linear:=` on `film.launch.py` | Testing at each speed step |
| Firmware throttle cap | command 0.5 ≈ 4.8 m/s no-load | `THROTTLE_LIMIT` in the robot sketch | Edit and reflash once the robot behaves |
| Manual (stick) speed | ≈ 2.4 m/s | `MANUAL_LIMIT` in the robot sketch | Edit and reflash |
| Printed axle sleeves | ~8 mph over 6 mm cracks, 6 mph over 10 mm lips | hardware (`docs/chassis_cad.md` §6b) | Steel tubes |
| Printed TPU belts | life unknown; inspect every session | hardware | HTD-3M 258 mm and 510 mm belts |
| Follow gap | 4.7 m at 8 mph, 7.5 m at 12, 10 m at 15 | `follow_distance`, `time_headway`, `brake_decel` | Only with measured braking that supports it |

## Stage 0 — bench, no motors (now)

1. **Mac:** compile and flash both boards (`firmware/esp32_ultrasonic`, `firmware/esp32_remote`), and do the remote
   check in `firmware/esp32_remote/README.md`.
2. **Pi:** `./scripts/pi_setup.sh`, then:
   ```bash
   journalctl -u honebot -n 30
   ```
   You should see `ESP32 firmware version 2.2.1` and `Remote (kill switch): stop`.
3. **Pi:** hold the remote's button 1.5 s. The log shows `Remote (kill switch): run`. Click: `stop`. Switch the
   remote off: `lost` within a second.
4. **Sensor beam test** (decides which sight limit applies outdoors). Hold the HC-SR04 as it will be mounted:
   upside down, 100 mm above open pavement, tilted 12° up, nothing ahead for 5 m.
   ```bash
   ros2 topic echo /ultrasonic/range --field range
   ```
   - `inf` most of the time → the 3.8 m/s limit applies.
   - About 1.9–2.0 → the 2.7 m/s limit applies.
   - 0.5–1.5 → it sees the ground too close: raise `us_tilt_deg` in the CAD before printing the bracket.
5. **How far does it see a person?** Stand 4 m in front of the sensor and walk toward it; note the distance at
   which the reading first drops from `inf` to a number. The sight limit assumes 3 m (`sight_distance`): if you
   measured less, set that value in `config/robot.yaml`.

## Stage 1 — motors on the bench, wheels off the ground

1. Configure both ESCs with the AM32 tool: bidirectional, sine start-up on, low-voltage cutoff 3.3 V per cell,
   and braking as the command falls (see `firmware/esp32_ultrasonic/README.md`, "Bench check").
2. **Find the smallest command that turns the wheels.** ESCs ignore small commands around neutral, and slow
   moves are tiny: 1 m/s is a command of about 0.10, and a slow pivot is only 0.02 per side. Remote in RUN.
   **Pi:** run this, then stop it (Ctrl-C) and repeat with `x: 0.2`, `0.4`, `0.6`, … until the wheels turn:
   ```bash
   ros2 topic pub -r 20 /cmd_vel_raw geometry_msgs/Twist "{linear: {x: 0.2}}"
   ```
   Note the first `x` that turns all four wheels steadily. Set `ESC_DEADBAND` in the robot sketch to
   `x / 9.6` minus a little (wheels first turn at `x: 0.6` → 0.0625 → use `0.05`) and reflash. After that, even
   `x: 0.1` should turn them slowly. If they already turn at `x: 0.2`, leave `ESC_DEADBAND` at 0.
3. **Direction.** With `x: 1.0`, all four wheels turn **forward**. If a side runs backward, set `THROTTLE_REVERSED` (left) or `STEER_REVERSED`
   (right) in the robot sketch and reflash. Do not use the bridge's `left_reversed` / `right_reversed`.
4. Push the remote's stick forward: wheels turn forward. Stick left: the right side runs faster.
5. **Every stop, one at a time.** Use `x: 2.0` so there is something to stop. The wheels must wind down in about a second each time:
   - click STOP on the remote;
   - switch the remote off;
   - Ctrl-C the `ros2 topic pub`;
   - put a hand 20 cm in front of the sensor (forward command only);
   - unplug the ESP32's USB cable from the Pi.
6. Belts: no skipping, no rubbing on flanges, pulleys do not wobble.

## Stage 2 — first ground drive, by stick

1. Remote in RUN, drive by stick at walking pace. Straight, both turns, stop.
2. **Radio range:** shell on, walk away until the remote's LED starts blinking slowly. That distance, with a
   margin, is how far you may ever be from the robot.
3. **Spin test:** command a slow pivot and compare with what the robot really does.
   ```bash
   ros2 topic pub -r 20 /cmd_vel_raw geometry_msgs/Twist "{angular: {z: 1.0}}"
   ```
   (If nothing moves, `ESC_DEADBAND` is not set yet: Stage 1, step 2. **The simulator predicts the robot will
   barely pivot at 1 rad/s even then**: the tyres need ~10 A per motor to scrub and this command gives ~1 A.
   `sim/README.md` "What it found" explains it, and the fix is a gyro yaw-rate loop. Note what really happens
   here: it decides whether that loop is needed.) Time one full turn. It should take 6.3 s. If it takes longer, raise `track_width` in `config/robot.yaml` in
   proportion (9 s → 0.39 × 9 / 6.3 = 0.56) and restart the service.
4. **Speed calibration:** drive a measured 10 m at `linear.x: 1.0` and time it. If it takes 12 s the robot does
   0.83 m/s per commanded 1.0: multiply `max_wheel_speed` by 0.83.
5. **Braking distance:** at `linear.x: 2.0`, click STOP as the nose passes a mark; measure where it stops. The
   gap law assumes 4.4 m/s² (0.45 g): from 2 m/s that is 0.45 m plus about 0.5 m of reaction. If it needs more,
   lower `brake_decel` in **both** places in `config/robot.yaml`.
6. Feel the motors and ESCs after 5 minutes: warm is fine, too hot to hold is not.

## Stage 3 — UWB on the bench

1. Plug in the anchor; find its port (`ls /dev/ttyUSB*`). Its firmware must print `A,<range_m>,<bearing_deg>`
   lines; if it prints something else, send a sample and the node gets a parser for it.
2. **Pi:**
   ```bash
   ros2 launch honebot_uwb uwb.launch.py port:=/dev/ttyUSB1
   ```
   ```bash
   ros2 topic echo /target --field point
   ```
   Walk the tag around: `x` is metres ahead, `y` is metres to the **left**. If left reads negative, set
   `bearing_sign: -1.0`.
3. Cover the tag or walk out of range: `/target` must **stop publishing** (the log's fixes/s drops to 0).
   That silence is what stops the robot.

## Stage 4 — following, on foot

1. Remote in RUN, robot on the ground, tag on you. **Pi:**
   ```bash
   ros2 launch honebot_bringup film.launch.py max_linear:=1.0 camera:=false
   ```
2. Walk away. The robot follows and stops 1.5 m short when you stop. Walk a circle: it arcs, it does not pivot.
3. Stop tests while it follows: click STOP; cover the tag; step in front of the sensor; Ctrl-C the launch.
4. Raise `max_linear` to 2.0, then 3.0, jogging.
5. Try the side shot:
   ```bash
   ros2 launch honebot_bringup film.launch.py max_linear:=2.0 follow_bearing:=0.45
   ```
   The robot settles behind you and to your right.

## Stage 5 — camera

1. GoPro on the mast, servo wired. Add the camera: drop `camera:=false`.
2. In Foxglove, `/camera/image/compressed` shows you with a box; the pan follows you. If it pans the wrong way,
   add `pan_sign:=-1.0` to the `film.launch.py` command.
3. No part of the robot may appear in frame at either pan limit.

## Stage 6 — on the board

Rolling slowly first. One speed step per session. Every run is logged to `~/bags/` by `film.launch.py`: when
something looks wrong, keep the bag and note the time.

Before going past 8 mph: steel axle tubes, real belts, a longer-range forward sensor, and a measured braking
distance that supports the gap.
