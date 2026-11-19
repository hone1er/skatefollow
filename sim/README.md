# honebot simulator (MuJoCo)

A physics simulation of the follow robot that runs natively on the Mac (Apple Silicon), on Linux and on the Pi 5.
It replaces turtlesim for testing the follow behaviour. turtlesim still works (`honebot_follow`), but it moves
an ideal point; this moves the robot.

## What is simulated, and from where

| Part | Model | Source of the numbers |
|---|---|---|
| Body | 4.02 kg rigid body at the CAD centre of gravity, 4 wheels on hinges, contact with the ground; scrub, sliding and tipping come out of the physics | CAD script `build_model()` + `cad/report/check_report.md` |
| Drivetrain | per side one 370 KV motor on an AM32 ESC in voltage mode: current = (throttle × 11.1 V − back-EMF) / 0.15 Ω, limited to 30 A; 2:1 belt; rear wheel tied 1:1 to the front; rolling resistance; air drag | `cad/check_design.py` drivetrain section |
| ESP32 | the **real** `drive_logic.h`, compiled on first run and called directly (who may drive, slew, fail-safe ramp, deadband), plus the sketch's loop code; every constant parsed from the `.ino` | `firmware/esp32_ultrasonic/` |
| Pi software | the **real** pure modules: `follow_command_relative`, `gate_command` with the sight limit, `twist_to_diffdrive`, uwb_node's `RangeFilter` + `aoa_position`, camera_aim's `PanController`; the nodes' rates and timeouts are mirrored | `src/honebot_*`, `robot.yaml` |
| HC-SR04 | rays fanned over the ±15° beam from the CAD mount: ground return ~1.95 m, obstacles, the skater; 60 ms pings, no echo past 4.29 m | CAD + firmware |
| UWB AoA | 3-D range + horizontal bearing, noise, drop-outs, worse when something blocks the line of sight | datasheet-class guesses |
| Camera | GoPro on the pan servo; detector at 10 Hz with 0.1 s latency and 95 % recall, hidden behind obstacles | measured det/s, guessed recall |
| Skater | scripted paths: walking, cruising at 6/8/12/15 mph with dead stops, a bail, slalom, U-turn, stop-and-go, a box to avoid, slow circles around the robot | `honebot_sim/skater.py` |

Everything runs on one simulated clock, so there are no false timeouts. Not exercised (they need ROS): message
transport, parameter handling, serial I/O, the camera node's frame grabbing and its Kalman tracker.

**Guesses that matter.** Tyre friction (0.8), UWB and camera noise, and motor resistance are estimates until
the real robot exists. The turning results depend directly on friction. Use the simulator to compare designs and
settings, and calibrate it against the first real drives (`docs/first_drive.md`): change `--friction` until
`--spin-test` matches the real spin test.

## Set up on the Mac (once)

macOS's own `python3` (3.9) is too old; MuJoCo needs 3.10+. Install Python from Homebrew. On the Mac, in a
terminal:

```bash
brew install python@3.12
```

```bash
cd <your clone of hone1er/skatefollow>
python3.12 -m venv ~/honebot-sim
```

```bash
~/honebot-sim/bin/pip install -r sim/requirements.txt
```

The first run compiles the firmware logic with Xcode's command-line tools (already installed). Check that it works:

```bash
cd sim && ~/honebot-sim/bin/python -m pytest -q tests
```

You should see `41 passed`.

## Use it (from the `sim` folder)

Watch a run in 3-D. On macOS the viewer must be started with `mjpython`, which comes with MuJoCo:

```bash
~/honebot-sim/bin/mjpython -m honebot_sim.run carve_6 --view
```

Drag to orbit, scroll to zoom. In the right-hand panel, *Rendering → Camera* switches to `chase` or `gopro` (what
the GoPro sees). `--speed 0.5` plays it in slow motion.

Run every scenario headless and get a pass/fail table (about 20× real time):

```bash
~/honebot-sim/bin/python -m honebot_sim.run --all
```

One run, with a plot of speeds, gap, turn rate, camera pan and motor current:

```bash
~/honebot-sim/bin/python -m honebot_sim.run cruise_stop_8 --plot run.png --csv run.csv
```

Try a setting before putting it on the robot (`--set` = any `robot.yaml` parameter, `--fw` = any firmware constant):

```bash
~/honebot-sim/bin/python -m honebot_sim.run --all --set follower.follow_bearing=0.45 --set follower.max_linear=2.5
```

### Drive it yourself: your handheld remote, or the keyboard

The simulated robot ESP32 runs the same code as the real one, so the remote works the same way. RUN = hold the
stick button 1.5 s with the stick centred. Any click = STOP. With the stick centred the follower drives; move the
stick and you drive, at most 25 % throttle (`MANUAL_LIMIT`), even if the Pi wants something else. STOP, or
unplugging the remote, stops the robot. The `playground` scenario has a skater standing 8 m ahead and boxes to
drive around.

**With the real remote** (flash it first: `firmware/esp32_remote/README.md`). Plug the remote ESP32 into the Mac
by USB; nothing else needs to be connected. It prints its stick and RUN/STOP over USB at 20 Hz, and the simulator
uses that in place of the radio. Find its port:

```bash
ls /dev/cu.usbserial-*
```

Then (with the port you found):

```bash
~/honebot-sim/bin/mjpython -m honebot_sim.run playground --view --remote /dev/cu.usbserial-XXXX
```

The terminal shows once a second who is driving (`nobody`, `Pi (follower)`, `STICK`), the speed and the gate
state. The remote's own LED shows "no robot" (slow blink) because no real robot answers it; that is expected.
Close the Arduino serial monitor first, because only one program can use the port.

**Without the remote:**

```bash
~/honebot-sim/bin/mjpython -m honebot_sim.run playground --view --keys
```

Click the 3-D view, press **R** for RUN, then use the **arrow keys** (each press moves the virtual stick 0.2)
and **Space** to centre it.

**Gentle turns need `--yaw-loop`.** Without it (today's firmware settings), the robot goes straight until the
stick is nearly sideways and then pivots hard: a diagonal does not give the tyres enough left/right difference to
slide (see "What it found"). With the firmware's gyro loop on, the side stick asks for a turn *rate*: 20° of stick
turns at 0.35 rad/s, 45° at 1.4, full side at 3 (`MANUAL_MAX_YAW`). The real robot needs an MPU6050 for this
(firmware README).

```bash
~/honebot-sim/bin/mjpython -m honebot_sim.run playground --view --remote /dev/cu.usbserial-XXXX --yaw-loop
```

Any scenario works with `--remote` or `--keys`: take over from the follower in `carve_6`, or press STOP mid-run
and watch the fail-safe ramp.

Other options: `--list` (scenarios), `--spin-test` (turn-rate sweep), `--profile target` (what 12–15 mph needs),
`--yaw-loop` (the firmware's gyro loop on, as if an MPU6050 were fitted), `--gyro-fault sign|stuck|dead`,
`--friction 0.6`, `--seed 3` (different sensor noise).

## What it found (2026-11-19)

With `robot.yaml` and the firmware exactly as they are (`--profile today`), 8 of the 11 test scenarios pass
(`--all` runs those 11; the 12th, `playground`, is for driving by hand).

1. **Straight-line following and stopping are safe.** The robot stops 1.1–1.2 m from a skater who stops dead,
   and it stops for the box. It never touches the skater in any scenario.
2. **The HC-SR04 caps the robot at about 2.7 m/s (6 mph)**, as the safety gate's sight limit intends. Faster
   skaters pull away, and past 30 m of UWB range the robot stops and stays stopped. (This was already known.)
3. **New, a model prediction: it barely turns on small commands.** To pivot, or to turn while rolling, the tyres must slide sideways.
   That takes about 11 N of push per side, about 10 A per motor. With the ESC in voltage mode, a turn command
   gives a current *difference* proportional to the *voltage* difference. The follower's usual turn requests
   (0.5–3 rad/s) give 1–5 A, so the robot goes straight. `--spin-test`: no turn at all for any
   command up to 6 rad/s. In the slalom, the U-turn and the slow circle, the skater ends up behind the robot, the UWB
   bearing passes 80° and the robot stops. The prediction holds for every MuJoCo contact model tried (elliptic or
   pyramidal friction, hard or soft) and for friction 0.5–0.8. Real urethane creeps a little sideways, which this
   rigid-friction model does not have, so expect heavy understeer rather than literally zero turn. **The real
   spin test (`docs/first_drive.md` Stage 2, step 3) settles it.** `ESC_DEADBAND` does not fix it either: at 0.10
   or 0.13 it lifts *every* wheel command, so the robot drove faster than the follower asked, hit the box in
   `obstacle` and, at 0.13, the skater in `cruise_stop_8`. Keep it for the ESC's own dead zone only.
4. **A yaw-rate loop fixes it.** With a gyro on the ESP32 and a PI loop that adds the missing wheel difference
   (firmware 2.3.0, `YAW_LOOP_ENABLED`, off by default; `--yaw-loop` here), all 11 scenarios pass. The spin test then tracks the
   command to within 0.05 rad/s at about 10 A. The effective track width comes out at 2–2.7 × geometric,
   not the 1.4 × guessed in `robot.yaml`. A gyro is a few dollars: an MPU6050 breakout on the ESP32's I2C, or
   the one on the Crius board.
5. **12–15 mph works in simulation** with the yaw loop, a 12 m forward lidar, `max_linear` 7.5 and
   `THROTTLE_LIMIT` 1.0 (`--profile target`): 15 mph cruise, then a dead stop 1.0 m short of the skater.
   **Mount such a lidar tilted up by half its beam (1° for a TFmini).** Level at 10 cm, its 2° beam hits the
   road at 5.9 m, and the gate would treat that as an obstacle and cap the speed at about 12.8 mph.

## Files

```
sim/
├── requirements.txt
├── honebot_sim/
│   ├── config.py      every number, read from the CAD, the report, robot.yaml and the .ino
│   ├── model.py       MuJoCo model (MJCF) + the motor/ESC model
│   ├── firmware.py    emulated ESP32: drive_logic.h via ctypes + the sketch's loop code
│   ├── fw_shim.cpp    C entry points around drive_logic.h
│   ├── sensors.py     HC-SR04 / TFmini rays, UWB AoA, camera detector
│   ├── stack.py       the Pi software (follower, gate, bridge, uwb_node, camera_aim), on the sim clock
│   ├── skater.py      scenarios
│   ├── sim.py         the closed loop + metrics + the gyro model (noise, bias, faults) + 50 Hz PWM frames
│   ├── remote_link.py the handheld remote over USB, or the keyboard, as the radio link
│   └── run.py         command line, viewer, plots, spin test
└── tests/             41 tests (also in CI)
```

The compiled firmware library is cached in `sim/.cache/` (git-ignored) and rebuilt whenever `drive_logic.h`
changes. If the sketch's `updateOutputs()`, `handleLine()` or `applyOutputs()` change, `tests/test_firmware.py`
fails on purpose. Mirror the change in `honebot_sim/firmware.py`, then update the hash in the test.
