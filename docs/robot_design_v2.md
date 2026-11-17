# honebot V2 design: a robot that films like a skate filmer (2026-11-15)

V1 (`robot_design.md`, plus the Fusion script in `honebot_ws/cad/`) stays as it is. This doc starts again from the job
and keeps only what the owner fixed: **Raspberry Pi 5, ESP32, the pan servo, the HC-SR04**. Everything else was
open: drivetrain, motors, controllers, layout and battery.

Prices are listing snapshots seen 2026-11-15. Rows marked *estimate* were not checked; re-check everything
before ordering.

## 0. Assumptions (change these and the numbers below move)

| | Value | Why |
|---|---|---|
| Top speed at full command | **15 mph (6.7 m/s)** on 4S; ~11 mph on 3S with the same pulleys | Owner (2026-11-15): 15–18 mph is plenty, probably too fast; build on 3–4S |
| Normal filming speed | 5–12 mph | Cruising, pushing, lines; following speed is capped by what the sensors can see (§1) |
| Mass | **~3.6 kg** all-up with the H5055 motors (475 g each); **~3.0 kg** with the light variant (§3). V1 estimate was 2.5 | Bottom-up: motors 0.95, frame + body 0.8, wheels/fork/belts/axles 0.6, battery 0.33, electronics + sensors 0.4, GoPro + pan stage + mast 0.36 |
| Size | **~39 × 31 × 21 cm** folded (from the CAD: GoPro off, mast on its lowest pin; the lidar on the nose is 18 cm) | Confirm with the backpack |
| Budget | **~$415–545** for the robot without battery, charger, UWB or the GoPro (§6); ~$60–70 less with the light variant. **Roughly double V1** | V1 was $150–260 with no speed feedback and no long-range sensing |
| Ground | Asphalt and concrete skate spots, cracks up to ~10 mm, curbs avoided | |

## 1. What "filming like a skate filmer" demands

A filmer rides **beside or slightly ahead of the skater, 1.5–3 m away, low, with a wide lens**, and matches their
speed smoothly. Each part of that turns into a requirement:

1. **Offset, not directly behind.** From 15 mph the robot needs about **6.8 m** to stop (0.45 g, plus 0.25 s to
   react). A skater who bails 2–3 m in front of a robot riding straight behind them ends up under it. A human
   filmer rides off to the side for the same reason. **Default shot: side-follow, about 2 m to the side and
   level with or 0.5 m behind the skater.** Also available: a front-3/4 lead and a trailing shot (slower, more distance).
2. **Pan well past ±45°.** In a side-follow the skater sits 60–90° off the robot's nose; in a 3/4 lead, about
   120–135°. **With the kept servo** (measure its travel with 500–2500 µs; standard servos usually reach about
   ±80–90°): trailing and side-follow shots with the robot 0.5–1 m behind level (skater at 63–76°). **Optional
   upgrade:** a 270° servo of the same size and mount adds the front-3/4 lead shot (±135°).
3. **Native GoPro recording.** Webcam mode gives 1080p30 with no HyperSmooth and no SD recording
   (`camera_plan.md`), which isn't filmer footage. **Decision:** the GoPro records natively (4K/2.7K, HyperSmooth,
   Wide lens). A small wide USB camera on the same pan stage feeds the detector (mode B in `camera_plan.md`).
4. **Camera height ~0.5 m, tilted up ~10°.** A full body plus the board from 1.5 m out needs 60° of vertical
   view at 0.5 m height. GoPro Wide gives an *estimated* ~66° vertically (Linear ~56°, too tight close in; GoPro doesn't publish per-mode angles, so check on a test clip). A fixed 10° up-tilt; no tilt
   servo. A folding mast takes it from 0.35 m (low-angle lines) to 0.65 m.
5. **Smooth speed matching.** The robot must hold a speed and arc precisely and gently. That needs **wheel-speed
   feedback, a yaw-rate sensor and no wheel that scrubs or shimmies**. V1's sensorless ESCs and free caster had
   none of that.
6. **See far enough.** The HC-SR04 (4 m) only allows stopping from **~10 mph**. A 12 m lidar allows ~20 mph,
   *if it really sees 12 m outdoors*. The LD19 is rated for 30 klux, and direct sun is roughly 100 klux, so that's
   **unverified**. **Add a 360° 2D lidar, measure its range in full sun (§7), and cap speed from the measured
   range.** Keep the HC-SR04 as a short-range backstop on the ESP32.
7. **Fail safe to a stop.** A link, Pi or kill-switch event must end with the robot braked to a stop and
   standing, which rules out anything that has to balance. **The kill switch commands a brake; it does not cut
   the battery.** Braking is regenerative and needs power, so a blown fuse or dead pack means the robot
   **coasts** (rolling resistance alone takes ~100 m from 15 mph). Keep the battery check and the fuse sized so
   that this is the rare case.

## 2. Layout: five options compared

μ = 0.8 (urethane on asphalt), wheelbase 0.30 m, CG height 0.13 m including the mast.

| Layout | Braking | Yaw at 15 mph | Turn in place | On power loss | Parts, mass, cost |
|---|---|---|---|---|---|
| V1: 2 driven + free caster | 0.36 g (60 % on drive wheels) | **Caster shimmy risk**; small casters drop into cracks | Yes | Coasts, stands | Fewest |
| **2 driven + servo-steered 3rd wheel** | **0.45 g** (battery and motors over the drive axle, 75 %) | **Stable**: the steered wheel holds its angle, no free swivel | **Yes** (wheel turns 90°, robot spins about the drive axle) | Stands; a light trail lets the wheel trail if the servo goes limp | +1 servo (~$15–25), fork |
| 4-wheel skid steer (1 motor per side, belts front + rear) | 0.8 g | Stable, but turns by sliding the tires sideways: stick-slip shakes the footage and odometry drifts | Yes, with heavy scrub (wheelbase ≈ track) | Stands | +2 belts and pulleys, longer frame, ~+0.3 kg |
| Car steering (Ackermann) | 0.45 g rear-drive | Stable | **No** (3-point turns) | Stands | Like the steered trike, but needs room to turn |
| 2-wheel balancer (the V5 ostrich) | Must lean first: ~0.3–0.5 g after a 0.3–0.5 s delay | Active | Yes | **Falls over** | Most control work and risk |

**Pick: two driven rear wheels plus a servo-steered front wheel.** It keeps diff-drive's turn-in-place and
the code already written. The steered wheel takes over the caster's job without shimmy, and the robot follows
smooth arcs without scrubbing, which matters most for footage.
- **Steering angle:** the ESP32 sets the front wheel to δ = atan(L·ω/v), the same formula as the existing Ackermann module.
- **Trade-off:** braking is 0.45 g against 0.8 g for skid steer, which is why the side-offset shot and the lidar
  matter.
- **Fallback:** if field tests show it can't brake hard enough, convert to 4-wheel skid steer. The drive modules
  are the same.

Side tip: **0.64 g** in the CAD (trike, centre of mass 77 % on the drive axle but 133 mm up with the mast raised),
below the ~0.8 g of grip, so a hard turn tips before it slides. The ESP32 caps lateral acceleration (v·ω ≤ 0.4 g). Keep the
battery low and over the axle, and the mast light.

## 3. Drivetrain

**Size from grip, not motor current.** On a 3.6 kg robot, cruising at 15 mph takes about **9 W**. A 0.3 g push
at 15 mph adds ~60 W, and a 5 % slope ~10 W. Tire grip caps the belt pull at ~25 N. Any 100–300 W motor class
is plenty. What matters is **feedback, a shaft that can carry a pulley, and documented dimensions**.

| Choice | V2 pick | Why |
|---|---|---|
| Motor | **Flipsky H5055 200KV, sensored (hall), 8 mm × 25 mm shaft**, listed [$63](https://flipsky.net/products/h5055-motor) ([other listing $43](https://flipsky.net/products/lipsky-sensored-outrunner-brushless-dc-motor-battle-hardened-5055-200kv-1500w)) | Built for a cantilevered belt pulley (an 8 mm shaft at 25 N sees ~13 MPa), halls for smooth starts and odometry, and the listing gives the size: [Ø49.5 × 55.5 mm body, 8 mm shaft, **475 g**](https://brushlessmotorsparade.com/brushless-motor/flipsky-h5055-200kv). On 4S it needs only **1.77:1** for 15 mph (1.33:1 on 3S). **Get the mounting drawing and pole count before ordering.** |
| Controller | **Flipsky Mini Dual FSESC 4.20 (VESC), 8–60 V**, [~$106](https://flipskycn.en.made-in-china.com/product/qRNYcnAPskVG/China-Flipsky-Mini-Dual-ESC-4-20-100A-Drive-with-Vesc-Technology-Aluminum-Heatsink.html) | One board drives both motors with sensored FOC, RPM (speed) control, current limits and regenerative braking, all settable in VESC Tool. The ESP32 talks to it over UART with the VescUart library and reads wheel RPM back for odometry. |
| Reduction | Belt, ~1.8:1 (e.g. 20T on the motor, 36T on the wheel; GT2/HTD 9 mm) | Short centre distance and a low ratio make for small pulleys. Tension by sliding the motor; with an 8 mm shaft it doesn't need an outboard bearing. |
| Wheels | 90 mm, 78A longboard wheels with a bolt-on pulley core (e-skate style), ×3 | Soft, big wheels roll over cracks; the same wheel goes on the front fork. |
| Battery | **4S 3000–4000 mAh LiPo** (~44–59 Wh, ~1.4 h of filming) *estimate $40–60*; 3S packs still fit for slower first tests | A 4S pack is about the same size and weight as V1's 3S 5000. The buck and servo BEC take 4S (16.8 V) fine. |

**Light variant** (~$60–70 cheaper, ~0.6 kg lighter): keep the 5010 360KV (~$19–37) and add a MT6701/AS5047 magnetic encoder per motor,
an [ODESC 3.6 dual](https://flipsky.net/collections/od-drive/products/odesc3-6-dual-drive-controller-56v-with-heat-sink-optimizes-high-performance-brushless-motor-high-power-foc-bldc-based-on-odrive)
(~$60, 8–56 V) and a fixed motor with an idler and outboard bearing (as in the V5 ostrich). Its catch: the
5010's shaft (3 or 4 mm, sometimes not protruding at all) has to be measured before you trust it, and the
encoders mean magnets and extra wiring. **I recommend the H5055 + VESC.** Pick the light variant if the 3.6 kg
total is too heavy for the backpack.

**Rejected:** 70 mm hub motors. A 60–100 KV hub on 4S tops out at **6–10 mph**, and each weighs ~615 g
([Maytech 60KV, $72–74](https://michobby.com/products/in-stock-mto7052hbm-60-ha-maytech-70mm-60kv-hub-motor-450w)). They only
make sense on 6–10S.

## 4. Camera, aiming and sensing

| Part | V2 | Notes |
|---|---|---|
| GoPro HERO9 | Native recording, Wide, HyperSmooth on, on a foam-damped plate on the pan stage, mast 0.35–0.65 m, ~10° up-tilt | Start and stop recording with the GoPro remote or app (later over BLE from the Pi) |
| Pan servo (kept) | The servo you have; measure its travel. Optional: a 270° version of the same size for ±135° | The ESP32 already slews it. Add jerk-limited moves (S-curve) so pans look like a hand pan. Measure its µs-to-degree curve before setting `half_range_deg`. |
| Vision camera | Small wide-angle USB camera on the pan stage beside the GoPro, *estimate $25–40* | YOLOv8n at 10–11 det/s is already proven on the Pi |
| UWB AoA anchor | **On the pan stage**, facing where the camera faces | The older Makerfabs AoA board was specified at [±60°](https://github.com/Makerfabs/UWB-AOA-with-Display-STM32F103C8T6/tree/main) (Gen2 field of view not published; check on arrival). A fixed forward anchor would lose a skater who is 90–135° off the nose. Bearing = pan angle + AoA angle. Leave a slack loop in the cable. |
| 2D lidar | **LDROBOT LD19** (360°, 12 m, rated for 30 klux sunlight), ~[$69](https://sensorlidar.com/products/ldrobot-ld06-ld19-d300-of-laser-radar-lidar-scanner-360-scanning-d500-lidar-kit-support-ros1-and-ros2-for-indoor-and-outdoor), USB-UART to the Pi | Obstacle stop and slow-down in every direction (side and lead shots), and it can see the skater's legs and board up close. A cheaper forward-only option is the [TFmini-S 12 m](https://www.waveshare.com/product/robotics/drivers-sensors/lidar.htm) (~$40–50). |
| HC-SR04 (kept) | Nose, 0–4 m, read by the ESP32 | Independent backstop: the ESP32 stops the robot by itself even if the Pi hangs |
| IMU | BNO085 or ICM-42688 class on the ESP32's I²C, *estimate $10–25* | Yaw-rate loop, so the robot holds a line on camber and cracks |

## 5. Electronics and protocol changes

**ESP32 pins:** 2 LED · 5/18 HC-SR04 · **16/17 UART2 → VESC** · **21/22 I²C: IMU + AS5600 steering-angle sensor** · **26 steering servo** (was
right ESC) · 27 pan servo · 25 free.

**Who does what:**
- **Pi:** perception (UWB, vision, lidar), the shot planner and safety logic. It sends a speed and turn-rate
  request, not raw motor commands.
- **ESP32:** real-time control at 100 Hz:
  - turns (v, ω) into the two wheel RPMs and the steering angle;
  - corrects ω with the IMU's yaw rate;
  - sends RPM to the VESC over UART (the second motor via the dual board's CAN forward);
  - runs the pan servo trajectory, the HC-SR04 stop and the watchdogs.

Steering angle and wheel speeds come from the same IMU-corrected ω. At a standstill the front wheel swings to 90°
first, and only then do the wheels counter-rotate.

**Serial lines (additions to the current protocol):**

| Direction | Line | Meaning |
|---|---|---|
| Pi → ESP32 | `T,<v_mps>,<w_radps>` | Replaces `C` in V2. The ESP32 clamps it and rate-limits it (max accel/decel set on the ESP32). |
| ESP32 → Pi | `O,<v_left>,<v_right>,<yaw_rate>,<steer_deg>` at 50 Hz | Odometry and IMU for the planner |
| ESP32 → Pi | `S,...` | Unchanged, plus a VESC fault flag |

**Fail-safe:** on link loss, a stale `T` or an HC-SR04 stop, the ESP32 **brakes actively** (ramps RPM to 0 at
~0.4 g), centres the steering and holds the pan.
- Separately, set the VESC's own UART timeout so it brakes if the ESP32 itself dies.
- Allow regen into the LiPo (VESC battery current limits), with a fuse and a master switch as before.

**Pi software (new):**
- **`skater_state`:** fuses UWB, the vision bearing, robot odometry and the lidar into the skater's position
  and **velocity** in a world frame.
- **`shot_planner`:** places the robot's goal relative to the skater (side-follow default, lead, trail), using
  the skater's velocity as a feed-forward so the robot matches speed instead of chasing.
- The safety gate gets lidar zones. Follow speed is capped by clear distance d:
  v ≤ −a·t + √((a·t)² + 2·a·(d − 0.5)), with a = 0.4 g and t = 0.25 s reaction time.

The existing follower, gate, UWB, camera and aim packages carry over; `esp32_bridge` gains `T`/`O`.

## 6. Bill of materials (V2 driving robot)

| Item | Qty | Price | Source |
|---|---|---|---|
| Flipsky H5055 200KV sensored motor, 8 mm shaft | 2 | $43–63 ea | [Flipsky](https://flipsky.net/products/h5055-motor), [Flipsky 5055](https://flipsky.net/products/lipsky-sensored-outrunner-brushless-dc-motor-battle-hardened-5055-200kv-1500w) |
| Flipsky Mini Dual FSESC 4.20 (VESC) | 1 | ~$106 | [listing](https://flipskycn.en.made-in-china.com/product/qRNYcnAPskVG/China-Flipsky-Mini-Dual-ESC-4-20-100A-Drive-with-Vesc-Technology-Aluminum-Heatsink.html) |
| Motor pulleys (8 mm bore) + wheel pulleys/cores + 2 belts | — | *estimate $30–45* | e-skate parts shops |
| 90 mm longboard wheels, 4-pack | 1 | $20–42 | as V1 (`robot_design.md` §4) |
| Steering servo (standard size, metal gear, ≥ 20 kg·cm, ≥ 180°) + front fork bearing and axle | 1 | *estimate $20–30* | |
| LDROBOT LD19 lidar | 1 | ~$69 | [listing](https://sensorlidar.com/products/ldrobot-ld06-ld19-d300-of-laser-radar-lidar-scanner-360-scanning-d500-lidar-kit-support-ros1-and-ros2-for-indoor-and-outdoor) |
| IMU (BNO085 / ICM-42688 class) | 1 | *estimate $10–25* | |
| Wide USB camera for vision | 1 | *estimate $25–40* | |
| 5 V 5 A buck (Pi), 6 V 3 A BEC (servos), fuse, switch, wiring, M8 axles, bearings, fasteners, inserts | — | *estimate $50–60* | as V1 |
| **Robot total** | | **≈ $415–545** | V1 was $150–260 (no feedback, no lidar, no second camera) |
| Optional: 270° pan servo, standard size (front-3/4 lead shot) | 1 | *estimate $15–20* | |
| Battery 4S 3000–4000 mAh + balance charger | | *estimate $40–60* + $30–60 | charger as V1 |
| UWB AoA anchor + tag | | $69.60 | `hardware_plan.md` |

## 7. First bench test (de-risks the drivetrain before any frame is printed)

1. One H5055 + the dual VESC on a 3S/4S pack. In VESC Tool run motor detection (sensored FOC) and set current
   limits low (10 A motor, 5 A battery).
   *Pass:* it detects the halls and spins smoothly from 0 rpm both ways.
2. One belt and one wheel on a printed test bracket. Drive RPM from the ESP32 over UART: hold 0.3, 1, 3 and
   6.7 m/s wheel-surface speed.
   *Pass:* speed holds within 3 %, starts from rest are smooth, the wheel stops within the set ramp, and RPM
   reads back correctly. Watch the slowest speeds: if the motor has 7 pole pairs (typical for a 5055; check),
   0.3 m/s is only ~800 ERPM. That's around the VESC's minimum for speed control (`s_pid_min_erpm`, check the
   value in VESC Tool), so creeping may need current control instead.
3. Unplug the ESP32's UART while the wheel spins.
   *Pass:* the VESC timeout brakes it to a stop.
4. Then the steering fork with the servo on the test frame: δ = 0/±45/±90°, and check the turn-in-place geometry.
5. The LD19 outdoors in full sun: range to a person and to a curb, from 2 to 12 m.
   *Pass:* sets the speed cap (§5) from the range it actually reaches.

Only then: frame, body, mast and the field tests (walk → jog → push → full speed, throttle capped on the ESP32).

## 7b. CAD (2026-11-15): `cad/honebot_v2/`
`honebot_v2.py` builds all of this in Fusion 360. Its review (`checks/check_v2.py`) changed a few things above:
- **Steering angle sensor.** The GT2 steering belt can skip if the front wheel hits something, which would
  leave the servo and the wheel disagreeing. An **AS5600** reads a magnet in the stem bolt, and the ESP32
  stops on a mismatch.
- **Steering hardware.** The stem has an **M8 bolt core**: printed alone it would see ~27 MPa across its layers
  in a 3 g wheel strike. Steering pulleys are 48T GT2 with a 280 mm belt.
- **Acceleration caps in hardware.** The front wheel lifts at 0.47 g with the mast up. The VESC current limits
  (≈ 4 A motor, ≈ −4 A brake per motor) cap acceleration at ~0.3 g and braking at ~0.44 g, on top of the ESP32 ramps.
- **Rear axles.** M8 bolts go in from outside the wheels into nuts captured in the chassis walls.
- **Motors.** Each sits in a 3 mm pocket in the wall to gain shaft length for the pulley hub. MEASURE the motor.
- **Bolt-on nose.** The nose and steering head are a separate part, because the tub and nose together don't
  fit the H2C bed.
- **Mass and size.** 3.5 kg, 393 × 308 mm. Lens at 0.43–0.50 m.

## 8. Still open
- H5055 weight, mounting pattern and drawing; the 8 mm-bore pulley and belt sizes to match.
- The kept pan servo's measured travel (decides whether the 270° upgrade is worth it). Field of view of the UWB AoA Gen2 anchor.
- Backpack inner dimensions → confirm the ~39 × 31 × 21 cm folded size.
- Does the GoPro footage from a 0.5 m mast look right? Try a few heights by hand with the GoPro before printing the mast.
