# honebot chassis CAD (Fusion 360 script) — design, checks, build notes (2026-11-16)

Files:
- `cad/fusion/HonebotChassis/HonebotChassis.py` (+ `.manifest`): **Fusion 360 script that builds the whole robot**
  (20 printed parts, reference hardware, keep-out volumes). It is the single geometry source.
- `cad/check_design.py`: offline checker. It imports the same file without Fusion and checks clearances, holes,
  assembly paths, the GoPro view, mass/CG, braking, structure, the sensor beam, wiring and printability. Output:
  `cad/report/check_report.md` + `views.png` + `camera_views.png`.
- `cad/tests/test_fusion_script.py`: runs the Fusion code path against a stand-in `adsk` API (Python errors, bad
  geometry such as zero-length cylinders or non-orthogonal boxes).

> **Not yet run inside real Fusion 360.** It uses only well-documented API calls (TemporaryBRepManager boxes,
> cylinders, booleans; `bRepBodies.add` in a direct-modelling design) and wraps every part in its own try/except,
> so a problem skips one part and is named in the final message box instead of stopping the script.
> **First run:** Fusion → Utilities → Add-Ins → Scripts and Add-Ins → "+" next to *My Scripts* → pick the
> `cad/fusion/HonebotChassis` folder → select *HonebotChassis* → Run. Expect a new document and a message
> "built 66 of 66 parts". If any part fails, send the message text back.
> If the model looks like it is lying on its back, your Fusion uses "Y up": the script models Z up (x forward, y left).

![views](../cad/report/views.png)

## 1. What changed from `robot_design.md` and why

| Topic | Before (2026-11-13 plan) | Now | Why |
|---|---|---|---|
| Wheel arrangement | Driven wheels **rear**, caster front | Driven wheels **front**, caster trails at the rear | Braking. Weight shifts forward when braking, onto the wheels that brake, and front-wheel braking is directionally stable (a rear-braked robot spins when its wheels slide). From 15 mph the checker gives **5.4 m vs 6.4 m** (good grip, μ 0.8) and **5.5 m vs 7.7 m** (dusty, μ 0.6). With the recommended 0.5 g software cap both layouts need ~6.3 m on good grip; front drive keeps that on poor grip, rear drive would not. Steering is unaffected (`base_link` = axle centre). |
| Motors | generic 5010 360KV + GT2 belt, pulley on the shaft | **Reference motor MAD 5010 EEE V2.0 370KV** (published size, base and prop holes; "EEE" = the maker's weather-resistant industrial line). **Printed 24T HTD-3M pulley bolted to the bell's prop holes** → printed 48T on the wheel (2:1), 231 mm belt. Motor sits on a **replaceable round plate** with a universal slotted pattern | Generic 5010 listings disagree on pattern, shaft and weight, and none showed the ~25 mm of bare shaft a shaft-mounted pulley would need here. Bolting to the bell needs no shaft length. HTD-3M's round tooth prints better and skips less than GT2. |
| Axle | One M8 × 300 rod through the frame | Two **M8 × 140 stub axles (12.9)**, head locked in a pocket in the boss, nyloc outside | Leaves the centre free for a battery tunnel, and each wheel comes off with one nut. |
| Battery | Strapped on the plate | **Tunnel in the floor**, loaded from the front behind a bumper-door; two prongs hold it against the rear wall | Low CG, no strap, charging needs no tools (2 thumb screws). |
| Camera | Damped mast above the axle | **Removable mast at the nose**, standard servo pan head, GoPro lens ahead of everything | With the lens ahead of the robot and high enough, nothing of the robot is in frame at ±45° pan, even in Wide 4:3 (see §3). |
| Frame | One 330 × 260 plate | **Tub + tail module + lids + pods**, every part ≤ 222 mm | The 330 mm plate did not fit the 325 mm bed reliably; the tub (a box section) is much stiffer than a plate. |
| Size / mass | ≈ 350 × 280 × 140 mm, ≈ 2.5 kg | **396 × 348 × 147 mm** (mast off), 288 mm tall with the mast, **≈ 3.1 kg** (estimate) | Front drive needs the CG well behind the axle (pitch-over), which needs length. Wheels off → 396 × 238 × 147 mm. **Measure your backpack** against this. |

### Drivetrain options checked (checker §6; 3S nominal, 90 mm wheels, needs ≥ 6.5 N at 15 mph)

| Option | Top speed (no load) | Thrust at 15 mph | Motor mass | Verdict |
|---|---|---|---|---|
| **5010 370KV + 24T:48T HTD-3M** (chosen) | 21.6 mph | 52 N (48 N on a sagged pack) | ≈ 0.30 kg | OK, wide margin (software caps the speed). Low-speed weakness handled by AM32 sine start-up (see below) |
| 5010 370KV + 20T:48T, pulley on a long shaft | 18.0 mph | 35 N | ≈ 0.30 kg | OK, but needs a motor with ~25 mm of free shaft |
| goBILDA 5203 3.7:1 planetary (brushed), direct | 15.8 mph | 1.1 N | 0.80 kg | Fails: no torque left near 15 mph, ~19 W peak per motor |
| same with 107 mm wheels | 18.8 mph | 3.7 N | 0.80 kg | Fails |
| Hoverboard hub motors | ~10–12 mph | high | ~5–6 kg | Too heavy and too slow for a backpack robot |
| e-skate 5055/5065 sensored + VESC | > 20 mph | high | ~1.2 kg | Best low-speed control, but ~$200+ in controllers and twice the weight |
| Small servos / actuators | ~1 m/s | — | — | Not suitable for continuous wheel drive |

Longevity measures for the chosen drive:
- **Pod caps** over the motors' upper half and the motor pulleys. They keep off spray from above; they are open
  toward the wheel, so grit thrown by the wheel can still reach the lower half.
- The weather-resistant reference motor and sealed 608 bearings.
- Belt slots for tensioning.
- AM32 **sine start-up** (a slow-speed mode for crawlers, part of the AM32 firmware) for smooth starts.

If start-up still stutters under load, the motor plate and pulley are the only parts to change for a sensored motor later.

**Speed vs ratio** (reprint the pulleys; change `pul_m_t`, `pul_w_t` and `belt_len`, then re-run the checker):
24T:48T ≈ 21.6 mph no-load (chosen; plenty of thrust and headroom, cap the speed in software), 24T:60T ≈ 17.3 mph.

## 2. Layout (mm; x forward from the drive axle, y left, z up from the ground)

| Item | Where | Notes |
|---|---|---|
| Drive wheels 90 × 52 | centre (0, ±138.5, 45) | track 277 mm |
| Motors (≤ Ø62 × 33) | centre (−35, ±62…95, 94), outside the tub walls | 24T pulley bolted to the bell, belt plane y = ±102; **HTD-3M 231 mm (77T)** belt → 60.4 mm centres |
| Caster 70 × 51 | pivot x = −223, wheel trails 25 mm | wheelbase 248 mm; swept circle r 65 mm is a checked keep-out |
| Battery tunnel | x −141…4, |y| ≤ 26.5, z 32…62 | 3S 5000 hard case ≤ 145 × 50 × 30 |
| Deck (removable) | z 79…82 | Pi 5 (USB end to the rear), two regulators beside the mast socket, 10 mm M3 grid |
| Tail deck | z 115 | ESP32 + SunFounder board (on 10 mm standoffs over the pivot nut), 2 ESCs, fuse + distribution, XT60 arming plug in the rear wall |
| HC-SR04 "eyes" | front, z 102, tilted up 12° | ground return at 1.95 m (> `slow_distance` 1.0 m) |
| Mast + pan head | x 38; servo seat z 195 | GoPro centre z 260, lens z 266 / x 60, ahead of the bumper line |
| UWB anchor | plate on 4 inserts in the pan-head front, z 151…192 | antenna faces forward, above the robot body; USB to the Pi (`uwb_node`, `/dev/ttyUSB1`) |

Power below the deck, signals above it. The battery lead and ESC phase wires run in the side channels (|y| 30…58,
under the deck, through Ø12 holes in the axle bridge beam), then up through the rear-corner deck slots and through
the rear-wall window to the tail. Signal cables (GoPro USB-C and servo + UWB inside the hollow mast, ESP32 USB,
HC-SR04) run over the deck. The mast has an **open rear channel** (and the socket an open rear slot): mast cables
are laid in, not threaded, and lift out with the mast. Servo and UWB cables drop from under the pan head into the
channel; the GoPro USB-C makes a service loop around the pan axis. Every route ends at its real connector,
including the regulator inputs from the tail fuse and the servo's 6 V. Lengths are in the report §9: cut about 30 % longer.

## 3. Checks (all pass; full numbers in `cad/report/check_report.md`)

- **Clearances**: every non-mated pair; 0.5 mm static, 3 mm against anything moving or swept: wheels, pulleys,
  belts, caster swivel, GoPro pan sweep, USB plug space, **hex-key access to every motor-plate bolt**. Mated pairs
  must not interpenetrate (> 0.3 mm). Seven simplified-hardware overlaps are allowed explicitly, e.g. bolts through
  solid wheels.
- **Holes**: every hardware hole runs through its part for the required depth (insert length, or 2 mm) with
  ≥ max(1.5 mm, d/2, ≤ 3 mm) of wall around it, and ≥ 1.5 mm between neighbouring holes.
- **Assembly / service**: mast, lids, deck (with Pi), bumper-door, battery, pods, wheels and motors each slide out
  along their exit direction without hitting anything that stays.
- **Camera**: the robot was sampled every 3 mm and projected into the GoPro frame for pan −45…+45°. It was tested
  in Wide 4:3 / 16:9, SuperView and Linear, with a 5° margin, under both a rectilinear and a fisheye lens model.
  **0 points in frame** in all modes. The camera can go down to a lens height of ~226 mm before the robot shows.
  Allowed down-tilt: **16° in Wide 4:3**, ≥ 30° in the 16:9 and Linear modes (tilting up is always fine).
  ![camera](../cad/report/camera_views.png)
- **Stability**: 77 % of the weight on the drive wheels. Sideways tip needs 0.84 g; urethane slides first.
  Forward pitch-over starts at **0.62 g**. **Bumper skids** (16 mm off the ground) catch the nose after 10.8° of
  pitch, so a full-brake stop makes it skid on the bumper instead of flipping. Still do the firmware change in §8.
  Trade-off: the skids leave 16 mm of clearance 39 mm ahead of the wheels, about a 22° approach angle. They clear
  kerb ramps but may scrape a steep driveway lip.
- **Structure**:
  - **M8 stub axle:** safety factor 3.9 (class 8.8) / 6.5 (12.9) for a 4 g bump; 1.8 / 3.0 for a 10 g kerb strike. Use **12.9**.
  - **Axle boss bearing:** factor > 10.
  - **Caster bearings and fork:** factor > 20.
  - **Belt:** pulls about 26 N at the traction limit.
  - **Mast:** first bending mode ≈ 118 Hz (ideal clamp; the flange joint lowers it, target > 30 Hz).
- **Printability**: every printed part fits 300 mm.

## 4. Printing

| Part | Qty | Material | Orientation / supports |
|---|---|---|---|
| tub | 1 | PAHT-CF, 5–6 walls, 40 % infill, dry filament | Floor on the bed. Supports: under the axle bridge beam (116 mm span), top of the battery opening and of the rear tail window. Heat-set inserts after printing. |
| tail | 1 | PAHT-CF | Rear wall vertical, skirt down; tree supports under the deck plate and pivot boss. |
| motor_plate_L/R | 2 | PAHT-CF | Flat, counterbores up |
| wheel_pulley_L/R, motor_pulley_L/R | 2 + 2 | PAHT-CF, 0.4 nozzle, 0.12 mm layers | Axis vertical. **Tooth form is approximate** (round 1.14 mm grooves). For best belt life, regenerate the 48T/24T HTD-3M tooth profile with a pulley generator (e.g. the OpenSCAD "Parametric Pulley" by droftarts, HTD 3 mm) and keep these parts' hubs, flanges and bolt holes. Motor pulley: drill the centre bore to fit the shaft (5 mm on the MAD motor, 10 mm deep). |
| caster_fork | 1 | PAHT-CF | Crown on the bed |
| mast | 1 | PAHT-CF | Lying on its side; supports under the flange and pan head |
| deck | 1 | PETG (or PAHT-CF) | Flat |
| bumper_door | 1 | PETG (TPU skin optional) | Back face on the bed (prongs up) |
| pod_L/R, lid, tail_lid | 2+1+1 | PETG, bright colour | Pods: wall-side rim on the bed. Lids: flat face down. |
| us_bracket, uwb_plate, pan_buckle | 1 each | PETG | Back face / disc down |
| servo9g_adapter | 1 | PETG | Bench only (hidden in Fusion) |

Tolerances are deliberately loose: M3/M4/M8 through-holes 3.4/4.5/8.5 mm, M3/M4 heat-set holes 4.0/5.6 mm (check
your insert's chart), 608 seats 22.2 mm (+ a drop of retaining compound), 0.3 mm per side slip fits, ≥ 3 mm next to
anything that moves. Change any of these in `PARAMS` and re-run both scripts.

## 5. Data quality: what is verified and what to measure

**Verified (fixed holes/pockets):** Raspberry Pi 5 85 × 56, M2.5 on 58 × 49 (3.5 mm from the non-USB end) ·
GoPro HERO9 71.0 × 55.0 × 33.6 mm, 158 g · Pololu D36V50Fx 25.4 × 25.4 × 9.5 mm · MG996R body 40.7 × 19.7 × 42.9,
ears 54 mm · HTD-3M pitch diameters · goBILDA 5203 data (used only to reject it).

**Measure before printing (you own these):**
1. HC-SR04: transducer centre distance (model 26 mm, windows Ø18) and PCB outline. Also the beam test below.
2. SunFounder ESP32 extension board: outline, hole positions, height with the ESP32 plugged in. It goes on the tail
   deck's 10 mm grid. Zone available: 125 × 60 × 12 mm on 10 mm standoffs.
3. 9 g servo: body length/width, ear hole spacing (adapter slots cover 26–30 mm), shaft offset from the body centre
   (`sg90_shaft_off`, 5.9 mm).
4. GoPro lens centre relative to the body (`gopro_lens_dy/dz`). It only shifts the view check slightly.

**Measure on arrival (then update PARAMS and re-run):** motor: body length (≤ 33 mm), prop holes (pulley needs 4 × M3
on D12; MAD 5010 EEE lists D12 and D18), shaft stub above the bell (≤ 9 mm, else drill the pulley through), M3 thread
depth in the bell (screw length); ESC size (≤ 48 × 28 × 14), wheel bearing inset (sets
the spacer length) and core material at Ø33 for the 4 pulley bolts, battery size (≤ 145 × 50 × 30),
UWB anchor board size/holes (plate slots fit ≤ 50 mm wide), standard-servo ear height (`servo_ear_z`).

## 6. Bill of materials (hardware not printed)

| Item | Qty | Notes |
|---|---|---|
| **MAD 5010 EEE V2.0 370KV** (reference), or any 5010-class 300–400 KV motor ≤ Ø62 × 33 with M3 base holes on 16–30 mm and **4 × M3 prop holes on D12** | 2 | Price not retrieved — check (industrial line, likely more than generic $20 motors). Generic 5010s fit the plate, but only use one with D12 prop holes. |
| M3 × 12 low-head (DIN 7984) screws | 8 | motor pulley → bell |
| HTD-3M **231 mm (77T)** 9 mm closed belt | 2 (+1 spare) | 228 or 234 mm also fit the ±4 mm slots |
| AM32 ESC 30–40 A, 2–4S, PWM input, bidirectional | 2 | + USB linker. Set: bidirectional, sine start-up on, LVC 3.3 V/cell |
| 90 × 52 longboard wheels (4-pack) + one 70 × ~51 wheel for the caster | 1 + 1 | Core must take 4 × M3 on Ø33 |
| 608-2RS bearings | 2 | caster swivel (the wheels bring their own) |
| M8 × 140 hex bolt **12.9** + washer + nyloc | 2 | stub axles |
| M8 × 70 (caster pivot) + M8 × 80 (caster axle) + nylocs, PTFE + spring washer | 1 each | friction washer under the pivot nut damps shimmy |
| Al tube 12 × 8.2 mm, ~50 mm | 2 | spacers, cut to measured length |
| M3 × 80 bolts + nylocs | 8 | wheel pulley through the core (stack ≈ 69.5 mm + washer + nut) |
| Heat-set inserts M3 × 5.7 (~25), M4 × 8.1 (~12) | | |
| M3/M4 screws, 2 × M4 thumb screws (mast), 2 × M4 thumb screws (door) | | |
| Pololu D36V50F5 (5 V 5.5 A, Pi) + D36V50F6 (6 V, servo) | 1 + 1 | zip-tied on the deck slots |
| XT60 pigtails, 20 A blade fuse holder, panel XT60 used as an arming plug | | pull the plug = everything off |
| MG996R or DS3218 servo, 470–1000 µF cap | 1 | |
| Right-angle USB-A cables (USB-C for the GoPro, micro-USB for the ESP32) | 2 | 25 mm plug space behind the Pi |
| 3S 5000 mAh hard case ≤ 145 × 50 × 30 | 1 | |

## 7. Assembly order

1. Inserts into tub, tail, motor plates, mast. 608s into the tail boss.
2. Motor → plate (screw heads in the counterbores); 24T pulley → bell (4 × M3 low head) → plate into the wall
   pocket, 3 × M3 from inside, loose.
3. Stub axle from inside (head in its pocket) → spacer → wheel with pulley bolted on → washer, nyloc.
4. Belt on, slide the motor plate up along its slots to tension (belt deflects ~3 mm under a finger), tighten.
5. Tail onto the rear posts (4 × M4 sideways + 2 × M4 through the lip). Caster fork + pivot bolt + friction washer.
6. Wiring in the side channels, deck with Pi/regulators in, lid, mast (2 thumb screws), GoPro.
7. Battery in from the front, lead end last (the XT60 stays at the door, then is tucked up past the prongs), bumper-door
   on, 2 thumb screws.

## 8. Tests before and after building

- **HC-SR04 beam (now, no printing):** hold the sensor at 100 mm height tilted 12° up on an empty floor or
  pavement, with `ros2 launch honebot_hardware esp32.launch.py` running. `/ultrasonic/range` should read `inf` or
  more than 1.9 m. If it reads 0.5–1.5 m, it sees the ground: tilt more (`us_tilt_deg`).
- **GoPro view (after printing the mast):** Wide 4:3, `camera.launch.py aim:=false`, then
  `ros2 topic pub /camera/pan_cmd std_msgs/Float32 "data: 1.0"` and `-1.0`. Look at `/camera/image/compressed` in
  Foxglove: no robot part may be in frame.
- **Shimmy:** at walking pace and then jogging, push on the tail. If the caster oscillates, tighten the pivot
  friction washer one flat at a time.
- **Firmware follow-up (safety):** the ESP32 currently drops to neutral instantly on fail-safe. With AM32
  "brake on stop" that is a full-strength stop, above the 0.62 g pitch-over limit. Ramp the speed command to
  zero at about 0.5 g instead (≈ 0.7 of full throttle per second at 15 mph). If your AM32 build has a brake
  strength setting, lower it too. The bumper skids are the backstop, not the plan.

## Prior art consulted

A short search found no published build that combines UWB following, outrunners with bell-mounted pulleys and a
printed differential drive. What informed this design:
- Makerfabs' UWB follow-me robot platform (in `hardware_plan.md`): one AoA anchor in front, following on range and bearing.
- A Fab Academy belt-driven differential transport robot: wheels with integrated printed pulleys.
- University capstone write-ups of UWB "follow-me wagons": tag on the user, anchors on the cart.
- Standard DIY e-skate practice: a wheel pulley bolted through the urethane wheel core, slotted motor mounts for
  belt tension.
- Standard RC-crawler practice: ESC sine start-up for low-speed control, an arming plug as the main switch.

## Sources (retrieved 2026-11-16)

- goBILDA 5203 3.7:1 data: [Abra Electronics](https://abra-electronics.com/robotics-actobotics-servocity/motion-en/dc-gear-motors-brushed/5203-series-yellow-jacket-planetary-gear-motors/5203-2402-0003-5203-series-yellow-jacket-planetary-gear-motor-3.7-1-ratio-24mm-length-8mm-rex-shaft-1620-rpm-3.3-5v-encoder-gobilda.html), [Core Electronics](https://core-electronics.com.au/5203-series-yellow-jacket-planetary-gear-motor-371-ratio-24mm-length-8mm-rex-shaft-1620-rpm-33-5v-encoder.html), [RobotShop](https://www.robotshop.com/products/5203-series-yellow-jacket-planetary-gear-motor-2691-223-rpm-33-5v-encoder)
- MAD 5010 EEE V2.0 (KV options incl. 370, O56 x 32.7, 5 mm shaft, base D25/D30 M3, prop D12/D18 M3): [mybotshop](https://www.mybotshop.de/MAD-5010-EEE-V20-brushless-drone-motor_1), [rcdrone.top](https://rcdrone.top/products/src-mad-467-5010-eee-v2-0-brushless-drone-motor), [MAD Components USA](https://www.madcomponentsusa.com/5010-eee-v2-0-drone-motor.html)
- Prior art: [Fab Academy belt-driven robot](https://fabacademy.org/2025/labs/bottrop/students/annasophia-brokuf/final-project/), [UIUC ECE445 UWB follow wagon](https://courses.grainger.illinois.edu/ece445/getfile.asp?id=24773)
- 5010 360KV variation between sellers: [RCTimer](https://www.rctimer.com/rctimer-5010-360kv-multicopter-brushless-motor-p0233.html), [Zbotic](https://zbotic.in/product/5010-360kv-high-torque-brushless-motor-for-drone/), [Amovrobot](https://amovrobotlab.com/products/5010-360kv-750kv-brushless-motor-motors); T-Motor MN5008: [GetFPV](https://www.getfpv.com/motors/commercial-cinematography-motors/t-motor-antigravity-mn5008-motor.html)
- AM32 (sine start-up, configurator): [am32-firmware/AM32](https://github.com/am32-firmware/AM32)
- Hobbywing QuicRun 1060 (brushed ESC, considered): [Hobbywing](https://www.hobbywingdirect.com/products/quicrun-10-esc-2-3s-brushed)
- Pololu D36V50F5: [pololu.com/product/4091](https://www.pololu.com/product/4091)
- GoPro HERO9 size/weight: [Willoughby's](https://www.willoughbys.com/GoPro_HERO9_Black_449200.html), [CameraSpecs](https://www.cameraspecs.co.uk/cameras/gopro/hero/gopro-hero9-black-682.html)
- MG996R: [Core Electronics](https://core-electronics.com.au/mg996r-metal-gear-servo-11-kg-cm.html); SG90: [Kpower](https://www.kpower.com/insight_servo/2399.html)
- Raspberry Pi hole template: [raspberrypi-spy](https://raspberrypi-spy.co.uk/?p=826)
- HC-SR04 bracket hole data (varies): [ProtoSupplies](https://protosupplies.com/product/hcsr04-ultrasonic-range-finder-module-bracket/)
- GoPro FOV figures used with a 5° margin are GoPro's published HERO9 values from memory (not re-retrieved); the
  live-preview test in §8 is the real check.
