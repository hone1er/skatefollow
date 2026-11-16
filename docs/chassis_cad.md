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
> "built 68 of 68 parts" (owner confirmed the earlier 66-part version runs). If any part fails, send the message text back.
> Rounded corners use `TemporaryBRepManager.createSphere`; if a Fusion version lacks it, those corners are skipped and the
> message box says how many.
> If the model looks like it is lying on its back, your Fusion uses "Y up": the script models Z up (x forward, y left).

![views](../cad/report/views.png)

## 1. What changed from `robot_design.md` and why

| Topic | Before (2026-11-13 plan) | Now | Why |
|---|---|---|---|
| Wheel arrangement | Driven wheels **rear**, caster front | Driven wheels **front**, caster trails at the rear | Braking. Weight shifts forward when braking, onto the wheels that brake, and front-wheel braking is directionally stable (a rear-braked robot spins when its wheels slide). From 15 mph the checker gives **5.4 m vs 6.4 m** (good grip, μ 0.8) and **5.5 m vs 7.7 m** (dusty, μ 0.6). With the recommended 0.5 g software cap both layouts need ~6.3 m on good grip; front drive keeps that on poor grip, rear drive would not. Steering is unaffected (`base_link` = axle centre). |
| Motors | generic 5010 360KV + GT2 belt, pulley on the shaft | **Reference motor MAD 5010 EEE V2.0 370KV** (published size, base and prop holes; "EEE" = the maker's weather-resistant industrial line). **Printed 28T HTD-3M pulley bolted to the bell's prop holes** → printed 56T on the wheel (2:1), **258 mm (86T) belt**. Motor sits on a **replaceable round plate** with a universal slotted pattern; a **pull screw** gives belt tension a positive stop | Generic 5010 listings disagree on pattern, shaft and weight, and none showed the ~25 mm of bare shaft a shaft-mounted pulley would need here. Bolting to the bell needs no shaft length. HTD-3M's round tooth prints better and skips less than GT2. |
| Axle | One M8 × 300 rod through the frame | Two **M8 × 150 stub axles (12.9)** inside an **18 × 8.2 mm 7075 sleeve** that runs from the bolt head (locked in a pocket in the boss) to the wheel's inner race; nyloc outside | Leaves the centre free for a battery tunnel; each wheel comes off with one nut. The sleeve makes the clamp all-metal and carries the bending at the wall. |
| Battery | Strapped on the plate | **Tunnel in the floor**, loaded from the front behind a bumper-door; two prongs hold it against the rear wall | Low CG, no strap, charging needs no tools (2 thumb screws). |
| Camera | Damped mast above the axle | **Removable mast at the nose**, standard servo pan head, GoPro lens ahead of everything | With the lens ahead of the robot and high enough, nothing of the robot is in frame at ±45° pan, even in Wide 4:3 (see §3). |
| Frame | One 330 × 260 plate | **Tub + tail module + lids + pods**, every part ≤ 222 mm | The 330 mm plate did not fit the 325 mm bed reliably; the tub (a box section) is much stiffer than a plate. |
| Size / mass | ≈ 350 × 280 × 140 mm, ≈ 2.5 kg | **399 × 348 × 173 mm** (mast off; the tail hood is the top), 288 mm tall with the mast, **≈ 3.4 kg** (estimate) | Front drive needs the CG well behind the axle (pitch-over), which needs length. The 2026-11-16 strength review added the axle sleeves, the caster post and bearings, and thicker tail walls (+0.3 kg). Wheels off → 399 × 238 × 173 mm. **Measure your backpack** against this. |

### Drivetrain options checked (checker §6; 3S nominal, 90 mm wheels, needs ≥ 6.5 N at 15 mph)

| Option | Top speed (no load) | Thrust at 15 mph | Motor mass | Verdict |
|---|---|---|---|---|
| **5010 370KV + 28T:56T HTD-3M** (chosen) | 21.6 mph | 52 N (48 N on a sagged pack) | ≈ 0.30 kg | OK, wide margin (software caps the speed). Low-speed weakness handled by AM32 sine start-up (see below) |
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
28T:56T ≈ 21.6 mph no-load (chosen; plenty of thrust and headroom, cap the speed in software). The 28T motor pulley
(not 24T) keeps ~3 mm of plastic between the screw counterbores and the tooth roots.

**Belts and pulleys.** Each belt is modelled as a true loop: two tangent runs plus the wrap arcs, 2 mm thick over the
tooth tips. The checker tests it against everything around it. The 28T pulley has 10–11 teeth in mesh.
**Tension:** press the belt at mid-span with about **2 N** (a 200 g weight); it should move **~1 mm** (span/64,
≈ 30 N per strand). Then tighten the pull screw against its boss, then the three plate screws.

**Motor shaft support.** The pulley is bolted straight to the bell, which keeps the overhang on the 5 mm shaft as
short as possible (~14 mm). At full pull the shaft sees ~68 MPa, a safety factor of 3.7 against fatigue. An extra
outboard bearing was evaluated and rejected: through a printed bracket it would be <1/7 as stiff as the shaft, so it
would carry <10 % of the load and could add stress through misalignment. The protections are the short overhang, the
tension spec above, and the pull-screw stop, which keeps tension from creeping up.

## 2. Layout (mm; x forward from the drive axle, y left, z up from the ground)

| Item | Where | Notes |
|---|---|---|
| Drive wheels 90 × 52 | centre (0, ±138.5, 45) | track 277 mm |
| Motors (≤ Ø62 × 33) | centre (−40, ±62…95, 96), outside the tub walls | 28T pulley bolted to the bell, belt plane y = ±102; **HTD-3M 258 mm (86T)** belt → 64.6 mm centres |
| Caster 70 × 51 | pivot x = −223, wheel trails 25 mm | wheelbase 248 mm; swept circle r 65 mm is a checked keep-out. Swivels on 2 × 6001-2RS 58 mm apart in a post; M12 12.9 pivot |
| Battery tunnel | x −141…4, |y| ≤ 26.5, z 32…62 | 3S 5000 hard case ≤ 145 × 50 × 30 |
| Deck (removable) | z 79…82 | Pi 5 (USB end to the rear), two regulators beside the mast socket, 10 mm M3 grid |
| Tail deck | z 115, under a raised PETG hood | **ESP32 in a printed slide-in cradle** (PCB in grooves 34 mm above the deck so the jumper plugs on its downward pins clear; USB end toward +y; plate has a 10 mm grid for the SunFounder board instead), 2 ESCs, fuse + distribution, XT60 arming plug in the rear wall |
| HC-SR04 "eyes" | front, z 102, tilted up 12°, **mounted upside down** | ground return at 1.95 m (> `slow_distance` 1.0 m). Upside down, the 4-pin header points up, so the plug and wires rise behind the bracket (modelled keep-out) into the wall hole at z 131. The beam cone was checked clear of the brow, cheeks and bumper |
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

- **Clearances**: every non-mated pair is checked: 0.5 mm static, and 3 mm against anything moving or swept:
  - wheels, pulleys, belt loops
  - caster swivel, GoPro pan sweep
  - USB, ESP32 jumper and HC-SR04 plug space
  - hex-key access to every motor-plate bolt

  Mated pairs must not interpenetrate (> 0.3 mm), except a few declared simplifications (bolts through solid wheels).
- **Holes**: every hole runs through its part for the required depth with ≥ max(1.5 mm, d/2, ≤ 3 mm) of wall, and
  ≥ 1.5 mm surface-to-surface between neighbours. **Fastener schedule:** report §11. Every thread in a printed part
  is a heat-set insert, never tapped plastic.
- **Assembly / service**: these slide out along their exit direction without hitting anything that stays:
  - mast, lid, hood, deck (with Pi)
  - bumper-door, battery
  - pods, wheels, motors
- **Camera**: robot sampled every 3 mm and projected into the GoPro frame for pan −45…+45°. Modes: Wide 4:3/16:9,
  SuperView and Linear, 5° margin, rectilinear and fisheye models. **0 points in frame.** Allowed down-tilt: **16° in
  Wide 4:3**, ≥ 30° in the other modes.
  ![camera](../cad/report/camera_views.png)
- **Street → driveway** (assumed profile: up to 20° slope change, lips up to 10 mm). The skids of the previous
  version (11° approach) are gone; the rounded bumper nose now sets the approach angle.

  | Angle | Clears |
  |---|---|
  | Approach | 21° |
  | Breakover | 26° |
  | Gutter | 23° |
  | Departure | 45° |

  Floor 28 mm, caster legs 22 mm. A driven 90 mm wheel climbs a ~10 mm lip from standstill. Treat lips as slow-down
  events.
- **Stability**: 74 % of the weight is on the drive wheels.
  - **Sideways tip:** needs 0.79 g. That is close to the sliding limit, so the follower should cap lateral
    acceleration (~0.5 g: v²/R).
  - **Forward pitch-over:** starts at **0.71 g**. If it pitches, the rounded bumper touches down at 22°, before the
    35° tip point, so it can't flip.
  - Do the firmware decel-ramp change in §8.
- **Structure at the worst road event.** The event is a wheel hitting a 6 mm crack at 15 mph: ~920 N on a drive
  wheel, ~840 N on the caster. It uses an assumed urethane contact stiffness of 50 N/mm; the 10 mm lip case at 3 m/s
  is lower. Every item has a safety factor ≥ 1.5:

  | Item | Safety factor |
  |---|---|
  | Axle sleeve | 3.8 |
  | M8 at the root | 8.3 |
  | M8 inside the wheel | 7.8 (needs the wheel's 12 × 8 bearing spacer) |
  | Axle boss bearing | 1.8 |
  | 6001 swivel bearings vs static rating | 1.6 |
  | Pivot post | 2.6 |
  | Fork legs, fore-aft / sideways | 6.9 / 1.6 |
  | Fork crown | 7.8 |
  | Tail rib | 1.8 |
  | Tail-to-tub M5 lap | 1.5 |
  | Motor shaft | 3.7 |
  | Mast first mode | 118 Hz |

  What the review changed to get there (each was below 1.5 or failing before):
  - **Axle:** a 7075 sleeve and a longer boss.
  - **Caster:**
    - 6001 bearings in a 58 mm-tall post; the 24 mm-apart 608s were at a factor of 0.5.
    - 13 mm legs; 9 mm was at 0.9.
    - The fork now prints **on its side**.
    - The M12 head is locked in the crown, so no nut hangs where the wheel passes under the pivot.
    - Steel sleeves make every clamp metal-to-metal.
  - **Tail:** 6 mm walls, M5 inserts, 4 per side on two posts; it was at 0.7.
- **Wiring**: every route ends at its real connector and avoids solids. Mast cables lay into an open channel.
- **Printability**: every printed part fits 300 mm.

## 4. Printing

| Part | Qty | Material | Orientation / supports |
|---|---|---|---|
| tub | 1 | PAHT-CF, 5–6 walls, 40 % infill, dry filament | Floor on the bed (bottom edges are 2.5 mm chamfers, no elephant foot). Supports: under the axle bridge beam, top of the battery opening and of the rear tail window. Heat-set inserts after printing. |
| tail | 1 | PAHT-CF | Upright, skirt down; tree supports under the deck plate and around the pivot post. |
| caster_fork | 1 | PAHT-CF | **On its side** (one leg's outer face on the bed): layers run along the legs and across the crown. |
| motor_plate_L/R | 2 | PAHT-CF | Flat, counterbores up |
| wheel_pulley_L/R, motor_pulley_L/R | 2 + 2 | PAHT-CF, 0.4 nozzle, 0.12 mm layers | Axis vertical. **Tooth form is approximate** (round 1.14 mm grooves). For best belt life, regenerate the 56T/28T HTD-3M tooth profile with a pulley generator (e.g. the OpenSCAD "Parametric Pulley" by droftarts, HTD 3 mm) and keep these parts' hubs, flanges and bolt holes. Motor pulley: drill the centre bore to fit the shaft (5 mm on the MAD motor, 8 mm deep). |
| mast | 1 | PAHT-CF | Lying on its side; supports under the flange and pan head |
| deck | 1 | PETG (or PAHT-CF) | Flat |
| bumper_door | 1 | PETG (TPU skin optional) | Back face on the bed (prongs up) |
| pod_L/R, lid, tail_lid (hood) | 2+1+1 | PETG, bright colour | Pods and hood: open side down. Lid: flat face down. |
| us_bracket, uwb_plate, pan_buckle, esp32_cradle | 1 each | PETG | Back face / plate down |
| servo9g_adapter | 1 | PETG | Bench only (hidden in Fusion) |

Tolerances are deliberately loose: M3/M4/M5/M8 through-holes 3.4/4.5/5.5/8.5 mm, M3/M4/M5 heat-set holes 4.0/5.6/6.4 mm
(check your insert's chart), bearing seats +0.2 mm with retaining compound, 0.3 mm per side slip fits, ≥ 3 mm next to
anything that moves. Change any of these in `PARAMS` and re-run both scripts. Visible edges are rounded: lids, pods,
hood, bumper, pan head and the tub/tail corners.

## 5. Data quality: what is verified and what to measure

**Verified (fixed holes/pockets):** Raspberry Pi 5 85 × 56, M2.5 on 58 × 49 (3.5 mm from the non-USB end) ·
GoPro HERO9 71.0 × 55.0 × 33.6 mm, 158 g · Pololu D36V50Fx 25.4 × 25.4 × 9.5 mm · MG996R body 40.7 × 19.7 × 42.9,
ears 54 mm · HTD-3M pitch diameters · goBILDA 5203 data (used only to reject it).

**Measure before printing (you own these):**
1. HC-SR04: transducer centre distance (model 26 mm, windows Ø18) and PCB outline. Also the beam test below.
2. ESP32 DevKit width (`esp_w`, 28.5 mm) and length (`esp_l`, 55 mm): the cradle grooves are cut to the width.
   Also check which way the pins point (down, as modelled). If you use the SunFounder extension board instead, measure
   its outline and holes; it bolts to the cradle plate's 10 mm grid.
3. HC-SR04 header: confirm the 4 pins lie in the board plane off one long edge (common version). The sensor is
   mounted upside down so the plug points up.
4. 9 g servo: body length/width, ear hole spacing (adapter slots cover 26–30 mm), shaft offset from the body centre
   (`sg90_shaft_off`, 5.9 mm).
5. GoPro lens centre relative to the body (`gopro_lens_dy/dz`). It only shifts the view check slightly.

**Measure on arrival (then update PARAMS and re-run):** motor: body length (≤ 33 mm), prop holes (pulley needs 4 × M3
on D12; MAD 5010 EEE lists D12 and D18), shaft stub above the bell (≤ 9 mm, else drill the pulley through), M3 thread
depth in the bell (screw length); ESC size (≤ 48 × 28 × 14); wheel bearing inset (sets the sleeve length), that the
wheel has a 12 × 8 mm spacer between its bearings, and core material at Ø35 for the 4 pulley bolts; urethane contact
stiffness (press the wheel onto a 10 mm square bar: N per mm) to replace the 50 N/mm assumption; battery size (≤ 145 × 50 × 30),
UWB anchor board size/holes (plate slots fit ≤ 50 mm wide), standard-servo ear height (`servo_ear_z`).

## 6. Bill of materials (hardware not printed)

| Item | Qty | Notes |
|---|---|---|
| **MAD 5010 EEE V2.0 370KV** (reference), or any 5010-class 300–400 KV motor ≤ Ø62 × 33 with M3 base holes on 16–30 mm and **4 × M3 prop holes on D12** | 2 | Price not retrieved — check (industrial line, likely more than generic $20 motors). Generic 5010s fit the plate, but only use one with D12 prop holes. |
| M3 × 12 low-head (DIN 7984) screws | 8 | motor pulley → bell |
| HTD-3M **258 mm (86T)** 9 mm closed belt | 2 (+1 spare) | 255 or 261 mm also fit the ±4 mm slots |
| AM32 ESC 30–40 A, 2–4S, PWM input, bidirectional | 2 | + USB linker. Set: bidirectional, sine start-up on, LVC 3.3 V/cell |
| 90 × 52 longboard wheels (4-pack) with a 12 × 8 bearing spacer + one 70 × ~51 wheel for the caster | 1 + 1 | Core must take 4 × M3 on Ø35 |
| 6001-2RS bearings (12 × 28 × 8) | 2 | caster swivel |
| M8 × 150 hex bolt **12.9** + 8 × 12 × 1.5 speed washer + nyloc | 2 | stub axles |
| 7075-T6 (or 6082-T6) sleeve 18 × 8.2 mm, ~76 mm | 2 | turned from 18 mm bar and drilled 8.2; cut to measured length (bolt head pocket → wheel inner race) |
| M12 × 90 hex bolt 12.9, M12 nyloc + 2 washers, steel spacer 16 × 12.2 (~50 mm), steel sleeve 18 × 12.2 × 7 | 1 | caster pivot stack (metal only) |
| M8 × 80 12.9 + nyloc, 2 × steel sleeve 12 × 8.2 × 13 | 1 | caster wheel axle (sleeves through the fork legs) |
| M3 × 80 bolts + nylocs | 8 | wheel pulley through the core (stack ≈ 69.5 mm + washer + nut) |
| Heat-set inserts M3 × 5.7 (~35), M4 × 8.1 (~8), M5 × 9.5 (8) | | |
| M5 × 16 (8, tail lap), M3/M4 screws, M3 × 25 (2, belt pull screws), 2 × M4 thumb screws (mast), 2 × M4 thumb screws (door) | | |
| Pololu D36V50F5 (5 V 5.5 A, Pi) + D36V50F6 (6 V, servo) | 1 + 1 | zip-tied on the deck slots |
| XT60 pigtails, 20 A blade fuse holder, panel XT60 used as an arming plug | | pull the plug = everything off |
| MG996R or DS3218 servo, 470–1000 µF cap | 1 | |
| Right-angle USB-A cables (USB-C for the GoPro, micro-USB for the ESP32) | 2 | 25 mm plug space behind the Pi |
| 3S 5000 mAh hard case ≤ 145 × 50 × 30 | 1 | |

## 7. Assembly order

1. Inserts into tub, tail, motor plates, mast, ESP32 cradle. 6001s into the tail post.
2. Motor → plate (screw heads in the counterbores); 28T pulley → bell (4 × M3 low head) → plate into the wall
   pocket, 3 × M3 from inside, loose; pull screw through its boss into the plate tab, loose.
3. Stub axle from inside (head in its pocket) → 18 mm sleeve → speed washer → wheel with pulley bolted on →
   washer, nyloc. Tighten so the stack is solid.
4. Belt on; turn the pull screw until the belt passes the 2 N / 1 mm check (§1); tighten the three plate screws.
5. Tail onto the tub (8 × M5 sideways + 2 × M4 through the lip). Caster: M12 head into the crown pocket → 18 mm
   sleeve → washer → lower 6001 → 16 mm spacer → upper 6001 → washer → PTFE + spring washer → nyloc (snug, not
   tight: it is the shimmy damper).
6. ESP32 slides into the cradle grooves from the −y end; cradle bolts up from under the tail deck before the tail
   goes on.
7. Wiring in the side channels, deck with Pi/regulators in, lid, mast (2 thumb screws), GoPro, hood.
8. Battery in from the front, lead end last (the XT60 stays at the door, then is tucked up past the prongs),
   bumper-door on, 2 thumb screws.

## 8. Tests before and after building

- **Wheel stiffness (now, 5 minutes):** press a longboard wheel onto a 10 mm square bar on a kitchen scale and
  read how many newtons each mm of compression takes. Replace `K_LIP` in `cad/check_design.py` and re-run; the
  structural table scales with its square root.
- **HC-SR04 beam (now, no printing):** hold the sensor (upside down) at 100 mm height tilted 12° up on an empty floor or
  pavement, with `ros2 launch honebot_hardware esp32.launch.py` running. `/ultrasonic/range` should read `inf` or
  more than 1.9 m. If it reads 0.5–1.5 m, it sees the ground: tilt more (`us_tilt_deg`).
- **GoPro view (after printing the mast):** Wide 4:3, `camera.launch.py aim:=false`, then
  `ros2 topic pub /camera/pan_cmd std_msgs/Float32 "data: 1.0"` and `-1.0`. Look at `/camera/image/compressed` in
  Foxglove: no robot part may be in frame.
- **Shimmy:** at walking pace and then jogging, push on the tail. If the caster oscillates, tighten the pivot
  friction washer one flat at a time.
- **Firmware follow-up (safety):** the ESP32 currently drops to neutral instantly on fail-safe. With AM32
  "brake on stop" that is a full-strength stop, above the 0.71 g pitch-over limit. Ramp the speed command to
  zero at about 0.5 g instead (≈ 0.7 of full throttle per second at 15 mph). If your AM32 build has a brake
  strength setting, lower it too. The rounded bumper nose is the backstop, not the plan. Also cap the follower's
  lateral acceleration at ~0.5 g (sideways tip is at 0.79 g).

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
