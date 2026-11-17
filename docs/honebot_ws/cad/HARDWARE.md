# honebot v5: design review and assembly hardware

All numbers come from the script's geometry (`honebot_chassis.py`, defaults) and the review checks. "MEASURE"
means the value depends on a part you haven't measured yet.

## 1. Verdict per platform

| Platform | Verdict |
|---|---|
| **Caster base** | **Buildable and drivable now.** Firmware 2.1.0 already drives it (AM32 ESCs on ESP32 GPIO 25/26). Stable: centre of mass 74 mm up; tips sideways only at 0.95 g, lifts the nose at 1.29 g, noses over at 1.84 g (tires grip ~0.8 g). **Main risk: the 50 mm caster** at speed (see §3). |
| **Ostrich legs** | **The mechanism is sound (wheel drive now on its own axle); the robot can't balance yet.** The leg geometry, servo torque (2.9× margin), belt coupling and power-loss behaviour (it settles on its skids) all check out. It needs balance firmware, an IMU and FOC drivers that don't exist yet. Main remaining risk: tall stances tip in hard turns (§2). |

## 2. Stability

### Ostrich (two wheels)
| Stance | Ride height | CoM above ground | CoM ahead of the axle | Tips sideways at | Speed cap on a 2 m turn (70 % of tip limit) |
|---|---|---|---|---|---|
| 5° (parked) | 104 mm | 103 mm | −4.0 mm | 1.13 g | 14 km/h |
| 15° (lowest filming) | 138 mm | 129 mm | −2.7 mm | 0.91 g | 13 km/h |
| 30° (riding) | 186 mm | 165 mm | 0 | 0.71 g | 11 km/h |
| 50° | 239 mm | 204 mm | +4.8 mm | 0.57 g | 10 km/h |
| 70° (tall) | 274 mm | 230 mm | +10.5 mm | 0.51 g | 9.5 km/h |

- **Ride low when fast.** Tall stances tip in hard turns. The controller should either cap speed × turn rate by
  height, or lower the inside leg to lean into turns (the legs can do this).
- **Balance point:** the CoM moves from 4 mm behind the axle to 10.5 mm ahead of it over the stroke. That's the
  knees swinging back, not the motor position (sweeping the motor angle barely changes it). The balance loop absorbs
  it as a lean of up to ~2.6°.
- **Power loss:** the legs fold until both skids touch, at a 3.3° leg angle, with the battery tray 15 mm off the ground.
  From the tall stance that's a 175 mm drop that back-drives the servo gears. **Precaution:** add a hard stop for the
  thigh at about 3° (not modelled yet), and have the firmware lower the robot before cutting power.
- **Side falls** (checked at 5°, 30° and 70°): the robot lands on the tire edge, the axle nyloc and a **bumper post
  next to each wheel motor**, then the GoPro. The motor and its 20T pulley never touch the ground.

### Caster base
Stable everywhere it can grip. See §3 for the caster wheel.

## 3. Motors and where they're installed: issues and what was done

| Issue | Effect | Done in v5 / recommendation |
|---|---|---|
| **Ostrich: the wheel hung on the 5010's 4 mm shaft** (direct drive) | A 10 g fall onto a wheel edge put ~790 MPa in the shaft, and only a circlip held the bell on. | **Changed (your decision):** the wheel now turns on 2 × 608 on a fixed M8 axle in the shank foot (220 MPa in the M8 at 10 g; 8.8 bolts yield at 640). The bolt head is captured in a slot in the shank's inner face, with a nyloc outside the hub. The 5010 sits on the shank 94 mm from the axle and drives the wheel through GT2 20T → 48T (2.4:1), carrying only belt tension. To tension the belt, the axle slides ±2 mm in its slot. **Positive stop:** once it's tight, fill the gap between the bolt head and the motor-side end of the slot with a printed shim (print 0.5/1/2/3 mm shims, 13 mm wide × 5 mm deep). Belt pull and wheel load both push toward the motor, so friction alone isn't enough. |
| Ostrich: motor/encoder wires | — | The motor sits on the shank near the knee, so its wires run in a short groove to the knee, then through zip-tie slots along the thigh's upper edge (both thigh faces are swept), over the hip-plate edge beside the deck, and down two pass-throughs into the electronics bay. **Not verified:** the knee turns ~130° over the stroke, and above the hip the wires cross the leg belt with nothing holding them clear. Leave a generous loop at the knee, tie the wires to the thigh edge right up to the hip, and check by hand over 5–70°. A wire guide over the hip belt is still to do. |
| Ostrich: drivers, MCU, IMU and servo BEC had no home | — | **Added:** a vertical electronics web at the front of the hip module. 2 FOC drivers (30 × 41 mm) go on the back face, the MCU and servo BEC on the front, all zip-tied. Two IMU bosses sit on the axle line between the hip servos. All of it is clear of the harness port, the HMI screws and the skids. |
| Ostrich: 1500 mAh is only ~15 min | — | **Bay enlarged** for a 3S 2200 mAh pack (36 × 106 × 26 mm, ~25 min). |
| Caster base: the bare LiPo was the lowest point (30 mm up) | Scrapes and punctures the pack. | **Added** a bolt-on battery **skid plate** under the cradle (26 mm up), 2 × M3 × 8 into inserts. The plate is 4 mm thick (0.3 mm sag even if it carried the pack). **The velcro strap is required**: it runs down 3 mm channels beside the pack and closes on the plate top behind the deck, and it's what carries the pack. |
| Caster base: no path for battery → ESC power | — | **Added** two 20 × 18 mm wiring pass-throughs behind the deck. |
| Caster base: 20T pulley grub screw on a 4 mm shaft | Slips or loosens under start/stop loads. | **Recommend** a 20T pulley that bolts to the bell's prop holes, or file a flat and use thread-lock. |
| Caster base: sensorless AM32 at low speed | Jerky starts and poor control below walking pace, and turning in place is rough. | Fine to start with. **Recommend** the same encoder + FOC drivers as the ostrich later; one electronics set would then run both bases. |
| Caster base: one motor screw per side is 4 mm from the deck wall | Re-tensioning the belt means lifting the deck. | Documented: tension before fitting the deck. |
| **Caster base: 50 mm caster at 24 km/h** | It spins at ~2600 rpm, shimmies, and can drop into 10–15 mm cracks and stop the robot dead. | **Decision for you (below).** |
| Deck: Pi 5 buck and pan-servo BEC had no home | — | **Added** a power web inside the bulkhead (2 pads, 43 × 28 mm), right over the harness port, behind the HC-SR04 and clear of the Pi. |
| Deck: pan-servo tab nuts unreachable behind the lintel | — | **Changed** to M3 heat-set inserts in bosses under the roof. |
| Deck: Pi standoff holes were 2.5 mm | An M2.5 self-tapping screw wouldn't bite. | **Fixed** to 2.2 mm. |
| Motor size | robot_design says Ø58; sellers list 5010 360KV at Ø50 × 26 mm. | **MEASURE** before printing: `motor_can_d`, `cb_motor`, both bolt circles, and the maximum screw depth into the motor (so screws can't touch the windings). |

### Decisions for you
1. **Ostrich wheel drive:** done. Switched to the belt drive with the wheel on its own axle.
2. **Caster.** Choose one:
   - **Two 50 mm casters** at the front corners (more stable, still small).
   - **A speed cap** of ~12 km/h on this base.
   - **A ≥75 mm caster**, which needs a taller plate. A raised centre mount isn't an option because it would sit in
     the HC-SR04 beam.

## 3b. Wiring for a prototype (quick to change)

Rule: **every wire crosses a module boundary through a plug, never a splice**, so swapping a part or a whole drivetrain
never needs a soldering iron.

| Boundary | Connector | Path in the model |
|---|---|---|
| Battery → anything | **XT30** (Y-splitter or a small Wago 221 block for the deck + ESCs/drivers) | caster: wiring pass-throughs 20 × 18 mm; ostrich: battery bay → electronics bay |
| Drivetrain ↔ deck | **XT30 + JST-XH 6** (INTERFACE.md pinout) | HMI harness port 18 × 26 mm, same on both drivetrains |
| Motor phases ↔ driver/ESC | **MR30** (3-pin) or 3.5 mm bullets | caster: short, on the plate; ostrich: along the leg (below) |
| Encoder ↔ driver | **JST-GH 5** (or Dupont for the first tests) | ostrich: alongside the phase wires |
| Hip servos ↔ hip MCU/BEC | standard 3-pin servo plugs + extension | inside the hip module |
| Pan servo ↔ deck BEC/ESP32 | servo plug | bulkhead (open at the back) |
| Pi 5 power | 5 V 5 A buck on the bulkhead power web, USB-C pigtail | right over the harness port |

**Ostrich leg wiring:**
1. The motor's phase wires and encoder cable leave the motor along the shank groove to the knee.
2. Leave a **generous loop at the knee** (it turns ~130° over the stroke).
3. Zip-tie the wires to the thigh's upper edge, then through the 2 thigh slots up to the hip.
4. At the hip, tie the wires through the **slot on the thigh's hip boss** (13.5 mm from the hip axis), then up to the
   **fairlead arm** that reaches out over the hip pulley on the hip axis line. The loop between them changes only
   12 mm over the whole stroke and never crosses the belt. Then go along the 2 zip-tie anchors on the hip plate
   beside the deck.
5. Drop into the electronics bay through the **open-edged slot** (20 × 12 mm) at the plate edge. Because it's open to
   the edge, the cable lifts out without unplugging or threading a connector.

Before connecting power, move each leg by hand from 5° to 70° and check no loop catches the belt, the knee pulley or
the wheel motor. This hasn't been verified in CAD.

**Other tips for a fast-changing prototype:**
- **Wire colours and labels:** red/black for power; label both ends (L/R, phase A/B/C).
- **Slack:** leave about 20 % in every run.
- **Zip-tie anchors:** 3 on the deck floor between the ESP32 and the Pi, 6 on the hip plate, slots on the power web, the
  electronics web, the ESC rows, the shank and the thigh. Use releasable ties or Velcro wraps so you can re-route fast.
- **Access:** lid off = 2 screws (use knurled M3 thumbscrews). Drivetrain off = 4 screws + 2 plugs. The bulkhead's
  open back exposes the buck, the BEC, the HC-SR04 and the pan-servo wiring.
- **ESP32 is mounted pins-up** (board upside down on 4 corner pads, 9 mm up so the USB plug fits underneath). Every
  GPIO takes a plug-in Dupont jumper, so there's no soldering to re-wire. Stick it to the pads with foam tape; the side
  fences stop it sliding.
- **Pi microSD:** the power pads leave the Pi's front edge (y 12–34) clear, so the card comes out with the lid off.
- **Measuring:** keep the ESP32's USB end facing forward (26 mm for the plug). Leave the JST-XH spares (pins 5/6)
  unconnected but present for probes or a kill switch.

## 3c. ESP32 ↔ Pi 5 USB cable

The ESP32 talks to the Pi over USB (`/dev/ttyUSB0`), and the Pi's USB-A ports face out of the deck's rear. The cable route:

1. **ESP32 USB end forward.** The plug has 18 mm in front of the board before the bulkhead power boards (0.8 mm spare).
   A **90° plug turning up** is the safer choice.
2. **Up from the plug and back along the deck floor**, tied to the floor anchors (y −11.5 to −6.8, just outside the Pi's edge).
3. **Out through the lid's rear slot.** It now starts at y −12 (the rear vents were trimmed back to clear it) and is open at the bottom.
4. **Round into a Pi USB-A port from behind.** A 90° USB-A plug keeps the loop tight.

Use a short (~25–30 cm) data cable with micro-USB or USB-C to match your board.

## 3d. Fits, servo slits, and stress/alignment review (v5 final)

**Fits** (tune `fit_press` / `fit_shaft` to your printer after a test print):
- **Bearing seats:** +0.1 mm (snug press). Printed shafts in bearings: −0.1 mm (slide).
- **Servo pockets:** +0.4 mm per side.
- **Others:** the SG90 plate lip 0.2/side, locating pegs 0.15/side, lid 0.3–0.4, ESP32 fences 0.45.

**Servo cable slits.**
- **Pan servo:** a 5 × 4 mm slit at the roof pocket's end away from the shaft lets the cable that exits low on the case
  pass. MEASURE which end your cable is on; to flip it, change the slit end in the script.
- **SG90 test plate:** its slit runs through the cable-end tab hole, so use an M2 screw + nut there.
- **Hip servos:** need none, because their bottoms never pass through a pocket.

**Stress and alignment (both drivetrains):**

| Item | Finding | Precaution taken |
|---|---|---|
| Ostrich foot belt | The belt runs 38 mm outboard of the shank, so its tension twists the motor mount. | Shank 9 mm; the outrigger plate now ties the shaft end, both posts and the axle end together, so the belt plane is held at both ends. **≤ 0.2° in PAHT-CF** (≤ 0.6° in PETG; upper bounds). |
| Ostrich outrigger restraint | It hung on one M3 at the post plus a slot at the axle, so it could pivot: it didn't hold the axle end in-plane. | **Fixed:** a second post on the far side of the belt (2 × M3 per outrigger) and a square-ended axle slot that takes flat printed shims. |
| Ostrich motor shaft bending | Tire grip caps the belt pull at ~29 N, below the 50 N preload, so the running shaft load is the preload: 88 MPa cantilevered on Ø4 (11 mm). | **Added** a 684ZZ in the outrigger on the shaft end, which roughly halves that and takes most of the side load off the motor's small bearing. The 30–40 A figures only apply with a jammed wheel (static, under yield). **Needs ≥ 22.1 mm of shaft past the bell (16 mm pulley, hub out): measure.** |
| Caster motor shaft bending | Tire grip caps the belt pull at ~22 N, below the 50 N belt preload, so the running load on the shaft is the preload: **103 MPa on Ø4** at 13 mm overhang (62 MPa at 30 N preload). Motor-current figures (30 A: 259 MPa) only apply with a jammed wheel, a static case well under yield. The real risks: 5010s are sold under one name with **3 mm shafts (245 MPa)** or with **no shaft stub past the bell**. | **Open: motor choice** (fixed motor + idler + outboard bearing, or an 8 mm-shaft belt motor). Buy only a motor whose drawing shows the shaft diameter and length past the bell. |
| Leg coupling belt | Up to 97 N over bumps twists the thigh. | **0.4° in PAHT-CF** (1.5° in PETG): print the thigh in PAHT-CF for real running. |
| Hip bearings | Two bearing seats printed sideways in the hip block would be oval and out of line. | They now sit in the hip pulley's sleeve, printed axis-up, so round and coaxial with the belt pulley. |
| Hip servo | The bearings plus a rigid horn over-constrain the servo output. | 4 mm horn-screw holes: the hub centres in its bearings, then the screws are tightened, so no side load on the servo. |
| Ostrich wheel axle | A cantilevered M8 bolt. | Supported at both ends: a 10 g side fall is ~46 MPa in the bolt (yield 640). |
| Caster motor webs | 1.5° lean in PETG from belt tension. | Two gussets per web: < 0.1°. |
| Printed GT2 teeth and PETG creep | Wear, and belt slack over time. | Inspect after the first runs; re-tension (ostrich: axle slot + shims; caster: motor slots); PAHT-CF creeps far less. |
| Belt tracking | Pulleys must line up within ~0.5 mm. | Set each 20T pulley on its shaft against the 48T teeth; flanges on all printed pulleys. |

## 4. Assembly hardware

### Deck module (both drivetrains)
| Qty | Part | Where |
|---|---|---|
| 4 | M3 heat-set insert, 5.7 mm (for Ø4.0 holes) | HMI bosses, pressed in from the underside |
| 2 | M3 heat-set insert | lid posts |
| 4 | M3 heat-set insert | pan-servo roof bosses |
| 3 | M3 heat-set insert | horn adapter (GoPro buckle screws) |
| 4 | M2.5 × 6 self-tapping pan head | Pi 5 to standoffs |
| 2 | M3 × 8 button/socket head | lid to posts |
| 4 | M3 × 8 + washers | standard pan servo tabs to roof |
| 4 | M3 × 5 button head | 25T aluminium disc horn to horn adapter (must not reach the servo case) |
| 3 | M3 × 6 socket head | GoPro buckle to horn adapter |
| 1 | M5 GoPro thumbscrew + nut | camera to buckle (usually comes with the GoPro) |
| 1 | 25T aluminium disc horn, Ø25, threaded M3 | standard pan servo |
| — | **SG90 bench test:** 4 × M3 × 12 (plate to roof inserts), 1 × M2 self-tapping + 1 × M2 × 10 with nut (cable end), 2 × M2 × 6 (horn arm to adapter) | |
| 1 | 5 V 5 A buck (Pi 5) + 6 V 3 A UBEC (pan servo) + 470–1000 µF capacitor, zip ties | bulkhead power web |
| 1 | XT30 pair + JST-XH 6-pin pair, 22/18 AWG silicone wire | harness through the HMI port |
| 1 | short USB data cable, ESP32 end (micro-USB/USB-C) to USB-A, ideally 90° plugs both ends, ~25–30 cm | ESP32 ↔ Pi 5 |
| — | releasable zip ties (2.5–3.6 mm), Velcro cable wraps, wire labels, a few Wago 221 lever connectors | wiring that changes often |

### Ostrich legs
| Qty | Part | Where |
|---|---|---|
| 4 | M3 × 8 socket head | hip module to deck (HMI) |
| 2 | standard 25T servo ≥ 35 kg·cm (DS3235 class) + 2 × 25T Ø25 aluminium disc horns | hips |
| 8 | M3 × 12 self-tapping pan head | servo tabs to hip blocks |
| 4 | 6804-2RS (20 × 32 × 7) | hips |
| 4 | 6802-2RS (15 × 24 × 5) | knees |
| 8 | M3 heat-set insert | hip-pulley mounts in the hip blocks |
| 8 | M3 × 16 socket head | 80T hip pulleys to blocks |
| 8 + 2 | M3 × 40 socket head + M3 × ~45 (spline screws, MEASURE thread depth) | thigh hub to horn, from the thigh's outer face |
| 2 | M5 × 40 + nyloc + 2 washers | knee through-bolts |
| 2 | 625ZZ + M5 × 25 + nut + washers | belt idlers |
| 2 | GT2 10 mm × 320 mm (160T) closed belt, glass/aramid cord | leg coupling (TPU test belt: printed) |
| 2 | 5010 360KV outrunner | wheel drive, on the shanks |
| 8 | M3 × 8 socket head (MEASURE max depth) | motor base to shank (tighten after the outrigger is on) |
| 2 + 2 | GT2 20T aluminium pulley + GT2 9 mm × 256 mm (128T) belt | wheel drive (check the belt length is sold; ≥ 126T works) |
| 4 | 608-2RS bearings | wheel hubs |
| 2 | M8 × 70 hex bolt + 3 washers + M8 nyloc, Ø12/Ø8 × 20.5 mm spacer tube (hub bore Ø12.6) | wheel axles (head captured in the shank's inner face, outer end through the outrigger) |
| 4 + 4 | M3 × 8 + M3 heat-set insert | outriggers to posts (2 per side) |
| 2 | 684ZZ (4 × 9 × 4); 685ZZ (5 × 11 × 5) if your shaft is Ø5 | motor shaft ends, in the outriggers |
| — | printed shims 0.5 / 1 / 2 / 3 mm | axle slots (behind the bolt head, and in the outrigger) |
| 2 | magnetic encoder breakout (MT6701 / AS5047, ABI) + Ø6 × 2.5 diametric magnet, epoxy | shank pockets |
| 2 | FOC driver (B-G431B-ESC1 class; verify the continuous rating) | electronics web |
| 1 | MCU for the balance loop (ESP32 / STM32) + IMU (ICM-42688 / BNO085 class) + 2 × M2.5 × 8 self-tapping | electronics web / IMU bosses |
| 1 | 6–7.4 V BEC, ≥ 6 A (2 hip servos) | electronics web |
| 1 | 3S 2200 mAh LiPo (36 × 106 × 26) + 15 mm velcro strap | battery bay (slides out forward under the front skid, 40 mm of room); the strap goes lengthwise round the tray |
| — | zip ties, spiral wrap, 3 × 18 AWG + encoder cable per leg, MR30 + JST-GH 5 plugs (2 each) | wiring along the legs (plugs at the electronics bay) |

### Caster base
| Qty | Part | Where |
|---|---|---|
| 4 | M3 × 8 socket head | base to deck (HMI) |
| 2 | 5010 360KV outrunner | drive |
| 8 | M3 × 12 socket head + washers (MEASURE max depth) | motors to 8 mm webs |
| 2 | GT2 20T aluminium pulley, 9 mm belt, ≤ 16 mm long, hub facing out; Ø5 bore (Ø5 shaft motors recommended) or a bell-bolted pulley | motor shafts |
| 2 | GT2 9 mm × 200 mm (100T) closed belt | drive (TPU test belt: printed) |
| 3 | Ø90 longboard wheels with 608 bearings + 10 mm skate spacers (2 drive + spare) | |
| 6 | M4 × 65 + nyloc | 48T pulley to wheel core |
| 1 | **M8 threaded rod cut to 360 mm** (robot_design's 300 mm is too short) | axle |
| 4 + 2 | M8 nuts (both sides of each axle block) + M8 nylocs | axle |
| 4 | M8 washers | axle ends |
| 2 | spacer tube Ø12 / Ø8 × 38 mm (aluminium or printed) | axle, inboard of each pulley |
| 1 | 50 mm swivel caster, top plate 63 × 48, M4 holes 48 × 33 (MEASURE) | see decision 2 |
| 4 | M4 × 16 + nyloc + washers | caster to plate |
| 2 | M3 heat-set insert + 2 × M3 × 8 | battery skid plate |
| 2 | AM32 ESC 30–40 A, 2–6S, bidirectional mode | zip-tied behind the deck |
| 1 | 3S 5000 mAh LiPo (138 × 43 × 36) | cradle |
| 1 | 20 mm velcro strap (**required**, it carries the battery) | down the cradle's side channels, closed on the plate top behind the deck |
| — | zip ties, XT30 Y-splitter (ESCs + deck) | |

### Tools / consumables
Soldering iron with heat-set insert tips (M2.5/M3), hex keys 2/2.5/3/4 mm, M8/M5 spanners, blue thread-lock, epoxy
(encoder magnets), and an AM32 USB linker for ESC setup.
