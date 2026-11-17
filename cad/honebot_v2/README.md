# honebot V2 — Fusion 360 script

`honebot_v2.py` builds the V2 robot from [`docs/robot_design_v2.md`](../../docs/robot_design_v2.md):
- **Drive:** two belt-driven rear wheels (Flipsky H5055 → HTD-3M 20T:36T).
- **Steering:** a servo-steered front wheel (GT2 belt, 1:1) with an AS5600 angle sensor on the stem.
- **Mast:** telescopes and folds forward, carrying the pan servo, GoPro, vision camera and UWB anchor on one pan stage.
- **Sensors:** an LD19 lidar and the HC-SR04 on a bolt-on nose.

| | |
|---|---|
| Size | 393 mm long × 308 mm wide (axle bolt heads). Folded with the GoPro off and the mast on the lowest pin: ~210 mm tall. |
| Mass | ~3.5 kg. Centre of mass 77 % on the rear axle, 133 mm up with the mast at its middle pin. |
| Camera | Lens ~0.43–0.50 m above the ground (mast pins at 30 / 60 / 90 mm). Pans on the servo you already have. |
| Speed | 15 mph at full command on 4S (1.8:1, 90 mm wheels). |

Nothing has been run in Fusion yet. The geometry is checked by the scripts in `checks/` (below).

## Running it in Fusion (first time: in stages)
1. Fusion 360 → Utilities → Add-Ins → Scripts and Add-Ins → **+** → pick the folder `honebot_v2`.
2. **Stage 1:** in `P` set `joints=False, hardware=False, belts=False`, then Run. This builds only the printed and
   bought parts. If it fails, the message box names the feature that broke; send it to me.
3. **Stage 2:** `hardware=True, belts=True`. This adds the bolts, nuts, bearings and belts.
4. **Stage 3:** `joints=True` (the default). Then **Assemble → Drive Joints**:

| Joint | Moves |
|---|---|
| Rear wheel L/R | the wheel; its motor pulley turns 1.8× through a motion link |
| Steering | the fork and front wheel, ±90°; the steering servo pulley follows 1:1 |
| Front wheel | rolls in the fork |
| Mast fold | the mast folds forward over the lid. Remove the GoPro and set the lowest pin first. |
| Mast height | the inner mast slides in the sleeve |
| Camera pan | the pan stage |

The chassis is grounded. Every number marked **MEASURE** in `P` came from a listing or a typical part. Measure yours
before printing; most of the geometry follows from `P`.

## Parts to print
| Body | Material | Orientation / notes |
|---|---|---|
| Chassis tub | PAHT-CF | Floor down. 277 × 154 × 139 mm. Hinge cheeks and the rear deck print as overhangs; use tree supports there. |
| Nose + steering head | PAHT-CF | Deck down (upside down), so both 6802 seats print round. Bolts to the tub's front wall (4 × M4 into inserts). |
| Steering fork (with its 48T pulley and hollow stem) | PAHT-CF | Crown down, stem up: the stem and pulley print round. The M8 bolt inside carries the bending. |
| Stem collar | PAHT-CF | Flat. M3 × 16 + nut across the slit. |
| Wheel pulleys L/R (36T HTD-3M) | PAHT-CF | Disc down, teeth up. |
| Steering servo pulley (48T GT2) | PAHT-CF | Flange down. |
| Spacers L/R (16/8.4, 11.5 nose) | PAHT-CF, or turned aluminium | Flat. |
| Mast sleeve | PAHT-CF | Lying on its side, so the pivot and lock holes print round. |
| Inner mast + pan servo head | PAHT-CF | Lying on its side. |
| Pan horn adapter | PAHT-CF | Flat. |
| Pan camera plate | PETG | Plate down. The GoPro prongs, webcam and UWB brackets stand up. |
| Lid | PETG | Flat. |
| Battery door | PETG | Flat. |

**Fits.** All of these are in `P`; tune them after a test print.

| Fit | Size |
|---|---|
| Bearing seats | +0.15 mm (snug press) |
| Printed shafts in bearings | −0.15 mm |
| Telescoping mast | 0.30 mm per side |
| Hinge knuckle in its cheeks | 0.50 mm per side |
| Servo pockets | +0.40 mm per side, with a 5 × 4 mm cable slit |
| Clearance holes | M3 3.4, M4 4.5, M5 5.5, M8 8.4 |
| M3 heat-set inserts | Ø4.0 × 5.7 mm |
| Self-tapping pilots | M2 1.8, M2.5 2.2, M3 2.6 |

## Hardware
| Qty | Part | Where |
|---|---|---|
| 2 | Flipsky H5055 200KV sensored, 8 mm shaft | Rear drive. MEASURE the mount pattern (`motor_pcd`) and the screw depth. |
| 8 | M4 × 8 button head (ISO 7380) | Motors to the walls, from outside, in tension slots |
| 2 + 2 | HTD-3M 20T pulley, 8 mm bore, ≤ 22 mm long with ≤ 7 mm hub; HTD-3M 225 mm × 10 mm belt | Hub faces out. File a flat on the shaft under the set screw. |
| 3 | 90 mm 78A longboard wheels, with their 608s and spacers | 2 rear, 1 front |
| 2 + 2 + 2 | M8 × 95 hex bolt (8.8), M8 washer, M8 nut | Rear axles: bolt from outside the wheel into a nut captured in the wall. ~2 N·m + threadlocker. |
| 6 + 6 | M4 × 60 + nyloc | Wheel pulleys to the wheel cores (3 per wheel; length depends on the wheel) |
| 1 + 1 | M8 × 85 + nyloc | Front axle. Snug only: the printed spacers on the fork legs clamp the inner races. |
| 1 + 1 | M8 × 60 low-head socket cap (DIN 7984, 10.9) + M8 nut | Down the hollow stem into the crown |
| 2 | 6802-2RS (15 × 24 × 5) | Steering head |
| 1 + 1 | AS5600 module + 6 × 2.5 mm diametric magnet | Magnet glued into the stem bolt's hex socket; board under the bridge (2 × M2 × 6) |
| 1 + 1 | GT2 280 mm × 9 mm closed belt; standard servo ≥ 20 kg·cm, ≥ 180°, 25T alu disc horn | Steering. 4 × M3 × 16 + nuts in the shelf slots (slide it to tension the belt), 4 × M3 × 8 horn screws. |
| 1 + 1 + 1 | M5 × 50 + nyloc, M4 × 45 + nyloc, M4 × 40 + nyloc | Mast pivot, mast lock, mast height pin |
| — | Your pan servo, 4 × M3 × 10 self-tapping; 4 × M3 × 8 horn screws; 4 × M3 × 8 + 4 short M3 inserts (plate to adapter) | Mast head |
| 4 + 4 | M4 heat-set inserts + M4 × 12 | Nose to tub |
| 12 | M3 heat-set inserts (6 lid, 2 door, 4 pan adapter, short) + M3 × 8 screws | |
| 4 + 2 + 4 + 4 | M2.5 × 6 self-tapping (Pi, IMU, lidar), M2 × 6 (webcam) | |
| 1 | LDROBOT LD19 lidar | Nose deck (4 × M2.5). MEASURE its hole pattern. |
| 1 | 4S 3000–3300 mAh LiPo, 20 mm velcro strap | Battery bay. Slides out of the rear door. MEASURE (`battery`). |
| — | Zip ties, 12 mm power switch, inline fuse, foam pad (ESP32), TPU pad (folded mast rest) | |

## Assembly order
1. **Inserts.** Heat-set all inserts: lid bosses, door bosses, nose bosses (M4), pan adapter.
2. **Rear axles.** Drop an M8 nut into each pocket on the inner face of the axle bosses (flats top and bottom).
   Then build the stack on the bolt, from outside in: bolt + washer, wheel (608s + skate spacer), wheel pulley
   (bolted to the wheel core), spacer (nose toward the bearing). Thread it into the nut: about 2 N·m plus threadlocker.
3. **Motors.** Bolt each motor in its pocket (4 × M4 button heads, finger-tight). Fit the 20T pulley in line with
   the 36T, hub outward, set screw on a filed flat. Loop the belt, slide the motor up its slots until the belt is
   snug, then tighten.
4. **Electronics, floor up.**
   - VESC on the right floor, zip-tied, heatsink up. Buck + BEC on the floor right of the battery.
   - ESP32 standing in the groove on the bulkhead's right side, zip tie round the top, USB end up through the top notch.
   - Pi 5 on its standoffs with the USB/Ethernet end facing the bulkhead; the plugs reach back through the bulkhead window.
   - IMU on the back of the bulkhead.
5. **Steering.**
   - Press the 6802s into the nose.
   - Fork stem up through both bearings. M8 nut into the crown pocket; the M8 × 60 low-head bolt down the stem into it, snug. Stem collar on top (M3 pinch).
   - Glue the magnet into the bolt's hex socket.
   - Front wheel into the fork.
   - Bolt the nose to the tub (4 × M4).
   - Steering servo into the shelf pocket (4 × M3 bolts + nuts in the slots). Belt on, slide the servo back to tension it, tighten.
   - AS5600 under the bridge, lidar on the deck, HC-SR04 into the bracket (hot glue or a zip tie).
6. **Battery** in from the rear, strap round it, door on (2 × M3).
7. **Mast.**
   - Sleeve between the hinge cheeks: M5 pivot and M4 lock bolts.
   - Inner mast in, M4 height pin.
   - Pan servo into the head; horn adapter on the horn (screws reachable now), then the camera plate (4 × M3).
   - GoPro on the prongs, webcam and UWB anchor on their brackets.
8. **Wiring and lid.** Run the wiring as below, then put the lid on (6 × M3).

## Wiring routes (made for quick changes)
- **Motors → VESC.** Phase and hall wires go down the inside of the motor walls, forward along the right floor,
  through the bulkhead's right floor pass. The left motor's wires cross over the top of the battery.
- **Battery → power.** Battery → fuse → switch (rear wall) → VESC and buck/BEC. All of this is on the right.
- **ESP32.**
  - USB up through the top notch, then to a Pi USB port through the window.
  - UART2 (GPIO 16/17) to the VESC.
  - I²C (GPIO 21/22) to the IMU and to the AS5600 through the nose cable pass.
  - Steering servo on GPIO 26, pan servo on GPIO 27.
- **Nose sensors.** The lidar, HC-SR04 and AS5600 cables go through the two passes in the nose flange and front wall.
- **Mast.** Camera, webcam, UWB and pan servo cables run down the inner mast bore and out of the sleeve's front
  window. They loop through the lid's rear notch, so the lid lifts off without unplugging anything. Leave a loop for folding.
- **Tie-down points.** Zip-tie anchors in the floor at the front wall and beside the battery. Every board is held by
  ties or screws, so nothing is glued except the magnet.

## Cooling
Air comes in through the front-wall slots, crosses the VESC and Pi, passes the motors, and leaves through the lid
slots over the motors, the Pi fan vent, and the rear wall and door slots.
- **Inside the tub:** about 22 W.
- **While driving:** ram air keeps the rise to about 3 K.
- **Parked:** the Pi fan alone holds it to about 9 K.

## Settings the design relies on
- **VESC Tool, per motor.** These caps stop the robot from lifting its front wheel under acceleration or locking the rear wheels:
  - motor current max ≈ 4 A (about 0.3 g of acceleration);
  - brake current ≈ −4 A (about 0.44 g, full rear-wheel braking);
  - battery current limits to match.
- **ESP32 firmware.**
  - Lateral acceleration cap v·ω ≤ 0.4 g; side tip is 0.64 g.
  - The same acceleration ramps as the VESC caps.
  - Stop if the AS5600 steering angle disagrees with the command by more than ~5°. The steering belt can skip on a wheel strike; this check catches it.

## Checks (run after any change to `P`)
Run these on the Mac, from `cad/honebot_v2/checks`:
```bash
python3 mock_fusion.py ../honebot_v2.py
```
```bash
python3 check_order.py ../honebot_v2.py mock_fusion.py
```
```bash
python3 check_v2.py ../honebot_v2.py
```
- **`mock_fusion.py`** runs the whole build against a mock Fusion API. It checks that outlines close, that joins
  and cuts touch their bodies, and that every joint finds its edge.
- **`check_order.py`** checks four things:
  - a later join doesn't refill an earlier cut;
  - every join touches existing solid;
  - no bolt, nut, board or bought part overlaps a printed part;
  - every printed part fits the H2C bed.
- **`check_v2.py`** prints the review: rear drive and steering-sweep clearances, electronics room, mast fold, fits,
  stability, stress, belt loads, acceleration caps and cooling. It exits non-zero on any FAIL.

**MEASURE before printing:**
- H5055 mounting pattern, the shaft length past the face, and whether its first 3 mm is non-rotating (the motor pocket).
- HTD-3M 20T pulley length and hub.
- Servo horn hole circle (`servo_horn_pcd`).
- LD19 hole pattern; UWB anchor board size and holes; webcam module holes.
- Battery size; VESC envelope; wheel core bolt circle.
