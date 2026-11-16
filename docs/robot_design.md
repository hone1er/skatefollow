# honebot custom chassis design (2026-11-13, budget revision)

> **Update 2026-11-16 — the CAD design supersedes parts of this file.** See [`chassis_cad.md`](chassis_cad.md)
> and the Fusion 360 script `cad/fusion/HonebotChassis/HonebotChassis.py`. Changed:
> - **Drive wheels at the front, caster at the rear.** This gives better, stable braking.
> - **Belt:** HTD-3M 9 mm, printed 24T pulley bolted to the motor bell → printed 48T (2:1, 231 mm belt), instead of GT2 on the shaft.
> - **Motor mount:** a replaceable slotted motor plate, so any 5010-class motor fits.
> - **Axles:** two M8 × 140 12.9 stub axles instead of the M8 × 300 rod.
> - **Battery:** in a floor tunnel loaded from the front.
> - **Camera:** removable mast at the nose.
> - **Frame:** printed as tub + tail module.
> - **Size and mass:** ≈ 396 × 348 × 147 mm (mast off) and ≈ 3.1 kg, not 350 × 280 × 140 / 2.5 kg.
>
> Unchanged and still valid here: §1 power/speed reasoning, the AM32 ESC set-up in §3, battery/charger choice.
> The layout sketches in §2/§2a and the frame-plate print row below describe the old design.

Decision: **build a small, light, custom differential-drive robot** — two belt-driven longboard
wheels, one caster, a 3D-printed frame and body. No RC car, no e-skate hub motors, no VESC.
`hardware_plan.md` covers the electronics around it (UWB, camera, Pi power, kill switch).

**Owner constraints (2026-11-13):**
- Whole robot fits in a backpack; lightweight and agile is the priority.
- Top speed **12–15 mph (5.4–6.7 m/s)** is enough. Payload is just the robot electronics and a GoPro.
- Cost: well under half of a $300 RC car. Target **≈ $150–260 for the driving robot** ($150 with generic ESCs and 83 mm wheels,
  $260 with branded ESCs and 90 mm wheels), plus battery + charger
  (~$80–120, which RC-car buyers also pay separately), plus UWB (~$70) when following is ready.
- Printer: Bambu Lab H2C; materials from PLA/PETG up to PAHT-CF and PPS-CF (see §2 Printing).

## 1. Why this is cheap and still fast enough

A 2.5 kg robot at 15 mph needs almost no power: rolling + air drag on asphalt is ~15–25 W, and accelerating
0→15 mph in 3 s needs a ~60 W peak. An RC car's 60 mph drivetrain is 10× overkill. Small brushless
outrunners (the kind sold for large drones) deliver 150–300 W for ~$20 each, and cheap AM32 ESCs run them
forward and reverse from the same 50 Hz servo signal the ESP32 firmware already produces.

Speed math, 3S LiPo (11.1 V nominal): 5010 360 KV → ~4,000 rpm no-load, ~3,400 rpm under light load.
90 mm wheel at 15 mph needs 1,420 rpm → **2.4:1 belt** (GT2 20T motor pulley, 48T printed wheel pulley).
Want more punch and 12 mph? Use 3:1 (20T:60T). The pulley ratio is the speed knob; no firmware change.
Torque at the wheel with 2.4:1: ~0.5 N·m → ~11 N per wheel → the robot is traction-limited, not motor-limited.

Weight budget: 2 motors 0.20 kg, 2 ESCs 0.10, 3S 5000 mAh 0.40, Pi 5 + cables 0.15, ESP32 ×2 0.03,
UWB 0.05, GoPro 0.15, 3 wheels 0.30, belts/pulleys/bearings/axles 0.20, printed frame + body 0.6–0.8
→ **≈ 2.3–2.6 kg**. Envelope ≈ 35 × 28 × 14 cm (camera mast folds).

## 2. Layout (3 wheels)

```
             front
      [ printed caster fork, 83–90 mm longboard wheel ]
      [ HC-SR04 + UWB anchor in the nose, TPU bumper   ]
   ------------------------------------------------------
   |  printed frame plate (PAHT-CF, one piece)           |
   |  Pi 5 | ESP32 | buck | 2 ESCs   3S pack low+centred |
   ------------------------------------------------------
   motor ==belt== [ 90 mm wheel ]      [ 90 mm wheel ] ==belt== motor      rear, driven
                   GoPro on a short damped mast above the axle
```
- Track width (wheel centre to centre) ≈ 0.28 m; wheelbase (caster → axle) ≈ 0.25 m. Set `track_width`
  in the bridge from measurement.
- Rear axle: one **M8 × 300 mm threaded rod** clamped in two printed split blocks under the plate; it does not
  rotate. Each wheel spins on its own two 608 bearings (already in skate wheels) and is driven by a printed 48T
  pulley bolted to the wheel core (the standard DIY e-skate "wheel pulley" pattern; use wheels with a
  spoked/holed core or drill 3 holes). Details and drawings: §2a.
- Motors on slotted printed L-brackets on top of the plate, 60 mm above and 25 mm ahead of the axle, shaft
  pointing outboard so the 20T pulley lines up with the wheel pulley just outside the plate edge.
- Caster: printed fork, ≥ 30 mm trail, a little friction on the swivel, same wheel as the drive wheels (one
  4-pack of wheels covers all three plus a spare). Add a second caster if it shimmies at speed.
- CG low and about one third of the wheelbase ahead of the driven axle.
- Body: printed shell panels on the frame plate, removable lid over the electronics bay, vents for the Pi.

### 2a. Drive module in detail (2026-11-13)

Scaled drawings (side view, plan, hub section) are in the "Honebot Drive Module" artifact:
https://claude.ai/artifact/JfPYvxVD6aSytuQ1F5Y3uJ . Summary of what they show, right side seen from outside:

```
                 (motor Ø58)            motor centre: 60 mm above, 25 mm ahead of the axle
                 (  20T  )  <- 20T GT2 aluminium pulley, grub screw, on the motor shaft
                   \\  //   <- GT2 belt, 9 mm wide, 200 mm (100T) closed loop, c-c 65 mm
   ---- frame plate 6 mm, underside 55 mm above ground, L-bracket on top, 4 vertical slots (10 mm) ----
                   [48T] <- printed 48T pulley (PD 30.6) on a Ø60 adapter disc, 3 × M4 × 65 into the wheel core
                 ((wheel)) Ø90 × 52 longboard wheel, axle 45 mm above ground, M8 rod in a printed block
   ==================================================================================== ground
```

- **No gearbox.** The single 2.4:1 belt stage is the whole reduction. Reprint the wheel pulley to change speed:
  20T:40T → 17.9 mph, 20T:48T → 14.9 mph, 20T:60T → 11.9 mph (90 mm wheel, 3400 rpm loaded on 3S; 83 mm wheels
  are ~7 % slower).
- **Motor above the axle**, not beside it: beside, the Ø58 can would hang 16 mm off the ground; above and
  25 mm forward it clears by ~76 mm and the belt runs nearly vertical, so sliding the motor up in its 10 mm
  slots tensions it directly. Belt length needed at 65 mm c-c is 199 mm → buy 200 mm.
- **The wheel carries the pulley.** Hub stack, inboard → outboard: axle block | 38 mm spacer tube (Ø12/Ø8) |
  [48T pulley 12 mm + adapter disc 6 mm, clearance bore, driven only by the 3 core bolts] | wheel (608, 10 mm
  skate spacer, 608) | washer | M8 nyloc. The nut clamps block, tube, inner races and washer into one stack;
  the pulley never touches the axle. Wheel inner face 114 mm from centreline, pulley 96–114 mm, plate edge
  90 mm (rear plate width 180 mm between the wheel wells; 260 mm ahead of them).
- **Loads are small.** 0.5 N·m at the wheel → ~33 N belt tension, about 9 teeth in mesh on the 20T. That is
  why GT2 (2 mm pitch) is enough here where e-skates need HTD-5M; GT2 20T pulleys also come in 4/5/6 mm bores.
- **Alignment** between the two pulleys within ~0.5 mm: set with the motor pulley's position on the shaft,
  file a flat for the grub screw.
- **Check before ordering pulleys:** (1) the motor's shaft — many 5010 multirotor motors end in an M5/M6
  threaded prop stub, not a plain 4 mm shaft; the 20T bore must match and ~14 mm of shaft must stick out past
  the bell. (2) The wheel core needs 3 holes/spokes on a ~40–50 mm circle; drill with a printed guide if not.

### Printing (Bambu Lab H2C)
Single-nozzle build volume is about 325 × 320 × 320 mm (dual-nozzle ~300 × 320 × 325; sources differ by a few
mm, check Bambu Studio), hardened nozzle, 350 °C hotend, so every part below fits in one piece and every
listed material prints natively.

| Part | Material | Notes |
|---|---|---|
| Frame plate (~330 × 260 mm), motor mounts, axle bearing blocks, caster fork, pulley hubs | **PAHT-CF** (or PPS-CF) | Stiff, creep-free, takes heat-set inserts and belt tension. 6 perimeters, 50 % infill, 0.6 mm nozzle is fine. Dry the filament. |
| Wheel pulleys (48T–60T GT2) | PAHT-CF or PETG | Print teeth-up with a 0.4 mm nozzle for accuracy. |
| Body shells, lid, electronics bay, GoPro mast | **PETG** | Light, cheap, UV-tolerant enough; 3 perimeters. |
| Nose bumper, Pi/GoPro vibration pads, cable grommets | TPU (95A) | |
| Nothing structural in PLA | | It softens in a hot car and cracks in cold. |

## 3. Control (what the software already does)

- ESP32 firmware v2: channel A (GPIO 25) → left ESC, channel B (GPIO 26) → right ESC; both are capped by
  `THROTTLE_LIMIT`, ramped, and forced to neutral on any fail-safe (`DIFF_DRIVE = true`).
- **ESCs: two AM32-firmware drone/boat ESCs (30–40 A, 2–4S or 2–6S) in bidirectional mode.** Decision
  2026-11-13: cheaper than RC-car ESCs *and* better behaved for a diff-drive robot. In bidirectional mode
  1500 µs = stop and reverse is immediate and symmetric, which the firmware's neutral/ramp logic assumes.
  RC-car ESCs usually need a "double tap" (first pull = brake, second = reverse), which breaks turning in
  place; avoid them unless they offer a crawler-style direct forward/reverse mode.
  One-time setup (~1 hour, done in a browser):
  1. Buy a USB linker for ESCs (BLHeli/AM32 "USB linker", ~$8–12) or use an Arduino Nano flashed as a 4-way
     interface. Some AM32 ESCs ship with the programming adapter.
  2. Open the AM32 web configurator (Chrome), connect via the linker to each ESC's signal wire + GND, power
     the ESC from the pack.
  3. Set **Bi-directional mode ON**, **Brake on stop ON**, **Startup power low**, PWM input (the ESC auto-detects
     1000–2000 µs servo PWM), **Low-voltage cutoff ON at 3.3 V/cell** (protects the LiPo), timing default.
  4. Bench test with the ESP32 at neutral first: motor must stay still at 1500 µs, spin both ways, and stop
     instantly when the ESP32's USB is unplugged (fail-safe).
  Avoid Bluejay-flashed ESCs: Bluejay is DShot-only and ignores servo PWM. Stock BLHeli_S also works
  (its manual confirms 1000–2000 µs input and a bidirectional setting) but AM32 is current and documented.
- Pi: bridge `drive_mode: diff`, `max_wheel_speed` ≈ 6.7 m/s, `track_width` ≈ 0.28; follower `ackermann: false`.
- Sensorless ESCs cog slightly starting from rest. The robot starts light and unloaded, so it's a non-issue.

## 4. Bill of materials (US prices seen 2026-11-13; re-check before ordering)

### The driving robot (≈ $150–260)
| Item | Qty | Price | Notes / link |
|---|---|---|---|
| 5010 360 KV brushless outrunner (2–6S, 4 mm shaft) | 2 | **$19–24 ea** | SpeedyFPV $18.99 (search "5010 360KV" on Amazon/eBay too), [Newegg $23.59](https://www.newegg.com/p/1W7-00TR-029V9), [3-pack $109](https://www.fruugo.co.nz/3x-5010-360kv-high-torque-brushless-motors-for-multicopter-quadcopter-multi-axis-aircraft/p-449141044-944765372). Generic; any 300–400 KV 50 mm-class outrunner works. |
| AM32 single ESC, 30–40 A, 2–4S or 2–6S, PWM input | 2 | **$15–40 ea** | Generic AM32 40 A 2–4S "supports PWM/DShot" listings ~$15–25 on Amazon/eBay/AliExpress; branded (e.g. [Flywoo GOKU G45M 45 A, AM32](https://flywoo.net/products/goku-g45m-32bit-128k-2-6s-45a-esc)) ~$30–45. Must be AM32 or stock BLHeli_S/32, **not Bluejay**. |
| USB linker for AM32/BLHeli ESC setup (one-time) | 1 | ~$8–12 | Or an Arduino Nano ($5) flashed as a 4-way interface. |
| GT2 20T aluminium pulleys (bore to match the motor shaft: 4, 5 or 6 mm, 9 mm belt width) + GT2 9 mm × 200 mm closed belts | 2 + 2 | ~$15–20 | Any 3D-printer parts shop. Wheel pulleys are printed (48T–60T). See §2a before ordering. |
| 83–90 mm longboard wheels, 4-pack | 1 | **$20–42** | Generic 83 mm 4-pack ~$20–25 ([TGM 83 mm](https://tgmskateboards.com/products/longboard-wheels-83mm-x-52mm-78a-red-4-pack)); [Powerslide 90 mm 4-pack $42](https://www.inlinewarehouse.com/Powerslide_Spinner/descpage-P905324.html). Pick a core you can bolt a pulley to. |
| 608 bearings ×8 (6 may come in the wheels), M8 × 300 threaded rod + nylocs, 2 × Ø12/Ø8 × 38 mm spacer tubes, M3/M4 hardware, heat-set inserts | — | ~$20 | |
| 5 V 5 A buck for the Pi 5 (9–36 V in) | 1 | **~$15–20** | [UCTRONICS 2-pack $39](https://www.newegg.com/p/0Y3-0680-00150); any 5 V/5 A module with ≥ 15 V input works on 3S (12.6 V full). Feed the Pi via GPIO 5 V pins. |
| ESP32 DevKit (kill-switch handheld; you already own the robot's) | 1–2 | $6–16 | 3-pack ~$16 |
| Filament: ~0.5 kg PAHT-CF (frame, mounts, fork), ~0.7 kg PETG (body), a little TPU (bumper, pads) | — | $0 (+ ~$20 TPU if not owned) | PAHT-CF and PETG are on hand. TPU is **not** in the owner's filament list: buy one spool or print the bumper/pads in PETG for now. |
| **Subtotal** | | **≈ $150–260** | Line items sum to $152 (generic ESCs, 83 mm wheels) up to $258 (branded ESCs, 90 mm wheels). |

### Battery and charger (≈ $80–120, reusable)
| Item | Price | Notes / link |
|---|---|---|
| 3S 5000 mAh LiPo, 50C, hard case | **$48–75** | [Gens Ace 3S 5000 50C $60.99–74.99](https://hobby-sports.com/products/gens-ace-3s-lipo-battery-50c-w), [Zeee 3S 5200 $47.82](https://rcdrone.top/products/zeee-3s-5200mah-battery). ~55 Wh ≈ 1 h of following with the Pi on. |
| Balance charger (1–6S) | **$30–60** | [HTRC C150 $57.99](https://pyrodrone.com/products/htrc-lipo-charger-1-6s-ac-dc-c150-for-nicd-li-ion-life-nimh-lihv-pb-smart-battery), [SkyRC iMAX B6AC V2 ~$60](https://motionrc.com/collections/skyrc-battery-chargers/products/skyrc-b6ac-v2-50w-6-cell-6s-ac-dc-lipo-battery-charger-sk-100008-01) (Pololu warns about counterfeits); DC-only B6 clones ~$25–30. |

### Later (see hardware_plan.md)
UWB AoA anchor + tag $69.60; kill-switch ESP32; GoPro already owned (printed, damped mount, no gimbal).

## 5. Build order

1. **Drive module test (first weekend):** configure both ESCs (above), then one motor + ESC + belt + wheel on a
   printed test bracket, ESC on the ESP32's channel A, Pi bridge in `diff` mode. Check direction, neutral, ramp, fail-safe (unplug ESP32 USB → wheel
   stops). Measure no-load wheel rpm → confirm the ratio gives ~15 mph.
2. **Frame plate + both drives + caster**, battery strapped, electronics taped on. Weigh it. Drive it slowly
   with the gamepad through the gate; then speed/shimmy tests in an empty lot (`THROTTLE_LIMIT` 0.3 → 1.0).
3. **Measure** track width, wheel diameter, speed at full command → set `track_width`, `max_wheel_speed`.
4. **Then print the body** around the proven frame: nose (HC-SR04 + UWB + bumper), electronics bay with vents,
   GoPro mast with foam damping, battery tray with a strap, lid.
5. UWB following, kill switch, camera aiming as in the main plan (Phases 3–7).

## 6. Open points
- Backpack main-compartment dimensions → confirm the ~35 × 28 × 14 cm envelope.
- Which wheels: pick a core you can bolt a pulley to (spoked cores or drillable), 83 vs 90 mm.
- Motor shaft type (plain 4 mm vs M5/M6 prop stub) → decides the 20T pulley bore (§2a).
- TPU: buy a spool or print bumper/pads in PETG.
