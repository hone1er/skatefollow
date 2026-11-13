# honebot custom chassis design (2026-11-13, budget revision)

Decision: **build a small, light, custom differential-drive robot** — two belt-driven longboard
wheels, one caster, a 3D-printed frame and body. No RC car, no e-skate hub motors, no VESC.
`hardware_plan.md` covers the electronics around it (UWB, camera, Pi power, kill switch).

**Owner constraints (2026-11-13):**
- Whole robot fits in a backpack; lightweight and agile is the priority.
- Top speed **12–15 mph (5.4–6.7 m/s)** is enough. Payload is just the robot electronics and a GoPro.
- Cost: well under half of a $300 RC car. Target **≈ $150–200 for the driving robot**, plus battery + charger
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
   |  printed frame plate (PETG/ASA, 2 halves dovetailed)|
   |  Pi 5 | ESP32 | buck | 2 ESCs   3S pack low+centred |
   ------------------------------------------------------
   motor ==belt== [ 90 mm wheel ]      [ 90 mm wheel ] ==belt== motor      rear, driven
                   GoPro on a short damped mast above the axle
```
- Track width (wheel centre to centre) ≈ 0.28 m; wheelbase (caster → axle) ≈ 0.25 m. Set `track_width`
  in the bridge from measurement.
- Rear axle: an **M8 bolt or 8 mm steel rod** through printed bearing blocks (608 bearings, the ones already in
  skate wheels). Each wheel is free on the axle and driven by its own printed pulley bolted to the wheel core
  (the standard DIY e-skate "wheel pulley" pattern; use wheels with a spoked/holed core or drill 3 holes).
- Motors on slotted printed mounts for belt tension, inboard of the wheels, just ahead of the axle.
- Caster: printed fork, ≥ 30 mm trail, a little friction on the swivel, same wheel as the drive wheels (one
  4-pack of wheels covers all three plus a spare). Add a second caster if it shimmies at speed.
- CG low and about one third of the wheelbase ahead of the driven axle.
- Body: printed shell panels on the frame plate, removable lid over the electronics bay, vents for the Pi.

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

### The driving robot (≈ $150–200)
| Item | Qty | Price | Notes / link |
|---|---|---|---|
| 5010 360 KV brushless outrunner (2–6S, 4 mm shaft) | 2 | **$19–24 ea** | SpeedyFPV $18.99 (search "5010 360KV" on Amazon/eBay too), [Newegg $23.59](https://www.newegg.com/p/1W7-00TR-029V9), [3-pack $109](https://www.fruugo.co.nz/3x-5010-360kv-high-torque-brushless-motors-for-multicopter-quadcopter-multi-axis-aircraft/p-449141044-944765372). Generic; any 300–400 KV 50 mm-class outrunner works. |
| AM32 single ESC, 30–40 A, 2–4S or 2–6S, PWM input | 2 | **$15–40 ea** | Generic AM32 40 A 2–4S "supports PWM/DShot" listings ~$15–25 on Amazon/eBay/AliExpress; branded (e.g. [Flywoo GOKU G45M 45 A, AM32](https://flywoo.net/products/goku-g45m-32bit-128k-2-6s-45a-esc)) ~$30–45. Must be AM32 or stock BLHeli_S/32, **not Bluejay**. |
| USB linker for AM32/BLHeli ESC setup (one-time) | 1 | ~$8–12 | Or an Arduino Nano ($5) flashed as a 4-way interface. |
| GT2 20T aluminium pulleys (4 mm bore) + GT2 6 mm or 9 mm closed belts | 2 + 2 | ~$15–20 | Any 3D-printer parts shop. Wheel pulleys are printed (48T–60T). |
| 83–90 mm longboard wheels, 4-pack | 1 | **$20–42** | Generic 83 mm 4-pack ~$20–25 ([TGM 83 mm](https://tgmskateboards.com/products/longboard-wheels-83mm-x-52mm-78a-red-4-pack)); [Powerslide 90 mm 4-pack $42](https://www.inlinewarehouse.com/Powerslide_Spinner/descpage-P905324.html). Pick a core you can bolt a pulley to. |
| 608 bearings ×8, M8 axle bolt/rod, M3/M4 hardware, heat-set inserts | — | ~$20 | |
| 5 V 5 A buck for the Pi 5 (9–36 V in) | 1 | **~$15–20** | [UCTRONICS 2-pack $39](https://www.newegg.com/p/0Y3-0680-00150); any 5 V/5 A module with ≥ 15 V input works on 3S (12.6 V full). Feed the Pi via GPIO 5 V pins. |
| ESP32 DevKit (kill-switch handheld; you already own the robot's) | 1–2 | $6–16 | 3-pack ~$16 |
| Filament: ~0.5 kg PAHT-CF (frame, mounts, fork), ~0.7 kg PETG (body), a little TPU | — | $0 | Owner has these on hand (see §2 Printing) |
| **Subtotal** | | **≈ $150–200** (filament owned) | |

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
