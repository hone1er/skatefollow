# honebot custom chassis design (2026-11-13, budget revision)

Decision: **build a small, light, custom differential-drive robot** — two belt-driven longboard
wheels, one caster, a 3D-printed frame and body. No RC car, no e-skate hub motors, no VESC.
`hardware_plan.md` keeps the UWB / camera / radio research; its chassis and battery sections are superseded.

**Owner constraints (2026-11-13):**
- Whole robot fits in a backpack; lightweight and agile is the priority.
- Top speed **12–15 mph (5.4–6.7 m/s)** is enough. Payload is just the robot electronics and a GoPro.
- Cost: well under half of a $300 RC car. Target **≈ $150–200 for the driving robot**, plus battery + charger
  (~$85, which RC-car buyers also pay separately), plus UWB (~$70) when following is ready.

## 1. Why this is cheap and still fast enough

A 2.5 kg robot at 15 mph needs almost no power: rolling + air drag on asphalt is ~15–25 W, and accelerating
0→15 mph in 3 s needs a ~60 W peak. An RC car's 60 mph drivetrain is 10× overkill. Small brushless
outrunners (the kind sold for large drones) deliver 150–300 W for ~$20 each, and cheap RC-car ESCs run them
with forward / brake / reverse from the same 50 Hz servo signal the ESP32 firmware already produces.

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
  PETG or ASA for structure (6+ perimeters, 40–60 % infill at the mounts), TPU for the bumper. Not PLA.

## 3. Control (what the software already does)

- ESP32 firmware v2: channel A (GPIO 25) → left ESC, channel B (GPIO 26) → right ESC; both are capped by
  `THROTTLE_LIMIT`, ramped, and forced to neutral on any fail-safe (`DIFF_DRIVE = true`).
- RC-car ESCs: set the mode to **forward / brake / reverse**, calibrate neutral/endpoints once, disable the
  low-voltage cutoff only if the Pi monitors the pack (otherwise keep it: it protects the LiPo).
  One ESC's BEC (5 V) can power the ESP32; the Pi gets its own buck.
- Pi: bridge `drive_mode: diff`, `max_wheel_speed` ≈ 6.7 m/s, `track_width` ≈ 0.28; follower `ackermann: false`.
- Sensorless ESCs cog slightly starting from rest. The robot starts light and unloaded, so it's a non-issue;
  if it bothers you later, sensored ESC + motor is a $40 upgrade, not a redesign.

## 4. Bill of materials (US prices seen 2026-11-13; re-check before ordering)

### The driving robot (≈ $150–200)
| Item | Qty | Price | Notes / link |
|---|---|---|---|
| 5010 360 KV brushless outrunner (2–6S, 4 mm shaft) | 2 | **$19–24 ea** | SpeedyFPV $18.99 (search "5010 360KV" on Amazon/eBay too), [Newegg $23.59](https://www.newegg.com/p/1W7-00TR-029V9), [3-pack $109](https://www.fruugo.co.nz/3x-5010-360kv-high-torque-brushless-motors-for-multicopter-quadcopter-multi-axis-aircraft/p-449141044-944765372). Generic; any 300–400 KV 50 mm-class outrunner works. |
| 45 A 2–3S brushless **car** ESC with reverse + 5 V BEC | 2 | **$25–30 ea** | [HobbyKing 45A w/ reverse](https://hobbyking.com/en_us/hobbykingtm-45a-2-3s-waterproof-brushless-car-esc-w-reverse-2a-bec.html); Surpass Hobby 45A [$43](https://www.bluekeyhealth.com/shop/pg/10168209028)–[$51](https://upgraderc.com/products/surpass-hobby-45a-waterproof-brushless-esc-1). Must say "forward/reverse". **Cheaper alternative:** two 30 A BLHeli_S drone ESCs (~$10 ea) in bidirectional/"3D" mode; needs a configurator session and gives no proportional brake. |
| GT2 20T aluminium pulleys (4 mm bore) + GT2 6 mm or 9 mm closed belts | 2 + 2 | ~$15–20 | Any 3D-printer parts shop. Wheel pulleys are printed (48T–60T). |
| 83–90 mm longboard wheels, 4-pack | 1 | **$20–42** | Generic 83 mm 4-pack ~$20–25 ([TGM 83 mm](https://tgmskateboards.com/products/longboard-wheels-83mm-x-52mm-78a-red-4-pack)); [Powerslide 90 mm 4-pack $42](https://www.inlinewarehouse.com/Powerslide_Spinner/descpage-P905324.html). Pick a core you can bolt a pulley to. |
| 608 bearings ×8, M8 axle bolt/rod, M3/M4 hardware, heat-set inserts | — | ~$20 | |
| 5 V 5 A buck for the Pi 5 (9–36 V in) | 1 | **~$15–20** | [UCTRONICS 2-pack $39](https://www.newegg.com/p/0Y3-0680-00150); any 5 V/5 A module with ≥ 15 V input works on 3S (12.6 V full). Feed the Pi via GPIO 5 V pins. |
| ESP32 DevKit (kill-switch handheld; you already own the robot's) | 1–2 | $6–16 | 3-pack ~$16 |
| Filament: 1 kg PETG/ASA + a little TPU | — | ~$25 | Free if you already have it |
| **Subtotal** | | **≈ $170–210** (≈ $130–170 with drone ESCs and owned filament) | |

### Battery and charger (≈ $80–120, reusable)
| 3S 5000 mAh LiPo, 50C, hard case | **$48–75** | [Gens Ace 3S 5000 50C $60.99–74.99](https://hobby-sports.com/products/gens-ace-3s-lipo-battery-50c-w), [Zeee 3S 5200 $47.82](https://rcdrone.top/products/zeee-3s-5200mah-battery). ~55 Wh ≈ 1 h of following with the Pi on. |
|---|---|---|
| Balance charger (1–6S) | **$30–60** | [HTRC C150 $57.99](https://pyrodrone.com/products/htrc-lipo-charger-1-6s-ac-dc-c150-for-nicd-li-ion-life-nimh-lihv-pb-smart-battery), [SkyRC iMAX B6AC V2 ~$60](https://motionrc.com/collections/skyrc-battery-chargers/products/skyrc-b6ac-v2-50w-6-cell-6s-ac-dc-lipo-battery-charger-sk-100008-01) (Pololu warns about counterfeits); DC-only B6 clones ~$25–30. |

### Later (unchanged from hardware_plan.md)
UWB AoA anchor + tag $69.60; second ESP32 kill switch (above); GoPro already owned (printed mount, no gimbal:
the GoPro's wide lens forgives aiming error while the robot keeps the skater roughly centred).

### Variants
- **Ultra-budget (~$110 robot):** two 2212/2216 920–1000 KV motors ($8–10 ea) + two 30 A drone ESCs ($10 ea),
  7:1 belt (12T:84T, 84T printed pulley ≈ 54 mm fits inside the wheel). Tops out ~12 mph, less margin.
- **Single motor + steered front wheel (tricycle, Ackermann):** saves one motor+ESC (~$45), adds a 20 kg·cm
  servo ($12) and a printed steering knuckle. Uses `drive_mode: ackermann` and `ackermann: true`. More
  mechanical design, no turning in place; only worth it if two belts feel like too much.

## 5. Build order

1. **Drive module test (first weekend):** one motor + ESC + belt + wheel on a scrap bracket, ESC on the ESP32's
   channel A, Pi bridge in `diff` mode. Check direction, neutral, ramp, fail-safe (unplug ESP32 USB → wheel
   stops). Measure no-load wheel rpm → confirm the ratio gives ~15 mph.
2. **Frame plate + both drives + caster**, battery strapped, electronics taped on. Weigh it. Drive it slowly
   with the gamepad through the gate; then speed/shimmy tests in an empty lot (`THROTTLE_LIMIT` 0.3 → 1.0).
3. **Measure** track width, wheel diameter, speed at full command → set `track_width`, `max_wheel_speed`.
4. **Then print the body** around the proven frame: nose (HC-SR04 + UWB + bumper), electronics bay with vents,
   GoPro mast with foam damping, battery tray with a strap, lid.
5. UWB following, kill switch, camera aiming as in the main plan (Phases 3–7).

## 6. Open points
- Printer bed size and materials (PETG? ASA? TPU?) → frame segmentation and bumper.
- Backpack main-compartment dimensions → confirm the ~35 × 28 × 14 cm envelope.
- Which wheels: pick a core you can bolt a pulley to (spoked cores or drillable), 83 vs 90 mm.
- ESCs: car ESC (plug-and-play, reverse, brake) vs drone ESC (half the price, needs configuring).
