# honebot custom chassis design (2026-11-13)

Decision: **build a custom differential-drive robot instead of buying an RC car.** Two driven wheels
plus a caster, a 2020-extrusion spine, and a 3D-printed body. This document is the engineering plan;
`hardware_plan.md` keeps the UWB / camera / radio / power research, whose chassis section is superseded.

**Size and weight constraint (owner, 2026-11-13): the whole robot must fit in a backpack; lightweight
and agile is the priority.** Design targets that follow from it:

| Target | Value | Why |
|---|---|---|
| Envelope | **≤ 40 × 30 × 15 cm** (camera mast removable/folding) | Fits a 25–30 L backpack main compartment |
| Mass, ready to run | **4–5 kg** (hard cap 6 kg) | Carryable; agile; hub motors barely notice it |
| Track width | 0.28–0.32 m (a standard e-skate hanger) | Sets the 30 cm width |
| Wheelbase (caster → driven axle) | 0.25–0.30 m | Fits the 40 cm length with a nose |
| Wheels | 90 mm hub motors (105 mm if they fit the height budget) | Bigger = better over cracks at speed |
| Battery | **10S1P Li-ion, 3.5–4.2 Ah, ~0.6–0.75 kg** | The single biggest weight lever; see §4 |

Weight budget (estimates): hub-motor pair + hanger ~2.0–2.2 kg, battery 0.6–0.75 kg, Dual FSESC 0.2 kg,
Pi 5 + ESP32s + UWB + regulator + cables 0.4 kg, extrusion spine + printed body + caster 0.8–1.0 kg,
camera (Osmo Pocket 3) 0.18 kg → **≈ 4.3–4.8 kg**. Hoverboard motors (≈ 2.5–3 kg *each*) and 10S2P+
packs are therefore out.

## 1. Feasibility: can a printed 2–3 wheel robot keep up with a skater?

The requirement is 15–25 km/h (4–7 m/s) with hard stops. That rules out the usual hobby-robot
drivetrain (gear-motors, printed gears, 65 mm wheels: ~5–8 km/h) but not a custom robot. The trick is
to use a drivetrain already engineered for exactly these speeds and loads, and print everything else:

| Drivetrain | Speed | Cost (pair of motors + ESC) | Verdict |
|---|---|---|---|
| **Electric-skateboard 90 mm hub-motor wheels + VESC** | 30+ km/h at 10S, plenty of torque for a 5 kg robot | $250–430 | **Recommended.** Motors live inside the wheels (no gears/belts), bolt to an aluminium truck hanger, and VESC gives current control, PPM input, UART telemetry (wheel speed = free odometry). ~1 kg per motor ([Maytech 90 mm listing: 1000 g](https://reach.dog/shop/michobbycom/products/mto9055rh-60-ha-maytech-90mm-60kv-hub-motor-1000w)). |
| Hoverboard 6.5" hub motors + reflashed hoverboard mainboard | ~10–15 km/h (6.5"), 15–20 km/h (10") | $30–150 (used hoverboard), ST-Link $10 | **Rejected for this build**: 2.5–3 kg per wheel plus a 2 kg pack → 8–10 kg, suitcase-sized. Fails the backpack constraint. Kept here only as a reference. |
| Gear-motors / printed drivetrain | 5–8 km/h | $60–120 | Walking pace only. Not viable for the goal. |

Where printed parts are and aren't OK: printed brackets, clamps, caster forks, shell panels and
mounts are fine in PETG/ASA with 6+ perimeters and 40–60 % infill (CF-nylon for the hanger clamps if
available). Do **not** print the axle or anything carrying wheel loads at speed: the e-skate truck hanger
(aluminium) carries the wheels; printed blocks only clamp it to the frame. Use PETG or ASA, not PLA
(soft in a hot car, brittle in cold).

Dynamics sanity check (5 kg robot, CG ~7 cm high, driven wheels ~10 cm behind the CG): pitch-over
needs > 1.5 g of deceleration, far above what rubber on asphalt gives, so hard stops are safe. The real
risk at speed is **caster shimmy**: a swivel caster oscillating above ~10 km/h. Mitigate with a large
wheel (≥ 70 mm), ≥ 30 mm trail, a stiff fork, and a bit of friction/damping on the swivel; if it still
shimmies, add a second caster for a 4-wheel stance. Test this on the mule before printing the body.

## 2. Layout (baseline: 3 wheels)

```
            front
      [ caster  (70–83 mm skate wheel, printed fork) ]
      [ HC-SR04 + UWB anchor in the nose              ]
   ---------------------------------------------------
   |  2020 extrusion frame ~ 350 x 280 mm             |
   |  Pi 5 + ESP32s + Pololu 5 V reg  (bay, printed)  |
   |  10S1P battery (low, centred, strapped)          |
   ---------------------------------------------------
      [ hub wheel ]====== truck hanger ======[ hub wheel ]   rear, driven
      camera mast above the rear axle, foam-damped
```
- Track width (wheel centre to centre) ≈ 0.30 m with a standard e-skate hanger; set `track_width` in the
  bridge from measurement. Wheelbase (caster to driven axle) ≈ 0.28 m. Mass target 4–5 kg with battery.
- Agility: differential drive turns in place (follower runs with `ackermann: false`). The short wheelbase
  and light mass make it nimble; the price is pitch sensitivity under hard braking, so keep the battery
  low and between the wheels and the camera mast short (or folding) and light.
- CG low and roughly 1/3 of the wheelbase ahead of the driven axle: enough weight on the caster to stop
  it chattering, most weight on the driven wheels for traction and braking.
- Optional 4th wheel: a second front caster, same printed part, if shimmy or tipping shows up.
- Body: printed shell panels that snap/bolt onto the extrusion (printer bed ~250 mm → body in 4–6
  segments with dovetails), TPU bumper strip around the nose, a removable lid over the electronics bay.

## 3. Control architecture (what changes vs the RC-car plan)

- The ESP32 firmware already outputs two 50 Hz 1000–2000 µs channels with fail-safe; **channel A → left
  VESC PPM input, channel B → right VESC PPM input.** VESC app: PPM, control type "Current" (bidirectional)
  with a modest current limit and the VESC's own PPM timeout enabled (second fail-safe).
- The Pi bridge converts Twist → (left, right) with `drive_mode: diff` (default now; `diffdrive.py`, tested).
- The follower runs with `ackermann: false` (it may turn in place), gate unchanged.
- Later: USB/UART from the Pi to the VESC (PyVESC) for ERPM, current and voltage → wheel odometry and a
  battery topic. The PPM path stays as the safety-critical one.

## 4. Bill of materials (US prices seen 2026-11-13; re-check before ordering)

### Drivetrain (route A, recommended)
| Item | Price | Notes / link |
|---|---|---|
| Dual 90 mm hub-motor wheel kit on a truck hanger (2 × ~600 W, 6364 class) | **$130–280** generic | [eBay 90 mm 600 W dual hub kit](https://www.ebay.com/itm/375924090663), [another](https://www.ebay.com/itm/388737114189). Branded kits were mostly sold out: Maytech dual hub $372 (OOS), [Michobby 90 mm dual + VESC $437 (sold out)](https://michobby.com/collections/hub-motors), Flipsky kits via [Banggood $399.99 (0 stock)](https://usa.banggood.com/Flipsky-Group-DIY-Electric-Skateboard-Double-Main-Shaft-Trucks-and-Motor-Kits-Dual-Drive-p-1865308.html). Check [flipsky.net](https://flipsky.net/collections/new-accessories?page=1) directly (it rate-limited my fetch). Prefer sensored motors. |
| Flipsky Dual FSESC 4.20 Plus (two VESC 4.x channels, anti-spark switch, PPM + UART) | **$116–150** | [flipsky.net](https://flipsky.net/products/dual-fsesc4-20-plus-based-on-vesc-with-anodized-aluminum-heatsink) ($116 sale / $150). Forum consensus: fine at ≤ 30 A battery current, which a 7 kg robot never approaches ([esk8 forum](https://forum.esk8.news/t/is-the-dual-4-2-flipsky-vesc-really-that-bad/22890?page=2)). |
| **10S1P Li-ion pack with BMS, XT60** (lightweight pick) | **$95–170** | [WREKD 10S1P Molicel P42A 4.2 Ah, 750 g, $170, in stock](https://tuah.wrekd.com/products/10s1p-36v-molicel-p42a-4200mah-li-ion-battery-pack) (quality cells, 150 Wh ≈ 25–35 min of following); [JKING 10S1P 36 V 2.5 Ah $95.80](https://m.reach.dog/shop/jkingthe-best-electric-skateboard-electric-longboard-shop/products/10s1p-battery-36v2-5a-for-h2e) (lighter, shorter sessions); a Samsung-18650 10S1P 3.5 Ah pack is ~600 g ([CM Batteries](https://cmbatteries.com/?p=22963), quote only). Check the BMS continuous current (want ≥ 20 A). |
| 10S2P 7 Ah alternative (longer sessions, +0.4 kg) | $90–130 generic | Generic 10S2P 7 Ah XT60 seen at $89.99 ([listing](https://reach.dog/shop/batteryint/products/36v-10s2p-7ah-450watt-18650-lithium-ion-battery-for-skateboard-electric-bike-xt60)); ~1 kg. Only if the weight budget allows. Not 6S LiPo: 90 mm hubs at 6S top out ~18–20 km/h, under the 25 km/h target. |
| 42 V 2 A charger | ~$25 | Any 10S Li-ion charger with the pack's connector. |
| Pololu D36V50F5 5 V 5.5 A regulator for the Pi (input to 50 V: OK on 10S) | $34.95 | [Pololu](https://www.pololu.com/product/4091) |
| XT60 leads, 30 A fuse + holder, master switch, 12 AWG wire | ~$25 | |

### Frame, wheels, printing
| Item | Price | Notes / link |
|---|---|---|
| 2020 T-slot extrusion, ~1.5 m total | **$38** | [Zoro 4 × 24" $37.99](https://www.zoro.com/powertec-4pcs-24in-2020-t-slot-aluminum-extrusion-profile-linear-rail-20mm-x-20mm-4pk-74001/i/G422706679), [VEVOR 10 × 1 m $66.90](https://www.vevor.com/s/1000-mm-2020-aluminum-extrusion), [Zyltech cut lengths](https://www.zyltech.com/2020-t-slot-aluminum-extrusion-pre-cut-lengths-300mm-2000mm) |
| M5 T-nuts + bolts, corner brackets (or print the corners) | ~$20 | |
| Caster: printed fork + skate wheel + 6001-2RS bearings | **~$25** | [MakerWorld caster (6001 bearings, M4, PETG + TPU tire)](https://makerworld.com/en/models/3100504-caster-wheel); [skateboard-wheel caster model](https://stlfinder.com/model/wheel-casters-skateboard-wheel-p9wFhIKs/4327908). Longboard wheels 70–83 mm, set of 4 ~$15–25. |
| Filament: 1–1.5 kg PETG or ASA + 0.5 kg TPU | ~$50 | Thin shell panels (2–3 perimeters, low infill) keep the body under ~0.6 kg; only clamps/forks go dense. |
| Hardware: M3/M4/M5 assortment, heat-set inserts, standoffs, foam/silicone dampers | ~$30 | |

### Electronics already planned (unchanged, see hardware_plan.md)
Makerfabs MaUWB_AOA Gen2 anchor + tag $109.40 (2 anchors) or $69.60 (1 anchor); 3 × ESP32 ~$16; FlySky FS-i6X
+ receiver ~$65; Logitech C920 ~$60; DJI Osmo Pocket 3 ~$430–550 (later).

### Totals
- Lightweight robot (generic hub kit, 10S1P pack, Dual FSESC, regulator, frame, printed parts, no UWB/radio/camera):
  **≈ $560–720**; with branded motors and the WREKD pack ≈ $900.
- Add UWB + radio + ESP32s: + ≈ $190. Camera later.
- The hoverboard "mule" route from the first draft is dropped: it cannot meet the size/weight constraint,
  and the lightweight frame is cheap enough to serve as its own mule (build it bare first, body later).

## 5. Build order

1. **Bare frame first (1 weekend):** 350 × 280 mm extrusion rectangle, hub-wheel hanger clamped at the back,
   one printed caster at the front, 10S1P battery strapped low, Dual FSESC + ESP32 + Pi on top. No body yet.
   Weigh it: if it is already over 4 kg, cut before adding the body.
   Verify: VESC motor detection, PPM fail-safe (unplug the ESP32 USB → wheels stop), `drive_mode: diff`
   on a stand, then low-speed driving with the gamepad through the gate.
2. **Speed and shimmy tests** in an empty lot: increase `THROTTLE_LIMIT`/VESC limits stepwise to 25 km/h;
   fix the caster (trail, damping, second caster) before anything else.
3. **Measure** track width, wheel radius, speed at full command → set `track_width`, `max_wheel_speed`.
4. **Then design the body** around the proven frame: sensor nose (HC-SR04 + UWB anchor + bumper), electronics
   bay with airflow for the Pi, camera mast with damping, battery tray. Print in segments; iterate.
5. UWB following, camera, RC override and kill switch as in the main plan (Phases 3–7).

## 6. Open points
- **Battery weight allowance** (decides 10S1P vs 10S2P, i.e. ~25–35 min vs ~1 h of following).
- Backpack dimensions: measure the main compartment to confirm the 40 × 30 × 15 cm envelope.
- Printer bed size and material you can print (PETG? ASA? TPU?) — decides panel segmentation.
- Hub-motor kit availability: the 90 mm generic kits come and go; buy when one is in stock and sensored.
- Whether to run the Pi from the 10S pack via the Pololu (recommended) or a separate 3S pack (simpler
  wiring, one more thing to charge).
