# Ostrich drivetrain (v5): two wheels, adjustable-height legs

Built by `honebot_chassis.py` with `drivetrain='ostrich'` (or `'both'`). All numbers below are the script defaults.
Change parameters in `P` and rerun; the checker numbers in the script's message box update.

## How it works

Each side has a reverse-knee leg: **thigh** (hip → knee, runs backwards) and **shank** (knee → axle,
runs forwards), both **99.3 mm** between centres (solved so a stock 160-tooth GT2 loop fits with ~0.3 mm
pre-stretch). A **GT2 belt** runs from an **80-tooth pulley fixed to
the hip module** to a **40-tooth pulley fixed to the shank**. That 2:1 ratio makes the shank turn up by
exactly as much as the thigh turns down, so:

- the **wheel axle stays directly under the hip** and moves straight up and down (0 mm sideways drift);
- **one servo per leg** sets the height. The deck underside travels from **104 mm** (parked) to
  **274 mm** (tall) above the ground, 169 mm of travel;
- the two legs can be set to different heights to keep the body level on a slope or to lean into turns.

The hip x position is calculated so the **centre of mass sits over the axle** at the riding stance
(30°). The GoPro on its standard-size pan servo is heavy and far forward, and the knees hang back, so
the result is x ≈ −9 mm.

| Stance | Leg angle | Deck underside above the ground | Notes |
|---|---|---|---|
| Parked | 5° | 104 mm | Tips 8° forward or back onto a skid. Battery tray 17 mm above the ground. |
| Lowest filming | 15° | 138 mm | Wheels are out of the GoPro shot at every stance from 5° up |
| Riding (as drawn) | 30° | 186 mm | Centre of mass ~105 mm above the axle |
| Tall | 70° | 274 mm | |

Track 238 mm, overall width ~341 mm (to the axle bolt ends), estimated mass ~1.7 kg (edit `mass` in `P`).

## Layers (left side; the right side mirrors it)

Every moving part runs in its own slice along Y, so nothing in a leg can collide with anything else
at any height:

| Y (mm) | Part |
|---|---|
| 36.5–69 | hip block (part of the hip module): hip servo; a bore for the hip pulley's bearing sleeve |
| 53–84 | hip pulley: 80T teeth (69–84) **plus a sleeve that carries both 6804 hip bearings** (printed axis-up, so round and coaxial with the pulley), bolted to the block with 4 × M3 × 16 |
| 69–84 | belt plane: 80T hip pulley, 40T knee pulley, GT2 10 mm belt |
| 84.5–94.5 | thigh (10 mm thick, 2 × 6802 bearings at the knee) |
| 95–104 | shank (**9 mm**): a triangular plate joining knee, wheel axle and wheel motor, plus **two posts** beside the motor, one each side of the belt (bumpers, and they carry the outrigger) |
| 104–108 | axle boss (M8 bolt head captured in the shank's inner face) |
| 104–130 | wheel motor, on the shank ~94 mm from the axle, 25° above the shank line (outside the tire) |
| 109–129 | TPU tire on the printed hub (2 × 608 on a fixed M8 axle) |
| 132–142 | GT2 9 mm belt plane: motor 20T ↔ 48T teeth printed into the wheel hub |
| 131.5–147.5 | bought 20T pulley (16 mm long, **set-screw hub facing out**, away from the bell, so the motor stays close to the belt) |
| 149–155 | **wheel outrigger** (6 mm plate): bolts to both posts, holds the axle's outer end (square-ended slot, shimmed) and carries a **684ZZ bearing on the motor shaft end**, so belt pull bends the shaft between two bearings instead of hanging off the motor's own small bearings |

## Printed parts

| Body | Qty | Material | Print orientation |
|---|---|---|---|
| Hip module (HMI v1) | 1 | PETG / PLA+ | plate down, supports under the hip bearing seats |
| Hip pulley 80T | 2 | PETG | flat, outer flange down |
| Knee pulley 40T | 2 | PETG | flat |
| Thigh | 2 | **PAHT-CF** (PETG works for first tests: ~1.5° belt misalignment over bumps vs 0.4°), 5 walls, 40 % | flat |
| Shank (with 2 posts) | 2 | **PAHT-CF** (PETG: 0.6° motor-mount twist vs 0.2°), 5 walls, 40 % | flat, post up |
| Wheel outrigger | 2 | PAHT-CF / PETG | flat |
| Wheel hub (48T pulley built in, 2 × 608 seats) | 2 | PETG | tire side down; check the 608s press in and the belt seats on the teeth |
| Tire | 2 | TPU 95A | flat |

The pulley teeth are an approximation of the GT2 profile. Print one pulley first and check the belt
seats cleanly. If it doesn't, buy metal 80T/40T GT2 pulleys and bore or bolt them in place.

## Bought parts (per robot)

| Part | Qty | Notes |
|---|---|---|
| Standard-size 25T servo, ≥ 35 kg·cm (e.g. a "DS3235-class" digital servo) | 2 | Hip/height. Static load is ~11.9 kg·cm per side at the lowest stance, so 35 gives 2.9× margin. **Measure** the body, tab height and hole pattern (`hip_servo_*`). |
| 25T aluminium disc horn, Ø25, threaded M3 holes | 2 | 4 × M3 × 40 + the spline screw, all put in from the thigh's outer face |
| 6804-2RS bearing (20 × 32 × 7) | 4 | hips |
| 6802-2RS bearing (15 × 24 × 5) | 4 | knees |
| GT2 closed belt, 10 mm wide, 320 mm (160 teeth), rubber with glass/aramid cords | 2 | **Check you can buy this size.** If not, set `belt_teeth` to one you can and rerun: the leg length follows. |
| 625ZZ bearing + M5 × 25 + nut | 2 | belt fine-tension idler in the thigh slot (the main tension comes from the 0.3 mm pre-stretch) |
| M5 × 40 bolt + nyloc nut + 2 washers | 2 | knee: clamps knee pulley + shank hub (33 mm stack) |
| **5010 360KV outrunner** (the same motor as the caster base), on the shank, belt-driving the wheel 2.4:1 | 2 | Wheel no-load ~1665 rpm vs 1105 rpm for 25 km/h; ~2.3 A for 0.3 g of balancing torque. The motor carries only belt tension: wheel loads go into the axle. **Measure** its base bolt pattern (`motor_stator_pcd`), can diameter (58 or 50?) and length. |
| GT2 20T aluminium pulley for 9 mm belt, bore to suit the shaft, ≤ 16 mm long with ≤ 5 mm hub (`pulley20`, MEASURE) + GT2 9 mm × 256 mm (128T) closed belt | 2 + 2 | **Check you can buy this length.** Anything from 126T up works: the script moves the motor to suit. |
| 608 bearings + M8 × 70 hex bolt + 3 washers + M8 nyloc + Ø12/Ø8 × 20.5 mm spacer tube; 2 × (M3 × 8 + heat-set insert) per side (outrigger to posts); **684ZZ** (Ø4 shaft) or **685ZZ** (Ø5) per outrigger | 4 + 4 + 2 | Wheel axle: the bolt head is captured in a slot in the shank's inner face, the nyloc goes outside the hub. The axle slides ±2 mm to tension the belt; printed shims lock it in both slots (behind the head, and on the motor side of the axle in the outrigger). **The shaft bearing needs ≥ 22.1 mm of shaft past the bell (the 24 mm default is a guess): measure yours** and set `motor_shaft_d` / `motor_shaft_past_bell`; if it's shorter the script leaves the seat out and says so. |
| Magnetic encoder breakout (MT6701 / AS5047 class, ABI output) + diametric magnet glued to the motor's shaft end | 2 | In a 16 × 16 × 3 mm pocket in the shank under the motor; the wires run along a groove to the knee. **Measure your breakout** (`enc_board`). |
| FOC driver with current sensing (B-G431B-ESC1 class: listed 3S–6S, 40 A peak; verify before ordering) | 2 | Torque control from standstill with encoder feedback |
| M3 heat-set inserts | 8 | pulley mounts on the hip blocks |
| M3 screws | see HARDWARE.md | lengths worked out from the geometry |
| **3S LiPo ~36 × 106 × 26 mm (2200 mAh, ~25 min; a 1500 mAh pack also fits)** | 1 | The battery bay under the hip servos, open at the front: slide it in (40 mm of room under the front skid). One 15 mm velcro strap goes lengthwise round the whole tray on the centre line: under the floor between the guides, up the back, over the roof between the servos (12.6 mm gap), and down across the open mouth. XT30 lead up through the HMI port. |

## Assembly

Order matters, because the pulley and belt sit between the hip block and the thigh with 0.5 mm gaps.

1. **Hip servos.** Fit each one from the middle of the hip module, top face outward, into its pocket,
   and screw the tabs to the block (4 × M3 self-tapping). Fit the first one fully before the second:
   there are 46.5 mm of room to swing the second one in.
2. **Horns.** Centre each servo (power it with a 1500 µs signal). Push a disc horn onto the spline
   through the empty bearing seat from the outside (Ø25 passes the Ø27 bearing stop), turned so its
   holes are at 45°. Don't fit the spline screw yet.
3. **Hip bearings.** Press 2 × 6804 into the hip pulley's sleeve, against its stop.
4. **Hip pulley.** Slide the pulley's sleeve into the block bore and bolt it to the block face with 4 × M3 × 16 into the heat-set inserts.
   The heads sit in the thick outer flange.
5. **Knee.** Press 2 × 6802 into the thigh's knee seat.
6. **Belt and thigh.** Loop the belt around the hip pulley and around the 40T knee pulley. Hold the
   knee pulley under the thigh's knee seat and slide the thigh hub through the hip pulley bore and the
   hip bearings until it meets the horn.
7. **Shank.** First drop the M8 × 70 wheel-axle bolt into the hex slot in the shank's inner face (it can't go in once
   the shank is on the thigh); tape it in place. Then push the shank's hex key through the knee bearings into the knee pulley and clamp it
   with the M5 × 40 through-bolt. Before tightening, set the leg straight: with the thigh at angle a
   below horizontal (pointing back), the shank points forward at the same angle a below horizontal,
   and the axle sits directly under the hip. With the servo centred, a should be ~37° (the middle of
   the 5–70° range).
8. **Thigh screws.** With the servo powered and centred, let the hub centre itself in the bearings (the 4 mm screw holes let the horn float), then from the thigh's outer face fit 4 × M3 × 40 into the horn and the spline screw
   (~M3 × 45; check its thread depth). This clamps the hub shoulder and the horn onto the two bearing
   inner races. Snug the idler.
9. **Wheel drive.** Bolt the motor to the shank (4 × M3 × 10 from the inside, **finger-tight for now**, heads in the counterbores; glue the
   encoder magnet on the shaft end first, encoder board in its pocket). Fit the 20T pulley on the shaft, **hub outward**, in line with
   the wheel's teeth. The M8 × 70 is already in its slot (step 7). Stack washer, wheel (608, spacer tube, 608) and washer on the bolt. Loop the belt, slide the axle away
   from the motor until the belt is snug, fill the gap behind the head with printed shims, press the 684ZZ into the **outrigger**, fit it over the
   bolt end and the shaft end, shim the motor side of its axle slot, screw it to both posts (2 × M3 × 8 into the inserts),
   washer, nyloc. **Now tighten the 4 motor screws**: the motor centres itself on the outrigger bearing (the 3.4 mm
   holes give it the play), so the shaft isn't forced sideways. Spin the wheel by hand: it should turn freely. Stretch the TPU tire on.
10. **Battery and deck.** Battery into the bay from the front, 15 mm velcro strap lengthwise round the tray,
    harness up through the HMI port, then 4 × M3 × 8 up into the deck.

**ESP32 USB cable:** the ESP32 cradle takes the board either way round. Put the USB end **forward**:
it then has ~26 mm in front of it (there are only ~14 mm at the rear, against the lid). Or use a
right-angle micro-USB cable.

## Fusion: moving parts

The script builds components joined by as-built joints:

| Joint | Moves |
|---|---|
| Hip L / R | thigh (with its test belt) turns on the hip module |
| Knee L / R | shank + knee pulley + wheel motor turn on the thigh |
| Leg coupling L / R | **motion link**, knee = 2 × hip: the belt's 2:1 coupling, so dragging a thigh moves the wheel straight up and down |
| Wheel L / R | wheel hub + tire turn on the fixed axle in the shank foot |
| Pan | the GoPro mount turns on the deck |

If the message box says the motion link wasn't created (older Fusion), add it yourself: Assemble → Motion Link,
choose the Hip joint, then the Knee joint, and set 10 deg : 20 deg. Tick **Reverse** if the axle swings forward
or back instead of staying under the hip.

Printed TPU belts (`printed_belts=True`) are for checking fit and turning by hand only. TPU stretches enough to let
the legs sag under the robot's weight; buy a rubber GT2 belt for running.

## Electronics this drivetrain needs (not just CAD)

A two-wheeled robot only stands while a fast control loop holds it up. The current deck electronics
can't balance it yet:

- **Wheel motors.** 5010 360KV + magnetic encoder + current-sensing FOC driver (above). The AM32 ESCs that
  run the caster base won't balance it: they're sensorless, so they have no torque at standstill and react
  too slowly on a 50 Hz signal.
- **IMU.** Add an IMU (MPU-6050 / ICM-42688 / BNO085 class) mounted rigidly. Close to the axle line in
  the hip module is best.
- **Loop rate.** Run the balance loop at ≥ 200 Hz on a small MCU in the hip module (IMU, drivers and balance
  loop all live there). The deck sends a **speed** target on JST pin 2, **turn** on pin 3 and **height** on
  pin 5, so the harness is the same as for the caster base.
- **Fail-safe.** Firmware 2.1.0 "fails safe" to neutral, and for a balancing robot neutral means it
  falls over. The safe action becomes: lower the legs to the parked height (skids within 8°), then cut
  the motors. Treat it as a firmware design change.
- **Hip servos.** Driven by the hip-module MCU from the height command (one per leg, so it can also level
  the body). Power them from a 6–7.4 V BEC sized for 2 × ~3 A stall, not from the Pi's 5 V.
