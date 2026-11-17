# Caster drivetrain (v5): two belt-driven wheels + a swivel caster

Built by `honebot_chassis.py` when `drivetrain` is `'caster'` (fitted under the deck) or `'both'` (drawn
400 mm to the side of the robot). It follows `skatefollow/docs/robot_design.md` §2/§2a. It bolts to the
same deck as the ostrich legs through **HMI v1** (4 × M3 + 2 pegs + one harness), so you can swap between
the two.

## Layout

| | |
|---|---|
| Frame plate | 290 × 124 mm, 5 mm thick (HMI v1 on top), two stiffening ribs underneath. Print in PAHT-CF (or PETG for a first test). |
| Drive | 2 × **5010 360KV** outrunner → GT2 **20T** (bought, aluminium) → printed **48T** pulley bolted to the wheel core (3 × M4 × 65), **2.4:1** |
| Belt | GT2 **9 mm, 200 mm (100T)** closed loop, 65.4 mm between centres. The motor slides ±5 mm in slotted webs to tension it. |
| Wheels | Ø90 × 52 longboard wheels on 608 bearings, fixed **M8 threaded rod** axle with nuts each side of the two axle blocks, a 38 mm Ø12/Ø8 spacer tube inboard of each pulley |
| Caster | Bought **50 mm swivel caster** (robot_design suggested a printed fork with a longboard wheel; that's too tall to fit under the plate) (top plate 63 × 48, M4 holes 48 × 33, 64 mm tall): **measure yours** and set `cb_caster`. A Ø90 caster won't fit under the plate. |
| Battery | **3S 5000 mAh** (138 × 43 × 36 mm) in a cradle under the plate, held by a **20 mm velcro strap**: down a 3 mm channel beside the pack, under it, up the other side, and closed on the plate top behind the deck (slots at x −85). A bolt-on 4 mm **skid plate** (2 × M3 × 8 into inserts in the end ears) guards the pack from below, 22 mm above the ground. |
| Motor webs | 8 mm webs with **two gussets each** under the plate: belt tension can't lean the motor. The motor sits 3 mm from the belt plane (room for the 20T pulley's hub) to keep the shaft overhang short. |
| ESCs | 2 × AM32, zip-tied on top of the plate behind the deck (two rows of slots). Two 16 × 18 mm wiring pass-throughs next to them run the battery leads up and the ESC signal wires down to the HMI port. |
| Size | Wheelbase 230 mm, track 280 mm, 332 × 310 mm overall, ~1.96 kg |
| Speed | ~1415 rpm at the wheel loaded on 3S → **~6.7 m/s (24 km/h)**. Reprint the wheel pulley as 60T for ~19 km/h with more punch. |
| Centre of mass | ~41 % of the wheelbase ahead of the axle (robot_design targets 33 %; the battery is already as far back as the axle blocks allow) |

The GoPro's view clears this base up to ±72° of pan (the camera plan uses ±45°).
The speed figure is calculated, not measured.

Two departures from robot_design §2a, both on purpose:
- **The plate sits higher**, with its underside 66 mm above the ground instead of 55, so the battery under it clears the
  ground by 30 mm.
- **The motor sits 20° up and forward of the axle** (instead of 60 mm straight above), at the same 65 mm belt
  centre distance. This keeps the Ø58 cans below the lid line, so the GoPro doesn't need a taller bulkhead.

## Motors and controllers (as the advisor recommended)

| Drivetrain | Motor | Controller | Why |
|---|---|---|---|
| **Caster base** | 5010 360KV, belt 2.4:1 | **2 × AM32 ESC** (30–40 A, bidirectional mode), 50 Hz PWM from the ESP32 on JST pins 2/3. **Motor choice is open** (see HARDWARE.md §3): grip caps the belt pull below the preload, so a Ø4 shaft is OK at 103 MPa, but some 5010s have 3 mm shafts or no shaft past the bell. | Already supported by firmware 2.1.0 (`DIFF_DRIVE = true`), cheap; sensorless is fine for driving. |
| **Ostrich legs** | the same 5010 360KV on the shank, belt 2.4:1 to a wheel on its own axle | a **FOC driver with current sensing + a magnetic encoder** per motor (B-G431B-ESC1 class + MT6701/AS5047 class) | Balancing needs torque control from standstill: ~2.3 A for 0.3 g, and the wheel runs at ~1665 rpm no-load vs 1105 rpm needed. |

One motor type covers both drivetrains. **Check before ordering:** the B-G431B-ESC1 is listed as 3S–6S input, 40 A *peak*;
its continuous rating differs between sources. Get the encoder breakout's size and its ABI output
setting from its datasheet.

## Printed belts (test only)

`printed_belts=True` adds TPU belts (GT2 profile, 1.2 mm backing) for both drivetrains.
- **Good for:** checking fit and alignment, and turning things by hand.
- **Not good for driving:** TPU stretches about 10 % at these loads, so the teeth skip under motor torque. On the ostrich, a
  stretched coupling belt lets the legs sag out of line.
- **For driving:** buy glass- or aramid-cord rubber GT2 belts (9 mm × 200 mm here, 10 mm × 320 mm for the legs).
- **Printing:** print the TPU belt flat in 95A TPU, slowly.

## Fusion: moving parts

| Joint | What moves |
|---|---|
| Drive wheel L / R | each wheel and its 48T pulley turn on the fixed axle |
| Caster swivel | the caster turns on its kingpin |

The plate, webs, axle blocks, motors (reference) and belts are one rigid component. Use **Assemble → Drive Joints**, or
drag a wheel.

## Assembly

1. No inserts on this base: the 4 × M3 × 8 HMI screws go up into the deck's inserts.
2. Thread the M8 rod through both axle blocks with a nut on each side of each block. Add the spacer tube, the
   pulley + wheel (pulley bolted to the wheel core), a washer and an M8 nyloc.
3. Bolt each motor to its web (4 × M3 × 12 in the slots, base against the web, shaft outboard). Fit the 20T pulley in line
   with the 48T, loop the belt and slide the motor along the slots to tension it. **Do this before fitting the deck**:
   one screw per motor sits 4 mm from the deck's side wall, so re-tensioning later means taking the deck off (4 screws).
4. Bolt the caster on (4 × M4) and zip-tie the ESCs on.
5. Push the battery up into the cradle, thread the strap through the plate slots behind the deck and close it on top, bolt the skid plate on, run the XT30 + JST-XH up through the HMI port, and fit the deck (4 × M3 × 8 from below).
6. On the ESP32 set `DIFF_DRIVE = true`; on the Pi set `drive_mode` for differential drive.
