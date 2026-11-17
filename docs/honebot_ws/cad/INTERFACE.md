# honebot Module Interface — HMI v1

The electronics/camera **deck** is one module. Every **drivetrain** (motors, wheels, battery,
ESCs) is another. They meet at this interface, so any drivetrain that follows it can be
bolted on, tested, and swapped without touching the deck, the electronics or the camera
geometry.

Source of truth for the numbers: the `hmi_*` parameters in
`cad/fusion/honebot_chassis/honebot_chassis.py`. If any of these change, call it **HMI v2**
and update this file, because old adapters will no longer fit.

## Datum

- Origin: **deck centre** (plan view), on the **deck underside** (z = 0).
- **+X = forward** (the camera/HC-SR04 end), +Y = left, +Z = up.
- The drivetrain lives entirely at **z ≤ 0**.

## Mechanical

| Feature | Spec |
|---|---|
| Fasteners | **4 × M3**, pattern **100 × 90 mm** at (±50, ±45). Screws go **up from below** through the adapter into **M3 heat-set inserts** (5.7 mm long, in a Ø4.0 × 7.2 mm hole so the screw tip passes through) in bosses on the deck. Nothing inside the deck has to come out to swap a drivetrain. The space directly under each screw must stay clear so a hex key can reach it. |
| Screw length | Adapter 5 mm thick with a **3.2 mm** counterbore (the 3.0 mm socket head sits flush) → **M3 × 8** socket head (6.2 mm into the deck). Knurled M3 thumbscrews make swaps tool-free. |
| Locating | Two Ø4.0 × 2.5 mm **pegs on the adapter**: one at **(−50, 0)** into a **Ø4.3 round hole**, one at **(+50, 0)** into a **4.3 × 8 mm slot along X**. Round + slot = exact, repeatable position without binding. |
| Mating face | Deck underside is flat. Adapter top face must be flat inside the deck outline (120 × 110 mm) apart from the pegs. |
| Harness port | **18 × 26 mm** opening centred at **(+25, −32)** in both the deck floor and the adapter. All drivetrain wiring passes through here. |
| Keep-out | No adapter or drivetrain parts above z = 0 inside the deck outline. |

## Electrical (one harness through the port)

Plug-and-play: unplugging these two connectors is all that's needed electrically.

| Connector (drivetrain side → deck side) | Pin | Signal |
|---|---|---|
| **XT30** (battery power) | + | VBAT (3S, 9–12.6 V) → deck buck converter → Pi 5 (5 V / 5 A) |
| | − | GND |
| **JST-XH 6-pin** (control) | 1 | GND |
| | 2 | ESC / channel A signal ← ESP32 GPIO 25 (left wheel, or throttle) |
| | 3 | ESC / channel B signal ← ESP32 GPIO 26 (right wheel, or steering) |
| | 4 | +5 V from the ESC BEC (optional, for servos) — *leave unconnected if unused* |
| | 5 | Spare (kill switch). **Ostrich: ride-height command** (50 Hz PWM) |
| | 6 | Spare (telemetry). **Ostrich: "balancing OK" from the hip-module MCU** |

**Per drivetrain:**
- **Caster base:** pins 2/3 go straight to two AM32 ESCs (A = left, B = right).
- **Ostrich legs:** pins 2/3 carry *speed* and *turn* targets to the balance MCU in the hip module, pin 5
  carries the ride height, and pin 6 reports back.

Both drivetrains carry their own 3S battery and send it up the XT30.

Channel meaning is set per drivetrain by the Pi's `drive_mode` and the ESP32's `DIFF_DRIVE`
flag (differential: A = left, B = right; Ackermann: A = throttle, B = steering). Signals are
50 Hz, 1000–2000 µs, 1500 µs = neutral, forced to neutral by the ESP32 watchdog.

## Revision note (v5 script)

Still **HMI v1**: every mating dimension is unchanged. The v5 deck has deeper insert holes
(7.2 mm, boss top 9.0 mm) and adapters have a 3.2 mm counterbore so an M3 × 8 head sits flush.
A v4 adapter (2.5 mm counterbore) still fits a v5 deck; its screw heads just stand 0.5 mm proud.
Don't pair a v5 adapter with a v4 deck: the M3 × 8 tip would bottom out in the shallower hole
(use M3 × 6 there).

The first real drivetrain is the **ostrich hip module** (`drivetrain='ostrich'`, see OSTRICH_LEGS.md),
and the second is the **caster base** (`drivetrain='caster'`, see CASTER_BASE.md). One deck fits both.
It's an HMI v1 plate with the hip blocks, battery bay and skids underneath.

## Making an adapter for a new drivetrain

1. In `honebot_chassis.py`, set `build_deck=False`, `build_pan_mount=False`, `build_lid=False`,
   `drivetrain='none'`, `build_adapter=True`.
2. Set `adapter_size` to cover the drivetrain (it can be much bigger than the deck) and
   `adapter_t` if it needs to be stiffer (then re-check screw length).
3. Bolt motors, brackets and battery straps to the 10 mm M3 grid (drop M3 nuts into the hex
   pockets in the top face first; the top must sit flush under the deck), or add
   drivetrain-specific features to the adapter body in Fusion after running the script.
4. Set `drive_envelope` to the drivetrain's outline and its highest point (z relative to
   the deck underside, negative = below it), and rebuild the deck. The camera tower grows
   automatically if the wheels or battery would appear in the GoPro's shot. For example, a
   400 × 300 mm drivetrain level with the deck underside raises the tower from 42 mm to
   ~115 mm.
5. Print, press nothing (the inserts live in the deck), fit the pegs into the deck, add 4
   screws, and plug in the XT30 + JST-XH.

## Swap procedure

1. Power off (unplug the XT30).
2. Unplug the JST-XH.
3. Remove 4 × M3 screws from below; lift the deck off.
4. Seat the next adapter's pegs, refit the screws, reconnect.
5. On the Pi, set the matching `drive_mode`, and on the ESP32 the matching `DIFF_DRIVE` /
   throttle limits, before driving.
