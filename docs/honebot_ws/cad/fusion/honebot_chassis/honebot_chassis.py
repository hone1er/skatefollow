"""honebot chassis v5 — Fusion 360 script.

An electronics/camera DECK that bolts through the "HMI v1" module interface
(see INTERFACE.md) to a two-wheeled, self-balancing "ostrich" drivetrain.

Printable bodies (each is its own body; right-click > Save As Mesh to print):

  DECK MODULE
  1. "Chassis deck" - rounded tray (120 x 110 mm) with
       * Raspberry Pi 5 on 10 mm standoffs (58 x 49 mm pattern, ports over the walls)
       * ESP32 DevKit slide-in cradle (fit the board USB-forward: ~26 mm for the plug)
       * full-width front BULKHEAD carrying the HC-SR04 (front face) and the
         standard-size pan servo (MG996R / DS3218, hanging from its roof), chamfered front edge
       * HMI v1: 4 insert bosses, locating hole + slot, harness port
  2. "Electronics lid" - vented cover. Slides on from behind, front tongues hook
       under keeper ledges in the bulkhead, 2 x M3 at the rear.
  3. "Pan horn adapter" + "GoPro buckle" - the adapter screws to the servo's 25T disc
       horn; the 3-prong GoPro buckle bolts onto it with 3 screws outside the prongs.
       Open all round so the GoPro can shed heat.

  SG90 BENCH TEST (pan_servo='sg90'): "SG90 test plate" bolts over the standard servo pocket and
  "Pan horn adapter (SG90 bench test)" takes its 2-arm horn; the GoPro buckle is shared.

  CASTER DRIVETRAIN (drivetrain='caster' or 'both'): HMI v1 frame plate with 2 belt-driven
  longboard wheels (5010 + 20T/48T GT2), a bought swivel caster and a 3S 5000 mAh battery
  cradle. See CASTER_BASE.md. Swappable with the ostrich legs.

  Components + as-built joints (joints=True): drag a thigh, wheel, the caster or the GoPro
  mount, or use Assemble > Drive Joints. A motion link couples knee = 2 x hip.

  OSTRICH DRIVETRAIN (drivetrain='ostrich' or 'both')
  4. "Hip module (HMI v1)" - bolts under the deck with the 4 HMI screws. Carries
       both hip servos, the hip bearings, the battery (low, as ballast) and two
       parking skids.
  5. Per side (L / R): "Hip pulley", "Thigh", "Knee pulley", "Shank",
       "Wheel hub", "Tire (TPU)"; plus "Wheel motor (reference)" (buy, don't print).

How the legs work (reverse-knee "ostrich" leg, side view):

        hip o-----(servo drives the thigh)
             \\  thigh (L)
              \\
         knee  o   <- knee points backwards
              /
             /  shank (L)
     axle  (O)--(M)  <- wheel on 2 x 608 on a fixed M8 axle; the 5010 (M) sits on the shank
                        and drives it with a GT2 20T -> 48T belt (wheel loads never reach the motor)

  Thigh and shank are the same length and a GT2 belt couples them 2:1 (80T
  pulley fixed to the hip module, 40T pulley fixed to the shank). When the hip
  servo turns the thigh down by a, the shank turns up by a, so the axle stays
  EXACTLY under the hip and moves straight up/down:  axle = hip - 2 L sin(a).
  One servo per leg sets the ride height (and can level the body on a slope).
  The wheels never move fore/aft, so height changes don't upset the balance.

  The leg length is solved from belt_teeth so a stock closed GT2 loop fits.
  The hip x position is CALCULATED from a mass table so the centre of mass sits
  over the axle at the drawn stance (the GoPro is heavy and far forward).

Camera placement: the tower height is calculated so no part of the chassis
(or the wheels, at any stance from leg_film_min_deg up) is in the GoPro's view at
any pan angle within +/- pan_range_deg.

Coordinates (mm): origin = deck centre, deck underside. +X forward, +Y left, +Z up.
Edit P and rerun (it always creates a new design). Everything marked MEASURE
should be checked against your actual parts before printing.

Run: Utilities > Add-Ins > Scripts and Add-Ins (Shift+S) > select honebot_chassis > Run.
"""

import math
import traceback

try:  # Only inside Fusion. layout() works without it (used by the checker).
    import adsk.core
    import adsk.fusion
except ImportError:  # pragma: no cover
    adsk = None

# ---------------------------------------------------------------------------
# Parameters (mm, degrees, grams)
# ---------------------------------------------------------------------------
P = dict(
    # Print fits (PETG/PAHT on a tuned printer). Holes print small and shafts print fat.
    fit_press=0.1,              # bearing seats: hole = bearing OD + this (snug press)
    fit_shaft=0.1,              # printed shafts into bearing bores: shaft = bore - this (slide fit)

    # Deck
    deck_length=120.0,          # X
    deck_width=110.0,           # Y
    wall_height=12.0,           # floor underside to wall top
    floor_t=3.0,
    wall_t=3.0,
    corner_r=6.0,               # plan-view corner radius used on deck, lid, plates (one style everywhere)

    # ---- Module interface "HMI v1" (see INTERFACE.md) ----
    hmi_bolts=[(50.0, 45.0), (50.0, -45.0), (-50.0, 45.0), (-50.0, -45.0)],  # 100 x 90 mm, M3
    hmi_insert_d=4.0,           # M3 heat-set insert hole, pressed in from the underside
    hmi_insert_depth=7.2,       # 5.7 mm insert + room for the screw tip to pass through it
    hmi_boss_od=9.0,
    hmi_boss_top=9.0,           # boss top above the floor underside (1.8 mm skin over the hole)
    hmi_peg_round=(-50.0, 0.0),
    hmi_peg_slot=(50.0, 0.0),
    hmi_peg_d=4.0,
    hmi_peg_h=2.5,              # must stay below floor_t
    hmi_peg_hole_d=4.3,
    hmi_slot_len=8.0,
    hmi_port=(25.0, -32.0, 18.0, 26.0),   # harness port: centre x, centre y, length X, width Y

    # ---- Plate that carries the HMI (used by the template adapter and the hip module) ----
    adapter_t=5.0,
    adapter_screw_clear=3.4,
    adapter_cbore_d=6.5,
    adapter_cbore_depth=3.2,    # M3 socket head (3.0) sits flush; 1.8 mm under the head -> M3 x 8
    # Template adapter (for other drivetrains)
    adapter_size=(120.0, 110.0),
    adapter_grid_pitch=10.0,
    adapter_grid_hole_d=3.4,
    adapter_nut_af=5.8,         # M3 nut pockets in the TOP face (the top sits flush under the deck)
    adapter_nut_depth=2.6,
    adapter_grid_keepout=7.0,
    adapter_edge_margin=6.0,
    adapter_explode=0.0,
    drive_envelope=(-60.0, -55.0, 60.0, 55.0, 0.0),  # extra camera obstacle when build_adapter=True

    # ---- Electronics lid ----
    lid_wall_t=2.4,
    lid_top_t=2.4,
    lid_fit=0.4,
    lid_skirt_t=1.6,
    lid_skirt_depth=3.0,
    lid_skirt_fit=0.3,
    lid_slide=34.0,             # set the lid on the deck walls this far back, slide forward
    lid_tongues=[(41.0, 51.5), (-51.5, -41.0)],  # front tongues (y ranges), next to the bulkhead side walls
    lid_tongue_len=6.0,
    lid_tongue_gap=0.3,         # under the keeper ledges
    lid_keeper_t=3.0,           # ledges on the bulkhead side walls that hold the tongues down
    lid_chamfer=1.5,            # top outer edges
    pi_cooler_h=14.0,           # Raspberry Pi Active Cooler height above the PCB — MEASURE
    fan_air_gap=8.0,
    usb_air=3.0,
    gpio_cable_h=18.5,
    esp_wiring_h=21.0,          # pins-up: header 8.5 + Dupont housing + wire bend
    esp_usb_plug=(18.0, 11.0, 7.5),  # USB plug body at the ESP32's front end (length, width, height) — a 90 deg plug is shorter
    lid_posts=[(-50.0, -45.0), (-50.0, -12.0)],
    lid_post_od=8.0,
    lid_post_gap=0.3,
    lid_screw_d=3.4,
    lid_rear_port=(-12.0, 52.0, 33.0),   # from y -12: also lets the ESP32's USB cable out to a Pi USB port
    lid_side_port=(-20.0, 25.0, 22.0),
    lid_top_vent_zone=(-31.0, 0.0, 24.0, 44.0),
    lid_top_vent_slot=(15.0, 2.5),
    lid_top_vent_pitch=5.0,
    lid_side_vent_x=(-48.0, 23.0),
    lid_rear_vent_y=(-48.0, -18.0),
    lid_vent_w=2.5,
    lid_vent_pitch=5.0,
    lid_vent_z=(15.0, 31.0),

    # What to build
    build_deck=True,
    build_pan_mount=True,
    build_lid=True,
    drivetrain='both',          # 'ostrich', 'caster', 'both' (caster base drawn beside the robot, y - 400 mm) or 'none'
    caster_draw_offset=-400.0,
    build_adapter=False,        # generic template adapter (occupies the same space as the hip module)
    pan_servo='sg90',           # 'sg90' = bench test plate + SG90 horn adapter; 'standard' = MG996R/DS3218
    printed_belts=True,         # TPU test belts (for checking motion; buy rubber GT2 belts for driving)
    joints=True,                # components + as-built joints so the mechanism moves in Fusion

    # Raspberry Pi 5
    pi_board_origin=(-56.0, -6.0),
    pi_board=(85.0, 56.0),
    pi_hole_dx=58.0,
    pi_hole_dy=49.0,
    pi_hole_inset=3.5,
    pi_hole_from_rear=True,
    pi_standoff_od=6.0,
    pi_standoff_h=10.0,
    pi_hole_d=2.2,              # M2.5 self-tapping into the printed standoff (2.5 would not bite)
    pi_tallest=16.0,

    # ESP32 DevKitC-style board
    esp_center=(-16.0, -31.0),
    esp_board_l=54.4,
    esp_board_w=27.9,
    esp_pcb_t=1.6,
    esp_pcb_z=9.0,              # board UPSIDE DOWN (pins up for plug-in jumpers); room underneath for the USB plug
    esp_ledge_w=2.5,
    esp_ledge_half_y=10.0,
    esp_ledge_y=(10.5, 14.0),   # 4 corner pads (clear of the WROOM module and the USB socket underneath)
    esp_lip_in=14.4,            # = fence: no lip over the board, so Dupont housings fit on the pin rows
    esp_relief=14.4,
    esp_rail_t=3.0,
    esp_lip_t=0.5,
    esp_fit=0.4,

    # Front bulkhead
    tower_width=110.0,
    tower_depth=30.0,           # deep enough for a standard-size pan servo
    tower_roof_t=4.0,
    tower_min_margin=2.0,
    tower_chamfer=3.0,          # front-top edge

    # HC-SR04 in the bulkhead front face
    sr04_spacing=26.0,          # transducer centre-to-centre — MEASURE
    sr04_hole_d=16.5,
    sr04_board_h=20.0,
    sr04_header_below=8.0,      # room under the board for its 4 header pins + soldered wires — MEASURE
                                # (use ~20 for straight Dupont jumpers; the bulkhead grows to suit)
    sr04_protrusion=10.0,

    # Pan servo: STANDARD size, metal gear (MG996R / DS3218 class, 25T spline) — the 9 g SG90 is for the
    # bench only (camera_plan.md). Hangs from the bulkhead roof, shaft up, tabs on the roof. MEASURE yours.
    servo_body=(21.0, 41.3),    # pocket X x Y (body ~20.2 x 40.5 + 0.4/side)
    servo_cable_slit=(5.0, 4.0),  # slit at the pocket end the cable exits (width, length) so the servo drops in
    servo_shaft_offset=10.0,    # shaft from the body centre, along the long axis
    servo_tab_drop=27.5,        # tab underside to body bottom
    servo_screw_spacing=49.5,   # tab holes along the long axis (Y)
    servo_screw_across=10.0,    # tab holes across (X)
    servo_screw_d=4.0,          # M3 heat-set inserts from above, in bosses under the roof (nuts would be unreachable)
    servo_insert_boss=(3.9, 4.0),   # boss radius, depth below the roof
    servo_axis_x_from_front=16.0,

    # SG90 bench-test plate (bolts to the standard servo's 4 tab holes; SG90 numbers from v4)
    sg90_body=(13.0, 23.6),
    sg90_cable_slit=(4.0, 2.8),   # runs through the tab screw hole at that end: use M2 screw + nut there
    sg90_shaft_offset=5.5,
    sg90_tab_drop=15.9,
    sg90_screw_spacing=27.8,
    sg90_screw_d=1.8,           # M2 self-tapping
    sg90_horn_top_above_tab=12.0,
    sg90_horn_arm=(33.0, 6.5, 1.6),
    sg90_horn_hub_d=8.0,
    sg90_horn_screw_x=(-10.0, 10.0),
    sg90_horn_screw_d=2.2,
    sg90_plate_lip=2.0,         # locating lip that drops into the big servo pocket

    # Pan mount = 2 parts: an adapter screwed to a 25T aluminium disc horn, and a GoPro buckle
    # bolted onto the adapter outside the prongs (so every screw can be reached from above)
    horn_top_above_roof=18.0,   # roof top (tab seat) -> top face of the disc horn — MEASURE
    horn_d=25.0,
    horn_pcd=14.0,              # 4 x M3 (button head) into the horn's threaded holes
    horn_pocket_depth=2.0,
    mount_disc_d=44.0,
    adapter_disc_t=6.0,
    buckle_base_t=4.0,
    buckle_screw_pcd=32.0,      # 3 x M3 x 10 down into heat-set inserts in the adapter
    mount_disc_t=10.0,          # adapter + buckle base (used for the lens height)

    # GoPro 3-prong buckle
    gp_prong_t=3.0,
    gp_gap=3.3,
    gp_prong_len=14.0,
    gp_hole_z=9.0,
    gp_hole_d=5.3,

    # Camera / view check
    cam_hfov_deg=118.0,
    cam_vfov_deg=69.0,
    pan_range_deg=60.0,         # camera_plan.md pans +/-45 deg; checked to 60. Past ~72 deg the caster
                                # base's rear wheels reach the corner of the frame
    lens_forward=20.0,
    lens_above_disc_top=47.0,
    view_margin=5.0,

    # ---- Ostrich legs -------------------------------------------------------
    # Leg angle a = thigh angle below horizontal (thigh runs back-down from the hip).
    leg_len=None,               # hip-knee = knee-axle (equal, for the straight-line motion). None = solved
                                # from belt_teeth so a stock closed GT2 loop fits exactly (~99 mm)
    belt_teeth=160,             # GT2 closed-loop belt size (teeth = length / 2 mm) — check availability
    belt_pretension=0.15,       # centre distance is made this much longer than nominal to pre-tension the belt
    leg_stance_deg=30.0,        # pose the legs are drawn in (also the stance used for the balance point)
    leg_park_deg=5.0,           # lowest: robot parked on its skids
    leg_film_min_deg=15.0,      # lowest stance that must give a clean camera shot
    leg_max_deg=70.0,           # highest stance
    hip_z=-27.0,                # hip axis below the deck underside
    hip_x=None,                 # None = put the axle under the centre of mass (calculated)
    hip_face_y=69.0,            # outer face of the hip module (hip pulley seat), each side
    hip_block_half_z=26.0,
    hip_block_half_x=31.0,
    hip_bearing=(20.0, 32.0, 7.0),   # 6804-2RS (id, od, width), 2 per hip
    hip_bearing_n=2,
    hip_lip=(27.0, 2.0),        # bearing stop: hole d, thickness
    # Hip servo: standard-size, 25T spline, >= 35 kg.cm (e.g. a "DS3235-class" digital servo) — MEASURE
    hip_servo_body=(40.5, 20.2, 38.0),   # length (X), width (Z), height (top face to bottom, along Y)
    hip_servo_clear=0.8,
    hip_servo_spline_offset=-10.0,       # body centre relative to the spline along X (negative = body ahead of the
                                         # hip axis, which keeps the hip blocks clear of the rear HMI screws)
    hip_servo_tab_from_top=11.5,         # mounting-tab face below the top face
    hip_servo_tab_holes=(49.5, 10.0),    # tab hole pattern (along X, along Z)
    hip_servo_tab_hole_d=2.6,            # M3 self-tapping into the block
    hip_servo_horn_stack=7.0,            # servo top face -> outer face of the 25 mm aluminium disc horn
    hip_servo_spline_h=4.5,              # spline height above the top face (for the insertion check)
    hip_servo_horn_d=25.0,
    hip_servo_horn_pcd=14.0,             # 4 x M3 horn screws, put in from the thigh's outer face
    hip_servo_torque_kgcm=35.0,
    # GT2 belt coupling (2:1)
    gt2_pitch=2.0,
    belt_w=10.0,                # 10 mm GT2 closed loop (length is calculated)
    pulley_flange=1.5,
    pulley_outer_flange=3.5,    # thicker outer flange holds the hip-pulley screw heads clear of the teeth
    pulley_hip_teeth=80,        # fixed to the hip module
    pulley_knee_teeth=40,       # fixed to the shank
    pulley_hip_bore=26.0,
    pulley_screw_pcd=44.0,      # 4 x M3 into heat-set inserts in the hip block
    pulley_sleeve_t=2.0,        # the hip pulley carries both hip bearings in a sleeve (printed round, coaxial)
    side_gap=0.5,               # running clearance between neighbouring parts along Y
    # Thigh
    thigh_t=10.0,
    thigh_r_hip=18.0,           # big enough for a wire tie slot near the hip axis
    thigh_r_knee=16.0,
    thigh_hub_d=20.0,           # through the hip bearings
    thigh_shoulder_d=24.0,      # presses the outer bearing's inner race
    knee_bearing=(15.0, 24.0, 5.0),  # 6802-2RS, 2 per knee (fills the thigh thickness)
    idler_slot=(8.0, 12.5, 5.3),     # belt fine-tension: M5 + 625 bearing, slot from / to (offset from link axis)
    # Shank
    shank_t=9.0,                # stiff enough that belt tension can't twist the motor mount (print in PAHT-CF)
    shank_r_knee=12.0,
    shank_r_foot=22.0,
    knee_hub_d=15.0,
    knee_hex_af=12.0,           # hex key into the knee pulley
    knee_bolt_d=5.3,            # M5 through-bolt clamps pulley + shank
    # Wheel drive: 5010 360KV outrunner on the shank (the same motor as the caster base) driving the wheel
    # through a GT2 20T -> 48T belt (2.4:1). The wheel turns on 2 x 608 bearings on a fixed M8 axle in the
    # shank foot, so knocks and falls load the axle, not the motor. Magnetic encoder (MT6701 / AS5047 class)
    # under the motor's shaft end + FOC driver (B-G431B-ESC1 class). MEASURE yours.
    motor_kv=360.0,
    motor_can_d=58.0,           # robot_design says 58; some sellers list 50 — MEASURE
    motor_h=26.0,               # base face to bell face
    motor_stator_pcd=25.0,      # 4 x M3 into the stator base — MEASURE
    motor_center_bore=8.0,      # shaft end / circlip clearance
    motor_screw_d=3.4,
    motor_head_cbore=(6.0, 3.2),
    axle_slot=2.0,              # +/- along the belt: the wheel axle slides in the shank to tension the belt
    enc_board=(16.0, 16.0),     # encoder breakout in a pocket in the shank face, under the motor's shaft end
    enc_pocket_depth=3.0,
    wire_groove=(8.0, 3.5),     # motor + encoder wires to the knee (width, depth)
    foot_belt=(128, 9.0),       # GT2 closed loop teeth (256 mm), width — check availability; >= 126T works
    foot_teeth=(20, 48),
    pulley20=(16.0, 5.0),       # bought GT2 20T for a 9 mm belt: overall length, set-screw hub length — MEASURE. The toothed
                                # part (with flanges) is centred on the belt; the hub faces OUT (away from the bell) on both
                                # drivetrains, so the motor sits close to the belt and the shaft overhang stays short        # motor pulley (bought, aluminium), wheel pulley (printed into the hub)
    foot_motor_angle=25.0,      # motor sits this far (deg) off the shank line, upward, to clear the knee bolt
    foot_boss=(13.0, 4.0),      # axle boss on the shank's outer face: radius, height
    axle_head=(13.0, 5.5),      # M8 bolt head captured in a slot in the shank's inner face: across flats, depth
    motor_bumper=(7.0, 3.0),    # two posts beside the wheel motor (radius, gap to the can), one each side of the belt:
                                # they land before the 20T pulley in a side fall and carry the outrigger
    post2_deg=140.0,            # second post: this far round the motor from the first (clears tire, knee, groove, belt)
    outrigger=(6.0, 1.6),       # bolt-on plate over the foot: 2 posts + motor shaft end + axle end (thickness, washer gap)
    # Outboard support for the motor shaft end: a small bearing in the outrigger over the 20T pulley, so belt pull
    # bends the shaft between two supports instead of as a cantilever off the motor's own small bearings.
    shaft_support=True,
    motor_shaft_d=4.0,          # MEASURE: 5010 shafts are 4 or 5 mm
    motor_shaft_past_bell=24.0, # MEASURE (unmeasured default): shaft length beyond the bell face; the message box says what's needed
    shaft_bearing=(4.0, 9.0, 4.0),  # 684ZZ (id, od, width) for a 4 mm shaft; use 685ZZ (5, 11, 5) for a 5 mm shaft
    bearing_608=(8.0, 22.0, 7.0),
    axle_spacer_od=12.0,        # tube between the two 608s (Ø12 / Ø8)
    # Wheel (printed hub with the 48T pulley built in + TPU tire)
    wheel_d=120.0,
    tire_t=8.0,
    tire_w=20.0,
    wheel_light_holes=6,
    # Battery bay (in the hip module, under the servos) — MEASURE your pack
    battery=(36.0, 106.0, 26.0),     # X, Y, Z: 3S 2200 mAh LiPo class (~25 min); a 1500 mAh pack also fits
    tray_wall=2.5,
    tray_strap_w=15.0,          # velcro strap round the tray (lengthwise, on the centre line, between the servos)
    tray_floor=3.0,
    # Electronics in the hip module (balance MCU, 2 FOC drivers, servo BEC) on a web at the front, IMU on the axle line
    hip_ebay=(42.0, 44.5, 38.0, -48.0),   # web x0, x1, half-width (Y), bottom z
    imu_holes=15.0,             # IMU board hole spacing along X — MEASURE (M2.5 self-tapping)
    hip_wire_pass=(34.0, 54.0, 57.0, 71.0),  # leg wires drop into the electronics bay here: slots open to the plate
                                # edge, 20 x 12 mm, so wires lift out without unplugging (x0, x1, |y|0, |y|1)
    hip_tie_anchors_x=(-38.0, 18.0),         # zip-tie anchors on the hip plate beside the deck (keep leg wires above the belt)
    hip_fairlead=(6.0, 93.0, 1.5, 7.0),      # arm over the hip pulley at hip_x: half-width, reach (y), z0, z1
    deck_tie_anchors_x=(-30.0, 0.0, 22.0),   # zip-tie anchors on the deck floor between the ESP32 and the Pi
    # Deck power electronics: 5 V 5 A buck for the Pi + 6 V BEC for the pan servo, zip-tied to a web in the bulkhead
    power_web=(40.0, 42.0, 31.0),   # x0, x1, top z
    # Parking skids
    skid_x=(-52.0, 57.0),       # front skid far enough forward to slide the battery out under it
    skid_w=14.0,
    skid_r=7.0,
    park_tilt_deg=8.0,          # robot tips this far onto a skid when parked

    # ---- Caster drivetrain (robot_design.md §2a): 2 belt-driven longboard wheels + a swivel caster ----
    # 5010 360KV outrunners, GT2 20T -> 48T printed pulley, AM32 ESCs in bidirectional mode on 50 Hz PWM.
    cb_plate=(-150.0, -62.0, 140.0, 62.0),  # frame plate outline (HMI v1 on top, deck centred at x = 0)
    cb_ground_clear=66.0,       # plate underside above the ground (§2a says 55; raised so the battery clears 26 mm)
    cb_axle_x=-125.0,
    cb_wheel=(90.0, 52.0),      # longboard wheel diameter, width
    cb_wheel_axle_h=45.0,
    cb_wheel_inner_y=114.0,     # wheel inner face
    cb_pulley=(48, 12.0, 6.0),  # printed wheel pulley: teeth, width, adapter disc thickness
    cb_pulley_disc_d=60.0,
    cb_pulley_bolts=(46.0, 4.5),  # 3 x M4 x 65 into the wheel core: PCD, hole
    cb_motor_teeth=20,
    cb_belt=(100, 9.0),         # GT2 closed loop 200 mm, 9 mm wide
    cb_belt_angle_deg=20.0,     # motor centre direction from the axle, above horizontal, forward
    cb_tension_slot=10.0,
    cb_motor=(58.0, 26.0),      # 5010 can diameter, base-to-bell length — MEASURE
    cb_motor_base_y=67.0,       # motor base face (outboard face of the web): 3 mm from the bell to the 48T pulley's plane, for the 20T hub
    cb_motor_holes=(25.0, 3.4), # base screw PCD, hole — MEASURE (often 16 x 19 mm; edit to suit)
    cb_web_t=8.0,               # thick enough that the motor sits close to the belt (short shaft overhang)
    cb_axle_block=(28.0, 44.0, 58.0),  # x length, y from, y to
    cb_axle_d=8.4,              # M8 threaded rod, nuts both sides of each block
    cb_caster_x=105.0,
    cb_caster=(63.0, 48.0, 64.0),      # bought 50 mm swivel caster: top plate X, Y, overall height — MEASURE
    cb_caster_holes=(48.0, 33.0, 4.5), # top-plate bolt pattern X, Y, hole (M4)
    cb_caster_swing=45.0,
    cb_battery=(138.0, 43.0, 36.0),    # 3S 5000 mAh LiPo: X, Y, Z — MEASURE
    cb_cradle_wall=2.5,
    cb_cradle_depth=39.0,       # battery + 3 (strap under the pack); closed by a bolt-on skid plate
    cb_strap_gap=3.0,           # channel each side of the pack for the velcro strap
    cb_strap_x=-85.0,           # strap slots through the plate, behind the deck (strap closes on the plate top)
    cb_skid_plate_t=4.0,
    cb_rib_y=(32.0, 35.0),
    cb_rib_depth=6.0,
    cb_esc_slots_x=(-140.0, -112.0),  # zip-tie slot rows for the 2 ESCs on top of the plate, behind the deck
    cb_mass=dict(plate=260, motor=100, esc=30, battery=400, wheel=140, caster=120, axle=100),

    # Mass table (grams) for the balance point — edit if your parts differ
    mass=dict(deck=120, lid=50, pi=70, esp=10, sr04=9, pan=75, gopro=160, power=45,
              hip_module=185, hip_servo=62, battery=180, hip_pulley=45, ebay=60,
              thigh=35, shank=25, knee=20, motor=100, wheel=70),
)


# ---------------------------------------------------------------------------
# Geometry (pure Python)
# ---------------------------------------------------------------------------
def _rect_pts(r, step=2.0):
    x0, y0, x1, y1 = r
    nx = max(1, int((x1 - x0) / step)); ny = max(1, int((y1 - y0) / step))
    pts = [(x0 + (x1 - x0) * i / nx, y) for i in range(nx + 1) for y in (y0, y1)]
    pts += [(x, y0 + (y1 - y0) * j / ny) for j in range(ny + 1) for x in (x0, x1)]
    return pts


def camera_clearance(axis, obstacles, p):
    """Minimum lens height so no obstacle is in view at any pan angle."""
    th = math.radians(p['cam_hfov_deg'] / 2)
    tv = math.tan(math.radians(p['cam_vfov_deg'] / 2))
    rng = p['pan_range_deg']
    need, worst = 0.0, None
    pts = [(pt, z, name) for name, r, z in obstacles for pt in _rect_pts(r)]
    for k in range(int(2 * rng) + 1):
        phi = math.radians(-rng + k)
        lx = axis[0] + p['lens_forward'] * math.cos(phi)
        ly = axis[1] + p['lens_forward'] * math.sin(phi)
        for (x, y), z, name in pts:
            dx, dy = x - lx, y - ly
            d = math.hypot(dx, dy)
            if d < 1e-6:
                continue
            off = math.atan2(math.sin(math.atan2(dy, dx) - phi), math.cos(math.atan2(dy, dx) - phi))
            if abs(off) <= th:
                req = z + d * tv
                if req > need:
                    need, worst = req, (name, round(math.degrees(phi)), (round(x, 1), round(y, 1)), round(d, 1))
    return need, worst


def pulley_pd(teeth, p):
    return teeth * p['gt2_pitch'] / math.pi


def leg_pose(f, p, a_deg):
    """(knee, axle) in the XZ plane for leg angle a (deg below horizontal)."""
    a = math.radians(a_deg); L = f['leg_len']; hx, hz = f['hip']
    return (hx - L * math.cos(a), hz - L * math.sin(a)), (hx, hz - 2 * L * math.sin(a))


def ride_height(f, p, a_deg):
    """Deck underside above the ground."""
    return p['wheel_d'] / 2 - leg_pose(f, p, a_deg)[1][1]


def foot_motor(f, p, a_deg):
    """Wheel-motor centre (x, z): on the shank, foot_cc from the axle, foot_motor_angle above the shank line."""
    knee, axle = leg_pose(f, p, a_deg)
    ang = math.radians(180.0 - a_deg - p['foot_motor_angle'])
    return (axle[0] + f['foot_cc'] * math.cos(ang), axle[1] + f['foot_cc'] * math.sin(ang))


def foot_bumper(f, p, a_deg):
    """Bumper post centre: beside the motor, perpendicular to the belt, on the upper side."""
    knee, axle = leg_pose(f, p, a_deg)
    mx, mz = foot_motor(f, p, a_deg)
    bx, bz = (axle[0] - mx) / f['foot_cc'], (axle[1] - mz) / f['foot_cc']
    px, pz = -bz, bx
    # pick the side that is up at the drawn stance (fixed in the shank frame)
    ka, aa = leg_pose(f, p, p['leg_stance_deg'])
    m0 = foot_motor(f, p, p['leg_stance_deg'])
    b0 = ((aa[0] - m0[0]) / f['foot_cc'], (aa[1] - m0[1]) / f['foot_cc'])
    if b0[0] < 0:     # perpendicular (-bz, bx) has z = bx: positive means up
        px, pz = -px, -pz
    d = p['motor_can_d'] / 2 + p['motor_bumper'][0] + p['motor_bumper'][1]
    return (mx + d * px, mz + d * pz)


def foot_posts(f, p, a_deg):
    """Both outrigger posts: the upper one (bumper) and a second one on the far side of the belt, turned post2_deg
    round the motor from the first, away from the knee (a straight mirror lands on the knee bolt)."""
    knee, axle = leg_pose(f, p, a_deg)
    mx, mz = foot_motor(f, p, a_deg)
    bx, bz = foot_bumper(f, p, a_deg)
    vx, vz = bx - mx, bz - mz
    sgn = 1.0 if (axle[0] - mx) * vz - (axle[1] - mz) * vx > 0 else -1.0   # which way the first post sits off the belt
    t = math.radians(-p['post2_deg'] * sgn)
    return [(bx, bz), (mx + vx * math.cos(t) - vz * math.sin(t), mz + vx * math.sin(t) + vz * math.cos(t))]


def wheel_obstacles(f, p, a_deg):
    R = p['wheel_d'] / 2; hx = f['hip'][0]; z = leg_pose(f, p, a_deg)[1][1] + R
    y0, y1 = f['wheel_y']
    hp = f['hip_pulley_y']; rf = f['hip_pulley_flange_r']
    mx, mz = foot_motor(f, p, a_deg); mr = p['motor_can_d'] / 2
    bpx, bpz = foot_bumper(f, p, a_deg)
    mz = max(mz, bpz + p['motor_bumper'][0] - mr)
    m0, m1 = f['motor_y'][0], max(f['hub_y'][1], f['p20_y'][1])
    return [('left wheel', (hx - R, y0, hx + R, y1), z), ('right wheel', (hx - R, -y1, hx + R, -y0), z),
            ('left wheel motor', (mx - mr, m0, mx + mr, m1), mz + mr), ('right wheel motor', (mx - mr, -m1, mx + mr, -m0), mz + mr),
            ('hip pulleys', (hx - rf, -hp[1], hx + rf, hp[1]), f['hip'][1] + rf)] + _outrigger_obstacles(f, p, a_deg)


def _outrigger_obstacles(f, p, a_deg):
    knee, axle = leg_pose(f, p, a_deg)
    mot = foot_motor(f, p, a_deg)
    br = p['motor_bumper'][0]; rb = p['shaft_bearing'][1] / 2 + 3.0
    lobes = [(q, br) for q in foot_posts(f, p, a_deg)] + [(mot, rb), (axle, 10.0)]
    x0 = min(q[0] - r for q, r in lobes); x1 = max(q[0] + r for q, r in lobes)
    zt = max(q[1] + r for q, r in lobes)
    y0, y1 = f['bumper_y'][0], f['outrigger_y'][1]
    return [('left posts + outrigger', (x0, y0, x1, y1), zt), ('right posts + outrigger', (x0, -y1, x1, -y0), zt)]


def caster_layout(p, f):
    """Caster drivetrain geometry (mounted position, deck coordinates)."""
    c = {}
    t = p['adapter_t']
    c['plate'] = p['cb_plate']
    c['plate_z'] = (-t, 0.0)
    gz = -t - p['cb_ground_clear']
    c['ground_z'] = gz
    ax_, R_ = p['cb_axle_x'], p['cb_wheel'][0] / 2
    az = gz + p['cb_wheel_axle_h']
    c['axle'] = (ax_, az)
    wy0 = p['cb_wheel_inner_y']
    c['wheel_y'] = (wy0, wy0 + p['cb_wheel'][1])
    nt, pw, dt_ = p['cb_pulley']
    c['pulley_y'] = (wy0 - dt_ - pw, wy0 - dt_)              # 48T teeth + flanges
    c['pulley_disc_y'] = (wy0 - dt_, wy0)
    c['belt_y'] = ((c['pulley_y'][0] + c['pulley_y'][1]) / 2 - p['cb_belt'][1] / 2,
                   (c['pulley_y'][0] + c['pulley_y'][1]) / 2 + p['cb_belt'][1] / 2)
    D, d = pulley_pd(nt, p), pulley_pd(p['cb_motor_teeth'], p)
    c['pd'] = (D, d)
    target = p['cb_belt'][0] * p['gt2_pitch']
    lo_, hi_ = 30.0, 300.0
    for _ in range(60):
        m_ = (lo_ + hi_) / 2
        L_ = 2 * m_ + math.pi * (D + d) / 2 + (D - d) ** 2 / (4 * m_)
        lo_, hi_ = (m_, hi_) if L_ < target else (lo_, m_)
    cc = (lo_ + hi_) / 2
    c['belt_cc'] = cc
    u = (math.cos(math.radians(p['cb_belt_angle_deg'])), math.sin(math.radians(p['cb_belt_angle_deg'])))
    c['u'] = u
    mx, mz = ax_ + cc * u[0], az + cc * u[1]
    c['motor'] = (mx, mz)
    cd, cl = p['cb_motor']
    c['motor_y'] = (p['cb_motor_base_y'], p['cb_motor_base_y'] + cl)
    pl20, ph20 = p['pulley20']
    bc_ = (c['belt_y'][0] + c['belt_y'][1]) / 2
    c['p20_y'] = (bc_ - (pl20 - ph20) / 2, bc_ + (pl20 - ph20) / 2 + ph20)
    c['shaft_need'] = c['p20_y'][1] - 1.0 - c['motor_y'][1]     # shaft through all but the last mm of the hub
    c['web_y'] = (p['cb_motor_base_y'] - p['cb_web_t'], p['cb_motor_base_y'])
    c['web_xz'] = (mx - 26, mz - 22, mx + 26, mz + 16)
    pr = p['cb_motor_holes'][0] / 2
    c['motor_holes'] = [(mx + pr * math.cos(math.radians(45 + 90 * i)), mz + pr * math.sin(math.radians(45 + 90 * i))) for i in range(4)]
    bl, by_, (b0, b1) = p['cb_axle_block'][0], p['cb_axle_block'], (p['cb_axle_block'][1], p['cb_axle_block'][2])
    c['axle_block_xz'] = (ax_ - bl / 2, az - 12, ax_ + bl / 2, -t + 0.5)
    c['axle_block_y'] = (b0, b1)
    # Battery cradle under the plate, as far back as the axle blocks allow (centre of mass forward of the axle
    # by ~1/3 of the wheelbase is the target, robot_design.md)
    bxl, byl, bzl = p['cb_battery']; w = p['cb_cradle_wall']
    lo_x = ax_ + bl / 2 + 4 + bxl / 2 + 1 + w
    hi_x = p['cb_caster_x'] - p['cb_caster_swing'] - 4 - bxl / 2 - 1 - w
    cbm = p['cb_mass']; ms = p['mass']
    wb = p['cb_caster_x'] - ax_
    target_x = ax_ + wb / 3
    deck_m = sum(m_ for _, m_, _ in f['deck_masses_x']); deck_mx = sum(m_ * x for _, m_, x in f['deck_masses_x'])
    rest = [(cbm['plate'], (p['cb_plate'][0] + p['cb_plate'][2]) / 2), (2 * cbm['motor'], mx), (2 * cbm['esc'], -126.0),
            (2 * cbm['wheel'] + cbm['axle'], ax_), (cbm['caster'], p['cb_caster_x'])]
    m_tot = deck_m + sum(m_ for m_, _ in rest) + cbm['battery']
    bx = (target_x * m_tot - deck_mx - sum(m_ * x for m_, x in rest)) / cbm['battery']
    bx = max(lo_x, min(hi_x, bx))
    c['battery_x'] = bx
    c['cg_x'] = (deck_mx + sum(m_ * x for m_, x in rest) + cbm['battery'] * bx) / m_tot
    c['cg_frac'] = (c['cg_x'] - ax_) / wb
    c['mass'] = m_tot
    c['wheelbase'] = wb
    sg = p['cb_strap_gap']
    c['cradle_outer'] = (bx - bxl / 2 - 1 - w, -(byl / 2 + sg + w), bx + bxl / 2 + 1 + w, byl / 2 + sg + w)
    c['cradle_inner'] = (bx - bxl / 2 - 1, -(byl / 2 + sg), bx + bxl / 2 + 1, byl / 2 + sg)
    c['strap_slots'] = [(p['cb_strap_x'] - 11, s_ * (byl / 2 + 0.3) if s_ > 0 else -(byl / 2 + sg - 0.3), p['cb_strap_x'] + 11, byl / 2 + sg - 0.3 if s_ > 0 else -(byl / 2 + 0.3)) for s_ in (1, -1)]
    c['cradle_z'] = (-t - p['cb_cradle_depth'], -t + 0.5)
    co_ = c['cradle_outer']
    c['cradle_ears'] = [(co_[0] - 9, -10, co_[0] + 0.5, 10), (co_[2] - 0.5, -10, co_[2] + 9, 10)]
    c['skid_plate'] = (co_[0] - 9, co_[1], co_[2] + 9, co_[3])
    c['skid_plate_z'] = (c['cradle_z'][0] - p['cb_skid_plate_t'], c['cradle_z'][0])
    c['skid_holes'] = [(co_[0] - 4.5, 0.0), (co_[2] + 4.5, 0.0)]
    c['battery_box'] = (bx - bxl / 2, -byl / 2, bx + bxl / 2, byl / 2)
    c['battery_z'] = (-t - bzl, -t)
    # Caster
    cx = p['cb_caster_x']; cpx, cpy, ch = p['cb_caster']
    c['caster_spacer'] = -t - (gz + ch)          # pad between plate underside and the caster top plate
    c['caster_plate'] = (cx - cpx / 2, -cpy / 2, cx + cpx / 2, cpy / 2)
    hx_, hy_, hd_ = p['cb_caster_holes']
    c['caster_holes'] = [(cx + i * hx_ / 2, j * hy_ / 2, hd_ / 2) for i in (-1, 1) for j in (-1, 1)]
    c['caster_top_z'] = gz + ch
    # ESC zip-tie slots
    # wiring pass-throughs behind the deck (battery under the plate -> ESCs on top; ESC signals -> HMI port)
    c['pass_throughs'] = [(-107.0, 38.0, -91.0, 56.0), (-107.0, -56.0, -91.0, -38.0)]
    c['web_gussets_x'] = [mx - 22.0, mx + 22.0]
    c['esc_slots'] = [(x - 1.5, yc - 4, x + 1.5, yc + 4) for x in p['cb_esc_slots_x'] for yc in (-26, -8, 8, 26)]
    # Camera obstacles
    c['obstacles'] = [
        ('caster base plate', p['cb_plate'], 0.0),
        ('caster base wheels', (ax_ - R_, -c['wheel_y'][1], ax_ + R_, c['wheel_y'][1]), az + R_),
        ('caster base motors', (mx - cd / 2, -c['motor_y'][1], mx + cd / 2, c['motor_y'][1]), mz + cd / 2),
        ('caster base ESCs', (p['cb_esc_slots_x'][0] - 10, -35, p['cb_esc_slots_x'][1] + 10, 35), 12.0),
    ]
    return c


def layout(p=P):
    L, W, ft, wt = p['deck_length'], p['deck_width'], p['floor_t'], p['wall_t']
    cr = p['corner_r']
    f = {}
    f['deck_outer'] = (-L / 2, -W / 2, L / 2, W / 2)
    f['deck_inner'] = (-L / 2 + wt, -W / 2 + wt, L / 2 - wt, W / 2 - wt)
    f['deck_r'], f['deck_inner_r'] = cr, max(0.5, cr - wt)
    # Module interface (HMI v1)
    f['hmi_bosses'] = [(x, y, p['hmi_boss_od'] / 2) for x, y in p['hmi_bolts']]
    f['hmi_boss_z'] = (ft, p['hmi_boss_top'])
    f['hmi_inserts'] = [(x, y, p['hmi_insert_d'] / 2) for x, y in p['hmi_bolts']]
    f['hmi_insert_z'] = (0.0, p['hmi_insert_depth'])
    rx, ry = p['hmi_peg_round']; sx_, sy_ = p['hmi_peg_slot']; hr = p['hmi_peg_hole_d'] / 2
    f['hmi_round_hole'] = (rx, ry, hr)
    f['hmi_slot'] = dict(rect=(sx_ - p['hmi_slot_len'] / 2 + hr, sy_ - hr, sx_ + p['hmi_slot_len'] / 2 - hr, sy_ + hr),
                         ends=[(sx_ - p['hmi_slot_len'] / 2 + hr, sy_, hr), (sx_ + p['hmi_slot_len'] / 2 - hr, sy_, hr)])
    f['hmi_pegs'] = [(rx, ry, p['hmi_peg_d'] / 2), (sx_, sy_, p['hmi_peg_d'] / 2)]
    cx, cy, cl, cw = p['hmi_port']
    f['cable_slot'] = (cx - cl / 2, cy - cw / 2, cx + cl / 2, cy + cw / 2)
    f['hmi_screw_len'] = 8.0
    f['hmi_under_head'] = p['adapter_t'] - p['adapter_cbore_depth']

    # Template adapter
    ax_, ay_ = p['adapter_size']; at = p['adapter_t']; ze = -p['adapter_explode']
    f['adapter_outer'] = (-ax_ / 2, -ay_ / 2, ax_ / 2, ay_ / 2)
    f['adapter_z'] = (ze - at, ze)
    keep = [(x, y, p['adapter_cbore_d'] / 2) for x, y in p['hmi_bolts']] + f['hmi_pegs']
    port = f['cable_slot']; k = p['adapter_grid_keepout']; m = p['adapter_edge_margin']
    pitch = p['adapter_grid_pitch']; gr = p['adapter_grid_hole_d'] / 2
    nut_r = p['adapter_nut_af'] / math.sqrt(3)
    grid = []
    nx, ny = int(ax_ / 2 // pitch), int(ay_ / 2 // pitch)
    for i in range(-nx, nx + 1):
        for j in range(-ny, ny + 1):
            x, y = i * pitch, j * pitch
            if abs(x) + nut_r > ax_ / 2 - m or abs(y) + nut_r > ay_ / 2 - m:
                continue
            if any(math.hypot(x - kx, y - ky) < kr + nut_r + k for kx, ky, kr in keep):
                continue
            if port[0] - k - nut_r < x < port[2] + k + nut_r and port[1] - k - nut_r < y < port[3] + k + nut_r:
                continue
            grid.append((x, y, gr))
    f['adapter_grid'] = grid
    f['adapter_nut_r'] = nut_r

    # Pi 5
    bx, by = p['pi_board_origin']; bl, bw = p['pi_board']; ins = p['pi_hole_inset']
    f['pi_board'] = (bx, by, bx + bl, by + bw)
    hx0 = bx + bl - ins - p['pi_hole_dx'] if p['pi_hole_from_rear'] else bx + ins
    f['pi_holes'] = [(hx0 + i * p['pi_hole_dx'], by + ins + j * p['pi_hole_dy']) for i in (0, 1) for j in (0, 1)]
    f['pi_standoff_z'] = (ft, ft + p['pi_standoff_h'])
    pi_top = ft + p['pi_standoff_h'] + 1.6 + p['pi_tallest']

    # ESP32 cradle
    ex, ey = p['esp_center']; el, ew = p['esp_board_l'], p['esp_board_w']
    pcb_bottom = ft + p['esp_pcb_z']
    lip_bottom = pcb_bottom + p['esp_pcb_t'] + p['esp_fit']
    rail_top = lip_bottom + p['esp_lip_t']
    lw, lh = p['esp_ledge_w'], p['esp_ledge_half_y']
    lin, rel, rt = p['esp_lip_in'], p['esp_relief'], p['esp_rail_t']
    f['esp_board'] = (ex - el / 2, ey - ew / 2, ex + el / 2, ey + ew / 2)
    ly0, ly1 = p['esp_ledge_y']
    f['esp_ledges'] = [(x0_, ey + j * ly0 if j > 0 else ey - ly1, x0_ + lw, ey + ly1 if j > 0 else ey - ly0)
                       for x0_ in (ex - el / 2, ex + el / 2 - lw) for j in (1, -1)]
    f['esp_ledge_z'] = (ft, pcb_bottom)
    f['esp_rails'] = [(ex - el / 2, ey + lin, ex + el / 2, ey + rel + rt), (ex - el / 2, ey - rel - rt, ex + el / 2, ey - lin)]
    f['esp_rail_z'] = (ft, rail_top)
    f['esp_reliefs'] = [(ex - el / 2 - 1, ey + lin, ex + el / 2 + 1, ey + rel), (ex - el / 2 - 1, ey - rel, ex + el / 2 + 1, ey - lin)]
    f['esp_relief_z'] = (ft, lip_bottom)
    f['esp_footprint'] = (ex - el / 2, ey - rel - rt, ex + el / 2, ey + rel + rt)
    esp_top = max(rail_top, pcb_bottom + p['esp_pcb_t'] + 3.0)

    # Bulkhead (tower) and servo
    tw, td = p['tower_width'], p['tower_depth']
    f['tower'] = (L / 2 - td, -tw / 2, L / 2, tw / 2)
    f['tower_cavity'] = (L / 2 - td - 1, -tw / 2 + wt, L / 2 - wt, tw / 2 - wt)
    axp = L / 2 - p['servo_axis_x_from_front']
    axis = (axp, 0.0)
    f['pan_axis'] = axis
    sbx, sby = p['servo_body']
    body_cy = -p['servo_shaft_offset']
    f['servo_pocket'] = (axp - sbx / 2, body_cy - sby / 2, axp + sbx / 2, body_cy + sby / 2)
    sp, sa = p['servo_screw_spacing'] / 2, p['servo_screw_across'] / 2
    f['servo_screws'] = [(axp + i * sa, body_cy + j * sp, p['servo_screw_d'] / 2) for i in (-1, 1) for j in (-1, 1)]

    # HC-SR04: board bottom edge sits sr04_header_below above the floor
    s = p['sr04_spacing'] / 2; sr = p['sr04_hole_d'] / 2
    sz = ft + p['sr04_header_below'] + p['sr04_board_h'] / 2
    f['sr04_center_z'] = sz
    f['sr04_holes'] = [(-s, sz, sr), (s, sz, sr)]   # (y, z, r) on the front face
    sr04_top = sz + p['sr04_board_h'] / 2
    f['sr04_board_inside'] = (L / 2 - wt - 15, -22.5, L / 2 - wt, 22.5)
    f['sr04_protrusion'] = (L / 2, -s - sr, L / 2 + p['sr04_protrusion'], s + sr)

    min_for_servo = sr04_top + p['tower_min_margin'] + p['servo_tab_drop']
    lens_rel = p['horn_top_above_roof'] + p['mount_disc_t'] + p['lens_above_disc_top']

    # ---- Lid ---------------------------------------------------------------
    pcb_top = ft + p['pi_standoff_h'] + 1.6
    needs = {
        'Pi USB/Ethernet stack': pi_top + p['usb_air'],
        'Pi active cooler + fan air gap': pcb_top + p['pi_cooler_h'] + p['fan_air_gap'],
        'Pi GPIO jumper wires': pcb_top + p['gpio_cable_h'],
        'ESP32 + wiring': esp_top + p['esp_wiring_h'],
    }
    ceiling = max(needs.values())
    lid_top = ceiling + p['lid_top_t']
    f['lid_needs'] = needs
    f['lid_ceiling'] = ceiling
    f['lid_top'] = lid_top
    f['lid_bottom'] = p['wall_height']
    lw_ = p['lid_wall_t']; fit = p['lid_fit']
    nx0 = L / 2 - td - fit
    f['lid_front_x'] = nx0
    f['lid_outer'] = (-L / 2, -W / 2, nx0, W / 2)
    f['lid_outer_r'] = (cr, 0.0, 0.0, cr)           # rear corners rounded like the deck, front square
    f['lid_inner'] = (-L / 2 + lw_, -W / 2 + lw_, nx0 + 1, W / 2 - lw_)
    f['lid_inner_r'] = (max(0.5, cr - lw_), 0.0, 0.0, max(0.5, cr - lw_))
    so = wt + p['lid_skirt_fit']; st = p['lid_skirt_t']
    yo = W / 2 - so
    xs0 = -L / 2 + wt + p['lid_slide'] + 0.5
    xs1 = nx0 - 0.5
    f['lid_skirt'] = [(xs0, yo - st, xs1, yo), (xs0, -yo, xs1, -yo + st)]
    # webs tie the skirts (inside the deck walls) to the lid walls, just above the deck wall top;
    # they stay 0.5 mm off the Pi board edge and below its USB-C/HDMI jacks
    yw0 = max(yo - st, f['pi_board'][3] + 0.5)
    f['lid_skirt_webs'] = [(xs0, yw0, xs1, W / 2 - lw_ + 0.5), (xs0, -(W / 2 - lw_ + 0.5), xs1, -yw0)]
    f['lid_skirt_web_z'] = (p['wall_height'], p['wall_height'] + 1.8)
    f['lid_tongues'] = [(nx0, y0, nx0 + p['lid_tongue_len'], y1) for y0, y1 in p['lid_tongues']]
    f['lid_tongue_z'] = (lid_top - p['lid_top_t'], lid_top)
    kz0 = lid_top + p['lid_tongue_gap']
    f['lid_keeper_z'] = (kz0, kz0 + p['lid_keeper_t'])
    tx0 = L / 2 - td
    f['lid_keepers'] = [(tx0, y0 - 1.0 if y0 > 0 else -W / 2 + wt - 1.0,
                         tx0 + p['lid_tongue_len'] + 1.0, W / 2 - wt + 1.0 if y0 > 0 else y1 + 1.0)
                        for y0, y1 in p['lid_tongues']]
    f['lid_skirt_z'] = (p['wall_height'] - p['lid_skirt_depth'], p['wall_height'] + 0.5)
    f['lid_posts'] = [(x, y, p['lid_post_od'] / 2) for x, y in p['lid_posts']]
    f['lid_post_z'] = (ft, ceiling - p['lid_post_gap'])
    f['lid_post_inserts'] = [(x, y, 2.0) for x, y in p['lid_posts']]
    f['lid_post_insert_z'] = (f['lid_post_z'][1] - 5.7, f['lid_post_z'][1])
    f['lid_screws'] = [(x, y, p['lid_screw_d'] / 2) for x, y in p['lid_posts']]
    y0, y1, zt_ = p['lid_rear_port']
    f['lid_rear_port'] = (-L / 2 - 1, y0, -L / 2 + lw_ + 1, y1)
    f['lid_rear_port_cut'] = (-L / 2 - 1, y0, -L / 2 + lw_ + 6, y1)   # deeper so it cuts cleanly through the rounded corner
    f['lid_rear_port_z'] = (p['wall_height'] - 1, zt_)
    x0, x1, zt2 = p['lid_side_port']
    f['lid_side_port'] = (x0, W / 2 - lw_ - 0.5, x1, W / 2 + 1)
    f['lid_side_port_z'] = (p['wall_height'] - 1, zt2)
    vz = p['lid_top_vent_zone']; sl, sw = p['lid_top_vent_slot']; pitch = p['lid_top_vent_pitch']
    ncols = max(1, int((vz[2] - vz[0] + 3) // (sl + 3)))
    gapx = ((vz[2] - vz[0]) - ncols * sl) / max(1, ncols - 1) if ncols > 1 else 0
    nrows = int((vz[3] - vz[1] - sw) // pitch) + 1
    top_vents = []
    for i in range(ncols):
        xa = vz[0] + i * (sl + gapx)
        for j in range(nrows):
            yc = vz[1] + sw / 2 + j * pitch
            top_vents.append((xa, yc - sw / 2, xa + sl, yc + sw / 2))
    f['lid_top_vents'] = top_vents                  # bounding boxes; cut as round-ended slots
    vw, vp = p['lid_vent_w'], p['lid_vent_pitch']
    sx0, sx1 = p['lid_side_vent_x']
    f['lid_side_vents'] = [(x - vw / 2, -W / 2 - 1, x + vw / 2, -W / 2 + lw_ + 1)
                           for x in [sx0 + k_ * vp for k_ in range(int((sx1 - sx0) // vp) + 1)]]
    ry0, ry1 = p['lid_rear_vent_y']
    f['lid_rear_vents'] = [(-L / 2 - 1, y - vw / 2, -L / 2 + lw_ + 1, y + vw / 2)
                           for y in [ry0 + k_ * vp for k_ in range(int((ry1 - ry0) // vp) + 1)]]
    f['lid_vent_z'] = p['lid_vent_z']
    f['pcb_top'] = pcb_top
    if p['build_lid']:
        # the keepers live in the bulkhead, so its roof must sit above them
        min_for_servo = max(min_for_servo, f['lid_keeper_z'][1])

    # ---- Ostrich legs: positions along Y (left side; the right side is mirrored) ----
    g = p['side_gap']; fl = p['pulley_flange']; bwid = p['belt_w']
    yf = p['hip_face_y']
    nb = p['hip_bearing_n']; bw_ = p['hip_bearing'][2]
    f['hip_bearing_y'] = (yf - nb * bw_, yf)
    f['hip_lip_y'] = (yf - nb * bw_ - p['hip_lip'][1], yf - nb * bw_)
    f['pulley_sleeve_r'] = p['hip_bearing'][1] / 2 + p['fit_press'] / 2 + p['pulley_sleeve_t']
    yh = yf - nb * bw_                                    # horn outer face = hub inner end (clamps the inner races)
    f['hub_end_y'] = yh
    ytop = yh - p['hip_servo_horn_stack']
    f['hip_servo_top_y'] = ytop
    f['hip_servo_tab_y'] = ytop - p['hip_servo_tab_from_top']
    f['hip_servo_bottom_y'] = ytop - p['hip_servo_body'][2]
    # fitting the 2nd servo: it must pass the 1st servo's bottom before its top enters its pocket
    f['hip_servo_insert_room'] = (ytop - p['hip_servo_tab_from_top']) + f['hip_servo_bottom_y']
    f['hip_servo_insert_need'] = p['hip_servo_body'][2] + p['hip_servo_spline_h'] + 1.0
    f['hip_block_y'] = (f['hip_servo_tab_y'], yf)
    pw = fl + bwid + p['pulley_outer_flange']
    f['hip_pulley_y'] = (yf, yf + pw)
    f['pulley_teeth_y'] = (yf + fl, yf + fl + bwid)
    f['thigh_y'] = (yf + pw + g, yf + pw + g + p['thigh_t'])
    f['shank_y'] = (f['thigh_y'][1] + g, f['thigh_y'][1] + g + p['shank_t'])
    s1_ = f['shank_y'][1]
    fbr, fbh = p['foot_boss']
    f['foot_boss_y'] = (s1_ - 0.5, s1_ + fbh)
    hub0 = s1_ + fbh + 1.0                                 # washer
    f['tire_y'] = (hub0, hub0 + p['tire_w'])
    f['wheel_y'] = f['tire_y']
    fl_ = p['pulley_flange']; bw9 = p['foot_belt'][1]
    f['foot_teeth_y'] = (f['tire_y'][1] + 2 * fl_, f['tire_y'][1] + 2 * fl_ + bw9 + 1.0)
    f['hub_y'] = (hub0, f['foot_teeth_y'][1] + fl_)
    f['motor_y'] = (s1_, s1_ + p['motor_h'])
    f['track'] = f['tire_y'][0] + f['tire_y'][1]          # wheel centre to wheel centre
    pl20, ph20 = p['pulley20']
    tc_ = (f['foot_teeth_y'][0] + f['foot_teeth_y'][1]) / 2
    f['p20_y'] = (tc_ - (pl20 - ph20) / 2, tc_ + (pl20 - ph20) / 2 + ph20)     # bought 20T: teeth + flanges, then the hub outboard
    ot, og = p['outrigger']
    o0_ = max(f['hub_y'][1], f['p20_y'][1]) + og
    f['outrigger_y'] = (o0_, o0_ + ot)
    f['axle_nyloc_y'] = (f['outrigger_y'][1] + 1.6, f['outrigger_y'][1] + 1.6 + 8.0)
    f['bumper_y'] = (s1_ - 0.5, f['outrigger_y'][0])
    bw_ = p['shaft_bearing'][2]
    f['shaft_need'] = f['outrigger_y'][0] + 0.75 * bw_ - f['motor_y'][1]     # shaft past the bell to seat 3/4 of the bearing
    f['shaft_supported'] = (p['shaft_support'] and p['motor_shaft_past_bell'] >= f['shaft_need']
                            and abs(p['shaft_bearing'][0] - p['motor_shaft_d']) < 0.05)
    f['axle_bolt_len'] = int(math.ceil((f['axle_nyloc_y'][1] + 1.5 - (f['shank_y'][0] + p['axle_head'][1])) / 5.0) * 5)
    f['axle_bolt_end'] = f['shank_y'][0] + p['axle_head'][1] + f['axle_bolt_len']
    f['overall_width'] = 2 * f['axle_bolt_end']
    t20, t48 = p['foot_teeth']
    fd, fD = pulley_pd(t20, p), pulley_pd(t48, p)
    f['foot_pd'] = (fd, fD)
    target_ = p['foot_belt'][0] * p['gt2_pitch']
    lo_, hi_ = 30.0, 400.0
    for _ in range(60):
        m_ = (lo_ + hi_) / 2
        Lb = 2 * m_ + math.pi * (fD + fd) / 2 + (fD - fd) ** 2 / (4 * m_)
        lo_, hi_ = (m_, hi_) if Lb < target_ else (lo_, m_)
    f['foot_cc'] = (lo_ + hi_) / 2
    dh, dk = pulley_pd(p['pulley_hip_teeth'], p), pulley_pd(p['pulley_knee_teeth'], p)
    f['pulley_pd'] = (dh, dk)
    f['hip_pulley_flange_r'] = dh / 2 - 0.254 + fl
    def belt_len(c):
        return 2 * c + math.pi * (dh + dk) / 2 + (dh - dk) ** 2 / (4 * c)
    if p['leg_len'] is None:
        lo_, hi_ = 40.0, 400.0
        target = p['belt_teeth'] * p['gt2_pitch']
        for _ in range(60):
            mid_ = (lo_ + hi_) / 2
            lo_, hi_ = (mid_, hi_) if belt_len(mid_) < target else (lo_, mid_)
        Ll = (lo_ + hi_) / 2 + p['belt_pretension']
    else:
        Ll = p['leg_len']
    f['leg_len'] = Ll
    f['belt_len'] = belt_len(Ll)
    f['belt_teeth'] = f['belt_len'] / p['gt2_pitch']

    # ---- Balance point: hip x so the centre of mass is over the axle at the drawn stance ----
    ms = p['mass']; a = math.radians(p['leg_stance_deg'])
    fixed = [('deck', ms['deck'], 0.0), ('lid', ms['lid'], -L / 2 + (nx0 + L / 2) / 2),
             ('pi', ms['pi'], (f['pi_board'][0] + f['pi_board'][2]) / 2), ('esp', ms['esp'], ex),
             ('sr04', ms['sr04'], L / 2 - wt - 5), ('pan', ms['pan'], axp),
             ('gopro', ms['gopro'], axp + p['lens_forward'] / 2), ('power', ms['power'], 20.0)]
    so_ = p['hip_servo_spline_offset']   # body centre = hip_x - so_
    rel_ = [('hip_module', ms['hip_module'], -5.0), ('hip_servo', 2 * ms['hip_servo'], -so_),
            ('battery', ms['battery'], 0.0), ('hip_pulley', 2 * ms['hip_pulley'], 0.0),
            ('thigh', 2 * ms['thigh'], -Ll / 2 * math.cos(a)), ('shank', 2 * ms['shank'], -Ll / 2 * math.cos(a)),
            ('knee', 2 * ms['knee'], -Ll * math.cos(a)),
            ('motor', 2 * ms['motor'], -f['foot_cc'] * math.cos(a + math.radians(p['foot_motor_angle']))), ('wheel', 2 * ms['wheel'], 0.0)]
    mf = sum(m_ for _, m_, _ in fixed)
    ebx = p['hip_ebay'][0] - 8.0                    # electronics bay boards sit just behind the web
    hip_x = (sum(m_ * x for _, m_, x in fixed) + ms['ebay'] * ebx + sum(m_ * dx for _, m_, dx in rel_)) / (mf + ms['ebay']) \
        if p['hip_x'] is None else p['hip_x']
    f['ebay_mass_x'] = (ms['ebay'], ebx)
    f['deck_masses_x'] = fixed
    f['hip'] = (hip_x, p['hip_z'])
    f['mass_total'] = sum(m_ for _, m_, _ in fixed) + sum(m_ for _, m_, _ in rel_) + ms['ebay']
    f['mass_sprung'] = sum(m_ for _, m_, _ in fixed) + ms['hip_module'] + 2 * ms['hip_servo'] + ms['battery'] + 2 * ms['hip_pulley']
    hz = p['hip_z']
    sc = hip_x - so_
    f['hip_servo_cx'] = sc
    hbx, hbz = p['hip_block_half_x'], p['hip_block_half_z']
    f['hip_block_xz'] = (sc - hbx, hz - hbz, sc + hbx, hz + hbz)
    sl_, sw_, _ = p['hip_servo_body']; c_ = p['hip_servo_clear']
    f['hip_servo_pocket_xz'] = (sc - sl_ / 2 - c_ / 2, hz - sw_ / 2 - c_ / 2, sc + sl_ / 2 + c_ / 2, hz + sw_ / 2 + c_ / 2)
    thx, thz = p['hip_servo_tab_holes']
    f['hip_servo_tab_holes_xz'] = [(sc + i * thx / 2, hz + j * thz / 2) for i in (-1, 1) for j in (-1, 1)]
    pr = p['pulley_screw_pcd'] / 2
    f['pulley_screws_xz'] = [(hip_x + pr * math.cos(math.radians(45 + 90 * i)), hz + pr * math.sin(math.radians(45 + 90 * i))) for i in range(4)]
    hrr = p['hip_servo_horn_pcd'] / 2
    f['horn_screws_xz'] = [(hip_x + hrr * math.cos(math.radians(45 + 90 * i)), hz + hrr * math.sin(math.radians(45 + 90 * i))) for i in range(4)]
    # Battery tray (under the hip block, centred on the axle)
    bxl, byl, bzl = p['battery']; twl, tfl = p['tray_wall'], p['tray_floor']
    zb = hz - hbz
    f['tray_outer'] = (hip_x - bxl / 2 - 1 - twl, -(byl / 2 + 1.5 + twl), hip_x + bxl / 2 + 1, byl / 2 + 1.5 + twl)
    f['tray_z'] = (zb - bzl - 1 - tfl, zb + 3.0)
    f['tray_inner'] = (hip_x - bxl / 2 - 1, -(byl / 2 + 1.5), hip_x + bxl / 2 + 2, byl / 2 + 1.5)
    f['tray_inner_z'] = (zb - bzl - 1, zb)
    f['strap_slot_x'] = (f['tray_outer'][2] - 8, f['tray_outer'][2] - 5)
    f['strap_slot_z'] = (f['tray_z'][0] + tfl + 3, zb - 3)
    f['hip_plate'] = (-L / 2, -yf, L / 2, yf)
    ex0, ex1, ehw, ezb = p['hip_ebay']
    f['hip_ebay'] = (ex0, -ehw, ex1, ehw)
    f['hip_ebay_z'] = (ezb, -p['adapter_t'] + 0.5)
    f['hip_ebay_slots'] = [(ex0 - 1, y - 1.5, ex1 + 1, y + 1.5, z - 3, z + 3)
                           for y in (-34.0, -4.0, 4.0, 34.0) for z in (-12.0, -40.0)]
    f['imu_bosses'] = [(hip_x + i * p['imu_holes'] / 2, 0.0, 3.0) for i in (-1, 1)]
    f['imu_boss_z'] = (-p['adapter_t'] - 4.0, -p['adapter_t'] + 0.5)
    wp = p['hip_wire_pass']
    f['hip_wire_pass'] = [(wp[0], wp[2], wp[1], wp[3]), (wp[0], -wp[3], wp[1], -wp[2])]
    f['hip_tie_anchors'] = [(x - 4, s_ * 58.0 if s_ > 0 else -67.0, x + 4, 67.0 if s_ > 0 else -58.0) for x in p['hip_tie_anchors_x'] for s_ in (1, -1)]
    f['hip_tie_anchor_z'] = (-0.5, 5.0)
    hw_, reach, fz0, fz1 = p['hip_fairlead']
    f['hip_fairleads'] = [(hip_x - hw_, 58.0, hip_x + hw_, reach), (hip_x - hw_, -reach, hip_x + hw_, -58.0)]
    f['hip_fairlead_z'] = (fz0, fz1)
    f['hip_fairlead_root'] = [(hip_x - hw_, 58.0, hip_x + hw_, p['hip_face_y']), (hip_x - hw_, -p['hip_face_y'], hip_x + hw_, -58.0)]
    f['hip_fairlead_tunnels'] = [(reach - 8, fz0 + 1.5, reach - 3, fz1 - 1.5), (-reach + 3, fz0 + 1.5, -reach + 8, fz1 - 1.5)]
    f['hip_tie_tunnels'] = [(r_[1] + 2.5, 1.2, r_[3] - 2.5, 3.4) for r_ in f['hip_tie_anchors']]
    # Parking skids
    tilt = math.tan(math.radians(p['park_tilt_deg']))
    f['ground_park_z'] = -ride_height(f, p, p['leg_park_deg'])
    f['skids'] = [(x, f['ground_park_z'] + abs(x - hip_x) * tilt) for x in p['skid_x']]
    f['skid_body_r'] = p['skid_r']

    # ---- Camera: tower tall enough that nothing is in shot ----------------
    de = p['drive_envelope']
    obstacles = [
        ('deck walls', f['deck_outer'], p['wall_height']),
        ('Pi 5 + USB stack', f['pi_board'], pi_top),
        ('ESP32 cradle', f['esp_footprint'], esp_top),
        ('HC-SR04 transducers', f['sr04_protrusion'], sr04_top),
    ]
    # One deck serves both drivetrains, so the camera must clear both of them
    obstacles += wheel_obstacles(f, p, p['leg_film_min_deg'])
    f['cb'] = caster_layout(p, f)
    obstacles += f['cb']['obstacles']
    if p['build_adapter']:
        obstacles.append(('drivetrain envelope', de[:4], de[4]))
    if p['build_lid']:
        obstacles.append(('lid', f['deck_outer'], lid_top))
    lens_min, worst = camera_clearance(axis, obstacles, p)
    tower_top = max(min_for_servo, lens_min + p['view_margin'] - lens_rel)
    obstacles.append(('tower roof', f['tower'], tower_top))
    lens_min2, worst2 = camera_clearance(axis, obstacles, p)
    if lens_min2 + p['view_margin'] - lens_rel > tower_top:
        tower_top = lens_min2 + p['view_margin'] - lens_rel
        lens_min, worst = lens_min2, worst2
    f['tower_top'] = tower_top
    # lintel across the open back of the bulkhead, above the lid (closes the gap and holds the tongues)
    f['lintel'] = (tx0, -W / 2 + wt - 1, tx0 + 3.0, W / 2 - wt + 1)
    f['lintel_z'] = (f['lid_keeper_z'][0], tower_top - p['tower_roof_t'] + 0.5)
    f['tower_min_for_servo'] = min_for_servo
    f['lens_z'] = tower_top + lens_rel
    f['lens_z_min'] = lens_min
    f['view_worst'] = worst
    f['obstacles'] = obstacles
    f['servo_body_bottom'] = tower_top - p['servo_tab_drop']
    f['sr04_top'] = sr04_top
    f['pi_top'], f['esp_top'] = pi_top, esp_top

    # Pan mount
    z0 = tower_top + p['horn_top_above_roof']
    f['mount_z0'] = z0
    f['adapter_disc_z'] = (z0, z0 + p['adapter_disc_t'])
    f['buckle_base_z'] = (z0 + p['adapter_disc_t'], z0 + p['adapter_disc_t'] + p['buckle_base_t'])
    f['mount_top'] = f['buckle_base_z'][1]
    hr_ = p['horn_pcd'] / 2
    f['horn_screws'] = [(axp + hr_ * math.cos(math.radians(45 + 90 * i)), hr_ * math.sin(math.radians(45 + 90 * i)), 1.7) for i in range(4)]
    br_ = p['buckle_screw_pcd'] / 2
    f['buckle_screws'] = [(axp + br_ * math.cos(math.radians(90 + 120 * i)), br_ * math.sin(math.radians(90 + 120 * i))) for i in range(3)]
    # SG90 bench-test plate on the roof: its horn top lands where the standard servo's does
    f['sg90_plate_t'] = p['horn_top_above_roof'] - p['sg90_horn_top_above_tab']
    f['sg90_plate_z'] = (tower_top, tower_top + f['sg90_plate_t'])
    hx_ = max(abs(x - axp) for x, y, r in f['servo_screws']) + 5.0
    ys_ = [y for x, y, r in f['servo_screws']]
    f['sg90_plate'] = (axp - hx_, min(ys_) - 5.0, axp + hx_, max(ys_) + 5.0)
    sp_ = f['servo_pocket']
    f['sg90_lip'] = (sp_[0] + 0.2, sp_[1] + 0.2, sp_[2] - 0.2, sp_[3] - 0.2)
    sbx2, sby2 = p['sg90_body']; cy2 = -p['sg90_shaft_offset']
    f['sg90_pocket'] = (axp - sbx2 / 2, cy2 - sby2 / 2, axp + sbx2 / 2, cy2 + sby2 / 2)
    sp2 = p['sg90_screw_spacing'] / 2
    f['sg90_screws'] = [(axp, cy2 - sp2, p['sg90_screw_d'] / 2), (axp, cy2 + sp2, p['sg90_screw_d'] / 2)]
    f['sg90_body_bottom'] = f['sg90_plate_z'][1] - p['sg90_tab_drop']
    br, bd = p['servo_insert_boss']
    f['servo_bosses'] = [(x, y, br) for x, y, _ in f['servo_screws']]
    f['servo_boss_z'] = (tower_top - p['tower_roof_t'] - bd, tower_top - p['tower_roof_t'] + 0.5)
    f['servo_insert_z'] = (tower_top - 7.0, tower_top + 1)
    px0, px1, pzt = p['power_web']
    f['power_web'] = [(px0, -W / 2 + wt - 1, px1, -10.0), (px0, 34.0, px1, W / 2 - wt + 1)]   # y 12-34 left clear for the Pi's microSD
    f['power_web_z'] = (ft - 0.5, pzt)
    f['deck_tie_anchors'] = [(x - 4, -11.5, x + 4, -6.8) for x in p['deck_tie_anchors_x']]
    f['deck_tie_anchor_z'] = (ft - 0.5, ft + 5.0)
    f['deck_tie_tunnel'] = (-10.6, ft + 1.2, -7.7, ft + 3.4)
    f['power_web_slots'] = [(px0 - 1, y - 1.5, px1 + 1, y + 1.5, z - 3, z + 3)
                            for y in (-46.0, -14.0, 37.0, 47.0) for z in (9.0, 24.0)]
    t, g_ = p['gp_prong_t'], p['gp_gap']
    f['gp_prong_y'] = [-(t + g_), 0.0, t + g_]
    f['gp_hole_z'] = f['mount_top'] + p['gp_hole_z']
    f['gp_total_w'] = 3 * t + 2 * g_

    if True:
        # lowest stance with a clean shot (wheels rise into view as the robot crouches)
        base = [o for o in obstacles if o[0] not in ('left wheel', 'right wheel', 'left wheel motor', 'right wheel motor', 'hip pulleys')]
        clean = None
        for k_ in range(int(p['leg_park_deg']), int(p['leg_max_deg']) + 1):
            need_k, _ = camera_clearance(axis, base + wheel_obstacles(f, p, k_), p)
            if need_k <= f['lens_z']:
                clean = k_
                break
        f['clean_shot_from_deg'] = clean
        # centre of mass height at the drawn stance (for the balance controller)
        knee, axle = leg_pose(f, p, p['leg_stance_deg'])
        zs = [('deck', ms['deck'], 6.0), ('lid', ms['lid'], (p['wall_height'] + lid_top) / 2), ('pi', ms['pi'], pcb_top + 5),
              ('esp', ms['esp'], pcb_bottom + 3), ('sr04', ms['sr04'], sz), ('pan', ms['pan'], tower_top + 8),
              ('gopro', ms['gopro'], f['lens_z'] - 12), ('power', ms['power'], 8.0),
              ('hip_module', ms['hip_module'], hz - 10), ('hip_servo', 2 * ms['hip_servo'], hz),
              ('battery', ms['battery'], (f['tray_inner_z'][0] + f['tray_inner_z'][1]) / 2), ('hip_pulley', 2 * ms['hip_pulley'], hz),
              ('thigh', 2 * ms['thigh'], (hz + knee[1]) / 2), ('shank', 2 * ms['shank'], (knee[1] + axle[1]) / 2),
              ('knee', 2 * ms['knee'], knee[1]), ('motor', 2 * ms['motor'], foot_motor(f, p, p['leg_stance_deg'])[1]), ('wheel', 2 * ms['wheel'], axle[1])]
        f['com_z'] = sum(m_ * z for _, m_, z in zs) / sum(m_ for _, m_, _ in zs)
        f['com_above_axle'] = f['com_z'] - axle[1]
    return f


# ---------------------------------------------------------------------------
# 2D outlines as segment lists: ('L', p0, p1) / ('A', p0, mid, p1), points (u, v)
# ---------------------------------------------------------------------------
def rrect_segs(r4, radii):
    """Rectangle (u0, v0, u1, v1) with corner radii (u0v0, u1v0, u1v1, u0v1); 0 = sharp."""
    u0, v0, u1, v1 = r4
    if isinstance(radii, (int, float)):
        radii = (radii,) * 4
    r0, r1, r2, r3 = radii
    k = 1 - 1 / math.sqrt(2)
    raw = [('L', (u0 + r0, v0), (u1 - r1, v0)),
           ('A', (u1 - r1, v0), (u1 - r1 * k, v0 + r1 * k), (u1, v0 + r1)),
           ('L', (u1, v0 + r1), (u1, v1 - r2)),
           ('A', (u1, v1 - r2), (u1 - r2 * k, v1 - r2 * k), (u1 - r2, v1)),
           ('L', (u1 - r2, v1), (u0 + r3, v1)),
           ('A', (u0 + r3, v1), (u0 + r3 * k, v1 - r3 * k), (u0, v1 - r3)),
           ('L', (u0, v1 - r3), (u0, v0 + r0)),
           ('A', (u0, v0 + r0), (u0 + r0 * k, v0 + r0 * k), (u0 + r0, v0))]
    return [s for s in raw if math.hypot(s[-1][0] - s[1][0], s[-1][1] - s[1][1]) > 1e-4]


def capsule_segs(c1, r1, c2, r2):
    """Two circles joined by their outer tangents (a tapered link or a round-ended slot)."""
    dx, dy = c2[0] - c1[0], c2[1] - c1[1]
    d = math.hypot(dx, dy)
    ex, ey = dx / d, dy / d
    nx, ny = -ey, ex
    a = (r1 - r2) / d
    b = math.sqrt(max(0.0, 1 - a * a))
    w1 = (a * ex + b * nx, a * ey + b * ny)
    w2 = (a * ex - b * nx, a * ey - b * ny)
    t1a = (c1[0] + r1 * w1[0], c1[1] + r1 * w1[1]); t2a = (c2[0] + r2 * w1[0], c2[1] + r2 * w1[1])
    t1b = (c1[0] + r1 * w2[0], c1[1] + r1 * w2[1]); t2b = (c2[0] + r2 * w2[0], c2[1] + r2 * w2[1])
    return [('L', t1a, t2a), ('A', t2a, (c2[0] + r2 * ex, c2[1] + r2 * ey), t2b),
            ('L', t2b, t1b), ('A', t1b, (c1[0] - r1 * ex, c1[1] - r1 * ey), t1a)]


def slot_segs(rect):
    """Round-ended slot filling a bounding box, along its long side."""
    u0, v0, u1, v1 = rect
    if u1 - u0 >= v1 - v0:
        r = (v1 - v0) / 2; v = (v0 + v1) / 2
        return capsule_segs((u0 + r, v), r, (u1 - r, v), r)
    r = (u1 - u0) / 2; u = (u0 + u1) / 2
    return capsule_segs((u, v0 + r), r, (u, v1 - r), r)


def poly_segs(pts):
    return [('L', pts[i], pts[(i + 1) % len(pts)]) for i in range(len(pts))]


def hex_pts(c, af, rot=0.0):
    r = af / math.sqrt(3)
    return [(c[0] + r * math.cos(math.radians(rot + 60 * i)), c[1] + r * math.sin(math.radians(rot + 60 * i))) for i in range(6)]


def teeth_circles(c, teeth, p):
    """GT2 grooves approximated by r = 0.6 mm circles (print, test with your belt)."""
    od = pulley_pd(teeth, p) - 0.508
    rc = od / 2 - 0.15
    return [(c[0] + rc * math.cos(2 * math.pi * i / teeth), c[1] + rc * math.sin(2 * math.pi * i / teeth), 0.6)
            for i in range(teeth)]


# ---------------------------------------------------------------------------
# Fusion (API works in cm)
# ---------------------------------------------------------------------------
def cm(v):
    return v / 10.0


def _pt(sk, x, y, z=0.0):
    """Model mm -> sketch space, snapped onto the sketch plane."""
    pt = sk.modelToSketchSpace(adsk.core.Point3D.create(cm(x), cm(y), cm(z)))
    return adsk.core.Point3D.create(pt.x, pt.y, 0.0)


def _profiles(sk):
    c = adsk.core.ObjectCollection.create()
    for pr in sk.profiles:
        c.add(pr)
    return c


def _arc_points(seg, n=12):
    """Points along a 3-point arc segment (model u, v), start to end inclusive."""
    (x1, y1), (x2, y2), (x3, y3) = seg[1], seg[2], seg[3]
    d = 2 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2))
    ux = ((x1 * x1 + y1 * y1) * (y2 - y3) + (x2 * x2 + y2 * y2) * (y3 - y1) + (x3 * x3 + y3 * y3) * (y1 - y2)) / d
    uy = ((x1 * x1 + y1 * y1) * (x3 - x2) + (x2 * x2 + y2 * y2) * (x1 - x3) + (x3 * x3 + y3 * y3) * (x2 - x1)) / d
    r = math.hypot(x1 - ux, y1 - uy)
    a1, a2, a3 = (math.atan2(y - uy, x - ux) for x, y in (seg[1], seg[2], seg[3]))
    d2 = (a2 - a1) % (2 * math.pi); d3 = (a3 - a1) % (2 * math.pi)
    sweep = d3 if d2 < d3 else d3 - 2 * math.pi
    return [(ux + r * math.cos(a1 + sweep * k / n), uy + r * math.sin(a1 + sweep * k / n)) for k in range(n + 1)]


def _draw_path(sk, segs, m):
    """Draw a closed outline; consecutive curves share sketch points so it forms a profile.
    Arcs are true 3-point arcs; if Fusion refuses one, that arc becomes a fine polyline."""
    lines = sk.sketchCurves.sketchLines
    arcs = sk.sketchCurves.sketchArcs
    first = prev = None
    n = len(segs)

    def pick(ent, tgt):
        a_, b_ = ent.startSketchPoint, ent.endSketchPoint
        return (b_, a_) if b_.geometry.distanceTo(tgt) <= a_.geometry.distanceTo(tgt) else (a_, b_)

    for i, seg in enumerate(segs):
        tgt = _pt(sk, *m(*seg[-1]))
        p0 = prev if prev is not None else _pt(sk, *m(*seg[1]))
        p1 = first if (i == n - 1 and first is not None) else tgt
        if seg[0] == 'L':
            end, start = pick(lines.addByTwoPoints(p0, p1), tgt)
        else:
            try:
                end, start = pick(arcs.addByThreePoints(p0, _pt(sk, *m(*seg[2])), p1), tgt)
            except Exception:
                q = _arc_points(seg)
                ln = lines.addByTwoPoints(p0, _pt(sk, *m(*q[1])))
                start = ln.startSketchPoint
                for pt_ in q[2:-1]:
                    ln = lines.addByTwoPoints(ln.endSketchPoint, _pt(sk, *m(*pt_)))
                ln = lines.addByTwoPoints(ln.endSketchPoint, p1)
                end = ln.endSketchPoint
        if first is None:
            first = start
        prev = end


LAST_FEATURE = ['(none yet)']   # reported if the script fails, so the failing feature is named


class Builder:
    def __init__(self, root):
        self.root = root
        self.ext = root.features.extrudeFeatures
        self.planes = root.constructionPlanes
        self._plane_cache = {}
        O = adsk.fusion.FeatureOperations
        self.ops = dict(new=O.NewBodyFeatureOperation, join=O.JoinFeatureOperation, cut=O.CutFeatureOperation)

    # -- planes -------------------------------------------------------------
    def plane(self, axis, dist, name=None):
        """Plane normal to `axis` ('x', 'y' or 'z') at model coordinate `dist` (mm)."""
        base = {'x': self.root.yZConstructionPlane, 'y': self.root.xZConstructionPlane,
                'z': self.root.xYConstructionPlane}[axis]
        if abs(dist) < 1e-9:
            return base
        key = (axis, round(dist, 4))
        if key in self._plane_cache:
            return self._plane_cache[key]

        def make(d):
            pin = self.planes.createInput()
            pin.setByOffset(base, adsk.core.ValueInput.createByReal(cm(d)))
            return self.planes.add(pin)
        pl = make(dist)
        o = pl.geometry.origin
        got = {'x': o.x, 'y': o.y, 'z': o.z}[axis] * 10.0
        if abs(got - dist) > 0.01:          # Fusion's offset direction isn't guaranteed: flip
            pl.deleteMe()
            pl = make(-dist)
        pl.name = name or f'{axis.upper()} = {dist:.1f}'
        pl.isLightBulbOn = False
        self._plane_cache[key] = pl
        return pl

    # -- sketches -------------------------------------------------------------
    def sketch(self, name, plane, m, paths=(), circles=()):
        LAST_FEATURE[0] = f'{self.root.name} / {name}'
        sk = self.root.sketches.add(plane)
        sk.name = name
        sk.isComputeDeferred = True
        for segs in paths:
            _draw_path(sk, segs, m)
        for (u, v, r) in circles:
            sk.sketchCurves.sketchCircles.addByCenterRadius(_pt(sk, *m(u, v)), cm(r))
        sk.isComputeDeferred = False
        return sk

    def _finish(self, op, feat, name):
        if op == 'new':
            body = feat.bodies.item(0)
            body.name = name
            return body
        return None

    # -- extrudes on the XY plane between z0 and z1 -------------------------
    def xy(self, op, name, z0, z1, paths=(), circles=(), body=None, rects=()):
        paths = list(paths) + [rrect_segs(r, 0.0) for r in rects]
        sk = self.sketch(name, self.root.xYConstructionPlane, lambda u, v: (u, v, 0.0), paths, circles)
        inp = self.ext.createInput(_profiles(sk), self.ops[op])
        inp.setDistanceExtent(False, adsk.core.ValueInput.createByReal(cm(z1 - z0)))
        if z0:
            inp.startExtent = adsk.fusion.OffsetStartDefinition.create(adsk.core.ValueInput.createByReal(cm(z0)))
        if body is not None and op != 'new':
            inp.participantBodies = [body]
        return self._finish(op, self.ext.add(inp), name)

    # -- extrudes on a side plane (normal to X or Y) between a0 and a1 ------
    def side(self, op, name, axis, a0, a1, paths=(), circles=(), body=None, rects=()):
        """axis 'y': sketch (u, v) = (x, z), extruded from y=a0 to y=a1.
        axis 'x': sketch (u, v) = (y, z), extruded from x=a0 to x=a1."""
        mid, t = (a0 + a1) / 2, abs(a1 - a0)
        pl = self.plane(axis, mid)
        m = (lambda u, v: (u, mid, v)) if axis == 'y' else (lambda u, v: (mid, u, v))
        paths = list(paths) + [rrect_segs(r, 0.0) for r in rects]
        sk = self.sketch(name, pl, m, paths, circles)
        inp = self.ext.createInput(_profiles(sk), self.ops[op])
        inp.setSymmetricExtent(adsk.core.ValueInput.createByReal(cm(t)), True)
        if body is not None and op != 'new':
            inp.participantBodies = [body]
        return self._finish(op, self.ext.add(inp), name)


# ---------------------------------------------------------------------------
# Bodies
# ---------------------------------------------------------------------------
class Assembly:
    """Components + as-built joints, so the mechanism can be moved in Fusion (Assemble > Drive Joints)."""

    def __init__(self, root, enabled=True):
        self.root = root
        self.enabled = enabled
        self.occ = {}
        self.notes = []
        self.joints = {}

    def comp(self, name):
        """New component at the origin; returns (occurrence, Builder for it)."""
        occ = self.root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        try:
            occ.component.name = name
        except Exception:
            pass
        self.occ[name] = occ
        return occ, Builder(occ.component)

    @staticmethod
    def circle_edge(body, center, r):
        """The circular edge of `body` with this centre (mm, model) and radius (mm)."""
        c = adsk.core.Point3D.create(cm(center[0]), cm(center[1]), cm(center[2]))
        best = None
        for e in body.edges:
            g = e.geometry
            if g.objectType != adsk.core.Circle3D.classType():
                continue
            if abs(g.radius - cm(r)) < 0.002:
                d = g.center.distanceTo(c)
                if best is None or d < best[0]:
                    best = (d, e)
        if best is None:
            raise ValueError(f'no circular edge r={r} on {body.name}')
        return best[1]

    def revolute(self, name, moving, fixed, body, body_occ, center_xz_or_xyz, r, axis_point=None):
        if not self.enabled:
            return None
        try:
            e = self.circle_edge(body, center_xz_or_xyz, r).createForAssemblyContext(body_occ)
            geo = adsk.fusion.JointGeometry.createByCurve(e, adsk.fusion.JointKeyPointTypes.CenterKeyPoint)
            inp = self.root.asBuiltJoints.createInput(moving, fixed, geo)
            inp.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
            j = self.root.asBuiltJoints.add(inp)
            j.name = name
            self.joints[name] = j
            return j
        except Exception as ex:   # keep building; report at the end
            self.notes.append(f'joint "{name}" not created ({ex})')
            return None

    def rigid(self, name, a, b):
        if not self.enabled:
            return None
        try:
            inp = self.root.asBuiltJoints.createInput(a, b, None)
            inp.setAsRigidJointMotion()
            j = self.root.asBuiltJoints.add(inp)
            j.name = name
            return j
        except Exception as ex:
            self.notes.append(f'joint "{name}" not created ({ex})')
            return None

    def link(self, name, j1, j2, v1_deg, v2_deg):
        """Motion link j1 -> j2 (Fusion API since Nov 2025). Falls back to a note with manual steps."""
        if not (self.enabled and j1 and j2):
            return
        try:
            mi = self.root.motionLinks.createInput(j1, j2)
            mi.valueOne = adsk.core.ValueInput.createByString(f'{v1_deg} deg')
            mi.valueTwo = adsk.core.ValueInput.createByString(f'{v2_deg} deg')
            ml = self.root.motionLinks.add(mi)
            ml.name = name
            if not getattr(self, '_link_hint', False):
                self._link_hint = True
                self.notes.append('check the leg motion links: drag a thigh; the wheel axle must stay under the hip. '
                                  'If it swings fore/aft, edit the motion link and tick Reverse')
        except Exception as ex:
            self.notes.append(f'motion link "{name}" not created ({ex}). Add it by hand: Assemble > Motion Link, '
                              f'pick the hip joint then the knee joint, {v1_deg} deg : {v2_deg} deg, tick Reverse if the '
                              'axle swings forward/back instead of staying under the hip')


def _path_lengths(segs):
    out = []
    for sgm in segs:
        if sgm[0] == 'L':
            out.append(math.hypot(sgm[2][0] - sgm[1][0], sgm[2][1] - sgm[1][1]))
        else:
            q = _arc_points(sgm, 48)
            out.append(sum(math.hypot(q[i + 1][0] - q[i][0], q[i + 1][1] - q[i][1]) for i in range(48)))
    return out


def points_along(segs, n):
    """n points equally spaced along a closed outline (segment list)."""
    lens = _path_lengths(segs)
    total = sum(lens)
    pts = []
    for k in range(n):
        s_ = total * k / n
        for sgm, ln in zip(segs, lens):
            if s_ <= ln or sgm is segs[-1]:
                t = min(1.0, s_ / ln) if ln else 0.0
                if sgm[0] == 'L':
                    pts.append((sgm[1][0] + t * (sgm[2][0] - sgm[1][0]), sgm[1][1] + t * (sgm[2][1] - sgm[1][1])))
                else:
                    q = _arc_points(sgm, 200)
                    pts.append(q[int(round(t * 200))])
                break
            s_ -= ln
    return pts, total


def printed_belt(b, name, y0, y1, c1, pd1, c2, pd2, teeth):
    """TPU test belt: GT2 profile approximation along the real belt path (pitch circles pd1, pd2)."""
    land1, land2 = pd1 / 2 - 0.254, pd2 / 2 - 0.254
    back = 1.2   # thicker backing than a moulded belt, for printing
    belt = b.side('new', name, 'y', y0, y1, paths=[capsule_segs(c1, land1 + back, c2, land2 + back)])
    b.side('cut', 'Belt inside', 'y', y0 - 1, y1 + 1, body=belt, paths=[capsule_segs(c1, land1, c2, land2)])
    inner = capsule_segs(c1, land1 - 0.2, c2, land2 - 0.2)
    pts, _ = points_along(inner, teeth)
    b.side('join', 'Belt teeth', 'y', y0, y1, body=belt, circles=[(x, z, 0.55) for x, z in pts])
    return belt


def build(root, p=P):
    f = layout(p)
    asm = Assembly(root, p['joints'])
    out = dict(f=f, bodies=[], asm=asm)
    ost = p['drivetrain'] in ('ostrich', 'both')
    cas = p['drivetrain'] in ('caster', 'both')
    deck_occ = None
    if p['build_deck']:
        deck_occ, b = asm.comp('Deck')
        out['bodies'].append(build_deck(b, root, f, p))
        if p['pan_servo'] == 'sg90':
            out['bodies'].append(build_sg90_plate(b, f, p))
        try:
            deck_occ.isGrounded = True
        except Exception:
            pass
    if p['build_lid']:
        lid_occ, b = asm.comp('Lid')
        out['bodies'].append(build_lid(b, f, p))
    if p['build_pan_mount']:
        pan_occ, b = asm.comp('Pan (GoPro mount)')
        bodies = build_pan_mount(b, root, f, p)
        out['bodies'] += bodies
        if deck_occ:
            asm.revolute('Pan', pan_occ, deck_occ, bodies[0], pan_occ, (f['pan_axis'][0], 0.0, f['adapter_disc_z'][1]), 3.0)
    if p['build_adapter']:
        ad_occ, b = asm.comp('Template adapter')
        out['bodies'].append(build_adapter(b, f, p))
    if ost:
        out['bodies'] += build_legs(asm, f, p, deck_occ)
    if cas:
        out['bodies'] += build_caster(asm, f, p, deck_occ, p['caster_draw_offset'] if p['drivetrain'] == 'both' else 0.0)
    return out


def build_sg90_plate(b, f, p):
    """Bench-test plate: bolts over the standard servo pocket, holds an SG90 on the same pan axis."""
    z0, z1 = f['sg90_plate_z']
    pl = b.xy('new', 'SG90 test plate', z0, z1, paths=[rrect_segs(f['sg90_plate'], 3.0)])
    b.xy('join', 'Locating lip', z0 - p['sg90_plate_lip'], z0 + 0.5, rects=[f['sg90_lip']], body=pl)
    b.xy('cut', 'SG90 pocket', z0 - p['sg90_plate_lip'] - 1, z1 + 1, rects=[f['sg90_pocket']], body=pl)
    gp_ = f['sg90_pocket']; gw_, gl_ = p['sg90_cable_slit']; gx_ = (gp_[0] + gp_[2]) / 2
    b.xy('cut', 'SG90 cable slit', z0 - p['sg90_plate_lip'] - 1, z1 + 1, rects=[(gx_ - gw_ / 2, gp_[1] - gl_, gx_ + gw_ / 2, gp_[1] + 0.5)], body=pl)
    b.xy('cut', 'SG90 screws (M2)', z0, z1 + 1, circles=f['sg90_screws'], body=pl)
    b.xy('cut', 'Plate screws (M3, into the roof inserts)', z0 - 3, z1 + 1, circles=[(x, y, 1.7) for x, y, _ in f['servo_screws']], body=pl)
    return pl


def build_pan_mount(b, root, f, p):
    """Two parts: horn adapter (screws to the servo horn) + GoPro buckle (bolts to the adapter)."""
    ax, ay = f['pan_axis']
    a0, a1 = f['adapter_disc_z']
    R_ = p['mount_disc_d'] / 2
    if p['pan_servo'] == 'sg90':
        ad = b.xy('new', 'Pan horn adapter (SG90 bench test)', a0, a1, circles=[(ax, ay, R_)])
        al, aw, adp = p['sg90_horn_arm']
        b.xy('cut', '2-arm horn pocket', a0 - 1, a0 + adp, rects=[(ax - al / 2, -aw / 2, ax + al / 2, aw / 2)],
             circles=[(ax, ay, p['sg90_horn_hub_d'] / 2)], body=ad)
        hs = [(ax + x, 0.0, p['sg90_horn_screw_d'] / 2) for x in p['sg90_horn_screw_x']]
        b.xy('cut', 'Horn screws (M2)', a0, a1 + 1, circles=hs, body=ad)
        b.xy('cut', 'Horn screw heads', a1 - 1.5, a1 + 1, circles=[(x, y, 2.2) for x, y, _ in hs], body=ad)
    else:
        ad = b.xy('new', 'Pan horn adapter', a0, a1, circles=[(ax, ay, R_)])
        b.xy('cut', 'Disc horn pocket', a0 - 1, a0 + p['horn_pocket_depth'], circles=[(ax, ay, p['horn_d'] / 2 + 0.3)], body=ad)
        b.xy('cut', 'Horn screws (M3 button head)', a0, a1 + 1, circles=f['horn_screws'], body=ad)
        b.xy('cut', 'Horn screw heads', a1 - 2.0, a1 + 1, circles=[(x, y, 2.9) for x, y, _ in f['horn_screws']], body=ad)
    b.xy('cut', 'Spline screw access', a0, a1 + 1, circles=[(ax, ay, 3.0)], body=ad)
    b.xy('cut', 'Buckle inserts (M3 heat-set, from above)', a1 - 5.7, a1 + 1, circles=[(x, y, 2.0) for x, y in f['buckle_screws']], body=ad)
    z0, zt = f['buckle_base_z']
    mount = b.xy('new', 'GoPro buckle', z0, zt, circles=[(ax, ay, R_)])
    half = p['gp_prong_len'] / 2
    hz = f['gp_hole_z']
    t = p['gp_prong_t']
    for i, y in enumerate(f['gp_prong_y']):
        b.side('join', f'GoPro prong {i + 1}', 'y', y - t / 2, y + t / 2, body=mount,
               rects=[(ax - half, zt - 0.5, ax + half, hz)], circles=[(ax, hz, half)])
    b.side('cut', 'GoPro screw hole', 'y', -f['gp_total_w'] / 2 - 2, f['gp_total_w'] / 2 + 2, body=mount,
           circles=[(ax, hz, p['gp_hole_d'] / 2)])
    b.xy('cut', 'Buckle screws (M3)', z0 - 1, zt + 1, circles=[(x, y, 1.7) for x, y in f['buckle_screws']], body=mount)
    b.xy('cut', 'Buckle screw heads', zt - 2.0, zt + 1, circles=[(x, y, 3.0) for x, y in f['buckle_screws']], body=mount)
    return [ad, mount]


def build_caster(asm, f, p, deck_occ, oy):
    """Caster drivetrain module (robot_design.md §2a), drawn `oy` mm along Y from its mounted position."""
    c = f['cb']
    bodies = []
    occ, b = asm.comp('Caster drivetrain (HMI v1)')

    def sh(r):  # shift a plan rect
        return (r[0], r[1] + oy, r[2], r[3] + oy)

    def Y(s, a0, a1):
        lo, hi = (a0, a1) if s > 0 else (-a1, -a0)
        return lo + oy, hi + oy
    t = p['adapter_t']
    fo = dict(f)
    fo['hmi_pegs'] = [(x, y + oy, r) for x, y, r in f['hmi_pegs']]
    fo['cable_slot'] = sh(f['cable_slot'])
    po = dict(p)
    po['hmi_bolts'] = [(x, y + oy) for x, y in p['hmi_bolts']]
    base = _hmi_plate(b, fo, po, 'Caster base (HMI v1)', sh(c['plate']), -t, 0.0)
    bodies.append(base)
    # stiffening ribs, battery cradle (strap channels each side, velcro strap closes on the plate top), caster pad
    r0, r1 = p['cb_rib_y']
    pl = c['plate']
    b.xy('join', 'Stiffening ribs', -t - p['cb_rib_depth'], -t + 0.5, body=base,
         rects=[sh((pl[0] + 8, r0, pl[2] - 8, r1)), sh((pl[0] + 8, -r1, pl[2] - 8, -r0))])
    b.xy('join', 'Battery cradle', *c['cradle_z'], paths=[rrect_segs(sh(c['cradle_outer']), 3.0)], body=base)
    b.xy('cut', 'Battery bay', c['cradle_z'][0] - 1, -t, rects=[sh(c['cradle_inner'])], body=base)
    co = c['cradle_outer']
    b.xy('join', 'Skid plate ears', c['cradle_z'][0], c['cradle_z'][0] + 8, rects=[sh(r_) for r_ in c['cradle_ears']], body=base)
    b.xy('cut', 'Skid plate inserts (M3 heat-set, from below)', c['cradle_z'][0] - 1, c['cradle_z'][0] + 6.2, body=base,
         circles=[(x, y + oy, 2.0) for x, y in c['skid_holes']])
    sk_ = b.xy('new', 'Battery skid plate', *c['skid_plate_z'], paths=[rrect_segs(sh(c['skid_plate']), 3.0)])
    b.xy('cut', 'Screws (M3)', c['skid_plate_z'][0] - 1, c['skid_plate_z'][1] + 1, circles=[(x, y + oy, 1.7) for x, y in c['skid_holes']], body=sk_)
    bodies.append(sk_)
    b.xy('cut', 'Battery strap slots (through the plate, beside the pack)', -t - 1, 1, rects=[sh(r_) for r_ in c['strap_slots']], body=base)
    if c['caster_spacer'] > 0.2:
        b.xy('join', 'Caster pad', -t - c['caster_spacer'], -t + 0.5, rects=[sh(c['caster_plate'])], body=base)
    b.xy('cut', 'Caster bolts (M4)', -t - c['caster_spacer'] - 1, 1, circles=[(x, y + oy, r) for x, y, r in c['caster_holes']], body=base)
    b.xy('cut', 'ESC zip-tie slots', -t - 1, 1, rects=[sh(r) for r in c['esc_slots']], body=base)
    b.xy('cut', 'Wiring pass-throughs', -t - 1, 1, paths=[rrect_segs(sh(r), 4.0) for r in c['pass_throughs']], body=base)
    cs = f['cable_slot']
    b.xy('cut', 'HMI harness port (through the rib)', -t - p['cb_rib_depth'] - 1, 0.0, body=base,
         rects=[sh((cs[0], cs[1], cs[2], min(cs[3], c['cradle_outer'][1] - 0.5)))])
    ax_, az = c['axle']
    mx, mz = c['motor']
    ux, uz = c['u']
    sl = p['cb_tension_slot'] / 2
    for s in (1, -1):
        b.side('join', 'Axle block', 'y', *Y(s, *c['axle_block_y']), body=base, paths=[rrect_segs(c['axle_block_xz'], (3.0, 3.0, 0.0, 0.0))])
        b.side('join', 'Motor web', 'y', *Y(s, *c['web_y']), body=base, paths=[rrect_segs(c['web_xz'], 4.0)])
        wy0 = c['web_y'][0]
        gz0 = max(c['web_xz'][1], -t - 22.0)
        for gx in c['web_gussets_x']:
            b.side('join', 'Web gusset', 'x', gx - 2.0, gx + 2.0, body=base,
                   paths=[poly_segs([(s * wy0 + oy, -t + 0.5), (s * (wy0 + 0.5) + oy, gz0), (s * (wy0 - 14.0) + oy, -t + 0.5)])])
        b.side('cut', 'Motor slots (belt tension)', 'y', *Y(s, c['web_y'][0] - 1, c['web_y'][1] + 1), body=base,
               paths=[capsule_segs((x - sl * ux, z - sl * uz), p['cb_motor_holes'][1] / 2, (x + sl * ux, z + sl * uz), p['cb_motor_holes'][1] / 2)
                      for x, z in c['motor_holes']] +
                     [capsule_segs((mx - sl * ux, mz - sl * uz), 6.5, (mx + sl * ux, mz + sl * uz), 6.5)])
    b.side('cut', 'Axle (M8 rod)', 'y', -c['axle_block_y'][1] - 1 + oy, c['axle_block_y'][1] + 1 + oy, body=base,
           circles=[(ax_, az, p['cb_axle_d'] / 2)])
    D, d = c['pd']
    py0, py1 = c['pulley_y']
    for s, side in ((1, 'L'), (-1, 'R')):
        mo = b.side('new', f'Drive motor {side} (5010 360KV, reference)', 'y', *Y(s, *c['motor_y']), circles=[(mx, mz, p['cb_motor'][0] / 2)])
        b.side('join', 'Shaft', 'y', *Y(s, c['motor_y'][1] - 0.5, c['p20_y'][0] + 0.5), circles=[(mx, mz, p['motor_shaft_d'] / 2)], body=mo)
        b.side('join', '20T pulley', 'y', *Y(s, *c['p20_y']), circles=[(mx, mz, d / 2 - 0.254)], body=mo)
        bodies.append(mo)
        if p['printed_belts']:
            by0, by1 = c['belt_y']
            bodies.append(printed_belt(b, f'Drive belt {side} (TPU test print, {p["cb_belt"][0]}T)', *Y(s, by0 + 0.2, by1 - 0.2),
                                       (ax_, az), D, (mx, mz), d, p['cb_belt'][0]))
        # drive wheel + printed 48T pulley (turns as one on the fixed axle)
        wocc, wb = asm.comp(f'Drive wheel {side}')
        wh = wb.side('new', f'Longboard wheel {side} (reference)', 'y', *Y(s, *c['wheel_y']), circles=[(ax_, az, p['cb_wheel'][0] / 2)])
        wb.side('cut', 'Bearing bore', 'y', *Y(s, c['wheel_y'][0] - 1, c['wheel_y'][1] + 1), body=wh, circles=[(ax_, az, 11.0)])
        pu = wb.side('new', f'Wheel pulley {side} ({p["cb_pulley"][0]}T GT2, printed)', 'y', *Y(s, py0 + 1.0, py1 - 1.0),
                     circles=[(ax_, az, D / 2 - 0.254)])
        wb.side('join', 'Flange', 'y', *Y(s, py0, py0 + 1.0), circles=[(ax_, az, D / 2 + 1.5)], body=pu)
        wb.side('join', 'Adapter disc', 'y', *Y(s, py1 - 1.0, c['pulley_disc_y'][1]), circles=[(ax_, az, p['cb_pulley_disc_d'] / 2)], body=pu)
        wb.side('cut', 'Teeth', 'y', *Y(s, py0 + 1.0, py1 - 1.0), circles=teeth_circles((ax_, az), p['cb_pulley'][0], p), body=pu)
        bp, bh = p['cb_pulley_bolts']
        wb.side('cut', 'Clearance bore + core bolts (3 x M4)', 'y', *Y(s, py0 - 1, c['pulley_disc_y'][1] + 1), body=pu,
                circles=[(ax_, az, 5.0)] + [(ax_ + bp / 2 * math.cos(math.radians(90 + 120 * i)), az + bp / 2 * math.sin(math.radians(90 + 120 * i)), bh / 2) for i in range(3)])
        bodies += [wh, pu]
        asm.revolute(f'Drive wheel {side}', wocc, occ, wh, wocc, (ax_, s * (c['wheel_y'][0]) + oy, az), p['cb_wheel'][0] / 2)
    # bought swivel caster (reference), on its own swivel joint
    cocc, cb_ = asm.comp('Caster (reference)')
    cx = p['cb_caster_x']; zt = c['caster_top_z']
    ctp = cb_.xy('new', 'Swivel caster top plate (reference)', zt - 4, zt, rects=[sh(c['caster_plate'])])
    cb_.xy('join', 'Swivel', zt - 12, zt - 4, circles=[(cx, oy, 15.0)], body=ctp)
    gz = c['ground_z']
    cw = cb_.side('new', 'Caster wheel 50 mm (reference)', 'y', oy - 10, oy + 10, circles=[(cx - 20, gz + 25, 25.0)])
    cb_.side('join', 'Fork', 'y', oy - 12, oy - 10, rects=[(cx - 25, gz + 25, cx + 5, zt - 12)], body=cw)
    cb_.side('join', 'Fork', 'y', oy + 10, oy + 12, rects=[(cx - 25, gz + 25, cx + 5, zt - 12)], body=cw)
    bodies += [ctp, cw]
    asm.revolute('Caster swivel', cocc, occ, ctp, cocc, (cx, oy, zt - 12), 15.0)
    if oy == 0 and deck_occ:
        asm.rigid('Caster drivetrain to deck', occ, deck_occ)
    else:
        try:
            occ.isGrounded = True
        except Exception:
            pass
    return bodies


def build_deck(b, root, f, p):
    L, W, ft, wt = p['deck_length'], p['deck_width'], p['floor_t'], p['wall_t']
    cr = p['corner_r']
    deck = b.xy('new', 'Chassis deck', 0, p['wall_height'], paths=[rrect_segs(f['deck_outer'], cr)])
    b.xy('cut', 'Deck cavity', ft, p['wall_height'] + 1, paths=[rrect_segs(f['deck_inner'], f['deck_inner_r'])], body=deck)
    # Module interface
    b.xy('join', 'HMI insert bosses', *f['hmi_boss_z'], circles=f['hmi_bosses'], body=deck)
    b.xy('cut', 'HMI insert holes (M3 heat-set, from below)', *f['hmi_insert_z'], circles=f['hmi_inserts'], body=deck)
    b.xy('cut', 'HMI locating hole (round)', 0, ft, circles=[f['hmi_round_hole']], body=deck)
    e0, e1 = f['hmi_slot']['ends']
    b.xy('cut', 'HMI locating slot', 0, ft, paths=[capsule_segs(e0[:2], e0[2], e1[:2], e1[2])], body=deck)
    b.xy('cut', 'HMI harness port', 0, ft, paths=[rrect_segs(f['cable_slot'], 3.0)], body=deck)
    # Pi 5
    b.xy('join', 'Pi 5 standoffs', *f['pi_standoff_z'], circles=[(x, y, p['pi_standoff_od'] / 2) for x, y in f['pi_holes']], body=deck)
    b.xy('cut', 'Pi 5 screw holes', 0, f['pi_standoff_z'][1], circles=[(x, y, p['pi_hole_d'] / 2) for x, y in f['pi_holes']], body=deck)
    # ESP32
    b.xy('join', 'ESP32 end ledges', *f['esp_ledge_z'], rects=f['esp_ledges'], body=deck)
    b.xy('join', 'ESP32 rails', *f['esp_rail_z'], rects=f['esp_rails'], body=deck)
    if p['esp_relief'] > p['esp_lip_in'] + 0.1:
        b.xy('cut', 'ESP32 rail relief', *f['esp_relief_z'], rects=f['esp_reliefs'], body=deck)
    # Bulkhead: solid block (front corners rounded like the deck), hollowed from the back
    top = f['tower_top']
    b.xy('join', 'Bulkhead', ft, top, paths=[rrect_segs(f['tower'], (0.0, cr, cr, 0.0))], body=deck)
    ir = f['deck_inner_r']
    b.xy('cut', 'Bulkhead cavity', ft, top - p['tower_roof_t'], paths=[rrect_segs(f['tower_cavity'], (0.0, ir, ir, 0.0))], body=deck)
    if p['build_lid']:
        b.xy('join', 'Lid keeper ledges', *f['lid_keeper_z'], rects=f['lid_keepers'], body=deck)
        if f['lintel_z'][1] > f['lintel_z'][0] + 0.6:
            b.xy('join', 'Bulkhead lintel', *f['lintel_z'], rects=[f['lintel']], body=deck)
    b.xy('cut', 'Servo pocket', top - p['tower_roof_t'] - 1, top + 1, rects=[f['servo_pocket']], body=deck)
    spk = f['servo_pocket']; sw_, sl_c = p['servo_cable_slit']; axp_ = f['pan_axis'][0]
    b.xy('cut', 'Servo cable slit', top - p['tower_roof_t'], top + 1, rects=[(axp_ - sw_ / 2, spk[1] - sl_c, axp_ + sw_ / 2, spk[1] + 0.5)], body=deck)
    b.xy('join', 'Servo insert bosses', *f['servo_boss_z'], circles=f['servo_bosses'], body=deck)
    b.xy('cut', 'Servo tab inserts (M3 heat-set, from above)', *f['servo_insert_z'], circles=f['servo_screws'], body=deck)
    b.xy('join', 'Power web (buck + servo BEC)', *f['power_web_z'], rects=f['power_web'], body=deck)
    b.xy('join', 'Zip-tie anchors (floor)', *f['deck_tie_anchor_z'], rects=f['deck_tie_anchors'], body=deck)
    for r_ in f['deck_tie_anchors']:
        b.side('cut', 'Zip-tie tunnel', 'x', r_[0] - 1, r_[2] + 1, rects=[f['deck_tie_tunnel']], body=deck)
    for (x0, y0, x1, y1, z0_, z1_) in f['power_web_slots']:
        b.xy('cut', 'Zip-tie slot', z0_, z1_, rects=[(x0, y0, x1, y1)], body=deck)
    c = p['tower_chamfer']
    if c > 0:
        b.side('cut', 'Bulkhead front chamfer', 'y', -W / 2 - 1, W / 2 + 1, body=deck,
               paths=[poly_segs([(L / 2 + 1, top - c - 1), (L / 2 - c - 1, top + 1), (L / 2 + 1, top + 1)])])
    # HC-SR04 through the front face
    b.side('cut', 'HC-SR04 holes', 'x', L / 2 - wt - 1, L / 2 + 1, circles=f['sr04_holes'], body=deck)
    if p['build_lid']:
        b.xy('join', 'Lid posts', *f['lid_post_z'], circles=f['lid_posts'], body=deck)
        b.xy('cut', 'Lid post inserts (M3 heat-set, from above)', *f['lid_post_insert_z'], circles=f['lid_post_inserts'], body=deck)
        # a lid post stands on an HMI insert: cut that insert hole again so the post doesn't fill it
        hit = [c for c in f['hmi_inserts'] if any(math.dist(c[:2], q[:2]) < q[2] + c[2] for q in f['lid_posts'])]
        if hit:
            b.xy('cut', 'HMI insert holes (under the lid posts)', *f['hmi_insert_z'], circles=hit, body=deck)
    return deck


def build_lid(b, f, p):
    """Vented electronics lid, drawn in its assembled position (print it upside down)."""
    L, W = p['deck_length'], p['deck_width']
    z0, zt = f['lid_bottom'], f['lid_top']
    lid = b.xy('new', 'Electronics lid', z0, zt, paths=[rrect_segs(f['lid_outer'], f['lid_outer_r'])])
    b.xy('cut', 'Lid cavity', z0 - 1, f['lid_ceiling'], paths=[rrect_segs(f['lid_inner'], f['lid_inner_r'])], body=lid)
    b.xy('join', 'Lid skirt webs', *f['lid_skirt_web_z'], rects=f['lid_skirt_webs'], body=lid)
    b.xy('join', 'Lid locating skirts', *f['lid_skirt_z'], rects=f['lid_skirt'], body=lid)
    b.xy('join', 'Lid front tongues', *f['lid_tongue_z'], rects=f['lid_tongues'], body=lid)
    b.xy('cut', 'Pi USB/Ethernet slot', *f['lid_rear_port_z'], paths=[rrect_segs(f['lid_rear_port_cut'], (0.0, 0.0, 0.0, 0.0))], body=lid)
    b.xy('cut', 'Pi USB-C/HDMI slot', *f['lid_side_port_z'], rects=[f['lid_side_port']], body=lid)
    # airflow: round-ended intake slots over the fan, exhaust slots low on the side and rear walls
    b.xy('cut', 'Intake vents (over Pi fan)', f['lid_ceiling'] - 1, zt + 1, paths=[slot_segs(v) for v in f['lid_top_vents']], body=lid)
    vz0, vz1 = f['lid_vent_z']
    lw = p['lid_wall_t']
    b.side('cut', 'Exhaust vents (side)', 'y', -W / 2 - 1, -W / 2 + lw + 1, body=lid,
           paths=[slot_segs((v[0], vz0, v[2], vz1)) for v in f['lid_side_vents']])
    b.side('cut', 'Exhaust vents (rear)', 'x', -L / 2 - 1, -L / 2 + lw + 1, body=lid,
           paths=[slot_segs((v[1], vz0, v[3], vz1)) for v in f['lid_rear_vents']])
    b.xy('cut', 'Lid screw holes', f['lid_ceiling'] - 1, zt + 1, circles=f['lid_screws'], body=lid)
    c = p['lid_chamfer']
    if c > 0:
        nx0 = f['lid_front_x']
        b.side('cut', 'Lid chamfer (rear)', 'y', -W / 2 - 1, W / 2 + 1, body=lid,
               paths=[poly_segs([(-L / 2 - 1, zt - c - 1), (-L / 2 + c + 1, zt + 1), (-L / 2 - 1, zt + 1)])])
        for s in (1, -1):
            b.side('cut', 'Lid chamfer (side)', 'x', -L / 2 - 1, nx0 + 1, body=lid,
                   paths=[poly_segs([(s * (W / 2 + 1), zt - c - 1), (s * (W / 2 - c - 1), zt + 1), (s * (W / 2 + 1), zt + 1)])])
    return lid


def _hmi_plate(b, f, p, name, outline, z0, z1):
    """A plate whose top face is the HMI v1 mating face (pegs up, screws from below)."""
    pl = b.xy('new', name, z0, z1, paths=[rrect_segs(outline, p['corner_r'])])
    b.xy('join', 'HMI locating pegs', z1, z1 + p['hmi_peg_h'], circles=f['hmi_pegs'], body=pl)
    b.xy('cut', 'HMI screw clearance', z0, z1, circles=[(x, y, p['adapter_screw_clear'] / 2) for x, y in p['hmi_bolts']], body=pl)
    b.xy('cut', 'HMI screw counterbores (from below)', z0 - 60, z0 + p['adapter_cbore_depth'],
         circles=[(x, y, p['adapter_cbore_d'] / 2) for x, y in p['hmi_bolts']], body=pl)
    b.xy('cut', 'HMI harness port', z0, z1, paths=[rrect_segs(f['cable_slot'], 3.0)], body=pl)
    return pl


def build_adapter(b, f, p):
    """Template adapter for other drivetrains: HMI on top, M3 grid with flush nut pockets."""
    z0, z1 = f['adapter_z']
    ad = _hmi_plate(b, f, p, 'Drive adapter (HMI v1)', f['adapter_outer'], z0, z1)
    if f['adapter_grid']:
        b.xy('cut', 'Mounting grid (M3)', z0, z1, circles=f['adapter_grid'], body=ad)
        b.xy('cut', 'M3 nut pockets (top face)', z1 - p['adapter_nut_depth'], z1, body=ad,
             paths=[poly_segs(hex_pts((x, y), p['adapter_nut_af'], 30)) for x, y, _ in f['adapter_grid']])
    return ad


def build_legs(asm, f, p, deck_occ):
    """Hip module + both legs, drawn at leg_stance_deg; each moving part is its own component."""
    bodies = []
    hip_occ, b = asm.comp('Ostrich hip module (HMI v1)')
    hx, hz = f['hip']
    hip = _hmi_plate(b, f, p, 'Hip module (HMI v1)', f['hip_plate'], -p['adapter_t'], 0.0)
    bodies.append(hip)
    bx0, bz0, bx1, bz1 = f['hip_block_xz']
    knee, axle = leg_pose(f, p, p['leg_stance_deg'])
    hb_id, hb_od, _ = p['hip_bearing']

    def Ys(s_, a0, a1):
        return (a0, a1) if s_ > 0 else (-a1, -a0)
    # ---- hip blocks (part of the hip module), hanging from the plate ----
    for s, side in ((1, 'L'), (-1, 'R')):
        def Y(a0, a1):
            return Ys(s, a0, a1)
        b.side('join', f'Hip block {side}', 'y', *Y(f['hip_block_y'][0], f['hip_block_y'][1]), body=hip,
               paths=[rrect_segs((bx0, bz0, bx1, min(bz1, -1.0)), (4.0, 4.0, 0.0, 0.0))])
        b.side('cut', f'Hip servo pocket {side}', 'y', *Y(f['hip_block_y'][0] - 1, f['hip_servo_top_y'] + 0.3), body=hip,
               rects=[f['hip_servo_pocket_xz']])
        b.side('cut', f'Hip servo tab screws {side}', 'y', *Y(f['hip_block_y'][0] - 1, f['hip_block_y'][0] + 10), body=hip,
               circles=[(x, z, p['hip_servo_tab_hole_d'] / 2) for x, z in f['hip_servo_tab_holes_xz']])
        b.side('cut', f'Hip horn clearance {side}', 'y', *Y(f['hip_servo_top_y'], f['hip_lip_y'][0]), body=hip,
               circles=[(hx, hz, p['hip_servo_horn_d'] / 2 + 1.5)])
        b.side('cut', f'Bore for the hip pulley bearing sleeve {side}', 'y', *Y(f['hip_lip_y'][0] - 0.3, f['hip_bearing_y'][1] + 1), body=hip,
               circles=[(hx, hz, f['pulley_sleeve_r'] + 0.2)])
        b.side('cut', f'Hip pulley inserts {side}', 'y', *Y(p['hip_face_y'] - 5.7, p['hip_face_y'] + 1), body=hip,
               circles=[(x, z, 2.0) for x, z in f['pulley_screws_xz']])
    # ---- battery tray under the servos (ties the two blocks together), strap slots, skids ----
    b.xy('join', 'Battery tray', *f['tray_z'], paths=[rrect_segs(f['tray_outer'], 4.0)], body=hip)
    b.xy('cut', 'Battery bay (open at the front)', *f['tray_inner_z'], rects=[f['tray_inner']], body=hip)
    # battery: one 15 mm velcro strap wrapped lengthwise round the whole tray on the centre line (under the floor,
    # up the closed rear wall, over the roof between the servos, down across the open mouth); guides keep it centred
    sw2 = p['tray_strap_w'] / 2 + 0.5
    b.xy('join', 'Strap guides (floor)', f['tray_z'][0] - 1.5, f['tray_z'][0] + 0.5, body=hip,
         rects=[(f['tray_outer'][0] + 3, sw2, f['tray_outer'][2] - 3, sw2 + 2), (f['tray_outer'][0] + 3, -sw2 - 2, f['tray_outer'][2] - 3, -sw2)])
    r = f['skid_body_r']
    for x, zs in f['skids']:
        b.side('join', 'Parking skid', 'y', -p['skid_w'] / 2, p['skid_w'] / 2, body=hip,
               paths=[capsule_segs((x, -r - 0.5), r, (x, zs + r), r)])
    # electronics web (2 FOC drivers on the back face, MCU + servo BEC on the front), IMU bosses on the axle line,
    # and pass-throughs where the leg wires come over the plate edge
    b.xy('join', 'Electronics web', *f['hip_ebay_z'], rects=[f['hip_ebay']], body=hip)
    for (x0, y0, x1, y1, z0_, z1_) in f['hip_ebay_slots']:
        b.xy('cut', 'Zip-tie slot', z0_, z1_, rects=[(x0, y0, x1, y1)], body=hip)
    b.xy('join', 'IMU bosses', *f['imu_boss_z'], circles=f['imu_bosses'], body=hip)
    b.xy('cut', 'IMU screws (M2.5 self-tapping)', f['imu_boss_z'][0] - 1, -1.0, circles=[(x, y, 1.05) for x, y, _ in f['imu_bosses']], body=hip)
    b.xy('cut', 'Leg wire slots (open to the edge)', -p['adapter_t'] - 1, 1, paths=[rrect_segs(r_, 3.0) for r_ in f['hip_wire_pass']], body=hip)
    b.xy('join', 'Zip-tie anchors (plate top)', *f['hip_tie_anchor_z'], rects=f['hip_tie_anchors'], body=hip)
    for r_, t_ in zip(f['hip_tie_anchors'], f['hip_tie_tunnels']):
        b.side('cut', 'Zip-tie tunnel', 'x', r_[0] - 1, r_[2] + 1, rects=[t_], body=hip)
    # wire fairlead: an arm over the hip pulley, on the hip axis line, so the leg wires never cross the belt
    b.xy('join', 'Wire fairlead roots', -0.5, f['hip_fairlead_z'][0] + 0.5, rects=f['hip_fairlead_root'], body=hip)
    b.xy('join', 'Wire fairlead arms', *f['hip_fairlead_z'], rects=f['hip_fairleads'], body=hip)
    for r_, t_ in zip(f['hip_fairleads'], f['hip_fairlead_tunnels']):
        b.side('cut', 'Zip-tie tunnel', 'x', r_[0] - 1, r_[2] + 1, rects=[t_], body=hip)

    for s, side in ((1, 'L'), (-1, 'R')):
        def Y(a0, a1):
            return Ys(s, a0, a1)
        # ---- hip pulley (80T, fixed) ----
        py0, py1 = f['hip_pulley_y']; ty0, ty1 = f['pulley_teeth_y']
        dh, dk = f['pulley_pd']
        hp = b.side('new', f'Hip pulley {side} ({p["pulley_hip_teeth"]}T GT2)', 'y', *Y(ty0, ty1), circles=[(hx, hz, dh / 2 - 0.254)])
        b.side('join', 'Flanges', 'y', *Y(py0, ty0), circles=[(hx, hz, f['hip_pulley_flange_r'])], body=hp)
        b.side('join', 'Flanges', 'y', *Y(ty1, py1), circles=[(hx, hz, f['hip_pulley_flange_r'])], body=hp)
        b.side('cut', 'Teeth', 'y', *Y(ty0, ty1), circles=teeth_circles((hx, hz), p['pulley_hip_teeth'], p), body=hp)
        b.side('join', 'Bearing sleeve', 'y', *Y(f['hip_lip_y'][0], py0 + 0.5), circles=[(hx, hz, f['pulley_sleeve_r'])], body=hp)
        b.side('cut', 'Bore', 'y', *Y(py0 - 1, py1 + 1), circles=[(hx, hz, p['pulley_hip_bore'] / 2)], body=hp)
        b.side('cut', 'Hip bearing seats (2 x 6804)', 'y', *Y(f['hip_bearing_y'][0], f['hip_bearing_y'][1] + 0.01), body=hp,
               circles=[(hx, hz, hb_od / 2 + p['fit_press'] / 2)])
        b.side('cut', 'Bearing stop', 'y', *Y(f['hip_lip_y'][0] - 1, f['hip_lip_y'][1] + 0.01), body=hp,
               circles=[(hx, hz, p['hip_lip'][0] / 2)])
        b.side('cut', 'Screws', 'y', *Y(py0 - 1, py1 + 1), circles=[(x, z, 1.7) for x, z in f['pulley_screws_xz']], body=hp)
        b.side('cut', 'Screw heads', 'y', *Y(py1 - 3.2, py1 + 1), circles=[(x, z, 3.0) for x, z in f['pulley_screws_xz']], body=hp)
        bodies.append(hp)

        # ---- thigh (own component: turns on the hip) ----
        th_occ, tb = asm.comp(f'Thigh {side}')
        t0, t1 = f['thigh_y']
        th = tb.side('new', f'Thigh {side}', 'y', *Y(t0, t1),
                    paths=[capsule_segs((hx, hz), p['thigh_r_hip'], knee, p['thigh_r_knee'])])
        tb.side('join', 'Hip hub shoulder', 'y', *Y(f['hip_bearing_y'][1] + 0.2, t0 + 0.5), body=th,
               circles=[(hx, hz, p['thigh_shoulder_d'] / 2)])
        tb.side('join', 'Hip hub', 'y', *Y(f['hub_end_y'], f['hip_bearing_y'][1] + 0.7), body=th,
               circles=[(hx, hz, (p['thigh_hub_d'] - p['fit_shaft']) / 2)])
        # horn screws (4) and the spline screw go in from the thigh's outer face, through the hub
        hs = [(x, z) for x, z in f['horn_screws_xz']] + [(hx, hz)]
        # 4.0 mm holes: the hub centres itself in the bearings and the horn floats to suit before the screws are tightened,
        # so a printed misalignment never forces the servo's output shaft sideways
        tb.side('cut', 'Horn + spline screw holes (M3, floating)', 'y', *Y(f['hub_end_y'] - 1, t1 + 1), body=th,
               circles=[(x, z, 2.0) for x, z in hs])
        tb.side('cut', 'Horn + spline screw heads', 'y', *Y(t1 - 3.2, t1 + 1), body=th,
               circles=[(x, z, 3.0) for x, z in hs])
        kb_id, kb_od, _ = p['knee_bearing']
        tb.side('cut', 'Knee bearing seat', 'y', *Y(t0 - 1, t1 + 1), body=th, circles=[(knee[0], knee[1], kb_od / 2 + p['fit_press'] / 2)])
        # belt tensioner slot across the middle of the thigh
        mx, mz = (hx + knee[0]) / 2, (hz + knee[1]) / 2
        ux, uz = (knee[0] - hx) / f['leg_len'], (knee[1] - hz) / f['leg_len']
        nx_, nz_ = -uz, ux
        if nz_ < 0:
            nx_, nz_ = -nx_, -nz_
        o0, o1, idd = p['idler_slot']
        tb.side('cut', 'Belt tensioner slot', 'y', *Y(t0 - 1, t1 + 1), body=th,
               paths=[capsule_segs((mx + nx_ * o0, mz + nz_ * o0), idd / 2, (mx + nx_ * o1, mz + nz_ * o1), idd / 2)])
        # zip-tie slot at the hip boss, close to the hip axis: the wire loop to the fairlead barely changes length
        hr_ = p['thigh_r_hip'] - 4.5
        tx_, tz_ = hx + nx_ * hr_, hz + nz_ * hr_
        tb.side('cut', 'Wire tie slot (hip)', 'y', *Y(t0 - 1, t1 + 1), body=th,
                paths=[capsule_segs((tx_ - ux * 2.5, tz_ - uz * 2.5), 1.5, (tx_ + ux * 2.5, tz_ + uz * 2.5), 1.5)])
        # zip-tie slots near the thigh's upper edge: the wires run along that edge (both faces are swept)
        for frac in (0.3, 0.7):
            cx_, cz_ = hx + (knee[0] - hx) * frac, hz + (knee[1] - hz) * frac
            rr_ = p['thigh_r_hip'] + (p['thigh_r_knee'] - p['thigh_r_hip']) * frac - 4.0
            sx_, sz_ = cx_ + nx_ * rr_, cz_ + nz_ * rr_
            tb.side('cut', 'Wire clip slot', 'y', *Y(t0 - 1, t1 + 1), body=th,
                    paths=[capsule_segs((sx_ - ux * 2.5, sz_ - uz * 2.5), 1.5, (sx_ + ux * 2.5, sz_ + uz * 2.5), 1.5)])
        bodies.append(th)
        if p['printed_belts']:
            bodies.append(printed_belt(tb, f'Leg belt {side} (TPU test print, {p["belt_teeth"]}T)', *Y(ty0 + 0.2, ty1 - 0.2),
                                       (hx, hz), dh, knee, dk, p['belt_teeth']))
        j_hip = asm.revolute(f'Hip {side}', th_occ, hip_occ, th, th_occ, (hx, s * f['hub_end_y'], hz), (p['thigh_hub_d'] - p['fit_shaft']) / 2)

        # ---- knee pulley + shank + wheel motor (own component: turns on the knee) ----
        sh_occ, sb = asm.comp(f'Shank {side}')
        kp = sb.side('new', f'Knee pulley {side} ({p["pulley_knee_teeth"]}T GT2)', 'y', *Y(ty0, ty1),
                    circles=[(knee[0], knee[1], dk / 2 - 0.254)])
        kfr = dk / 2 - 0.254 + p['pulley_flange'] + 1.0
        sb.side('join', 'Flanges', 'y', *Y(py0, ty0), circles=[(knee[0], knee[1], kfr)], body=kp)
        sb.side('join', 'Flanges', 'y', *Y(ty1, py1), circles=[(knee[0], knee[1], kfr)], body=kp)
        sb.side('cut', 'Teeth', 'y', *Y(ty0, ty1), circles=teeth_circles(knee, p['pulley_knee_teeth'], p), body=kp)
        sb.side('cut', 'Hex socket', 'y', *Y(py0 - 1, py1 + 1), body=kp,
               paths=[poly_segs(hex_pts(knee, p['knee_hex_af'] + 0.3, 0.0))])
        bodies.append(kp)

        # ---- shank: knee, foot (wheel axle) and the wheel motor, as one triangular plate ----
        s0, s1 = f['shank_y']
        mot = foot_motor(f, p, p['leg_stance_deg'])
        mr_plate = p['motor_can_d'] / 2 - 2.0
        sh = sb.side('new', f'Shank {side}', 'y', *Y(s0, s1),
                     paths=[capsule_segs(knee, p['shank_r_knee'], axle, p['shank_r_foot'])])
        sb.side('join', 'Motor plate', 'y', *Y(s0, s1), body=sh, paths=[capsule_segs(axle, p['shank_r_foot'], mot, mr_plate)])
        sb.side('join', 'Knee-motor web', 'y', *Y(s0, s1), body=sh, paths=[capsule_segs(knee, p['shank_r_knee'], mot, mr_plate)])
        # post lugs join before any cut at the motor, or they would refill the screw holes, encoder pocket and wire groove
        br_ = p['motor_bumper'][0]
        posts = foot_posts(f, p, p['leg_stance_deg'])
        for i_, q in enumerate(posts):     # one extrude per capsule: overlapping outlines in one sketch are unreliable
            sb.side('join', f'Post lug {i_ + 1}', 'y', *Y(s0, s1), body=sh, paths=[capsule_segs(mot, mr_plate, q, br_)])
        sb.side('join', 'Knee hub', 'y', *Y(py1 + 0.3, s0 + 0.5), body=sh, circles=[(knee[0], knee[1], (p['knee_hub_d'] - p['fit_shaft']) / 2)])
        sb.side('join', 'Knee hex key', 'y', *Y(py0, py1 + 0.3 + 0.2), body=sh,
                paths=[poly_segs(hex_pts(knee, p['knee_hex_af'], 0.0))])
        sb.side('cut', 'Knee bolt (M5)', 'y', *Y(py0 - 1, s1 + 1), body=sh, circles=[(knee[0], knee[1], p['knee_bolt_d'] / 2)])
        # wheel axle: M8 bolt from outside into a nut captured in a boss on the shank's outer face
        fbr, fbh = p['foot_boss']
        sb.side('join', 'Axle boss', 'y', *Y(*f['foot_boss_y']), body=sh, circles=[(axle[0], axle[1], fbr)])
        naf, nt_ = p['axle_head']
        # belt tension: the axle slides +/- axle_slot along the belt line; the nut sits flats-to-walls so it can't turn
        bx_, bz_ = (axle[0] - mot[0]) / f['foot_cc'], (axle[1] - mot[1]) / f['foot_cc']
        px_, pz_ = -bz_, bx_
        sl_ = p['axle_slot']
        hl_ = naf / math.sqrt(3) + sl_ + 0.3
        hw_ = naf / 2 + 0.15
        nut_slot = [(axle[0] + i_ * hl_ * bx_ + j_ * hw_ * px_, axle[1] + i_ * hl_ * bz_ + j_ * hw_ * pz_) for i_, j_ in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
        sb.side('cut', 'Axle bolt head slot (M8, inner face)', 'y', *Y(s0 - 1, s0 + nt_), body=sh,
                paths=[poly_segs(nut_slot)])
        sb.side('cut', 'Axle slot (M8)', 'y', *Y(s0 - 1, f['foot_boss_y'][1] + 1), body=sh,
                paths=[capsule_segs((axle[0] - sl_ * bx_, axle[1] - sl_ * bz_), 4.2, (axle[0] + sl_ * bx_, axle[1] + sl_ * bz_), 4.2)])
        # motor mount: fixed, encoder pocket centred under the shaft end
        pcd = p['motor_stator_pcd'] / 2
        mh = [(mot[0] + pcd * math.cos(math.radians(90 * i)), mot[1] + pcd * math.sin(math.radians(90 * i))) for i in range(4)]  # on the pocket's flats
        sb.side('cut', 'Motor mount: screws', 'y', *Y(s0 - 1, s1 + 1), body=sh,
                circles=[(x, z, p['motor_screw_d'] / 2) for x, z in mh] + [(mot[0], mot[1], p['motor_center_bore'] / 2)])
        hd, hdep = p['motor_head_cbore']
        sb.side('cut', 'Motor mount: screw heads', 'y', *Y(s0 - 1, s0 + hdep), body=sh, circles=[(x, z, hd / 2) for x, z in mh])
        eb = p['enc_board']; ed = p['enc_pocket_depth']
        sb.side('cut', 'Encoder pocket (under the motor shaft end)', 'y', *Y(s1 - ed, s1 + 1), body=sh,
                rects=[(mot[0] - eb[0] / 2 - 0.2, mot[1] - eb[1] / 2 - 0.2, mot[0] + eb[0] / 2 + 0.2, mot[1] + eb[1] / 2 + 0.2)])
        kd = math.hypot(knee[0] - mot[0], knee[1] - mot[1])
        gw, gd = p['wire_groove']
        ge = (mot[0] + (knee[0] - mot[0]) * (kd - p['shank_r_knee'] - 2) / kd, mot[1] + (knee[1] - mot[1]) * (kd - p['shank_r_knee'] - 2) / kd)
        sb.side('cut', 'Motor + encoder wire groove (to the knee)', 'y', *Y(s1 - gd, s1 + 1), body=sh, paths=[capsule_segs(mot, gw / 2, ge, gw / 2)])
        # two posts, one each side of the belt: they land before the motor's 20T pulley in a side fall and carry the outrigger
        sb.side('join', 'Posts (bumpers + outrigger supports)', 'y', *Y(*f['bumper_y']), body=sh, circles=[(q[0], q[1], br_) for q in posts])
        sb.side('cut', 'Outrigger inserts (M3 heat-set, post ends)', 'y', *Y(f['bumper_y'][1] - 6.2, f['bumper_y'][1] + 1), body=sh,
                circles=[(q[0], q[1], 2.0) for q in posts])
        bodies.append(sh)
        j_knee = asm.revolute(f'Knee {side}', sh_occ, th_occ, sh, sh_occ, (knee[0], s * py0, knee[1]), p['knee_bolt_d'] / 2)
        asm.link(f'Leg coupling {side} (belt 2:1)', j_hip, j_knee, 10, 20)

        # ---- wheel motor + 20T pulley (reference only), and the TPU test belt (all fixed to the shank) ----
        fd, fD = f['foot_pd']
        mo = sb.side('new', f'Wheel motor {side} (5010 360KV, reference - buy, do not print)', 'y', *Y(*f['motor_y']),
                     circles=[(mot[0], mot[1], p['motor_can_d'] / 2)])
        sb.side('join', 'Shaft', 'y', *Y(f['motor_y'][1] - 0.5, f['p20_y'][0] + 0.5), body=mo, circles=[(mot[0], mot[1], p['motor_shaft_d'] / 2)])
        sb.side('join', '20T pulley', 'y', *Y(*f['p20_y']), body=mo, circles=[(mot[0], mot[1], fd / 2 - 0.254)])
        bodies.append(mo)
        # outrigger: a plate over the foot, bolted to both posts, holding the axle's outer end and (with shaft_support)
        # a bearing on the motor shaft end, so belt pull closes inside the plate instead of bending the shaft and shank
        o0, o1 = f['outrigger_y']
        sl_o = p['axle_slot']
        bxo, bzo = (axle[0] - mot[0]) / f['foot_cc'], (axle[1] - mot[1]) / f['foot_cc']
        sid, sod, sbw = p['shaft_bearing']
        rb_ = sod / 2 + 3.0
        arms = [capsule_segs(q, br_, axle, 10.0) for q in posts] + [capsule_segs(q, br_, mot, rb_) for q in posts]
        og_ = sb.side('new', f'Wheel outrigger {side}', 'y', *Y(o0, o1), paths=[arms[0]])
        for arm in arms[1:]:       # one extrude per arm: overlapping outlines in one sketch are unreliable
            sb.side('join', 'Outrigger arm', 'y', *Y(o0, o1), body=og_, paths=[arm])
        # axle slot with square ends, so flat printed shims lock the axle here as well as behind the bolt head
        hl_o, hw_o = sl_o + 4.2, 4.2
        sb.side('cut', 'Axle slot (M8, square ends for shims)', 'y', *Y(o0 - 1, o1 + 1), body=og_,
                paths=[poly_segs([(axle[0] + i_ * hl_o * bxo - j_ * hw_o * bzo, axle[1] + i_ * hl_o * bzo + j_ * hw_o * bxo)
                                  for i_, j_ in ((-1, -1), (1, -1), (1, 1), (-1, 1))])])
        sb.side('cut', 'Post screws (M3)', 'y', *Y(o0 - 1, o1 + 1), body=og_, circles=[(q[0], q[1], 1.7) for q in posts])
        sb.side('cut', 'Post screw heads', 'y', *Y(o1 - 2.0, o1 + 1), body=og_, circles=[(q[0], q[1], 3.0) for q in posts])
        if f['shaft_supported']:
            sb.side('cut', f'Shaft bearing seat ({sid:.0f}x{sod:.0f}x{sbw:.0f})', 'y', *Y(o0 - 1, o0 + sbw), body=og_,
                    circles=[(mot[0], mot[1], sod / 2 + p['fit_press'] / 2)])
        sb.side('cut', 'Shaft end clearance', 'y', *Y(o0 - 1, o1 + 1), body=og_, circles=[(mot[0], mot[1], p['motor_shaft_d'] / 2 + 1.0)])
        bodies.append(og_)
        if p['printed_belts']:
            ft0, ft1 = f['foot_teeth_y']
            bodies.append(printed_belt(sb, f'Wheel belt {side} (TPU test print, {p["foot_belt"][0]}T)', *Y(ft0 + 0.3, ft1 - 0.3),
                                       axle, fD, mot, fd, p['foot_belt'][0]))

        # ---- wheel: printed hub with the 48T pulley built in, 2 x 608 on the fixed axle, TPU tire ----
        wh_occ, wb = asm.comp(f'Wheel {side}')
        R = p['wheel_d'] / 2; rr = R - p['tire_t']
        w0, w1 = f['tire_y']
        h0, h1 = f['hub_y']
        ft0, ft1 = f['foot_teeth_y']
        b_id, b_od, b_w = p['bearing_608']
        wh = wb.side('new', f'Wheel hub {side}', 'y', *Y(w0, w1), circles=[(axle[0], axle[1], rr)])
        wb.side('join', 'Pulley core', 'y', *Y(w1 - 0.5, h1), body=wh, circles=[(axle[0], axle[1], fD / 2 - 0.254)])
        wb.side('join', 'Pulley flange', 'y', *Y(ft0 - p['pulley_flange'], ft0), body=wh, circles=[(axle[0], axle[1], fD / 2 + 2.0)])
        wb.side('join', 'Pulley flange', 'y', *Y(ft1, h1), body=wh, circles=[(axle[0], axle[1], fD / 2 + 2.0)])
        wb.side('cut', '48T teeth', 'y', *Y(ft0, ft1), body=wh, circles=teeth_circles(axle, p['foot_teeth'][1], p))
        wb.side('cut', 'Spacer tube clearance', 'y', *Y(h0 - 1, h1 + 1), body=wh, circles=[(axle[0], axle[1], p['axle_spacer_od'] / 2 + 0.3)])
        wb.side('cut', 'Bearing seat (inner)', 'y', *Y(h0 - 1, h0 + b_w), body=wh, circles=[(axle[0], axle[1], b_od / 2 + p['fit_press'] / 2)])
        wb.side('cut', 'Bearing seat (outer)', 'y', *Y(h1 - b_w, h1 + 1), body=wh, circles=[(axle[0], axle[1], b_od / 2 + p['fit_press'] / 2)])
        n = p['wheel_light_holes']
        r_in, r_out = b_od / 2 + 4.0, rr - 3.0
        rl = (r_in + r_out) / 2
        hl = min((r_out - r_in) / 2 - 1.0, rl * math.sin(math.pi / n) - 3.0)
        wb.side('cut', 'Lightening holes', 'y', *Y(w0 - 1, w1 + 1), body=wh,
                circles=[(axle[0] + rl * math.cos(2 * math.pi * (i + 0.5) / n), axle[1] + rl * math.sin(2 * math.pi * (i + 0.5) / n), hl) for i in range(n)])
        bodies.append(wh)
        ti = wb.side('new', f'Tire {side} (TPU)', 'y', *Y(w0, w1), circles=[(axle[0], axle[1], R)])
        wb.side('cut', 'Tire bore', 'y', *Y(w0 - 1, w1 + 1), body=ti, circles=[(axle[0], axle[1], rr)])
        bodies.append(ti)
        asm.revolute(f'Wheel {side}', wh_occ, sh_occ, ti, wh_occ, (axle[0], s * w0, axle[1]), p['wheel_d'] / 2)
    if deck_occ:
        asm.rigid('Hip module to deck', hip_occ, deck_occ)
    return bodies


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
        design = adsk.fusion.Design.cast(app.activeProduct)
        design.designType = adsk.fusion.DesignTypes.ParametricDesignType
        out = build(design.rootComponent)
        f, asm = out['f'], out['asm']
        app.activeViewport.fit()
        c = f['cb']
        msg = (f'honebot v5 built: {len(out["bodies"])} bodies in {len(asm.occ)} components, '
               f'{len(asm.joints)} moving joints.\n\n'
               f'Deck {P["deck_length"]:.0f} x {P["deck_width"]:.0f} mm, bulkhead {f["tower_top"]:.1f} mm, '
               f'GoPro lens ~{f["lens_z"]:.0f} mm above the deck underside (pan servo: {P["pan_servo"]}).\n')
        if P['drivetrain'] in ('ostrich', 'both'):
            lo, hi = ride_height(f, P, P['leg_park_deg']), ride_height(f, P, P['leg_max_deg'])
            msg += (f'\nOstrich: deck {lo:.0f}-{hi:.0f} mm above the ground ({hi - lo:.0f} mm travel), '
                    f'legs {f["leg_len"]:.1f} mm, belt {P["belt_teeth"]}T GT2 {P["belt_w"]:.0f} mm. '
                    f'~{f["mass_total"] / 1000:.2f} kg.\n')
        if P['drivetrain'] in ('caster', 'both'):
            msg += (f'Caster base: wheelbase {c["wheelbase"]:.0f} mm, belt c-c {c["belt_cc"]:.1f} mm '
                    f'({P["cb_belt"][0]}T), centre of mass {c["cg_frac"] * 100:.0f}% of the wheelbase ahead of the axle, '
                    f'~{c["mass"] / 1000:.2f} kg.' + (' Drawn beside the robot.' if P['drivetrain'] == 'both' else '') + '\n')
        msg += ('\nTry the motion: Assemble > Drive Joints (or drag a thigh, wheel or the GoPro mount). '
                'The deck is grounded.\n')
        if P['drivetrain'] in ('ostrich', 'both'):
            if not P['shaft_support']:
                asm.notes.append('motor shaft support is off (shaft_support=False): the 20T pulley hangs off the motor bearings')
            elif not f['shaft_supported']:
                asm.notes.append(f'motor shaft support NOT cut: the shaft needs >= {f["shaft_need"]:.1f} mm past the bell '
                                 f'(motor_shaft_past_bell = {P["motor_shaft_past_bell"]:.1f}) and shaft_bearing must match '
                                 f'motor_shaft_d. Measure the motor, edit, re-run.')
            else:
                asm.notes.append(f'motor shaft support: {P["shaft_bearing"][0]:.0f}x{P["shaft_bearing"][1]:.0f}x'
                                 f'{P["shaft_bearing"][2]:.0f} bearing in each outrigger; needs >= {f["shaft_need"]:.1f} mm '
                                 'of shaft past the bell (MEASURE yours)')
        if asm.notes:
            msg += '\nNotes:\n- ' + '\n- '.join(asm.notes[:8]) + '\n'
        msg += ('\nPrint each body separately (right-click > Save As Mesh). "reference" bodies are bought parts. '
                'Docs: OSTRICH_LEGS.md, CASTER_BASE.md, INTERFACE.md.')
        ui.messageBox(msg)
    except Exception:
        if ui:
            ui.messageBox('honebot chassis script failed while building:\n  {}\n\n{}'.format(LAST_FEATURE[0], traceback.format_exc()))
