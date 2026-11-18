"""honebot handheld remote case: Fusion 360 script that builds the printed case plus reference hardware.

Run it in Fusion 360: Utilities > Add-Ins > Scripts and Add-Ins > "+" next to My Scripts > pick this folder
(cad/remote_case/fusion/HonebotRemote) > select HonebotRemote > Run. A new design document is created.

This one file is the ONLY geometry source. `cad/remote_case/check_remote.py` imports it without Fusion and runs the
fit / tolerance / joystick-travel / assembly checks on exactly these primitives; `cad/remote_case/mesh_remote.py`
rebuilds the same primitives with OpenCascade to export STL files and preview images.

What is inside (see docs/remote_case.md):
  ESP32 DevKit (pins down, jumper housings below it) in the rear half = the grip; its USB port opens in the rear end.
  TP4056 USB-C charger under the ESP32, its port also in the rear end, below the ESP32 port.
  1S LiPo under the joystick in the front half, with a printed joystick carrier over it.
  Mini rocker switch (KCD11 / KCD117S class, snap-in) recessed flush in the bottom.
  Pololu 5 V step-up (U1V11F5) in a pocket under the ESP32; its output feeds the DevKit's 5V/VIN pin.

Three printed parts: remote_base (tray), remote_lid (cap), js_carrier (joystick carrier / battery cover).
Lid -> base: 4 x M2 from underneath into heat-set inserts in the lid. Carrier -> base and joystick -> carrier:
M2 into heat-set inserts. No glue.

Conventions (millimetres): x = forward (rear end with the USB ports at x = 0, joystick end at x = L), y = left,
z = up (bottom face at z = 0). Thumb on the stick, palm around the rear half.

Data-quality tags in PARAMS comments:
  [V]  from a retrieved source (listed in docs/remote_case.md)
  [M]  MEASURE your part with calipers before printing (these vary between sellers)
  [D]  design choice; change it and re-run cad/remote_case/check_remote.py
"""

import math
import traceback

# ----------------------------------------------------------------------------------------------------------
# Parameters
# ----------------------------------------------------------------------------------------------------------
PARAMS = {
    # --- fit / tolerance policy (FDM on a Bambu H2C, PETG or PLA) ---------------------------------------
    "clr_pcb": 0.3,        # [D] per side: a PCB in its guides / pocket
    "clr_part": 0.5,       # [D] per side: battery, switch body, modules without exact outlines
    "clr_move": 1.0,       # [D] joystick moving parts (stick, cap) to anything, at every tilt
    "gap_hold": 0.3,       # [D] hold-down rib above a board (no preload: nothing gets crushed)
    "port_clr": 0.75,      # [D] per side around a USB plug's overmould in the end-wall openings
    "lip_side": 0.15,      # [D] lap joint between lid and base: clearance per side
    "lip_vert": 0.3,       # [D] lap joint: clearance above the tongue
    "lip_h": 2.0,          # [D] tongue height
    "lip_in": 0.6,         # [D] tongue stands this far inside the wall (tongue = wall/2 - lip_side + lip_in thick)
    # --- M2 hardware (owner has M2 screws + M2 heat-set inserts; no bought standoffs) -------------------------
    "clr_m2": 2.4,         # [D] M2 through-hole
    "ins_m2": 3.6,         # [D] heat-set insert hole (same size the chassis uses; fits the owner's M2 inserts)
    "ins_m2_len": 4.0,     # [M] insert length (common: 3.0 or 4.0)
    "m2_head_d": 3.8,      # [M] M2 head diameter (socket head 3.8, pan head 4.0)
    "m2_head_h": 2.0,      # [M] head height
    "case_screw_len": 8.0,     # [M] lid screws, from underneath (M2 x 8)
    "carrier_screw_len": 6.0,  # [M] carrier -> base posts (M2 x 6)
    "js_screw_len": 6.0,       # [M] joystick module -> carrier (M2 x 6)
    # --- shell -------------------------------------------------------------------------------------------
    "W": 50.0,             # [D] minimum width; grows by itself when the battery or the stick funnel needs more
    "R_plan": 14.0,        # [D] plan-view corner radius
    "wall": 2.4, "floor": 2.4, "top": 2.4,   # [D] 6 perimeters at 0.4 mm
    "c_top": 4.0,          # [D] 45-degree bevel around the top (lid prints top face down: a bevel needs no support)
    "c_bot": 2.0,          # [D] 45-degree bevel around the bottom (bed side of the base)
    "H": None,             # [D] total height; None = smallest that fits (rounded up to 0.5)
    "L": None,             # [D] total length; None = smallest that fits
    # --- ESP32 DevKit (default: 38-pin WROOM-32 DevKit, micro-USB) ------------------------------------------
    "esp_l": 55.0, "esp_w": 28.5, "esp_pcb_t": 1.6,  # [M] board outline (30-pin DOIT: ~51.5 x 28.5)
    "esp_pins_side": 19,   # [M] pins per side (38-pin: 19, 30-pin: 15), 2.54 mm pitch
    "esp_pin_in": 1.3,     # [M] pin-row centre from the long edge
    "esp_mod_l": 25.5, "esp_mod_w": 18.0, "esp_mod_h": 3.3,   # [M] WROOM module (flush with the antenna end)
    "esp_usb_w": 8.0, "esp_usb_h": 3.0, "esp_usb_l": 5.5,     # [M] USB receptacle on the DevKit (USB-C: 9.0 x 3.3 x 7.4)
    "esp_usb_over": 1.0,   # [M] receptacle sticks out past the PCB edge by this much
    "esp_plug_w": 11.0, "esp_plug_h": 8.0,   # [M] overmould of YOUR flashing cable's plug (it must enter the opening)
    "plug_r": 1.5,         # [M] overmould corner radius (both cables)
    "esp_comp_h": 3.5,     # [M] tallest other part on top (EN/BOOT buttons, regulator)
    "esp_led_dx": 8.0, "esp_led_dy": -6.0,   # [M] blue LED (GPIO 2) from the USB-end edge / from the board centre line
    "esp_under": 19.5,     # [D] free depth under the PCB: header plastic 2.5 + female jumper housing 14 + wire bend 3
    # --- 5-pin joystick module (KY-023 class) --------------------------------------------------------------------
    "js_l": 34.0, "js_w": 26.0, "js_pcb_t": 1.6,   # [V] 34 x 26 (several listings) [M]
    "js_hole_dx": 27.0, "js_hole_dy": 20.0,        # [V one listing: 27 / 20] [M] hole spacing
    "js_hole_d": 3.2,      # [M]
    "js_stick_dx": 17.0,   # [M] stick axis from the header-side edge (along the board's long side)
    "js_frame_l": 18.0, "js_frame_w": 18.0, "js_frame_h": 11.0,   # [M] metal frame + pots footprint, height above PCB
    "js_under": 3.0,       # [M] solder legs below the PCB
    "js_pivot_h": 8.5,     # [M] tilt pivot above the PCB top (about the middle of the round dome)
    "js_dome_r": 7.0,      # [M] round dome that turns with the stick
    "js_shaft_d": 5.0,     # [M]
    "js_cap_d": 24.0, "js_cap_z0": 18.0, "js_cap_h": 10.0,   # [M] cap diameter, cap underside / height above PCB top
    "js_tilt": 25.0,       # [M] max tilt, degrees (push the stick into a corner and measure)
    "js_press": 0.6,       # [M] click travel (the stick moves down)
    "js_hdr_len": 8.0,     # [M] right-angle header pins beyond the PCB edge (rigid)
    "js_plug_len": 14.0,   # [D] jumper housings + bend beyond the pins (flexible)
    "js_hdr_w": 13.0, "js_hdr_h": 2.6,
    # --- battery: 1S LiPo pouch 803040 (1000 mAh) with protection PCB and JST-PH lead ---------------------------
    "bat_l": 40.0, "bat_w": 30.0, "bat_h": 8.0,   # [M] incl. the protection PCB fold
    "bat_swell": 0.8,      # [D] room above the pouch (LiPo swells ~10 % with age)
    "bat_wall_h": 4.0,     # [D] pocket walls
    "ant_gap": 8.0,        # [D] ESP32 antenna end -> battery (a metal-foil pouch next to the antenna costs range)
    # --- TP4056 USB-C charger with protection (DW01A + 8205A) --------------------------------------------
    "tp_l": 28.0, "tp_w": 17.2, "tp_pcb_t": 1.6,   # [V 28 x 17 .. 29 x 17.3] [M]
    "tp_usb_w": 9.0, "tp_usb_h": 3.3, "tp_usb_l": 7.4, "tp_usb_over": 0.6,   # [M]
    "tp_plug_w": 12.5, "tp_plug_h": 7.0,   # [M] overmould of YOUR charging cable
    "tp_comp_h": 3.5, "tp_lift": 1.5,      # [D] board sits on two rails this high (parts on its underside)
    # --- Pololu U1V11F5 5 V step-up ------------------------------------------------------------------------------
    "bst_l": 11.4, "bst_w": 10.2, "bst_h": 4.5,   # [V 0.45" x 0.4"] height incl. inductor and wire stubs
    # --- mini rocker switch, snap-in, 15 x 10 mm bezel (KCD11 / KCD117S class) ---------------------------------
    "sw_cut_l": 13.5, "sw_cut_w": 9.0,     # [V one listing: 13.5 x 9] [M] check YOUR switch's drawing
    "sw_bez_l": 15.0, "sw_bez_w": 10.3, "sw_bez_h": 2.0,   # [V 14.8 x 10.3] [M] bezel
    "sw_depth": 14.5,      # [M] body + terminals behind the panel (overall 16.5 incl. bezel)
    "sw_panel": 1.6,       # [D] panel thickness at the cut-out (snap clips grip ~1-2 mm)
    "sw_body_l": 13.0, "sw_body_w": 8.6,
    # --- printed joystick carrier ---------------------------------------------------------------------------------
    "car_t": 2.4,          # [D] plate
    "car_spacer_d": 6.0,   # [D] spacers under the joystick PCB (insert from the top)
    "car_post_d": 7.0,     # [D] posts on the base floor
    "car_post_y": 19.0,    # [D]
    "col_d": 7.0,          # [D] lid-screw columns in the base
    "col_y": 19.0,         # [D]
    "boss_d": 6.0,         # [D] lid bosses (inserts)
    # --- LED window: a 1.75 mm filament stub (clear / natural PETG or PLA) pushed in works as a light pipe ----------
    "led_hole_d": 2.0, "led_tube_od": 4.4,
}

COL = {
    "case": (235, 235, 238), "lid": (255, 122, 26), "carrier": (20, 190, 180), "pcb_blue": (30, 70, 170),
    "pcb_green": (30, 120, 60), "metal": (170, 170, 175), "dark": (40, 40, 44), "battery": (230, 200, 40),
    "keepout": (255, 40, 40), "red": (200, 40, 40),
}

X, Y, Z = (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)
S2 = math.sqrt(0.5)


# ----------------------------------------------------------------------------------------------------------
# Primitive helpers (pure python; shared by the Fusion builder, the checker and the mesher)
#   box: oriented box {"t":"box","c":centre,"s":(len_u,len_v,len_w),"u":dir,"v":dir}  (w = u x v)
#   cyl: cylinder or cone {"t":"cyl","a":p0,"b":p1,"r":radius at a, "r2": radius at b (optional)}
#   sph: sphere; diff: a minus every primitive in b
# ----------------------------------------------------------------------------------------------------------
def box(x0, x1, y0, y1, z0, z1):
    return {"t": "box", "c": ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2),
            "s": (abs(x1 - x0), abs(y1 - y0), abs(z1 - z0)), "u": X, "v": Y}


def obox(c, lu, lv, lw, u, v):
    return {"t": "box", "c": tuple(c), "s": (lu, lv, lw), "u": tuple(u), "v": tuple(v)}


def cyl(a, b, r, r2=None):
    p = {"t": "cyl", "a": tuple(a), "b": tuple(b), "r": r}
    if r2 is not None and abs(r2 - r) > 1e-9:
        p["r2"] = r2
    return p


def sph(c, r):
    return {"t": "sph", "c": tuple(c), "r": r}


def pdiff(a, bs):
    return {"t": "diff", "a": a, "b": list(bs)}


def cyl_z(z0, z1, x, y, r, r2=None):
    return cyl((x, y, z0), (x, y, z1), r, r2)


def rbox(x0, x1, y0, y1, z0, z1, r):
    """Box with rounded vertical edges (plan-view corner radius r)."""
    r = min(r, (x1 - x0) / 2 - 0.01, (y1 - y0) / 2 - 0.01)
    if r < 0.05:
        return [box(x0, x1, y0, y1, z0, z1)]
    out = [box(x0 + r, x1 - r, y0, y1, z0, z1), box(x0, x1, y0 + r, y1 - r, z0, z1)]
    for cx in (x0 + r, x1 - r):
        for cy in (y0 + r, y1 - r):
            out.append(cyl_z(z0, z1, cx, cy, r))
    return out


def rbox_x(x0, x1, y0, y1, z0, z1, r):
    """Box with rounded edges parallel to x (end-view corner radius r): USB openings."""
    r = min(r, (y1 - y0) / 2 - 0.01, (z1 - z0) / 2 - 0.01)
    out = [box(x0, x1, y0 + r, y1 - r, z0, z1), box(x0, x1, y0, y1, z0 + r, z1 - r)]
    for cy in (y0 + r, y1 - r):
        for cz in (z0 + r, z1 - r):
            out.append(cyl((x0, cy, cz), (x1, cy, cz), r))
    return out


def ring(x0, x1, y0, y1, z0, z1, R, inset_out, inset_in):
    """Plan ring between the outline inset by inset_out and by inset_in (both measured from x0..y1, radius R)."""
    inner = rbox(x0 + inset_in, x1 - inset_in, y0 + inset_in, y1 - inset_in, z0 - 1, z1 + 1, R - inset_in)
    return [pdiff(p, inner) for p in rbox(x0 + inset_out, x1 - inset_out, y0 + inset_out, y1 - inset_out, z0, z1,
                                           R - inset_out)]


def bevel_band(x0, x1, y0, y1, za, zb, R, top):
    """45-degree bevel band of a plan-rounded slab: full outline at the wide end, inset by (zb - za) at the narrow
    end (top=True: narrow at zb). Built from two strips with wedge cuts and four corner cones."""
    c = zb - za
    if c < 0.05:
        return []
    out = []
    zedge = zb if top else za                     # height of the bevelled edge
    sz = 1.0 if top else -1.0
    a = c * math.sqrt(2.0)
    # strip along x (between the corner centres), wedges on the y0 / y1 edges
    if x1 - x0 - 2 * R > 0.05:
        b = box(x0 + R, x1 - R, y0, y1, za, zb)
        xm, lx = (x0 + x1) / 2, x1 - x0 - 2 * R + 2
        cuts = [obox((xm, y0, zedge), lx, a, a, X, (0.0, S2, sz * S2)),
                obox((xm, y1, zedge), lx, a, a, X, (0.0, S2, -sz * S2))]
        out.append(pdiff(b, cuts))
    if y1 - y0 - 2 * R > 0.05:
        b = box(x0, x1, y0 + R, y1 - R, za, zb)
        ym, ly = (y0 + y1) / 2, y1 - y0 - 2 * R + 2
        cuts = [obox((x0, ym, zedge), ly, a, a, Y, (S2, 0.0, -sz * S2)),
                obox((x1, ym, zedge), ly, a, a, Y, (S2, 0.0, sz * S2))]
        out.append(pdiff(b, cuts))
    for cx in (x0 + R, x1 - R):
        for cy in (y0 + R, y1 - R):
            if top:
                out.append(cyl_z(za, zb, cx, cy, R, R - c))
            else:
                out.append(cyl_z(za, zb, cx, cy, R - c, R))
    return out


def bevel_slab(x0, x1, y0, y1, z0, z1, R, cb, ct):
    """Plan-rounded slab (corner radius R) with 45-degree bevels cb (bottom) and ct (top)."""
    assert R > ct + 0.5 and R > cb + 0.5, "bevel larger than the corner radius"
    out = list(rbox(x0, x1, y0, y1, z0 + cb, z1 - ct, R))
    out += bevel_band(x0, x1, y0, y1, z1 - ct, z1, R, top=True)
    out += bevel_band(x0, x1, y0, y1, z0, z0 + cb, R, top=False)
    return out


# ----------------------------------------------------------------------------------------------------------
# Joystick motion (shared with the checker): sample points on the moving stick and cap
# ----------------------------------------------------------------------------------------------------------
def stick_samples(P, D, tilt_deg, az_deg, press=0.0, n_ring=24, grow=0.0):
    """Points on the surface of the moving shaft + cap at one tilt, world coordinates. grow inflates radii/ends."""
    t, a = math.radians(tilt_deg), math.radians(az_deg)
    ax = (math.sin(t) * math.cos(a), math.sin(t) * math.sin(a), math.cos(t))      # stick axis
    e1 = (math.cos(t) * math.cos(a), math.cos(t) * math.sin(a), -math.sin(t))     # perpendicular, in the tilt plane
    e2 = (-math.sin(a), math.cos(a), 0.0)
    piv = (D["stick_x"], 0.0, D["z_j1"] + P["js_pivot_h"] - press)
    pts = []

    def circle(h, r):
        for k in range(n_ring):
            ph = 2 * math.pi * k / n_ring
            cx, sx = math.cos(ph) * r, math.sin(ph) * r
            pts.append(tuple(piv[i] + ax[i] * h + e1[i] * cx + e2[i] * sx for i in range(3)))

    h_cap0 = P["js_cap_z0"] - P["js_pivot_h"] - grow
    h_cap1 = P["js_cap_z0"] + P["js_cap_h"] - P["js_pivot_h"] + grow
    rc, rs = P["js_cap_d"] / 2 + grow, P["js_shaft_d"] / 2 + grow
    for k in range(6):                                            # cap side wall and rims
        circle(h_cap0 + (h_cap1 - h_cap0) * k / 5, rc)
    for f in (0.0, 0.33, 0.66):                                   # cap underside
        circle(h_cap0, rc * f + rs * (1 - f))
    h0 = P["js_dome_r"] * 0.6
    for k in range(5):                                            # shaft between the dome and the cap
        circle(h0 + (h_cap0 - h0) * k / 4, rs)
    return pts


def joystick_opening(P, D):
    """Smallest lid opening (radius at the lid's inner face; it widens at 45 degrees to the top) that keeps clr_move
    to the moving stick + cap at every tilt and when clicked. The checker re-tests the result in 3-D."""
    z_li, H, clr = D["H"] - P["top"], D["H"], P["clr_move"]
    need = P["js_dome_r"] + clr
    for press in (0.0, P["js_press"]):
        for tilt in [P["js_tilt"] * k / 6 for k in range(7)]:
            for (x, y, z) in stick_samples(P, D, tilt, 0.0, press, n_ring=72):
                if z > H + clr or z < z_li - clr:
                    continue
                rho = math.hypot(x - D["stick_x"], y)
                # normal distance to a 45-degree cone is the horizontal gap / sqrt(2)
                need = max(need, rho + clr * math.sqrt(2.0) - min(max(z - z_li, 0.0), H - z_li))
    # dome (turns in place): at the lid's inner face it is narrower than its radius
    return math.ceil(need * 10) / 10


# ----------------------------------------------------------------------------------------------------------
# Layout: every position follows from PARAMS (the "stack-up"); the checker prints this table
# ----------------------------------------------------------------------------------------------------------
def corner_inset(P, y_abs, inset):
    """Extra x-distance needed at the rounded ends for something reaching to |y| = y_abs, measured from a surface
    that is `inset` inside the outline."""
    R = P["R_plan"] - inset
    yc = P["W"] / 2 - P["R_plan"]
    d = y_abs - yc
    if d <= 0:
        return 0.0
    if d >= R:
        return float("inf")
    return R - math.sqrt(R * R - d * d)


def layout(P):
    """Stack-up. Width grows past P["W"] when the battery + screw columns or the stick funnel's rim need it."""
    D = _layout(P)
    if D["W_need"] > P["W"] + 1e-9:
        D = _layout(dict(P, W=math.ceil(D["W_need"] * 2) / 2))
    return D


def _layout(P):
    D = {"W": P["W"]}
    t, fl, tp = P["wall"], P["floor"], P["top"]
    hw_in = P["W"] / 2 - t                      # cavity half-width
    D["hw_in"] = hw_in
    # ESP32: pins down, USB at the rear end
    D["x_face_e"] = t + P["lip_in"] + 0.3       # USB receptacle face (behind the tongue)
    D["x_e0"] = D["x_face_e"] + P["esp_usb_over"]
    D["x_e1"] = D["x_e0"] + P["esp_l"]
    # the joystick's jumper housings run under the ESP32's front end: keep the ESP32 above them
    js_jumper_top = fl + P["bat_h"] + P["bat_swell"] + 0.5 + P["car_t"] + P["js_under"] + 0.5 + P["js_pcb_t"] \
        + P["js_hdr_h"] + 0.5
    D["z_e0"] = max(fl + P["esp_under"], js_jumper_top + 0.5)
    D["z_e1"] = D["z_e0"] + P["esp_pcb_t"]
    D["esp_top"] = D["z_e1"] + max(P["esp_mod_h"], P["esp_usb_h"], P["esp_comp_h"])
    D["esp_end_margin"] = (P["esp_l"] - P["esp_pins_side"] * 2.54) / 2   # PCB end -> first jumper housing
    H_esp = D["esp_top"] + P["gap_hold"] + 1.0 + tp
    # the ESP32 USB opening may run at most 2 mm into the top bevel
    H_usb = D["z_e1"] + P["esp_usb_h"] / 2 + P["esp_plug_h"] / 2 + P["port_clr"] + P["c_top"] - 2.0
    # charger under the ESP32 (rear), switch, step-up (they set where the battery may start, too)
    D["x_face_t"] = t + 0.3
    D["x_t0"] = D["x_face_t"] + P["tp_usb_over"]
    D["x_t1"] = D["x_t0"] + P["tp_l"]
    D["x_sw"] = D["x_t1"] + P["clr_pcb"] + 1.2 + 2.0 + P["sw_bez_l"] / 2 + 1.5
    D["x_bst0"] = D["x_sw"] + P["sw_bez_l"] / 2 + 1.5
    D["x_bst1"] = D["x_bst0"] + P["bst_l"]
    # battery under the joystick, front half: away from the antenna and behind the step-up pocket
    D["x_b0"] = max(D["x_e1"] + P["ant_gap"], D["x_bst1"] + P["clr_pcb"] + 1.2 + P["clr_part"] + 0.3)
    D["x_b1"] = D["x_b0"] + P["bat_l"]
    D["z_b1"] = fl + P["bat_h"]
    # joystick carrier and module
    D["z_c0"] = D["z_b1"] + P["bat_swell"] + 0.5
    D["z_c1"] = D["z_c0"] + P["car_t"]
    D["z_j0"] = D["z_c1"] + P["js_under"] + 0.5
    D["z_j1"] = D["z_j0"] + P["js_pcb_t"]
    H_js = D["z_j1"] + P["js_frame_h"] + 0.5 + tp
    D["H"] = P["H"] if P["H"] else math.ceil(max(H_esp, H_js, H_usb) * 2) / 2
    D["H_need"] = max(H_esp, H_js, H_usb)
    D["x_j1"] = D["x_b1"]
    D["x_j0"] = D["x_j1"] - P["js_l"]
    D["stick_x"] = D["x_j0"] + P["js_stick_dx"]
    D["js_open_r"] = joystick_opening(P, D)
    # the funnel's top edge must stay 2 mm short of the front bevel: slide the joystick back over the battery
    # (at most to the battery's rear end), and lengthen the case only if that is not enough
    c_in_b = max(0.0, P["c_bot"] - fl * (2 - math.sqrt(2)))      # small inner bevel where floor meets wall
    L_bat = D["x_b1"] + P["clr_part"] + corner_inset(P, P["bat_w"] / 2 + P["clr_part"], t) + t + c_in_b
    stick_max = L_bat - P["c_top"] - 2.0 - D["js_open_r"] - tp
    # (not so far that the joystick's rigid header pins end up under the ESP32: the carrier must lift out)
    back = min(max(0.0, D["stick_x"] - stick_max), max(0.0, D["x_j0"] - (D["x_b0"] - 2.0)),
               max(0.0, D["x_j0"] - P["js_hdr_len"] - (D["x_e1"] + 0.5)))
    for k in ("x_j0", "x_j1", "stick_x"):
        D[k] -= back
    # length: the battery / joystick / carrier corners must clear the rounded front end
    y_reach = max(P["bat_w"] / 2 + P["clr_part"], P["js_w"] / 2 + 1.0 + P["clr_part"])
    L_need = max(D["x_b1"], D["x_j1"] + 0.3) + P["clr_part"] + corner_inset(P, y_reach, t) + t + c_in_b
    L_need = max(L_need, D["stick_x"] + D["js_open_r"] + tp + P["c_top"] + 2.0)
    D["L"] = P["L"] if P["L"] else math.ceil(L_need * 2) / 2
    D["L_need"] = L_need
    # split plane: the lap joint must lie in the vertical part of the wall, under the top bevel
    D["s"] = math.floor((D["H"] - P["c_top"] - P["lip_h"] - P["lip_vert"] - 0.5) * 2) / 2
    D["z_t0"] = fl + P["tp_lift"]
    D["z_t1"] = D["z_t0"] + P["tp_pcb_t"]
    D["z_panel"] = P["sw_bez_h"] + P["sw_panel"]          # top of the panel around the switch cut-out
    # port centres
    D["z_usb_e"] = D["z_e1"] + P["esp_usb_h"] / 2
    D["z_usb_t"] = D["z_t1"] + P["tp_usb_h"] / 2
    # screws: rear pair at the end of the straight sides, front pair likewise
    D["col_x"] = (P["R_plan"], D["L"] - P["R_plan"])
    D["engage_case"] = min(P["ins_m2_len"] - 0.5, P["case_screw_len"] - 4.5)
    D["case_thru"] = P["case_screw_len"] - D["engage_case"]          # unthreaded length in the base column
    # carrier posts: between the two front columns' keep-out and the battery's rear end
    front_col = D["col_x"][1] - P["col_d"] / 2 - 0.6 - 4.0
    D["car_post_x"] = (D["x_j0"] + 4.0, min(front_col, D["x_j0"] + 4.0 + 16.0))
    # joystick holes (module centred on y = 0; holes symmetric about the PCB centre)
    xm = (D["x_j0"] + D["x_j1"]) / 2
    D["js_holes"] = [(xm + sx * P["js_hole_dx"] / 2, sy * P["js_hole_dy"] / 2) for sx in (-1, 1) for sy in (-1, 1)]
    # screw columns and carrier posts beside the battery; width for them (lid insert inside the lap joint) and for a
    # 2 mm flat rim beside the stick funnel
    side = P["bat_w"] / 2 + P["clr_part"]
    D["col_y"] = max(P["col_y"], side + P["col_d"] / 2)
    D["car_post_y"] = max(P["car_post_y"], side + P["car_post_d"] / 2)
    w_cols = 2 * (max(D["col_y"], D["car_post_y"]) + P["ins_m2"] / 2 + 0.6 + t + P["lip_in"] + P["lip_side"])
    w_rim = 2 * (D["js_open_r"] + tp + P["c_top"] + 2.0)
    D["W_need"] = max(w_cols, w_rim)
    return D


# ----------------------------------------------------------------------------------------------------------
# Model
# ----------------------------------------------------------------------------------------------------------
class Model:
    def __init__(self):
        self.parts = []      # dicts: name, group, color, adds, cuts, pre, mount, assembly, flexible
        self.fits = []       # (name, prims, why): space that must stay free of printed material (clearance policy)
        self.screws = []     # numeric screw records
        self.info = {}

    def part(self, name, group, color, adds, cuts=None, pre=None, mount="base", assembly=None, flexible=False,
             hidden=False, material=None, orient=None, note=""):
        p = {"name": name, "group": group, "color": color, "adds": list(adds), "cuts": list(cuts or []),
             "pre": [(list(a), list(c)) for a, c in (pre or [])], "mount": mount, "assembly": assembly or name,
             "flexible": flexible, "hidden": hidden, "material": material, "orient": orient, "note": note}
        self.parts.append(p)
        return p

    def get(self, name):
        for p in self.parts:
            if p["name"] == name:
                return p
        raise KeyError(name)


def build_model(P=None):
    if P is None:
        P = dict(PARAMS)
    D = layout(P)
    P = dict(P, W=D["W"])          # width actually used (may have grown)
    M = Model()
    M.info = D
    t, fl, tp, W, R = P["wall"], P["floor"], P["top"], P["W"], P["R_plan"]
    L, H, s = D["L"], D["H"], D["s"]
    y0, y1 = -W / 2, W / 2
    hw_in = D["hw_in"]

    # ---- shell -------------------------------------------------------------------------------------------
    c_in_t = P["c_top"] - t * (2 - math.sqrt(2))       # inner bevels (offset of a 45-degree face by the wall)
    c_in_b = max(0.0, P["c_bot"] - fl * (2 - math.sqrt(2)))
    outer = bevel_slab(0, L, y0, y1, 0, H, R, P["c_bot"], P["c_top"])
    cavity = bevel_slab(t, L - t, y0 + t, y1 - t, fl, H - tp, R - t, c_in_b, c_in_t)
    D["c_in_t"], D["c_in_b"] = c_in_t, c_in_b

    def tunnel(xa, xb, zc, w, h, r=P["plug_r"] + P["port_clr"]):
        return rbox_x(xa, xb, -w / 2, w / 2, zc - h / 2, zc + h / 2, r)

    pc = P["port_clr"]
    tun_e = tunnel(-1.0, D["x_face_e"] + 0.01, D["z_usb_e"], P["esp_plug_w"] + 2 * pc, P["esp_plug_h"] + 2 * pc)
    tun_t = tunnel(-1.0, D["x_face_t"] + 0.01, D["z_usb_t"], P["tp_plug_w"] + 2 * pc, P["tp_plug_h"] + 2 * pc)
    D["tun_e"] = (D["z_usb_e"] - P["esp_plug_h"] / 2 - pc, D["z_usb_e"] + P["esp_plug_h"] / 2 + pc)
    D["tun_t"] = (D["z_usb_t"] - P["tp_plug_h"] / 2 - pc, D["z_usb_t"] + P["tp_plug_h"] / 2 + pc)

    # ---- base -----------------------------------------------------------------------------------------------
    b_adds, b_cuts = [], []
    # tongue of the lap joint (inner part of the wall, standing above the split plane)
    b_adds += ring(0, L, y0, y1, s - 0.5, s + P["lip_h"], R, t / 2 + P["lip_side"], t + P["lip_in"])
    # lid-screw columns (merge into the walls), counterbored from underneath
    cr = P["col_d"] / 2
    for cx in D["col_x"]:
        for sy in (-1, 1):
            cy = sy * D["col_y"]
            b_adds.append(cyl_z(0.6, s, cx, cy, cr))
            b_cuts.append(cyl_z(-1.0, s - D["case_thru"], cx, cy, (P["m2_head_d"] + 0.8) / 2))
            b_cuts.append(cyl_z(-1.0, s + 1.0, cx, cy, P["clr_m2"] / 2))
            M.screws.append({"id": "case %+.0f/%+.0f" % (cx, cy), "len": P["case_screw_len"], "engage": D["engage_case"],
                             "hole_depth": P["ins_m2_len"] + 1.0, "thru": D["case_thru"]})
    # ESP32 cradle: four corner blocks (side guide + end stop + ledge tab), tied to the side walls
    hw, cp = P["esp_w"] / 2, P["clr_pcb"]
    tab = max(0.8, min(2.5, D["esp_end_margin"] - 0.5))
    D["esp_tab"] = tab
    for xe, sx in ((D["x_e0"], -1), (D["x_e1"], 1)):
        for sy in (-1, 1):
            def yy(a, b):
                return (min(sy * a, sy * b), max(sy * a, sy * b))
            ya, yb = yy(hw + cp, hw + cp + 1.6)
            xa, xb = sorted((xe - sx * 6.0, xe + sx * (cp + 1.6)))
            b_adds.append(box(xa, xb, ya, yb, fl - 0.5, D["z_e1"]))                       # side guide
            ya, yb = yy(hw - 3.0, hw + cp + 1.6)
            xa, xb = sorted((xe + sx * cp, xe + sx * (cp + 1.6)))
            b_adds.append(box(xa, xb, ya, yb, fl - 0.5, D["z_e1"]))                       # end stop
            ya, yb = yy(hw - 3.0, hw + cp + 0.1)
            xa, xb = sorted((xe, xe - sx * tab))
            b_adds.append(box(xa, xb, ya, yb, fl - 0.5, D["z_e0"]))                       # ledge under the PCB corner
            ya, yb = yy(hw + cp + 1.0, hw_in + 0.6)
            xa, xb = sorted((xe + sx * cp, xe + sx * (cp + 1.6)))
            b_adds.append(box(xa, xb, ya, yb, fl - 0.5, D["z_e1"] - 3.0))                 # rib to the side wall
    # TP4056 pocket: two rails under the long edges, walls with snap lips (0.4 mm over the PCB edge)
    tw = P["tp_w"] / 2
    xt0, xt1 = D["x_t0"], D["x_t1"]
    for sy in (-1, 1):
        b_adds.append(box(xt0, xt1, *sorted((sy * (tw - 1.5), sy * (tw + cp))), fl - 0.5, D["z_t0"]))
        wy = sorted((sy * (tw + cp), sy * (tw + cp + 1.2)))
        b_adds.append(box(xt0 - 1.0, xt1 + cp + 1.2, *wy, fl - 0.5, D["z_t1"] + 1.2))
        ly = sorted((sy * (tw - 0.4), sy * (tw + cp + 0.1)))
        b_adds.append(box(xt0 + 4.0, xt1 - 4.0, *ly, D["z_t1"] + 0.15, D["z_t1"] + 1.2))   # snap lip
    b_adds.append(box(xt1 + cp, xt1 + cp + 1.2, -(tw + cp + 1.2), tw + cp + 1.2, fl - 0.5, D["z_t1"] + 0.8))
    # switch: flush recess for the bezel, thicker pad inside, snap-in cut-out
    xs = D["x_sw"]
    pad = [P["sw_bez_l"] / 2 + 2.5, P["sw_bez_w"] / 2 + 2.5]
    b_adds.append(box(xs - pad[0], xs + pad[0], -pad[1], pad[1], fl - 0.5, D["z_panel"]))
    b_cuts.append(box(xs - P["sw_bez_l"] / 2 - cp, xs + P["sw_bez_l"] / 2 + cp, -P["sw_bez_w"] / 2 - cp,
                      P["sw_bez_w"] / 2 + cp, -1.0, P["sw_bez_h"]))
    b_cuts.append(box(xs - P["sw_cut_l"] / 2, xs + P["sw_cut_l"] / 2, -P["sw_cut_w"] / 2, P["sw_cut_w"] / 2,
                      -1.0, D["z_panel"] + 1.0))
    # step-up pocket (open top, snap lips on the long sides)
    bw = P["bst_w"] / 2
    xb0, xb1 = D["x_bst0"], D["x_bst1"]
    b_adds.append(box(xb0 - 1.2 - cp, xb1 + cp + 1.2, -(bw + cp + 1.2), bw + cp + 1.2, fl - 0.5, fl + 1.0))
    b_cuts.append(box(xb0 - cp, xb1 + cp, -(bw + cp), bw + cp, fl + 0.6, fl + 5.0))
    zbt = fl + 0.6 + 1.6                       # step-up PCB top
    for sy in (-1, 1):
        b_adds.append(box(xb0 - 1.2 - cp, xb1 + cp + 1.2, *sorted((sy * (bw + cp), sy * (bw + cp + 1.2))),
                          fl - 0.5, zbt + 1.2))
        b_adds.append(box(xb0 + 2.0, xb1 - 2.0, *sorted((sy * (bw - 0.4), sy * (bw + cp + 0.1))), zbt + 0.15,
                          zbt + 1.2))           # snap lip
    # battery pocket: side and rear walls (front: the case wall); gap in the rear wall for the lead
    bw2 = P["bat_w"] / 2 + P["clr_part"]
    xb_0 = D["x_b0"] - P["clr_part"]
    for sy in (-1, 1):
        b_adds.append(box(xb_0 - 1.2, D["x_b1"] - 6.0, *sorted((sy * bw2, sy * (bw2 + 1.2))), fl - 0.5,
                          fl + P["bat_wall_h"]))
        b_adds.append(box(xb_0 - 1.2, xb_0, *sorted((sy * 6.0, sy * (bw2 + 1.2))), fl - 0.5, fl + P["bat_wall_h"]))
    # carrier posts (heat-set insert from the top)
    for cx in D["car_post_x"]:
        for sy in (-1, 1):
            b_adds.append(cyl_z(fl - 0.5, D["z_c0"], cx, sy * D["car_post_y"], P["car_post_d"] / 2))
            b_cuts.append(cyl_z(D["z_c0"] - P["ins_m2_len"] - 1.0, D["z_c0"] + 1.0, cx, sy * D["car_post_y"],
                                P["ins_m2"] / 2))
    D["carrier_engage"] = min(P["ins_m2_len"] - 0.3, P["carrier_screw_len"] - P["car_t"])
    M.screws.append({"id": "carrier", "len": P["carrier_screw_len"], "engage": D["carrier_engage"],
                     "hole_depth": P["ins_m2_len"] + 1.0, "thru": P["car_t"]})
    b_cuts += tun_e + tun_t
    outside = pdiff(box(-10, L + 10, -W, W, -10, H + 10), outer)     # trims every add to the outer shell
    b_cuts.append(outside)
    M.info["outer"] = outer
    M.part("remote_base", "printed", COL["case"], b_adds, b_cuts,
           pre=[(outer, list(cavity) + [box(-5, L + 5, -W, W, s, H + 5)])],
           mount="base", material="PETG", orient=("z", 0),
           note="prints on its bottom face; no support (the switch recess roof is a short bridge)")

    # ---- lid -------------------------------------------------------------------------------------------------
    l_adds, l_cuts = [], []
    zli = H - tp
    for cx in D["col_x"]:
        for sy in (-1, 1):
            cy = sy * D["col_y"]
            l_adds.append(cyl_z(s, zli + 0.5, cx, cy, P["boss_d"] / 2))
            l_cuts.append(cyl_z(s - 1.0, s + P["ins_m2_len"] + 1.0, cx, cy, P["ins_m2"] / 2))
    # hold-down ribs: over the WROOM module shield and over the USB receptacle
    zr = D["z_e1"] + P["esp_mod_h"] + P["gap_hold"]
    l_adds.append(box(D["x_e1"] - P["esp_mod_l"] + 6.0, D["x_e1"] - 7.0, -3.0, 3.0, zr, zli + 0.5))
    zu = D["z_e1"] + P["esp_usb_h"] + P["gap_hold"]
    xu0 = D["x_face_e"] + 0.5
    l_adds.append(box(xu0, xu0 + max(2.0, P["esp_usb_l"] - 1.5), -2.5, 2.5, zu, zli + 0.5))
    # LED light pipe tube
    xl, yl = D["x_e0"] + P["esp_led_dx"], P["esp_led_dy"]
    zl = D["esp_top"] + P["gap_hold"]          # tube ends above the tallest parts; light shines up it
    l_adds.append(cyl_z(zl, zli + 0.5, xl, yl, P["led_tube_od"] / 2))
    l_cuts.append(cyl_z(zl - 1.0, H + 1.0, xl, yl, P["led_hole_d"] / 2))
    D["led"] = (xl, yl, zl)
    # joystick opening: cylinder through the lid's inner face, 45-degree cone up to the top face
    ro = D["js_open_r"]
    l_cuts.append(cyl_z(zli - 3.0, zli + 0.01, D["stick_x"], 0.0, ro))
    l_cuts.append(cyl_z(zli, H + 0.01, D["stick_x"], 0.0, ro, ro + tp + 0.01))
    l_cuts.append(cyl_z(H, H + 2.0, D["stick_x"], 0.0, ro + tp + 0.01))
    rebate = ring(0, L, y0, y1, s - 1.0, s + P["lip_h"] + P["lip_vert"], R, t / 2,
                  t + P["lip_in"] + P["lip_side"])
    l_cuts += tun_e + tun_t + rebate           # also notches the screw bosses where the tongue passes
    l_cuts.append(outside)
    M.part("remote_lid", "printed", COL["lid"], l_adds, l_cuts,
           pre=[(outer, list(cavity) + [box(-5, L + 5, -W, W, -5, s)] + rebate)],
           mount="lid", material="PETG", orient=("x", 180),
           note="prints top face down (bevel on the bed, bosses and ribs pointing up); no support")

    # ---- joystick carrier (also keeps the battery down and covers the joystick's solder legs) ---------------
    c_adds, c_cuts = [], []
    xc0, xc1 = D["x_j0"] - 1.0, D["x_j1"]
    hy = P["js_w"] / 2 + 1.0
    c_adds += rbox(xc0, xc1, -hy, hy, D["z_c0"], D["z_c1"], 3.0)
    for cx in D["car_post_x"]:
        for sy in (-1, 1):
            cy = sy * D["car_post_y"]
            c_adds += rbox(cx - 4.0, cx + 4.0, *sorted((sy * (hy - 1.0), cy + sy * 2.6)), D["z_c0"], D["z_c1"], 1.5)
            c_cuts.append(cyl_z(D["z_c0"] - 1.0, D["z_c1"] + 1.0, cx, cy, P["clr_m2"] / 2))
    for hx, hyy in D["js_holes"]:
        c_adds.append(cyl_z(D["z_c1"] - 0.5, D["z_j0"], hx, hyy, P["car_spacer_d"] / 2))
        c_cuts.append(cyl_z(D["z_j0"] - P["ins_m2_len"] - 1.0, D["z_j0"] + 1.0, hx, hyy, P["ins_m2"] / 2))
    D["js_engage"] = min(P["ins_m2_len"] - 0.3, P["js_screw_len"] - P["js_pcb_t"])
    M.screws.append({"id": "joystick", "len": P["js_screw_len"], "engage": D["js_engage"],
                     "hole_depth": P["ins_m2_len"] + 1.0, "thru": P["js_pcb_t"]})
    # wire slot at the rear (the five jumper wires pass down beside the battery lead)
    M.part("js_carrier", "printed", COL["carrier"], c_adds, c_cuts, mount="carrier", material="PETG",
           orient=("z", 0), note="prints flat, spacers up; no support")

    # ---- reference hardware -----------------------------------------------------------------------------------
    hwd = []
    # ESP32 DevKit
    ze0, ze1 = D["z_e0"], D["z_e1"]
    hwd.append(("esp_pcb", "esp32", COL["pcb_blue"], [box(D["x_e0"], D["x_e1"], -hw, hw, ze0, ze1)], "base", False))
    hwd.append(("esp_module", "esp32", COL["metal"],
                [box(D["x_e1"] - P["esp_mod_l"], D["x_e1"], -P["esp_mod_w"] / 2, P["esp_mod_w"] / 2, ze1,
                     ze1 + P["esp_mod_h"])], "base", False))
    hwd.append(("esp_usb", "esp32", COL["metal"],
                [box(D["x_face_e"], D["x_face_e"] + P["esp_usb_l"], -P["esp_usb_w"] / 2, P["esp_usb_w"] / 2, ze1,
                     ze1 + P["esp_usb_h"])], "base", False))
    hwd.append(("esp_parts", "esp32", COL["dark"],
                [box(D["x_e0"] + P["esp_usb_l"], D["x_e1"] - P["esp_mod_l"] - 1.0, -hw + 3.0, hw - 3.0, ze1,
                     ze1 + P["esp_comp_h"])], "base", False))
    span = P["esp_pins_side"] * 2.54
    xr0 = (D["x_e0"] + D["x_e1"]) / 2 - span / 2
    rows = []
    for sy in (-1, 1):
        yc = sy * (hw - P["esp_pin_in"])
        rows.append(box(xr0, xr0 + span, yc - 1.27, yc + 1.27, ze0 - 2.5, ze0))
    hwd.append(("esp_headers", "esp32", COL["dark"], rows, "base", False))
    housings = []
    for sy in (-1, 1):
        yc = sy * (hw - P["esp_pin_in"])
        housings.append(box(xr0, xr0 + span, yc - 1.3, yc + 1.3, ze0 - 2.5 - 14.0, ze0 - 2.5))
    hwd.append(("esp_jumpers", "esp_jumpers", COL["dark"], housings, "base", True))
    # joystick module (on the carrier)
    zj0, zj1 = D["z_j0"], D["z_j1"]
    xm = (D["x_j0"] + D["x_j1"]) / 2
    hwd.append(("js_pcb", "joystick", COL["pcb_green"],
                [box(D["x_j0"], D["x_j1"], -P["js_w"] / 2, P["js_w"] / 2, zj0, zj1)], "carrier", False))
    fx, fy = P["js_frame_l"] / 2, P["js_frame_w"] / 2
    sx_ = D["stick_x"]
    hwd.append(("js_frame", "joystick", COL["metal"],
                [box(sx_ - fx, sx_ + fx, -fy, fy, zj1, zj1 + P["js_frame_h"]),
                 box(sx_ - fx, sx_ + fx, -fy, fy, zj0 - P["js_under"], zj0)], "carrier", False))
    hwd.append(("js_dome", "joystick", COL["dark"], [sph((sx_, 0.0, zj1 + P["js_pivot_h"]), P["js_dome_r"])],
                "carrier", False))
    hwd.append(("js_stick", "joystick", COL["dark"],
                [cyl_z(zj1 + P["js_pivot_h"], zj1 + P["js_cap_z0"] + 0.5, sx_, 0.0, P["js_shaft_d"] / 2),
                 cyl_z(zj1 + P["js_cap_z0"], zj1 + P["js_cap_z0"] + P["js_cap_h"], sx_, 0.0, P["js_cap_d"] / 2)],
                "carrier", False))
    hwd.append(("js_header", "joystick", COL["dark"],
                [box(D["x_j0"] - P["js_hdr_len"], D["x_j0"], -P["js_hdr_w"] / 2, P["js_hdr_w"] / 2, zj1,
                     zj1 + P["js_hdr_h"])], "carrier", False))
    hwd.append(("js_jumpers", "js_jumpers", COL["dark"],
                [box(D["x_j0"] - P["js_hdr_len"] - P["js_plug_len"], D["x_j0"] - P["js_hdr_len"],
                     -P["js_hdr_w"] / 2, P["js_hdr_w"] / 2, zj1 - 0.5, zj1 + P["js_hdr_h"] + 0.5)], "carrier", True))
    # battery, charger, step-up, switch
    hwd.append(("battery", "battery", COL["battery"],
                rbox(D["x_b0"], D["x_b1"], -P["bat_w"] / 2, P["bat_w"] / 2, fl, D["z_b1"], 2.0), "base", False))
    hwd.append(("tp4056", "tp4056", COL["pcb_blue"],
                [box(xt0, xt1, -tw, tw, D["z_t0"], D["z_t1"]),
                 box(D["x_face_t"], D["x_face_t"] + P["tp_usb_l"], -P["tp_usb_w"] / 2, P["tp_usb_w"] / 2, D["z_t1"],
                     D["z_t1"] + P["tp_usb_h"]),
                 box(xt0 + P["tp_usb_l"], xt1 - 1.0, -tw + 2.0, tw - 2.0, D["z_t1"], D["z_t1"] + 1.5)],
                "base", False))
    hwd.append(("stepup", "stepup", COL["pcb_green"],
                [box(xb0, xb1, -bw, bw, fl + 0.6, zbt),
                 box(xb0 + 0.5, xb1 - 0.5, -bw + 1.0, bw - 1.0, zbt, fl + 0.6 + P["bst_h"])], "base", False))
    hwd.append(("switch", "switch", COL["dark"],
                [box(xs - P["sw_bez_l"] / 2, xs + P["sw_bez_l"] / 2, -P["sw_bez_w"] / 2, P["sw_bez_w"] / 2, 0.0,
                     P["sw_bez_h"]),
                 box(xs - P["sw_body_l"] / 2, xs + P["sw_body_l"] / 2, -P["sw_body_w"] / 2, P["sw_body_w"] / 2,
                     P["sw_bez_h"], P["sw_bez_h"] + P["sw_panel"] + P["sw_depth"])], "base", False))
    for name, asm, col, prims, mount, flex in hwd:
        M.part(name, "hardware", col, prims, mount=mount, assembly=asm, flexible=flex)

    # ---- keep-outs: plug overmoulds (inserted cables) ---------------------------------------------------------
    M.part("plug_esp", "keepout", COL["keepout"],
           rbox_x(-20.0, D["x_face_e"], -P["esp_plug_w"] / 2, P["esp_plug_w"] / 2, D["z_usb_e"] - P["esp_plug_h"] / 2,
                  D["z_usb_e"] + P["esp_plug_h"] / 2, P["plug_r"]), mount="world", assembly="plug_esp", hidden=True)
    M.part("plug_tp", "keepout", COL["keepout"],
           rbox_x(-20.0, D["x_face_t"], -P["tp_plug_w"] / 2, P["tp_plug_w"] / 2, D["z_usb_t"] - P["tp_plug_h"] / 2,
                  D["z_usb_t"] + P["tp_plug_h"] / 2, P["plug_r"]), mount="world", assembly="plug_tp", hidden=True)

    # ---- clearance policy: spaces that must be free of PRINTED material (the checker tests them) -----------------
    e = 0.05   # numerical margin: a fit passes when the gap is at least (policy - 0.05 mm)
    cpe, cq = cp - e, P["clr_part"] - e
    M.fits += [
        ("ESP32 PCB, %.2f mm per side" % cp, [box(D["x_e0"] - cpe, D["x_e1"] + cpe, -hw - cpe, hw + cpe, ze0 + 0.2, ze1)]),
        ("ESP32 module, hold-down gap %.2f mm" % P["gap_hold"],
         [box(D["x_e1"] - P["esp_mod_l"], D["x_e1"], -P["esp_mod_w"] / 2, P["esp_mod_w"] / 2, ze1,
              ze1 + P["esp_mod_h"] + P["gap_hold"] - e)]),
        ("ESP32 USB receptacle, hold-down gap %.2f mm" % P["gap_hold"],
         [box(D["x_face_e"], D["x_face_e"] + P["esp_usb_l"], -P["esp_usb_w"] / 2, P["esp_usb_w"] / 2, ze1,
              ze1 + P["esp_usb_h"] + P["gap_hold"] - e)]),
        ("ESP32 other parts (buttons, regulator, LED), %.2f mm" % P["gap_hold"],
         [box(D["x_e0"] + P["esp_usb_l"], D["x_e1"] - P["esp_mod_l"] - 1.0, -hw + 0.3, hw - 0.3, ze1,
              ze1 + P["esp_comp_h"] + P["gap_hold"] - e)]),
        ("ESP32 jumper housings (any pin)", housings),
        ("joystick PCB, 0.5 mm per side", [box(D["x_j0"] - 0.45, D["x_j1"] + 0.45, -P["js_w"] / 2 - 0.45,
                                               P["js_w"] / 2 + 0.45, zj0 + 0.01, zj1 + 0.2)]),
        ("joystick frame + pots, 0.5 mm", [box(sx_ - fx - 0.45, sx_ + fx + 0.45, -fy - 0.45, fy + 0.45, zj1,
                                               zj1 + P["js_frame_h"] + 0.45)]),
        ("joystick solder legs, 0.5 mm", [box(sx_ - fx - 0.45, sx_ + fx + 0.45, -fy - 0.45, fy + 0.45,
                                              zj0 - P["js_under"] - 0.45, zj0)]),
        ("joystick header + jumpers", [box(D["x_j0"] - P["js_hdr_len"] - P["js_plug_len"], D["x_j0"],
                                           -P["js_hdr_w"] / 2 - 0.45, P["js_hdr_w"] / 2 + 0.45, zj1 - 0.5,
                                           zj1 + P["js_hdr_h"] + 0.5)]),
        ("battery %.2f mm per side + swell %.1f mm" % (P["clr_part"], P["bat_swell"]),
         rbox(D["x_b0"] - cq, D["x_b1"] + cq, -P["bat_w"] / 2 - cq, P["bat_w"] / 2 + cq, fl + 0.01,
              D["z_b1"] + P["bat_swell"] - e, 2.0)),
        ("TP4056 PCB, %.2f mm per side" % cp, [box(xt0 - cpe, xt1 + cpe, -tw - cpe, tw + cpe, D["z_t0"] + 0.01, D["z_t1"])]),
        ("TP4056 parts on top", [box(xt0, xt1 - 0.5, -tw + 0.45, tw - 0.45, D["z_t1"] + 0.01, D["z_t1"] + P["tp_comp_h"])]),
        ("step-up PCB, %.2f mm per side" % cp, [box(xb0 - cpe, xb1 + cpe, -bw - cpe, bw + cpe, fl + 0.61, zbt)]),
        ("step-up parts on top", [box(xb0 + 0.5, xb1 - 0.5, -bw + 0.45, bw - 0.45, zbt + 0.01, fl + 0.6 + P["bst_h"])]),
        ("switch bezel in its recess, %.2f mm" % cp,
         [box(xs - P["sw_bez_l"] / 2 - cpe, xs + P["sw_bez_l"] / 2 + cpe, -P["sw_bez_w"] / 2 - cpe,
              P["sw_bez_w"] / 2 + cpe, -1.0, P["sw_bez_h"] - e)]),
        ("switch body + terminals, %.2f mm" % P["clr_part"],
         [box(xs - P["sw_body_l"] / 2 - cq, xs + P["sw_body_l"] / 2 + cq, -P["sw_body_w"] / 2 - cq,
              P["sw_body_w"] / 2 + cq, D["z_panel"] + e, D["z_panel"] + P["sw_depth"] + 2.0)]),
        ("ESP32 USB plug overmould, %.2f mm per side" % pc,
         rbox_x(-20.0, D["x_face_e"] - e, -P["esp_plug_w"] / 2 - pc + e, P["esp_plug_w"] / 2 + pc - e,
                D["z_usb_e"] - P["esp_plug_h"] / 2 - pc + e, D["z_usb_e"] + P["esp_plug_h"] / 2 + pc - e,
                P["plug_r"] + pc - e)),
        ("charger USB-C plug overmould, %.2f mm per side" % pc,
         rbox_x(-20.0, D["x_face_t"] - e, -P["tp_plug_w"] / 2 - pc + e, P["tp_plug_w"] / 2 + pc - e,
                D["z_usb_t"] - P["tp_plug_h"] / 2 - pc + e, D["z_usb_t"] + P["tp_plug_h"] / 2 + pc - e,
                P["plug_r"] + pc - e)),
    ]
    return M


# ----------------------------------------------------------------------------------------------------------
# Fusion 360 builder
# ----------------------------------------------------------------------------------------------------------
def run(context):
    import adsk.core
    import adsk.fusion
    app = adsk.core.Application.get()
    ui = app.userInterface
    failures = []
    try:
        app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
        design = adsk.fusion.Design.cast(app.activeProduct)
        try:
            design.designType = adsk.fusion.DesignTypes.DirectDesignType
        except Exception:
            pass
        direct = design.designType == adsk.fusion.DesignTypes.DirectDesignType
        root = design.rootComponent
        tbm = adsk.fusion.TemporaryBRepManager.get()
        model = build_model()
        S = 0.1  # mm -> cm
        bool_fail = []
        sphere_fail = []

        def P3(p):
            return adsk.core.Point3D.create(p[0] * S, p[1] * S, p[2] * S)

        def V3(v):
            n = math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)
            return adsk.core.Vector3D.create(v[0] / n, v[1] / n, v[2] / n)

        def boolop(a, b, kind):
            try:
                tbm.booleanOperation(a, b, kind)
            except Exception:
                bool_fail.append(1)

        def prim(p):
            if p["t"] == "sph":
                try:
                    return tbm.createSphere(P3(p["c"]), p["r"] * S)
                except Exception:
                    sphere_fail.append(1)
                    return None
            if p["t"] == "diff":
                a = prim(p["a"])
                for q in p["b"]:
                    bq = prim(q)
                    if a is not None and bq is not None:
                        boolop(a, bq, adsk.fusion.BooleanTypes.DifferenceBooleanType)
                return a
            if p["t"] == "box":
                u = p["u"]
                w = (u[1] * p["v"][2] - u[2] * p["v"][1], u[2] * p["v"][0] - u[0] * p["v"][2],
                     u[0] * p["v"][1] - u[1] * p["v"][0])
                v = (w[1] * u[2] - w[2] * u[1], w[2] * u[0] - w[0] * u[2], w[0] * u[1] - w[1] * u[0])
                obb = adsk.core.OrientedBoundingBox3D.create(P3(p["c"]), V3(u), V3(v),
                                                             p["s"][0] * S, p["s"][1] * S, p["s"][2] * S)
                return tbm.createBox(obb)
            return tbm.createCylinderOrCone(P3(p["a"]), p["r"] * S, P3(p["b"]), p.get("r2", p["r"]) * S)

        def merge(prims):
            body = None
            for q in prims:
                b = prim(q)
                if b is None:
                    continue
                if body is None:
                    body = b
                else:
                    boolop(body, b, adsk.fusion.BooleanTypes.UnionBooleanType)
            return body

        appearances = {}

        def appearance(name, rgb):
            if name in appearances:
                return appearances[name]
            a = None
            try:
                a = design.appearances.itemByName(name)
                if a is None:
                    base = None
                    for i in range(app.materialLibraries.count):
                        lib = app.materialLibraries.item(i)
                        if "Appearance" not in lib.name:
                            continue
                        for nm in ("Plastic - Matte (Yellow)", "Plastic - Glossy (Yellow)"):
                            base = lib.appearances.itemByName(nm)
                            if base:
                                break
                        if base:
                            break
                    if base is not None:
                        a = design.appearances.addByCopy(base, name)
                        for j in range(a.appearanceProperties.count):
                            cp = adsk.core.ColorProperty.cast(a.appearanceProperties.item(j))
                            if cp and cp.id in ("opaque_albedo", "generic_diffuse", "plastic_albedo", "color"):
                                cp.value = adsk.core.Color.create(int(rgb[0]), int(rgb[1]), int(rgb[2]), 0)
            except Exception:
                a = None
            appearances[name] = a
            return a

        groups = {}
        for gname, label in (("printed", "Printed parts"), ("hardware", "Reference hardware"),
                             ("keepout", "Keep-out volumes")):
            occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
            occ.component.name = label
            groups[gname] = occ.component
        for part in model.parts:
            try:
                occ = groups[part["group"]].occurrences.addNewComponent(adsk.core.Matrix3D.create())
                comp = occ.component
                comp.name = part["name"]
                body = None
                for st_adds, st_cuts in part.get("pre", []):
                    sb = merge(st_adds)
                    if st_cuts:
                        boolop(sb, merge(st_cuts), adsk.fusion.BooleanTypes.DifferenceBooleanType)
                    if body is None:
                        body = sb
                    else:
                        boolop(body, sb, adsk.fusion.BooleanTypes.UnionBooleanType)
                ab = merge(part["adds"]) if part["adds"] else None
                if ab is not None:
                    if body is None:
                        body = ab
                    else:
                        boolop(body, ab, adsk.fusion.BooleanTypes.UnionBooleanType)
                if part["cuts"]:
                    boolop(body, merge(part["cuts"]), adsk.fusion.BooleanTypes.DifferenceBooleanType)
                if direct:
                    b = comp.bRepBodies.add(body)
                else:
                    bf = comp.features.baseFeatures.add()
                    bf.startEdit()
                    b = comp.bRepBodies.add(body, bf)
                    bf.finishEdit()
                b.name = part["name"]
                ap = appearance("honebot %d-%d-%d" % part["color"], part["color"])
                if ap is not None:
                    b.appearance = ap
                if part["group"] == "keepout" or part.get("hidden"):
                    occ.isLightBulbOn = False
            except Exception:
                failures.append("%s: %s" % (part["name"], traceback.format_exc().splitlines()[-1]))
        try:
            app.activeViewport.fit()
        except Exception:
            pass
        msg = "honebot remote: built %d of %d parts." % (len(model.parts) - len(failures), len(model.parts))
        if failures:
            msg += "\n\nFailed parts:\n" + "\n".join(failures)
        if bool_fail:
            msg += "\n\n%d boolean operations failed and were skipped." % len(bool_fail)
        if sphere_fail:
            msg += "\n\n(Spheres unavailable: %d skipped.)" % len(sphere_fail)
        msg += "\n\nTo print: right-click a component under 'Printed parts' > Save As Mesh (3MF/STL)."
        ui.messageBox(msg, "honebot remote")
    except Exception:
        if ui:
            ui.messageBox("honebot remote script failed:\n%s" % traceback.format_exc(), "honebot remote")
