# honebot hardware plan (researched 2026-11-13)

> **Superseded in part (2026-11-13, later the same day):** the owner chose a **custom 3D-printed
> differential-drive robot** over an RC car. Sections 1–2 (chassis, battery) are kept for reference only;
> the drivetrain, frame and battery plan is now in [`robot_design.md`](robot_design.md). Sections 3–7
> (UWB, camera, Pi power, RC/kill switch, misc) still apply.

Prices are US retail as found on the date above; most are from retailer listings or price
trackers, not official MSRPs, and several items showed "out of stock / backorder". Re-check
before ordering. Links go to the pages the prices were taken from.

## Summary of recommendations

| Need | Pick | Price | Why |
|---|---|---|---|
| Chassis | **Traxxas Slash 4x4 VXL** (RTR, brushless, TSM) | ~$400–430 | Same platform F1TENTH uses; flat deck, huge parts/upgrade supply, 4WD, stable. Spare parts everywhere. |
| Chassis (cheaper) | Arrma Senton 4x4 3S BLX | ~$330–340 | Same class, usually cheaper, but many listings discontinued/sold out. |
| Battery + charger | 3S 5000 mAh hard-case LiPo + balance charger | ~$75–90 | RTR cars ship **without** battery/charger. Traxxas needs the iD connector (or adapter). |
| UWB following | **Makerfabs MaUWB_AOA Gen2**: 2× anchor + 1 tag | 2×$39.80 + $29.80 = **$109.40** | Angle-of-arrival anchor gives range **and bearing** from one module; two for left/right redundancy. Open-source STM32 firmware, UART. |
| UWB (alt., 2-range trilateration) | Makerfabs ESP32 UWB DW3000 ×3 (2 anchors + 1 tag) | 3×$43.80 = $131.40 | What `honebot_uwb` is written for today (two ranges). Arduino-programmable. |
| UWB (alt., pro) | Qorvo DWM3001CDK ×3 | 3×~$30 | Reference-grade, but nRF52 firmware work, stock spotty. |
| Camera + gimbal | **DJI Osmo Pocket 3** (or Pocket 4 when US-available) | ~$430–550 | Integrated 3-axis gimbal with its own subject tracking. Removes gimbal control from the robot entirely. |
| Vision test camera | Logitech C920/C920s | ~$50–70 | For Phase 5 person detection on the Pi. |
| Pi power | Pololu D36V50F5 5 V 5.5 A regulator | $34.95 | Runs the Pi 5 from the car's 3S pack (input 5.5–50 V). |
| RC override | FlySky FS-i6X + iA6B/X6B receiver | ~$60–70 | Manual override + kill channel into the ESP32 (RC always wins). The Traxxas TQi radio can also be used if you tap its receiver. |
| Kill switch + UWB MCUs | 2× ESP32 DevKit (CP2102) | ~$16 for 3 | ESP-NOW handheld kill button; spare. |

Rough totals: **minimum** (Slash + battery/charger + AOA UWB + regulator + ESP32s) ≈ **$640**;
**with Pocket 3 and RC radio** ≈ **$1,170**.

## 1. Chassis

Requirement: car-like, 1/10, brushless, fast enough for 15–25 km/h with headroom, a flat area
to bolt a plate for the Pi/ESP32/UWB, and parts you can buy anywhere when you break it.

- **Traxxas Slash 4x4 VXL (68386-4 / 68286-4)** — ~$399.95 at
  [AMain](https://amainhobbies.com/traxxas-slash-vxl-1-10-rtr-4x4-brushless-short-course-truck-red-tra68386-4-red/p1577422),
  $429.95 at [Wrekd](https://wrekd.com/products/traxxas-slash-vxl-1-10-rtr-4x4-brushless-short-course-truck-red);
  an orange one was on sale at $349.95 at Dollar Hobbyz.
  "HD" ($399.95 sale / $499.95) and "Ultimate" ($449.95 sale / $529.95) variants at
  [Hobby Addicts](https://www.hobby-addicts.com/products/traxxas-slash-4x4-vxl-hd) add stronger driveshafts/shocks;
  not needed for this project. Official page: [traxxas.com](https://traxxas.com/slash-4x4-vxl-ultimate-68277-4).
  Several listings were out of stock/backorder.
  - Wheelbase ≈ 0.324 m, max steer ≈ 20–25°. Set `wheelbase` and `max_steer_angle` in the bridge
    from measurement. Top speed 60+ mph on 3S: keep `THROTTLE_LIMIT` low in firmware.
  - Why this one: the F1TENTH autonomous-racing platform is built on it, so mounts, sensor plates and
    Ackermann parameters are documented online.
- **Arrma Senton 4x4 3S BLX (ARA4303V3)** — $329.99 at
  [Excel RC](https://www.excelrc.com/products/senton-4x4-3s-blx-brushless-1-10th-4wd-sc-blue) and
  [AMain (discontinued online)](https://amainhobbies.com/arrma-senton-4x4-v3-3s-blx-1-10-rtr-brushless-short-course-truck-red-ara4303v3t2/p1250111),
  $339.99 at [Hobby Express](https://www.hobbyexpressinc.com/products/senton-4x4-3s-blx-rtr-red-sc).
  Equivalent truck, usually cheaper; stock looked thin.
- Avoid: toy-grade cars (no standard ESC/servo signals), 2WD buggies (narrow, tippy with a payload),
  1/8 scale (too fast/heavy for a first robot).

## 2. Battery and charger

RTR trucks ship without battery and charger.
- Traxxas 2832X 5000 mAh 3S 25C iD hard case — $79.95 (Limitless RC; official page
  [traxxas.com](https://traxxas.com/products/parts/batteries/idpowercellbatteries/lipo/2832X-5000mah-111v-3s-25c)).
  Traxxas 3S battery + iD charger "completer pack" — $89.99
  ([Adrenaline RC](https://adrenalinercracing.com/products/traxxas-3s-battery-charger-completer-pack-w-5000mah-11-1v-lipo-battery)); the
  easiest single purchase for a Slash.
- Generic: Gens Ace 3S 5000 mAh 50C EC5 — $74.99 ([Hobby Sports](https://hobby-sports.com/products/gens-ace-3s-lipo-battery-50c-w));
  EcoPower Basher 3S 60C — $76.99 (Sheldon's Hobbies).
  Check the connector (Traxxas iD vs EC5/XT90) and get two packs: the Pi + UWB + camera share the pack.
- Budget power for the computer stack: the Pi 5 (~5 W idle, 12 W+ busy) + ESP32s + UWB ≈ 15 W, i.e.
  ~1.3 A from 11.1 V. Negligible next to the motor; one 5000 mAh pack lasts a session.

## 3. UWB following

**Recommended: Makerfabs MaUWB_AOA Gen2** (angle of arrival).
- Anchor $39.80, in stock ([product page](https://www.makerfabs.com/mauwb-aoa-gen2-anchor.html));
  tag $29.80. "1 anchor + 1 tag" is the minimum system; the anchor has two antennas and reports the
  tag's direction from phase difference. Makerfabs sells an
  AOA Gen2 4WD auto-following robot platform ($199.90, listed on [makerfabs.com](https://www.makerfabs.com/)) using exactly this
  (two anchors for 360°, open-source STM32 firmware), which is strong evidence the modules do the job
  honebot needs. Gen2 is **not** compatible with Gen1 parts.
- Plan: one anchor facing forward on the robot gives (range, bearing) directly → `uwb_node` publishes
  `target` the same way; a second anchor (rear) only matters if the skater can be behind the robot.
  The 2-range trilateration in `honebot_uwb/localize.py` becomes a fallback.
- Unknowns to verify on arrival: angle accuracy, field of view, update rate, serial protocol (datasheet and
  demo code are on Makerfabs' GitHub).

**Alternative: Makerfabs ESP32 UWB DW3000** — $43.80 each, in stock ([product page](https://www.makerfabs.com/esp32-uwb-dw3000.html)),
also on [Tindie](https://www.tindie.com/products/makingfab/esp32-uwb-dw3000/). ESP32 + DW3000, Arduino-programmable,
channels 5/9. Needs 3 boards (2 anchors + 1 tag) = $131.40 and your own ranging firmware. This is what the
current `U,<r_left>,<r_right>` protocol was written for. Makerfabs also lists the
[MaUWB_DW3000 with STM32 AT command](https://makerfabs.com/mauwb-dw3000-with-stm32-at-command.html) module ($38.90) which does ranging
over AT commands with no firmware writing ([review](https://www.cnx-software.com/2024/04/16/mauwb_dw3000-with-stm32-at-command-review-arduino-uwb-range-precision-indoor-positioning/)).

**Alternative: Qorvo DWM3001CDK** — ~$29.50–35 each ([Octopart](https://octopart.com/part/qorvo/DWM3001CDK),
[Avnet, 0 stock](https://www.avnet.com/americas/product/qorvo/dwm3001cdk/evolve-66520078/),
[DigiKey](https://www.digikey.co.uk/en/products/detail/qorvo/DWM3001CDK/24367348)). The reference design, but nRF52 SDK work and spotty stock.

Range reality check: all of these are good for 30–100 m line of sight; a skater 3–10 m away is easy.
Update rates of 10–20 Hz are typical and fine for the follower (it already handles a stale target).

## 4. Camera and gimbal

**Recommended: DJI Osmo Pocket 3** — integrated 1-inch sensor camera on a 3-axis gimbal with its own
face/subject tracking (ActiveTrack). Mounted on the robot it keeps the skater framed by itself, so the
robot software only has to point the car roughly at the skater. Prices have fallen since the Pocket 4
announcement: Creator Combo ~$494–549 ([Slickdeals](https://slickdeals.net/f/19211899-439-dji-osmo-pocket-3-gimbal-mounted-4k-vlog-camera-w-wi-fi-bluetooth-at-amazon),
[camelcamelcamel history](https://camelcamelcamel.com/product/B0CG19FGQ5)); base unit ~$549 list, seen at $430–439
([Imaging Resource](https://www.imaging-resource.com/deals/dji-osmo-pocket-3-drops-to-record-low-price-ahead-of-pocket-4-rumors/)).
- **Osmo Pocket 4** launched 16 April 2026 (4K/240, £445 / €479 / ~$439–499) but was **not on sale in the US at launch**
  pending FCC authorisation ([PhotoWorkout](https://www.photoworkout.com/?p=219302),
  [Camera Jabber](https://camerajabber.com/photography-news/dji-osmo-pocket-4-price-specs-and-availability-announced/)). Check US availability before buying a Pocket 3.
- Vibration: an RC truck shakes hard. Mount the camera on a foam/silicone-damped plate; the gimbal handles
  the rest. Consider a "wireless mic"-style remote start/stop later.
- Alternative: GoPro/Insta360 on a separate 3-axis gimbal (Hohem iSteady Pro 4 class, ~$100). More parts,
  no built-in tracking → then Phase 5 vision must drive the gimbal.

**For Phase 5 development now:** Logitech C920/C920s USB webcam, ~$50–70
([Walmart](https://www.walmart.com/ip/19721137), [price tracker](https://aislegopher.com/p/logitech-hd-pro-c920-video-calling-and-recording-1080p-laptop-webcam/22962)).
Works with OpenCV on the Pi with no drivers. The Pi Camera Module 3 is cheaper but libcamera on Ubuntu
is extra work (see CLAUDE.md Phase 4).

## 5. Power for the Pi and electronics

- **Pololu D36V50F5** 5 V 5.5 A step-down, input 5.5–50 V, $34.95 ([Pololu](https://www.pololu.com/product/4091),
  [RobotShop $39.95](https://www.robotshop.com/products/5v-55a-step-down-voltage-regulator-d36v50f5)). Feed the Pi 5 via the GPIO 5V pins
  (bypasses USB-C PD negotiation; the Pi then assumes a 5 A supply only if you set `usb_max_current_enable=1` in config.txt) or via a
  USB-C pigtail. Do **not** run the Pi from the ESC's BEC: it is 6 V/3 A at best and sags on throttle.
- Fuse the 3S feed (5 A blade fuse) and add a master switch for the electronics separate from the ESC.

## 6. RC manual override and kill switch

- **FlySky FS-i6X + iA6B/X6B receiver** — ~$60–70 ([SuperDroid](https://www.superdroidrobots.com/shop/item.aspx/flysky-fs-i6x-2-4g-6-channel-transmitter-receiver/3192),
  [FlyingTech £62.50](https://www.flyingtech.co.uk/product/flysky-fs-i6x-10ch-transmitter-fs-x6b-2-4ghz-ibus-receiver/?qa=1)).
  The X6B/iA6B receivers output iBUS (one serial wire, all channels) which the ESP32 can read with a UART —
  simpler than PWM capture. Channels: throttle, steer, a 3-position **mode** switch (manual / auto / kill).
  The Slash's own TQi radio is a 2-channel pistol grip; usable for manual driving but has no spare switch
  for a kill channel, hence the FlySky.
- **ESP-NOW kill button**: 2× ESP32 DevKit (Hosyond 3-pack ~$15.99,
  [Amazon via tracker](https://www.slashskill.com/deals/product/B0C7C2HQ7P/hosyond-3pack-esp32-esp-wroom-32-development-board-wifi-bluetooth-cp2102-dual/)).
  One in a handheld box with a big button and a battery; the robot's ESP32 forces neutral when the button is
  pressed **or** when the heartbeat from the handheld stops. Belt and braces with the RC kill channel.

## 7. Mounting and misc (~$40)

- 3 mm aluminium or carbon plate (or 3D print) on the Slash body posts; M3 standoffs for the Pi.
- Short USB-A to micro-USB cables for ESP32/UWB; USB-C PD pigtail or GPIO power leads.
- Foam-damped camera mount; velcro battery strap; 5 A fuse holder; XT60/Traxxas Y-lead to split the pack.

## Suggested order of purchase

1. UWB AOA Gen2 anchor + tag ($69.60) — unblocks Phase 6 on the bench immediately, no chassis needed.
2. Pololu regulator + ESP32 3-pack (~$51) — bench the kill switch and power path.
3. Chassis + battery/charger (~$500) — Phase 2 on a stand, then field tests.
4. FlySky radio (~$65) — before the first outdoor run.
5. Webcam (~$60) for Phase 5 development; Osmo Pocket 3/4 (~$450–550) once following works.
