# honebot electronics plan: UWB, camera, power, kill switch (researched 2026-11-13)

The chassis, motors, ESCs and battery are in [`robot_design.md`](robot_design.md). This file covers the rest.
Prices are US retail as found on the date above (retailer listings, not MSRPs); re-check before ordering.

## Summary

| Need | Pick | Price | Why |
|---|---|---|---|
| Following sensor | **Makerfabs MaUWB_AOA Gen2**: 1 anchor + 1 tag | $39.80 + $29.80 = **$69.60** | Angle-of-arrival anchor reports range **and bearing** from one module over UART; in stock. A second anchor ($39.80) only if the skater can be behind the robot. |
| Camera | **Your GoPro** on a printed, foam-damped fixed mount | $0 | Its wide lens forgives aiming error while the robot keeps you roughly centred. No gimbal. |
| Vision dev camera (Phase 5, optional) | Logitech C920/C920s, or the GoPro in USB webcam mode | ~$50–70 / $0 | Only needed when person-detection work starts; the GoPro (Hero 8 and newer) can act as a USB webcam on Linux. |
| Pi power | 5 V 5 A buck, 9–36 V input | ~$15–20 | Runs the Pi 5 from the 3S pack via the GPIO 5 V pins. |
| Kill switch | 1 extra ESP32 DevKit + push button + small LiPo/USB bank in a printed handheld | ~$6–16 | ESP-NOW to the robot's ESP32; robot forces neutral on button press **or** loss of the handheld's heartbeat. |
| Manual driving | Any USB/Bluetooth gamepad you already own | $0 | `teleop_joy.launch.py` → safety gate. No RC radio needed. |

Total for this list: **≈ $90–110** (UWB + buck + ESP32), with the GoPro and gamepad already owned.

## 1. UWB following

**Recommended: Makerfabs MaUWB_AOA Gen2** (angle of arrival).
- Anchor $39.80, tag $29.80, in stock ([product page](https://www.makerfabs.com/mauwb-aoa-gen2-anchor.html)).
  The anchor has two antennas and reports the tag's range and direction from phase difference over UART.
  Makerfabs sells a $199.90 auto-following robot platform built on exactly this (two anchors for 360°,
  open-source STM32 firmware), good evidence it does what honebot needs. Gen2 is **not** compatible with Gen1.
- Plan: one anchor facing forward in the robot's nose → `uwb_node` reads `A,<range>,<bearing_deg>` lines and
  publishes `/target`. The two-range trilateration path (`U,` lines) in `honebot_uwb/localize.py` is the
  fallback for plain DW3000 boards.
- Verify on arrival: angle accuracy, field of view, update rate (10–20 Hz is fine), and the exact serial
  format (datasheet and demo code on Makerfabs' GitHub) → write the matching line printer on the board or a
  parser in `uwb_node`.

**Alternatives**
- Makerfabs ESP32 UWB DW3000, $43.80 each, in stock ([product page](https://www.makerfabs.com/esp32-uwb-dw3000.html),
  [Tindie](https://www.tindie.com/products/makingfab/esp32-uwb-dw3000/)): needs 3 boards (2 anchors + 1 tag =
  $131.40) and your own ranging firmware. More money, more work; only if AoA disappoints.
- Makerfabs [MaUWB_DW3000 with STM32 AT command](https://makerfabs.com/mauwb-dw3000-with-stm32-at-command.html)
  ($38.90): ranging over AT commands, no firmware writing ([review](https://www.cnx-software.com/2024/04/16/mauwb_dw3000-with-stm32-at-command-review-arduino-uwb-range-precision-indoor-positioning/)).
- Qorvo DWM3001CDK ~$30 each ([Octopart](https://octopart.com/part/qorvo/DWM3001CDK)): reference-grade but nRF52
  firmware work and spotty stock.

All of these range 30–100 m line of sight; a skater 3–10 m away is easy.

## 2. Camera

The GoPro goes on a printed mount with a foam or silicone pad between mount and frame (the robot vibrates;
GoPro stabilisation handles the rest). Mount it above the rear axle on a short mast that folds for the
backpack. Start/stop recording by hand or with the GoPro remote/app; the robot does not control the camera.

For Phase 5 (person detection to improve framing): the GoPro works as a USB webcam on Linux (Hero 8+), or a
Logitech C920 (~$50–70, [Walmart](https://www.walmart.com/ip/19721137)) plugs straight into OpenCV. Not needed
until following works.

## 3. Power for the Pi and electronics

- Everything runs from the one 3S LiPo (see robot_design.md). Budget: Pi 5 ~5 W idle / 12 W busy, ESP32s + UWB
  ~2 W, motors ~20–60 W → ~55 Wh pack ≈ 1 h of following.
- Pi 5: **5 V 5 A buck with ≥ 15 V input** (3S is 12.6 V full), e.g. [UCTRONICS 9–36 V → 5 V 5 A, 2-pack $39](https://www.newegg.com/p/0Y3-0680-00150)
  or any equivalent module. Feed the Pi through the GPIO 5 V pins (bypasses USB-C PD negotiation) and set
  `usb_max_current_enable=1` in `/boot/firmware/config.txt` so the Pi allows full USB current.
- ESP32: from the Pi's USB (as now). The ESCs' BEC output, if present, stays unused (AM32 drone ESCs have none).
- Fuse the pack lead (20 A blade fuse) and add a master switch for the electronics. Keep the LiPo low-voltage
  protection on: either the ESCs' cutoff or a $3 LiPo alarm on the balance lead.

## 4. Kill switch and manual override

- **ESP-NOW kill button**: one ESP32 DevKit in a printed handheld with a big button and a small battery
  (3-pack of ESP32s ~$16, [example listing](https://www.slashskill.com/deals/product/B0C7C2HQ7P/hosyond-3pack-esp32-esp-wroom-32-development-board-wifi-bluetooth-cp2102-dual/)).
  The robot's ESP32 forces neutral when the button is pressed **or** when the handheld's heartbeat stops
  (same pattern as the Pi heartbeat watchdog). To be added to the firmware in Phase 7.
- **Manual driving**: a gamepad on the Pi (`teleop_joy.launch.py` → `/cmd_vel_raw` → safety gate). Releasing
  the enable button stops the robot. No RC transmitter/receiver is planned.

## Suggested order of purchase

1. UWB AoA anchor + tag ($69.60) — unblocks Phase 6 on the bench; nothing else needed to start.
2. Buck converter + ESP32 pack (~$35) — Pi power path and the kill switch.
3. Drivetrain and battery per `robot_design.md`.
