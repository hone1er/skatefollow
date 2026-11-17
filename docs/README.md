# docs index

## The robot being built now

| Read this | For |
|---|---|
| [`first_drive.md`](first_drive.md) | The staged plan from printed parts to the first filmed run, and the table of today's speed limits |
| [`chassis_cad.md`](chassis_cad.md) | The chassis being printed: 4-wheel skid steer. Checks, print settings, parts list, assembly |
| [`hardware_plan.md`](hardware_plan.md) | UWB, power and radio choices with prices |
| [`camera_plan.md`](camera_plan.md) | GoPro modes, pan servo wiring, detector model |

**The printed parts come from one script: `cad/fusion/HonebotChassis/HonebotChassis.py`** (message box:
"built 83 of 83 parts"). The other Fusion scripts in this repo build different robots.

## Other designs kept in the repo

| Path | What it is |
|---|---|
| [`robot_design.md`](robot_design.md) | The first chassis plan (two driven wheels + caster). Its power and ESC sections still apply; the layout does not |
| [`robot_design_v2.md`](robot_design_v2.md), `cad/honebot_v2/` | "V2": two driven wheels + a servo-steered third wheel, sensored motors, lidar, folding mast, side-follow shots. A different robot from the one being printed. Its filming requirements (§1) are worth keeping in view: the offset follow and the sight-limited speed in the current software come from them |
| `honebot_ws/` | An older copy of the workspace docs and an earlier chassis script (caster base, "ostrich legs"). `honebot_ws/CLAUDE.md` is an old handoff file: the current one is the `CLAUDE.md` at the repository root |
