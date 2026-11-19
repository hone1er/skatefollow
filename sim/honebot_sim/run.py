"""Command line: run scenarios headless (pass/fail table) or watch one in the 3-D viewer.

  python -m honebot_sim.run --list
  python -m honebot_sim.run walk_stop                    # one scenario, summary
  python -m honebot_sim.run --all                        # every scenario, table + exit code
  python -m honebot_sim.run --all --profile target       # what 12-15 mph needs (long-range sensor, limits raised)
  mjpython -m honebot_sim.run carve_6 --view             # 3-D viewer (macOS needs `mjpython`, Linux `python`)
  python -m honebot_sim.run cruise_stop_8 --csv run.csv  # time series for plotting
  python -m honebot_sim.run --spin-test                  # effective track width for the bridge (friction guess!)
  python -m honebot_sim.run --all --side --yaw-loop      # side view: src/honebot_bringup/config/side_shot.yaml

Profiles:
  today    robot.yaml and the firmware exactly as they are (HC-SR04, max_linear 3.0, THROTTLE_LIMIT 0.5)
  target   a 12 m forward lidar (TFmini-class) instead of the HC-SR04, sight_distance 12 m, max_linear 7.5 m/s,
           THROTTLE_LIMIT 1.0: the configuration the 12-15 mph goal needs
"""

import argparse
import csv
import math
import sys
import time

from .config import REPO, load_config, load_robot_yaml
from .sim import Sim
from .skater import MPH, SCENARIOS

PROFILES = {
    "today": dict(stack={}, firmware={}, sensor=None),
    # blind_timeout 0: the lidar is tilted up so it never sees flat ground; "no return" is its normal reading
    "target": dict(stack={"follower": {"max_linear": 7.5}, "safety_gate": {"sight_distance": 12.0, "blind_timeout": 0.0}},
                   firmware={"THROTTLE_LIMIT": 1.0}, sensor="tfmini"),
}


def evaluate(sc, mt, cfg):
    """(passed, [reasons]) for one run. Safety first; keeping up only where the scenario expects it."""
    fails = []
    if mt.skater_contact:
        fails.append("hit the skater")
    if mt.obstacle_contact:
        fails.append("hit an obstacle")
    if mt.tipped:
        fails.append("tipped over")
    if mt.min_gap < 0.3:
        fails.append("closer than 0.3 m (%.2f)" % mt.min_gap)
    if mt.final_speed > 0.2:
        fails.append("still moving at the end (%.1f m/s)" % mt.final_speed)
    if mt.max_current > cfg.motor.current_limit + 0.5:
        fails.append("motor current above the ESC limit")
    side = bool(cfg.overrides.get("side"))
    # side view: after a dead stop the robot ends up alongside, the skater out of the UWB cone; only judge
    # tracking and framing while the skater moves there
    lost, framing = (mt.lost_moving, mt.framing_moving) if side else (mt.lost_time, mt.framing)
    if sc.expect_keep_up and lost > 1.0:
        fails.append("lost the UWB target for %.1f s" % lost)
    if sc.expect_keep_up and framing < 0.85:
        fails.append("skater in frame only %.0f %% of the time" % (100 * framing))
    if side:
        f = load_side_shot()["follower"]
        dist, ang = mt.side()
        if mt.min_gap_moving < SIDE_MIN_GAP:
            fails.append("side view: drove within %.2f m of the skater" % mt.min_gap_moving)
        if sc.expect_keep_up and sc.side_check and mt.side_dist and not (abs(dist - f["side_distance"]) <= SIDE_DIST_TOL and
                                                        abs(ang - math.degrees(f["side_angle"])) <= SIDE_ANGLE_TOL):
            fails.append("side view held at %.1f m / %.0f deg" % (dist, ang))
    return not fails, fails


SIDE_SHOT_YAML = REPO / "src" / "honebot_bringup" / "config" / "side_shot.yaml"
# side view pass marks, fixed before tuning: median distance and angle while the skater moves, and how close the
# robot may drive (robot moving) to the skater's centre
SIDE_DIST_TOL = 0.6          # m either side of side_distance
SIDE_ANGLE_TOL = 12.0        # deg either side of side_angle
SIDE_MIN_GAP = 0.5           # m, robot footprint to skater centre while the robot moves


def load_side_shot():
    """The side-view preset the robot uses (film.launch.py side_shot:=true): {node: {param: value}}."""
    return load_robot_yaml(SIDE_SHOT_YAML)


def make(name, profile="today", seed=0, log=False, fw=None, ros=None, yaw_loop=False, side=False, **overrides):
    """fw: {CONST: value} firmware overrides; ros: {node: {param: value}} robot.yaml overrides;
    side: layer config/side_shot.yaml on top (before `ros`), like film.launch.py side_shot:=true."""
    p = PROFILES[profile]
    if p["sensor"]:
        overrides.setdefault("forward_sensor", p["sensor"])
    cfg = load_config(**overrides)
    sc = SCENARIOS[name]
    if profile == "target":
        sc = _expect_all(sc)
    stack = {k: dict(v) for k, v in p["stack"].items()}
    if side:
        cfg.overrides["side"] = True
        for node, params in load_side_shot().items():
            stack.setdefault(node, {}).update(params)
    for node, params in (ros or {}).items():
        stack.setdefault(node, {}).update(params)
    firmware = dict(p["firmware"]); firmware.update(fw or {})
    if yaw_loop:
        firmware["YAW_LOOP_ENABLED"] = True
    return Sim(cfg, sc, seed=seed, stack_overrides=stack, firmware_overrides=firmware, log=log), cfg, sc


def _value(text):
    if text.lower() in ("true", "false"):
        return text.lower() == "true"
    try:
        return float(text)
    except ValueError:
        return text


def parse_overrides(fw_list, set_list):
    fw = {}
    for item in fw_list or []:
        k, v = item.split("=", 1)
        fw[k.strip()] = _value(v.strip())
    ros = {}
    for item in set_list or []:
        k, v = item.split("=", 1)
        node, param = k.strip().split(".", 1)
        ros.setdefault(node, {})[param] = _value(v.strip())
    return fw, ros


def _expect_all(sc):
    import copy
    sc = copy.copy(sc)
    sc.expect_keep_up = True
    return sc


def summary_row(name, sc, mt, ok, why):
    states = sorted(mt.gate_time.items(), key=lambda kv: -kv[1])
    gate = ", ".join("%s %.0f%%" % (k, 100 * v / max(mt.duration, 1e-9)) for k, v in states[:3])
    side = ""
    if mt.side_dist:
        dist, ang = mt.side()
        side = "side %.1f m %+3.0f deg | " % (dist, ang)
    return ("%-15s %-4s min gap %4.2f m | max gap %5.1f m | top %4.1f m/s (%4.1f mph) | lost %4.1f s | framed %3.0f%% | "
            "%s%s%s" % (name, "PASS" if ok else "FAIL", mt.min_gap, mt.max_gap_moving, mt.max_speed, mt.max_speed / MPH,
                        mt.lost_time, 100 * mt.framing, side, gate, ("  <- " + "; ".join(why)) if why else ""))


def run_one(name, args):
    fw, ros = parse_overrides(args.fw, args.set)
    if args.yaw_loop:
        fw["YAW_LOOP_ENABLED"] = True
    sim, cfg, sc = make(name, args.profile, args.seed, log=bool(args.csv or args.plot), fw=fw, ros=ros, friction=args.friction,
                        gyro_fault=args.gyro_fault, side=args.side)
    if args.remote or args.keys:
        if not args.view:
            print("--remote and --keys drive the robot live: add --view")
            return False
        from .remote_link import KeyboardRemote, RemoteSerial
        sim.remote_input = RemoteSerial(args.remote) if args.remote else KeyboardRemote()
    if args.view:
        return view(sim, args)
    t0 = time.time()
    mt = sim.run()
    ok, why = evaluate(sc, mt, cfg)
    print(summary_row(name, sc, mt, ok, why) + "   (%.1fx real time)" % (sc.duration / max(time.time() - t0, 1e-6)))
    if args.csv:
        with open(args.csv, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(sim.log[0].keys()))
            w.writeheader()
            for r in sim.log:
                w.writerow({k: (float(v) if not isinstance(v, str) else v) for k, v in r.items()})
        print("time series: %s (%d rows, 20 Hz)" % (args.csv, len(sim.log)))
    if args.plot:
        plot(sim.log, args.plot, "%s (%s profile%s): %s" % (name, args.profile, ", yaw loop" if args.yaw_loop else "",
                                                          "PASS" if ok else "FAIL " + "; ".join(why)))
    return ok


def view(sim, args):
    import mujoco
    try:
        import mujoco.viewer
        keys = getattr(sim.remote_input, "key", None)
        handle = mujoco.viewer.launch_passive(sim.m, sim.d, key_callback=keys) if keys else \
            mujoco.viewer.launch_passive(sim.m, sim.d)
    except RuntimeError as e:          # macOS: the passive viewer must run under `mjpython`
        print("could not open the viewer: %s\nOn a Mac start it with:  mjpython -m honebot_sim.run %s --view"
              % (e, sim.sc.name))
        return False
    print("viewer: drag to orbit, right-drag to pan, scroll to zoom; Tab/Shift-Tab hide the side panels; the "
          "Rendering panel's camera list has 'chase' and 'gopro' (what the GoPro sees). Close the window to stop.")
    if keys:
        print("keyboard remote: R = RUN/STOP (starts in STOP), arrows = stick (steps of 0.2), Space = centre. "
              "Click the 3-D view first so it has the keyboard.")
    elif sim.remote_input is not None:
        print("real remote on %s: hold the stick button 1.5 s (stick centred) for RUN, any click = STOP"
              % sim.remote_input.ser.port)
    last_print = 0.0
    with handle as v:
        v.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        v.cam.trackbodyid = sim.chassis
        v.cam.distance, v.cam.elevation, v.cam.azimuth = 6.0, -25.0, 200.0
        start = time.time()
        while v.is_running() and sim.t < sim.sc.duration + 5.0:
            with v.lock():
                for _ in range(10):
                    sim.step()
            v.sync()
            if sim.remote_input is not None and sim.t - last_print > 1.0:
                last_print = sim.t
                st, fx, fy = sim.remote_input.poll()
                src = {0: "nobody", 1: "Pi (follower)", 2: "STICK"}.get(sim.esp.source, "?")
                print("\r t=%5.1f s  remote %-4s stick %+.2f %+.2f  driving: %-13s speed %.1f m/s  gate %s   "
                      % (sim.t, st.upper(), fx, fy, src, math.hypot(sim.d.qvel[0], sim.d.qvel[1]),
                         sim.pi.gate_state), end="", flush=True)
            lag = sim.t / args.speed - (time.time() - start)
            if lag > 0:
                time.sleep(lag)
    mt = sim.metrics
    ok, why = evaluate(sim.sc, mt, sim.cfg)
    print(summary_row(sim.sc.name, sim.sc, mt, ok, why))
    return ok


def plot(log, path, title):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    t = [r["t"] for r in log]
    fig, ax = plt.subplots(4, 1, figsize=(10, 10), sharex=True)
    ax[0].plot(t, [r["skater_speed"] for r in log], label="skater")
    ax[0].plot(t, [r["robot_speed"] for r in log], label="robot")
    ax[0].plot(t, [r["cmd_lin"] for r in log], ":", label="follower asks")
    ax[0].plot(t, [r["gate_lin"] for r in log], "--", label="gate allows")
    ax[0].set_ylabel("m/s"); ax[0].legend(loc="upper right"); ax[0].grid(alpha=.3)
    ax[1].plot(t, [r["gap"] for r in log], label="gap robot -> skater")
    ax[1].plot(t, [min(r["range"], 5.0) for r in log], label="forward sensor (capped at 5)")
    ax[1].set_ylabel("m"); ax[1].legend(loc="upper right"); ax[1].grid(alpha=.3)
    ax[2].plot(t, [r["cmd_ang"] for r in log], label="yaw rate asked (rad/s)")
    ax[2].plot(t, [r["pan"] for r in log], label="camera pan (rad)")
    ax[2].legend(loc="upper right"); ax[2].grid(alpha=.3)
    ax[3].plot(t, [r["i_l"] for r in log], label="left motor A")
    ax[3].plot(t, [r["i_r"] for r in log], label="right motor A")
    ax[3].plot(t, [10 * r["failsafe"] for r in log], label="ESP32 fail-safe (x10)")
    ax[3].set_xlabel("s"); ax[3].legend(loc="upper right"); ax[3].grid(alpha=.3)
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    print("plot: %s" % path)


def spin_test(args):
    """Pivot in place: command yaw rates through the bridge (track_width from robot.yaml) and the firmware, and
    measure what the physics does. Shows the scrub breakaway and the effective track width."""
    from .skater import Scenario, Segment
    cfg = load_config(friction=args.friction, gyro_fault=args.gyro_fault)
    fw = {"YAW_LOOP_ENABLED": True} if args.yaw_loop else {}
    print("spin test, tyre friction %.2f (a guess), firmware yaw loop %s" % (cfg.friction, "ON" if args.yaw_loop else "off"))
    print("  commanded   achieved   wheel speed diff   effective track   motor current")
    for w_cmd in (0.5, 1.0, 2.0, 3.0, 4.0, 6.0):
        sc = Scenario("spin", "spin test", [Segment(6.0, 0.0)], robot_start=(0.0, 0.0, 0.0), settle=0.0)
        sim = Sim(cfg, sc, firmware_overrides=fw)
        sim.path.at = lambda t: (50.0, 50.0, 0.0, 0.0)
        sim.pi.follower_tick = lambda t: None
        acc = []

        def drive(s):
            s.pi.cmd_raw, s.pi.cmd_raw_t = (0.0, w_cmd), s.t
            if s.t > 4.0:
                acc.append((s.d.qvel[5], (s.d.qvel[s.jFR] - s.d.qvel[s.jFL]) * cfg.geom.wheel_r,
                            max(abs(s.motors[0].current), abs(s.motors[1].current))))
        sim.run(until=6.0, callback=drive)
        n = max(1, len(acc))
        yaw = sum(a[0] for a in acc) / n
        dv = sum(a[1] for a in acc) / n
        cur = sum(a[2] for a in acc) / n
        eff = ("%.2f m (%.1fx)" % (dv / yaw, dv / yaw / cfg.geom.track)) if yaw > 0.1 else "   -  (no turn)"
        print("  %5.1f rad/s  %5.2f rad/s   %5.2f m/s          %-17s %4.0f A" % (w_cmd, yaw, dv, eff, cur))
    print("Friction is a guess until the real robot exists: read this as a trend, and set the bridge's track_width "
          "from the real spin test (docs/first_drive.md).")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("scenario", nargs="?", help="scenario name (see --list)")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--all", action="store_true", help="run every scenario headless")
    ap.add_argument("--profile", choices=sorted(PROFILES), default="today")
    ap.add_argument("--view", action="store_true", help="3-D viewer (on macOS start with mjpython)")
    ap.add_argument("--speed", type=float, default=1.0, help="viewer playback speed (2 = twice real time)")
    ap.add_argument("--csv", help="write a 20 Hz time series here")
    ap.add_argument("--plot", help="save a PNG of speeds, gap, yaw, pan and motor current (needs matplotlib)")
    ap.add_argument("--seed", type=int, default=0, help="sensor-noise seed")
    ap.add_argument("--friction", type=float, default=None, help="tyre friction (default 0.8, a guess)")
    ap.add_argument("--spin-test", action="store_true")
    ap.add_argument("--remote", metavar="PORT",
                    help="drive with the real handheld remote plugged in by USB, e.g. /dev/cu.usbserial-XXXX (needs --view)")
    ap.add_argument("--keys", action="store_true", help="drive with the arrow keys in the viewer (needs --view)")
    ap.add_argument("--yaw-loop", action="store_true",
                    help="firmware 2.3.0 gyro yaw-rate loop ON (= YAW_LOOP_ENABLED true, a gyro fitted)")
    ap.add_argument("--gyro-fault", choices=["none", "sign", "stuck", "dead"], default="none",
                    help="with --yaw-loop: gyro mounted upside down, stuck at zero, or not answering")
    ap.add_argument("--side", action="store_true",
                    help="side view: 2 m, 45 deg behind on the right (config/side_shot.yaml); needs --yaw-loop (gyro)")
    ap.add_argument("--fw", action="append", metavar="CONST=VALUE",
                    help="override a firmware constant, e.g. --fw ESC_DEADBAND=0.12 (repeatable)")
    ap.add_argument("--set", action="append", metavar="NODE.PARAM=VALUE",
                    help="override a robot.yaml parameter, e.g. --set follower.max_linear=4 (repeatable)")
    args = ap.parse_args(argv)
    if args.friction is None:
        args.friction = load_config().friction
    if args.side and not args.yaw_loop:
        print("note: --side needs the gyro (--yaw-loop); without it the follower stays on the normal law")
    if args.list:
        for n, s in SCENARIOS.items():
            print("%-15s %5.0f s  %s" % (n, s.duration, s.description))
        return 0
    if args.spin_test:
        spin_test(args)
        return 0
    if args.all:
        results = [run_one(n, args) for n in SCENARIOS if n != "playground"]
        print("\n%d of %d scenarios pass (profile '%s')" % (sum(results), len(results), args.profile))
        return 0 if all(results) else 1
    if not args.scenario:
        ap.print_help()
        return 2
    if args.scenario not in SCENARIOS:
        print("unknown scenario %r; --list shows them" % args.scenario)
        return 2
    return 0 if run_one(args.scenario, args) else 1


if __name__ == "__main__":
    sys.exit(main())
