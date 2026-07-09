"""Toolhead RGB + button -> live visualisation + button-gated robot motion.

STATUS: experimental / internal. NOT validated for student use. The live RS485
path is REAL-ARM ONLY (URSim has no tool RS485) - but the whole host side runs
offline with --source mock.

WHAT HAPPENS
    A background thread reads toolhead frames (R,G,B,C,BTN) from either the live
    UR Tool Communication Interface (TCP <robot-ip>:54321) or a synthetic mock.
    The SAME stream drives two things at once:
      1. a live matplotlib window (RGB swatch + bars + button state), and
      2. robot motion: on each button press, R/G/B map to base-frame X/Y/Z
         offsets (5-30 mm) and the TCP steps by that amount (cumulative),
         clamped into a small safe box and validated by the safety layer.

ASSUMPTIONS / SAFETY (flagged per project rules)
    - Tool: the instrumented toolhead (Arduino + APDS9960), < 500 g. You MUST
      declare its payload (--payload) and TCP (--tcp-z) before real motion;
      the safety gate refuses real motion otherwise.
    - Motion is base-frame XYZ only (orientation held from the start pose).
    - Cumulative stepping is clamped to +/- --span-mm around the start pose so
      it cannot walk away; the SafetyLimits profile remains the hard backstop.
    - Real motion additionally requires --allow-real AND a *validated* profile
      (cart_against_wall must have validated=True). Sim is the default.

EXAMPLES
    # fully offline - synthetic data, just the visualisation (no robot, no deps):
    python examples/toolhead_rgb_motion.py --source mock --no-motion

    # offline full pipeline against URSim:
    python examples/toolhead_rgb_motion.py --source mock --target sim

    # live lab run against the real arm + real toolhead:
    python examples/toolhead_rgb_motion.py --source tci --robot-ip 192.168.1.10 \\
        --target real --allow-real --payload 0.3 --tcp-z 0.06
"""

from __future__ import annotations

import argparse
import threading
import time

# Package import is light here; Robot (ur_rtde) is imported lazily only if motion
# is enabled, so --no-motion runs with no robot dependencies at all.
from ur5e.toolio import (
    ButtonEdge,
    MockToolSource,
    TciStream,
    map_rgb_to_offset,
)


# --------------------------------------------------------------------------- #
# Shared state between the reader/motion thread and the viz (main) thread
# --------------------------------------------------------------------------- #
class Shared:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.frame = None          # latest ToolFrame
        self.last_offset_mm = (0.0, 0.0, 0.0)
        self.moves = 0
        self.note = ""
        self.stop = False


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


# --------------------------------------------------------------------------- #
# Reader + motion thread
# --------------------------------------------------------------------------- #
def run_reader(source, shared: Shared, args, robot) -> None:
    """Consume frames: update shared state, and on each button rising edge,
    step the robot (cumulative RGB->XYZ, clamped). Single owner of `robot`."""
    edge = ButtonEdge()

    base = None
    safety_violation = Exception
    if robot is not None:
        base = list(robot.pose())  # capture start pose; orientation held
        try:
            from ur5e.safety import SafetyViolation as safety_violation  # type: ignore
        except Exception:
            safety_violation = Exception

    try:
        for frame in source.frames():
            if shared.stop:
                break
            with shared.lock:
                shared.frame = frame

            if not edge.rising(frame.button):
                continue

            # --- button pressed: compute + execute a motion step ---
            dx, dy, dz = map_rgb_to_offset(
                frame, min_mm=args.min_mm, max_mm=args.max_mm, signed=args.signed
            )
            with shared.lock:
                shared.last_offset_mm = (dx * 1000, dy * 1000, dz * 1000)

            if robot is None:
                with shared.lock:
                    shared.moves += 1
                    shared.note = "press (no-motion)"
                continue

            cur = list(robot.pose())
            span = args.span_mm / 1000.0
            if args.mode == "cumulative":
                tx, ty, tz = cur[0] + dx, cur[1] + dy, cur[2] + dz
            else:  # absolute: offset from the captured base pose
                tx, ty, tz = base[0] + dx, base[1] + dy, base[2] + dz
            # keep inside a small safe box around the start pose
            tx = clamp(tx, base[0] - span, base[0] + span)
            ty = clamp(ty, base[1] - span, base[1] + span)
            tz = clamp(tz, base[2] - span, base[2] + span)
            target = [tx, ty, tz, cur[3], cur[4], cur[5]]

            try:
                robot.move_to(target, speed=args.speed, accel=args.accel)
                with shared.lock:
                    shared.moves += 1
                    shared.note = f"move #{shared.moves}"
            except safety_violation as exc:  # type: ignore
                with shared.lock:
                    shared.note = f"rejected: {exc}"
    finally:
        try:
            source.close()
        except Exception:
            pass


# --------------------------------------------------------------------------- #
# Visualisation (matplotlib, main thread)
# --------------------------------------------------------------------------- #
def run_viz(shared: Shared, args) -> None:
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    from matplotlib.patches import Rectangle

    fig, (ax_sw, ax_bar) = plt.subplots(1, 2, figsize=(8, 3.4))
    fig.canvas.manager.set_window_title("Toolhead RGB / button")

    ax_sw.set_xlim(0, 1); ax_sw.set_ylim(0, 1); ax_sw.axis("off")
    swatch = Rectangle((0, 0), 1, 1, color=(0, 0, 0))
    ax_sw.add_patch(swatch)
    sw_text = ax_sw.text(0.5, 0.5, "", ha="center", va="center",
                         fontsize=12, fontweight="bold")

    ax_bar.set_ylim(0, 255); ax_bar.set_xlim(-0.5, 2.5)
    ax_bar.set_xticks([0, 1, 2]); ax_bar.set_xticklabels(["R", "G", "B"])
    ax_bar.set_ylabel("0-255"); ax_bar.set_title("waiting for data...")
    bars = ax_bar.bar([0, 1, 2], [0, 0, 0],
                      color=["#d33", "#3a3", "#36d"])

    def update(_):
        with shared.lock:
            frame = shared.frame
            offs = shared.last_offset_mm
            note = shared.note
            moves = shared.moves
        if frame is None:
            return
        r8, g8, b8 = frame.rgb8
        swatch.set_color((r8 / 255, g8 / 255, b8 / 255))
        sw_text.set_text("PRESSED" if frame.button else "")
        sw_text.set_color("white" if (r8 + g8 + b8) < 384 else "black")
        for bar, v in zip(bars, (r8, g8, b8)):
            bar.set_height(v)
        ax_bar.set_title(
            f"R{r8} G{g8} B{b8}   btn={int(frame.button)}\n"
            f"last move dxyz={offs[0]:+.0f},{offs[1]:+.0f},{offs[2]:+.0f} mm"
            f"   moves={moves}  {note}"
        )
        return

    anim = FuncAnimation(fig, update, interval=80, cache_frame_data=False)
    try:
        plt.show()
    finally:
        shared.stop = True
    _ = anim  # keep ref


# --------------------------------------------------------------------------- #
def build_source(args):
    if args.source == "mock":
        return MockToolSource(period=args.period)
    if args.source == "tci":
        if not args.robot_ip:
            raise SystemExit("--source tci requires --robot-ip")
        return TciStream(args.robot_ip).connect()
    raise SystemExit(f"unknown source {args.source!r}")


def build_robot(args):
    if args.no_motion:
        return None
    from ur5e import Robot  # lazy: only needed when actually moving
    host = args.robot_ip if args.target == "real" else None
    robot = Robot(target=args.target, allow_real=args.allow_real, host=host).connect()
    if args.target == "real":
        if args.payload is None:
            robot.disconnect()
            raise SystemExit("real motion requires --payload (kg) for the toolhead")
        robot.set_tool(payload_kg=args.payload, tcp=(0, 0, args.tcp_z, 0, 0, 0))
    return robot


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--source", choices=["mock", "tci"], default="mock")
    p.add_argument("--robot-ip", default=None,
                   help="UR controller IP (for --source tci and --target real)")
    p.add_argument("--target", choices=["sim", "real"], default="sim")
    p.add_argument("--allow-real", action="store_true",
                   help="required to drive the real arm (plus a validated profile)")
    p.add_argument("--payload", type=float, default=None,
                   help="toolhead payload mass in kg (required for real motion)")
    p.add_argument("--tcp-z", type=float, default=0.0,
                   help="TCP Z offset in m from the flange (tool length)")
    p.add_argument("--no-motion", action="store_true",
                   help="visualise only; do not connect to / move any robot")
    p.add_argument("--no-viz", action="store_true",
                   help="run headless (no matplotlib window)")
    p.add_argument("--mode", choices=["cumulative", "absolute"], default="cumulative")
    p.add_argument("--min-mm", type=float, default=5.0)
    p.add_argument("--max-mm", type=float, default=30.0)
    p.add_argument("--span-mm", type=float, default=40.0,
                   help="half-size of the safe box around the start pose (mm)")
    p.add_argument("--speed", type=float, default=0.05, help="TCP speed m/s")
    p.add_argument("--accel", type=float, default=0.2, help="TCP accel m/s^2")
    p.add_argument("--signed", action="store_true",
                   help="map colors to signed +/- offsets (see map_rgb_to_offset)")
    p.add_argument("--period", type=float, default=0.05,
                   help="mock sample period (s)")
    args = p.parse_args()

    shared = Shared()
    source = build_source(args)
    robot = build_robot(args)

    print(f"[toolhead] source={args.source} target={args.target} "
          f"motion={'off' if robot is None else 'on'} viz={'off' if args.no_viz else 'on'}")
    if robot is not None:
        print(f"[toolhead] start pose captured; mode={args.mode}, "
              f"step {args.min_mm}-{args.max_mm} mm, box +/-{args.span_mm} mm")

    reader = threading.Thread(target=run_reader, args=(source, shared, args, robot),
                              daemon=True)
    reader.start()

    try:
        if args.no_viz:
            while reader.is_alive():
                time.sleep(0.2)
        else:
            run_viz(shared, args)
    except KeyboardInterrupt:
        pass
    finally:
        shared.stop = True
        try:
            source.close()
        except Exception:
            pass
        reader.join(timeout=2.0)
        if robot is not None:
            robot.disconnect()


if __name__ == "__main__":
    main()
