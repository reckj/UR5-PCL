"""Grid traversal with per-point DI0 gating — sim by default, real with --target real.

Moves through a 2 x 4 grid (8 points). At each point it waits for a RISING EDGE
on digital input 0 (DI0) before advancing — i.e. press DI0 to step to the next
point. Built on the student-facing Robot API and the validated cart_against_wall
profile (real).

The grid is centered in the active profile's workspace box (so every point is
in-bounds regardless of where the TCP starts), at the current TCP height and
orientation. Each move is validated by the safety layer.

Note: in SIM, DI0 reads 0 unless you set it HIGH in URSim's I/O tab, so the wait
will block until you toggle it. On the real arm, wire DI0 / use the I/O panel.

Usage:
    # sim:
    python scripts/grid_di0_test.py
    # real (lab):
    docker compose run --rm --no-deps -e UR_HOST=192.168.x.x dev bash
    python scripts/grid_di0_test.py --target real
"""

from __future__ import annotations

import argparse
import time

from ur5e import Robot, grid_poses
from ur5e.safety import SafetyViolation

NX, NY = 2, 4            # 2 x 4 = 8 points
DX, DY = 0.06, 0.06      # 60 mm spacing
DI_PIN = 0
POLL_S = 0.02


def wait_for_di_rising(robot: Robot, pin: int) -> None:
    """Block until a rising edge on DIn: wait for release (if held), then press."""
    while robot.read_digital_in(pin):
        time.sleep(POLL_S)
    while not robot.read_digital_in(pin):
        time.sleep(POLL_S)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--target", choices=["sim", "real"], default="sim")
    args = ap.parse_args()
    real = args.target == "real"
    profile = "cart_against_wall" if real else "sim_open"

    with Robot(target=args.target, profile=profile, allow_real=real) as robot:
        if real:
            robot.set_tool(0.0)  # no tool installed

        # Center the grid in the workspace box, at the current height/orientation.
        box = robot.limits.workspace
        cur = robot.pose()
        z = min(max(cur[2], box.z[0]), box.z[1])
        cx = (box.x[0] + box.x[1]) / 2.0
        cy = (box.y[0] + box.y[1]) / 2.0
        x0 = cx - (NX - 1) * DX / 2.0
        y0 = cy - (NY - 1) * DY / 2.0
        origin = [x0, y0, z, cur[3], cur[4], cur[5]]
        points = grid_poses(origin, nx=NX, ny=NY, dx=DX, dy=DY)

        print(f"{len(points)} grid points, centered in profile '{robot.limits.name}'.")
        print(f"Press DI{DI_PIN} to advance at each point.\n")
        try:
            for k, p in enumerate(points):
                robot.move_to(p)
                print(f"  point {k + 1}/{len(points)} reached -> waiting for DI{DI_PIN} ...")
                wait_for_di_rising(robot, DI_PIN)
            print("grid complete; returning to start ...")
            robot.move_to(origin)
            print("done.")
        except SafetyViolation as e:
            print("REJECTED:", e)
            print("(a grid point left the workspace box — reduce spacing or recheck the profile)")
        except KeyboardInterrupt:
            print("\naborted by user.")


if __name__ == "__main__":
    main()
