"""Grid traversal gated by digital input 0 (DI0) — wait at each point for DI0.

A 2x4 grid (8 points) centred on the current TCP pose, at the current
orientation, generated with grid_poses(). At each point the robot moves there,
then WAITS for DI0 to trigger (a low->high edge) before advancing. The wait is a
plain, visible polling loop — no callbacks — so the control flow is obvious for
students to read and adapt.

Default target is sim (toggle DI0 in URSim's I/O tab to advance). Pass
--target real to run on hardware via the validated cart_against_wall profile.

REAL preconditions: arm ON + Remote Control, UR_HOST set, speed slider down,
hand on the e-stop, no tool. Position the TCP centrally over the surface first —
the grid extends ~+/-2.5 cm in X and ~+/-7.5 cm in Y around the start pose and
must stay inside the validated box, or moves are rejected.

Usage:
    python scripts/grid_di_test.py                  # sim
    # real:
    docker compose run --rm --no-deps -e UR_HOST=192.168.x.x dev bash
    python scripts/grid_di_test.py --target real
"""

from __future__ import annotations

import argparse
import time

from ur5e import Robot, grid_poses

NX, NY = 2, 4            # 2 x 4 = 8 points
DX, DY = 0.05, 0.05      # grid spacing (m)
DI_PIN = 0               # digital input to wait on
POLL_S = 0.05


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--target", choices=["sim", "real"], default="sim")
    args = ap.parse_args()
    allow_real = args.target == "real"

    with Robot(target=args.target, allow_real=allow_real) as robot:
        if allow_real:
            robot.set_tool(0.0)   # no tool installed

        start = list(robot.pose())
        # Centre the grid on the current pose; keep the current orientation.
        ox = start[0] - (NX - 1) * DX / 2
        oy = start[1] - (NY - 1) * DY / 2
        origin = [ox, oy, start[2], start[3], start[4], start[5]]
        cells = grid_poses(origin, NX, NY, DX, DY)

        print(f"{len(cells)} points ({NX}x{NY}), centred on current pose, "
              f"z={start[2]:.3f}. Trigger DI{DI_PIN} to advance at each point.")

        for k, cell in enumerate(cells):
            print(f"-> point {k + 1}/{len(cells)}: moving ...")
            robot.move_to(cell)

            # Wait for a rising edge on DI0 — plain visible polling loop.
            print(f"   at point {k + 1}; waiting for DI{DI_PIN} (low -> high) ...")
            while robot.read_digital_in(DI_PIN):       # ensure it's released first
                time.sleep(POLL_S)
            while not robot.read_digital_in(DI_PIN):   # then wait for the trigger
                time.sleep(POLL_S)
            print(f"   DI{DI_PIN} triggered.")

        print("grid complete; returning to start ...")
        robot.move_to(start)
    print("done.")


if __name__ == "__main__":
    main()
