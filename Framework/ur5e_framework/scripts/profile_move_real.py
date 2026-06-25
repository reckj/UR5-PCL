"""Validated real-arm move using the cart_against_wall PROFILE — supervised.

Confirms the persistent safety profile (not a per-session live box) drives real
motion end to end. Requires cart_against_wall validated=True (run
teach_workspace first, paste the Box, set validated=True). Small slow vertical
hop with a typed confirmation.

Preconditions: arm ON, brakes released, Remote Control; UR_HOST set; speed slider
down; hand on the e-stop; area clear; no tool installed.

Usage:
    docker compose run --rm -e UR_HOST=192.168.x.x dev bash
    python scripts/profile_move_real.py
"""

from __future__ import annotations

from ur5e import Robot

UP_DELTA_M = 0.04


def main() -> None:
    with Robot(target="real", profile="cart_against_wall", allow_real=True) as robot:
        robot.set_tool(0.0)  # no tool installed -> zero payload + zero TCP
        pose = list(robot.pose())
        print("live TCP pose:", [round(v, 4) for v in pose])

        target = list(pose)
        target[2] += UP_DELTA_M
        print(f"planned: up {UP_DELTA_M*1000:.0f} mm at profile speed, then back.")
        print("CHECK: speed slider down, hand on e-stop, area clear.")
        if input("type 'move' to proceed: ").strip().lower() != "move":
            print("aborted — no motion.")
            return

        robot.move_to(target)
        robot.move_to(pose)
        print("done — back at start.")


if __name__ == "__main__":
    main()
