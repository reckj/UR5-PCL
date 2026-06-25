"""First REAL-ARM move — small validated Cartesian move above the table.

STATUS: experimental / internal bring-up tool. Supervised use only.

Unlike scripts/smoke_test_move.py (sim-only, bypasses safety), this goes through
the validated MotionController path with the real-arm gate enabled.

How it stays grounded without hard-coded cell coordinates: it reads the arm's
LIVE TCP pose and builds a tight workspace box AROUND that measured point, so the
software guard is derived from reality. The move is a small vertical hop (+Z, up,
away from the table) and back — slow, and you confirm before it runs.

Preconditions (REAL arm):
  - UR_HOST set to the arm's IP; arm powered on, brakes released, Remote Control.
  - PolyScope safety planes configured (wall + table). Speed slider turned DOWN.
  - Hand on the e-stop.
  - No tool installed -> payload declared as 0.

Usage (dev container, interactive):
    docker compose run --rm -e UR_HOST=192.168.x.x dev bash
    python scripts/first_real_move.py
"""

from __future__ import annotations

import math

from ur5e.connection import RobotConnection
from ur5e.config import resolve_host, is_sim
from ur5e.motion import MotionController
from ur5e.safety import SafetyLimits, Box

UP_DELTA_M = 0.04        # 40 mm vertical hop
BOX_MARGIN_M = 0.08      # workspace box half-size around the live pose
TCP_SPEED = 0.05         # 50 mm/s — extra slow for the first real move
TCP_ACCEL = 0.2

_TAU = 2 * math.pi


def main() -> None:
    host = resolve_host("real")
    if is_sim(host):
        raise SystemExit("UR_HOST is not set to a real arm (resolves to a sim host).")

    with RobotConnection(host, control=True, io=True) as r:
        if not r.connected:
            raise SystemExit("control interface not connected; arm ON + Remote Control?")

        # No tool installed -> declare zero payload + zero TCP (required by the gate).
        r.set_payload(0.0)
        r.set_tcp([0, 0, 0, 0, 0, 0])

        pose = list(r.receive.getActualTCPPose())
        x, y, z = pose[0], pose[1], pose[2]
        print("live TCP pose:", [round(v, 4) for v in pose])

        # Workspace box derived from the LIVE measured pose -> validated against reality.
        m = BOX_MARGIN_M
        limits = SafetyLimits(
            name="first_real_move_live",
            tcp_speed_max=TCP_SPEED, tcp_accel_max=TCP_ACCEL,
            joint_speed_max=0.2, joint_accel_max=0.4,
            joint_limits=tuple((-_TAU, _TAU) for _ in range(6)),
            workspace=Box(x=(x - m, x + m), y=(y - m, y + m), z=(z - m, z + m + UP_DELTA_M)),
            payload_max_kg=0.5,
            require_payload=True, require_tcp=True,
            validated=True,   # box built from the measured live pose
        )
        mc = MotionController(r, limits, allow_real=True)

        target = [x, y, z + UP_DELTA_M, pose[3], pose[4], pose[5]]
        print(f"planned move: up {UP_DELTA_M*1000:.0f} mm at {TCP_SPEED*1000:.0f} mm/s, then back.")
        print("CHECK: speed slider down, hand on e-stop, area clear.")
        if input("type 'move' to proceed: ").strip().lower() != "move":
            print("aborted — no motion.")
            return

        print("moving up ...")
        mc.move_l(target)
        print("returning ...")
        mc.move_l(pose)
        print("done — back at start.")


if __name__ == "__main__":
    main()
