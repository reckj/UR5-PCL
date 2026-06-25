"""Safety-layer demo — SIMULATION ONLY.

STATUS: experimental / internal.

Exercises the validated motion layer against URSim: shows an accepted slow move
and several correctly-rejected ones (out-of-workspace, over-speed). Run the
viewer alongside to watch the accepted move.

Usage (inside the dev container, URSim ON/STARTED):
    python scripts/safe_move_demo.py
"""

from __future__ import annotations

import math

from ur5e.connection import RobotConnection
from ur5e.config import resolve_host
from ur5e.motion import MotionController
from ur5e.safety import get_profile, SafetyViolation


def expect_reject(label, fn):
    try:
        fn()
        print(f"  [FAIL] {label}: was NOT rejected")
    except SafetyViolation as e:
        print(f"  [ok]   rejected {label}: {e}")


def main() -> None:
    host = resolve_host("sim")
    with RobotConnection(host, control=True) as r:
        # sim_open: generous box so a normal move passes; slow-speed guards active.
        mc = MotionController(r, get_profile("sim_open"))

        q0 = r.receive.getActualQ()
        print("start q:", [round(x, 3) for x in q0])

        print("accepted: small wrist move at profile speed ...")
        target = list(q0)
        target[4] += math.radians(10.0)
        mc.move_j(target)
        mc.move_j(q0)  # return
        print("  [ok]   move accepted and completed")

        print("rejections (should all be rejected):")
        expect_reject("over-speed move_j", lambda: mc.move_j(q0, speed=99.0))
        expect_reject("out-of-box move_l",
                      lambda: mc.move_l([0.0, 0.0, 5.0, 0.0, 0.0, 0.0]))

        # Tight, unvalidated cell profile: home is likely outside the provisional
        # box, demonstrating the workspace guard.
        cart = get_profile("cart_against_wall")
        mc_cart = MotionController(r, cart)
        expect_reject("move_j outside cart box",
                      lambda: mc_cart.move_j(q0))

    print("done.")


if __name__ == "__main__":
    main()
