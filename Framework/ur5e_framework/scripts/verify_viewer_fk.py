"""Verify the viewer's mesh chain against the framework's DH kinematics.

STATUS: internal validation tool. No robot needed, no RTDE import.

Checks, for many random joint vectors:
  1. flange POSITION  : viewer chain (ur_description joint tree, rooted in the
                        controller base frame) == DH FK (ur5e.kinematics)
  2. flange ORIENTATION: viewer ``tool0`` rotation == DH frame-6 rotation
  3. joint origins    : viewer j1..j6 origins vs DH frame origins. j1, j4, j5,
                        j6 coincide; j2/j3 differ by exactly a2 = 0.425 / a3 =
                        0.3922 because DH puts a frame at the END of each arm
                        link while the URDF puts it on the joint axis. So the
                        viewer's "elbow" (j3 origin) is the DH frame-2 origin.
  4. sanity           : home pose (0,-90,0,-90,0,0) tool0 = (0, -0.2329, 1.0794) m
                        (up: d1 + a2 + a3 + d5 = 0.1625+0.425+0.3922+0.0997,
                        sideways: d4 + d6 = 0.1333+0.0996)

Run inside the dev container:  python scripts/verify_viewer_fk.py
Exit code 0 = pass.
"""

from __future__ import annotations

import sys

import numpy as np

from ur5e.kinematics import fk_frames
from ur5e.viewer.robot_model import fk, HOME_Q

TOL_POS = 1e-9   # metres
TOL_ROT = 1e-9


def main() -> int:
    rng = np.random.default_rng(0)
    n = 2000
    worst_pos = worst_rot = 0.0
    worst_joint = np.zeros(6)
    reach = 0.0
    for _ in range(n):
        q = rng.uniform(-2 * np.pi, 2 * np.pi, 6)
        dh = fk_frames(q)            # list: base + 6 frames (DH)
        vw = fk(q)
        dp = np.linalg.norm(dh[6][:3, 3] - vw["tool0"][:3, 3])
        dr = np.abs(dh[6][:3, :3] - vw["tool0"][:3, :3]).max()
        worst_pos = max(worst_pos, dp)
        worst_rot = max(worst_rot, dr)
        for i in range(1, 7):
            worst_joint[i - 1] = max(worst_joint[i - 1],
                                     np.linalg.norm(dh[i][:3, 3] - vw[f"j{i}"][:3, 3]))
        reach = max(reach, np.linalg.norm(vw["tool0"][:3, 3]))

    print(f"samples: {n}")
    print(f"flange position  max |viewer - DH| = {worst_pos:.3e} m   (tol {TOL_POS})")
    print(f"flange rotation  max |viewer - DH| = {worst_rot:.3e}     (tol {TOL_ROT})")
    print("joint-origin max deviation per joint [m]:",
          " ".join(f"j{i+1}={worst_joint[i]:.2e}" for i in range(6)))
    print(f"max flange distance from base origin over samples = {reach:.4f} m")
    home = fk(HOME_Q)["tool0"][:3, 3]
    print("home pose tool0 xyz =", np.round(home, 4).tolist())
    home_ok = np.allclose(home, [0.0, -(0.1333 + 0.0996), 0.1625 + 0.425 + 0.3922 + 0.0997], atol=1e-9)

    ok = worst_pos < TOL_POS and worst_rot < TOL_ROT and home_ok
    print("RESULT:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
