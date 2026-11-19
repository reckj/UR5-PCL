"""Check the cart cell (`cart_360`) keep-out logic — INSTRUCTOR tool.

STATUS: internal (not synced to the student repo).

    python scripts/verify_cart_cell.py                 offline: geometry + motion-layer logic on a
                                                       FIXED test cart (independent of the measured cell)
    python scripts/verify_cart_cell.py --target sim    + live against URSim with the MEASURED cell:
                                                       controller IK available? rejections reject,
                                                       allowed moves above the cart top execute
    python scripts/verify_cart_cell.py --target sim --reach
                                                       additionally try to go beside and below the cart
                                                       (prints what the checks say; watch the viewer)

Offline mode needs no robot (and no ur_rtde: it is stubbed if missing). The
live mode moves the SIMULATED arm; it refuses --target real.
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time
import types

import numpy as np

try:
    import rtde_control  # noqa: F401
except ImportError:                       # offline on a machine without ur_rtde
    for _name, _cls in (("rtde_control", "RTDEControlInterface"), ("rtde_receive", "RTDEReceiveInterface"),
                        ("rtde_io", "RTDEIOInterface")):
        _m = types.ModuleType(_name)
        setattr(_m, _cls, type(_cls, (), {}))
        sys.modules[_name] = _m

from ur5e import kinematics as kin                      # noqa: E402
from ur5e.motion import MotionController                # noqa: E402
from ur5e.safety import SafetyViolation, cart_profile, get_profile   # noqa: E402

DOWN = [0.0, math.pi, 0.0]                              # tool pointing straight down
Q_URSIM_START = [math.radians(v) for v in (-92, -99, -126, -46, 91, -2)]

TEST_CELL = {"format": "ur5e-cell/1", "name": "test_cart", "validated": False,
             "cart": {"x": [-0.56, 0.20], "y": [-0.32, 0.32], "top_z": 0.0}, "floor_z": -0.85,
             "side_clearance": 0.05, "arm_clearance": 0.02, "floor_clearance": 0.05, "flange_floor_clearance": 0.20,
             "reach": {"x": [-0.95, 0.95], "y": [-0.95, 0.95], "z_max": 1.0},
             "extras": [{"name": "handle", "x": [-0.45, 0.0], "y": [0.32, 0.38], "z": [-0.07, -0.04]}]}

results = []


def check(name: str, ok: bool, detail: str = "") -> None:
    results.append(ok)
    print(f"  {'PASS' if ok else 'FAIL'}  {name}" + (f"   [{detail}]" if detail else ""))


def rejected(fn):
    """The SafetyViolation text if fn() is rejected, else None."""
    try:
        fn()
    except SafetyViolation as e:
        return str(e)
    return None


def numeric_ik(pose, q_near, iters=200):
    """Damped least-squares IK on the nominal DH model (offline stand-in for the controller)."""
    q = np.array(q_near, dtype=float)
    Rt, pt = kin.rotvec_to_matrix(pose[3:6]), np.array(pose[:3], dtype=float)

    def err(qq):
        T = kin.fk_frames(qq)[-1]
        return np.concatenate([pt - T[:3, 3], kin.matrix_to_rotvec(Rt @ T[:3, :3].T)])

    for _ in range(iters):
        e = err(q)
        if np.linalg.norm(e[:3]) < 1e-6 and np.linalg.norm(e[3:]) < 1e-6:
            return [float(v) for v in q]
        J = np.zeros((6, 6))
        for j in range(6):
            dq = np.zeros(6)
            dq[j] = 1e-6
            J[:, j] = -(err(q + dq) - e) / 1e-6
        q = q + 0.7 * np.linalg.solve(J.T @ J + 1e-6 * np.eye(6), J.T @ e)
    return None


def pose_of(q):
    T = kin.fk_frames(q)[-1]
    return [*T[:3, 3], *kin.matrix_to_rotvec(T[:3, :3])]


# ------------------------------------------------------------- fake robot ----
class _FakeReceive:
    def __init__(self, robot):
        self.robot = robot

    def getActualQ(self):
        self.robot.reads += 1
        return list(self.robot.q)

    def getActualTCPPose(self):
        self.robot.reads += 1
        return pose_of(self.robot.q)


class _FakeControl:
    def __init__(self, robot):
        self.robot = robot

    def moveJ(self, q, speed, accel):
        self.robot.q = list(q)
        self.robot.moves.append(("J", list(q)))
        return True

    def moveL(self, pose, speed, accel):
        q = numeric_ik(pose, self.robot.q)
        assert q is not None, "test asked the fake robot for an unreachable moveL"
        self.robot.q = q
        self.robot.moves.append(("L", list(pose)))
        return True


class FakeRobot:
    """Just enough of RobotConnection for MotionController, on the nominal model."""
    host = "127.0.0.1"          # counts as sim: no real-arm gate
    connected = True
    tcp = None

    def __init__(self, q):
        self.q, self.moves, self.reads = list(q), [], 0
        self.receive, self.control = _FakeReceive(self), _FakeControl(self)


# ---------------------------------------------------------------- offline ----
def offline() -> None:
    P = cart_profile(TEST_CELL)
    side_y = -0.32                     # the -Y side of the test cart
    print("\n1. profiles")
    check("cart profile has keep-outs (cart + handle) and a floor", P.has_obstacles and len(P.keepout) == 2 and P.floor_z == -0.85)
    check("sim_open / cart_against_wall have none (unchanged behaviour)",
          not get_profile("sim_open").has_obstacles and not get_profile("cart_against_wall").has_obstacles)
    check("measured cell 'cart_360' loads", get_profile("cart_360").has_obstacles,
          f"validated={get_profile('cart_360').validated}")

    print("\n2. target point (TCP)")
    ok = lambda p: rejected(lambda: P.check_pose_target([*p, *DOWN])) is None
    check("above the cart top: allowed", ok((-0.30, 0.0, 0.20)))
    check("touching the cart top (z = top): allowed", ok((-0.30, 0.0, 0.0)))
    check("inside the cart body: rejected", not ok((-0.30, 0.0, -0.10)))
    check("beside the cart, below the top, 30 mm from the side: rejected", not ok((-0.30, side_y - 0.03, -0.10)))
    check("beside the cart, below the top, 60 mm from the side: allowed", ok((-0.30, side_y - 0.06, -0.10)))
    check("beside the cart, ABOVE the top, 30 mm out: allowed", ok((-0.30, side_y - 0.03, 0.05)))
    check("40 mm above the floor: rejected (floor clearance 50)", not ok((-0.30, -0.60, -0.81)))
    check("outside the reach box: rejected", not ok((-0.30, -1.00, 0.20)))

    check("in the handle on the +Y side (below the top, 30 mm out): rejected", not ok((-0.30, 0.35, -0.05)))
    check("under the handle, 300 mm below the top, 60 mm from the cart side: allowed", ok((-0.30, 0.38, -0.30)))
    check("beside the handle's end, same height, outside it: allowed", ok((0.10, 0.38, -0.05)))

    print("\n3. straight lines (linear move, no IK)")
    line = lambda a, b: rejected(lambda: P.check_linear_path([*a, *DOWN], [*b, *DOWN]))
    msg = line((-0.30, -0.10, 0.20), (-0.30, -0.57, -0.25))
    check("from above the top straight to beside-and-below: rejected", msg is not None and "TCP path" in msg, (msg or "")[:60])
    check("same target via a waypoint outside the edge: both legs allowed",
          line((-0.30, -0.10, 0.20), (-0.30, -0.57, 0.20)) is None and line((-0.30, -0.57, 0.20), (-0.30, -0.57, -0.25)) is None)
    check("along the top surface: allowed", line((-0.50, -0.20, 0.0), (0.0, 0.20, 0.0)) is None)
    check("under the cart from side to side: rejected", line((-0.30, -0.60, -0.40), (-0.30, 0.60, -0.40)) is not None)
    check("straight down past the cart side through the handle: rejected",
          line((-0.30, 0.38, 0.20), (-0.30, 0.38, -0.30)) is not None)
    check("straight down 150 mm out, clear of the handle: allowed", line((-0.30, 0.47, 0.20), (-0.30, 0.47, -0.30)) is None)
    msg = line((-0.30, 0.0, -0.20), (-0.30, 0.0, 0.30))
    check("start already inside: message says so", msg is not None and "ALREADY" in msg)
    tcp = [0.0, 0.0, 0.15, 0.0, 0.0, 0.0]
    sideways = list(kin.matrix_to_rotvec(np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]], dtype=float).T))   # tool Z along +Y
    msg = rejected(lambda: P.check_pose_target([-0.30, side_y - 0.10, -0.10, *sideways], tcp=tcp))
    check("tool pointing at the cart side, TCP 100 mm out, flange further out: allowed", msg is None, (msg or "")[:80])
    away = list(kin.matrix_to_rotvec(np.array([[0, 0, -1], [1, 0, 0], [0, -1, 0]], dtype=float).T))    # tool Z along -Y
    msg = rejected(lambda: P.check_pose_target([-0.30, side_y - 0.10, -0.10, *away], tcp=tcp))
    check("tool pointing away, flange would be inside the cart side: rejected", msg is not None and "flange to TCP" in msg)

    print("\n4. arm links")
    arm = lambda q: rejected(lambda: P.check_arm(q, None, "test"))
    q_table = numeric_ik([-0.30, 0.0, 0.02, *DOWN], Q_URSIM_START)
    check("table work, flange 20 mm above the top, tool down: allowed", q_table is not None and arm(q_table) is None)
    msg = arm([0.0, 0.0, -math.pi / 2, 0.0, 0.0, 0.0])
    msg2 = arm([0.0, 0.0, math.pi / 2, 0.0, 0.0, 0.0])
    check("forearm folded down through the cart top: rejected", any(m and "forearm" in m for m in (msg, msg2)))
    q_out = numeric_ik([-0.20, -0.57, -0.20, *DOWN], Q_URSIM_START)
    q_near = numeric_ik([-0.20, -0.45, -0.20, *DOWN], Q_URSIM_START)
    check("TCP 250 mm beside, 200 mm below the top: allowed", q_out is not None and arm(q_out) is None)
    msg = arm(q_near)
    check("TCP 130 mm beside, 200 mm below: forearm too close to the edge -> rejected", msg is not None and "forearm" in msg)
    long_tool = [0.0, 0.0, 0.30, 0.0, 0.0, 0.0]
    low = lambda z, tcp: rejected(lambda: P.check_pose_target([-0.20, -0.62, z, *DOWN], tcp=tcp))
    check("300 mm tool, tip 100 mm above the floor (flange 400): allowed", low(-0.75, long_tool) is None)
    msg = low(-0.75, None)
    check("no tool, flange 100 mm above the floor: rejected (test cart has a pendant floor plane at 200)", msg is not None and "flange" in msg)

    # the lab cart (numbers of 2026-11-18), links at their real radii, no extra clearance:
    # JReck's reference on the real arm (2026-11-19) — base 0 deg, shoulder -222 deg clears the cart's back edge
    lab = cart_profile({"format": "ur5e-cell/1", "name": "lab_cart", "validated": False,
                        "cart": {"x": [-0.566, 0.124], "y": [-0.345, 0.345], "top_z": -0.010}, "floor_z": -0.800,
                        "side_clearance": 0.02, "arm_clearance": 0.0, "floor_clearance": 0.05,
                        "reach": {"x": [-0.95, 0.95], "y": [-0.95, 0.95], "z_max": 1.0}})
    dip = lambda deg: rejected(lambda: lab.check_arm([0.0, math.radians(deg), 0.0, -math.pi / 2, 0.0, 0.0], None, "test"))
    check("lab cart: base 0, shoulder -222 deg (arm dipping over the back edge): allowed", dip(-222.0) is None)
    msg = dip(-224.0)
    check("lab cart: shoulder -224 deg: upper arm touches the back edge -> rejected", msg is not None and "upper arm" in msg)

    print("\n5. joint path (moveJ)")
    q_other = list(q_out)
    q_other[0] -= math.pi                   # same pose mirrored to the +Y side; the base turns through the FRONT (-X)
    a, b = arm(q_out), arm(q_other)
    msg = rejected(lambda: P.check_joint_path(q_out, q_other))
    check("both ends allowed, but the swing passes the cart front: rejected on the way",
          a is None and b is None and msg is not None and "of the way" in msg, (msg or "")[:70])
    q_up = numeric_ik([-0.20, -0.57, 0.20, *DOWN], q_out)
    check("straight up beside the cart (joint move): allowed", rejected(lambda: P.check_joint_path(q_out, q_up)) is None)

    print("\n6. motion layer (MotionController on a fake robot)")
    rob = FakeRobot(q_up)
    mc = MotionController(rob, P)
    mc.move_l([-0.20, -0.57, -0.20, *DOWN])
    check("move_l down beside the cart executes", rob.moves[-1][0] == "L")
    n = len(rob.moves)
    msg = rejected(lambda: mc.move_l([-0.30, 0.0, 0.20, *DOWN]))
    check("move_l from beside-and-below straight to above the top: rejected, nothing sent", msg is not None and len(rob.moves) == n)
    msg = rejected(lambda: mc.move_l([-0.20, -0.45, -0.20, *DOWN]))
    check("move_l closer to the cart: rejected by the arm check", msg is not None and "linear move" in msg and len(rob.moves) == n, (msg or "")[:70])
    msg = rejected(lambda: mc.move_j(q_other))
    check("move_j swinging through the cart: rejected, nothing sent", msg is not None and len(rob.moves) == n)
    mc.move_j(q_up)
    check("move_j back up executes", rob.moves[-1][0] == "J")

    q_ik = kin.ik_near([-0.20, -0.57, -0.20, *DOWN], q_up)
    check("framework IK (ik_near) agrees with the independent test IK", q_ik is not None and max(abs(a - b) for a, b in zip(q_ik, q_out)) < 1e-3)
    rng = np.random.default_rng(7)
    worst, fails, tried = 0.0, 0, 0
    while tried < 200:
        q0 = rng.uniform([-3, -2.6, -2.4, -3, 0.4, -3], [3, -0.5, -0.4, 3, 2.7, 3])      # elbow and wrist not stretched
        wrist = kin.fk_frames(q0)[5][:3, 3]
        if math.hypot(wrist[0], wrist[1]) < 0.20:        # wrist over the base axis = shoulder singularity: skip
            continue
        tried += 1
        tcp = [0.0, 0.02, 0.12, 0.0, 0.0, 0.3]
        q1 = q0 + rng.uniform(-0.12, 0.12, 6)
        T = kin.fk_frames(q1)[-1]
        R_tcp = T[:3, :3] @ kin.rotvec_to_matrix(tcp[3:6])
        pose = [*(T[:3, :3] @ np.array(tcp[:3]) + T[:3, 3]), *kin.matrix_to_rotvec(R_tcp)]
        sol = kin.ik_near(pose, q0, tcp)
        if sol is None:
            fails += 1
        else:
            worst = max(worst, float(np.max(np.abs(np.array(sol) - q1))))
    check("ik_near recovers 200 nearby joint configurations (TCP offset, away from singularities)", fails == 0 and worst < 1e-3,
          f"{fails} not converged, worst {worst:.1e} rad")
    check("ik_near: pose out of reach -> None", kin.ik_near([-1.5, 0.0, 0.3, *DOWN], Q_URSIM_START) is None)

    rob = FakeRobot(Q_URSIM_START)
    mc = MotionController(rob, get_profile("sim_open"))
    mc.move_l([-0.147, -0.437, 0.30, *pose_of(Q_URSIM_START)[3:]])
    mc.move_j(Q_URSIM_START)
    check("sim_open: moves run and the robot state is never read for a path check", len(rob.moves) == 2 and rob.reads == 0)

    print("\n7. cell selection + taught workspace")
    from ur5e.config import cell_profile_name
    from ur5e.workspace import Workspace
    saved = os.environ.pop("UR5E_CELL", None)
    try:
        check("defaults: sim -> sim_open, real -> cart_360",
              cell_profile_name("sim") == "sim_open" and cell_profile_name("real") == "cart_360")
        os.environ["UR5E_CELL"] = "cart_360"
        check("UR5E_CELL=cart_360 applies to the sim too", cell_profile_name("sim") == "cart_360")
        cell = get_profile("cart_360")
        cx, cy = cell.keepout[0].box.x, cell.keepout[0].box.y
        ya, yb = cy[0] - 0.40, cy[0] - 0.15          # a box beside the cart's -Y side, from below the top to above it
        xa, xb = max(cx[0], -0.35), min(cx[1], -0.05)
        rot = DOWN
        low = [[x, y, -0.30, *rot] for x, y in ((xa, ya), (xb, ya), (xb, yb), (xa, yb))]
        high = [[p[0], p[1], 0.30, *rot] for p in low]
        ws = Workspace.from_points("sim", low, high, [(xa + xb) / 2, (ya + yb) / 2, 0.25, *rot], Q_URSIM_START)
        prof = ws.profile("sim")
        check("workspace taught beside the cart keeps the cell's keep-out and ceilings",
              len(prof.keepout) == len(cell.keepout) and prof.tcp_speed_max == cell.tcp_speed_max and prof.workspace.z[0] == -0.30)
        os.environ["UR5E_CELL"] = "open"
        check("UR5E_CELL=open: sim unaffected", cell_profile_name("sim") == "sim_open")
    finally:
        os.environ.pop("UR5E_CELL", None)
        if saved is not None:
            os.environ["UR5E_CELL"] = saved


# ------------------------------------------------------------------- live ----
def live(reach: bool) -> None:
    from ur5e import Robot

    cell = get_profile("cart_360")
    k = cell.keepout[0]
    cx, cy, top = k.box.x, k.box.y, k.box.z[1]
    print(f"\n8. live against URSim, measured cell: cart {k.describe()}  validated={cell.validated}")
    with Robot(target="sim", profile="cart_360") as robot:
        start = robot.pose()
        rot = start[3:6]
        q_now = robot.joints()
        t0 = time.time()
        q = kin.ik_near(start, q_now, robot.conn.tcp)
        dt = (time.time() - t0) * 1000
        off = max(abs(a - b) for a, b in zip(q, q_now)) if q is not None else float("nan")
        fk_mm = math.dist(kin.fk_tcp_position(q_now, robot.conn.tcp), start[:3]) * 1000
        check("framework IK reproduces the controller's joints at the current pose", q is not None and off < 0.02,
              f"max joint difference {math.degrees(off):.2f} deg, nominal FK vs controller pose {fk_mm:.1f} mm, {dt:.0f} ms")

        x = min(max(-0.30, cx[0] + 0.10), cx[1] - 0.10)
        above = [x, cy[0] + 0.20, top + 0.25, *rot]
        inside = [x, cy[0] + 0.20, top - 0.10, *rot]
        beside_low = [x, cy[0] - 0.25, top - 0.25, *rot]
        beside_high = [x, cy[0] - 0.25, top + 0.25, *rot]

        msg = rejected(lambda: robot.move_to(inside))
        check("target inside the cart: rejected", msg is not None, (msg or "")[:70])
        msg = rejected(lambda: robot.move_to(above))
        here = robot.pose()
        check("move above the cart top executes", msg is None and math.dist(here[:3], above[:3]) < 0.002, (msg or "")[:90])
        msg = rejected(lambda: robot.move_to(beside_low))
        check("from above the top straight to beside-and-below: rejected", msg is not None, (msg or "")[:70])
        check("... and the arm did not move", math.dist(robot.pose()[:3], here[:3]) < 0.001)

        if reach:
            print("\n   --reach: via a waypoint outside the edge, then down (informational)")
            for name, pose in (("out to the side, above the top", beside_high), ("down beside the cart", beside_low),
                               ("back up", beside_high), ("back above the top", above)):
                msg = rejected(lambda: robot.move_to(pose))
                print(f"     {name}: " + ("executed" if msg is None else "REJECTED — " + msg))
                if msg is not None:
                    break
        msg = rejected(lambda: robot.move_to(start))
        print("   back to the start pose: " + ("done" if msg is None else "REJECTED — " + msg))


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--target", choices=["sim", "real"], default=None)
    ap.add_argument("--reach", action="store_true")
    args = ap.parse_args()
    if args.target == "real":
        raise SystemExit("this script moves the arm on its own — simulator only. On the real arm use the "
                         "checklist of teach_cart_cell.py --validate.")
    offline()
    if args.target == "sim":
        live(args.reach)
    bad = results.count(False)
    print(f"\n{'ALL PASS' if not bad else str(bad) + ' FAILED'}  ({len(results)} checks)")
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
