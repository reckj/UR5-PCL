"""Capture the real workspace box by freedrive — REAL ARM, supervised.

Hand-guide the TCP to the corners of the working area (and your highest working
point); records the live poses and prints the exact Box(...) to paste into the
cart_against_wall profile. This removes all guesswork about base-frame placement
and which direction is "front".

Preconditions: arm ON, brakes released, Remote Control; UR_HOST set; hand near
the e-stop. Freedrive lets you move the arm by hand.

Usage:
    docker compose run --rm -e UR_HOST=192.168.x.x dev bash
    python scripts/teach_workspace.py
"""

from __future__ import annotations

from ur5e.connection import RobotConnection
from ur5e.config import resolve_host, is_sim


def main() -> None:
    host = resolve_host("real")
    if is_sim(host):
        raise SystemExit("UR_HOST is not set to a real arm.")

    pts = []
    with RobotConnection(host, control=True) as r:
        if not r.connected:
            raise SystemExit("control not connected; arm ON + Remote Control?")

        print("FREEDRIVE ON — move the TCP by hand.")
        print("Record: the 4 corners of the work area at the surface, then your")
        print("highest working point. Press Enter to record each; type 'done' to finish.\n")
        r.control.teachMode()
        try:
            while True:
                s = input(f"[{len(pts)} captured] Enter=record / 'done': ").strip().lower()
                if s in ("done", "q", "quit"):
                    break
                p = list(r.receive.getActualTCPPose())
                pts.append(p)
                print("   recorded:", [round(v, 4) for v in p[:3]])
        finally:
            r.control.endTeachMode()

    if len(pts) < 2:
        raise SystemExit("need at least 2 points to form a box.")

    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    zs = [p[2] for p in pts]
    b = (round(min(xs), 4), round(max(xs), 4),
         round(min(ys), 4), round(max(ys), 4),
         round(min(zs), 4), round(max(zs), 4))
    print(f"\ncaptured {len(pts)} points.")
    print("Paste into safety.py 'cart_against_wall', then set validated=True:\n")
    print(f"        workspace=Box(x=({b[0]}, {b[1]}), y=({b[2]}, {b[3]}), z=({b[4]}, {b[5]})),")


if __name__ == "__main__":
    main()
