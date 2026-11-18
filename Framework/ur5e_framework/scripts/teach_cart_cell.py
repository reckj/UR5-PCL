"""Measure the cart cell (`cart_360`) — INSTRUCTOR tool, receive-only.

STATUS: internal (not synced to the student repo). Never moves the arm.

`cart_360` = the arm on the free-standing cart, allowed to reach around and
below the cart top. Everything derives from five numbers in the robot Base
frame: the cart footprint (x low/high, y low/high), the height of the cart top
(z) and the height of the floor (z). This script captures them and writes

    src/ur5e/cells/cart_360.json      what safety.py builds the profile from
    scenes/cart_360/                  the same cell for the viewer (cart, handle, plate)

and prints what the pendant installation needs.

Ways to get the numbers
    touch   (default) with a POINTED tool mounted and its TCP active on the
            pendant: guide the tip (freedrive button) to the four top corners
            of the cart, then to the floor. At any prompt you may instead type
            the point in mm ("x y" or "x y z") — for corners the arm cannot reach.
            For the floor you may type the cart height in mm ("height 850").
    --set   typed numbers from a tape measure, in metres (distances from the
            centre of the robot base; front of the lab cart = -X).

Usage (dev shell; real arm: -e UR_HOST=192.168.1.11)
    python scripts/teach_cart_cell.py --target real
    python scripts/teach_cart_cell.py --set -0.56 0.20 -0.32 0.32 --height 0.85
    python scripts/teach_cart_cell.py --show        print the cell + pendant values
    python scripts/teach_cart_cell.py --validate    after the checklist: unlock real-arm motion

Optional, with any of the above
    --side-clearance 0.05   distance the tool keeps from the cart sides (m)
    --arm-radius 0.06       distance the arm links keep from the cart (m)
    --flange-floor 0.25     only if the pendant gets a floor plane again: its height
                            above the floor (m); it limits the FLANGE (cart_360: none)
    --reach X0 X1 Y0 Y1     outer box for the TCP (m); shrink a side when a wall
                            or the visitor line is closer than the arm's reach
    --z-max 1.0             highest TCP height (m)

Afterwards: check in the viewer (scene cart_360), enter the planes on the
pendant, run scripts/verify_cart_cell.py, then --validate, sync, commit.
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CELL_FILE = ROOT / "src" / "ur5e" / "cells" / "cart_360.json"
SCENE_DIR = ROOT / "scenes" / "cart_360"



# ------------------------------------------------------------------ cell file
def read_cell() -> dict:
    return json.loads(CELL_FILE.read_text())


def write_cell(d: dict) -> None:
    tmp = CELL_FILE.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(d, indent=2) + "\n")
    tmp.replace(CELL_FILE)


# --------------------------------------------------------------- viewer scene
def _box_stl(x, y, z) -> str:
    """ASCII STL of an axis-aligned box, coordinates in metres (Base frame)."""
    v = [(x[i], y[j], z[k]) for i in (0, 1) for j in (0, 1) for k in (0, 1)]
    quads = [((0, 1, 3, 2), (-1, 0, 0)), ((4, 6, 7, 5), (1, 0, 0)), ((0, 4, 5, 1), (0, -1, 0)),
             ((2, 3, 7, 6), (0, 1, 0)), ((0, 2, 6, 4), (0, 0, -1)), ((1, 5, 7, 3), (0, 0, 1))]
    out = ["solid cart"]
    for (a, b, c, d), n in quads:
        for tri in ((a, b, c), (a, c, d)):
            out.append(f" facet normal {n[0]} {n[1]} {n[2]}\n  outer loop")
            out += [f"   vertex {v[i][0]:.4f} {v[i][1]:.4f} {v[i][2]:.4f}" for i in tri]
            out.append("  endloop\n endfacet")
    out.append("endsolid cart")
    return "\n".join(out) + "\n"


def planes(d: dict) -> list:
    """The planes of the cart installation, as (name, displacement m, mode, pendant mode text).
    Only the floor: no Reduced-mode trigger below the cart top (decided 2026-11-18 —
    groups work beside the cart below its top and should not run slowed down there)."""
    if d.get("flange_floor_clearance") is None:      # cart_360: no planes on the pendant (JReck, 2026-11-18)
        return []
    floor = d["floor_z"] + d["flange_floor_clearance"]               # limits the FLANGE (default Tool Position)
    return [("floor", floor, "normal", "Normal")]


def write_scene(d: dict) -> None:
    cart = d["cart"]
    (SCENE_DIR / "models").mkdir(parents=True, exist_ok=True)
    (SCENE_DIR / "models" / "cart.stl").write_text(
        _box_stl(cart["x"], cart["y"], (d["floor_z"], cart["top_z"])))
    extras = []
    for e in d.get("extras", []):
        fname = "".join(ch if ch.isalnum() else "_" for ch in e["name"]) + ".stl"
        (SCENE_DIR / "models" / fname).write_text(_box_stl(e["x"], e["y"], e["z"]))
        extras.append({"id": fname[:-4], "mesh": f"models/{fname}", "mesh_units": "m",
                       "pose": {"xyz": [0.0, 0.0, 0.0], "rpy_deg": [0.0, 0.0, 0.0]},
                       "color": "#4a5058", "opacity": 0.95, "visible": True})
    state = "measured " + d["measured_at"] if d.get("measured_at") else "PLACEHOLDER numbers, not measured"
    scene = {
        "format": "ur5e-scene/1",
        "name": "cart_360",
        "description": (f"Lab cell 'cart_360' ({state}): the cart the robot stands on, with its "
                        f"handle and the robot's mounting plate. Generated by "
                        f"scripts/teach_cart_cell.py from src/ur5e/cells/cart_360.json - edit there, "
                        f"not here. Drawn only; nothing in the viewer limits the robot."),
        "units_note": "lengths in metres, angles in degrees unless the key says _rad; frames in robot Base",
        "robot": {"model": "ur5e", "mount": {"xyz": [0.0, 0.0, 0.0], "rpy_deg": [0.0, 0.0, 0.0]}},
        "toolhead": {"mesh": None, "mesh_units": "mm",
                     "mesh_offset": {"xyz": [0.0, 0.0, 0.0], "rpy_deg": [0.0, 0.0, 0.0]},
                     "tcp": {"xyz": [0.0, 0.0, 0.0], "rotvec_rad": [0.0, 0.0, 0.0]},
                     "payload_kg": 0.0, "color": "#d0d0d0"},
        "objects": [{"id": "cart", "mesh": "models/cart.stl", "mesh_units": "m",
                     "pose": {"xyz": [0.0, 0.0, 0.0], "rpy_deg": [0.0, 0.0, 0.0]},
                     "color": "#6f7780", "opacity": 0.85, "visible": True}] + extras,
        "features": [],
        "safety_planes": [{"name": name, "feature": "base", "displacement": round(disp, 4), "mode": mode,
                           "restrict_elbow": True, "size": [2.4, 2.4], "visible": True}
                          for name, disp, mode, _ in planes(d)],
        "view": {"camera_position": [1.9, -1.9, 0.9], "camera_target": [0.0, 0.0, -0.2]},
    }
    (SCENE_DIR / "scene.json").write_text(json.dumps(scene, indent=2) + "\n")


# --------------------------------------------------------------------- report
def report(d: dict) -> str:
    cart, reach = d["cart"], d["reach"]
    mm = lambda v: f"{v * 1000:+.0f}"
    w = [f"cell '{d['name']}'   validated: {d['validated']}   "
         f"{'measured ' + d['measured_at'] if d.get('measured_at') else 'NOT MEASURED (placeholder numbers)'}",
         f"  cart footprint   x {mm(cart['x'][0])} .. {mm(cart['x'][1])} mm    y {mm(cart['y'][0])} .. {mm(cart['y'][1])} mm"
         f"    ({(cart['x'][1] - cart['x'][0]) * 1000:.0f} x {(cart['y'][1] - cart['y'][0]) * 1000:.0f} mm)",
         f"  cart top         z {mm(cart['top_z'])} mm      floor  z {mm(d['floor_z'])} mm"
         f"    (cart height {(cart['top_z'] - d['floor_z']) * 1000:.0f} mm)",
         *[f"  + {e['name']:<14} x {mm(e['x'][0])} .. {mm(e['x'][1])}   y {mm(e['y'][0])} .. {mm(e['y'][1])}"
           f"   z {mm(e['z'][0])} .. {mm(e['z'][1])} mm" for e in d.get("extras", [])],
         f"  software         tool keeps {d['side_clearance'] * 1000:.0f} mm from the cart sides, arm links "
         f"{d['arm_radius'] * 1000:.0f} mm;",
         f"                   above the floor: TCP {d['floor_clearance'] * 1000:.0f} mm, arm links clear of it"
         + (f", flange {d['flange_floor_clearance'] * 1000:.0f} mm (= the pendant's floor plane)"
            if d.get("flange_floor_clearance") is not None else ""),
         f"  TCP outer box    x {mm(reach['x'][0])} .. {mm(reach['x'][1])}   y {mm(reach['y'][0])} .. {mm(reach['y'][1])}"
         f"   z up to {mm(reach['z_max'])} mm",
         "",
         "PENDANT — Installation > Safety > Planes:"]
    if not planes(d):
        w.append("  NO planes: delete (or set to Disabled) the wall and table planes of the old installation.\n"
                 "  Robot Limits stay on the most-restricted preset. Cart and floor are guarded by the\n"
                 "  framework's checks only.")
    for name, disp, _, text in planes(d):
        w.append(f"  {name:<15} {abs(disp) * 1000:.0f} mm {'BELOW' if disp < 0 else 'ABOVE'} the robot's mounting plane   "
                 f"{text:<8} Restrict Elbow ON   arrow UP")
    if planes(d):
        w.append("  Displacement field: enter that distance and CHECK THE 3D VIEW — the plane must show below the\n"
                 "  robot with the arrow pointing up. The sign the pendant wants is not confirmed for 5.9.1: if the\n"
                 "  plane shows above the robot, flip the sign.")
    w += ["  full procedure: Documentation/16-cart-360-safety-setup.md"]
    if d.get("note"):
        w.append(f"\nnote: {d['note']}")
    return "\n".join(w)


# ---------------------------------------------------------------------- teach
def _typed_point(text: str):
    """'x y' or 'x y z' in mm -> metres list, else None."""
    try:
        vals = [float(t) / 1000.0 for t in text.replace(",", " ").split()]
    except ValueError:
        return None
    return vals if len(vals) in (2, 3) else None


def teach(target: str) -> dict:
    from ur5e.config import resolve_host
    from ur5e.connection import RobotConnection

    host = resolve_host(target)
    print(f"connecting (receive-only) to {host} ...")
    corners, top_zs = [], []
    with RobotConnection(host) as r:
        print("\nPointed tool mounted, its TCP active on the pendant? The numbers are TCP positions.\n"
              "Guide the tip to each point (freedrive button; sim: Move tab), then press Enter.\n"
              "Or type the point in mm instead of moving:  x y   /   x y z\n")
        for i in range(4):
            s = input(f"[{i + 1}/5] cart TOP corner {i + 1} — tip on the corner of the cart top: ").strip()
            p = _typed_point(s) if s else list(r.receive.getActualTCPPose())[:3]
            if p is None:
                raise SystemExit(f"could not read {s!r} — expected nothing (record) or 'x y [z]' in mm")
            corners.append(p[:2])
            if len(p) == 3:
                top_zs.append(p[2])
            print(f"      corner {i + 1}: x {p[0] * 1000:+.0f}  y {p[1] * 1000:+.0f}"
                  + (f"  z {p[2] * 1000:+.0f}" if len(p) == 3 else "  (no z)") + " mm\n")
        if not top_zs:
            raise SystemExit("no corner had a z value — touch at least one corner with the arm (or type x y z)")
        top_z = max(top_zs)
        s = input("[5/5] FLOOR — tip on the floor next to the cart, then Enter; or type 'height <mm>': ").strip().lower()
        if s.startswith("height"):
            floor_z = top_z - float(s.split()[1]) / 1000.0
        elif s:
            raise SystemExit(f"could not read {s!r} — expected nothing (record) or 'height <mm>'")
        else:
            floor_z = list(r.receive.getActualTCPPose())[2]
    xs, ys = sorted(c[0] for c in corners), sorted(c[1] for c in corners)
    # bounding box of the corners = the keep-out can only be larger than the cart
    skew = max(xs[1] - xs[0], xs[3] - xs[2], ys[1] - ys[0], ys[3] - ys[2])
    if skew > 0.02:
        print(f"\nnote: the corners are up to {skew * 1000:.0f} mm off a rectangle aligned with the robot's "
              f"x/y — the cart is rotated against the base or a corner was missed. The keep-out is the "
              f"bounding box (larger than the cart).")
    if max(top_zs) - min(top_zs) > 0.01:
        print(f"note: top corners differ by {(max(top_zs) - min(top_zs)) * 1000:.0f} mm in z; using the highest.")
    return {"x": [xs[0], xs[3]], "y": [ys[0], ys[3]], "top_z": top_z, "floor_z": floor_z, "how": f"touch ({target})"}


# ----------------------------------------------------------------------- main
CHECKLIST = """Before validating — all of these on the REAL cart:
  [ ] the numbers above come from the real cart (touched or tape-measured), not the placeholders
  [ ] viewer, scene 'cart_360', connected to the real arm: the grey box sits where the cart is
      (touch a cart corner with the tool tip — the TCP frame is on the box corner)
  [ ] pendant: installation 'cart_360' is loaded and applied; old wall/table planes gone (no planes);
      limits still on the most-restrictive preset; the safety checksum is noted
  [ ] freedrive below the cart top and towards the floor: moves freely (no planes any more)
  [ ] scripts/verify_cart_cell.py passes offline and with --target sim
"""


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--target", choices=["sim", "real"], default=None, help="touch-teach on this target")
    ap.add_argument("--set", type=float, nargs=4, metavar=("X0", "X1", "Y0", "Y1"),
                    help="cart footprint in metres, Base frame (typed instead of touched)")
    ap.add_argument("--height", type=float, help="with --set: cart height floor -> top in metres")
    ap.add_argument("--top-z", type=float, default=0.0, help="with --set: z of the cart top (default 0 = mounting plane)")
    ap.add_argument("--side-clearance", type=float)
    ap.add_argument("--arm-radius", type=float)
    ap.add_argument("--flange-floor", type=float, help="height of the pendant's floor plane above the floor (m)")
    ap.add_argument("--reach", type=float, nargs=4, metavar=("X0", "X1", "Y0", "Y1"))
    ap.add_argument("--z-max", type=float)
    ap.add_argument("--show", action="store_true")
    ap.add_argument("--validate", action="store_true")
    args = ap.parse_args()

    d = read_cell()
    measured = None
    if args.set:
        if args.height is None:
            raise SystemExit("--set needs --height (cart height in metres)")
        x0, x1, y0, y1 = args.set
        measured = {"x": sorted([x0, x1]), "y": sorted([y0, y1]), "top_z": args.top_z,
                    "floor_z": args.top_z - args.height, "how": "typed (--set)"}
    elif args.target:
        measured = teach(args.target)

    changed = False
    if measured:
        if not (0.3 < measured["top_z"] - measured["floor_z"] < 1.5):
            raise SystemExit(f"cart height {(measured['top_z'] - measured['floor_z']) * 1000:.0f} mm is not plausible — nothing written")
        if not (measured["x"][0] < 0 < measured["x"][1] and measured["y"][0] < 0 < measured["y"][1]):
            print("!! the robot base (0, 0) is not inside the footprint — check the numbers "
                  "(the robot stands ON the cart).")
        d["cart"] = {"x": [round(v, 4) for v in measured["x"]], "y": [round(v, 4) for v in measured["y"]],
                     "top_z": round(measured["top_z"], 4)}
        d["floor_z"] = round(measured["floor_z"], 4)
        d["measured_at"] = time.strftime("%Y-%m-%d %H:%M")
        d["measured_with"] = measured["how"]
        d["note"] = ""
        d["validated"] = False          # new geometry always starts unvalidated
        if d.get("extras"):
            print("!! 'extras' (handle, plate ...) in the cell file were kept as they are — check that "
                  "they still fit the new cart numbers (edit src/ur5e/cells/cart_360.json).")
        changed = True
    for key, val in (("side_clearance", args.side_clearance), ("arm_radius", args.arm_radius),
                     ("flange_floor_clearance", args.flange_floor)):
        if val is not None:
            d[key] = val
            changed = True
    if args.reach:
        d["reach"]["x"], d["reach"]["y"] = sorted(args.reach[:2]), sorted(args.reach[2:])
        changed = True
    if args.z_max is not None:
        d["reach"]["z_max"] = args.z_max
        changed = True

    if args.validate:
        print(report(d), "\n")
        if not d.get("measured_at"):
            raise SystemExit("the cell still holds placeholder numbers — measure first.")
        print(CHECKLIST)
        if input("All done? Type 'validated' to unlock real-arm motion for this cell: ").strip().lower() != "validated":
            raise SystemExit("not validated — nothing changed.")
        d["validated"] = True
        changed = True

    if changed:
        write_cell(d)
        write_scene(d)
        print(f"\nwritten: {CELL_FILE.relative_to(ROOT)}  +  {SCENE_DIR.relative_to(ROOT)}/\n")
    elif not args.show:
        ap.print_help()
        return
    print(report(d))
    if changed and not d["validated"]:
        print("\nnext: viewer scene 'cart_360' -> pendant planes -> verify_cart_cell.py -> --validate")


if __name__ == "__main__":
    main()
