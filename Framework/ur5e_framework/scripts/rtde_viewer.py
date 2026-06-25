"""Live RTDE 3D viewer (meshcat) for the UR5e — SIM or REAL.

STATUS: experimental / internal.

Renders the arm in a browser from live RTDE joint state (getActualQ), so you can
SEE motion even in Remote Control mode (where PolyScope hides the 3D view), and
later against the real arm (which has no sim graphics at all). Receive-only: it
never commands motion, and multiple RTDE receive clients are fine — run this in
one shell while a control script runs in another.

It draws the kinematic skeleton from UR5e DH parameters (link segments + joint
markers), not a CAD mesh. Enough to see pose and motion clearly. Orbit/zoom with
the mouse in the browser.

Open in the Mac browser:  http://localhost:7001/static/

Usage (inside the dev container, URSim running):
    python scripts/rtde_viewer.py --target sim
"""

from __future__ import annotations

import argparse
import asyncio
import math
import threading
import time

import numpy as np
from meshcat import Visualizer
import meshcat.geometry as g
import meshcat.transformations as tf
from meshcat.servers.zmqserver import ZMQWebSocketBridge

from ur5e.config import resolve_host
from ur5e.connection import RobotConnection
from ur5e.kinematics import fk_origins


def start_bridge(host="0.0.0.0"):
    """Start a meshcat web server bound to all interfaces (for host browser
    access through the published port). Returns its zmq url."""
    urls = {}
    ready = threading.Event()

    def run():
        asyncio.set_event_loop(asyncio.new_event_loop())
        bridge = ZMQWebSocketBridge(host=host)
        urls["zmq"] = bridge.zmq_url
        ready.set()
        bridge.run()

    threading.Thread(target=run, daemon=True).start()
    ready.wait(5)
    return urls["zmq"]


def draw(vis, q):
    pts = fk_origins(q)
    vis["arm"].set_object(g.Line(g.PointsGeometry(pts),
                                 g.LineBasicMaterial(color=0x1f77b4, linewidth=3)))
    for i in range(pts.shape[1]):
        node = vis[f"j{i}"]
        node.set_object(g.Sphere(0.03 if i < pts.shape[1] - 1 else 0.04))
        node.set_transform(tf.translation_matrix(pts[:, i]))


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    g_ = ap.add_mutually_exclusive_group(required=True)
    g_.add_argument("--target", choices=["sim", "real"])
    g_.add_argument("--host")
    ap.add_argument("--rate", type=float, default=20.0, help="update Hz")
    args = ap.parse_args()

    zmq_url = start_bridge()
    vis = Visualizer(zmq_url=zmq_url)
    print("viewer ready -> open http://localhost:7001/static/ in your browser")

    host = args.host or resolve_host(args.target)
    period = 1.0 / args.rate
    print(f"streaming joint state from {host} (Ctrl-C to stop) ...")
    with RobotConnection(host) as r:
        try:
            while True:
                draw(vis, r.receive.getActualQ())
                time.sleep(period)
        except KeyboardInterrupt:
            print("\nstopped.")


if __name__ == "__main__":
    main()
