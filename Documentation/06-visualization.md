# Visualization (2026-06-16)

- **Constraint:** the default `ur_rtde` control interface uploads a script over the network, which the UR controller permits only in **Remote Control** mode (per UR5e manual §29.4.5: Local Control disallows receiving/executing programs over the network). Remote Control deliberately hides PolyScope's Move tab and its 3D view — so the arm can't be watched in PolyScope while driving it via `ur_rtde`.
- **Decision:** the framework ships its own **browser-based 3D viewer** (`scripts/rtde_viewer.py`, meshcat) driven by live RTDE `getActualQ` — receive-only, runs alongside a control script, works in Remote mode and (later) against the real arm, which has no sim graphics at all. Renders a DH-based kinematic skeleton (validated: zero-config reach 0.852 m ≈ UR5e 850 mm spec). Served on `:7000`; added as a `viewer` compose service. Control smoke test (`scripts/smoke_test_move.py`) passes against URSim.
- Note: PolyScope's native 3D is only available in Local mode (no external script control) or via the ExternalControl URCap + Play workflow — not pursued; the RTDE-driven viewer is more general and also covers the real arm.

## Update 2026-10-28 — viewer rebuilt on viser as a simulation workspace 🔬

The meshcat skeleton viewer has been replaced by a viser-based viewer with real
UR5e link meshes, student scene objects (STL/OBJ with gizmo placement), a
toolhead + TCP offset, scene persistence (`scenes/<name>/scene.json`) and
PolyScope-style safety planes (display only). Same receive-only RTDE path,
same compose service, URL now `http://localhost:7001/`. Rationale, scene file
format and open validation items: [11-viewer-scenes-and-safety-planes.md](11-viewer-scenes-and-safety-planes.md).
