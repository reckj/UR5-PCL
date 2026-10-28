# Viewer as simulation workspace — meshes, scenes, safety planes (2026-10-28) 🔬

**Status: experimental / internal. Not validated for student use; nothing from this
section goes into student docs yet.** Built in the cloud sandbox, verified
offline (FK test + headless browser) and **against URSim on the dev Mac
(2026-10-28, see *Validation* below)**.

Goal: make `scripts/rtde_viewer.py` usable as the students' simulation
workspace — it replaces PolyScope's Move tab, which Remote Control mode hides.
Four features, in order: real UR5e meshes, student scene objects, persistence,
safety planes in PolyScope terms. Visualization only throughout: **no motion
blocking, no safety logic** (the project safety model is still being defined).

## Decision: meshcat → viser 🟡

The meshcat viewer was one-way (Python → browser). Features 2–3 need the
browser to talk back (gizmo drags, numeric fields, load/save, file upload).
Options weighed:

| | meshcat + custom page + sidecar server | **viser** (chosen) | hand-rolled three.js |
|---|---|---|---|
| new deps | none | `viser==1.1.1`, `trimesh==5.2.0` — pure-Python / wheel-only, identical on arm64 + amd64, installed only in the image | none, lots of JS |
| gizmo / GUI inputs | hand-built against meshcat internals | built in (`add_transform_controls`, number/vector/dropdown/upload widgets) | hand-built |
| servers / ports | two | one (HTTP + WS on 7000; compose keeps `7001:7000`) | one |
| DAE mesh loading | unreliable (meshcat-python #84) | via trimesh / GLB | own |

viser is MIT, actively maintained (1.1.1, Sep 2026). The RTDE path is
unchanged: a receive-only thread polls `getActualQ()` (now also
`getActualTCPPose()`), the main thread pushes transforms at 20 Hz. Cost:
students rebuild the image once; the URL loses `/static/`
(`http://localhost:7001/`). Compose, Dockerfile and `pyproject` `viz` extra updated;
`meshcat` dropped.

The viewer now lives in the package (`src/ur5e/viewer/`), with
`scripts/rtde_viewer.py` as a thin CLI, so compose is unchanged:

```
src/ur5e/viewer/
  robot_model.py    UR5e link chain + mesh offsets (ur_description 4.4.0), FK of every frame
  scene.py          scene.json data model, load/save, model files, unit handling
  safety_planes.py  plane pose from feature + displacement, signed distances, colours
  server.py         viser app: 3D scene, GUI folders, gizmos, RTDE thread, render loop
  transforms.py     rpy / rotation-vector / quaternion helpers (numpy only)
  assets/ur5e/      7 GLB meshes + LICENSE (BSD-3) + SOURCE.md
scripts/rtde_viewer.py       CLI: --target/--host/--no-robot, --scene, --rate, --port, --joints
scripts/verify_viewer_fk.py  internal check: mesh chain FK == DH FK
scenes/<name>/               scene.json + models/
```

## 1 — Robot model 🔬

- **Meshes:** Universal Robots `Universal_Robots_ROS2_Description` 4.4.0,
  commit `81f7c9f` (2026-10-22), `meshes/ur5e/visual/*.dae`, **BSD-3-Clause**
  (UR's stricter "graphical documentation" terms apply only to UR20/UR30).
  Converted once, offline, to GLB (transforms applied → metres / Z-up, vertices
  merged, 50 % decimation above 2000 faces, material colours baked): 9.4 MB →
  1.4 MB, bounding boxes identical to 0.1 mm. Provenance + command in
  `assets/ur5e/SOURCE.md`; license file vendored next to the meshes.
- **Chain:** joint origins from `config/ur5e/default_kinematics.yaml`, visual
  offsets from `visual_parameters.yaml`, written out in `robot_model.LINKS`
  (no URDF parser, no yourdfpy dependency). Rooted in the **controller Base
  frame** (= `ur_description`'s `base` link = `base_link` rotated 180° about Z),
  so what the viewer draws is in the frame students read on the pendant.
- **Verification (`scripts/verify_viewer_fk.py`, PASS):** over 2000 random q,
  viewer flange pose vs `ur5e.kinematics` DH FK — position 3e-16 m, rotation
  4e-16 (tool0 == DH frame 6, orientation included). Home pose
  (0,−90,0,−90,0,0) → tool0 (0, −0.2329, 1.0794) m = d1+a2+a3+d5 up, d4+d6
  sideways. DH frames 2/3 sit at the *ends* of the arm links (a2, a3) whereas
  the URDF puts joint frames on the axes; the viewer's **elbow** (joint-3
  origin) therefore equals the DH frame-2 origin.
- Drawn: base frame + label, tool0 axes, TCP frame (red origin, from the
  scene), elbow marker (toggle), 3 × 3 m grid with 0.1 m cells / 1 m lines and
  0.5/1 m labels. Camera up = +Z.

## 2 — Scene objects + toolhead 🔬

- *Objects* folder: upload STL/OBJ/GLB/PLY (stored into the scene's `models/`)
  or add a file already there; per object: visible, **gizmo** (viser transform
  controls, synced both ways with the fields), position [mm], rotation RPY
  [deg], colour, opacity, remove. Units are chosen per file (`mm` default — the
  usual CAD export; `m`); after adding, the message line shows the bounding box
  in mm so a 1000× mistake is obvious.
- *Toolhead + TCP* folder: TCP position [mm] and rotation vector [rad] in the
  **tool0 frame** — the same numbers PolyScope *Installation → General → TCP*
  takes (default representation); RPY [deg] shown read-only for the alternate
  pendant setting. Optional toolhead mesh attached to tool0 with its own
  offset. Payload is stored for the record only.
- **Toolhead library (added 2026-10-28):** presets shared by all scenes in
  `toolheads/<id>/toolhead.json` (format `ur5e-toolhead/1`: mesh file name,
  units, mesh offset in tool0, TCP, payload, notes) with the mesh next to it.
  *Toolhead preset → Use selected preset* copies the mesh into the scene's
  `models/` (scene stays self-contained) and fills the fields; the scene records
  `"library": "<id>"` until the fields are edited by hand. *Save current
  toolhead to library* does the reverse. First preset: `tool_example` (John's
  example tool: STL in metres along +Y → roll 90° onto tool0 Z; 80 mm UR-pattern
  flange plate, probe to 108 mm, TCP preset at the probe tip (0, 21, 108) mm —
  **tip assumed from geometry, payload unknown (0 = placeholder)**).
- **TCP check:** the Robot folder shows the controller's `getActualTCPPose()`
  next to FK(q)∘scene-TCP and flags a mismatch > 3 mm — a receive-only way to
  notice that the pendant's active TCP is not the one in the scene (the viewer
  cannot read the TCP offset over RTDE receive; `getTCPOffset` needs the
  control interface).

## 3 — Persistence 🔬

`scenes/<name>/scene.json` + `models/` next to it (relative paths) → one
folder is the whole scene; commit / zip / AirDrop it. `--scene <name>` CLI flag
(compose: `VIEWER_SCENE=<name> docker compose up -d viewer`, default
`default`, created empty if missing) and in-page *Load / Save / Save as / New /
Reload*; *Save as* copies `models/`. JSON via stdlib, atomic write
(`.tmp` + rename), validated on load with a clear `SceneError` (unknown mode,
missing feature, bad units). Format `ur5e-scene/1`:

```jsonc
{
  "format": "ur5e-scene/1",
  "name": "example_cart",
  "description": "free text",
  "units_note": "lengths in metres, angles in degrees unless the key says _rad; frames in robot Base",
  "robot": { "model": "ur5e", "mount": { "xyz": [0,0,0], "rpy_deg": [0,0,0] } },   // for later wall/truss mounts; identity now
  "toolhead": {
    "mesh": "models/rgb_toolhead.stl", "mesh_units": "mm",
    "mesh_offset": { "xyz": [0,0,0], "rpy_deg": [0,0,0] },                          // mesh frame in tool0
    "tcp": { "xyz": [0,0,0.085], "rotvec_rad": [0,0,0] },                             // = PolyScope TCP (m, rad)
    "payload_kg": 0.35, "color": "#d0d0d0",
    "library": "tool_example"                                                       // preset it came from, or null
  },
  "objects": [
    { "id": "table_top", "mesh": "models/table_top.stl", "mesh_units": "mm",
      "pose": { "xyz": [0.30,0,-0.01], "rpy_deg": [0,0,0] },                         // in Base
      "color": "#8a7a5a", "opacity": 1.0, "visible": true }
  ],
  "features": [                                                                      // PolyScope plane features
    { "id": "wall", "type": "plane", "pose": { "xyz": [-0.2,0,0], "rpy_deg": [0,90,0] } }
  ],
  "safety_planes": [
    { "name": "wall", "feature": "wall", "displacement": 0.0, "mode": "normal",
      "restrict_elbow": true, "size": [1.5,1.5], "visible": true },
    { "name": "table_top", "feature": "base", "displacement": 0.0, "mode": "normal",
      "restrict_elbow": false, "size": [1.6,1.6], "visible": true }
  ],
  "view": { "camera_position": [1.7,-1.7,1.2], "camera_target": [0.2,0,0.3] }       // optional
}
```

RPY = PolyScope's RPY (R = Rz(yaw)·Ry(pitch)·Rx(roll)). `size` is render-only.
Shipped scenes: `default` (empty) and `example_cart` (illustrative 600 × 500 mm
table top at +X, top at z = 0, wall feature behind the robot — **placement
assumed, not measured**; it is an example of the format, not the cell).

## 4 — Safety planes (display only) 🔬

Modelled on UR5e User Manual §21.11 (checked, not from memory): up to 8
planes; each copies a **feature** (Base or an Installation feature), the
feature's **Z axis is the normal**, *displacement* shifts the plane along it;
restriction modes **Disabled / Normal / Reduced / Normal & Reduced / Trigger
Reduced Mode**; per-plane **Restrict Elbow** checkbox; the arrow on the pendant
marks the side the tool/elbow is allowed on. The GUI uses exactly those words so
a plane can be re-entered 1:1; features are created in the *Features* folder
(numeric pose or gizmo), with the rotation vector shown for the pendant's
feature editor.

Rendering: translucent rectangle + normal arrow + label, colour by mode
(yellow normal, orange reduced, cyan trigger, grey disabled). **Stretch goal
done:** signed distance of the TCP and (if Restrict Elbow) the elbow to each
plane; a strict plane turns **red** when crossed, a trigger plane purple
("would trigger Reduced mode"); per-plane status text in the GUI. Nothing is
enforced, and the code says so in every file header.

## Assumptions flagged

- **Allowed side = +Z of the shifted feature frame** (arrow = normal = allowed
  side, per manual). To confirm against the pendant's 3D view in the lab; one
  constant in `safety_planes.py` if the sign is the other way.
- Nominal UR5e geometry, no per-robot calibration (fine for visualization; the
  TCP check tolerance of 3 mm leaves room for calibration offsets).
- Elbow point = joint-3 origin (PolyScope "elbow joint").
- The TCP shown is the **scene's** TCP; the controller's active TCP is only
  compared, never read.
- Payload is not modelled. Workspace/mount: Base frame = world; `robot.mount`
  exists in the format for the later truss/wall mount but is identity.
- Scene file paths are resolved against `./scenes` (= `/workspace/scenes` in
  the container); `UR5E_SCENES_DIR` overrides.

## Validation

Sandbox (no robot): FK test PASS; headless-Chromium renders at home and bent
poses — meshes continuous at every joint, UR colours, grid/labels/planes
correct, plane turns red when the TCP is beyond the wall; functional test of
upload → add → move → feature → plane → save-as → reload → remove → invalid
file rejected → RTDE thread survives an unreachable host.

**Dev Mac, URSim (2026-10-28):** `docker compose build` + `up -d ursim viewer`
→ page at `http://localhost:7001/`:

- RTDE connected at 20 Hz; **controller `getActualTCPPose()` == viewer
  FK(q)∘TCP to 0.0 mm and in rotation vector** → the mesh chain is in the
  controller Base frame, confirmed empirically (not just by the DH comparison).
- `example_cart` loads in the page; plane status lines agree with the
  controller numbers (TCP x = −144 vs wall at −200 → "+56 mm ✓", z = 202 →
  table "+202 mm ✓").
- `scripts/smoke_test_move.py` (control client in a second container):
  **arm moves live in the viewer**.
- TCP mismatch badge fired when the controller's active TCP changed to 200 mm
  along tool Z while the scene TCP was zero ("⚠ Δ 200.0 mm").

**Learning — stale RTDE receive stream.** On the first attempt the viewer
showed "connected" but the pose never changed while the smoke test moved the
arm. The viewer had connected while URSim was still booting (ur_rtde printed
`connect: Connection refused` / `End of file`), and ur_rtde's
`RTDEReceiveInterface` keeps returning its *last* values when the stream
stops instead of raising. Fix in `server.rtde_thread`: a watchdog on the robot
timestamp (`getTimestamp()`) + `isConnected()` — no new timestamp for 1.5 s
→ drop the connection and reconnect; the Connection line shows
"last update N s ago, robot t=…" so staleness is visible. Also
`PYTHONUNBUFFERED=1` on the compose service, otherwise the viewer's own
messages never reach `docker compose logs viewer`.

**Still to check in the lab:** plane sign convention against a pendant plane
(arrow = allowed side), gizmo + upload in Safari/Chrome on the student
laptops, `scenes/` file ownership on the bind mount (files are written from
the container), Windows/amd64 build of the image. Then decide what becomes
student-facing.
