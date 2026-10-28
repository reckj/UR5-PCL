# ur5e framework — research base (experimental / internal)

> **STATUS: experimental / internal. NOT validated for student use.**
> Upstream toolbuilding. Students only ever receive validated, stable framework.

> **Students:** start with [`STUDENT_SETUP.md`](STUDENT_SETUP.md) — Docker is the
> only thing you install.

Host-parameterized RTDE framework for the UR5e. The same code targets URSim or
the real arm — only the host changes (`src/ur5e/config.py`, env-overridable).

## What's here

```
src/ur5e/
  connection.py   # RobotConnection: the sim<->real swap point
  config.py       # named targets: sim / real (env-overridable hosts)
scripts/
  check_connection.py   # receive-only connectivity check (no motion)
  rtde_viewer.py        # browser 3D viewer + scene editor (viser), receive-only
  verify_viewer_fk.py   # checks the viewer's mesh chain against the DH FK
src/ur5e/viewer/        # viewer implementation + vendored UR5e meshes (BSD-3)
scenes/                 # viewer scenes: scenes/<name>/scene.json + models/
toolheads/              # toolhead presets shared by all scenes: toolheads/<id>/toolhead.json + mesh
notebooks/
  01_explore.py   # exploration entry point (VS Code cells / Jupyter)
Dockerfile, docker-compose.yml, entrypoint.sh   # dev environment
```

No motion primitives and no safety scaffolding yet — both deliberately deferred
(safety until a safety model is defined).

## Why a container (not native macOS)

`ur_rtde` 1.6.3 ships no macOS wheel, so pip compiles it from source — and the
source build fails against Homebrew's Boost 1.90 (which made `system`
header-only, breaking `ur_rtde`'s `find_package`). Debian's apt Boost (~1.74) is
the compatible era and configures cleanly (verified). So the dev environment is
a **Debian/arm64 container**, which on Apple Silicon runs natively (no
emulation) and matches a Debian/aarch64 deployment target.

## Setup on the dev Mac

Requires Docker Desktop.

```bash
# 1. Build the dev image (compiles ur_rtde against apt Boost — a few minutes once)
docker compose build

# 2. Start URSim (x86 -> emulated on Apple Silicon; sluggish but works)
docker compose up -d ursim
#    Open http://localhost:6080/vnc.html , press ON then START to init the arm.

# 3. Open a shell in the dev container (framework auto-installed editable)
docker compose run --rm dev bash
```

Inside the container the project is bind-mounted at `/workspace`, so edits in
your Mac editor are live. For an integrated workflow, VS Code's **Dev Containers**
extension can open the folder directly inside the container.

## Verify the connection

```bash
# Inside the dev container, against URSim (URSIM_HOST=ursim is set by compose):
python scripts/check_connection.py --target sim

# Against the real arm: set UR_HOST to its LAN IP (see docker-compose.yml), then:
python scripts/check_connection.py --target real
```

Expected: robot mode, joint angles, TCP pose, then a clean disconnect.
Receive-only — never commands motion.

## Seeing the arm move — viewer + scene editor (experimental)

The default `ur_rtde` control interface uploads a script over the network, which
the UR controller only permits in **Remote Control** mode — and Remote Control
deliberately hides PolyScope's Move tab (its 3D view). So you cannot watch the
arm in PolyScope while driving it via `ur_rtde`.

The framework ships its own browser-based viewer instead, driven by live RTDE
joint state (`getActualQ`). It works in Remote mode and, later, against the real
arm (which has no sim graphics at all). It's receive-only, so it runs alongside
a control script.

```bash
# Start URSim + the viewer:
docker compose up -d ursim viewer
# Open http://localhost:7001/ in your browser (orbit/zoom with the mouse).

# In another shell, run a control script and watch it move in the browser:
docker compose run --rm dev bash
python scripts/smoke_test_move.py
```

The viewer (`scripts/rtde_viewer.py` → `src/ur5e/viewer/`, built on
[viser](https://github.com/nerfstudio-project/viser)) renders the UR5e with its
real link meshes (from Universal Robots' `ur_description`, BSD-3), the Base and
tool0/TCP frames and a metric grid. Around the arm it is a small **simulation
workspace**:

- load your own STL/OBJ/GLB models (table, fixtures, work pieces; mm or m) and
  place them with a drag gizmo or numeric fields,
- attach a toolhead mesh to the flange with a TCP offset in the same numbers
  PolyScope's *Installation → TCP* takes — or pick a preset from the shared
  `toolheads/<id>/` library (mesh + offset + TCP), e.g. `tool_example`,
- draw **safety planes** in the vocabulary of *Installation → Safety → Planes*
  (copy feature, displacement, restriction mode, restrict elbow) so you can type
  them into the pendant 1:1; a plane turns red when TCP/elbow cross it.

Everything is saved as a *scene*: `scenes/<name>/scene.json` plus the model files
next to it, so a scene folder can be committed or AirDropped as one unit.
The viewer starts on the `default` scene; switch scenes in the page (Scene
panel). To start on another scene, put `VIEWER_SCENE=my_table` in a `.env` file
next to `docker-compose.yml` (works on macOS/Windows/Linux). `--no-robot` runs
it without URSim (joint sliders).

> **Nothing in the viewer limits or protects the robot.** Safety planes there
> are a planning sketch; the controller's Safety Configuration is authoritative.
> Status: experimental / internal, not validated for student use. Details and
> the scene file format: `Documentation/11-viewer-scenes-and-safety-planes.md`.

## Student-facing API (`ur5e.robot`)

The growing base-functions surface students build projects on — the only module
they need. It hides the RTDE / safety / motion internals behind a small API:

```python
from ur5e import Robot, grid_poses

with Robot(target="sim") as robot:
    here = robot.pose()
    for p in grid_poses(here, nx=3, ny=3, dx=0.04, dy=0.04):
        robot.move_to(p)                 # validated, slow, sim-gated
        if robot.read_analog_in(0) > 0.5: # sensor
            robot.set_digital_out(0, True)  # actuator
            robot.set_digital_out(0, False)
```

Motion is validated against a safety profile and **sim by default**; the real
arm requires `Robot(target="real", allow_real=True)` with a *validated* profile
and `robot.set_tool(payload_kg, tcp)` declared first.

`scripts/` holds internal/validation tools; `examples/` holds student-facing
example processes. First example: `examples/grid_process.py` — grid traversal
with sensor-gated actuation.

> Status: experimental. This surface will *become* the stable student framework
> once validated on hardware; it is not validated for student use yet.

## Notes for the eventual clean setup

Pin: the base image digest, apt Boost version, `ur_rtde` version, and the URSim
image tag validated on hardware. The compile is reproducible from the Dockerfile.
