# Student-Facing Motion API Design (2026-06-22) 🔬

Brainstormed and decided in session. Not yet implemented — this is the design target for the framework's motion layer.

## Design philosophy

- **Flexibility first** — students go in different directions (geometry, process, interaction); API must not force one path.
- **Scalable complexity** — simple things must be simple; power must be accessible; nothing is hidden or removed as you go deeper.
- **Documentation + examples are as important as the API itself** — students pattern-match from working code.
- **No abstraction lock-in** — all three levels are always available and composable; students can mix them freely.

## Three-layer API

**Level 1 — Raw access** (power users, geometry work, internal research)

Direct pass-through to `ur_rtde`. No validation beyond the safety layer. Full control.

```python
arm.control.moveL([0.3, 0.1, 0.2, 0, 3.14159, 0], speed=0.1, accel=0.5)
arm.control.moveJ([0, -1.57, 1.57, -1.57, -1.57, 0], speed=1.0, accel=1.0)
```

**Level 2 — Motion primitives** (main student interface)

Sane defaults, orientation abstracted, safety validated before execution.

```python
# Cartesian
arm.move_to(x=0.3, y=0.1, z=0.2)                             # tool pointing down implied
arm.move_to(x=0.3, y=0.1, z=0.2, orientation="down")         # named preset
arm.move_to(x=0.3, y=0.1, z=0.2, rx=0, ry=180, rz=45)        # euler degrees

# Joint space
arm.move_joints([0, -90, 90, -90, -90, 0])                   # degrees, not radians

# Path — list of (x,y,z) or (x,y,z,rx,ry,rz) or named waypoints
arm.move_through([(0.3, 0.0, 0.1), (0.3, 0.1, 0.1), (0.3, 0.2, 0.1)])
arm.move_through(path, blend=0.01)                           # opt-in blending
```

**Level 3 — Process primitives** (fabrication-focused; describes the process not the robot)

```python
arm.approach(x=0.3, y=0.1, z=0.05, clearance=0.05)
arm.retract(clearance=0.08)
arm.traverse(path, retract_between=False)
# grid: grid_poses(...) generator + move_through(...) or a manual loop — no separate scan_grid
arm.probe(direction="z-", max_distance=0.05, threshold_n=5.0)
```

## Orientation handling

Axis-angle (raw UR format) hidden by default. Exposed only at Level 1.

```python
from ur5e import Orientation

Orientation.DOWN                                   # tool Z pointing at table — most common
Orientation.FORWARD
Orientation.from_euler(rx=0, ry=180, rz=45)        # degrees
Orientation.from_quaternion(0, 0, 1, 0)
Orientation.from_axis_angle([0, 1, 0], 3.14159)    # raw, Level 1 territory
```

Accepted anywhere an orientation parameter appears.

## Waypoint recording

```python
with arm.freedrive():
    input("Move to position, press Enter")
    arm.record_waypoint("start")

arm.move_to("start")
arm.follow_path(["start", "mid", "end"])
arm.save_waypoints("session.json")
arm.load_waypoints("session.json")
```

## Path blending — decided

- **Default: stop-at-each-waypoint.** Robot comes to a full stop at every point. Required when doing something at each point (sensor read, actuator trigger). Safer, more predictable for students.
- **Opt-in: blend radius.** `arm.move_through(path, blend=0.01)` — smooth continuous motion, slight overshoot near waypoints. For pure traversal where precision at each point doesn't matter.
- Rationale: fabrication context requires sensor-dependent execution at waypoints; stopping is the correct default. Blending is useful for geometric paths with no per-point action.

## Documentation structure (target)

```
docs/
  01_getting_started.md       # connect, home, single move — 10 lines
  02_motion_basics.md         # move_to, move_joints, move_through
  03_orientation.md           # presets, euler, when to care
  04_paths_and_grids.md       # move_through, grid_poses, blending
  05_recording_waypoints.md   # freedrive, record, save/load
  06_process_primitives.md    # approach/retract/traverse, sensor-gated motion
  07_sensor_io.md             # digital/analog read + write
  08_advanced.md              # raw access, real-time loops, servoJ
  examples/
    hello_robot.py            # move to a point and back
    draw_square.py            # follow_path with 4 waypoints
    grid_scan.py              # grid_poses + sensor read at each point
    freedrive_record.py       # record and replay a path
    reactive_motion.py        # move until I/O condition
    custom_geometry.py        # programmatic path generation (numpy)
```

## Next steps for this section

- Implement `Orientation` class and helpers.
- Implement `follow_path` with `blend` opt-in.
- Implement freedrive recording + waypoint save/load.
- Implement Level 3 process primitives (approach/retract/traverse/scan_grid).
- Write example scripts before writing docs (students learn from examples).
- **DECIDED (2026-06-22): student writes the per-point loop manually (Level 2) — no callback as default.** Geometry generation and motion are kept separate: a public `grid_points(...)` (geometry only) returns the list/generator of points; the student loops over it and does sensor read / actuator write themselves with Level 2 calls. This is the transparent, composable primitive. `scan_grid(...)` stays as a thin convenience wrapper over `grid_points` for the pure-traversal case. Any future per-point callback is sugar layered on top — never the only way in. Mirrors the existing `grid_poses`/`line_poses` pattern generators from the 2026-06-16 session.

	```python
	# Default pattern — student owns the loop
	for point in arm.grid_points(origin=(0.15, -0.10, 0.02), width=0.30, depth=0.20, cols=8, rows=6):
	    arm.move_to(*point)
	    value = arm.read_analog_in(0)
	    if value > threshold:
	        arm.set_digital_out(0, True)
	```

## Naming reconciled (2026-06-22) — DECIDED

The 2026-06-22 design sketch and the 2026-06-16 `Robot` facade used different names for the same concepts. Reconciled by keeping the names already implemented in the 06-16 facade (least churn) and aligning the sketch to them. The `move_*` family stays consistent across single/sequence moves.

**Settled namespace:**

- **Geometry generators** (return a list of poses, no motion): `grid_poses(origin, width, depth, cols, rows, order="snake", orientation=...)`, `line_poses(start, end, n, orientation=...)`. Grow this family (`arc_poses`, etc.).
- **Single moves** (Level 2): `move_to(pose | x, y, z, orientation=...)` — accepts a pose sequence OR kwargs; `move_joints([deg×6])`.
- **Sequence move** (Level 2): `move_through(path, blend=None)` — default stop-at-each, `blend` opt-in.
- **Process primitives** (Level 3, add behavior beyond generate+execute): `approach`, `retract`, `traverse(path, retract_between=False)` (builds on `move_through`), `probe`.

**Renames / drops:**

- `follow_path` (06-22) → **`move_through`** (06-16 name kept; already built).
- `grid_points` (06-22) → **`grid_poses`** (06-16 name kept). Generators return full poses, not bare points.
- `scan_grid` (06-22) → **dropped.** Pure traversal is `move_through(grid_poses(...))` — a clean one-liner that needs no separate name once the manual-loop pattern (Option B) is the default.

**Generator/executor split (explicit):** generators *produce* poses; `move_through` *consumes* them. Pure traversal = `move_through(grid_poses(...))`; sensor-gated work = manual loop over `grid_poses(...)`. Requires `move_to` to accept a pose sequence (not only kwargs) so the loop body stays clean: `for pose in arm.grid_poses(...): arm.move_to(pose)`.
