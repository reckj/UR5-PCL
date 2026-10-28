# Function templates + taught workspace (2026-10-28, late) 🟢

**Status: 🟢 validated for student use in simulation** (URSim 5.9.4, every
template run on both a `--demo` and an interactively taught box; the taught
flow driven end-to-end with a second control client moving the arm between
the four Enter presses). Real-arm path not yet exercised.

## What was added

- `src/ur5e/workspace.py` — `Workspace`: the four taught poses (+ home joint
  angles), `box` (bounding box of all four points), `pose(u, v, w)` in
  fractions 0…1 of the box (clamped onto the edge for u/v/w = 0/1),
  `grid(nx, ny, w, margin)`, `line(a_uvw, b_uvw, n)`, `profile(target)` →
  `SafetyLimits`, `save()`/`load()` (`workspace.json`, `$UR5E_WORKSPACE`),
  `from_points()` with validation (≥ 20 mm per axis, top above surface),
  `demo_sim_workspace()` (300 × 300 × 200 mm around the sim start pose).
  Format `ur5e-workspace/1`, JSON, metres / rotation vector rad / Base frame.
- `Robot(profile=<Workspace>)` — two lines in `robot.py`: a Workspace passed
  as `profile` becomes `ws.profile(target)`. `Workspace` exported from `ur5e`.
- `examples/function_templates/` — `README.md` + ten scripts, one API call
  each: `00_teach_workspace` (4 steps, receive-only), `01_read_state`,
  `02_move_joints` (moveJ), `03_move_to` (moveL, speed/accel), `04_move_through`
  + `ws.line`, `05_grid_poses` (student owns the loop), `06_digital_io`,
  `07_analog_io`, `08_safety_rejections` (`SafetyViolation` caught and
  printed), `09_raw_rtde` (Level 1, pinned to sim).
- Student repo: `.gitignore` → `workspace*.json`; USAGE §3 + README rewritten
  around the workspace; sync script copies the whole templates folder.

## Decisions

- **Teach receive-only.** `00_teach_workspace.py` opens only the receive
  interface; the student moves the arm (sim: pendant Move tab, works in Local
  and Remote; real: freedrive button on the pendant). No `teachMode()` over
  RTDE, so the same four steps work identically in sim and lab, with no
  control connection. The internal `scripts/teach_workspace.py` stays as is.
- **Four points, not three:** corner A + corner B (surface diagonal → x/y
  extents + surface z), top (z max), **home** (pose + joints). Every template
  does `move_joints(ws.home_joints)` first and last, so it is runnable from any
  starting configuration. The box is the bounding box of all four points, so
  home is always inside it.
- **Orientation = the one taught at corner A.** Templates never invent a tool
  orientation (there is no `Orientation` class yet).
- **Real arm: taught box ∩ `cart_against_wall`**, speed ceilings and
  `validated=True` inherited from the cell profile. Students can only shrink
  the space. A box taught on `sim` is refused for `real` (and vice versa) by
  `profile(target)`. No new safety rules; the existing gate (allow_real +
  validated + set_tool) is unchanged.
- **Demo workspace** behind an explicit `--demo` flag rather than a silent
  fallback: the file must exist, but a student can get going in 10 s.

## Assumptions flagged

- Poses are the controller's TCP poses with the pendant's active TCP at
  teaching time; the box is in those terms.
- `SafetyLimits.check_joint_target` checks the **flange** FK position against
  the box. With a tool, the flange sits above the TCP → `top`/`home` must be
  taught with that headroom or `move_joints(home)` is rejected. Zero-TCP in the
  sim → no issue. Candidate for a later fix in `safety.py` (apply the set TCP
  in the joint check).
- URSim's start pose (TCP ≈ (−144, −436, 202) mm) is the demo box centre;
  reachability of the demo corners verified by running 03/04/05.
- `workspace.json` written from the container is root-owned on a Linux host;
  on macOS/Windows Docker Desktop it appears as the user's file (not
  re-checked on the Mac mini tonight).

## Verification log (sandbox, URSim 5.9.4)

Unit: box/pose/grid/line, real-profile clamping (oversized box → clipped to
the cell; non-overlapping box → `WorkspaceError`), target mismatch, degenerate
points, fraction range, demo save/load round-trip. Live: `00 --demo` →
01…09 all pass; interactive teach driven by a harness (4 moves + 4 Enters) →
`workspace.taught.json` (240 × 200 × 200 mm) → 03/04/05 pass on it.
Bug found + fixed on the way: `pose(1, 0, w)` landed 1e-17 outside the box
edge and was rejected → values clamped onto the edge.

## Open

- Real-arm run of `00` with freedrive + `03` (lab).
- Draw the taught box in the viewer (would make the fractions visible).
- `Orientation` presets; blend radius for `move_through` (both in the design
  doc 08, not implemented).
