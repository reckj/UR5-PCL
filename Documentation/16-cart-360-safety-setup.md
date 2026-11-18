# `cart_360` — safety setup for the arm on the free-standing cart (2026-11-18) 🟡

**Status (2026-11-18, 10:55): 🟢 sim-validated, unlocked for the real arm.**
44 offline checks PASS (`scripts/verify_cart_cell.py`, also inside the dev
container); live run against URSim 5.9.4 with the measured cell: all PASS,
49 checks (JReck) — after the controller-IK fix described below. Cart geometry
from JReck's drawing (`assets/cart_Drawing.pdf`), confirmed by him;
`validated: true` set with `teach_cart_cell.py --validate`. **Not yet run on
the lab arm; not yet synced to the student repo.**

## The cart, in the robot Base frame

From the drawing: cart 690 × 690 mm, 790 mm high; robot on a 180 × 180 × 10 mm
plate, centred across the cart and 124 mm from one edge; a handle bar
450 × 60 mm on one side face just below the top.

| body | x [mm] | y [mm] | z [mm] |
|---|---|---|---|
| cart | −566 … +124 | −345 … +345 | −800 (floor) … −10 (cart top) |
| handle | −455 … +10 | −405 … −345 | −75 … −35 |
| mounting plate | −90 … +90 | −90 … +90 | −10 … 0 |

z = 0 is the robot's mounting plane = top of the plate; the cart top is at
z = −10 mm. The handle and the plate are `extras` in the cell file: keep-outs
of their own, so the space under the handle stays usable.

**Confirmed by JReck (2026-11-18):** the base frame is aligned with the cart;
the long part of the top (566 mm from the robot axis) is −X; the handle is on
the **−Y** side at the position drawn; 790 mm is floor to cart top, and the
robot's mounting plane is the top of the 10 mm plate, 10 mm above the cart top.
(The handle's position along the cart and its height were read off the
drawing's geometry and padded by a few mm.)

## Why

The `cart_against_wall` setup (wall plane + table plane on the pendant, a
box over the table in software) only allows work on the cart top. The HS26
projects need the arm on the cart to reach **around the cart and below its
top surface**. One setup serves all groups.

## Decisions

- **New cell profile `cart_360`**, default for `target="real"`.
  `cart_against_wall` stays available (`UR5E_CELL=cart_against_wall`), as does
  the instructor override `UR5E_CELL=open` → `real_open`.
- **The cart body is a keep-out in software.** PolyScope planes cannot do
  this: planes bound a convex space, and the cart is an obstacle *inside* the
  space the arm has to reach around. Keeping the arm off the cart is the
  software layer's job, backed by the controller's force and speed limits.
- **No safety planes on the pendant at all** (JReck, 2026-11-18, 10:35): the
  draft's floor plane (Base feature displaced to 250 mm above the floor) is
  dropped so the installation does not depend on a displaced Base plane. The
  pendant installation `cart_360` = the old one minus the wall and table
  planes; Robot Limits unchanged. The floor is guarded by the software checks
  only (TCP ≥ 50 mm above it, arm links clear of it).
- **No Reduced mode below the cart top** (JReck, 2026-11-18): groups work
  beside the cart below its top and should not run slowed down there. One
  group reaches under a table that stands on the cart's **+Y side**. The first draft had a Trigger-Reduced-Mode plane
  100 mm below the top (150 mm/s); it is removed. Normal-mode limits apply
  everywhere.
- **Tool clearance to the cart sides 20 mm** (JReck, 2026-11-18; first draft
  50 mm). Arm-link radius stays 60 mm — the tubes are about 40–45 mm in radius,
  so that is roughly the same 15–20 mm of air.
- **Nothing else is within the arm's reach** where the cart stands (JReck) →
  no outer planes, `reach` box left at ±0.95 m.
- **One source for the geometry**: `src/ur5e/cells/cart_360.json` (cart
  footprint, cart top, floor, clearances). The software profile, the viewer
  scene `scenes/cart_360/` and the plane values for the pendant are all derived
  from it by `scripts/teach_cart_cell.py`.
- **Paths are checked, not only targets** — for profiles with a keep-out
  only. `sim_open`, `cart_against_wall` and `real_open` behave exactly as
  before.
- **Rehearsal in the sim**: `--cell cart_360` on any example (or
  `UR5E_CELL=cart_360`) applies the lab cell in URSim; the viewer scene
  `cart_360` shows the cart.

## The two layers

| | Pendant installation `cart_360` (authoritative) | Software profile `cart_360` (secondary) |
|---|---|---|
| floor | no plane | TCP ≥ 50 mm above the floor, arm links above the floor |
| planes | **none** | — |
| cart body, handle, plate | not expressible with planes | keep-outs: tool ≥ 20 mm from their sides and undersides (tops may be touched), arm links ≥ 60 mm all round |
| speed | 250 mm/s tool (most-restrictive preset), everywhere | TCP ≤ 150 mm/s, joints ≤ 0.5 rad/s |
| force | 100 N tool / elbow (preset) | — |
| outer boundary | none (nothing within reach); add a plane per side if that changes | TCP box ±0.95 m, z up to 1.0 m (`reach` in the cell file) |
| payload / TCP | Installation → General | ≤ 0.5 kg; payload and TCP must be declared |

## Software layer — what is checked

For every move with `cart_360` (also for a taught `Workspace` clipped to it —
the keep-out and ceilings are inherited):

| move | checked before anything is sent |
|---|---|
| `move_to` / `move_through` (moveL) | target TCP; the straight TCP line start → target; the flange line; the tool line flange → TCP at the target; flange height; arm links at up to 12 points along the line, joint angles from the framework's own inverse kinematics (`kinematics.ik_near`: nominal model, iterated from the current joints and chained point to point, so it follows the branch the linear move takes) |
| `move_joints` (moveJ) | target as above via forward kinematics; arm links and tool line every ≤ 3° of the largest joint travel between the current and the target joints |

Arm links = spheres of 60 mm radius along the physical centre lines (upper arm
with its 138 mm lateral offset, elbow, forearm, wrist 1–3; same geometry as the
viewer's mesh chain, compared numerically). A violation raises
`SafetyViolation` naming the link or line and the point; nothing is clamped.

**Not checked:** the shape of the mounted tool beyond the flange → TCP line
(a wide gripper body is not modelled), cables, objects standing next to the
cart, people. Base and shoulder are fixed on the mount and excluded. Nominal
kinematics (a few mm off the calibrated arm).

**Finding (URSim 5.9.4, 2026-11-18):** the first version asked the controller
for the joint angles (`getInverseKinematicsHasSolution` +
`getInverseKinematics`). Through ur_rtde 1.6.3 on PolyScope 5.9 that call
answers *False* for reachable poses — even for the pose the arm is standing
in — so every linear move was rejected with "no joint solution". The check now
uses `ik_near` in the framework: no controller round trip, identical in sim,
on the real arm and offline. Limits: nominal kinematics; close to a
singularity (wrist over the base axis, arm fully stretched) it may not
converge or may pick the neighbouring branch — a move through such a point is
then rejected as "no joint solution" or checked on a slightly different arm
posture.

## How to set it up (lab, about 20 minutes)

Needs: tape measure, the pointer tool, the safety password, the dev repo shell
on the robot network.

1. **Cart height.** Done: 790 mm (drawing) → floor at z = −800 mm.
2. **Keep the old installation.** Pendant: load the current installation,
   *Save Installation As…* → `cart_360`. The wall setup stays in the old file.
3. **Planes.** Installation → Safety → unlock → Planes:
   delete the wall plane and the table plane (or set both to Disabled). No
   plane is added — the list ends up empty / all Disabled.
4. **Limits.** Robot Limits stay on the most-restrictive preset; no Reduced
   mode is used. Joint Limits: if a tool cable is attached, untick
   *Unrestricted Range* for Wrist 3.
5. **Apply and restart**, confirm, save the installation, write down the new
   safety checksum (top right).
6. **Confirm the cart numbers** (already entered from the drawing; the
   touch-teach below is only needed if the corner check in step 7 is off by
   more than about 10 mm). Mount the pointer, make its TCP active on the pendant.
   ```
   docker compose run --rm --no-deps -e UR_HOST=192.168.1.11 dev bash
   python scripts/teach_cart_cell.py --target real
   ```
   Receive-only: guide the tip to the four top corners of the cart, then to
   the floor (or type `height <mm>`). Corners the arm cannot reach can be typed
   in mm. Tape-measure alternative:
   `python scripts/teach_cart_cell.py --set X0 X1 Y0 Y1 --height H` (metres,
   from the centre of the robot base; front of the lab cart = −X).
   The script writes the cell file and the scene.
7. **Look at it.** Viewer, scene `cart_360`, connected to the real arm: touch a
   cart corner with the tip — the TCP frame must sit on the corner of the grey
   box.
8. **Sim check.** With URSim running:
   `python scripts/verify_cart_cell.py --target sim --reach`
   → all PASS, including "framework IK reproduces the controller's joints".
9. **Real-arm check, by hand.** Freedrive the flange below the cart top and
   towards the floor: it moves freely (no planes any more).
10. **Unlock.** `python scripts/teach_cart_cell.py --validate` (prints the
    checklist, asks for the word `validated`).
11. **Release** — everything reaches the students through the student repo:
    ```
    cd Framework/ur5e_framework
    ./sync_to_student_repo.sh --dry-run
    ./sync_to_student_repo.sh
    cd ../../../UR5-PCL-EmbodiedInteraction
    ./context/make_source_snapshot.sh
    git status
    git add -A
    git commit -m "cart_360: safety setup for the arm on the cart"
    git push
    ```
    Then commit the dev repo. Students: `git pull`, then
    `docker compose down` and `docker compose up -d ursim viewer` (code is
    bind-mounted; no new image, no dependency changed). The sync copies
    `src/ur5e/` incl. `cells/cart_360.json` and `scenes/cart_360/`; the docs and
    `context/` files are already in the student clone's working tree.

Tuning without re-measuring: `teach_cart_cell.py --side-clearance …
--arm-radius … --flange-floor … --reach X0 X1 Y0 Y1 --z-max …`. Any new
geometry resets `validated` to false.

## For the students

`docs/SAFETY_SETUP.md` in the student repo is the hand-out;
`context/CART_360.md` is the same setup written for the students' AI
assistants (rules for planning moves around the cart, the rejection messages,
what not to do), referenced from `AGENTS.md`, `CLAUDE.md`,
`.github/copilot-instructions.md`, `context/README.md`, INSTRUCTIONS §4 and
KNOWLEDGE §6. Existing
`workspace.json` files over the table keep working (they lie inside the new
cell); only poses with a wrist or forearm closer than 60 mm to the cart top
are newly rejected. To work beside the cart, a group teaches a workspace
there, or uses absolute poses with `Robot(target="real", …)`.

## Assumptions flagged

- Geometry is from the drawing, with frame orientation, height and handle
  position confirmed by JReck; the corner check in step 7 remains the on-arm
  cross-check. Level floor.
- With no planes, the **authoritative layer has no geometric limit**: on the
  pendant only force, speed, power and momentum limits act. Floor and cart are
  protected by the framework's checks (secondary) — raw `robot.conn.control`
  calls, pendant programs and freedrive are not covered by them. No tool is
  longer than 200 mm (JReck).
- Clearance 20 mm is measured from the **line** flange → TCP, not from the
  tool's body: a tool wider than about 40 mm can touch the cart side while the
  check still passes. Arm links 60 mm from their centre line.
- The one group's table on the +Y side (and anything else standing next to the
  cart) is not modelled; reaching under it is checked against the cart and the
  floor only.
- Payload ceiling stays 0.5 kg; speed ceilings as `cart_against_wall`.
- People are in no check. The space where hands can get between arm and cart
  edge is new with this setup; who stands where during the demo and the
  exhibition is still an open safety-model question, not decided here.

## Files (dev repo, `Framework/ur5e_framework/`)

| file | change |
|---|---|
| `src/ur5e/cells/cart_360.json` | new — cell geometry (cart, `extras`: handle + mounting plate, floor, clearances) + `validated` flag (format `ur5e-cell/1`) |
| `src/ur5e/safety.py` | `KeepOut`, `Box.segment_inside`, `SafetyLimits.keepout / arm_radius / floor_z / flange_min_z` (the last one unused by `cart_360`), `check_arm`, `check_joint_path`, `check_linear_path`, `describe()`, `load_cell`, `cart_profile`, profile `cart_360` |
| `src/ur5e/kinematics.py` | `arm_spheres`, `ik_near`, rotation-vector helpers, `interpolate_pose`, `flange_position_from_pose` |
| `src/ur5e/motion.py` | path checks for profiles with a keep-out (linear moves via `ik_near`) |
| `src/ur5e/config.py` | `cell_profile_name` (moved here), `REAL_CELL_DEFAULT = "cart_360"`, `UR5E_CELL` accepts any profile name |
| `src/ur5e/workspace.py`, `robot.py`, `cli.py` | use `cell_profile_name`; `--cell NAME`; `describe()` notes a box reaching into the keep-out |
| `src/ur5e/cell.py` | new — `python -m ur5e.cell` prints the cell numbers |
| `scripts/teach_cart_cell.py` | new, internal — measure / `--set` / `--show` / `--validate`; writes cell file + scene |
| `scripts/verify_cart_cell.py` | new, internal — offline checks on a fixed test cart; `--target sim` live checks |
| `scenes/cart_360/` | new, generated — cart, handle, plate (no planes) |
| `Documentation/assets/cart_Drawing.pdf` | JReck's drawing of the cart |
| `pyproject.toml`, `examples/function_templates/README.md`, `examples/grid_process.py` | package data; wording |

The working tree also still holds the earlier uncommitted changes (TCP-aware
joint check, `real_open`); `cart_360` builds on them.
