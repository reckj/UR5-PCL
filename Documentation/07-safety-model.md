# Safety Model (DEFINING — gate before any real-arm motion) 🟡

*Opened 2026-06-16. No motion is commanded on hardware until this is defined and encoded. No software safety rules added as boilerplate before this.*

**Foundational principle:** the robot's own **PolyScope Safety Configuration** (joint limits, TCP speed/force limits, safety planes/boundaries) is the **authoritative, certified safety layer**. The framework's software checks are a **secondary guardrail** that must be *consistent with but more conservative than* the controller config — never a replacement for it.

**Proposed software-layer design (to confirm):**

- Explicit `SafetyLimits` config: max speed/accel, joint limits, workspace bounds, required payload + TCP.
- Motion primitives **validate-and-reject** on violation (raise a clear error) rather than silently clamping — surfaces mistakes instead of hiding them.
- **Mandatory explicit payload + TCP** before any motion (no silent defaults — already enforced in `connection.py`).
- **Sim-default, explicit real-arm opt-in** so students can't accidentally drive hardware.
- Clean stop on error/disconnect (`stopScript` — already in the connection layer).

**Site-specific inputs needed (cannot be assumed — from JReck):**

- Physical workspace bounds of the actual cell (table, surrounding apparatus, the digital-fabrication setup).
- The real tool / TCP offset, and the payload (mass + CoG).
- Appropriate speed/accel ceilings for the supervised teaching context.
- Current state of the robot's PolyScope Safety Configuration (is it set for this cell? what limits?).
- Operational model: who runs real-arm motion, supervision, e-stop reachability.

## Cell description + safety decisions (2026-06-16)

**Current physical setup:**

- UR5e mounted on a solid metal cart, controller inside the cart.
- Working surface on top of the cart, in front of the robot: ~**500 × 400 mm**.
- Cart stands against a wall → only **~180° workable** at present. The wall is a hard physical boundary.
- **No tool** installed yet. Upcoming tool exploration: light tools with sensors + actuators, **< 500 g**. May increase in later student projects.
- PolyScope: **default setup** — i.e. the authoritative safety layer is currently empty (only e-stop + speed slider protect right now).

**Known future change (design must not hard-code current cell):**

- Later module/tests: full **360°** usable; robot possibly **wall- or truss-mounted**. → workspace bounds + mounting orientation must be **swappable named profiles** (e.g. `cart_against_wall`, later `truss_360`), like the host parameter.

**First target process (defines motion vocabulary):**

- Repeated process along a **path or grid**, with **sensor-dependent execution** (digital + analog), driving an **actuator tool** (tool with sensor + servo; additional digital/analog sensors on the process area).
- Implies: `moveJ`/`moveL` to waypoints, `moveL` across grid points, RTDE digital/analog **read** (sensors) + **write** (actuator) via `RTDEIOInterface`.

**Decisions:**

- 🟢 **Slow speed for first real tests** — bring-up at jog speed for visible, interruptible motion (target TCP ≤ ~100–250 mm/s; exact ceiling in the profile).
- 🟡 **TCP-above-surface height: unknown** — parameterize the "don't hit the cart top" plane; measure once on hardware.
- **Gating task:** configure PolyScope Safety Configuration first (still at defaults) — wall safety plane, plane above cart surface, reduced TCP speed/force, payload set to tool. Authoritative layer; values to be provided once profile numbers are picked.
- **Software layer:** swappable `SafetyLimits` profile mirroring the cell (workspace box ≈ 500 × 400 + height range, wall bound, joint/speed/accel ceilings below PolyScope limits, < 500 g payload + tool TCP). Motion primitives validate-and-reject. Sim-default, explicit real-arm opt-in.

## Safety layer implemented (sim-validated) (2026-06-16)

Built and unit-validated (logic only; no hardware). Files: `src/ur5e/safety.py`, `src/ur5e/motion.py`, `src/ur5e/kinematics.py` (FK promoted into the package; viewer now reuses it), `scripts/safe_move_demo.py`.

- **`SafetyLimits` profiles (swappable by name):**
	- `cart_against_wall` — PROVISIONAL, `validated=False`: TCP speed ≤ 0.15 m/s (150 mm/s), payload ≤ 0.5 kg, workspace box ≈ 500×400 surface region. **Box coords + cart-top height (z lower bound) are placeholders — measure on hardware and set `validated=True` before real motion.**
	- `sim_open` — generous box for free sim exploration; slow-speed/joint guards still active.
- **`MotionController`** wraps a connection + profile; `move_j`/`move_l` **validate-and-reject** (raise `SafetyViolation`) on out-of-workspace, over-speed, over-accel, joint-limit — never silent clamp.
- **Real-arm gate (all four enforced, sim skips them):** non-sim host requires `allow_real=True` AND `profile.validated` AND payload set AND TCP set. Verified: real motion refused on each missing condition; sim motion allowed.
- **Principle preserved:** these are the SECONDARY software guard; PolyScope Safety Configuration remains authoritative and must be set stricter than software — software stays ≤ controller limits.

**Next on hardware (next lab morning):** configure PolyScope (wall plane, cart-top plane, TCP speed/force, payload) → measure cart-top height + surface extents → replace `cart_against_wall` box coords with measured base-frame values → set `validated=True`. Only then does `allow_real=True` motion run.

## Student-facing API + first example (2026-06-16)

Pivot to the student framework surface. Two tracks now reflected in the layout: `scripts/` = internal/validation tools, `examples/` = student-facing example processes.

- **`src/ur5e/robot.py` — `Robot` facade (the growing student base-functions module).** The only module students need; hides RTDE / safety / motion internals. API: `connect/disconnect` (context manager), `joints()`, `pose()`, `move_joints/move_to/move_through` (validated), `set_tool(payload, tcp)`, sensor reads `read_digital_in/read_analog_in`, actuator writes `set_digital_out/set_tool_digital_out/set_analog_out`. Exported as `from ur5e import Robot`.
- **Pattern generators (grow this section):** `grid_poses` (snake/boustrophedon order), `line_poses`. Reusable path/grid helpers.
- **Auto safety profile:** `target="sim"` → `sim_open`; `target="real"` → `cart_against_wall`. Real motion still fully gated (allow_real + validated + tool set).
- **`examples/grid_process.py` — first example:** grid traversal + sensor-gated actuation (analog/digital read → conditional actuator). The skeleton students adapt. Sim inputs read static 0 (set them HIGH in URSim I/O tab to see the branch fire).
- Validated in sim (API surface, profile selection, pattern generators). Status: experimental — becomes the stable student framework only once hardware-validated.

**Design principle reinforced:** students only touch `ur5e.robot` + `examples/`; the lower layers stay internal. The module is meant to grow with reusable base functions as processes develop.
