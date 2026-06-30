# Tech Stack Decisions

*Updated 2026-06-16. All decisions below are 🟡 provisional research decisions — selected with reasoning, not yet validated on hardware.*

## Comms library — `ur_rtde` (Python, RTDE protocol) 🟡

- Python interface via `pip install ur_rtde`.
- Officially tested on macOS 15 (Sequoia); e-Series compatible (UR5e is e-Series).
- **To validate:** install path on the actual dev Mac (see open questions on arm64 wheel).

## Simulation — URSim (Dockerized, e-Series image) 🟡

- **Canonical sim target.** Runs the actual PolyScope/controller firmware and exposes the real RTDE ports (30001–30004), so `ur_rtde` code talks to it **byte-identically** to the real arm.
- Chosen specifically for **sim↔real code parity**: the same framework runs against sim or real, switching only the host IP.
- **Caveat (Apple Silicon):** URSim is x86 Linux → runs under qemu emulation on M-series Macs. Functional but CPU-heavy. To load-test early.
- PyBullet/MuJoCo + UR5 URDF kept as an optional 🔬 lightweight previewer for kinematics/visualization only — **never** the API students code against (would diverge from RTDE).

## Language & setup — Python, two-track split 🟡

- **Research/exploration + internal validation:** notebooks — Jupyter or VS Code interactive cells (`# %%`). State inspection, sensor-data plotting, log analysis, single-shot motion tests.
- **Student-facing framework + any real-time control loop + any real-robot execution:** plain importable Python package, run from terminal. Rationale: `RTDEControlInterface` holds a live socket (interrupted notebook cells can leave the robot mid-command); `servoJ`/`speedJ` run at 125–500 Hz (notebooks are poor at deterministic loops); airdrop favors a package over notebook state.
- Framework built as an **installable package** (`pip install -e .`). Notebooks and scripts both `import` it — neither reimplements control logic.
- **Sim↔real swap = single `host`/IP parameter.** The one architectural through-line.

## Dev workflow — develop on the MacBook, clean setup later 🟡

- **Build phase (now):** develop directly on the dev MacBook via editable install (`pip install -e .`). Iterate freely; version pinning/reproducibility intentionally deferred. The scaffold is the seed — this install mode *is* the dev setup, no extra work needed now.
- **Clean setup (deferred, post-stabilization):** once stable and validated on hardware, do a clean reproducible pass — pin dependencies to exactly the validated versions (lockfile), confirm the prebuilt `ur_rtde` wheel (no source build), document the URSim image **and tag**, rebuild the venv from scratch to prove reproducibility. This clean base is what the validated student-facing framework is cut from.
- Trigger: framework stable + validated against URSim and the real arm.
- *To pin later:* whether "clean setup" targets the same dev Mac or a separate handoff/student environment (slightly different artifacts).
