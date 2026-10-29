# Student repo, prebuilt image and release workflow (2026-10-28) 🟢

**Status: 🟢 validated for student use in simulation** (amd64 path verified
end-to-end against URSim 5.25.2 and then 5.9.4; arm64 image path is exercised
when the multi-arch image is built on the dev Mac and on the Mac mini test).
First course session 2026-10-29.

## Decisions (not to be reopened)

- Students install **Docker Desktop, VS Code, Git** — nothing else. No host
  Python, no Homebrew, no Dev Containers extension, no Jupyter service.
- **Separate student repo** `github.com/reckj/UR5-PCL-EmbodiedInteraction`
  (clone at `/Users/jlr/_LOCAL-DEV/UR5-PCL-EmbodiedInteraction`), populated by
  copying the student-facing parts of `Framework/ur5e_framework/`. The dev repo
  stays the upstream.
- **Prebuilt multi-arch image** `ghcr.io/reckj/ur5e-dev:hs26.1` (amd64 +
  arm64). Students never build: student `docker-compose.yml` uses `image:`;
  `docker-compose.dev.yml` (dev repo only) overrides with `build: .` and the same
  tag, so a local build satisfies the student compose file for testing.
  `docker-compose.yml` is now **identical in both repos** (synced).
- **URSim pinned to `universalrobots/ursim_e-series:5.9.4`.** Pendant checked
  2026-10-28 evening (photo): **URSoftware 5.9.1.1031110 (Jul 2020)**, hostname
  `ur-20195501197`, **IP 192.168.1.11 (static)**, Remote Control already
  enabled (Local/Remote icon in the header). Docker Hub has no 5.9.1; `5.9.4`
  is the only 5.9.x image → same minor line, programs carry over. Decided
  against updating the robot before the first session (lab job: backup,
  possibly stepped update, re-confirm safety config + URCap); update later in a
  calm slot, then bump the tag (one line, `git pull`). The pendant version + IP
  and where to find them (☰ → About) are now in the student README "In short"
  and USAGE §4; the IP is deliberately in the docs.
- `src/`, `scripts/`, `examples/`, `scenes/`, `toolheads/`, `docs/` are
  bind-mounted (`.:/workspace`) → updates reach students via `git pull` +
  `docker compose down && up`; only dependency changes need a new image tag.
- Student workflow: `docker compose up -d ursim viewer` → URSim at
  `localhost:6080/vnc.html` (Connect → power → ON → START → **Remote Control**)
  → viewer `localhost:7001` → `docker compose run --rm dev bash` →
  `python <script>`; own work in `students/<name>/` (git-ignored).
- Viewer scenes switched in the page; `VIEWER_SCENE`/.env not documented for
  students. Viewer promoted from 🔬 to student-facing (display only; "nothing in
  the viewer limits or protects the robot" note kept).
- PolyScope programs land in `./programs` (volume); students copy `.urp` to USB,
  **never** the sim's `.installation`/`.variables`.
- `internal/` and everything experimental stays out of the student repo.

## What went where

**Student repo (45 tracked files, 1.9 MB):** `README.md`, `docs/SETUP.md`,
`docs/USAGE.md`, `docker-compose.yml`, `pyproject.toml`, `src/ur5e/` (minus
`toolio/`), `scripts/{check_connection,smoke_test_move,rtde_viewer}.py`,
`examples/grid_process.py`, `scenes/` (default, example_cart), `toolheads/`
(tool_example), `programs/.gitkeep`, `students/README.md`, `.gitignore`,
`LICENSE` (placeholder; UR meshes keep their BSD-3 LICENSE file),
`Start UR5e Sim.command` (chmod +x) / `Start UR5e Sim.bat`.

**Kept internal (not copied):** `scripts/{first_real_move, profile_move_real,
teach_workspace, grid_di0_test, grid_di_test, safe_move_demo, verify_viewer_fk,
toolhead_rgb_selftest}.py`, `examples/toolhead_rgb_motion.py`,
`src/ur5e/toolio/`, `notebooks/`, `internal/`, `programs/*` test files,
`Dockerfile`, `entrypoint.sh`, `.dockerignore`, `STUDENT_SETUP.md` (superseded
by `docs/SETUP.md`), dev `README.md`.

**Added to the dev repo:** `docker-compose.dev.yml`, `RELEASING.md` (login,
buildx multi-arch push, make package public, tag-bump rules, sync, checklist),
`sync_to_student_repo.sh` (rsync whitelist; `--dry-run`; never touches the
student repo's own docs/glue). Status-line edits in the files that said
"experimental / internal — NOT validated for student use" (`__init__`, `robot`,
`connection`, `motion`, `safety`, `viewer/*`, `rtde_viewer`, `smoke_test_move`,
`scenes/README`, `toolheads/README`, `pyproject` description, viewer banner +
panel label) → "HS26 student framework (sim-validated)".

## Fixes found by the verification

- **Entrypoint needed network on every container start.** `pip install -e .
  --no-deps` builds in an isolated env and downloads `setuptools` each time
  (the `python:3.12` image ships none); offline the containers hung in pip and
  `|| true` would have left `ur5e` uninstalled. Fix: Dockerfile bakes
  `setuptools>=68 wheel`, entrypoint uses `--no-build-isolation`. Offline start
  ≈ 20 s. Goes into `hs26.1`.
- **Remote Control is required against URSim too** for `RTDEControlInterface`
  (Local mode → "Failed to start RTDE data synchronization"). On a fresh URSim
  container: confirm the Safety Configuration dialog → Settings → System →
  Remote Control → Enable (once) → header icon → Remote (after every start; it
  came back as Local after a container restart). Documented in SETUP step 7.
- **URSim 5.25.2 cannot drive digital inputs for RTDE clients.** The I/O tab's
  input checkboxes are read-only; with the pendant's *Simulation* toggle they
  become clickable but `actual_digital_input_bits` stays 0 (and the toggle is
  unavailable in Remote mode). Consequences: `grid_di_test.py` blocks at point 1
  in the sim → **dropped from the student repo** (one whitelist line to re-add);
  the "set the input HIGH in URSim's I/O tab" sentence in `grid_process.py`'s
  docstring and `grid_di*_test.py` is wrong for this version (docstrings left
  as they are — John's call); USAGE.md tells students to fake the sensor in
  code. Outputs do show in the I/O tab.
- URSim's start pose is the bent pose (TCP ≈ (−144, −436, 202) mm), inside
  `sim_open`, so the grid examples run from the start pose.
- **URSim 5.9.4 re-run (after the pin change):** identical flow and results —
  Safety-Configuration confirm dialog on first boot, Settings → System →
  Remote Control → Enable, header icon → Remote, `check_connection` ✓ (mode 7,
  same start pose), `smoke_test_move` PASS, `grid_process` 9 cells, viewer
  streaming; I/O-tab inputs read-only here too. One trap: a `programs/` folder
  containing a `default.installation` written by 5.25.2 makes 5.9.4 show "Not
  an Installation File" on boot → empty `programs/` when switching versions
  (RELEASING.md). Student clones start with an empty folder.

## Verification log (sandbox, x86_64, Docker 29 / Compose v5)

Image built from the final Dockerfile (base image swapped for a CA-trusting
copy of `python:3.12-bookworm` only because of the sandbox proxy; amd64 →
`ur_rtde` manylinux wheel, no compile). Student clone as-is:
`docker compose config` ✓ · `up -d ursim viewer` ✓ · dashboard `power on` +
`brake release` → RUNNING · `check_connection.py --target sim` ✓ (mode 7) ·
Remote Control enabled → `smoke_test_move.py` PASS ✓ · `grid_process.py` 9
cells ✓ · viewer page renders the arm, "RTDE: connected to ursim (20 Hz)",
controller TCP == viewer TCP ✓, Scene/Objects/Toolhead/Safety-planes panels
present ✓. Static: `py_compile` all files ✓; IP/secret grep: only the
`192.168.1.10` placeholder in `check_connection.py`'s docstring (not the
robot's IP) ✓; largest file 464 KB (`upperarm.glb`) ✓; no `toolio` references ✓.

**Fresh Mac mini M1 (lab, 2026-10-28 late evening):** image published
multi-arch via buildx (arm64 `ur_rtde` compile ~9 min), package public,
anonymous pull on the Mac mini, `docker compose up` → URSim 5.9.4 + viewer,
dev shell, `grid_process.py` runs with visible motion in the viewer,
`Start UR5e Sim.command` works by double-click. One doc bug found: pasted
`command  # comment` lines fail on default macOS zsh (no interactive
comments) → all inline comments removed from student code blocks
(student repo commit `7a4c956`). → **arm64 path validated.**

**Not verified yet:** `scenes/` file ownership when the viewer saves from the container;
Safari/Chrome gizmo + upload; a `.urp` saved in the 5.9.4 sim loaded on the
5.9.1 pendant.

## Assumptions carried into the student repo (flag)

- `Robot(target="real")` defaults to `cart_against_wall`: `validated=True`, box
  captured by `teach_workspace` (front = −X), TCP ≤ 0.15 m/s, payload ≤ 0.5 kg,
  `set_tool(payload, tcp)` required — students only reach it with
  `allow_real=True` + `UR_HOST`.
- `tool_example` preset TCP (0, 21, 108) mm assumed from geometry, payload 0 is
  a placeholder. `example_cart` placement assumed, not measured.
- Safety model unchanged: no new safety rules or warnings added beyond the
  existing texts.

## Lab network — first real connection (2026-10-29)

First `check_connection.py --target real` from the lab Mac hung and then
reported `Could not connect to: 192.168.1.11 at 30004`. Cause: the Mac's
Ethernet adapter had a manual `192.168.10.10/24` — a different subnet from the
robot's static `192.168.1.11/24`. Setting the Mac to `192.168.1.20/24` fixed it;
RTDE receive-only works from inside the dev container with no Docker changes.

Facts for the record: the robot's wired network has **no DHCP**; every laptop
needs a manual `192.168.1.x/24` address (students: `.1NN`, instructor `.20`).
Diagnostic rule now in the student docs: *refused* immediately = host reachable,
service down (URSim off); *hangs* = address unreachable (wrong subnet / cable).
Documented in student repo `e90c704` (USAGE §4 block, SETUP troubleshooting,
README, context/KNOWLEDGE). Assumption to verify: that there is really no DHCP
on the lab switch — if there is, the manual-IP step can be dropped.

## Release steps for tonight (see RELEASING.md)

1. Check the diff in the dev repo (status lines, Dockerfile, entrypoint,
   compose) and commit.
2. `docker compose -f docker-compose.yml -f docker-compose.dev.yml build`,
   run the three checks against URSim on the Mac (arm64 path).
3. `docker login ghcr.io` → `docker buildx create --name multiarch --driver
   docker-container --use` → `docker buildx build --platform
   linux/amd64,linux/arm64 -t ghcr.io/reckj/ur5e-dev:hs26.1 --push .` →
   `docker buildx imagetools inspect …` → make the package **public**.
4. In the student clone: review `git status`/`git diff --cached`, commit, push.
5. Fresh-machine test on the lab Mac mini M1 per RELEASING.md §E (needs 3 and
   4 done first — a fresh machine only sees the public repo + public image).
