# Releasing to students (dev repo only)

Two artefacts reach students:

| What | Where | Students get it via |
|---|---|---|
| **Code + docs** — `src/`, `scripts/`, `examples/`, `scenes/`, `toolheads/`, `docs/`, `docker-compose.yml` | git repo `reckj/UR5-PCL-EmbodiedInteraction` | `git pull` (or re-download ZIP) + `docker compose down && docker compose up -d ursim viewer` |
| **Dev/viewer image** — Python 3.12, `ur_rtde` 1.6.3, `viser`, `trimesh`, `numpy` | `ghcr.io/reckj/ur5e-dev:<tag>` (multi-arch amd64 + arm64) | pulled automatically by `docker compose up` |

Everything under `/workspace` is bind-mounted, so code changes never need a
new image. **Only a dependency change (Dockerfile / pyproject `dependencies`)
needs a new tag.**

Current tag: **`hs26.1`**. URSim pin: `universalrobots/ursim_e-series:5.9.4`
— the lab robot runs **PolyScope 5.9.1** (pendant ☰ → About, checked
2026-10-28; IP 192.168.1.11 static), and 5.9.4 is the only 5.9.x image on
Docker Hub (`5.9` / `5.9.4`). After a PolyScope update on the robot: pick the
matching tag (`curl -s "https://hub.docker.com/v2/repositories/universalrobots/ursim_e-series/tags?page_size=100&name=5.2"`
lists them), change the line in `docker-compose.yml`, sync, push. Students get
it with `git pull` + `docker compose up`.

**When you switch URSim versions, empty `programs/` first** (keep
`.gitkeep`): URSim writes `default.installation` there, and a newer version's
file makes an older URSim show "Not an Installation File" on boot. Students
start with an empty folder, so they only hit this if they used a different
version before.

The image bakes in `setuptools`/`wheel` and the entrypoint runs
`pip install -e . --no-deps --no-build-isolation`, so a container start needs
**no network** (verified: ~20 s offline start; without this, pip downloads
setuptools on every start and hangs offline).

---

## A. Build + test the image locally (before publishing)

In `Framework/ur5e_framework/`:

```bash
docker compose -f docker-compose.yml -f docker-compose.dev.yml build
# -> local image tagged ghcr.io/reckj/ur5e-dev:hs26.1 (native arch only)

docker compose up -d ursim viewer
# power on URSim at http://localhost:6080/vnc.html (confirm safety config on a
# fresh container, ON -> START, header icon -> Remote Control), then:
docker compose run --rm dev python scripts/check_connection.py --target sim
docker compose run --rm dev python scripts/smoke_test_move.py
docker compose run --rm dev python examples/grid_process.py
docker compose down
```

Because the local build carries the published name, the *student* compose file
(which only has `image:`) resolves to it too — test the student clone the same
way without pulling.

## B. Publish the multi-arch image to GHCR

One-time:

```bash
# 1. Personal access token (classic) with scopes: write:packages, read:packages
#    (+ delete:packages if you want to prune). github.com -> Settings ->
#    Developer settings -> Personal access tokens.
echo "<TOKEN>" | docker login ghcr.io -u reckj --password-stdin

# 2. A buildx builder that can emit multi-arch manifests (the default "docker"
#    driver cannot). Run once; it persists in Docker Desktop.
docker buildx create --name multiarch --driver docker-container --use
docker buildx inspect --bootstrap
```

Every release:

```bash
cd Framework/ur5e_framework
docker buildx build \
  --platform linux/amd64,linux/arm64 \
  -t ghcr.io/reckj/ur5e-dev:hs26.1 \
  --push .
```

Notes:

- arm64 is built natively on the Mac; amd64 runs under emulation inside buildx.
  The amd64 side is fast anyway: `ur_rtde` ships a manylinux amd64 wheel, so
  nothing is compiled there. The arm64 side compiles `ur_rtde` (a few minutes).
- `--push` is required for multi-arch (a manifest list cannot be `--load`ed into
  the local daemon). To also keep a local copy for testing, run step A.
- Verify both platforms landed:
  `docker buildx imagetools inspect ghcr.io/reckj/ur5e-dev:hs26.1`
  → should list `linux/amd64` and `linux/arm64`.

### Make the package public (once, after the first push)

GitHub → your profile → **Packages** → `ur5e-dev` → **Package settings** →
*Danger Zone* → **Change visibility** → **Public**. Students must be able to
pull without `docker login`. Optionally link the package to the student repo
(*Manage Actions access / Connect repository*) so it shows up on the repo page.

Test anonymously: `docker logout ghcr.io && docker pull ghcr.io/reckj/ur5e-dev:hs26.1`.

## C. When to bump the tag

| Change | New tag? | Students do |
|---|---|---|
| Python code, scripts, examples, scenes, toolheads, docs, compose comments | no | `git pull`, `docker compose down`, `docker compose up -d ursim viewer` |
| New/changed Python dependency, Dockerfile change, base-image bump | **yes** → `hs26.2`, … | `git pull` (compose file carries the new tag) → `docker compose pull` or just `up` |
| URSim version (pendant updated) | no (different image) | `git pull` → empty `programs/` → `docker compose up` pulls the new URSim tag |

Bump procedure: change the tag in **both** `docker-compose.yml` (synced) and
`docker-compose.dev.yml`, build + test (A), publish (B), sync (D), commit, push.
Never overwrite a tag students already use — old clones would silently get a
different image.

## D. Sync code into the student repo

```bash
cd Framework/ur5e_framework
./sync_to_student_repo.sh --dry-run          # see what would change
./sync_to_student_repo.sh                    # default target ../../../UR5-PCL-EmbodiedInteraction
cd ../../../UR5-PCL-EmbodiedInteraction
git diff                                     # review
git add -A && git commit -m "sync framework" && git push
```

What the script copies (whitelist, see the script header): `src/ur5e/` minus
`toolio/`, `scripts/{check_connection,smoke_test_move,rtde_viewer}.py`,
`examples/grid_process.py`, `examples/function_templates/` (whole folder),
`scenes/`, `toolheads/`, `pyproject.toml`, `docker-compose.yml`. It deletes files in those synced folders that no longer
exist here. (`grid_di_test.py` is kept out on purpose: URSim 5.25 cannot drive
digital inputs for RTDE clients, so it blocks at point 1 in the sim.)

What it never touches (owned by the student repo): `README.md`, `docs/`,
`.gitignore`, `LICENSE`, `Start UR5e Sim.command/.bat`, `students/`,
`programs/.gitkeep`. Edit those in the student clone.

To promote another script/example to students: add it to the whitelist in the
script, re-run, review the diff.

## E. Fresh-machine test (what a student experiences)

Only meaningful **after** B (image public) and D (student repo pushed) — a
fresh machine has nothing but the public repo and the public image. On the
machine (e.g. the lab Mac mini M1), follow `docs/SETUP.md` literally, with
these checkpoints:

1. Docker Desktop (Apple Silicon build) installed, running, **Rosetta for
   amd64 emulation** on (Settings → General). VS Code + Git installed; `git
   --version` works in the VS Code terminal.
2. `git clone https://github.com/reckj/UR5-PCL-EmbodiedInteraction.git` into
   the home folder (not iCloud Desktop/Documents).
3. `docker compose up -d ursim viewer` — watch that **both** pulls happen
   anonymously: `ghcr.io/reckj/ur5e-dev:hs26.1` (must say `linux/arm64`, no
   "platform mismatch" warning) and `universalrobots/ursim_e-series:5.9.4`.
   `docker compose ps` → both `Up`. `docker image inspect
   ghcr.io/reckj/ur5e-dev:hs26.1 --format '{{.Architecture}}'` → `arm64`.
4. `localhost:6080/vnc.html` → confirm Safety Configuration → ON → START →
   Settings → System → Remote Control → Enable → header icon → Remote. Note
   the URSim boot time under emulation (expect 1–3 min on M1).
5. `localhost:7001/` → arm visible, Connection line "RTDE: connected".
6. `docker compose run --rm dev bash` → the shell appears within ~30 s (the
   editable install runs offline); `python scripts/check_connection.py
   --target sim`, `python scripts/smoke_test_move.py` (watch the viewer),
   `python examples/grid_process.py`, `exit`.
7. Double-click `Start UR5e Sim.command` once: Gatekeeper prompt? (right-click
   → Open), both tabs open?
8. In the viewer: Objects → Upload an STL → gizmo drag → Save as `test` →
   check `scenes/test/` appears on the Mac with readable files (ownership on
   the bind mount). Safari and Chrome.
9. PolyScope: write a 2-waypoint program, Save → `programs/test.urp` appears
   on the Mac. Copy it to a USB stick → load on the real pendant (5.9.1).
10. `docker compose down`; `docker compose up -d ursim viewer` again → seconds,
    no downloads; Remote Control still enabled (header icon present).

Anything that fails here goes back into `docs/SETUP.md` troubleshooting.

## F. Release checklist (per session)

1. `docker compose -f docker-compose.yml -f docker-compose.dev.yml build` + the
   three test commands in A pass against URSim.
2. If deps changed: bump tag (C), `buildx … --push` (B), `imagetools inspect`.
3. `./sync_to_student_repo.sh`, review `git diff` in the clone.
4. Grep the clone for the lab IP / secrets: `git grep -nE '192\.168\.[0-9]+\.[0-9]+'`
   (placeholders like `192.168.x.x` are fine).
5. Commit + push the student repo. Tell students: `git pull` (+ `docker compose
   up` re-pulls the image if the tag changed).
6. First release only: run E on a fresh machine.
