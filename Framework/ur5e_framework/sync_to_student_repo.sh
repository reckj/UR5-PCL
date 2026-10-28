#!/bin/bash
# sync_to_student_repo.sh — copy the student-facing parts of ur5e_framework
# into the student repo clone (https://github.com/reckj/UR5-PCL-EmbodiedInteraction).
#
# Dev repo (this folder) is the source of truth for CODE:
#   src/ur5e/ (minus toolio/), the whitelisted scripts/ and examples/, scenes/,
#   toolheads/, pyproject.toml, docker-compose.yml.
# The student repo owns its own DOCS and glue (README.md, docs/, .gitignore,
# LICENSE, launchers, students/, programs/.gitkeep) — this script never touches
# those. Edit them in the student repo directly.
#
# Usage:
#   ./sync_to_student_repo.sh                 # default target: ../../../UR5-PCL-EmbodiedInteraction
#   ./sync_to_student_repo.sh /path/to/clone
#   ./sync_to_student_repo.sh --dry-run       # (or -n) show what would change
#
# Afterwards: cd <clone> && git status && git diff && git add -A && git commit && git push
set -euo pipefail

SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DST=""
DRY=()
for a in "$@"; do
  case "$a" in
    -n|--dry-run) DRY=(--dry-run) ;;
    *) DST="$a" ;;
  esac
done
DST="${DST:-$SRC/../../../UR5-PCL-EmbodiedInteraction}"

if [ ! -d "$DST/.git" ]; then
  echo "error: $DST is not a git clone (expected the student repo)" >&2
  exit 1
fi
DST="$(cd "$DST" && pwd)"
echo "sync  $SRC"
echo "  ->  $DST   ${DRY[*]:-}"

[ ${#DRY[@]} -eq 0 ] && mkdir -p "$DST/src/ur5e" "$DST/scripts" "$DST/examples" "$DST/scenes" "$DST/toolheads"

# Never copied anywhere:
COMMON_EXCLUDES=(--exclude '__pycache__/' --exclude '*.py[cod]' --exclude '.DS_Store'
                 --exclude '*.egg-info/' --exclude '.ipynb_checkpoints/')

# --- 1. package: everything in src/ur5e except the internal toolio/ ----------
rsync -a --delete "${DRY[@]}" "${COMMON_EXCLUDES[@]}" \
  --exclude 'toolio/' \
  "$SRC/src/ur5e/" "$DST/src/ur5e/"

# --- 2. scripts: whitelist (everything else in scripts/ is internal) ---------
rsync -a --delete "${DRY[@]}" "${COMMON_EXCLUDES[@]}" \
  --include 'check_connection.py' \
  --include 'smoke_test_move.py' \
  --include 'rtde_viewer.py' \
  --exclude '*' \
  "$SRC/scripts/" "$DST/scripts/"

# --- 3. examples: whitelist ---------------------------------------------------
# (grid_di_test.py deliberately NOT included: URSim 5.25 cannot drive DI0, so
#  it blocks forever in the sim. Add it here once there is a sim-side way.)
rsync -a --delete "${DRY[@]}" "${COMMON_EXCLUDES[@]}" \
  --include 'grid_process.py' \
  --include 'function_templates/' \
  --include 'function_templates/**' \
  --exclude '*' \
  "$SRC/examples/" "$DST/examples/"

# --- 4. viewer scenes + toolhead presets (whole folders) ----------------------
rsync -a --delete "${DRY[@]}" "${COMMON_EXCLUDES[@]}" "$SRC/scenes/"    "$DST/scenes/"
rsync -a --delete "${DRY[@]}" "${COMMON_EXCLUDES[@]}" "$SRC/toolheads/" "$DST/toolheads/"

# --- 5. single files ----------------------------------------------------------
rsync -a "${DRY[@]}" "$SRC/pyproject.toml"      "$DST/pyproject.toml"
rsync -a "${DRY[@]}" "$SRC/docker-compose.yml"  "$DST/docker-compose.yml"

# NOT synced (dev only): Dockerfile, .dockerignore, entrypoint.sh (baked into the
# image), docker-compose.dev.yml, RELEASING.md, this script, README.md,
# STUDENT_SETUP.md, notebooks/, internal/, programs/*, src/ur5e/toolio/,
# scripts/{first_real_move,profile_move_real,teach_workspace,grid_di0_test,
# grid_di_test,safe_move_demo,verify_viewer_fk,toolhead_rgb_selftest}.py,
# examples/toolhead_rgb_motion.py.

echo
if [ ${#DRY[@]} -eq 0 ]; then
  (cd "$DST" && git status --short)
  echo
  echo "review:  cd '$DST' && git diff"
  echo "publish: git add -A && git commit -m 'sync framework' && git push"
fi
