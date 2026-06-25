#!/bin/sh
# Install the framework editable against the bind-mounted source (fast; ur_rtde
# is already compiled in the image). Idempotent — safe to run every start.
pip install -e . --no-deps -q 2>/dev/null || true
exec "$@"
