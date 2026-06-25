# Dev container for the UR5e framework.
#
# Debian Bookworm (same base family as Raspberry Pi OS). Its apt Boost (~1.74)
# is compatible with ur_rtde 1.6.3 — unlike Homebrew's Boost 1.90, which breaks
# the build. On Apple Silicon this image runs NATIVELY as arm64 (no emulation).
#
# No arm64 Linux wheel exists for ur_rtde, so it compiles from source here. That
# compile is baked into an image layer, so editing the framework never recompiles it.
FROM python:3.12-bookworm

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake \
        libboost-system-dev libboost-thread-dev libboost-program-options-dev \
    && rm -rf /var/lib/apt/lists/*

# Compile ur_rtde against apt Boost (cached layer).
RUN pip install --no-cache-dir "ur_rtde==1.6.3"

# Viewer deps (meshcat web 3D + numpy for FK).
RUN pip install --no-cache-dir meshcat numpy

WORKDIR /workspace

# The project is bind-mounted at /workspace; install it editable on start so the
# framework resolves to the live, mounted source. ur_rtde is already present, so
# this is fast (--no-deps, no recompile).
COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
