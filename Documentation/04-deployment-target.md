# Deployment Target — Raspberry Pi 5 per student group 🟡

*Decision 2026-06-16. Development stays on the Mac (preference noted); Pi is the student-facing runtime.*

- Each student group gets **one preconfigured Raspberry Pi 5** (aarch64, Raspberry Pi OS / Debian) — reproducible, identical per group, flashable image.
- **Why it also fixes the build:** there is no aarch64 Linux wheel for `ur_rtde` either, so the Pi compiles from source — but against Debian's `apt` Boost (~1.74), the compatible era for `ur_rtde` 1.6.3. The Mac breakage is specific to Boost 1.90. So the Pi build is expected to succeed where the Mac fails.
- **Development on the Mac:** recommended inside an **aarch64 Debian dev container** that mirrors the Pi image. Apple Silicon runs aarch64 Linux containers natively (no emulation), `ur_rtde` builds cleanly against apt Boost, and dev matches the deployment target exactly. Honors the "develop on the Mac" preference while avoiding the native Boost breakage.
- ❓ **Sim location for the Pis (key open question).** URSim is x86 and will not run on aarch64 Pis (emulation impractical on a Pi). Decide the Pi's role: offline work against a **networked URSim** on a shared server/Mac, vs. **real-robot lab sessions**, vs. **student laptops** for offline sim. The per-group-offline goal and the URSim-parity goal are in tension here.
