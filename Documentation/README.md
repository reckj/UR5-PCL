# UR5e — Embodied Interaction HS26

Research base and toolbuilding for a UR5e cobot, feeding an embodied interaction module at ZHdK. Two output tracks: **internal research/validation tools** and a **stable student-facing framework**. Students only ever receive validated, stable framework — this documentation tracks the upstream work.

> **Status legend:** 🟢 validated for student use · 🟡 selected, to validate · 🔬 experimental/internal · ❓ open question

## Contents

This folder mirrors the project's Notion research log. Each file is one major section:

1. [Tech Stack Decisions](01-tech-stack-decisions.md) — comms library, simulation, language/setup, dev workflow
2. [Open Questions](02-open-questions.md) — unresolved forks and decisions
3. [Next Steps](03-next-steps.md) — immediate scaffolding tasks
4. [Deployment Target — Raspberry Pi 5](04-deployment-target.md) — student-facing runtime
5. [Build Status](05-build-status.md) — macOS vs aarch64 build, end-to-end validation
6. [Visualization](06-visualization.md) — browser-based RTDE 3D viewer
7. [Safety Model](07-safety-model.md) — authoritative PolyScope layer + software guardrail, cell description, implementation
8. [Student-Facing Motion API Design](08-motion-api-design.md) — three-layer API, orientation, waypoints, naming
9. [Tool I/O — Electrical Interface](09-tool-io-electrical-interface.md) — 8-pin connector, pinout, power budget, framework mapping
10. [Toolhead RGB + Button over RS485 / TCI](10-toolhead-rgb-rs485-interface.md) — APDS9960 + button Arduino toolhead, TCI :54321 host receive, live viz + button-gated motion

## About this documentation

Going forward, project documentation lives here as markdown and is versioned alongside the code in this repository, synced via GitHub. The content was migrated from Notion; dates in each section reflect when those decisions were made.
