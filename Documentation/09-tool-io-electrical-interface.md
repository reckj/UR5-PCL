# Tool I/O — Electrical Interface (2026-06-22) 🔬

Research/internal. Source: UR5e User Manual §5.8 (authoritative). No hardware powered or driven yet — this is the electrical-interface reference for the first tool (light tool, sensor + servo, < 500 g, per the cell description).

## Connector

- **8-pin connector** adjacent to the tool flange on Wrist #3. Provides power + control signals for grippers/sensors.
- Mating cable: **Lumberg RKMV 8-354**. Tighten connector by hand to **max 0.4 Nm**.
- Tool flange is tied to GND (same as the red wire).

## Pinout (by wire color)

| Color | Signal | Function |
| --- | --- | --- |
| Red | GND | Ground (= flange) |
| Gray | POWER | 0V / 12V / 24V (software-selected, Installation tab) |
| Blue | TO0 / PWR | Digital Output 0 — or 2nd power pin (Dual Pin mode) |
| Pink | TO1 / GND | Digital Output 1 — or 2nd ground pin (Dual Pin mode) |
| Yellow | TI0 | Digital Input 0 |
| Green | TI1 | Digital Input 1 |
| White | AI2 / RS485+ | Analog In 2 — or RS485+ |
| Brown | AI3 / RS485- | Analog In 3 — or RS485- |

Per connector, max: **2 digital out, 2 digital in, 2 analog in** — OR trade the two analog lines for an **RS485 bus** (mutually exclusive: enabling the Tool Communication Interface / TCI disables tool analog input).

## Power budget (POWER pin, set to 12V or 24V via Installation tab)

- **Single-pin:** 1000 mA typ, **2000 mA peak** (peak ≤ 1s, ≤ 10% duty; 10s avg ≤ typical).
- **Dual-pin:** 1500 mA typ, 2000 mA peak. Wire Gray→Blue (POWER→TO0) and Red→Pink (GND→TO1).
- Capacitive load ≤ 8000 µF; 400 ms soft-start; **no hot-plugging capacitive loads**.
- Continuous power available: **~24W @ 24V (~36W dual-pin), ~12W @ 12V.** Brief 48W peaks. Ample for a < 500 g tool.
- ⚠️ **On E-stop, POWER → 0V and both digital outputs → High-Z.** Tool-powered actuators/latches release on E-stop. Relevant to safety model + fail-safe design.

## Digital Outputs (TO0/TO1)

- Modes: Sinking (NPN), Sourcing (PNP), Push/Pull — set per pin in Installation tab.
- ≤ 1000 mA per output; ≤ 26V open voltage.
- ⚠️ **Tool digital outputs are NOT current-limited — overcurrent causes permanent damage.** Deactivate (High-Z) on E-stop.

## Digital Inputs (TI0/TI1)

- PNP with weak pull-down → **floating reads LOW**.
- Logical low ≤ 2.0V, logical high ≥ 5.5V, 0–26V range, ~47 kΩ input.
- Accepts: industrial PNP/sourcing prox sensors (inductive/capacitive), photoelectric/break-beam, limit/microswitches, reed/hall switches, simple buttons (POWER→TI0).
- ⚠️ NPN (sinking) sensors need a pull-up. Bare 3.3V logic (ESP GPIO) may not reach the 5.5V high threshold — level-shift or drive from ≥ 5V.

## Analog Inputs (AI2/AI3)

- Non-differential. Per pin: voltage (0–10V) OR current (4–20 mA), set in Installation tab. 12-bit.
- Force/pressure/distance/potentiometer-type sensors.
- Unavailable when RS485/TCI is enabled.
- ⚠️ Current mode is NOT overvoltage-protected.

## Tool Communication (RS485 over White/Brown, via TCI)

- Baud up to 5M; 1–2 stop bits; parity none/odd/even. Internal fail-safe biasing. Latency 2–4 ms.
- Enabling TCI consumes the two analog inputs.

## Answers to the bring-up questions

- **ESP in the tool, arm-powered:** Yes — recommended. ESP avg < 250 mA (WiFi peaks ~500 mA), well within 1A. Power via tool **12V → ESP VIN** (cleanest), or 24V → external buck (don't feed 24V to VIN). Talk-back options: (1) **RS485 over TCI** — stays on the one tool cable, inside the I/O model, reproducible in framework; (2) **ESP WiFi direct to Mac/Pi** — flexible but a separate channel outside the sim↔real parity model (internal/experimental only, never against URSim).
- **Driving a small servo:** Don't drive a hobby servo from a tool digital output (on/off only, not PWM, not current-limited). **Recommended:** MCU in the tool — tool 12V → buck to 5–6V for servo, MCU generates PWM, command in via RS485 (TCI) or WiFi. Co-locates sense + actuation at the tool (matches the first target process). Minimal alt: TO0 can *trigger* a self-contained actuator/driver board (binary fire/retract only). Assumes a small hobby/digital servo in the < 500 g envelope.
- **Digital sensors connectable:** Directly on TI0/TI1 — any PNP/sourcing digital sensor in 0–26V (prox, photoelectric, limit/reed/hall switches, buttons). Analog on AI2/AI3 (0–10V or 4–20mA). Richer sensors (I²C/SPI/encoders/camera) go through a tool MCU → RS485/WiFi.

## Framework mapping (ur_rtde) + gaps

- Read TI0/TI1 → RTDE tool digital-in bits → facade `read_digital_in()`. ✅ maps.
- Read AI2/AI3 → `getToolAnalogInput0/1()` / RTDE `tool_analog_input0/1`. ⚠️ **Gap:** facade has `read_analog_in` (controller analog) but no distinct **tool** analog read — add `read_tool_analog_in()`.
- Write TO0/TO1 → `RTDEIOInterface.setToolDigitalOut(id, bool)` → facade `set_tool_digital_out()`. ✅ maps.
- Set POWER voltage (0/12/24V) → **PolyScope Installation tab only — NOT settable over RTDE** (`tool_output_voltage` is read-back). ⚠️ Document as a manual precondition; framework can't control it.
- RS485 / TCI → controller bridge, not in current facade. 🔬 separate design thread.

## Open questions / next steps

- ❓ **Tool talk-back fork:** RS485-over-TCI (in-model, parity-friendly) vs. ESP-WiFi (flexible, off-parity). Prototype both early.
- ❓ **POWER voltage selection** is a manual PolyScope step — decide how the framework documents/asserts it as a precondition (can't set it over RTDE).
- Add `read_tool_analog_in()` to the facade; confirm tool digital-in bit indices on hardware.
- Validate against real arm: power a load on TO0, read a button on TI0, read an analog sensor on AI2 — receive-only + single-output, still pre-safety-gate.

## Cable candidate — Belden/Lumberg RKMV 8-354/2M (2026-06-22)

Conrad CH order-no. 1234965 (EAN 2050002444126). Not a cheaper substitute — it's the **same Lumberg connector family the manual specifies** ("Lumberg RKMV 8-354"), sold under Belden (Belden owns Lumberg Automation). The `/2M` = 2 m length. Pigtail / open wire end — correct for a self-built tool (land the 8 colored wires on own board/terminal).

**Spec check vs. tool flange:**

| Spec | This cable | UR5e req | OK |
| --- | --- | --- | --- |
| Type | RKMV 8-354/2M (Lumberg) | "Lumberg RKMV 8-354" | ✅ exact family |
| Poles | 8 | 8 | ✅ |
| Rated current | 1 A | 1 A typ/pin (2 A peak) | ✅ |
| Rated voltage | 32 V | 24 V max | ✅ |
| Conductor | 0.14 mm² (~AWG26) | — | ⚠️ thin — fine for signals + ESP/sensor current; near rating only if dual-pin power pushed ~1.5 A (not our < 500 g tool case) |
| Protection | IP67/IP69K | — | ✅ |
| Temp | -40…+90 °C | — | ✅ |

**Two checks before ordering / before trusting wiring:**

- ⚠️ **Connector gender at the flange.** This is an M8 **socket/Buchse, straight**. UR5e Wrist #3 tool connector is typically male pins → socket is correct. But e-Series revisions vary — visually confirm the flange connector is male pins (→ this RKMV socket) vs. female (→ would need the **RSM 8-354/2M** plug variant, Conrad 1234964). Quick look at the arm.
- ⚠️ **Wire-color → pin mapping.** Download the cable datasheet (PDF on the Conrad page) and confirm colors match the manual's table (Red=GND, Gray=POWER, Blue=TO0, Pink=TO1, Yellow=TI0, Green=TI1, White=AI2/RS485+, Brown=AI3/RS485-). Third-party color maps can differ from UR's table — verify, don't assume.

**Status:** candidate, pending the two checks. Sibling parts on same page: RSM 8-354/2M (plug, 1234964); 5 m lengths (1234966); 10-packs (1718775).

## First real-arm move — prepared (2026-06-16)

- **PolyScope safety config done in lab:** wall plane + table plane set (authoritative geometry now in place). **Not yet set:** reduced TCP speed/force limits → controller does not cap speed; software profile + speed slider + e-stop are the speed guard for now. **TODO:** add a reduced TCP speed safety limit in PolyScope so the authoritative layer covers speed too.
- **`scripts/first_real_move.py`** (internal bring-up tool, supervised): small validated Cartesian hop (+Z 40 mm, then back) at 50 mm/s through `MotionController` with the real-arm gate. Avoids the bare `smoke_test_move.py`, which bypasses the safety layer and must not be pointed at hardware.
- **Grounding trick:** reads the arm's LIVE TCP pose and builds a tight workspace box around that measured point (`validated=True` because derived from reality) — no hard-coded cell coordinates needed. Declares zero payload/TCP (no tool installed). Requires typed confirmation before moving. Logic verified in sim (gate passes, correct slow target).
- Still outstanding for the persistent `cart_against_wall` profile: measure surface extents + cart-top height, replace placeholder box coords, set `validated=True`.

## Cell measured + PolyScope robot limits (2026-06-16)

**Measured cell:** work surface ~**600 × 500 mm** (wide × depth) in front; **table at mounting height (z = 0** in base frame); no tool.

**PolyScope robot limits — factory "most restrictive" preset applied:** power 80 W, momentum 5.0 kg·m/s, stopping time 200 ms, stopping distance 200 mm, **tool speed 250 mm/s**, tool force 100 N, elbow speed 250 mm/s, elbow force 100 N. (Authoritative dynamic limits now set, in addition to the wall + table planes.)

**Profile update (`cart_against_wall`):** box set to measured size `x=(0.0,0.50), y=(-0.30,0.30), z=(0.0,0.40)`; `tcp_speed_max=0.15 m/s` stays under the controller's 250 mm/s tool-speed limit (software stricter than controller ✓). **`validated=False`** still — the box SIZE is measured but base-frame PLACEMENT (front=+X vs −X, exact offset) is assumed.

**`scripts/teach_workspace.py` (new):** freedrive capture — hand-guide the TCP to the real corners + highest working point, prints the exact `Box(...)` to paste. Removes placement guesswork; after pasting, set `validated=True` to unlock the full profile for real motion. Meanwhile `first_real_move.py` (live-pose box) remains the safe mover.

**Software speed guard note:** with the controller's tool-speed limit now at 250 mm/s, the speed-guard gap from the previous session is closed at the controller level; software stays stricter at 150 mm/s.
