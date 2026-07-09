# Toolhead RGB + Button over RS485 / TCI (2026-06-30) 🔬

Research/internal. First instrumented-toolhead bring-up: an Arduino on the tool
head reads an **Adafruit APDS9960** RGB sensor (I2C) and a **push button**, and
sends the data as **digital RS485** over the UR5e tool flange. The host receives
the same stream in two consumers at once — a **live visualisation** and the
**motion example** — and on each button press maps R/G/B to base-frame X/Y/Z
offsets to nudge the arm.

**Status:** experimental / internal. NOT validated for student use.

## Decision: how the tool RS485 reaches the host (was an open thread)

The 09-tool-io doc flagged `RS485 / TCI → controller bridge, not in current
facade. 🔬 separate design thread`. **Resolved:**

- UR provides the **Tool Communication Forwarder URCap** (`rs485-1.0.urcap` for
  PolyScope 5; a URCapX for PolyScope X). Installed on the controller, it
  forwards the tool RS485 to **TCP `<robot_ip>:54321`**.
- Two host access patterns:
  1. `socat pty,link=/tmp/ttyUR,raw,ignoreeof,waitslave tcp:<robot_ip>:54321`
     → a local serial device, opened with pyserial. (ROS-driver pattern.)
  2. **Open a plain TCP socket to `<robot_ip>:54321` and read the framed bytes
     directly** — no socat, no `/dev` permissions. **Chosen** (simpler, fewer
     moving parts). Implemented in `TciStream`.

Sources: UR ROS driver tool-comm setup; `Universal_Robots_ToolComm_Forwarder_URCap`.

## Parity caveat (important)

This link is **real-arm only**. URSim has no tool RS485, so it has nothing on
:54321 — this feature **breaks the sim↔real parity** the framework otherwise
keeps. Mitigation: a **`MockToolSource`** generates synthetic frames so the host
side (parser, visualisation, button/motion logic) is fully developable offline,
and the motion half can even run against URSim with `--source mock --target sim`.

## Wire frame

One ASCII line per sample, line-framed on `\n`, ~20 Hz:

```
R,G,B,C,BTN\n      e.g.  18233,9044,512,27800,1
```

R,G,B,C = 16-bit APDS9960 channels; BTN = 1 pressed / 0 released. ASCII CSV was
chosen over binary for debuggability and easy resync (skip any line that isn't
5 integers). Parsed by `parse_line()` into `ToolFrame`.

## Toolhead hardware (chosen 2026-06-30)

- **Board: SparkFun ESP32-S3 Thing Plus (3.3 V).** Switched from the Uno: the
  S3's flexible GPIO mux gives RS485 its own hardware UART (`Serial1` on GPIO5/6)
  and the button any GPIO (GPIO4) — no pin conflicts. Native USB makes the USB
  Serial Monitor independent of the RS485 UART, so frames can be watched over USB
  *while* RS485 runs. RS485 is **one-way** (tool→robot); transceiver driver held
  enabled (DE/RE→3V3). We do **not** use WiFi, so there are no 500 mA WiFi peaks.
- **Sensor: APDS9960 over Qwiic** — the Qwiic cable carries 3.3 V/GND/SDA/SCL, so
  there is no sensor wiring and no level shifting (S3 is native 3.3 V).
- **Transceiver: 3.3 V (MAX3485 / SP3485)**, not a 5 V MAX485.
- **Power:** tool **12 V** (Gray) → small buck (MP1584) → **3.3 V** into the 3V3
  pin (one rail feeds ESP + MAX3485 + Qwiic sensor); alt buck → 5 V into VUSB.
  **Never 12 V to VUSB/3V3** (onboard LDO would overheat). ~150 mA total,
  well inside the ~12 W @12 V budget and < 500 g envelope.
- **RS485 pair:** MAX3485 A→tool **White (RS485+)**, B→tool **Brown (RS485-)**.
  Swap if the host sees garbled/no data (polarity).

Firmware + wiring table: `Framework/ur5e_framework/toolhead/examples/rgb_button_rs485/`.

## UR-side preconditions (manual, one-time in PolyScope)

1. Installation → **Tool Communication Interface**: enable; **baud 19200,
   parity none, 1 stop bit** (must match the firmware `BAUD`).
2. Installation → **Tool Output voltage = 12 V**.
   (Enabling TCI **disables tool analog in AI2/AI3** — not used here.)
3. Install the **rs485 ToolComm Forwarder URCap** (exposes :54321).
4. For motion: robot in **Remote Control** (RTDE control needs it) — independent
   of the :54321 read path.

## Host software (new)

- `src/ur5e/toolio/tci_rs485.py` — `ToolFrame`, `parse_line`,
  `map_rgb_to_offset`, `ButtonEdge`, `TciStream` (live, :54321), and
  `MockToolSource` (offline). No `ur_rtde` import — pure and unit-testable.
- `examples/toolhead_rgb_motion.py` — one process, **both consumers at once**:
  a background thread reads frames and (a) updates a **live matplotlib viz**
  (RGB swatch + R/G/B bars + button state) and (b) on each button **rising
  edge** steps the robot. Single TCP client (the forwarder serves one), fanned
  out in-process.
- `scripts/toolhead_rgb_selftest.py` — offline checks (no hardware/ur_rtde).
  **All checks pass** (parse, mapping ranges, button edges, mock, cumulative
  clamp).

## Motion mapping (chosen: cumulative)

`r→x, g→y, b→z` in the **base frame**, orientation held from the start pose.
Each channel is normalised against the clear channel C; default magnitude
**5–30 mm** per axis (`min_mm + norm·(max_mm−min_mm)`). On each press the TCP
steps **cumulatively** from its current pose, then is **clamped into a safe box
±`span_mm` (default 40 mm) around the start pose** so it cannot walk away.

- Note: cumulative + positive magnitudes march toward the +box corner and clamp.
  Flags expose alternatives without code changes: `--mode absolute`
  (offset from the fixed start pose) and `--signed` (dim→−, bright→+, ±max_mm).
- Speed default **50 mm/s**, accel 0.2 — inside the `cart_against_wall`
  150 mm/s software cap and the controller's 250 mm/s tool-speed limit.

## Safety gating (unchanged model, just applied)

The existing model is the backstop — **PolyScope Safety Configuration is
authoritative**; the software `SafetyLimits` profile is the secondary guard.
Real motion still requires **all** of: non-sim host, `--allow-real`, a
**`validated=True`** profile, **payload set** (`--payload`), and **TCP set**
(`--tcp-z`). `move_to` validates-and-rejects out-of-box / over-speed targets
(rejection is surfaced in the viz, motion skipped). Sim is the default.

**Assumptions flagged:** base-frame XYZ only; toolhead payload + TCP are
caller-declared (the Arduino+sensor tool, < 500 g); `cart_against_wall` is still
`validated=False` until the box placement is taught on hardware — so real motion
remains gated exactly as before.

## How to run

```bash
# offline, visualisation only (no robot, no ur_rtde needed):
python examples/toolhead_rgb_motion.py --source mock --no-motion

# offline full pipeline against URSim:
python examples/toolhead_rgb_motion.py --source mock --target sim

# live lab run (real arm + real toolhead), supervised:
python examples/toolhead_rgb_motion.py --source tci --robot-ip <IP> \
    --target real --allow-real --payload 0.3 --tcp-z 0.06

# offline self-test:
python scripts/toolhead_rgb_selftest.py
```

## Open items / next on hardware

- Confirm RS485 **A/B polarity** at the flange (White/Brown) on first power-up.
- Confirm APDS9960 **integration time / gain** give a useful dynamic range in
  the studio lighting; tune `full_scale` / normalisation if values saturate.
- Validate the **:54321 socket** delivers frames once the URCap is installed
  (first end-to-end real-arm test, receive-only before enabling motion).
- Decide whether to promote `read_tool_analog_in()` etc. — **N/A while TCI is
  on** (analog inputs disabled); revisit only if a non-TCI tool is used.
- If two *separate* processes must consume the stream, add a small local
  re-broadcast (UDP) fan-out (single TCI client limitation).
