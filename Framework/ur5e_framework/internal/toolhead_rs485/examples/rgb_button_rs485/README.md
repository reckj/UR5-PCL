# Toolhead: APDS9960 RGB + button -> RS485 (TCI)

**Status:** experimental / internal bring-up. Not student-validated.

Firmware for the first instrumented toolhead: a **SparkFun ESP32-S3 Thing Plus**
reads an **APDS9960** RGB sensor over **Qwiic** (I2C) and a momentary **push
button**, and streams the data as RS485 over the UR5e tool flange. The UR
controller forwards that RS485 to the host PC on TCP `:54321` via the Tool
Communication Interface.

## Frame format

One ASCII line per sample, line-framed on `\n`, ~20 Hz:

```
R,G,B,C,BTN\n
```

| Field | Meaning                          | Type   |
|-------|----------------------------------|--------|
| R     | APDS9960 red channel             | uint16 |
| G     | APDS9960 green channel           | uint16 |
| B     | APDS9960 blue channel            | uint16 |
| C     | APDS9960 clear channel           | uint16 |
| BTN   | 1 = pressed, 0 = released (GPIO4)| 0/1    |

## Wiring

**Sensor — Qwiic cable only:** ESP32-S3 Thing Plus Qwiic ⟷ APDS9960 Qwiic /
STEMMA QT. Carries 3.3 V, GND, SDA, SCL — nothing to solder.

| Tool wire (UR §5.8) | Signal     | Goes to                                   |
|---------------------|------------|-------------------------------------------|
| Gray                | POWER 12 V | buck (MP1584) input                       |
| Red                 | GND        | common GND (flange is GND)                |
| White               | RS485+     | MAX3485 **A**                             |
| Brown               | RS485-     | MAX3485 **B**                             |

| ESP32-S3   | To                                              |
|------------|-------------------------------------------------|
| GPIO4      | button -> GND (`INPUT_PULLUP`, pressed = LOW)    |
| GPIO5 (TX) | MAX3485 **DI**                                  |
| GPIO6 (RX) | MAX3485 **RO** (wired, unused - link is one-way)|
| 3V3        | MAX3485 VCC, buck 3.3 V out (and feeds Qwiic)   |
| GND        | MAX3485 GND, buck GND, tool Red                 |

MAX3485 **DE + RE -> 3V3** (driver always enabled; transmit-only).

> **Transceiver must be 3.3 V** (MAX3485 / SP3485), not a 5 V MAX485 module —
> the ESP32-S3 is 3.3 V logic. Same A/B → White/Brown wiring.

### Power

Use a small buck (e.g. **MP1584**) from the tool's 12 V. **Do not feed 12 V to
VUSB or 3V3** — the onboard regulator would overheat.

- **Recommended:** buck **12 V → 3.3 V** into the **3V3** pin. One rail powers
  the ESP, the MAX3485, and the Qwiic sensor. Coolest/simplest (bypasses the
  onboard LDO). Don't also power over USB while injecting 3.3 V.
- **Alt:** buck **12 V → 5 V** into the **VUSB** pin; the onboard LDO makes 3.3 V.

We don't use WiFi (RS485 is the channel), so there are no 500 mA WiFi peaks —
~150 mA total, well inside the tool's ~12 W @ 12 V budget and < 500 g envelope.

### Pin notes

GPIO4/5/6 are defaults — any free broken-out GPIO works. Avoid strapping pins
(0/3/45/46), USB (19/20), the Qwiic I2C pins (8/9), and the microSD pins if you
use that slot. Edit the constants in the sketch to match your wiring.

## UR side (one-time, in PolyScope)

1. Installation tab -> **Tool Communication Interface**: enable, set
   **baud 19200, parity none, stop bits 1** (must match `BAUD` in the sketch).
2. Installation tab -> **Tool Output**: voltage **12 V**.
   (Enabling TCI disables tool analog inputs AI2/AI3 - not used here.)
3. Install the **rs485 ToolComm Forwarder URCap** so the stream is forwarded to
   TCP `:54321` on the host. See
   `../../../../Documentation/10-toolhead-rgb-rs485-interface.md`.

## Arduino IDE

- Board: **SparkFun ESP32-S3 Thing Plus** (maps Qwiic to the default `Wire`).
- Tools → **USB CDC On Boot: Enabled** (so the USB Serial Monitor works).
- Library Manager: **Adafruit APDS9960**.

## Caveats

- Native USB means you can watch frames over the **USB Serial Monitor (115200)**
  while RS485 runs — handy for bench testing (`DEBUG_USB` in the sketch).
- RS485 polarity: if the host sees no/garbled data, swap White/Brown (A/B).
- This link exists **only on the real arm** - URSim has no tool RS485. Develop
  the host side offline with the `mock` source (see the Python example).
