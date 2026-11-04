# Gripper v2 — wiring (T-CAN485 + LSS servo on the UR5e tool connector)

Experimental / internal. Pin names from `ref/pinmap.jpg` (LilyGO T-CAN485 V1.0).

## UR tool cable (Lumberg RKMV 8-354) -> T-CAN485

| wire  | UR signal      | T-CAN485                        |
| ----- | -------------- | ------------------------------- |
| grey  | POWER (12 V)   | `POWER DC` **+**                |
| red   | GND            | `POWER DC` **GND**              |
| white | RS485+ (AI2)   | RS485 terminal **`RS_A`**       |
| brown | RS485- (AI3)   | RS485 terminal **`RS_B`**       |
| —     | —              | RS485 terminal `GND` -> same GND (optional, keeps the pair referenced) |

PolyScope: Installation > General > Tool I/O: voltage **12 V**, Communication
Interface **on**, 115200 baud, 8 data bits, no parity, 1 stop bit (must match
`HOST_BAUD` in the firmware). Enabling TCI disables the tool analog inputs.
ST1 servo: single-pin power is enough; for an HT1 use Dual-Pin power.

Board internals you don't wire but the firmware uses: RS485 UART on IO21 (RX)
/ IO22 (TX), transceiver enables IO17 + IO19 (driven HIGH), WS2812 LED IO04.
The CAN terminal stays unused.

## LSS servo -> T-CAN485 (signal on the 12-pin header)

LSS connector (both sides, pin order reversed side to side): GND, VCC, Rx, Tx.

| LSS pin | goes to                                   | note                                             |
| ------- | ----------------------------------------- | ------------------------------------------------ |
| VCC     | 12 V, same node as the board's `POWER DC +` | LSS wants 6–12.6 V; 12 V recommended           |
| GND     | `POWER DC GND` / header `GND`               | common ground with the board                   |
| Rx      | header **IO32**  (firmware `SERVO_TX`)      | 3.3 V TTL out of the ESP32; LSS reads it as high |
| Tx      | header **IO33**  (firmware `SERVO_RX`)      | **measure first**: if the LSS Tx idles at 5 V, put a 1k (series) / 2k (to GND) divider in front of IO33 — ESP32 inputs are not 5 V tolerant |

Why IO32/IO33: they are two of the "8 reserved GPIO" on the header (GND, IO25,
IO32, IO33, IO05, IO12, IO34, IO35, IO18, VDD) and are plain in/out pins. Do not
use IO34/IO35 for TX (input-only) and avoid IO12 (boot strap). The firmware
opens them as a second hardware UART (`Serial(1)`, 115200 8N1 = LSS default).

The servo's serial is **TTL full duplex, not RS485** — it does not go on the
board's RS485 terminal; that terminal is the link to the robot.

## Command path

host (Python, `ur5e.toolio.gripper`) -> TCP `<robot>:54321` (rs485 URCap) ->
tool RS485 -> T-CAN485 `RS_A/RS_B` -> firmware -> IO32/IO33 TTL -> LSS.

## Flashing (Arduino IDE)

LilyGO's own settings (`ref/arduino_setting.png`): Boards Manager → install
**esp32 by Espressif** (URL `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`),
then **Board: "ESP32 Dev Module"**, Upload Speed 921600, CPU 240 MHz, Flash
80 MHz / QIO / 4 MB, Partition "Default 4MB with spiffs", PSRAM Disabled.
Library Manager → **Adafruit NeoPixel** (Preferences ships with the core).
USB-C uses a CH9102 bridge: recent macOS has the driver; otherwise LilyGO's
`CH9102_Mac_Driver`. If the upload does not start, hold BOOT while it connects.

TX/RX naming, to avoid the classic swap: the header pins are only labelled by
IO number; the firmware makes **IO32 the board's TX** (`Servo.begin(…, RX=33,
TX=32)`) and **IO33 the board's RX**. Serial lines cross: board TX → servo Rx,
servo Tx → board RX, i.e. **servo Tx → IO33**, **servo Rx → IO32**. The pins
printed RS485_TX/RX (IO21/22) belong to the RS485 transceiver — leave them.

## Bench test without the robot

USB-C to the Mac, 12 V bench supply on `POWER DC` (+ the servo), serial monitor at
115200: type `S`, `O`, `C`, `G50`, `CAL MIN`, ... — the firmware mirrors the host
protocol on USB. LED: blue = booting, yellow = servo ok but uncalibrated, green =
calibrated, red = no answer from the servo.
