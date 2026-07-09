/*
 * Toolhead RGB + button -> RS485 (UR5e Tool Communication Interface / TCI)
 * Board:  SparkFun ESP32-S3 Thing Plus (3.3 V)        [chosen by JReck 2026-06-30]
 * Sensor: APDS9960 over Qwiic (I2C) - no sensor wiring, just the cable.
 * STATUS: experimental / internal bring-up firmware. NOT student-validated.
 *
 * WHAT IT DOES
 *   Reads an APDS9960 (RGB) over the Qwiic I2C bus and a momentary push button,
 *   and streams one ASCII CSV line per sample over RS485:
 *
 *       R,G,B,C,BTN\n
 *
 *   R,G,B,C  16-bit APDS9960 channels (red, green, blue, clear)   [uint16]
 *   BTN      1 while the button is pressed, else 0
 *
 *   ~20 Hz, line-framed on '\n'. The host receives these bytes from the UR
 *   controller's TCI forwarder on TCP <robot_ip>:54321
 *   (see Documentation/10-toolhead-rgb-rs485-interface.md).
 *
 * WHY THE ESP32-S3 IS NICE HERE
 *   - Flexible GPIO mux: RS485 gets its own hardware UART (Serial1) on any free
 *     pins, the button sits on any GPIO - no pin conflicts.
 *   - Native USB: Serial (USB CDC) is independent of Serial1, so you can watch
 *     frames over USB WHILE RS485 runs (set DEBUG_USB below).
 *   - 3.3 V logic: pairs directly with the 3.3 V APDS9960 and a 3.3 V RS485
 *     transceiver - no level shifting.
 *   - We do NOT use WiFi (RS485/TCI is the channel), so there are no 500 mA
 *     WiFi current peaks to budget for.
 *
 * WIRING
 *   Sensor   Qwiic cable: ESP32-S3 Thing Plus Qwiic <-> APDS9960 Qwiic/STEMMA QT
 *            (carries 3.3 V, GND, SDA, SCL - nothing to solder)
 *   Button   GPIO4 --[btn]--> GND          INPUT_PULLUP, pressed = LOW
 *   RS485    GPIO5 (TX) -> MAX3485 DI
 *            GPIO6 (RX) -> MAX3485 RO       (wired but unused; one-way link)
 *            MAX3485 DE + RE -> 3V3         (driver always enabled = transmit only)
 *            MAX3485 A -> tool White (RS485+)
 *            MAX3485 B -> tool Brown (RS485-)
 *   Power    tool 12 V (Gray) -> buck (e.g. MP1584) -> 3.3 V rail ->
 *            ESP 3V3 pin + MAX3485 VCC (Qwiic sensor is fed from the board's 3V3)
 *            tool GND (Red) -> common GND (flange is GND too)
 *            [alt: buck to 5 V -> ESP VUSB pin; the onboard LDO makes 3.3 V]
 *            DO NOT feed 12 V to VUSB/3V3 - the onboard regulator would cook.
 *
 *   Pin notes: GPIO4/5/6 are defaults - any free broken-out GPIO works. Avoid
 *   strapping pins (0/3/45/46), USB (19/20), the Qwiic I2C pins (8/9), and the
 *   microSD pins if you use that slot. Update the constants below to match your
 *   wiring.
 *
 * ARDUINO IDE
 *   Board: "SparkFun ESP32-S3 Thing Plus" (maps Qwiic to the default Wire bus).
 *   Tools -> USB CDC On Boot: Enabled  (so the USB Serial Monitor works).
 *   Library: "Adafruit APDS9960".
 *
 * UR SIDE (PolyScope, Installation tab -> Tool Communication Interface)
 *   Enable TCI. Set: baud 19200, parity none, stop bits 1.   <-- MUST match BAUD.
 *   Tool Output Voltage = 12 V. (Enabling TCI disables tool analog AI2/AI3.)
 *   Install the "rs485" ToolComm Forwarder URCap so the stream reaches :54321.
 */

#include <Wire.h>
#include "Adafruit_APDS9960.h"

#define DEBUG_USB 1                 // 1 = also mirror frames to USB Serial Monitor

const int      PIN_BTN    = 4;      // button -> GND, INPUT_PULLUP, active-low
const int      PIN_RS_TX  = 5;      // -> MAX3485 DI
const int      PIN_RS_RX  = 6;      // -> MAX3485 RO (unused, one-way)
const long     BAUD       = 19200;  // MUST equal the UR TCI baud (UART can do more)
const uint16_t PERIOD_MS  = 50;     // ~20 Hz sample/stream rate

Adafruit_APDS9960 apds;

bool     sensorOk = false;
uint16_t r = 0, g = 0, b = 0, c = 0;   // last-known color (resent if not ready)

void setup() {
#if DEBUG_USB
  Serial.begin(115200);             // USB CDC - independent of the RS485 UART
#endif
  pinMode(PIN_BTN, INPUT_PULLUP);

  Serial1.begin(BAUD, SERIAL_8N1, PIN_RS_RX, PIN_RS_TX);   // RS485 hardware UART

  Wire.begin();                     // Qwiic I2C (SparkFun board maps the pins)
  sensorOk = apds.begin();
  if (sensorOk) {
    apds.enableColor(true);         // start RGBC engine (default gain/integration)
  }
  // Sensor absent -> still stream zeros + the button, so the link stays testable.
}

void loop() {
  static uint32_t last = 0;
  uint32_t now = millis();
  if (now - last < PERIOD_MS) return;
  last = now;

  if (sensorOk && apds.colorDataReady()) {
    apds.getColorData(&r, &g, &b, &c);   // refresh; otherwise resend last values
  }

  uint8_t btn = (digitalRead(PIN_BTN) == LOW) ? 1 : 0;   // pull-up: pressed = LOW

  char line[40];
  int n = snprintf(line, sizeof(line), "%u,%u,%u,%u,%u\n", r, g, b, c, btn);
  Serial1.write((const uint8_t *)line, n);   // -> RS485 -> robot -> :54321
#if DEBUG_USB
  Serial.write((const uint8_t *)line, n);    // -> USB Serial Monitor (bench)
#endif
}
