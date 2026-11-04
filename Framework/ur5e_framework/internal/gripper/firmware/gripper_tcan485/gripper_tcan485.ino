/*
 * Gripper v2 firmware — LilyGO T-CAN485 (ESP32) bridging the UR5e tool RS485
 * to one Lynxmotion Smart Servo (LSS) over TTL serial.
 *
 * STATUS: experimental / internal. Not validated. Bench-test on USB first.
 *
 * WIRING (see Documentation/15 and internal/gripper/ref/pinmap.jpg)
 *   UR tool cable  -> T-CAN485
 *     grey  POWER 12 V -> "POWER DC" +      (PolyScope: Tool I/O voltage = 12 V)
 *     red   GND        -> "POWER DC" GND
 *     white RS485+     -> RS485 terminal "RS_A"
 *     brown RS485-     -> RS485 terminal "RS_B"
 *   LSS servo (4-pin Molex: GND, VCC, Rx, Tx)
 *     VCC -> same 12 V as the board's DC-in (+)   GND -> DC-in GND (common ground)
 *     Rx  <- header IO32  (SERVO_TX)              3.3 V TTL, normally read as high by the LSS
 *     Tx  -> header IO33  (SERVO_RX)              MEASURE the LSS Tx idle level first: if it is
 *                                                 5 V, use a 1k/2k divider (ESP32 is not 5 V tolerant)
 *   Board RS485 transceiver (MAX13487E, auto direction): UART2 on IO21 (RX) / IO22 (TX),
 *   enable pins IO17 + IO19 driven HIGH.  WS2812 status LED on IO04.
 *
 * HOST PROTOCOL (ASCII lines over RS485 / TCI, HOST_BAUD 8N1 — set the same in
 * PolyScope > Installation > Tool I/O > Communication Interface):
 *   G<pct>      go to opening 0 (closed) .. 100 (open), e.g. "G35"
 *   O / C       open / close  (= G100 / G0)
 *   F<mA>       holding-current limit used for moves (grip force), stored
 *   V<deg/s>    speed limit in deg/s, stored
 *   L / H       limp (servo free, for calibration) / halt & hold
 *   S           status
 *   CAL MIN     store current servo angle as CLOSED   (jaws pushed together while limp) — needs servo Tx
 *   CAL MAX     store current servo angle as OPEN     (jaws pushed fully apart while limp) — needs servo Tx
 *   CAL RESET   back to the geometric defaults (uncalibrated)
 *   #<lss cmd>  raw pass-through to the servo (debug): "#0LED3" blue LED, "#0D300" move, "#0QID" query id
 *   T<tenths>   raw move to a servo angle (e.g. "T-200"), to find the limits WITHOUT feedback
 *   R<tenths>   RELATIVE move from the current position (e.g. "R100" = +10 deg) — works with no
 *               calibration and no feedback: set the jaws by hand while limp, then +/-N deg
 *   MIN<tenths> / MAX<tenths>   set the CLOSED / OPEN angle directly (open-loop calibration)
 * Every command is answered with one status line:
 *   *pct=<0-100>,deg=<tenths>,mA=<current>,target=<pct>,fb=<0|1>,cal=<0|1>,closed=<t>,open=<t>,f=<mA>,v=<deg/s>\n
 * fb=0 means the servo does not answer (Tx not wired) -> open loop: pct echoes the target, deg=-9999.
 *
 * LSS PROTOCOL used (115200 8N1, ID = LSS_ID below):  #<id>D<tenths>SD<0.1deg/s>CH<mA>\r  move,
 *   #<id>QD\r position, #<id>QC\r current, #<id>L\r limp, #<id>H\r hold.  Angles in tenths of a degree.
 *   Replies start with '*' followed by the servo's real ID, so lssQuery() parses on the command letters only.
 */

#include <Arduino.h>
#include <Preferences.h>
#include <Adafruit_NeoPixel.h>

// ---- pins (T-CAN485 pinmap) --------------------------------------------
#define RS485_RX 21
#define RS485_TX 22
#define RS485_EN 17
#define RS485_SE 19
#define PIN_5V_EN 16  // ME2107 boost enable: powers the 5 V RS485 transceiver — MUST be HIGH or RS485 is dead
#define SERVO_TX 32   // -> LSS Rx
#define SERVO_RX 33   // <- LSS Tx
#define LED_PIN  4

// ---- settings -------------------------------------------------------------
#define HOST_BAUD   115200
#define LSS_BAUD    115200
#define LSS_ID      254   // broadcast: the one servo on this bus answers whatever its stored ID is
                          // (bench test 2026-11-03: servo ignored #0 and #1, obeyed #254). With a single
                          // servo this is fine for queries too; never use it with several servos chained.
// geometric defaults, tenths of a degree, valid when the LSS origin offset (#0CO)
// puts the "links straight ahead" pose at 0 deg: closed = -20 deg, open = +32 deg
#define DEFAULT_CLOSED  (-200)
#define DEFAULT_OPEN    (320)
#define DEFAULT_FORCE_MA 350
#define DEFAULT_SPEED_DPS 90

HardwareSerial Host(2);    // RS485 <-> UR controller (TCI)
HardwareSerial Servo(1);   // TTL   <-> LSS
Preferences prefs;
Adafruit_NeoPixel led(1, LED_PIN, NEO_GRB + NEO_KHZ800);

long closedT = DEFAULT_CLOSED, openT = DEFAULT_OPEN;
int forceMa = DEFAULT_FORCE_MA, speedDps = DEFAULT_SPEED_DPS;
bool calibrated = false;
int targetPct = -1;
// OPEN LOOP: if the servo Tx line is not connected (or the servo never answers),
// queries time out. We then stop querying (re-probe every 5 s), report the target
// as the position and flag fb=0 in the status line. Commands still work.
bool feedbackOk = false;
uint32_t lastProbeMs = 0;

// ---- LSS helpers ------------------------------------------------------------
void lssSend(const String& cmd) {
  Servo.print('#'); Servo.print(LSS_ID); Servo.print(cmd); Servo.print('\r');
}

// query like "QD" -> value after the echoed command, or `def` on timeout
long lssQuery(const char* q, long def, uint32_t timeoutMs = 60) {
  while (Servo.available()) Servo.read();
  lssSend(q);
  String rx; uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    while (Servo.available()) {
      char c = Servo.read();
      if (c == '\r') {
        int i = rx.indexOf(q);            // "*0QD1234"
        if (rx.startsWith("*") && i >= 0) return rx.substring(i + strlen(q)).toInt();
        rx = "";
      } else rx += c;
    }
  }
  return def;
}

long pctToTenths(int pct) { return closedT + (openT - closedT) * (long)pct / 100; }
int tenthsToPct(long t) {
  long span = openT - closedT; if (span == 0) return 0;
  long p = (t - closedT) * 100 / span; return (int)constrain(p, 0, 100);
}

void moveTo(int pct) {
  pct = constrain(pct, 0, 100);
  targetPct = pct;
  lssSend("D" + String(pctToTenths(pct)) + "SD" + String(speedDps * 10) + "CH" + String(forceMa));
}

// ---- persistence --------------------------------------------------------
void loadCal() {
  prefs.begin("grip", true);
  closedT = prefs.getLong("closed", DEFAULT_CLOSED);
  openT = prefs.getLong("open", DEFAULT_OPEN);
  forceMa = prefs.getInt("force", DEFAULT_FORCE_MA);
  speedDps = prefs.getInt("speed", DEFAULT_SPEED_DPS);
  calibrated = prefs.getBool("cal", false);
  prefs.end();
}
void saveCal() {
  prefs.begin("grip", false);
  prefs.putLong("closed", closedT); prefs.putLong("open", openT);
  prefs.putInt("force", forceMa); prefs.putInt("speed", speedDps);
  prefs.putBool("cal", calibrated);
  prefs.end();
}

// ---- status -------------------------------------------------------------
// probe the servo for feedback (only costs the timeout when it is absent)
bool probeFeedback() {
  if (feedbackOk || millis() - lastProbeMs > 5000) {
    lastProbeMs = millis();
    feedbackOk = (lssQuery("QD", -9999) != -9999);
  }
  return feedbackOk;
}

void sendStatus() {
  long deg = -9999, ma = -1;
  int pct;
  if (probeFeedback()) {
    deg = lssQuery("QD", -9999);
    ma = lssQuery("QC", -1);
    pct = (deg == -9999) ? -1 : tenthsToPct(deg);
  } else {
    pct = targetPct;                         // open loop: assume we got there
  }
  String s = "*pct=" + String(pct) + ",deg=" + String(deg) + ",mA=" + String(ma) +
             ",target=" + String(targetPct) + ",fb=" + String(feedbackOk ? 1 : 0) +
             ",cal=" + String(calibrated ? 1 : 0) + ",closed=" + String(closedT) +
             ",open=" + String(openT) + ",f=" + String(forceMa) + ",v=" + String(speedDps) + "\n";
  Host.print(s);      // to the robot (RS485 / TCI)
  Serial.print(s);    // mirror on USB for bench tests
  // LED: green = feedback + calibrated, yellow = uncalibrated, magenta = open loop (no servo Tx)
  led.setPixelColor(0, !feedbackOk ? led.Color(30, 0, 30) : (calibrated ? led.Color(0, 30, 0) : led.Color(30, 20, 0)));
  led.show();
}

// ---- host command handling ------------------------------------------------
void handle(String line) {
  line.trim(); line.toUpperCase();
  if (line.length() == 0) return;
  if (line.startsWith("#")) {              // raw LSS pass-through for debugging, e.g. "#0LED3", "#0D300", "#0QID"
    Servo.print(line); Servo.print('\r');
    Serial.println("-> servo: " + line + "\\r");
    delay(60);
    String rx; while (Servo.available()) rx += (char)Servo.read();
    Serial.println(rx.length() ? "<- servo: " + rx : "<- servo: (no answer)");
    return;
  }
  if (line.startsWith("CAL")) {            // hand calibration — needs servo feedback (Tx line)
    if (line.endsWith("MIN") && probeFeedback()) { closedT = lssQuery("QD", closedT); calibrated = true; saveCal(); }
    else if (line.endsWith("MAX") && probeFeedback()) { openT = lssQuery("QD", openT); calibrated = true; saveCal(); }
    else if (line.endsWith("RESET")) { closedT = DEFAULT_CLOSED; openT = DEFAULT_OPEN; calibrated = false; saveCal(); }
  } else if (line.startsWith("MIN")) {     // set the CLOSED angle directly (tenths of a degree) — open-loop calibration
    closedT = line.substring(3).toInt(); calibrated = true; saveCal();
  } else if (line.startsWith("MAX")) {     // set the OPEN angle directly
    openT = line.substring(3).toInt(); calibrated = true; saveCal();
  } else {
    char c = line[0];
    long v = line.substring(1).toInt();
    switch (c) {
      case 'G': moveTo((int)v); break;
      case 'T': targetPct = -1;             // raw move to a servo angle in tenths of a degree (for finding min/max)
                lssSend("D" + String(v) + "SD" + String(speedDps * 10) + "CH" + String(forceMa)); break;
      case 'R': targetPct = -1;             // RELATIVE move in tenths of a degree from wherever the servo is now
                lssSend("MD" + String(v) + "SD" + String(speedDps * 10) + "CH" + String(forceMa)); break;
      case 'O': moveTo(100); break;
      case 'C': moveTo(0); break;
      case 'F': forceMa = constrain((int)v, 50, 1200); saveCal(); break;
      case 'V': speedDps = constrain((int)v, 5, 360); saveCal(); break;
      case 'L': lssSend("L"); targetPct = -1; break;
      case 'H': lssSend("H"); break;
      case 'S': break;
      default: break;   // unknown -> just answer with status
    }
  }
  sendStatus();
}

void setup() {
  pinMode(PIN_5V_EN, OUTPUT); digitalWrite(PIN_5V_EN, HIGH);   // 5 V rail for the RS485 transceiver
  pinMode(RS485_EN, OUTPUT); digitalWrite(RS485_EN, HIGH);
  pinMode(RS485_SE, OUTPUT); digitalWrite(RS485_SE, HIGH);
  delay(50);
  Host.begin(HOST_BAUD, SERIAL_8N1, RS485_RX, RS485_TX);
  Servo.begin(LSS_BAUD, SERIAL_8N1, SERVO_RX, SERVO_TX);
  Serial.begin(115200);                    // USB debug mirror
  led.begin(); led.setPixelColor(0, led.Color(0, 0, 30)); led.show();
  loadCal();
  delay(300);
  lssSend("L");                            // boot: servo stays limp until the host commands it
  Serial.println("gripper_tcan485 ready");
}

String hostLine;
void loop() {
  while (Host.available()) {
    char c = Host.read();
    if (c == '\n' || c == '\r') { if (hostLine.length()) { Serial.println("> rs485: " + hostLine); handle(hostLine); } hostLine = ""; }
    else if (hostLine.length() < 32) hostLine += c;
  }
  // USB serial mirrors the host protocol for bench tests
  static String usbLine;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') { if (usbLine.length()) { handle(usbLine); } usbLine = ""; }
    else if (usbLine.length() < 32) usbLine += c;
  }
}
