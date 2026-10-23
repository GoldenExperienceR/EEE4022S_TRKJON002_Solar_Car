// Test — Pan + tilt tracking at the same time. Arduino Nano, Serial 115200, "Newline".
// = U-15_track_pan and U-15_track_tilt merged: each axis runs its own controller every 100 ms.
// LDRs: TL A0, TR A1, BL A2, BR A3. Servos: pan D9, tilt D10.
// POWER: both servos can now move together (~0.5-0.8 A, up to ~1.2 A stalled) -> servo V+ from Buck A 5 V
//        with a 470-1000 uF cap at the servos, NOT the Nano 5V pin. Common GND.
// Each axis: starts when |difference| > x, stops below x/2 (hysteresis; was x/3 — stays still with the LDRs
// a little less balanced, 19/10), stepDeg per step toward the light;
// balanced for 2 s -> that servo is released (no holding current) and its angle saved to EEPROM.
// Start-up: both servos attached AT their last saved angles (no jump), 3 s wait.
// Commands: x NN = deadband both axes (default 30) | s N = degrees per step both axes (default 3)
//           ip / it = invert pan / tilt | gp NNN / gt NNN = move pan / tilt slowly to an angle
#include <Servo.h>
#include <EEPROM.h>

const int MIN_DEG = 0, MAX_DEG = 180, EE_MAGIC = 0x5A;
const unsigned long STEP_MS = 100, SETTLE_MS = 2000, START_WAIT_MS = 3000;
int tol = 30, stepDeg = 3;
const int STOP_DIV = 2;                        // stop when |difference| < tol / STOP_DIV (was 3)

struct Axis {
  const char* name; uint8_t pin, ee; int sign;
  Servo servo; int pos = 90, d = 0; bool correcting = false, on = false; unsigned long balancedSince = 0;
  Axis(const char* n, uint8_t p, uint8_t e, int s) : name(n), pin(p), ee(e), sign(s) {}
  void engage()  { if (!on) { servo.write(pos); servo.attach(pin); on = true; } }
  void release() { if (on) { servo.detach(); on = false; } }
  void save()    { EEPROM.update(ee, pos); EEPROM.update(10 + ee, EE_MAGIC); }
  void load()    { pos = (EEPROM.read(10 + ee) == EE_MAGIC) ? EEPROM.read(ee) : 90; pos = constrain(pos, MIN_DEG, MAX_DEG); }
  void goSlow(int t) { t = constrain(t, MIN_DEG, MAX_DEG); engage(); while (pos != t) { pos += t > pos ? 1 : -1; servo.write(pos); delay(20); } save(); }
  void update(int diff) {                       // same logic as the single-axis tests
    d = diff;
    correcting = correcting ? abs(d) > tol / STOP_DIV : abs(d) > tol;
    if (correcting) {
      engage();
      int next = constrain(pos + sign * (d > 0 ? stepDeg : -stepDeg), MIN_DEG, MAX_DEG);
      if (next != pos) { pos = next; servo.write(pos); }
      balancedSince = millis();
    } else if (on && millis() - balancedSince >= SETTLE_MS) { release(); save(); }
  }
  const char* state() { return correcting ? "MOVING" : on ? "BALANCED" : "SETTLED"; }
};
Axis pan ("pan",  9,  0,  1);                 // signs as the single-axis tests (tilt -1 confirmed 14/10)
Axis tilt("tilt", 10, 1, -1);

void setup() {
  Serial.begin(115200);
  pan.load(); tilt.load();
  Serial.print("Pan+tilt tracking test. Start pan "); Serial.print(pan.pos);
  Serial.print(", tilt "); Serial.print(tilt.pos); Serial.println(" (from EEPROM). Waiting 3 s...");
  delay(START_WAIT_MS);
  Serial.println("tracking");
}

void loop() {
  if (Serial.available()) {
    String c = Serial.readStringUntil('\n'); c.trim();
    if (c[0] == 'x') { tol = c.substring(1).toInt(); Serial.print("deadband "); Serial.println(tol); }
    else if (c[0] == 's') { stepDeg = constrain(c.substring(1).toInt(), 1, 10); Serial.print("step "); Serial.println(stepDeg); }
    else if (c == "ip") { pan.sign = -pan.sign; Serial.println("pan inverted"); }
    else if (c == "it") { tilt.sign = -tilt.sign; Serial.println("tilt inverted"); }
    else if (c.startsWith("gp")) { pan.goSlow(c.substring(2).toInt()); Serial.print("pan at "); Serial.println(pan.pos); }
    else if (c.startsWith("gt")) { tilt.goSlow(c.substring(2).toInt()); Serial.print("tilt at "); Serial.println(tilt.pos); }
  }

  static unsigned long tStep = 0, tPrint = 0;
  if (millis() - tStep >= STEP_MS) {
    tStep = millis();
    int v[4];
    for (int i = 0; i < 4; i++) { long s = 0; for (int k = 0; k < 4; k++) s += analogRead(A0 + i); v[i] = s / 4; }
    pan.update((v[0] + v[2]) - (v[1] + v[3]));   // left - right
    tilt.update((v[0] + v[1]) - (v[2] + v[3]));  // top - bottom
  }

  if (millis() - tPrint >= 500) {
    tPrint = millis();
    Serial.print("PAN "); Serial.print(pan.state()); Serial.print(" diff "); Serial.print(pan.d); Serial.print(" @ "); Serial.print(pan.pos);
    Serial.print("  |  TILT "); Serial.print(tilt.state()); Serial.print(" diff "); Serial.print(tilt.d); Serial.print(" @ "); Serial.println(tilt.pos);
  }
}
