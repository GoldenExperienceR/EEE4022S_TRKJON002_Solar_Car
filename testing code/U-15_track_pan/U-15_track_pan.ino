// Test — Pan tracking only. Arduino Nano, Serial 115200, "Newline".
// LDRs: TL A0, TR A1, BL A2, BR A3. Servo: pan on D9 (the other servo is not driven).
// Start-up: the last angle is kept in EEPROM, so the servo is attached AT that angle (no jump).
// It then waits 3 s and starts tracking: stepDeg (3) degrees per step toward the brighter side (left - right),
// starting when |difference| > x and stopping when it falls below x/3 (hysteresis).
// Balanced for 2 s -> servo released (no holding current) and the angle saved.
// Commands: x NN = deadband (default 30) | s N = degrees per step (default 3; 1 = old behaviour)
//           i = invert direction | g NNN = go to angle slowly
// Note: on the very first run (empty EEPROM) the servo goes to 90 once.
#include <Servo.h>
#include <EEPROM.h>

const int MIN_DEG = 0, MAX_DEG = 180;
const int EE_ADDR = 0, EE_MAGIC_ADDR = 10 + EE_ADDR, EE_MAGIC = 0x5A;
const unsigned long STEP_MS = 100, SETTLE_MS = 2000, START_WAIT_MS = 3000;
Servo servo;
int pos, tol = 30, stepDeg = 3, sign = 1;
bool correcting = false, attached_ = false;
unsigned long balancedSince = 0;

void engage()  { if (!attached_) { servo.write(pos); servo.attach(9); attached_ = true; } }
void release() { if (attached_) { servo.detach(); attached_ = false; } }
void save()    { EEPROM.update(EE_ADDR, pos); EEPROM.update(EE_MAGIC_ADDR, EE_MAGIC); }

void goSlow(int target) {
  target = constrain(target, MIN_DEG, MAX_DEG);
  engage();
  while (pos != target) { pos += target > pos ? 1 : -1; servo.write(pos); delay(20); }
  save();
}

int diff() {                                   // left - right difference
  int v[4];
  for (int i = 0; i < 4; i++) { long s = 0; for (int k = 0; k < 4; k++) s += analogRead(A0 + i); v[i] = s / 4; }
  return (v[0] + v[2]) - (v[1] + v[3]);
}

void setup() {
  Serial.begin(115200);
  pos = (EEPROM.read(EE_MAGIC_ADDR) == EE_MAGIC) ? EEPROM.read(EE_ADDR) : 90;
  pos = constrain(pos, MIN_DEG, MAX_DEG);
  Serial.print("Pan tracking test. Start angle "); Serial.print(pos); Serial.println(" (from EEPROM). Waiting 3 s...");
  delay(START_WAIT_MS);
  Serial.println("tracking");
}

void loop() {
  if (Serial.available()) {
    String c = Serial.readStringUntil('\n'); c.trim();
    if (c[0] == 'x') { tol = c.substring(1).toInt(); Serial.print("deadband "); Serial.println(tol); }
    else if (c[0] == 's') { stepDeg = constrain(c.substring(1).toInt(), 1, 10); Serial.print("step "); Serial.println(stepDeg); }
    else if (c == "i") { sign = -sign; Serial.println("direction inverted"); }
    else if (c[0] == 'g') { goSlow(c.substring(1).toInt()); Serial.print("at "); Serial.println(pos); }
  }

  static unsigned long tStep = 0, tPrint = 0;
  static int d = 0;
  if (millis() - tStep >= STEP_MS) {
    tStep = millis();
    d = diff();
    correcting = correcting ? abs(d) > tol / 3 : abs(d) > tol;
    if (correcting) {
      engage();
      int next = constrain(pos + sign * (d > 0 ? stepDeg : -stepDeg), MIN_DEG, MAX_DEG);
      if (next != pos) { pos = next; servo.write(pos); }
      balancedSince = millis();
    } else if (attached_ && millis() - balancedSince >= SETTLE_MS) {
      release(); save();
    }
  }

  if (millis() - tPrint >= 500) {
    tPrint = millis();
    Serial.print(correcting ? "MOVING " : (attached_ ? "BALANCED " : "SETTLED (released) "));
    Serial.print(" diff "); Serial.print(d);
    Serial.print("  pan "); Serial.println(pos);
  }
}
