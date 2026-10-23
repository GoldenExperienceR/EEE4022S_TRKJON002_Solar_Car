// U-14 / U-15 / INT-06 — LDR array, tracker mechanism and torch tracking test. Arduino Nano.
// Pin-out matches the KiCad schematic (U2): SERVO_PAN D9, SERVO_TILT D10, LDR_TL/TR/BL/BR A0..A3,
// MOVING interlock from the ESP32 on D2. Servos on a separate 5 V supply (common GND).
// LDRs: 5V -> LDR -> A0..A3 -> R -> GND (reading rises with light), all four R equal.
//   A0 = top-left, A1 = top-right, A2 = bottom-left, A3 = bottom-right (as seen facing the panel)
// Serial 115200, "Newline". Commands:
//   pNNN / tNNN = pan / tilt angle, moved 1 deg at a time | vNN = ms per degree (default 15, 0 = jump)
//   a   = 20-sample average of all four LDRs + differences
//   k   = tracking on / off (follows the brightest direction, e.g. a torch)
//   xNN = tracking deadband in ADC counts (default 20) | ip / it = invert pan / tilt tracking direction
// While tracking: MOVING (D2) HIGH -> panel driven to FLAT and held; LOW -> tracking resumes.
// Live status prints every 0.5 s.
#include <Servo.h>

const int PAN_MIN = 0, PAN_MAX = 180, TILT_MIN = 0, TILT_MAX = 180;  // narrow these if anything binds
const int FLAT_PAN = 90, FLAT_TILT = 90;   // set to the angles where the panel lies flat on the car
const char* NAME[4] = {"TL", "TR", "BL", "BR"};
Servo pan, tilt;
int panPos = 90, tiltPos = 90;
int msPerDeg = 15;               // slew rate
bool tracking = false;
int tol = 20;                    // deadband (ADC counts on the pair difference)
int panSign = 1, tiltSign = -1;  // tilt reversed (confirmed 14/10); flip with ip / it if the panel moves away from the light

void slew(Servo& s, int& pos, int target) {      // smooth move, 1 deg at a time
  int step = target > pos ? 1 : -1;
  while (pos != target) { pos += step; s.write(pos); delay(msPerDeg); }
}

void readAvg(int n, float* v) {
  for (int i = 0; i < 4; i++) v[i] = 0;
  for (int k = 0; k < n; k++) { for (int i = 0; i < 4; i++) v[i] += analogRead(A0 + i); delay(2); }
  for (int i = 0; i < 4; i++) v[i] /= n;
}

// one tracking step: nudge each axis 1 deg toward the brighter side if outside the deadband
void trackStep(int& dv, int& dh) {
  float v[4]; readAvg(4, v);
  dv = (v[0] + v[1]) - (v[2] + v[3]);            // top - bottom
  dh = (v[0] + v[2]) - (v[1] + v[3]);            // left - right
  if (abs(dv) > tol) { tiltPos = constrain(tiltPos + tiltSign * (dv > 0 ? 1 : -1), TILT_MIN, TILT_MAX); tilt.write(tiltPos); }
  if (abs(dh) > tol) { panPos  = constrain(panPos  + panSign  * (dh > 0 ? 1 : -1), PAN_MIN,  PAN_MAX);  pan.write(panPos); }
}

void setup() {
  Serial.begin(115200);
  pan.attach(9); tilt.attach(10);
  pan.write(panPos); tilt.write(tiltPos);
  pinMode(2, INPUT);                       // MOVING interlock (ESP32 output)
  Serial.println("pNNN/tNNN angle, vNN speed, a LDR avg, k track on/off, xNN deadband, ip/it invert");
}

void loop() {
  if (Serial.available()) {
    String c = Serial.readStringUntil('\n'); c.trim();
    if (c == "a") {
      float v[4]; readAvg(20, v);
      float mean = (v[0] + v[1] + v[2] + v[3]) / 4;
      Serial.print("AVG20");
      for (int i = 0; i < 4; i++) { Serial.print(' '); Serial.print(NAME[i]); Serial.print(' '); Serial.print(v[i], 0); }
      Serial.print("  | spread "); Serial.print(100.0 * (max(max(v[0], v[1]), max(v[2], v[3])) - min(min(v[0], v[1]), min(v[2], v[3]))) / mean, 1);
      Serial.print("%  top-bottom "); Serial.print((v[0] + v[1]) - (v[2] + v[3]), 0);
      Serial.print("  left-right "); Serial.println((v[0] + v[2]) - (v[1] + v[3]), 0);
    }
    else if (c == "k")  { tracking = !tracking; Serial.println(tracking ? "TRACKING ON" : "TRACKING OFF"); }
    else if (c == "ip") { panSign = -panSign;   Serial.println("pan direction inverted"); }
    else if (c == "it") { tiltSign = -tiltSign; Serial.println("tilt direction inverted"); }
    else if (c.length() > 1) {
      int a = c.substring(1).toInt();
      if (c[0] == 'p') { a = constrain(a, PAN_MIN, PAN_MAX); slew(pan, panPos, a); }
      else if (c[0] == 't') { a = constrain(a, TILT_MIN, TILT_MAX); slew(tilt, tiltPos, a); }
      else if (c[0] == 'v') { msPerDeg = constrain(a, 0, 100); }
      else if (c[0] == 'x') { tol = constrain(a, 0, 500); }
      Serial.print("> "); Serial.print(c); Serial.print(" -> "); Serial.println(a);
    }
  }

  bool moving = digitalRead(2);
  static int dv = 0, dh = 0;
  static unsigned long tStep = 0;
  if (tracking && millis() - tStep >= 50) {        // ~20 tracking steps per second
    tStep = millis();
    if (moving) { slew(pan, panPos, FLAT_PAN); slew(tilt, tiltPos, FLAT_TILT); }   // interlock: flat and hold
    else trackStep(dv, dh);
  }

  static unsigned long t = 0;
  if (millis() - t > 500) {
    t = millis();
    Serial.print(moving ? "MOVING " : "STOPPED ");
    Serial.print(tracking ? (moving ? "[FLAT] " : "[TRACK] ") : "");
    Serial.print("LDR");
    for (int i = 0; i < 4; i++) { Serial.print(' '); Serial.print(NAME[i]); Serial.print(' '); Serial.print(analogRead(A0 + i)); }
    Serial.print("  pan "); Serial.print(panPos); Serial.print(" tilt "); Serial.print(tiltPos);
    if (tracking) { Serial.print("  dV "); Serial.print(dv); Serial.print(" dH "); Serial.print(dh); }
    Serial.println();
  }
}
