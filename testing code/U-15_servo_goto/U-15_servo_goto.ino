// Test 2 — move to a pan / tilt angle pair slowly. Arduino Nano, Serial 115200, "Newline".
// Servos: pan D9, tilt D10 (separate 5 V supply, common GND).
// Type two numbers:  PAN TILT   e.g.  120 45   (degrees, 0-180)
// Tilt moves first, then pan (one servo at a time), 1 deg per step, and the current angles print every 100 ms.
// v NN sets the speed in ms per degree (default 20). ? prints the current angles.
// Note: hobby servos have no position feedback, so "current angle" is the angle commanded so far.
#include <Servo.h>

const int PAN_MIN = 0, PAN_MAX = 180, TILT_MIN = 0, TILT_MAX = 180;
Servo pan, tilt;
int panPos = 90, tiltPos = 90;
int msPerDeg = 20;

void report(const char* tag) {
  Serial.print(tag); Serial.print(" pan "); Serial.print(panPos); Serial.print("  tilt "); Serial.println(tiltPos);
}

void moveTo(int p, int t) {
  p = constrain(p, PAN_MIN, PAN_MAX); t = constrain(t, TILT_MIN, TILT_MAX);
  Serial.print("target pan "); Serial.print(p); Serial.print("  tilt "); Serial.println(t);
  unsigned long last = 0;
  while (tiltPos != t) {                       // tilt first, then pan: one servo at a time
    tiltPos += t > tiltPos ? 1 : -1; tilt.write(tiltPos); delay(msPerDeg);
    if (millis() - last >= 100) { last = millis(); report("  moving tilt"); }
  }
  while (panPos != p) {
    panPos += p > panPos ? 1 : -1; pan.write(panPos); delay(msPerDeg);
    if (millis() - last >= 100) { last = millis(); report("  moving pan"); }
  }
  report("ARRIVED");
}

void setup() {
  Serial.begin(115200);
  pan.write(panPos); tilt.write(tiltPos);
  pan.attach(9); tilt.attach(10);
  Serial.println("Test 2: type  PAN TILT  (e.g. 120 45);  v NN = ms/deg;  ? = angles");
  report("start");
}

void loop() {
  if (!Serial.available()) return;
  String c = Serial.readStringUntil('\n'); c.trim();
  if (c == "?") { report("now"); return; }
  if (c[0] == 'v') { msPerDeg = constrain(c.substring(1).toInt(), 0, 100); Serial.print("speed "); Serial.print(msPerDeg); Serial.println(" ms/deg"); return; }
  int sp = c.indexOf(' ');
  if (sp < 0) sp = c.indexOf(',');
  if (sp < 0) { Serial.println("enter two numbers: PAN TILT"); return; }
  moveTo(c.substring(0, sp).toInt(), c.substring(sp + 1).toInt());
}
