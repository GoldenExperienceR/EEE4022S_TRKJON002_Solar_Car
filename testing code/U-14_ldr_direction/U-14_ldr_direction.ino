// Test 1 — LDR direction only (no servos move). Arduino Nano, Serial 115200.
// LDRs: TL A0, TR A1, BL A2, BR A3 (5V -> LDR -> Ax -> R -> GND, higher = brighter).
// Shine a torch on the array: every 0.5 s it prints the readings and which way the
// panel should turn to face the light, and the servo angle change that would make.
// x NN (Newline) sets the deadband in ADC counts (default 30).

const int PAN_SIGN = 1, TILT_SIGN = -1;  // tilt reversed (confirmed 14/10); same convention as the tracker firmware: +1 deg toward the light
int tol = 30;

void setup() { Serial.begin(115200); Serial.println("Test 1: LDR direction (servos not driven)"); }

void loop() {
  if (Serial.available()) {
    String c = Serial.readStringUntil('\n'); c.trim();
    if (c[0] == 'x') { tol = c.substring(1).toInt(); Serial.print("deadband "); Serial.println(tol); }
  }
  int v[4];
  for (int i = 0; i < 4; i++) { long s = 0; for (int k = 0; k < 8; k++) s += analogRead(A0 + i); v[i] = s / 8; }
  int dv = (v[0] + v[1]) - (v[2] + v[3]);   // top - bottom
  int dh = (v[0] + v[2]) - (v[1] + v[3]);   // left - right

  Serial.print("TL "); Serial.print(v[0]); Serial.print("  TR "); Serial.print(v[1]);
  Serial.print("  BL "); Serial.print(v[2]); Serial.print("  BR "); Serial.print(v[3]);
  Serial.print("  | dV "); Serial.print(dv); Serial.print(" dH "); Serial.print(dh);

  Serial.print("  | TILT: ");
  if (abs(dv) <= tol) Serial.print("hold");
  else { Serial.print(dv > 0 ? "UP " : "DOWN "); Serial.print(TILT_SIGN * (dv > 0 ? 1 : -1) > 0 ? "(+deg)" : "(-deg)"); }
  Serial.print("  PAN: ");
  if (abs(dh) <= tol) Serial.print("hold");
  else { Serial.print(dh > 0 ? "LEFT " : "RIGHT "); Serial.print(PAN_SIGN * (dh > 0 ? 1 : -1) > 0 ? "(+deg)" : "(-deg)"); }
  Serial.println();
  delay(3000);
}
