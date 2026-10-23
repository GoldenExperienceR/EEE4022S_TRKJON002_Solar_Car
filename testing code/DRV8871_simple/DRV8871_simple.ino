// DRV8871 simple test — ESP32 (core 3.x), Serial 115200, "Newline". Same logic as the Adafruit example:
//   forward = IN1 LOW, PWM on IN2  |  reverse = IN2 LOW, PWM on IN1  |  stop = both LOW (coast)
// Pins (KiCad schematic): left IN1 GPIO25, IN2 GPIO26 | right IN1 GPIO27, IN2 GPIO14. No direction swap in code.
// Commands (one per line):
//   L / R / B          select left, right or both (default B); stops the old selection first
//   f NNN / r NNN      forward / reverse at duty 0-255 on the selection   e.g. "f 150", "r150"
//   s                  stop (both LOW)
//   p AAA BBB          raw: IN1 = AAA, IN2 = BBB (0-255) on the selection, e.g. "p 255 0", "p 255 255"
//   a                  Adafruit example sequence on the selection: ramp fwd, 1 s, down; ramp rev, 1 s, down
//   h NN / l NN        plain GPIO test (no PWM): set GPIO NN HIGH / LOW, e.g. "h 25" (reset to restore PWM)
// Every command prints the exact duty written to each GPIO, so the scope reading can be compared with it.
// Note: only ledcWrite is used. On ESP32 core 3.x, digitalWrite() on a pin already used for PWM is ignored,
// so the Adafruit example's digitalWrite(IN1, LOW) would not work as-is on this board.

const int IN1[2] = {25, 27}, IN2[2] = {26, 14};   // [0] left, [1] right
int sel = 2;                                      // 0 left, 1 right, 2 both
int out[2][2];

void wrIN(int s, int a, int b) { ledcWrite(IN1[s], a); ledcWrite(IN2[s], b); out[s][0] = a; out[s][1] = b; }
void apply(int a, int b) { for (int s = 0; s < 2; s++) if (sel == 2 || sel == s) wrIN(s, a, b); }
void show() {
  for (int s = 0; s < 2; s++)
    Serial.printf("  %s: IN1 GPIO%d = %3d   IN2 GPIO%d = %3d\n", s ? "R" : "L", IN1[s], out[s][0], IN2[s], out[s][1]);
}
void ramp(bool fwd) {
  for (int i = 0; i <= 255; i++) { fwd ? apply(0, i) : apply(i, 0); delay(10); }
  delay(1000);
  for (int i = 255; i >= 0; i--) { fwd ? apply(0, i) : apply(i, 0); delay(10); }
}

void setup() {
  Serial.begin(115200); delay(300);
  Serial.println("\nDRV8871 simple test. L/R/B | f NNN | r NNN | s | p AAA BBB | a");
  for (int s = 0; s < 2; s++) {
    bool a = ledcAttach(IN1[s], 20000, 8), b = ledcAttach(IN2[s], 20000, 8);
    Serial.printf("PWM %s: GPIO%d %s, GPIO%d %s\n", s ? "right" : "left", IN1[s], a ? "ok" : "FAILED", IN2[s], b ? "ok" : "FAILED");
    wrIN(s, 0, 0);
  }
  Serial.println("selection: B (both)");
}

void loop() {
  if (!Serial.available()) return;
  String c = Serial.readStringUntil('\n'); c.trim();
  if (!c.length()) return;
  char k = c[0];
  String rest = c.substring(1); rest.trim();
  int n = constrain(rest.toInt(), 0, 255);

  if (k == 'L' || k == 'R' || k == 'B') { apply(0, 0); sel = k == 'L' ? 0 : k == 'R' ? 1 : 2; Serial.printf("selection: %c (old selection stopped)\n", k); }
  else if (k == 'f') { apply(0, n); Serial.printf("FORWARD duty %d (IN1 LOW, PWM on IN2)\n", n); }
  else if (k == 'r') { apply(n, 0); Serial.printf("REVERSE duty %d (IN2 LOW, PWM on IN1)\n", n); }
  else if (k == 's') { apply(0, 0); Serial.println("STOP (both LOW)"); }
  else if (k == 'p') {
    int sp = rest.indexOf(' ');
    int a = constrain(rest.substring(0, sp).toInt(), 0, 255), b = sp > 0 ? constrain(rest.substring(sp + 1).toInt(), 0, 255) : 0;
    apply(a, b); Serial.printf("RAW IN1 %d, IN2 %d\n", a, b);
  }
  else if (k == 'h' || k == 'l') {               // plain GPIO test, no PWM: "h 25" = GPIO25 HIGH, "l 25" = LOW
    int pin = rest.toInt(); ledcDetach(pin); pinMode(pin, OUTPUT); digitalWrite(pin, k == 'h');
    Serial.printf("GPIO%d set %s as a plain output (PWM detached; reset the board to restore PWM)\n", pin, k == 'h' ? "HIGH" : "LOW");
    return;
  }
  else if (k == 'a') { Serial.println("ramp forward..."); ramp(true); Serial.println("ramp reverse..."); ramp(false); Serial.println("done"); }
  else { Serial.println("? commands: L/R/B | f NNN | r NNN | s | p AAA BBB | a"); return; }
  show();
}
