// U-10 / U-11 — Motor driver tests (Master Test Plan). ESP32 core 3.x (ledcAttach API).
// DRV8871 wiring: left IN1 GPIO25, IN2 GPIO26; right IN1 GPIO32, IN2 GPIO33.
// Encoders (3V3): A GPIO34 (rear left, M2), B GPIO35 (rear right, M4). Serial 115200, "Newline".
// Commands: L / R / A = select left, right, all (both sides; default)
//           f r c b = forward, reverse, coast, brake (full duty) on the selection
//           +/-0-255 = duty on the selection (negative = reverse) | d = flip direction of selection
//           s = sweep UP 0 -> 255, both sides (start-up duty, RPM per step) | S = sweep DOWN 255 -> 0
//           x = coast vs brake stop test, both sides | g = 20 hard starts, both sides
//           k1 / k20 = PWM kHz | m = toggle PWM decay mode (slow = default, fast)
// Slow decay: one input held HIGH, the other PWMs (drive / brake). Stronger at low duty than
// fast decay (drive / coast), so the motors keep turning at lower duty.
#include <Arduino.h>

const int IN1[2] = {25, 32}, IN2[2] = {26, 33};      // [0] left, [1] right
const uint8_t RES = 8;
bool INV[2] = {true, true};   // true = forward for this build's wiring (confirmed 13/10)
bool slow = true;             // PWM decay mode
int sel = 2;                  // 0 left, 1 right, 2 both

// --- encoders (same filter as U-12: falling edge, >= 500 us LOW then HIGH) ---
#define SLOTS 30
const uint8_t ENC[2] = {34, 35};
const uint32_t MIN_PULSE_US = 500;
volatile uint32_t cnt[2], lastRise[2], lastFall[2];
volatile bool armed[2];
void IRAM_ATTR edge(uint8_t i) {
  uint32_t now = micros();
  if (digitalRead(ENC[i])) { armed[i] = (now - lastFall[i] >= MIN_PULSE_US); lastRise[i] = now; }
  else { if (armed[i] && now - lastRise[i] >= MIN_PULSE_US) cnt[i]++; armed[i] = false; lastFall[i] = now; }
}
void IRAM_ATTR isrA() { edge(0); }
void IRAM_ATTR isrB() { edge(1); }
uint32_t snap(uint8_t i) { noInterrupts(); uint32_t c = cnt[i]; interrupts(); return c; }

void rpm(uint32_t ms, float& a, float& b) {             // average RPM over ms
  uint32_t a0 = snap(0), b0 = snap(1);
  delay(ms);
  a = (snap(0) - a0) * 60000.0 / SLOTS / ms;
  b = (snap(1) - b0) * 60000.0 / SLOTS / ms;
}

void setOut(int s, int a, int b) {
  if (INV[s]) { int t = a; a = b; b = t; }
  ledcWrite(IN1[s], a); ledcWrite(IN2[s], b);
}
void drive(int s, int d) {                               // d = -255..255
  d = constrain(d, -255, 255);
  int m = abs(d);
  int a = slow ? 255 : m, b = slow ? 255 - m : 0;
  if (m == 0) { a = 0; b = 0; }                          // coast
  if (d >= 0) setOut(s, a, b); else setOut(s, b, a);
}
void brake(int s) { ledcWrite(IN1[s], 255); ledcWrite(IN2[s], 255); }
void selDrive(int d) { for (int s = 0; s < 2; s++) if (sel == 2 || sel == s) drive(s, d); }
void selBrake()      { for (int s = 0; s < 2; s++) if (sel == 2 || sel == s) brake(s); }

void sweep(int from, int to, int step) {                 // both sides; 1 s settle + 2 s RPM
  for (int d = from; step > 0 ? d <= to : d >= to; d += step) {
    drive(0, d); drive(1, d); delay(1000);
    float a, b; rpm(2000, a, b);
    Serial.printf("duty %3d (%3d%%)  RPM A %.0f  B %.0f\n", d, d * 100 / 255, a, b);
  }
  drive(0, 0); drive(1, 0);
}

uint32_t timeToStop() {                                  // ms until no pulses on either encoder for 200 ms
  uint32_t t0 = millis(), last = t0, a = snap(0), b = snap(1);
  while (millis() - last < 200 && millis() - t0 < 5000)
    if (snap(0) != a || snap(1) != b) { a = snap(0); b = snap(1); last = millis(); }
  return last - t0;
}

void stopTest() {                                        // both sides at full speed, then coast / brake
  for (int i = 0; i < 2; i++) {
    drive(0, 255); drive(1, 255); delay(2000);
    float a, b; rpm(500, a, b);
    if (i == 0) { drive(0, 0); drive(1, 0); } else { brake(0); brake(1); }
    uint32_t ms = timeToStop();
    Serial.printf("%s from A %.0f / B %.0f RPM: stopped in %lu ms\n", i ? "BRAKE" : "COAST", a, b, (unsigned long)ms);
    drive(0, 0); drive(1, 0); delay(1000);
  }
}

void setup() {
  Serial.begin(115200);
  for (int s = 0; s < 2; s++) { ledcAttach(IN1[s], 20000, RES); ledcAttach(IN2[s], 20000, RES); drive(s, 0); }
  for (uint8_t i = 0; i < 2; i++) pinMode(ENC[i], INPUT);
  attachInterrupt(ENC[0], isrA, CHANGE);
  attachInterrupt(ENC[1], isrB, CHANGE);
  Serial.println("L/R/A | f r c b | +/-0-255 | d flip | s up-sweep | S down-sweep | x stop test | g starts | k1/k20 | m decay");
  Serial.printf("INV L=%d R=%d, selection: both\n", INV[0], INV[1]);
}

void loop() {
  static uint32_t t = 0;
  if (millis() - t >= 1000) {                            // live RPM once a second
    t = millis();
    static uint32_t pa = 0, pb = 0;
    uint32_t a = snap(0), b = snap(1);
    Serial.printf("RPM A %.0f  B %.0f\n", (a - pa) * 60.0 / SLOTS, (b - pb) * 60.0 / SLOTS);
    pa = a; pb = b;
  }
  if (!Serial.available()) return;
  String c = Serial.readStringUntil('\n'); c.trim();
  if (c == "L" || c == "R" || c == "A") { selDrive(0); sel = c == "L" ? 0 : c == "R" ? 1 : 2; }
  else if (c == "f") selDrive(255);
  else if (c == "r") selDrive(-255);
  else if (c == "c") selDrive(0);
  else if (c == "b") selBrake();
  else if (c == "d") {
    selDrive(0);
    for (int s = 0; s < 2; s++) if (sel == 2 || sel == s) INV[s] = !INV[s];
    Serial.printf("INV L=%d R=%d\n", INV[0], INV[1]);
  }
  else if (c == "m") { slow = !slow; selDrive(0); Serial.println(slow ? "slow decay" : "fast decay"); }
  else if (c[0] == 'k') {
    uint32_t f = c.substring(1).toInt() * 1000;
    for (int s = 0; s < 2; s++) { ledcChangeFrequency(IN1[s], f, RES); ledcChangeFrequency(IN2[s], f, RES); }
  }
  else if (c == "s") sweep(0, 255, 15);
  else if (c == "S") sweep(255, 0, -15);
  else if (c == "x") stopTest();
  else if (c == "g") {
    for (int i = 1; i <= 20; i++) {
      drive(0, 255); drive(1, 255); delay(1500);
      drive(0, 0);   drive(1, 0);   delay(1000);
      Serial.printf("start %d/20\n", i);
    }
  }
  else if (c.length() && (isDigit(c[0]) || c[0] == '-')) selDrive(c.toInt());
  Serial.printf("> %s\n", c.c_str());
}
