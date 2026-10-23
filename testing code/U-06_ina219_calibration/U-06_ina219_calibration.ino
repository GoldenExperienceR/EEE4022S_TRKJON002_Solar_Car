// U-06 — INA219 calibration (Master Test Plan)
// Battery INA219 = 0x40, solar INA219 = 0x41 (A0 bridged).
// Wiring: VCC 3V3, GND, SDA GPIO21, SCL GPIO22. Serial 115200, line ending "Newline".
// Commands:  z = 30 s zero-offset average (no load)   a = 20-sample average now
// Vbus = voltage at VIN- (load side). Vsup = Vbus + shunt drop (VIN+, supply side).
// Power: P = INA219 power register (mW); VxI = bus V x current, for comparison.
#include <Wire.h>
#include <Adafruit_INA219.h>

Adafruit_INA219 bat(0x40), sol(0x41);
bool batOk, solOk;

void scanI2C() {
  Serial.print("I2C found:");
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) Serial.printf(" 0x%02X", a);
  }
  Serial.println();
}

void average(uint16_t n, uint16_t dt) {
  float bv = 0, bi = 0, bp = 0, sv = 0, si = 0, sp = 0;
  for (uint16_t k = 0; k < n; k++) {
    if (batOk) { bv += bat.getBusVoltage_V(); bi += bat.getCurrent_mA(); bp += bat.getPower_mW(); }
    if (solOk) { sv += sol.getBusVoltage_V(); si += sol.getCurrent_mA(); sp += sol.getPower_mW(); }
    delay(dt);
  }
  bv /= n; bi /= n; bp /= n; sv /= n; si /= n; sp /= n;
  Serial.printf("AVG(%u) BAT %.3f V %.1f mA P %.1f mW (VxI %.1f) | SOL %.3f V %.1f mA P %.1f mW (VxI %.1f)\n",
                n, bv, bi, bp, bv * bi, sv, si, sp, sv * si);
}

void printBoard(const char* name, Adafruit_INA219& ina) {
  float vbus = ina.getBusVoltage_V();
  float vsh  = ina.getShuntVoltage_mV();
  Serial.printf("%s Vbus(load) %.3f V  Vshunt %.2f mV  Vsup %.3f V  I %.1f mA  P %.1f mW\n",
    name, vbus, vsh, vbus + vsh / 1000.0, ina.getCurrent_mA(), ina.getPower_mW());
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Wire.begin(21, 22);
  scanI2C();
  batOk = bat.begin();   // default 32 V / 2 A calibration (reads to ~3.2 A)
  solOk = sol.begin();
  Serial.printf("BAT 0x40 %s | SOL 0x41 %s\n", batOk ? "OK" : "MISSING", solOk ? "OK" : "MISSING");
  Serial.println("z = 30 s zero offset, a = 20-sample average");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'z') { Serial.println("Zero offset: no load, 30 s..."); average(300, 100); }
    else if (c == 'a') average(20, 50);
  }
  static uint32_t t = 0;
  if (millis() - t >= 2000) {
    t = millis();
    if (batOk) printBoard("BAT", bat);
    if (solOk) printBoard("SOL", sol);
  }
}
