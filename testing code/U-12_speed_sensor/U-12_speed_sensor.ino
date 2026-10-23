// U-12 — Speed sensor (Master Test Plan)
// Encoder OUT -> GPIO34 (A) or GPIO35 (B); test one at a time. VCC 3V3, GND. Serial 115200, "Newline".
// Prints every 1 s: RPM (1 s window), speed, distance, raw pulse total.
// Commands:  z = zero totals   a = 10 s average RPM (same window as U-09)
#include <Arduino.h>

#define SLOTS      30
#define WHEEL_D_MM 65.0          // measure your wheel and update
const uint8_t  PIN[2] = {34, 35};
const char*    NAME[2] = {"A", "B"};
// Noise filter (time-domain hysteresis): a slot is counted on its FALLING edge only if the
// signal was LOW for >= MIN_PULSE_US before rising and then HIGH for >= MIN_PULSE_US.
// Real slots/bars last ~4 ms at 250 RPM (30 slots), so 500 us rejects short spikes with 8x margin.
const uint32_t MIN_PULSE_US = 500;

volatile uint32_t cnt[2], lastRise[2], lastFall[2];
volatile bool armed[2];

void IRAM_ATTR edge(uint8_t i) {
  uint32_t now = micros();
  if (digitalRead(PIN[i])) {                                  // rising
    armed[i] = (now - lastFall[i] >= MIN_PULSE_US);           // previous LOW long enough?
    lastRise[i] = now;
  } else {                                                    // falling
    if (armed[i] && now - lastRise[i] >= MIN_PULSE_US) cnt[i]++;  // HIGH long enough -> real slot
    armed[i] = false;
    lastFall[i] = now;
  }
}
void IRAM_ATTR isrA() { edge(0); }
void IRAM_ATTR isrB() { edge(1); }

uint32_t snap(uint8_t i) { noInterrupts(); uint32_t c = cnt[i]; interrupts(); return c; }
float rpmOf(uint32_t pulses, float sec) { return pulses * 60.0 / SLOTS / sec; }
float mPerRev() { return PI * WHEEL_D_MM / 1000.0; }

void setup() {
  Serial.begin(115200);
  for (uint8_t i = 0; i < 2; i++) pinMode(PIN[i], INPUT);   // 34/35 have no pull-ups; module has its own
  attachInterrupt(PIN[0], isrA, CHANGE);
  attachInterrupt(PIN[1], isrB, CHANGE);
  Serial.println("U-12 speed sensor. z = zero, a = 10 s average");
}

void loop() {
  static uint32_t prev[2], t = 0;
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'z') { noInterrupts(); cnt[0] = cnt[1] = 0; interrupts(); prev[0] = prev[1] = 0; Serial.println("Zeroed"); }
    else if (c == 'a') {
      uint32_t s0 = snap(0), s1 = snap(1);
      Serial.println("10 s average...");
      delay(10000);
      Serial.printf("AVG10 A %.1f RPM | B %.1f RPM\n", rpmOf(snap(0) - s0, 10), rpmOf(snap(1) - s1, 10));
    }
  }
  if (millis() - t >= 1000) {
    t = millis();
    for (uint8_t i = 0; i < 2; i++) {
      uint32_t c = snap(i), d = c - prev[i]; prev[i] = c;
      float rpm = rpmOf(d, 1.0);
      Serial.printf("%s %5.0f RPM  %.2f m/s  %.2f m  total %lu   ", NAME[i], rpm,
                    rpm / 60.0 * mPerRev(), (float)c / SLOTS * mPerRev(), (unsigned long)c);
    }
    Serial.println();
  }
}
