// U-13 — Ultrasonic accuracy + crosstalk / firing-scheme test (Master Test Plan). ESP32.
// 4x HC-SR04 at 5 V. ECHO via 2.2k (series) / 4.7k (to GND) divider -> ~3.4 V.
// Pins = KiCad ESP32 schematic (U1): US1..US4 TRIG / ECHO. Assumed US1 = F, US2 = L, US3 = R, US4 = B.
// Sensors: F (front 0deg), L (+30deg), R (-30deg), B (rear 180deg). Serial 115200, "Newline".
// Commands:
//   a = accuracy: each sensor fired alone, 10 readings each -> mean / SD / min / max / missed
//   q = reference: all four one at a time, 30 ms slots, 100 cycles (run FIRST for f / s)
//   f = forward scheme: F, L, R one at a time, 20 ms slots, range <= 200 cm, 100 cycles
//   s = F, L, R fired at once, 30 ms slot, 100 cycles (shows crosstalk)
//   g = ghost check: scheme f with NOTHING within 2 m; any echo counts as a ghost
//   r F L R B = manual reference in cm (tape-measured board distances), e.g. r 50 40 60 30
//   pNN = pair scheme: L + R fired together, wait NN ms, F + B fired together, wait NN ms (e.g. p50)
//   P   = pair sweep: gaps 10, 15, 20, 25, 30, 40, 50, 60 ms, one summary line each
//   l = live readout, all four in turn (any key stops)
#include <Arduino.h>

const uint8_t  N = 4;
const char*    NAME[N] = {"F", "L", "R", "B"};
const uint8_t  TRIG[N] = {5, 17, 16, 4};       // US1 GPIO5, US2 GPIO17, US3 GPIO16, US4 GPIO4
const uint8_t  ECHO[N] = {18, 19, 23, 13};     // US1 GPIO18, US2 GPIO19, US3 GPIO23, US4 GPIO13
const float    TEMP_C = 20.0;                                  // room temperature -> speed of sound
const float    CM_PER_US = (331.3 + 0.606 * TEMP_C) / 2e4;     // echo time (us) -> one-way distance (cm)
const uint16_t CYCLES = 100, ACC_N = 10;
const float    OUTLIER_CM = 10.0;                              // > 10 cm from reference = ghost / cross-echo

// Defined before any function: the Arduino IDE inserts function prototypes above the first function.
struct Mode { char key; const char* name; uint8_t slots; uint8_t mask[4]; uint16_t slotMs; float maxCm; };
const Mode MODES[] = {
  {'q', "REF: all four in turn, 30 ms", 4, {1, 2, 4, 8}, 30, 400},
  {'f', "FWD: F, L, R in turn, 20 ms",  3, {1, 2, 4, 0}, 20, 200},
  {'s', "F + L + R at once, 30 ms",     1, {7, 0, 0, 0}, 30, 400},
  {'g', "GHOST: FWD, nothing in 2 m",   3, {1, 2, 4, 0}, 20, 200},
};

struct Stats { float mean, sd, bias, missPct, ghostPct; };

volatile uint32_t riseUs[N], fallUs[N];
void IRAM_ATTR echoIsr(uint8_t i) { if (digitalRead(ECHO[i])) riseUs[i] = micros(); else fallUs[i] = micros(); }
void IRAM_ATTR isr0() { echoIsr(0); }  void IRAM_ATTR isr1() { echoIsr(1); }
void IRAM_ATTR isr2() { echoIsr(2); }  void IRAM_ATTR isr3() { echoIsr(3); }

float ref[N];  bool haveRef = false, manualRef = false;
float buf[N][CYCLES];  uint16_t nBuf[N];

// Fire the sensors in mask together, wait one slot; out = cm, -1 = no echo / out of range, NAN = not fired
void fire(uint8_t mask, uint16_t slotMs, float maxCm, float* out) {
  for (uint8_t i = 0; i < N; i++) { riseUs[i] = fallUs[i] = 0; out[i] = NAN; }
  for (uint8_t i = 0; i < N; i++) if (mask & (1 << i)) digitalWrite(TRIG[i], HIGH);
  delayMicroseconds(10);
  for (uint8_t i = 0; i < N; i++) if (mask & (1 << i)) digitalWrite(TRIG[i], LOW);
  delay(slotMs);
  for (uint8_t i = 0; i < N; i++) {
    if (!(mask & (1 << i))) continue;
    uint32_t r = riseUs[i], f = fallUs[i];
    float cm = (r && f > r) ? (f - r) * CM_PER_US : -1;
    out[i] = (cm > 0 && cm <= maxCm) ? cm : -1;
  }
}

float median(float* a, uint16_t n) {
  float t[CYCLES]; uint16_t k = 0;
  for (uint16_t i = 0; i < n; i++) if (a[i] > 0) t[k++] = a[i];
  if (!k) return -1;
  for (uint16_t i = 1; i < k; i++) { float v = t[i]; int j = i - 1; while (j >= 0 && t[j] > v) { t[j + 1] = t[j]; j--; } t[j + 1] = v; }
  return t[k / 2];
}

void accuracy() {                                   // each sensor alone, ACC_N readings
  Serial.printf("\n== ACCURACY: %u readings per sensor, fired alone (30 ms) ==\n", ACC_N);
  Serial.println("Sensor  mean(cm)  SD(cm)  min(cm)  max(cm)  missed");
  float d[N];
  for (uint8_t i = 0; i < N; i++) {
    float sum = 0, sq = 0, mn = 1e9, mx = 0; uint16_t ok = 0;
    for (uint16_t k = 0; k < ACC_N; k++) {
      fire(1 << i, 30, 400, d);
      if (d[i] < 0) continue;
      sum += d[i]; sq += d[i] * d[i]; mn = min(mn, d[i]); mx = max(mx, d[i]); ok++;
    }
    if (!ok) { Serial.printf("%-6s  no echo (%u/%u missed)\n", NAME[i], ACC_N, ACC_N); continue; }
    float mean = sum / ok, sd = ok > 1 ? sqrt(max(0.0f, (sq - sum * sum / ok) / (ok - 1))) : 0;
    Serial.printf("%-6s  %8.1f  %6.2f  %7.1f  %7.1f  %u/%u\n", NAME[i], mean, sd, mn, mx, ACC_N - ok, ACC_N);
  }
}

Stats stats(uint8_t i) {                            // over buf[i]; ghost = > OUTLIER_CM from reference
  float sum = 0, sq = 0; uint16_t ok = 0, miss = 0, ghost = 0;
  for (uint16_t k = 0; k < nBuf[i]; k++) {
    float v = buf[i][k];
    if (v < 0) { miss++; continue; }
    if (ref[i] > 0 && fabs(v - ref[i]) > OUTLIER_CM) { ghost++; continue; }
    sum += v; sq += v * v; ok++;
  }
  Stats st;
  st.mean = ok ? sum / ok : -1;
  st.sd = ok > 1 ? sqrt(max(0.0f, (sq - sum * sum / ok) / (ok - 1))) : 0;
  st.bias = ok ? st.mean - ref[i] : 0;
  st.missPct = nBuf[i] ? 100.0 * miss / nBuf[i] : 0;
  st.ghostPct = nBuf[i] ? 100.0 * ghost / nBuf[i] : 0;
  return st;
}

// run CYCLES cycles of a scheme; returns seconds taken
float runScheme(uint8_t slots, const uint8_t* mask, uint16_t slotMs, float maxCm) {
  for (uint8_t i = 0; i < N; i++) nBuf[i] = 0;
  float d[N];
  uint32_t t0 = millis();
  for (uint16_t c = 0; c < CYCLES; c++)
    for (uint8_t s = 0; s < slots; s++) {
      fire(mask[s], slotMs, maxCm, d);
      for (uint8_t i = 0; i < N; i++) if (!isnan(d[i]) && nBuf[i] < CYCLES) buf[i][nBuf[i]++] = d[i];
    }
  return (millis() - t0) / 1000.0;
}

void printTable() {
  Serial.println("Sensor  ref(cm)  mean(cm)  SD(cm)  bias(cm)  bias%  missed%  ghost%");
  for (uint8_t i = 0; i < N; i++) {
    if (!nBuf[i]) continue;                         // sensor not fired in this scheme
    Stats st = stats(i);
    Serial.printf("%-6s  %7.1f  %8.1f  %6.2f  %8.1f  %5.1f  %7.1f  %6.1f\n", NAME[i], ref[i], st.mean, st.sd,
                  st.bias, ref[i] > 0 ? 100 * st.bias / ref[i] : 0, st.missPct, st.ghostPct);
  }
}

void runMode(const Mode& m) {
  if ((m.key == 'f' || m.key == 's') && !haveRef) { Serial.println("Set a reference first: q, or r F L R B."); return; }
  float sec = runScheme(m.slots, m.mask, m.slotMs, m.maxCm);
  if (m.key == 'q' && !manualRef) { for (uint8_t i = 0; i < N; i++) ref[i] = median(buf[i], nBuf[i]); haveRef = true; }

  Serial.printf("\n== %s: %u cycles in %.2f s -> %.1f Hz per sensor, cycle %.0f ms ==\n",
                m.name, CYCLES, sec, CYCLES / sec, 1000 * sec / CYCLES);
  if (m.key == 'g') {                               // nothing in range: every echo is a ghost
    Serial.println("Sensor  ghost%  nearest ghost (cm)");
    for (uint8_t i = 0; i < N; i++) {
      if (!nBuf[i]) continue;
      uint16_t gh = 0; float near = 1e9;
      for (uint16_t k = 0; k < nBuf[i]; k++) if (buf[i][k] > 0) { gh++; near = min(near, buf[i][k]); }
      if (gh) Serial.printf("%-6s  %6.1f  %.1f\n", NAME[i], 100.0 * gh / nBuf[i], near);
      else    Serial.printf("%-6s  %6.1f  -\n", NAME[i], 0.0);
    }
    return;
  }
  printTable();
}

// Pair scheme: L + R fired together, wait gapMs, F + B fired together, wait gapMs (one pass = all four)
const uint8_t PAIR[2] = {2 | 4, 1 | 8};
void pairRun(uint16_t gapMs) {
  float sec = runScheme(2, PAIR, gapMs, 200);
  Serial.printf("\n== PAIR: (L+R) then (F+B), gap %u ms: %.1f Hz per sensor, cycle %.0f ms ==\n",
                gapMs, CYCLES / sec, 1000 * sec / CYCLES);
  printTable();
}

// Sweep the gap: one line per gap, F / L / R bias, missed%, ghost%
void pairSweep() {
  const uint16_t GAPS[] = {10, 15, 20, 25, 30, 40, 50, 60};
  Serial.println("\n== PAIR SWEEP: (L+R) then (F+B) ==");
  Serial.println("gap(ms)  Hz    | F bias miss% ghost% | L bias miss% ghost% | R bias miss% ghost% | B bias miss% ghost%");
  for (uint16_t g : GAPS) {
    float sec = runScheme(2, PAIR, g, 200);
    Serial.printf("%5u   %5.1f ", g, CYCLES / sec);
    for (uint8_t i = 0; i < N; i++) {
      Stats st = stats(i);
      Serial.printf("| %5.1f %5.1f %5.1f  ", st.bias, st.missPct, st.ghostPct);
    }
    Serial.println();
  }
  Serial.println("Note: a gap of G ms can only see echoes up to ~G x 17 cm (10 ms -> ~170 cm).");
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(50);
  for (uint8_t i = 0; i < N; i++) { pinMode(TRIG[i], OUTPUT); digitalWrite(TRIG[i], LOW); pinMode(ECHO[i], INPUT); }
  attachInterrupt(ECHO[0], isr0, CHANGE); attachInterrupt(ECHO[1], isr1, CHANGE);
  attachInterrupt(ECHO[2], isr2, CHANGE); attachInterrupt(ECHO[3], isr3, CHANGE);
  Serial.println("U-13 ultrasonic. a accuracy | q reference | r F L R B (manual reference, cm) | f forward |");
  Serial.println("s simultaneous | pNN pair (L+R, NN ms, F+B) | P pair gap sweep | g ghost | l live");
}

void loop() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n'); cmd.trim();
  if (!cmd.length()) return;
  char c = cmd[0];
  if (c == 'a') accuracy();
  else if (c == 'r') {                              // manual reference: r 50 40 60 30 (F L R B, cm)
    float v[N]; int n = sscanf(cmd.c_str() + 1, "%f %f %f %f", &v[0], &v[1], &v[2], &v[3]);
    if (n == N) { for (uint8_t i = 0; i < N; i++) ref[i] = v[i]; haveRef = manualRef = true;
                  Serial.printf("Reference set: F %.1f  L %.1f  R %.1f  B %.1f cm (q no longer overwrites it)\n", v[0], v[1], v[2], v[3]); }
    else Serial.println("Usage: r F L R B   e.g. r 50 40 60 30");
  }
  else if (c == 'p' && cmd.length() > 1) {
    if (!haveRef) { Serial.println("Set a reference first: q, or r F L R B."); return; }
    pairRun(constrain(cmd.substring(1).toInt(), 5, 200));
  }
  else if (c == 'P') {
    if (!haveRef) { Serial.println("Set a reference first: q, or r F L R B."); return; }
    pairSweep();
  }
  else if (c == 'l') {
    float d[N], last[N];
    while (!Serial.available()) {
      for (uint8_t s = 0; s < N; s++) { fire(1 << s, 20, 400, d); last[s] = d[s]; }
      Serial.printf("F %6.1f  L %6.1f  R %6.1f  B %6.1f cm\n", last[0], last[1], last[2], last[3]);
    }
    while (Serial.available()) Serial.read();
  }
  else for (const Mode& m : MODES) if (c == m.key) runMode(m);
}
