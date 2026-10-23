// car_main — Solar car main firmware (ESP32 30/32-pin, ESP32 Arduino core 3.x). First version for INT-07.
// Phone: join Wi-Fi "SolarCar" (password solarcar1), open http://192.168.4.1  -> dashboard (served gzipped from flash).
// Libraries: "WebSockets" by Markus Sattler, "Adafruit INA219". Board: ESP32 Dev Module, Flash 40 MHz / DIO.
//
// Modules (pins = KiCad schematic). Each is checked while running; a missing one is reported to the dashboard,
// which greys its tile out, and the rest of the car keeps working.
//   Drive    : DRV8871 L IN1/IN2 25/26, R 27/14. U-10 slow-decay drive (fwd/rev mirrored), stop = LOW/LOW (coast).
//   Encoders : SPD_L 34, SPD_R 35, 30 slots, wheel 65 mm, falling edge + 500 us filter (U-12).
//   Ultrason.: TRIG/ECHO F 5/18, L 17/19, R 16/23, B 4/13; L+R then F+B, 30 ms gap (U-13). Missing = no echo for 1 s.
//   INA219   : battery 0x41 (V, I, SOC), solar 0x40 (PV power). Missing = no I2C reply (re-checked every 1 s).
//   MOVING   : GPIO32 -> tracker Nano D2, HIGH while the motors are driven (tracker parks the panel).
// Safety: no message from the page for 500 ms (or socket closed) -> motors stop, gear back to STOP.
//         Copilot: forward stops if front < 20 cm, reverse stops if back < 20 cm.  Pack < 9.0 V for 3 s -> drive locked.
// Page -> car: T0..100 throttle | D1 / D-1 gear | S stop | J-100..100 steer | M0/M1 mode | K keep-alive (every 200 ms)
// Car -> page (10 Hz JSON): mps v i w soc dF dL dR dB chg fs blk lv ok{bat,pv,enc,us[4]}
// Log: 5 Hz in RAM (11 min ring). http://192.168.4.1/log.csv downloads it, /log/clear empties it.
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include "dashboard_gz.h"                     // const uint8_t DASH_GZ[] PROGMEM, DASH_GZ_LEN

// ------------------------------------------------------------------ tuning
const int   DUTY_MAX = 238;                   // 7.0 V / 7.5 V cap (INT-03)
// Drive scheme = U-10 (forward AND reverse confirmed there): one input held HIGH, the other PWMs with 255 - duty.
// Stop = both LOW (coast). Slow decay keeps torque at low duty: the loaded car started at duty 45-60 in U-10.
// If the speed slider stops controlling speed (motor always full), set SLOW_DECAY false: INT-04 / supplier
// pattern (PWM on one input, other LOW), which starts the loaded car at ~163.
const bool  SLOW_DECAY = false;                // supplier method (21/10): never both HIGH; HIGH/HIGH drove the motors on this build
const int   DUTY_START = 102;                  // 40 % duty at 1 % throttle (21/10). Car starts ~64 % duty (INT-04); the bottom of the slider is a gentle, non-moving start by choice
const uint32_t LINK_TIMEOUT_MS = 500;
const float STOP_CM = 20, CLEAR_CM = 25;      // copilot
const float LV_CUT = 9.0;                     // pack low-voltage lock (stretch goal)
const float PACK_AH = 3.5;
const int   SLOTS = 30; const float WHEEL_MM = 65.0;

// ------------------------------------------------------------------ drive
const int IN1[2] = {25, 27}, IN2[2] = {26, 14};
const bool INV[2] = {true, true};             // forward direction for this build (INT-03)
const int MOVING_PIN = 32;
int gear = 0, thr = 0, steer = 0, mode = 0;   // gear 1 fwd / 0 stop / -1 rev; mode 0 manual / 1 copilot
bool failsafe = false, blocked = false, lvLock = false, driving = false;
uint32_t lastMsg = 0;

int outIN[2][2];                              // last duty written to [side][IN1, IN2] (Serial trace)
void wr(int s, int a, int b) { ledcWrite(IN1[s], a); ledcWrite(IN2[s], b); outIN[s][0] = a; outIN[s][1] = b; }
void setOut(int s, int a, int b) { if (INV[s]) { int t = a; a = b; b = t; } wr(s, a, b); }
void motor(int s, int dir, int pct) {         // pct 0..100 -> duty DUTY_START..DUTY_MAX
  if (dir == 0 || pct <= 0) { wr(s, 0, 0); return; }
  int duty = DUTY_START + (DUTY_MAX - DUTY_START) * constrain(pct, 0, 100) / 100;
  int a = SLOW_DECAY ? 255 : duty, b = SLOW_DECAY ? 255 - duty : 0;   // exactly U-10's drive()
  if (dir > 0) setOut(s, a, b); else setOut(s, b, a);
}
void logDrive() {                              // Serial trace whenever the drive command changes
  static int pg = 99, pt = -1, ps = 999;
  if (gear == pg && thr == pt && steer == ps) return;
  pg = gear; pt = thr; ps = steer;
  Serial.printf("[drive] gear %d  throttle %d%%  steer %d  mode %d%s%s | L IN1(G%d)=%d IN2(G%d)=%d | R IN1(G%d)=%d IN2(G%d)=%d\n",
    gear, thr, steer, mode, blocked ? " COPILOT-BLOCKED" : "", lvLock ? " LV-LOCK" : "",
    IN1[0], outIN[0][0], IN2[0], outIN[0][1], IN1[1], outIN[1][0], IN2[1], outIN[1][1]);
}
void stopAll(const char* why) {
  gear = 0; thr = 0; motor(0, 0, 0); motor(1, 0, 0);
  if (why) Serial.printf("[stop] %s\n", why);
}

// ------------------------------------------------------------------ encoders
const uint8_t ENC[2] = {34, 35};
volatile uint32_t cnt[2], eRise[2], eFall[2]; volatile bool armed[2];
void IRAM_ATTR encEdge(uint8_t i) {
  uint32_t now = micros();
  if (digitalRead(ENC[i])) { armed[i] = (now - eFall[i] >= 500); eRise[i] = now; }
  else { if (armed[i] && now - eRise[i] >= 500) cnt[i]++; armed[i] = false; eFall[i] = now; }
}
void IRAM_ATTR encL() { encEdge(0); }  void IRAM_ATTR encR() { encEdge(1); }
uint32_t snap(int i) { noInterrupts(); uint32_t c = cnt[i]; interrupts(); return c; }
float mps = 0; uint32_t prevC[2], tSpd = 0;

// ------------------------------------------------------------------ ultrasonics (pair scheme)
const uint8_t TRIG[4] = {5, 17, 16, 4}, ECHO[4] = {18, 19, 23, 13};   // F, L, R, B
const uint8_t PAIR[2] = {0b0110, 0b1001};
volatile uint32_t uRise[4], uFall[4], uSeen[4];
void IRAM_ATTR uEdge(uint8_t i) { uint32_t t = micros(); if (digitalRead(ECHO[i])) { uRise[i] = t; uSeen[i] = millis(); } else uFall[i] = t; }
void IRAM_ATTR u0() { uEdge(0); } void IRAM_ATTR u1() { uEdge(1); } void IRAM_ATTR u2() { uEdge(2); } void IRAM_ATTR u3() { uEdge(3); }
float dist[4] = {-1, -1, -1, -1}; bool usOk[4];
uint8_t slot = 0, lastMask = 0; uint32_t tSlot = 0;

void usService() {
  if (millis() - tSlot < 30) return;
  tSlot = millis();
  for (int i = 0; i < 4; i++) if (lastMask & (1 << i)) {
    uint32_t r = uRise[i], f = uFall[i];
    float cm = (r && f > r) ? (f - r) * 0.01717f : -1;
    dist[i] = (cm > 2 && cm < 400) ? cm : 400;          // no echo back = nothing in range
  }
  for (int i = 0; i < 4; i++) { usOk[i] = millis() - uSeen[i] < 1000; if (!usOk[i]) dist[i] = -1; }
  lastMask = PAIR[slot]; slot ^= 1;
  for (int i = 0; i < 4; i++) if (lastMask & (1 << i)) { uRise[i] = uFall[i] = 0; digitalWrite(TRIG[i], HIGH); }
  delayMicroseconds(10);
  for (int i = 0; i < 4; i++) if (lastMask & (1 << i)) digitalWrite(TRIG[i], LOW);
}

// ------------------------------------------------------------------ power (INA219 x2)
Adafruit_INA219 inaBat(0x41), inaPv(0x40);
bool batOk = false, pvOk = false;
float packV = 0, packI = 0, pvW = 0, soc = -1;
bool i2cHere(uint8_t a) { Wire.beginTransmission(a); return Wire.endTransmission() == 0; }
float socFromV(float v) {                     // rough resting curve per cell (INR18650-35E)
  const float c[] = {3.0, 3.3, 3.5, 3.6, 3.7, 3.8, 3.9, 4.0, 4.1, 4.2}, s[] = {0, 5, 12, 20, 35, 50, 65, 78, 90, 100};
  float x = v / 3;
  if (x <= c[0]) return 0;
  if (x >= c[9]) return 100;
  for (int k = 1; k < 10; k++) if (x < c[k]) return s[k - 1] + (s[k] - s[k - 1]) * (x - c[k - 1]) / (c[k] - c[k - 1]);
  return 100;
}
void powerCheck() {                           // 1 Hz: detect / re-init modules
  bool b = i2cHere(0x41), p = i2cHere(0x40);
  if (b && !batOk) { inaBat.begin(); Serial.println("[ina] battery 0x41 found"); }
  if (!b && batOk) Serial.println("[ina] battery 0x41 LOST");
  if (p && !pvOk) { inaPv.begin(); Serial.println("[ina] solar 0x40 found"); }
  if (!p && pvOk) Serial.println("[ina] solar 0x40 LOST");
  batOk = b; pvOk = p;
}
void powerRead(float dt) {                    // 10 Hz
  if (batOk) {
    packV = inaBat.getBusVoltage_V() + inaBat.getShuntVoltage_mV() / 1000;
    packI = inaBat.getCurrent_mA() / 1000;    // + = discharging, - = charging (U-06 / INT-02 sign)
    if (soc < 0) soc = socFromV(packV);       // first reading: start from the voltage
    soc = constrain(soc - packI * dt / 3600 / PACK_AH * 100, 0.0f, 100.0f);
  } else { packV = packI = 0; soc = -1; }
  pvW = pvOk ? inaPv.getBusVoltage_V() * inaPv.getCurrent_mA() / 1000 : 0;
  if (pvW < 0) pvW = 0;
}

// ------------------------------------------------------------------ log (5 Hz ring, ~11 min)
struct Row { uint32_t t; uint16_t mV; int16_t mA; uint16_t pvmW, mmps, d[4]; uint8_t soc, flags; };
const int LOG_N = 3300; Row* logBuf; int logHead = 0, logCount = 0;
void logRow() {
  if (!logBuf) return;
  Row& r = logBuf[logHead];
  r.t = millis(); r.mV = packV * 1000; r.mA = packI * 1000; r.pvmW = pvW * 1000; r.mmps = mps * 1000;
  for (int i = 0; i < 4; i++) r.d[i] = dist[i] < 0 ? 0xFFFF : (uint16_t)dist[i];
  r.soc = soc < 0 ? 255 : (uint8_t)soc;
  r.flags = (gear & 3) | (thr > 0) << 2 | failsafe << 3 | blocked << 4 | lvLock << 5 | batOk << 6 | pvOk << 7;
  logHead = (logHead + 1) % LOG_N; if (logCount < LOG_N) logCount++;
}

// ------------------------------------------------------------------ web
WebServer http(80);
WebSocketsServer ws(81);

void sendLog() {
  http.setContentLength(CONTENT_LENGTH_UNKNOWN);
  http.send(200, "text/csv", "t_s,pack_V,pack_A,pv_W,speed_mps,soc_pct,dF_cm,dL_cm,dR_cm,dB_cm,gear,driving,failsafe,copilot_block,lv_lock,bat_ok,pv_ok\n");
  char line[160]; String chunk;
  int start = (logHead - logCount + LOG_N) % LOG_N;
  for (int k = 0; k < logCount; k++) {
    Row& r = logBuf[(start + k) % LOG_N];
    int g = r.flags & 3; if (g == 3) g = -1;
    char d[4][8]; for (int i = 0; i < 4; i++) { if (r.d[i] == 0xFFFF) strcpy(d[i], ""); else snprintf(d[i], 8, "%u", r.d[i]); }
    snprintf(line, sizeof line, "%.2f,%.3f,%.3f,%.3f,%.3f,%s,%s,%s,%s,%s,%d,%d,%d,%d,%d,%d,%d\n",
      r.t / 1000.0, r.mV / 1000.0, r.mA / 1000.0, r.pvmW / 1000.0, r.mmps / 1000.0,
      r.soc == 255 ? "" : String(r.soc).c_str(), d[0], d[1], d[2], d[3], g,
      (r.flags >> 2) & 1, (r.flags >> 3) & 1, (r.flags >> 4) & 1, (r.flags >> 5) & 1, (r.flags >> 6) & 1, (r.flags >> 7) & 1);
    chunk += line;
    if (chunk.length() > 1400) { http.sendContent(chunk); chunk = ""; }
  }
  if (chunk.length()) http.sendContent(chunk);
  http.sendContent("");
}

void applyDrive() {
  if (gear == 0 || thr == 0 || lvLock || blocked) { motor(0, 0, 0); motor(1, 0, 0); return; }
  int l = thr * (100 + steer) / 100, r = thr * (100 - steer) / 100;   // steer -100..100, inner wheel slows to 0
  motor(0, gear, min(l, 100)); motor(1, gear, min(r, 100));
}

void onWs(uint8_t num, WStype_t type, uint8_t* payload, size_t len) {
  if (type == WStype_CONNECTED) { Serial.printf("[ws] client %u connected\n", num); lastMsg = millis(); return; }
  if (type == WStype_DISCONNECTED) {
    Serial.printf("[ws] client %u disconnected\n", num);
    if (ws.connectedClients() == 0 && gear != 0) { failsafe = true; stopAll("socket closed"); }
    return;
  }
  if (type != WStype_TEXT || len == 0) return;
  lastMsg = millis();
  char c = payload[0]; int v = atoi((const char*)payload + 1);
  switch (c) {
    case 'T': thr = constrain(v, 0, 100); break;
    case 'D': gear = v > 0 ? 1 : -1; thr = 0; failsafe = false; break;
    case 'S': gear = 0; thr = 0; break;
    case 'J': steer = constrain(v, -100, 100); break;
    case 'M': mode = (v == 1) ? 1 : 0; break;   // 2 (automatic) not implemented yet -> manual
    case 'K': break;                             // keep-alive
  }
}

void broadcast() {
  char b[360];
  snprintf(b, sizeof b,
    "{\"mps\":%.2f,\"v\":%.2f,\"i\":%.3f,\"w\":%.2f,\"soc\":%.1f,\"dF\":%.0f,\"dL\":%.0f,\"dR\":%.0f,\"dB\":%.0f,"
    "\"chg\":%d,\"fs\":%d,\"blk\":%d,\"lv\":%d,\"gear\":%d,\"ok\":{\"bat\":%d,\"pv\":%d,\"enc\":1,\"us\":[%d,%d,%d,%d]}}",
    mps, packV, packI, pvW, soc, dist[0], dist[1], dist[2], dist[3],
    batOk && packI < -0.02, failsafe, blocked, lvLock, gear, batOk, pvOk, usOk[0], usOk[1], usOk[2], usOk[3]);
  ws.broadcastTXT(b);
}

// ------------------------------------------------------------------ setup / loop
void setup() {
  Serial.begin(115200); delay(200); Serial.println("\ncar_main boot");
  for (int s = 0; s < 2; s++) {
    bool a = ledcAttach(IN1[s], 20000, 8), b = ledcAttach(IN2[s], 20000, 8); motor(s, 0, 0);
    Serial.printf("PWM %s: IN1 GPIO%d %s, IN2 GPIO%d %s\n", s ? "right" : "left", IN1[s], a ? "ok" : "FAILED", IN2[s], b ? "ok" : "FAILED");
  }
  pinMode(MOVING_PIN, OUTPUT); digitalWrite(MOVING_PIN, LOW);
  for (int i = 0; i < 2; i++) pinMode(ENC[i], INPUT);
  attachInterrupt(ENC[0], encL, CHANGE); attachInterrupt(ENC[1], encR, CHANGE);
  for (int i = 0; i < 4; i++) { pinMode(TRIG[i], OUTPUT); digitalWrite(TRIG[i], LOW); pinMode(ECHO[i], INPUT); }
  attachInterrupt(ECHO[0], u0, CHANGE); attachInterrupt(ECHO[1], u1, CHANGE);
  attachInterrupt(ECHO[2], u2, CHANGE); attachInterrupt(ECHO[3], u3, CHANGE);
  Wire.begin(21, 22); powerCheck();
  logBuf = (Row*)malloc(sizeof(Row) * LOG_N);
  Serial.printf("log buffer %s (%u bytes)\n", logBuf ? "ok" : "FAILED", (unsigned)(sizeof(Row) * LOG_N));

  WiFi.softAP("SolarCar", "solarcar1");
  http.on("/", []() { http.sendHeader("Content-Encoding", "gzip"); http.send_P(200, "text/html", (const char*)DASH_GZ, DASH_GZ_LEN); });
  http.on("/log.csv", sendLog);
  http.on("/log/clear", []() { logHead = logCount = 0; http.send(200, "text/plain", "log cleared"); });
  http.begin();
  ws.begin(); ws.onEvent(onWs);
  Serial.printf("AP SolarCar up, http://%s  (bat INA %s, PV INA %s)\n", WiFi.softAPIP().toString().c_str(),
                batOk ? "ok" : "missing", pvOk ? "ok" : "missing");
}

void loop() {
  ws.loop(); http.handleClient();
  usService();
  uint32_t now = millis();

  // link-loss failsafe
  if (gear != 0 && now - lastMsg > LINK_TIMEOUT_MS) { failsafe = true; stopAll("link lost > 500 ms"); }

  // copilot: block motion toward a close obstacle
  float ahead = gear > 0 ? dist[0] : gear < 0 ? dist[3] : -1;
  if (mode == 1 && ahead > 0 && ahead < 400) { if (ahead < STOP_CM) blocked = true; else if (ahead > CLEAR_CM) blocked = false; }
  else blocked = false;

  applyDrive(); logDrive();
  driving = gear != 0 && thr > 0 && !blocked && !lvLock;
  digitalWrite(MOVING_PIN, driving);

  static uint32_t t1 = 0, t10 = 0, t5 = 0, tLow = 0;
  if (now - t1 >= 1000) { t1 = now; powerCheck(); }
  if (now - tSpd >= 250) {                     // speed from encoders, 4 Hz window, both wheels averaged
    float dt = (now - tSpd) / 1000.0; tSpd = now;
    uint32_t c0 = snap(0), c1 = snap(1);
    mps = ((c0 - prevC[0]) + (c1 - prevC[1])) / 2.0 / SLOTS * PI * WHEEL_MM / 1000 / dt;
    prevC[0] = c0; prevC[1] = c1;
  }
  if (now - t10 >= 100) {
    float dt = (now - t10) / 1000.0; t10 = now;
    powerRead(t10 ? dt : 0.1);
    if (batOk && packV > 1 && packV < LV_CUT) { if (!tLow) tLow = now; else if (now - tLow > 3000 && !lvLock) { lvLock = true; stopAll("pack below 9.0 V"); } }
    else tLow = 0;
    broadcast();
  }
  if (now - t5 >= 200) { t5 = now; logRow(); }
}
