// =====================================================================================================
// car_main_v4 — main firmware for the EEE4022S solar car (ESP32 Dev Module, ESP32 Arduino core 3.x)
// =====================================================================================================
// What this program does
//   - Creates the car's own Wi-Fi network and serves the phone dashboard from flash memory.
//   - Receives drive commands from the dashboard over a WebSocket and drives the motors.
//   - Reads the wheel encoders, the four ultrasonic sensors and the two INA219 power sensors.
//   - Sends live telemetry to the dashboard 10 times a second and logs it in RAM for CSV download.
//   - Co-pilot mode: steers away from side obstacles and stops before frontal ones.
//   - Safety: stops if the phone link is lost, and locks the drive if the pack voltage stays too low.
//
// How to use it
//   Phone: join Wi-Fi "UCT_SolarCar_2026_TRKJON002" (password solarcar2026), then open http://192.168.4.1
//   Log:   http://192.168.4.1/log.csv downloads the log, http://192.168.4.1/log/clear empties it.
//   Editing the dashboard: change page/dashboard.html, then run "python make_header.py" and re-flash.
//
// Libraries: WiFi, WebServer (both part of the ESP32 Arduino core), "WebSockets" by Markus Sattler,
//            "Adafruit INA219". Board setting: ESP32 Dev Module, Flash 40 MHz / DIO.
//
// Pins (as in the KiCad schematic, report Fig. 3.2)
//   Motor drivers : DRV8871 left IN1/IN2 = 25/26, right IN1/IN2 = 27/14.
//                   Forward only: IN1 held LOW, PWM on IN2. Stop = both LOW (coast).
//   Encoders      : left 34, right 35 (30-slot discs, 65 mm wheels).
//   Ultrasonics   : TRIG/ECHO front 17/19, left 5/18, right 16/23, back 4/13
//                   (front and left are swapped relative to the schematic, as wired on the car).
//   INA219 (I2C)  : 0x40 = pack (voltage, current, SOC), 0x41 = MPPT output (solar power). SDA 21, SCL 22.
//   MOVING line   : GPIO32 -> tracker Nano D2, HIGH while driving (the tracker then parks the panels flat).
//
// Messages
//   Phone -> car (text): T0..100 throttle | D1 drive | S stop | J-100..100 steer | M0 manual / M1 co-pilot
//                        | K keep-alive (sent every 200 ms)
//   Car -> phone (JSON, 10 Hz): mps v i w soc dF dL dR dB chg fs blk lv gear ok{bat,pv,enc,us[4]}
//
// Sensor faults: a sensor that stops responding is reported as missing; its dashboard tile greys out and
// the rest of the car keeps working.
// =====================================================================================================
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include "dashboard_gz.h"                     // the dashboard page, gzipped: DASH_GZ[] (in flash), DASH_GZ_LEN

// ===================================================================== settings you can tune
// Motor PWM (8-bit, 0..255). Throttle 1..100 % maps onto DUTY_START..DUTY_MAX.
const int   DUTY_MAX   = 238;                 // top of the throttle range (about 93 % duty)
const int   DUTY_START = 102;                 // duty at 1 % throttle (40 %). The car only starts moving at about
                                              // 64 % duty at full weight (INT-02), so the bottom of the slider
                                              // is a gentle, non-moving start by choice.
const uint32_t LINK_TIMEOUT_MS = 500;         // stop if no message from the phone for this long

// Co-pilot front guard: stop below STOP_CM in front, release above CLEAR_CM (the gap avoids chatter).
const float STOP_CM  = 25;
const float CLEAR_CM = STOP_CM + 5;

// Co-pilot side correction (proportional only), report Section 3.5:
//   closeness p = (D_ACT_CM - d) / D_ACT_CM when d < D_ACT_CM, else 0   (1 = touching, 0 = far or no echo)
//   error     e = p_left - p_right
//   correction u = W_S * KP * e, added to the left side and subtracted from the right
//   (u > 0 means the obstacle is on the left, so the car turns right). In manual mode u = 0.
const float D_ACT_CM = 60;                    // side sensors start reacting below this distance (cm)
const float KP       = 100;                   // %-points of wheel speed per unit of error
const float E_DEAD   = 0.10;                  // dead band: |e| below this gives no correction
const float W_U      = 1.0;                   // weight on the driver's throttle and steering
const float W_S      = 1.0;                   // weight on the sensor correction

const float LV_CUT  = 9.0;                    // over-discharge lock: pack below this for 3 s locks the drive
const float PACK_AH = 3.35;                   // pack capacity, Samsung INR18650-35E standard capacity (Ah)
const int   SLOTS = 30;                       // encoder slots per wheel revolution
const float WHEEL_MM = 65.0;                  // wheel diameter (mm)

// ===================================================================== drive
const int IN1[2] = {25, 27}, IN2[2] = {26, 14};   // [0] = left, [1] = right
const int MOVING_PIN = 32;
int gear = 0, thr = 0, steer = 0, mode = 0;   // gear: 1 drive / 0 stop; mode: 0 manual / 1 co-pilot
bool failsafe = false;                        // true after a link-loss stop
bool blocked = false;                         // true while the co-pilot front guard holds the car
bool lvLock = false;                          // true after the over-discharge lock (until reset)
uint32_t lastMsg = 0;                         // time of the last message from the phone
float cpE = 0, cpU = 0;                       // co-pilot error e and correction u

int outIN[2][2];                              // last duty written to [side][IN1, IN2], for the Serial trace
void wr(int s, int a, int b) { ledcWrite(IN1[s], a); ledcWrite(IN2[s], b); outIN[s][0] = a; outIN[s][1] = b; }

// Set one side's motors: pct 0..100 % -> PWM duty on IN2 (IN1 stays LOW). 0 % or off = coast.
void motor(int s, int on, int pct) {
  if (!on || pct <= 0) { wr(s, 0, 0); return; }
  wr(s, 0, DUTY_START + (DUTY_MAX - DUTY_START) * constrain(pct, 0, 100) / 100);
}

void logDrive() {                             // Serial trace, printed only when the drive command changes
  static int pg = 99, pt = -1, ps = 999;
  if (gear == pg && thr == pt && steer == ps) return;
  pg = gear; pt = thr; ps = steer;
  Serial.printf("[drive] gear %d  throttle %d%%  steer %d  mode %d%s%s | L IN1(G%d)=%d IN2(G%d)=%d | R IN1(G%d)=%d IN2(G%d)=%d\n",
    gear, thr, steer, mode, blocked ? " COPILOT-BLOCKED" : "", lvLock ? " LV-LOCK" : "",
    IN1[0], outIN[0][0], IN2[0], outIN[0][1], IN1[1], outIN[1][0], IN2[1], outIN[1][1]);
}

void stopAll(const char* why) {               // stop both sides and return to STOP
  gear = 0; thr = 0; motor(0, 0, 0); motor(1, 0, 0);
  if (why) Serial.printf("[stop] %s\n", why);
}

// ===================================================================== wheel encoders
// Each encoder pin interrupts on every edge. A slot is counted on its falling edge only if the pin was
// LOW for at least 500 us before the rising edge AND HIGH for at least 500 us after it; shorter edges are
// treated as noise (the 500 us filter in report Section 3.5, tested in U-09).
const uint8_t ENC[2] = {34, 35};
volatile uint32_t cnt[2], eRise[2], eFall[2];
volatile bool armed[2];
void IRAM_ATTR encEdge(uint8_t i) {
  uint32_t now = micros();
  if (digitalRead(ENC[i])) { armed[i] = (now - eFall[i] >= 500); eRise[i] = now; }                  // rising edge
  else { if (armed[i] && now - eRise[i] >= 500) cnt[i]++; armed[i] = false; eFall[i] = now; }       // falling edge
}
void IRAM_ATTR encL() { encEdge(0); }
void IRAM_ATTR encR() { encEdge(1); }
uint32_t snap(int i) { noInterrupts(); uint32_t c = cnt[i]; interrupts(); return c; }   // safe copy of a count
float mps = 0;                                // speed (m/s), updated every 250 ms in loop()
uint32_t prevC[2], tSpd = 0;

// ===================================================================== ultrasonic sensors
// Sensors fired together can hear each other's pulses (crosstalk), so they fire in pairs:
// left + right, then front + back, 30 ms apart (longer than the 23 ms round trip at 4 m). Report Section 3.5, U-10.
// Echo pulse times are captured by interrupts. No echo for 1 s = sensor reported missing.
const uint8_t TRIG[4] = {17, 5, 16, 4}, ECHO[4] = {19, 18, 23, 13};   // index 0..3 = front, left, right, back
const uint8_t PAIR[2] = {0b0110, 0b1001};     // bit masks: {left + right}, {front + back}
volatile uint32_t uRise[4], uFall[4], uSeen[4];
void IRAM_ATTR uEdge(uint8_t i) { uint32_t t = micros(); if (digitalRead(ECHO[i])) { uRise[i] = t; uSeen[i] = millis(); } else uFall[i] = t; }
void IRAM_ATTR u0() { uEdge(0); } void IRAM_ATTR u1() { uEdge(1); } void IRAM_ATTR u2() { uEdge(2); } void IRAM_ATTR u3() { uEdge(3); }
float dist[4] = {-1, -1, -1, -1};             // distance (cm); 400 = nothing in range; -1 = sensor missing
bool usOk[4];
uint8_t slot = 0, lastMask = 0;
uint32_t tSlot = 0;

void usService() {                            // called every loop; acts every 30 ms
  if (millis() - tSlot < 30) return;
  tSlot = millis();
  // 1. read the pair fired last time: distance (cm) = echo time (us) x 0.01717 (speed of sound / 2)
  for (int i = 0; i < 4; i++) if (lastMask & (1 << i)) {
    uint32_t r = uRise[i], f = uFall[i];
    float cm = (r && f > r) ? (f - r) * 0.01717f : -1;
    dist[i] = (cm > 2 && cm < 400) ? cm : 400;
  }
  for (int i = 0; i < 4; i++) { usOk[i] = millis() - uSeen[i] < 1000; if (!usOk[i]) dist[i] = -1; }
  // 2. fire the other pair (10 us trigger pulse)
  lastMask = PAIR[slot]; slot ^= 1;
  for (int i = 0; i < 4; i++) if (lastMask & (1 << i)) { uRise[i] = uFall[i] = 0; digitalWrite(TRIG[i], HIGH); }
  delayMicroseconds(10);
  for (int i = 0; i < 4; i++) if (lastMask & (1 << i)) digitalWrite(TRIG[i], LOW);
}

// ===================================================================== power sensing (two INA219s) and SOC
Adafruit_INA219 inaBat(0x40), inaPv(0x41);   // 0x40 = pack (net current, both directions), 0x41 = MPPT output
bool batOk = false, pvOk = false;             // true while each sensor answers on I2C
float packV = 0, packI = 0;                   // pack voltage (V) and current (A, + = discharging, - = charging)
float pvW = 0, pvI = 0;                       // solar power (W) and current (A) at the MPPT output
float soc = -1;                               // SOC shown on the dashboard (%); -1 = no battery reading
float socEst = -1;                            // running SOC estimate, kept if the sensor drops out
bool i2cHere(uint8_t a) { Wire.beginTransmission(a); return Wire.endTransmission() == 0; }

// Initial SOC from the rested pack voltage: linear interpolation in the open-circuit-voltage table
// (report Table 3.6; Samsung INR18650-26E rested OCV per cell [48], as no free 35E table was available).
float socFromV(float v) {
  const float c[] = {2.80, 3.4212, 3.5024, 3.5517, 3.5877, 3.6272, 3.6867, 3.7732, 3.8702, 3.9732, 4.1307},
              s[] = {0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100};
  float x = v / 3;                            // per-cell voltage; assumes balanced cells (passive balancer, U-01)
  if (x <= c[0]) return 0;
  if (x >= c[10]) return 100;
  for (int k = 1; k < 11; k++) if (x < c[k]) return s[k - 1] + (s[k] - s[k - 1]) * (x - c[k - 1]) / (c[k] - c[k - 1]);
  return 100;
}

void powerCheck() {                           // once a second: find sensors that appear or disappear
  bool b = i2cHere(0x40), p = i2cHere(0x41);
  if (b && !batOk) { inaBat.begin(); Serial.println("[ina] battery 0x40 found"); }
  if (!b && batOk) Serial.println("[ina] battery 0x40 LOST");
  if (p && !pvOk) { inaPv.begin(); Serial.println("[ina] solar 0x41 found"); }
  if (!p && pvOk) Serial.println("[ina] solar 0x41 LOST");
  batOk = b; pvOk = p;
}

void powerRead(float dt) {                    // every 100 ms; dt = seconds since the last call
  if (batOk) {
    packV = inaBat.getBusVoltage_V() + inaBat.getShuntVoltage_mV() / 1000;
    packI = inaBat.getCurrent_mA() / 1000;
    if (socEst < 0) socEst = socFromV(packV); // first reading only: start from the rested voltage at power-up
    // coulomb counting (report Eq. 3.6): SOC(k) = SOC(k-1) - 100 * I * dt / (3600 * Q), coulombic efficiency = 1
    socEst = constrain(socEst - packI * dt / 3600 / PACK_AH * 100, 0.0f, 100.0f);
    soc = socEst;
  } else {
    packV = packI = 0; soc = -1;              // sensor missing: hide SOC but keep socEst, so a reconnection
  }                                           // under load does not restart from a sagging voltage
  pvI = pvOk ? inaPv.getCurrent_mA() / 1000 : 0;
  if (pvI < 0) pvI = 0;
  pvW = pvOk ? inaPv.getBusVoltage_V() * pvI : 0;
}

// ===================================================================== data log (RAM ring buffer)
// 3300 rows: 5 Hz while driving, 1 Hz when parked (about 55 min parked). Downloaded as CSV from /log.csv.
struct Row { uint32_t t; uint16_t mV; int16_t mA; uint16_t pvmW, pvmA, mmps, d[4]; uint8_t soc, flags; int8_t u; };
const int LOG_N = 3300;
Row* logBuf;
int logHead = 0, logCount = 0;

void logRow() {
  if (!logBuf) return;
  Row& r = logBuf[logHead];
  r.t = millis(); r.mV = packV * 1000; r.mA = packI * 1000; r.pvmW = pvW * 1000; r.pvmA = pvI * 1000; r.mmps = mps * 1000;
  for (int i = 0; i < 4; i++) r.d[i] = dist[i] < 0 ? 0xFFFF : (uint16_t)dist[i];
  r.soc = soc < 0 ? 255 : (uint8_t)soc;
  r.u = (int8_t)constrain((int)lroundf(cpU), -127, 127);
  r.flags = (gear & 1) | (thr > 0) << 2 | failsafe << 3 | blocked << 4 | lvLock << 5 | batOk << 6 | pvOk << 7;
  logHead = (logHead + 1) % LOG_N; if (logCount < LOG_N) logCount++;
}

// ===================================================================== web: HTTP page + WebSocket
WebServer http(80);                           // serves the dashboard page and the log (HTTP, port 80)
WebSocketsServer ws(81);                      // live two-way link with the page (WebSocket, port 81)

void sendLog() {                              // stream the log as CSV, in chunks
  http.setContentLength(CONTENT_LENGTH_UNKNOWN);
  http.send(200, "text/csv", "t_s,pack_V,pack_A,pv_W,pv_A,speed_mps,soc_pct,dF_cm,dL_cm,dR_cm,dB_cm,gear,driving,failsafe,copilot_block,lv_lock,bat_ok,pv_ok,copilot_u\n");
  char line[200]; String chunk;
  int start = (logHead - logCount + LOG_N) % LOG_N;
  for (int k = 0; k < logCount; k++) {
    Row& r = logBuf[(start + k) % LOG_N];
    char d[4][8]; for (int i = 0; i < 4; i++) { if (r.d[i] == 0xFFFF) strcpy(d[i], ""); else snprintf(d[i], 8, "%u", r.d[i]); }
    snprintf(line, sizeof line, "%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%s,%s,%s,%s,%s,%d,%d,%d,%d,%d,%d,%d,%d\n",
      r.t / 1000.0, r.mV / 1000.0, r.mA / 1000.0, r.pvmW / 1000.0, r.pvmA / 1000.0, r.mmps / 1000.0,
      r.soc == 255 ? "" : String(r.soc).c_str(), d[0], d[1], d[2], d[3], r.flags & 1,
      (r.flags >> 2) & 1, (r.flags >> 3) & 1, (r.flags >> 4) & 1, (r.flags >> 5) & 1, (r.flags >> 6) & 1, (r.flags >> 7) & 1, r.u);
    chunk += line;
    if (chunk.length() > 1400) { http.sendContent(chunk); chunk = ""; }
  }
  if (chunk.length()) http.sendContent(chunk);
  http.sendContent("");
}

// ===================================================================== co-pilot and drive mixing
float prox(float d) { return (d > 0 && d < D_ACT_CM) ? (D_ACT_CM - d) / D_ACT_CM : 0; }   // closeness p

void copilotUpdate() {                        // 20 Hz: side correction u (zero in manual mode or when stopped)
  bool active = mode == 1 && gear == 1 && thr > 0;
  cpE = prox(dist[1]) - prox(dist[2]);        // left closeness - right closeness
  cpU = (active && fabsf(cpE) >= E_DEAD) ? W_S * KP * cpE : 0;
}

// Mixer (report Section 3.5, Fig. 3.5): L = W_U*T*(1+S) + u, R = W_U*T*(1-S) - u, with S = steer/100.
// If one side asks for more than 100 %, the excess is taken off the other side so the turn is kept.
void applyDrive() {
  if (gear == 0 || thr == 0 || lvLock || blocked) { motor(0, 0, 0); motor(1, 0, 0); return; }
  float base = W_U * thr;
  float l = base * (100 + steer) / 100 + cpU, r = base * (100 - steer) / 100 - cpU;
  if (l > 100) { r -= l - 100; l = 100; }
  if (r > 100) { l -= r - 100; r = 100; }
  motor(0, gear, constrain((int)lroundf(l), 0, 100)); motor(1, gear, constrain((int)lroundf(r), 0, 100));
}

// ===================================================================== WebSocket messages
void onWs(uint8_t num, WStype_t type, uint8_t* payload, size_t len) {
  if (type == WStype_CONNECTED) { Serial.printf("[ws] client %u connected\n", num); lastMsg = millis(); return; }
  if (type == WStype_DISCONNECTED) {
    Serial.printf("[ws] client %u disconnected\n", num);
    if (ws.connectedClients() == 0 && gear != 0) { failsafe = true; stopAll("socket closed"); }
    return;
  }
  if (type != WStype_TEXT || len == 0) return;
  lastMsg = millis();
  char c = payload[0]; int v = atoi((const char*)payload + 1);   // one letter + a number
  switch (c) {
    case 'T': thr = constrain(v, 0, 100); break;                       // throttle %
    case 'D': if (v > 0) { gear = 1; thr = 0; failsafe = false; } break; // DRIVE (always starts from rest)
    case 'S': gear = 0; thr = 0; break;                                 // STOP
    case 'J': steer = constrain(v, -100, 100); break;                   // joystick x
    case 'M': mode = (v == 1) ? 1 : 0; break;                           // manual / co-pilot
    case 'K': break;                                                    // keep-alive
  }
}

void broadcast() {                            // 10 Hz telemetry to every connected phone
  char b[400];
  snprintf(b, sizeof b,
    "{\"mps\":%.2f,\"v\":%.2f,\"i\":%.3f,\"w\":%.2f,\"soc\":%.1f,\"dF\":%.0f,\"dL\":%.0f,\"dR\":%.0f,\"dB\":%.0f,"
    "\"chg\":%d,\"fs\":%d,\"blk\":%d,\"lv\":%d,\"gear\":%d,\"ok\":{\"bat\":%d,\"pv\":%d,\"enc\":1,\"us\":[%d,%d,%d,%d]}}",
    mps, packV, packI, pvW, soc, dist[0], dist[1], dist[2], dist[3],
    batOk && packI < -0.02,                   // chg: current flowing into the pack (from USB or solar)
    failsafe, blocked, lvLock, gear, batOk, pvOk, usOk[0], usOk[1], usOk[2], usOk[3]);
  ws.broadcastTXT(b);
}

// ===================================================================== setup
void setup() {
  Serial.begin(115200); delay(200); Serial.println("\ncar_main boot");
  for (int s = 0; s < 2; s++) {               // 20 kHz, 8-bit PWM on all four driver inputs
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

  WiFi.softAP("UCT_SolarCar_2026_TRKJON002", "solarcar2026");   // the car's own Wi-Fi network (192.168.4.1)
  http.on("/", []() {                                          // the dashboard, sent gzipped from flash
    http.sendHeader("Content-Encoding", "gzip");
    http.send_P(200, "text/html", (const char*)DASH_GZ, DASH_GZ_LEN);
  });
  http.on("/log.csv", sendLog);
  http.on("/log/clear", []() { logHead = logCount = 0; http.send(200, "text/plain", "log cleared"); });
  http.begin();
  ws.begin(); ws.onEvent(onWs);
  Serial.printf("AP UCT_SolarCar_2026_TRKJON002 up, http://%s  (bat INA %s, PV INA %s)\n", WiFi.softAPIP().toString().c_str(),
                batOk ? "ok" : "missing", pvOk ? "ok" : "missing");
}

// ===================================================================== main loop
void loop() {
  ws.loop(); http.handleClient();             // handle phone messages and page requests
  usService();                                // ultrasonic pairs, every 30 ms
  uint32_t now = millis();

  // link-loss failsafe: no message from the phone for 500 ms -> stop
  if (gear != 0 && now - lastMsg > LINK_TIMEOUT_MS) { failsafe = true; stopAll("link lost > 500 ms"); }

  // co-pilot front guard (front sensor only; the car drives forward only)
  float ahead = gear > 0 ? dist[0] : -1;
  if (mode == 1 && ahead > 0 && ahead < 400) { if (ahead < STOP_CM) blocked = true; else if (ahead > CLEAR_CM) blocked = false; }
  else blocked = false;

  // co-pilot side correction, 20 Hz
  static uint32_t tCp = 0, tPr = 0;
  if (now - tCp >= 50) {
    tCp = now; copilotUpdate();
    if (cpU != 0 && now - tPr >= 250) { tPr = now;
      Serial.printf("[copilot] dL %.0f dR %.0f  e %+.2f  u %+.1f\n", dist[1], dist[2], cpE, cpU); }
  }

  applyDrive(); logDrive();
  digitalWrite(MOVING_PIN, gear != 0 && thr > 0 && !blocked && !lvLock);   // tell the tracker the car is moving

  static uint32_t t1 = 0, t10 = 0, t5 = 0, tLow = 0;
  if (now - t1 >= 1000) { t1 = now; powerCheck(); }

  // speed, every 250 ms: new slots per wheel, averaged over both wheels
  // revolutions = counts / SLOTS; distance = revolutions x pi x wheel diameter; speed = distance / dt
  if (now - tSpd >= 250) {
    float dt = (now - tSpd) / 1000.0; tSpd = now;
    uint32_t c0 = snap(0), c1 = snap(1);
    mps = ((c0 - prevC[0]) + (c1 - prevC[1])) / 2.0 / SLOTS * PI * WHEEL_MM / 1000 / dt;
    prevC[0] = c0; prevC[1] = c1;
  }

  // power, SOC, over-discharge lock and telemetry, every 100 ms
  if (now - t10 >= 100) {
    float dt = t10 ? (now - t10) / 1000.0 : 0.1;   // first call: assume one 100 ms step
    t10 = now;
    powerRead(dt);
    if (batOk && packV > 1 && packV < LV_CUT) {    // pack below LV_CUT for 3 s -> lock the drive until reset
      if (!tLow) tLow = now;
      else if (now - tLow > 3000 && !lvLock) { lvLock = true; stopAll("pack below 9.0 V"); }
    } else tLow = 0;
    broadcast();
  }

  // log a row: 5 Hz while driving or moving, 1 Hz when parked (stretches the log for charging sessions)
  if (now - t5 >= (gear != 0 || mps > 0.01 ? 200u : 1000u)) { t5 = now; logRow(); }
}
