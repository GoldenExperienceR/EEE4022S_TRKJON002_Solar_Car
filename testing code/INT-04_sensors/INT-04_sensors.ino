// INT-04 — Sensors in the chassis: speed sensors + 4 HC-SR04 (pair scheme) + drive, ESP32 on Buck A (no USB).
// Pins = KiCad ESP32 schematic. Motors L 25/26, R 27/14 | Encoders SPD_L 34, SPD_R 35 (30-slot discs)
// Ultrasonic TRIG/ECHO: F 5/18, L 17/19, R 16/23, B 4/13 (ECHO via 2.2k/4.7k divider). INA219 battery 0x41.
// Phone: join "SolarCar-INT04" (password solarcar1), open http://192.168.4.1
//   Zero = reset counts and sensor min/max | Fwd / Coast / Brake at the speed slider
//   Drive 2 m / Drive 5 m = drive straight, BRAKE when the average encoder distance reaches the target
//   Full speed = wheels lifted, 100 % command, read steady RPM and m/s (expected max speed)
// Ultrasonic: L+R fired together, 30 ms, F+B together, 30 ms (60 ms cycle, U-13 decision). Each sensor shows
//   last / min / max since Zero and a spike count (jump > 20 cm between consecutive readings).
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>

// ---- drive (as INT-03) ----
const int IN1[2] = {25, 27}, IN2[2] = {26, 14};
const bool INV[2] = {true, true};
const int DUTY_MAX = 238, DUTY_MIN = 51;              // 7.0 V / 7.5 V cap; ~20 % start threshold
void setOut(int s, int a, int b) { if (INV[s]) { int t = a; a = b; b = t; } ledcWrite(IN1[s], a); ledcWrite(IN2[s], b); }
// Supplier reference (19/10): FWD = H/L, REV = L/H, STOP = L/L; speed = PWM on the HIGH input. HH never used.
void drive(int s, int dir, int duty) {                // dir 1 fwd, -1 rev, 0/2 stop (LL)
  if (dir == 0 || dir == 2 || duty == 0) { ledcWrite(IN1[s], 0); ledcWrite(IN2[s], 0); return; }
  if (dir > 0) setOut(s, duty, 0); else setOut(s, 0, duty);
}
int dutyFromPct(int p) { return p <= 0 ? 0 : DUTY_MIN + (DUTY_MAX - DUTY_MIN) * p / 100; }
void both(int dir, int duty) { drive(0, dir, duty); drive(1, dir, duty); }

// ---- encoders (U-12 filter: falling edge, >= 500 us low then high) ----
const uint8_t ENC[2] = {34, 35};
const int SLOTS = 30; const float WHEEL_MM = 65.0;
const uint32_t MIN_PULSE_US = 500;
volatile uint32_t cnt[2], eRise[2], eFall[2]; volatile bool armed[2];
void IRAM_ATTR encEdge(uint8_t i) {
  uint32_t now = micros();
  if (digitalRead(ENC[i])) { armed[i] = (now - eFall[i] >= MIN_PULSE_US); eRise[i] = now; }
  else { if (armed[i] && now - eRise[i] >= MIN_PULSE_US) cnt[i]++; armed[i] = false; eFall[i] = now; }
}
void IRAM_ATTR encL() { encEdge(0); }  void IRAM_ATTR encR() { encEdge(1); }
uint32_t snap(int i) { noInterrupts(); uint32_t c = cnt[i]; interrupts(); return c; }
float metres(uint32_t c) { return c * PI * WHEEL_MM / 1000.0 / SLOTS; }

// ---- ultrasonic, pair scheme ----
const char* UN[4] = {"F", "L", "R", "B"};
const uint8_t TRIG[4] = {5, 17, 16, 4}, ECHO[4] = {18, 19, 23, 13};
uint16_t GAP_MS = 30; const float CM_PER_US = 0.01717;
const uint8_t PAIR[2] = {0b0110, 0b1001};             // L+R, then F+B
const uint8_t SEQ[4] = {0b0001, 0b0010, 0b0100, 0b1000}; // F, L, R, B one at a time
bool seqMode = false;                                 // page: "US pairs" / "US one-by-one", gap buttons
volatile uint32_t uRise[4], uFall[4];
void IRAM_ATTR uEdge(uint8_t i) { if (digitalRead(ECHO[i])) uRise[i] = micros(); else uFall[i] = micros(); }
void IRAM_ATTR u0() { uEdge(0); } void IRAM_ATTR u1() { uEdge(1); } void IRAM_ATTR u2() { uEdge(2); } void IRAM_ATTR u3() { uEdge(3); }
float uLast[4] = {-1, -1, -1, -1}, uMin[4], uMax[4]; uint32_t uSpikes[4], uMiss[4], uReads[4];
uint8_t slot = 0, lastMask = 0; uint32_t tSlot = 0, cycleStart = 0, cycleMs = 0;

void readSlot(uint8_t mask) {
  for (int i = 0; i < 4; i++) if (mask & (1 << i)) {
    uint32_t r = uRise[i], f = uFall[i];
    float cm = (r && f > r) ? (f - r) * CM_PER_US : -1;
    uReads[i]++;
    if (cm <= 0 || cm > 400) { uMiss[i]++; continue; }
    if (uLast[i] > 0 && fabs(cm - uLast[i]) > 20) uSpikes[i]++;
    uLast[i] = cm; uMin[i] = min(uMin[i], cm); uMax[i] = max(uMax[i], cm);
  }
}
void fireSlot(uint8_t mask) {
  for (int i = 0; i < 4; i++) if (mask & (1 << i)) { uRise[i] = uFall[i] = 0; digitalWrite(TRIG[i], HIGH); }
  delayMicroseconds(10);
  for (int i = 0; i < 4; i++) if (mask & (1 << i)) digitalWrite(TRIG[i], LOW);
}
void zeroAll() {
  noInterrupts(); cnt[0] = cnt[1] = 0; interrupts();
  for (int i = 0; i < 4; i++) { uMin[i] = 1e9; uMax[i] = 0; uSpikes[i] = uMiss[i] = uReads[i] = 0; uLast[i] = -1; }
}

// ---- modes ----
enum Mode { IDLE, MANUAL, TARGET, FULL } mode = IDLE;
const char* MN[] = {"idle", "manual fwd", "drive to target", "full speed"};
int speedPct = 60; float targetM = 0, stopAtM = -1;
float rpm[2]; uint32_t tRpm = 0, prevC[2];

WebServer server(80); Preferences prefs; uint32_t boots; const char* reason;
const char* resetName(esp_reset_reason_t r) {
  switch (r) { case ESP_RST_POWERON: return "power-on"; case ESP_RST_EXT: return "EN button"; case ESP_RST_SW: return "software";
    case ESP_RST_PANIC: return "crash"; case ESP_RST_BROWNOUT: return "BROWNOUT"; default: return "other/watchdog"; }
}

const char PAGE[] PROGMEM = R"HTML(<html><head><meta name='viewport' content='width=device-width'><style>
body{font-family:sans-serif;margin:10px}button{font-size:18px;padding:10px;margin:3px;min-width:80px}
table{border-collapse:collapse;font-size:16px}td,th{border:1px solid #999;padding:3px 6px}</style></head><body>
<div><button onclick="c('z')">Zero</button><button onclick="c('f')">Fwd</button><button onclick="c('c')">Coast</button><button style='background:#e66' onclick="c('b')">Stop</button></div>
<div><button onclick="c('t2')">Drive 2 m</button><button onclick="c('t5')">Drive 5 m</button><button onclick="c('m')">Full speed</button></div>
<div><button onclick="c('up')">US pairs</button><button onclick="c('us')">US one-by-one</button>
<button onclick="c('g20')">gap 20</button><button onclick="c('g25')">gap 25</button><button onclick="c('g30')">gap 30</button><button onclick="c('g40')">gap 40</button><button onclick="c('g50')">gap 50</button></div>
<div>Raw IN1 IN2: <button onclick="c('k00')">LL</button><button onclick="c('k01')">LH</button><button onclick="c('k10')">HL</button><button onclick="c('k11')">HH</button></div>
<div>Speed <input type=range min=0 max=100 value=60 style='width:55%' onchange="c('v'+this.value)"> <span id=sp>60</span>%</div>
<div id=st></div><script>
function c(x){fetch('/c?x='+x);}
setInterval(()=>fetch('/s').then(r=>r.json()).then(j=>{let u='<table><tr><th></th><th>last</th><th>min</th><th>max</th><th>spikes</th><th>miss</th></tr>';
for(const s of j.us)u+='<tr><td>'+s.n+'</td><td>'+s.l+'</td><td>'+s.mn+'</td><td>'+s.mx+'</td><td>'+s.sp+'</td><td>'+s.ms+'</td></tr>';
document.getElementById('sp').textContent=j.sp;
document.getElementById('st').innerHTML='Mode: <b>'+j.mode+'</b> | boots '+j.boots+' ('+j.reason+')<br>Counts L '+j.cL+' R '+j.cR+
'<br>Distance L '+j.dL+' R '+j.dR+' avg <b>'+j.dA+' m</b>'+(j.stop>=0?' | braked at '+j.stop+' m':'')+
'<br>RPM L '+j.rL+' R '+j.rR+' | speed '+j.ms+' m/s<br>US '+j.sch+', gap '+j.gap+' ms, cycle '+j.cy+' ms'+u+'</table>';}),400);
</script></body></html>)HTML";

void cmd() {
  String x = server.arg("x");
  if (x == "z") { zeroAll(); stopAtM = -1; }
  else if (x == "f") { mode = MANUAL; both(1, dutyFromPct(speedPct)); }
  else if (x == "c") { mode = IDLE; both(0, 0); }
  else if (x == "b") { mode = IDLE; both(2, 0); }
  else if (x == "m") { mode = FULL; both(1, DUTY_MAX); }
  else if (x[0] == 't') { zeroAll(); targetM = x.substring(1).toFloat(); stopAtM = -1; mode = TARGET; both(1, dutyFromPct(speedPct)); }
  else if (x[0] == 'k') {                              // raw truth table, both sides: k<IN1><IN2>, no INV swap
    mode = IDLE; int a = x[1] == '1' ? 255 : 0, b = x[2] == '1' ? 255 : 0;
    for (int s = 0; s < 2; s++) { ledcWrite(IN1[s], a); ledcWrite(IN2[s], b); }
  }
  else if (x == "up") { seqMode = false; zeroAll(); }
  else if (x == "us") { seqMode = true; zeroAll(); }
  else if (x[0] == 'g') { GAP_MS = constrain(x.substring(1).toInt(), 15, 60); zeroAll(); }
  else if (x[0] == 'v') { speedPct = constrain(x.substring(1).toInt(), 0, 100); if (mode == MANUAL || mode == TARGET) both(1, dutyFromPct(speedPct)); }
  server.send(200, "text/plain", "ok");
}

String f1(float v, int d) { return v < 0 || v > 1e8 ? String("-") : String(v, d); }
void status() {
  uint32_t a = snap(0), b = snap(1);
  float dA = (metres(a) + metres(b)) / 2, ms = (rpm[0] + rpm[1]) / 2 * PI * WHEEL_MM / 1000 / 60;
  String s = "{\"mode\":\"" + String(MN[mode]) + "\",\"boots\":" + boots + ",\"reason\":\"" + reason + "\",\"sp\":" + speedPct +
    ",\"cL\":" + a + ",\"cR\":" + b + ",\"dL\":\"" + String(metres(a), 3) + "\",\"dR\":\"" + String(metres(b), 3) +
    "\",\"dA\":\"" + String(dA, 3) + "\",\"stop\":" + String(stopAtM, 3) + ",\"rL\":\"" + String(rpm[0], 0) + "\",\"rR\":\"" + String(rpm[1], 0) +
    "\",\"ms\":\"" + String(ms, 2) + "\",\"cy\":" + cycleMs + ",\"gap\":" + GAP_MS + ",\"sch\":\"" + (seqMode ? "one-by-one" : "pairs") + "\",\"us\":[";
  for (int i = 0; i < 4; i++)
    s += String(i ? "," : "") + "{\"n\":\"" + UN[i] + "\",\"l\":\"" + f1(uLast[i], 1) + "\",\"mn\":\"" + f1(uMin[i], 1) + "\",\"mx\":\"" +
         f1(uMax[i] > 0 ? uMax[i] : -1, 1) + "\",\"sp\":" + uSpikes[i] + ",\"ms\":" + uMiss[i] + "}";
  server.send(200, "application/json", s + "]}");
}

void setup() {
  Serial.begin(115200); delay(200); Serial.println("INT-04 boot");
  pinMode(2, OUTPUT);
  for (int s = 0; s < 2; s++) { ledcAttach(IN1[s], 20000, 8); ledcAttach(IN2[s], 20000, 8); drive(s, 0, 0); }
  for (int i = 0; i < 2; i++) pinMode(ENC[i], INPUT);
  attachInterrupt(ENC[0], encL, CHANGE); attachInterrupt(ENC[1], encR, CHANGE);
  for (int i = 0; i < 4; i++) { pinMode(TRIG[i], OUTPUT); digitalWrite(TRIG[i], LOW); pinMode(ECHO[i], INPUT); }
  attachInterrupt(ECHO[0], u0, CHANGE); attachInterrupt(ECHO[1], u1, CHANGE);
  attachInterrupt(ECHO[2], u2, CHANGE); attachInterrupt(ECHO[3], u3, CHANGE);
  zeroAll();
  prefs.begin("int04", false); boots = prefs.getUInt("boots", 0) + 1; prefs.putUInt("boots", boots);
  reason = resetName(esp_reset_reason());
  WiFi.softAP("SolarCar-INT04", "solarcar1");
  server.on("/", []() { server.send_P(200, "text/html", PAGE); });
  server.on("/c", cmd); server.on("/s", status);
  server.on("/zb", []() { boots = 0; prefs.putUInt("boots", 0); server.send(200, "text/plain", "boot counter reset"); });
  server.begin();
  Serial.printf("boot %lu (%s), AP SolarCar-INT04 at %s\n", (unsigned long)boots, reason, WiFi.softAPIP().toString().c_str());
}

void loop() {
  server.handleClient();
  digitalWrite(2, (millis() / 500) % 2);

  if (millis() - tSlot >= GAP_MS) {                    // ultrasonic pair scheme
    tSlot = millis();
    if (lastMask) readSlot(lastMask);
    uint8_t n = seqMode ? 4 : 2; if (slot >= n) slot = 0;
    if (slot == 0) { cycleMs = millis() - cycleStart; cycleStart = millis(); }
    lastMask = seqMode ? SEQ[slot] : PAIR[slot]; fireSlot(lastMask); slot++;
  }

  if (millis() - tRpm >= 1000) {                       // wheel speed, 1 s window
    tRpm = millis();
    for (int i = 0; i < 2; i++) { uint32_t c = snap(i); rpm[i] = (c - prevC[i]) * 60.0 / SLOTS; prevC[i] = c; }
  }

  if (mode == TARGET && (metres(snap(0)) + metres(snap(1))) / 2 >= targetM) {   // drive-to-distance
    both(0, 0); stopAtM =                               // STOP = LOW LOW (brake HH drives on this build, 19/10)
(metres(snap(0)) + metres(snap(1))) / 2; mode = IDLE;
  }
}
