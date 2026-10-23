// INT-03 — Drive: both DRV8871s + four motors on Buck B, ESP32 on Buck A (no USB), car lifted.
// Pins = KiCad ESP32 schematic: MOT_L_IN1/IN2 GPIO25/26, MOT_R_IN1/IN2 GPIO27/14. INA219 battery = 0x41.
// Phone: join "SolarCar-INT03" (password solarcar1), open http://192.168.4.1
//   L / R / Both = which side the buttons drive | Fwd / Rev / Coast / Brake | speed slider (0-100 %)
//   "20 hard starts" = full duty 1.5 s, coast 1 s, x20, both sides | STOP = coast both
//   Status (2 Hz): boot count + last reset reason (a reset during the test shows here), pack V / current.
// Duty cap: 100 % on the slider = DUTY_MAX, so the motors average ~MOTOR_V with VM ~VM_EST.
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <Wire.h>
#include <Adafruit_INA219.h>

const int IN1[2] = {25, 27}, IN2[2] = {26, 14};      // [0] left, [1] right
const bool INV[2] = {true, true};                     // forward for this build's wiring (U-10, 13/10)
const float MOTOR_V = 7.0, VM_EST = 7.5;              // target motor voltage / measured VM at the drivers
const int DUTY_MAX = (int)(255 * MOTOR_V / VM_EST);   // ~238 = 93 %
const int DUTY_MIN = 51;                              // ~20 %: below this the loaded car does not move (U-10)

WebServer server(80);
Preferences prefs;
Adafruit_INA219 ina(0x41);
bool inaOk; uint32_t boots; const char* reason;
int sel = 2, speedPct = 100, state[2] = {0, 0};       // state: 1 fwd, -1 rev, 0 coast, 2 brake
int startsLeft = 0; bool startOn = false; uint32_t tStart = 0;

void setOut(int s, int a, int b) { if (INV[s]) { int t = a; a = b; b = t; } ledcWrite(IN1[s], a); ledcWrite(IN2[s], b); }
void drive(int s, int dir, int duty) {                // slow decay: one input HIGH, the other PWMs
  if (dir == 2) { ledcWrite(IN1[s], 255); ledcWrite(IN2[s], 255); return; }
  if (dir == 0 || duty == 0) { ledcWrite(IN1[s], 0); ledcWrite(IN2[s], 0); return; }
  if (dir > 0) setOut(s, 255, 255 - duty); else setOut(s, 255 - duty, 255);
}
int dutyFromPct(int p) { return p <= 0 ? 0 : DUTY_MIN + (DUTY_MAX - DUTY_MIN) * p / 100; }
void apply() { for (int s = 0; s < 2; s++) drive(s, state[s], dutyFromPct(speedPct)); }

const char* resetName(esp_reset_reason_t r) {
  switch (r) { case ESP_RST_POWERON: return "power-on"; case ESP_RST_EXT: return "EN button";
    case ESP_RST_SW: return "software"; case ESP_RST_PANIC: return "crash";
    case ESP_RST_BROWNOUT: return "BROWNOUT"; case ESP_RST_INT_WDT: case ESP_RST_TASK_WDT: case ESP_RST_WDT: return "watchdog";
    default: return "other"; }
}

const char PAGE[] PROGMEM = R"HTML(<html><head><meta name='viewport' content='width=device-width'><style>
body{font-family:sans-serif;margin:12px}button{font-size:20px;padding:12px;margin:4px;min-width:90px}
.on{background:#7c7}#st{font-size:18px;margin-top:10px}</style></head><body>
<div><button id=s0 onclick="c('L')">L</button><button id=s1 onclick="c('R')">R</button><button id=s2 onclick="c('A')">Both</button></div>
<div><button onclick="c('f')">Fwd</button><button onclick="c('r')">Rev</button><button onclick="c('c')">Coast</button><button onclick="c('b')">Brake</button></div>
<div>Speed <input type=range min=0 max=100 value=100 style='width:60%' onchange="c('v'+this.value)"> <span id=sp>100</span>%</div>
<div><button onclick="c('g')">20 hard starts</button><button style='background:#e66' onclick="c('x')">STOP</button></div>
<div id=st></div><script>
function c(x){fetch('/c?x='+x);}
setInterval(()=>fetch('/s').then(r=>r.json()).then(j=>{
 document.getElementById('st').innerHTML='Boot count: <b>'+j.boots+'</b> | last reset: <b>'+j.reason+'</b><br>Uptime '+j.up+' s<br>L '+j.L+' | R '+j.R+' | speed '+j.sp+'% (duty '+j.duty+'/255)<br>Hard starts left: '+j.left+'<br>Pack '+j.v+' V, '+j.ma+' mA';
 document.getElementById('sp').textContent=j.sp; for(let i=0;i<3;i++)document.getElementById('s'+i).className=(i==j.sel)?'on':'';}),500);
</script></body></html>)HTML";

const char* stName(int s) { return s == 1 ? "fwd" : s == -1 ? "rev" : s == 2 ? "brake" : "coast"; }

void cmd() {
  String x = server.arg("x");
  if (x == "L") sel = 0; else if (x == "R") sel = 1; else if (x == "A") sel = 2;
  else if (x == "x") { startsLeft = 0; state[0] = state[1] = 0; }
  else if (x == "g") { startsLeft = 20; startOn = false; tStart = 0; state[0] = state[1] = 0; }
  else if (x[0] == 'v') speedPct = constrain(x.substring(1).toInt(), 0, 100);
  else { int d = x == "f" ? 1 : x == "r" ? -1 : x == "b" ? 2 : 0;
         for (int s = 0; s < 2; s++) if (sel == 2 || sel == s) state[s] = d; }
  if (!startsLeft) apply();
  server.send(200, "text/plain", "ok");
}

void status() {
  char b[300];
  snprintf(b, sizeof b, "{\"boots\":%lu,\"reason\":\"%s\",\"up\":%lu,\"L\":\"%s\",\"R\":\"%s\",\"sp\":%d,\"duty\":%d,\"sel\":%d,\"left\":%d,\"v\":\"%.3f\",\"ma\":\"%.0f\"}",
    (unsigned long)boots, reason, millis() / 1000, stName(state[0]), stName(state[1]), speedPct, dutyFromPct(speedPct), sel, startsLeft,
    inaOk ? ina.getBusVoltage_V() + ina.getShuntVoltage_mV() / 1000 : 0.0, inaOk ? ina.getCurrent_mA() : 0.0);
  server.send(200, "application/json", b);
}

void setup() {
  Serial.begin(115200); delay(200);
  Serial.println("INT-03 boot");
  pinMode(2, OUTPUT);
  for (int i = 0; i < 3; i++) { digitalWrite(2, HIGH); delay(80); digitalWrite(2, LOW); delay(80); }
  for (int s = 0; s < 2; s++) { ledcAttach(IN1[s], 20000, 8); ledcAttach(IN2[s], 20000, 8); drive(s, 0, 0); }
  Serial.println("1 PWM ok");
  prefs.begin("int03", false);
  boots = prefs.getUInt("boots", 0) + 1; prefs.putUInt("boots", boots);
  reason = resetName(esp_reset_reason());
  Serial.println("2 flash ok");
  Wire.begin(21, 22); inaOk = ina.begin();
  Serial.println(inaOk ? "3 INA219 ok" : "3 INA219 MISSING");
  Serial.println(WiFi.softAP("SolarCar-INT03", "solarcar1") ? "4 Wi-Fi AP ok" : "4 Wi-Fi AP FAILED");
  server.on("/", []() { server.send_P(200, "text/html", PAGE); });
  server.on("/c", cmd); server.on("/s", status);
  server.on("/z", []() { boots = 0; prefs.putUInt("boots", 0); server.send(200, "text/plain", "boot counter reset"); });
  server.begin();
  Serial.printf("boot %lu, reset: %s, INA219 0x41 %s, AP SolarCar-INT03 up at %s\n",
                (unsigned long)boots, reason, inaOk ? "OK" : "missing", WiFi.softAPIP().toString().c_str());
}

void loop() {
  server.handleClient();
  digitalWrite(2, (millis() / 500) % 2);            // 1 Hz heartbeat = running
  if (startsLeft > 0 && millis() - tStart >= (startOn ? 1500UL : 1000UL)) {   // hard-start cycle
    tStart = millis(); startOn = !startOn;
    for (int s = 0; s < 2; s++) drive(s, startOn ? 1 : 0, DUTY_MAX);
    if (!startOn) startsLeft--;
  }
}
