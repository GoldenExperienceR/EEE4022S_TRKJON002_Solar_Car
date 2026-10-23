/*
 * SPIKE 01 — Live web dashboard over WebSocket
 * ------------------------------------------------------------
 * Proves: ESP32 can serve a dashboard, push live telemetry to it at 10 Hz
 *         with smooth animation, AND receive control input back from it.
 *
 * Telemetry here is SIMULATED. No sensors required — only an ESP32.
 * Throttle from the dashboard drives the onboard LED brightness, so you
 * can see dashboard -> hardware without a scope.
 *
 * THROWAWAY CODE. Hardcoded pins, blocking-ish loop, no error handling.
 * Do not promote into firmware/ — promote the lesson, not the code.
 *
 * Library required (Arduino Library Manager):
 *   "WebSockets" by Markus Sattler
 *
 * Board: any ESP32 dev board. Tools > Board > ESP32 Dev Module.
 */

#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>

// ----------------------------- config --------------------------------
#define AP_MODE 1                      // 1 = ESP makes its own network (no router needed)

const char *AP_SSID  = "SolarCar-POC";
const char *AP_PASS  = "solarcar2026";  // >= 8 chars

const char *STA_SSID = "YOUR_WIFI";     // only used if AP_MODE 0
const char *STA_PASS = "YOUR_PASS";

#define LED_PIN   2                     // onboard LED on most ESP32 devkits
#define LED_CH    0
#define TX_PERIOD 100                   // telemetry period, ms -> 10 Hz

// --------------------------- global state ----------------------------
WebServer        http(80);
WebSocketsServer ws(81);

int  g_throttle = 0;    // 0..100, set from dashboard
int  g_dir      = 1;    // 1 fwd, -1 rev
bool g_led      = false;

uint32_t g_lastTx = 0;
float    g_soc    = 88.0f;

// ------------------------------ page ---------------------------------
static const char PAGE[] PROGMEM = R"HTMLPAGE(
<!doctype html>
<html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Solar Car — Live</title>
<style>
  :root{
    --bg:#0d1117; --panel:#161b22; --line:#263041;
    --fg:#e6edf3; --dim:#8b949e;
    --accent:#f2b01e; --good:#3fb950; --cool:#58a6ff;
  }
  *{box-sizing:border-box}
  body{margin:0;background:var(--bg);color:var(--fg);
       font:14px/1.5 ui-sans-serif,system-ui,-apple-system,Segoe UI,Roboto,sans-serif;
       padding:16px;padding-block:20px}
  h1{font-size:17px;margin:0 0 2px;letter-spacing:.2px}
  .sub{color:var(--dim);font-size:12px;margin-bottom:18px}
  .pill{display:inline-block;padding:2px 9px;border-radius:20px;font-size:11px;
        font-weight:600;letter-spacing:.4px;text-transform:uppercase}
  .up{background:rgba(63,185,80,.15);color:var(--good)}
  .down{background:rgba(248,81,73,.15);color:#f85149}
  .grid{display:grid;gap:12px;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));
        margin-bottom:12px}
  .card{background:var(--panel);border:1px solid var(--line);border-radius:10px;padding:14px}
  .lab{color:var(--dim);font-size:11px;text-transform:uppercase;letter-spacing:.6px}
  .val{font-size:26px;font-weight:650;font-variant-numeric:tabular-nums;
       transition:color .3s}
  .unit{font-size:13px;color:var(--dim);font-weight:400;margin-left:3px}
  .bar{height:5px;background:#21262d;border-radius:3px;overflow:hidden;margin-top:10px}
  .fill{height:100%;width:0;border-radius:3px;background:var(--accent);
        transition:width .25s cubic-bezier(.4,0,.2,1)}
  .wide{grid-column:1/-1}
  canvas{width:100%;height:120px;display:block}
  .ctl{display:flex;flex-wrap:wrap;gap:14px;align-items:center;margin-top:10px}
  input[type=range]{flex:1;min-width:160px;accent-color:var(--accent)}
  button{background:#21262d;color:var(--fg);border:1px solid var(--line);
         border-radius:7px;padding:7px 14px;font-size:13px;cursor:pointer;
         transition:background .15s,border-color .15s}
  button:hover{background:#2d333b}
  button.on{background:var(--accent);border-color:var(--accent);color:#0d1117;font-weight:600}
  .needle{transition:transform .25s cubic-bezier(.4,0,.2,1);transform-origin:100px 100px}
  @media (prefers-color-scheme:light){}
</style></head><body>

<h1>Solar Car — Live Telemetry</h1>
<div class="sub">Spike 01 · WebSocket · simulated data ·
  <span id="st" class="pill down">connecting</span></div>

<div class="grid">
  <div class="card">
    <div class="lab">Speed</div>
    <div class="val"><span id="kph">0.0</span><span class="unit">km/h</span></div>
    <div class="bar"><div class="fill" id="kphb" style="background:var(--cool)"></div></div>
  </div>
  <div class="card">
    <div class="lab">Pack voltage</div>
    <div class="val"><span id="v">0.00</span><span class="unit">V</span></div>
    <div class="bar"><div class="fill" id="vb" style="background:var(--good)"></div></div>
  </div>
  <div class="card">
    <div class="lab">Current</div>
    <div class="val"><span id="i">0.00</span><span class="unit">A</span></div>
    <div class="bar"><div class="fill" id="ib"></div></div>
  </div>
  <div class="card">
    <div class="lab">PV power</div>
    <div class="val"><span id="w">0.00</span><span class="unit">W</span></div>
    <div class="bar"><div class="fill" id="wb"></div></div>
  </div>
</div>

<div class="grid">
  <div class="card">
    <div class="lab">State of charge</div>
    <svg viewBox="0 0 200 120" style="width:100%;max-width:100%;height:auto">
      <path d="M20 100 A80 80 0 0 1 180 100" fill="none" stroke="#21262d" stroke-width="12"
            stroke-linecap="round"/>
      <path id="arc" d="M20 100 A80 80 0 0 1 180 100" fill="none" stroke="var(--good)"
            stroke-width="12" stroke-linecap="round"
            stroke-dasharray="251.3" stroke-dashoffset="251.3"
            style="transition:stroke-dashoffset .4s ease-out"/>
      <line id="nd" class="needle" x1="100" y1="100" x2="100" y2="34"
            stroke="var(--fg)" stroke-width="2.5" stroke-linecap="round"/>
      <circle cx="100" cy="100" r="5" fill="var(--fg)"/>
      <text x="100" y="118" text-anchor="middle" fill="var(--dim)" font-size="13"
            font-family="sans-serif"><tspan id="soct">--</tspan>%</text>
    </svg>
  </div>
  <div class="card wide" style="min-width:0">
    <div class="lab">Power trace <span style="color:var(--accent)">— PV</span>
      <span style="color:var(--cool)">— load</span></div>
    <canvas id="cv"></canvas>
  </div>
</div>

<div class="card">
  <div class="lab">Control — dashboard to ESP32</div>
  <div class="ctl">
    <span style="width:74px">Throttle <b id="tv">0</b>%</span>
    <input id="th" type="range" min="0" max="100" value="0">
  </div>
  <div class="ctl">
    <button id="fw" class="on">Forward</button>
    <button id="rv">Reverse</button>
    <button id="ld">Headlight</button>
    <span style="color:var(--dim);font-size:12px">Throttle drives the onboard LED</span>
  </div>
</div>

<script>
const $=id=>document.getElementById(id);
let sock, pv=[], ld=[];

function connect(){
  sock = new WebSocket('ws://'+location.hostname+':81/');
  sock.onopen  = ()=>{ $('st').textContent='live';    $('st').className='pill up'; };
  sock.onclose = ()=>{ $('st').textContent='offline'; $('st').className='pill down';
                       setTimeout(connect,1200); };
  sock.onmessage = e=>paint(JSON.parse(e.data));
}

function paint(d){
  $('kph').textContent = d.kph.toFixed(1);
  $('v').textContent   = d.v.toFixed(2);
  $('i').textContent   = d.i.toFixed(2);
  $('w').textContent   = d.w.toFixed(2);
  $('kphb').style.width = Math.min(100, d.kph/8*100)+'%';
  $('vb').style.width   = Math.min(100,(d.v-3.0)/1.2*100)+'%';
  $('ib').style.width   = Math.min(100, d.i/2*100)+'%';
  $('wb').style.width   = Math.min(100, d.w/6*100)+'%';

  $('soct').textContent = d.soc.toFixed(0);
  $('arc').style.strokeDashoffset = (251.3*(1-d.soc/100)).toFixed(1);
  $('nd').style.transform = 'rotate('+(-90+d.soc*1.8)+'deg)';

  pv.push(d.w); ld.push(d.v*d.i);
  if(pv.length>140){pv.shift(); ld.shift();}
  draw();
}

function draw(){
  const c=$('cv'), g=c.getContext('2d'), r=devicePixelRatio||1;
  c.width=c.clientWidth*r; c.height=c.clientHeight*r; g.scale(r,r);
  const W=c.clientWidth, H=c.clientHeight, max=6;
  g.clearRect(0,0,W,H);
  g.strokeStyle='#21262d'; g.lineWidth=1;
  for(let k=1;k<4;k++){const y=H*k/4; g.beginPath(); g.moveTo(0,y); g.lineTo(W,y); g.stroke();}
  const line=(arr,col)=>{
    if(arr.length<2) return;
    g.strokeStyle=col; g.lineWidth=2; g.beginPath();
    arr.forEach((v,k)=>{
      const x=k/(140-1)*W, y=H-Math.min(1,v/max)*H*0.92-2;
      k?g.lineTo(x,y):g.moveTo(x,y);
    });
    g.stroke();
  };
  line(pv,'#f2b01e'); line(ld,'#58a6ff');
}

const send=m=>{ if(sock&&sock.readyState===1) sock.send(m); };
$('th').oninput = e=>{ $('tv').textContent=e.target.value; send('T'+e.target.value); };
$('fw').onclick = ()=>{ send('D1');  $('fw').classList.add('on'); $('rv').classList.remove('on'); };
$('rv').onclick = ()=>{ send('D-1'); $('rv').classList.add('on'); $('fw').classList.remove('on'); };
$('ld').onclick = ()=>{ const on=!$('ld').classList.contains('on');
                        $('ld').classList.toggle('on',on); send('L'+(on?1:0)); };
addEventListener('resize',draw);
connect();
</script>
</body></html>
)HTMLPAGE";

// ---------------------------- websocket ------------------------------
void onWsEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t len) {
  if (type == WStype_CONNECTED) {
    Serial.printf("[ws] client %u connected\n", num);
    return;
  }
  if (type == WStype_DISCONNECTED) {
    Serial.printf("[ws] client %u gone\n", num);
    return;
  }
  if (type != WStype_TEXT) return;

  // Compact commands, not JSON — this is a spike. Real firmware uses JSON.
  //   T<0..100>  throttle     D<1|-1>  direction     L<0|1>  headlight
  char c = (char)payload[0];
  int  v = atoi((const char *)payload + 1);

  if (c == 'T') {
    g_throttle = constrain(v, 0, 100);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(LED_PIN, map(g_throttle, 0, 100, 0, 255));
#else
    ledcWrite(LED_CH, map(g_throttle, 0, 100, 0, 255));
#endif
    Serial.printf("[rx] throttle = %d%%\n", g_throttle);
  } else if (c == 'D') {
    g_dir = (v < 0) ? -1 : 1;
    Serial.printf("[rx] direction = %s\n", g_dir > 0 ? "FWD" : "REV");
  } else if (c == 'L') {
    g_led = v;
    Serial.printf("[rx] headlight = %s\n", g_led ? "ON" : "OFF");
  }
}

// ---------------------------- telemetry ------------------------------
void pushTelemetry() {
  float t = millis() / 1000.0f;

  // Simulated plant. Enough motion to prove the animation path works.
  float kph = (g_throttle / 100.0f) * 7.5f * (0.94f + 0.06f * sinf(t * 3.1f));
  float amp = (g_throttle / 100.0f) * 1.6f + 0.05f;
  float pv  = 2.8f + 2.0f * sinf(t * 0.45f) + 0.25f * sinf(t * 5.0f);
  if (pv < 0) pv = 0;
  float vlt = 4.05f - 0.35f * amp - (100.0f - g_soc) * 0.008f;

  // Crude coulomb count so SOC visibly moves. Not a real SOC estimator.
  g_soc += ((pv / vlt) - amp) * (TX_PERIOD / 1000.0f) * 0.35f;
  g_soc = constrain(g_soc, 0.0f, 100.0f);

  char buf[160];
  snprintf(buf, sizeof(buf),
           "{\"kph\":%.2f,\"v\":%.3f,\"i\":%.3f,\"w\":%.2f,\"soc\":%.1f,\"dir\":%d}",
           kph, vlt, amp, pv, g_soc, g_dir);
  ws.broadcastTXT(buf);
}

// ------------------------------ setup --------------------------------
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== SPIKE 01: WebSocket dashboard ===");

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(LED_PIN, 5000, 8);
#else
  ledcSetup(LED_CH, 5000, 8);
  ledcAttachPin(LED_PIN, LED_CH);
#endif

#if AP_MODE
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.printf("AP  : %s / %s\n", AP_SSID, AP_PASS);
  Serial.printf("open: http://%s\n", WiFi.softAPIP().toString().c_str());
#else
  WiFi.begin(STA_SSID, STA_PASS);
  Serial.print("joining wifi");
  while (WiFi.status() != WL_CONNECTED) { Serial.print("."); delay(400); }
  Serial.printf("\nopen: http://%s\n", WiFi.localIP().toString().c_str());
#endif

  http.on("/", []() { http.send_P(200, "text/html", PAGE); });
  http.onNotFound([]() { http.send(404, "text/plain", "no"); });
  http.begin();

  ws.begin();
  ws.onEvent(onWsEvent);
  Serial.println("http :80   ws :81   ready");
}

void loop() {
  http.handleClient();
  ws.loop();

  uint32_t now = millis();
  if (now - g_lastTx >= TX_PERIOD) {
    g_lastTx = now;
    if (ws.connectedClients() > 0) pushTelemetry();
  }
}
