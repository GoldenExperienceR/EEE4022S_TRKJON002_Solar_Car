/*
 * SPIKE 01a — Data path, both directions, with nothing but the ESP32
 * ------------------------------------------------------------------
 * Proves:  sensor -> ESP32 -> dashboard   (capacitive touch on GPIO4)
 *          dashboard -> ESP32 -> actuator (onboard LED on GPIO2)
 *
 * No external components. A short wire in GPIO4 makes a better touch pad,
 * but a fingertip on the pin header works.
 *
 * Board: ESP32 DevKit V1 / WROOM-32
 *
 * THROWAWAY CODE — do not promote into firmware/.
 *
 * Library: "WebSockets" by Markus Sattler (Library Manager)
 */

#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>

// ----------------------------- pins ----------------------------------
#define TOUCH_PIN   T0      // T0 == GPIO4 on the classic ESP32
#define LED_PIN     2       // onboard blue LED on DevKit V1

// ----------------------------- config --------------------------------
const char *AP_SSID = "SolarCar-POC";
const char *AP_PASS = "solarcar2026";

#define TX_PERIOD   50      // 20 Hz — fast enough to feel instant
#define TOUCH_RATIO 0.75f   // touched when reading < baseline * this
#define DEBOUNCE_N  2       // consecutive samples required to change state

// --------------------------- state -----------------------------------
WebServer        http(80);
WebSocketsServer ws(81);

uint32_t g_baseline = 0;    // calibrated at boot
uint32_t g_thresh   = 0;
uint16_t g_raw      = 0;
bool     g_touched  = false;
bool     g_ledOn    = false;
uint32_t g_touchCount = 0;
uint8_t  g_run      = 0;    // consecutive samples agreeing
bool     g_pending  = false;
uint32_t g_lastTx   = 0;

// ------------------------------ page ---------------------------------
static const char PAGE[] PROGMEM = R"HTMLPAGE(
<!doctype html>
<html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Spike 01a — Touch & LED</title>
<style>
  :root{--bg:#0d1117;--panel:#161b22;--line:#263041;--fg:#e6edf3;
        --dim:#8b949e;--accent:#f2b01e;--good:#3fb950;--bad:#f85149}
  *{box-sizing:border-box}
  body{margin:0;background:var(--bg);color:var(--fg);padding:16px;padding-block:20px;
       font:14px/1.5 ui-sans-serif,system-ui,-apple-system,Segoe UI,Roboto,sans-serif;
       -webkit-tap-highlight-color:transparent}
  h1{font-size:17px;margin:0 0 2px}
  .sub{color:var(--dim);font-size:12px;margin-bottom:18px}
  .pill{display:inline-block;padding:2px 9px;border-radius:20px;font-size:11px;
        font-weight:600;letter-spacing:.4px;text-transform:uppercase}
  .up{background:rgba(63,185,80,.15);color:var(--good)}
  .down{background:rgba(248,81,73,.15);color:var(--bad)}
  .card{background:var(--panel);border:1px solid var(--line);border-radius:10px;
        padding:16px;margin-bottom:12px}
  .lab{color:var(--dim);font-size:11px;text-transform:uppercase;letter-spacing:.6px;
       margin-bottom:10px}
  .row{display:flex;gap:16px;align-items:center;flex-wrap:wrap}

  /* ---- inbound: touch ---- */
  .orb{width:96px;height:96px;border-radius:50%;flex:0 0 auto;
       border:3px solid var(--line);background:#21262d;
       display:grid;place-items:center;font-size:12px;color:var(--dim);
       font-weight:600;letter-spacing:.5px;
       transition:background .12s,border-color .12s,box-shadow .12s,color .12s}
  .orb.hit{background:var(--good);border-color:var(--good);color:#0d1117;
           box-shadow:0 0 26px rgba(63,185,80,.55)}
  .nums{font-variant-numeric:tabular-nums;font-size:13px;color:var(--dim);line-height:1.9}
  .nums b{color:var(--fg);font-size:15px}
  canvas{width:100%;height:110px;display:block;margin-top:14px}

  /* ---- outbound: LED ---- */
  .btn{background:#21262d;color:var(--fg);border:1px solid var(--line);border-radius:9px;
       padding:14px 22px;font-size:15px;font-weight:600;cursor:pointer;
       transition:background .15s,border-color .15s;touch-action:manipulation}
  .btn:active{transform:scale(.97)}
  .bulb{width:34px;height:34px;border-radius:50%;background:#21262d;
        border:2px solid var(--line);flex:0 0 auto;transition:all .15s}
  .bulb.on{background:var(--accent);border-color:var(--accent);
           box-shadow:0 0 22px rgba(242,176,30,.6)}
  .note{color:var(--dim);font-size:12px;margin-top:10px}
</style></head><body>

<h1>Spike 01a — Touch in, LED out</h1>
<div class="sub">ESP32 only · no external parts ·
  <span id="st" class="pill down">connecting</span></div>

<div class="card">
  <div class="lab">Inbound — capacitive touch on GPIO4</div>
  <div class="row">
    <div class="orb" id="orb">IDLE</div>
    <div class="nums">
      raw <b id="raw">--</b><br>
      baseline <b id="base">--</b> · threshold <b id="thr">--</b><br>
      touches <b id="cnt">0</b>
    </div>
  </div>
  <canvas id="cv"></canvas>
  <div class="note">On the classic ESP32 the reading <b>drops</b> when touched.
    The yellow line is the threshold — if the trace crosses it without you
    touching anything, the threshold is too tight.</div>
</div>

<div class="card">
  <div class="lab">Outbound — onboard LED on GPIO2</div>
  <div class="row">
    <button class="btn" id="tg">Toggle LED</button>
    <div class="bulb" id="bulb"></div>
    <span class="nums">state <b id="ls">off</b></span>
  </div>
  <div class="note">The indicator follows what the <b>ESP32 reports back</b>,
    not what the button did. If it lights, the full round trip works.</div>
</div>

<script>
const $=id=>document.getElementById(id);
let sock, trace=[], base=0, thr=0;

function connect(){
  sock=new WebSocket('ws://'+location.hostname+':81/');
  sock.onopen =()=>{$('st').textContent='live';   $('st').className='pill up';};
  sock.onclose=()=>{$('st').textContent='offline';$('st').className='pill down';
                    setTimeout(connect,1200);};
  sock.onmessage=e=>paint(JSON.parse(e.data));
}

function paint(d){
  base=d.base; thr=d.thr;
  $('raw').textContent  = d.raw;
  $('base').textContent = d.base;
  $('thr').textContent  = d.thr;
  $('cnt').textContent  = d.cnt;
  $('orb').classList.toggle('hit', d.t);
  $('orb').textContent = d.t ? 'TOUCH' : 'IDLE';
  $('bulb').classList.toggle('on', d.led);
  $('ls').textContent = d.led ? 'on' : 'off';

  trace.push(d.raw);
  if(trace.length>160) trace.shift();
  draw();
}

function draw(){
  const c=$('cv'), g=c.getContext('2d'), r=devicePixelRatio||1;
  c.width=c.clientWidth*r; c.height=c.clientHeight*r; g.scale(r,r);
  const W=c.clientWidth, H=c.clientHeight;
  const hi=Math.max(base*1.15,10), lo=0;
  const y=v=>H-(v-lo)/(hi-lo)*H*0.94-3;
  g.clearRect(0,0,W,H);

  g.strokeStyle='#263041'; g.setLineDash([4,4]); g.lineWidth=1;
  g.beginPath(); g.moveTo(0,y(base)); g.lineTo(W,y(base)); g.stroke();
  g.strokeStyle='#f2b01e';
  g.beginPath(); g.moveTo(0,y(thr)); g.lineTo(W,y(thr)); g.stroke();
  g.setLineDash([]);

  if(trace.length>1){
    g.strokeStyle='#58a6ff'; g.lineWidth=2; g.beginPath();
    trace.forEach((v,k)=>{const x=k/159*W; k?g.lineTo(x,y(v)):g.moveTo(x,y(v));});
    g.stroke();
  }
}

const send=m=>{ if(sock&&sock.readyState===1) sock.send(m); };
$('tg').onclick=()=>send('L');       // ESP toggles and reports back
addEventListener('resize',draw);
connect();
</script>
</body></html>
)HTMLPAGE";

// --------------------------- websocket -------------------------------
void onWsEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t len) {
  if (type == WStype_CONNECTED)    { Serial.printf("[ws] client %u up\n", num);   return; }
  if (type == WStype_DISCONNECTED) { Serial.printf("[ws] client %u down\n", num); return; }
  if (type != WStype_TEXT || len < 1) return;

  if (payload[0] == 'L') {
    g_ledOn = !g_ledOn;
    digitalWrite(LED_PIN, g_ledOn ? HIGH : LOW);
    Serial.printf("[rx] LED -> %s\n", g_ledOn ? "ON" : "OFF");
  }
}

// ---------------------------- sampling -------------------------------
void sampleTouch() {
  g_raw = touchRead(TOUCH_PIN);
  bool now = (g_raw < g_thresh);

  // Debounce: require DEBOUNCE_N samples in a row before believing a change.
  if (now != g_touched) {
    if (!g_pending) { g_pending = true; g_run = 1; }
    else            { g_run++; }
    if (g_run >= DEBOUNCE_N) {
      g_touched = now;
      g_pending = false;
      if (g_touched) {
        g_touchCount++;
        Serial.printf("[tx] touch #%lu  raw=%u thr=%lu\n",
                      (unsigned long)g_touchCount, g_raw, (unsigned long)g_thresh);
      }
    }
  } else {
    g_pending = false;
  }
}

// ------------------------------ setup --------------------------------
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== SPIKE 01a: touch in, LED out ===");

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Calibrate the untouched baseline. DO NOT touch the pin during boot.
  Serial.print("calibrating touch (hands off)");
  uint32_t sum = 0;
  for (int k = 0; k < 32; k++) { sum += touchRead(TOUCH_PIN); delay(20); }
  g_baseline = sum / 32;
  g_thresh   = (uint32_t)(g_baseline * TOUCH_RATIO);
  Serial.printf("\nbaseline=%lu  threshold=%lu\n",
                (unsigned long)g_baseline, (unsigned long)g_thresh);

  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.printf("AP  : %s / %s\n", AP_SSID, AP_PASS);
  Serial.printf("open: http://%s\n", WiFi.softAPIP().toString().c_str());

  http.on("/", []() { http.send_P(200, "text/html", PAGE); });
  http.begin();
  ws.begin();
  ws.onEvent(onWsEvent);
  Serial.println("ready");
}

void loop() {
  http.handleClient();
  ws.loop();
  sampleTouch();

  uint32_t now = millis();
  if (now - g_lastTx >= TX_PERIOD) {
    g_lastTx = now;
    if (ws.connectedClients() > 0) {
      char buf[140];
      snprintf(buf, sizeof(buf),
               "{\"raw\":%u,\"base\":%lu,\"thr\":%lu,\"t\":%s,\"led\":%s,\"cnt\":%lu}",
               g_raw, (unsigned long)g_baseline, (unsigned long)g_thresh,
               g_touched ? "true" : "false", g_ledOn ? "true" : "false",
               (unsigned long)g_touchCount);
      ws.broadcastTXT(buf);
    }
  }
}
