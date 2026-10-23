/*
 * SPIKE 01b — Drive two motors and read distance, all from a phone
 * -----------------------------------------------------------------
 * Proves:  HC-SR04 -> ESP32 -> dashboard      (live distance)
 *          dashboard -> ESP32 -> L298N -> motors  (mode, speed, steering)
 *          ESP32-local safety loop             (distance guard, comms failsafe)
 *
 * If this works, ESP-NOW is not needed: a phone on the ESP32's own AP is
 * already a usable remote control.
 *
 * Board: ESP32 DevKit V1 / WROOM-32
 * Supply: bench PSU into the L298N motor terminals, current limit ON.
 *
 * !! HC-SR04 ECHO IS 5V. Divide it down: ECHO -1k- GPIO16 -2k- GND !!
 *
 * THROWAWAY CODE — do not promote into firmware/.
 *
 * Library: "WebSockets" by Markus Sattler (Library Manager)
 */

#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>

// ------------------------------ pins ---------------------------------
#define ENA   25    // motor A PWM
#define IN1   26
#define IN2   27
#define ENB   33    // motor B PWM
#define IN3   14
#define IN4   13
#define TRIG  17    // plain GPIO, no strapping / flash conflict
#define ECHO  16    // via 1k/2k divider from the HC-SR04's 5V ECHO

// ----------------------------- config --------------------------------
const char *AP_SSID = "SolarCar-POC";
const char *AP_PASS = "solarcar2026";

#define PWM_FREQ     1000   // Hz. L298N is a slow BJT bridge — don't go high.
#define PWM_BITS     8
#define PING_PERIOD  100    // ms, 10 Hz sonar
#define TX_PERIOD     50    // ms, 20 Hz telemetry
#define FAILSAFE_MS  600    // no command in this long -> stop
#define GUARD_CM      15    // auto-stop distance when guard is armed
#define ECHO_TIMEOUT 25000  // us, ~4.3 m

// ------------------------------ state --------------------------------
WebServer        http(80);
WebSocketsServer ws(81);

int   g_mode  = 0;      // 0 stop, 1 forward, 2 reverse
int   g_speed = 0;      // 0..100
int   g_steer = 0;      // -100 left .. +100 right
bool  g_guard = true;   // distance auto-stop armed
bool  g_blocked = false;

float g_cm     = -1;    // -1 = no echo
int   g_dutyL  = 0;     // signed, -100..100, after mixing
int   g_dutyR  = 0;

uint32_t g_lastRx = 0, g_lastPing = 0, g_lastTx = 0;

// ------------------------------ page ---------------------------------
static const char PAGE[] PROGMEM = R"HTMLPAGE(
<!doctype html>
<html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>Spike 01b — Drive</title>
<style>
  :root{--bg:#0d1117;--panel:#161b22;--line:#263041;--fg:#e6edf3;--dim:#8b949e;
        --accent:#f2b01e;--good:#3fb950;--bad:#f85149;--cool:#58a6ff}
  *{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
  html,body{height:100%;margin:0;overscroll-behavior:none}
  body{background:var(--bg);color:var(--fg);overflow:hidden;
       font:14px/1.4 ui-sans-serif,system-ui,-apple-system,Segoe UI,Roboto,sans-serif;
       user-select:none;-webkit-user-select:none;display:flex;flex-direction:column}

  header{display:flex;justify-content:space-between;align-items:center;
         padding:8px 16px;border-bottom:1px solid var(--line);flex:0 0 auto}
  .ttl{font-size:12px;color:var(--dim);letter-spacing:.5px;text-transform:uppercase}
  .pill{padding:2px 9px;border-radius:20px;font-size:10px;font-weight:700;
        letter-spacing:.5px;text-transform:uppercase}
  .up{background:rgba(63,185,80,.15);color:var(--good)}
  .down{background:rgba(248,81,73,.15);color:var(--bad)}

  .stage{flex:1;display:flex;gap:10px;padding:10px;min-height:0}
  .zone{display:flex;flex-direction:column;gap:10px;min-width:0}
  .left,.right{flex:0 0 27%;max-width:230px}
  .mid{flex:1;min-width:0}
  .card{background:var(--panel);border:1px solid var(--line);border-radius:12px;
        padding:12px;display:flex;flex-direction:column;min-height:0}
  .lab{color:var(--dim);font-size:10px;text-transform:uppercase;letter-spacing:.7px;
       margin-bottom:8px;flex:0 0 auto}

  /* ---------- left: mode + dial ---------- */
  .modes{display:flex;flex-direction:column;gap:7px}
  .mb{padding:12px 8px;border-radius:9px;border:1px solid var(--line);
      background:#21262d;color:var(--fg);font-size:14px;font-weight:700;
      letter-spacing:.5px;touch-action:manipulation;transition:all .12s}
  .mb:active{transform:scale(.97)}
  .mb.on{color:#0d1117}
  #mF.on{background:var(--good);border-color:var(--good)}
  #mS.on{background:var(--bad);border-color:var(--bad);color:#fff}
  #mR.on{background:var(--cool);border-color:var(--cool)}

  .dialwrap{flex:1;display:grid;place-items:center;min-height:0;touch-action:none}
  .dialwrap svg{width:100%;max-width:130px;height:auto}
  #knob{transition:transform .05s linear;transform-origin:60px 60px}

  /* ---------- middle: sensor ---------- */
  .big{font-size:clamp(38px,11vw,78px);font-weight:700;line-height:1;
       font-variant-numeric:tabular-nums;text-align:center;transition:color .2s}
  .big.warn{color:var(--bad)}
  .unit{font-size:.32em;color:var(--dim);font-weight:400}
  .barwrap{height:8px;background:#21262d;border-radius:4px;overflow:hidden;margin:12px 0}
  #dbar{height:100%;width:0;background:var(--good);border-radius:4px;
        transition:width .2s cubic-bezier(.4,0,.2,1),background .2s}
  canvas{width:100%;flex:1;min-height:50px;display:block}
  .tel{display:flex;justify-content:space-around;font-size:11px;color:var(--dim);
       font-variant-numeric:tabular-nums;padding-top:8px;border-top:1px solid var(--line);
       margin-top:8px}
  .tel b{color:var(--fg);display:block;font-size:14px;text-align:center}
  .guard{display:flex;align-items:center;gap:8px;font-size:11px;color:var(--dim);
         margin-top:8px}
  .sw{width:40px;height:22px;border-radius:12px;background:#21262d;
      border:1px solid var(--line);position:relative;transition:background .15s;
      flex:0 0 auto;touch-action:manipulation}
  .sw::after{content:'';position:absolute;top:2px;left:2px;width:16px;height:16px;
             border-radius:50%;background:var(--dim);transition:all .15s}
  .sw.on{background:rgba(63,185,80,.3);border-color:var(--good)}
  .sw.on::after{left:20px;background:var(--good)}

  /* ---------- right: joystick ---------- */
  .stickwrap{flex:1;display:grid;place-items:center;min-height:0;touch-action:none}
  .stick{width:100%;aspect-ratio:1;max-width:150px;border-radius:50%;
         background:radial-gradient(circle,#1c222b 0%,#161b22 70%);
         border:2px solid var(--line);position:relative;touch-action:none}
  .stick::before,.stick::after{content:'';position:absolute;background:var(--line)}
  .stick::before{left:8%;right:8%;top:50%;height:1px}
  .stick::after{top:8%;bottom:8%;left:50%;width:1px}
  .thumb{position:absolute;width:38%;height:38%;border-radius:50%;
         background:var(--accent);left:31%;top:31%;
         box-shadow:0 3px 12px rgba(0,0,0,.5);transition:transform .12s ease-out}
  .thumb.live{transition:none}

  #rot{display:none;position:fixed;inset:0;background:var(--bg);z-index:9;
       place-items:center;text-align:center;padding:30px;color:var(--dim)}
  @media (orientation:portrait){#rot{display:grid}}
</style></head><body>

<div id="rot"><div><div style="font-size:40px;margin-bottom:12px">⟳</div>
  Rotate to landscape<br><span style="font-size:12px">controls need the width</span></div></div>

<header>
  <span class="ttl">Spike 01b · Drive &amp; Ranging</span>
  <span><span id="fs" class="pill down" style="margin-right:6px;display:none">FAILSAFE</span>
  <span id="st" class="pill down">connecting</span></span>
</header>

<div class="stage">

  <!-- LEFT: mode + speed dial -->
  <div class="zone left">
    <div class="card" style="flex:0 0 auto">
      <div class="lab">Drive mode</div>
      <div class="modes">
        <button class="mb" id="mF">FORWARD</button>
        <button class="mb on" id="mS">STOP</button>
        <button class="mb" id="mR">REVERSE</button>
      </div>
    </div>
    <div class="card" style="flex:1">
      <div class="lab">Speed — PWM duty</div>
      <div class="dialwrap" id="dw">
        <svg viewBox="0 0 120 120">
          <path d="M25 95 A49 49 0 1 1 95 95" fill="none" stroke="#21262d"
                stroke-width="9" stroke-linecap="round"/>
          <path id="darc" d="M25 95 A49 49 0 1 1 95 95" fill="none" stroke="var(--accent)"
                stroke-width="9" stroke-linecap="round"
                stroke-dasharray="231" stroke-dashoffset="231"/>
          <line id="knob" x1="60" y1="60" x2="60" y2="26" stroke="var(--fg)"
                stroke-width="3" stroke-linecap="round"/>
          <circle cx="60" cy="60" r="4.5" fill="var(--fg)"/>
          <text x="60" y="112" text-anchor="middle" fill="var(--fg)" font-size="17"
                font-weight="700" font-family="sans-serif"><tspan id="sv">0</tspan>%</text>
        </svg>
      </div>
    </div>
  </div>

  <!-- MIDDLE: sensor readings -->
  <div class="zone mid">
    <div class="card" style="flex:1">
      <div class="lab">Time of flight — HC-SR04</div>
      <div class="big" id="cm">--<span class="unit">cm</span></div>
      <div class="barwrap"><div id="dbar"></div></div>
      <canvas id="cv"></canvas>
      <div class="tel">
        <div>L duty<b id="dl">0</b></div>
        <div>R duty<b id="dr">0</b></div>
        <div>steer<b id="sr">0</b></div>
      </div>
      <div class="guard">
        <div class="sw on" id="gd"></div>
        <span>Auto-stop under <b>15 cm</b></span>
      </div>
    </div>
  </div>

  <!-- RIGHT: joystick -->
  <div class="zone right">
    <div class="card" style="flex:1">
      <div class="lab">Steering</div>
      <div class="stickwrap">
        <div class="stick" id="stk"><div class="thumb" id="thb"></div></div>
      </div>
      <div style="text-align:center;font-size:11px;color:var(--dim);margin-top:6px">
        left / right · springs back
      </div>
    </div>
  </div>

</div>

<script>
const $=id=>document.getElementById(id);
let sock, trace=[], mode=0, speed=0, steer=0, guard=true;

/* ---------------- transport ---------------- */
function connect(){
  sock=new WebSocket('ws://'+location.hostname+':81/');
  sock.onopen =()=>{$('st').textContent='live';   $('st').className='pill up'; push();};
  sock.onclose=()=>{$('st').textContent='offline';$('st').className='pill down';
                    setTimeout(connect,1000);};
  sock.onmessage=e=>paint(JSON.parse(e.data));
}
const send=m=>{ if(sock&&sock.readyState===1) sock.send(m); };
const push=()=>send('C'+mode+','+Math.round(speed)+','+Math.round(steer)+','+(guard?1:0));
setInterval(push,150);                    // also serves as the heartbeat

/* ---------------- inbound ---------------- */
function paint(d){
  const el=$('cm');
  if(d.cm<0){ el.innerHTML='---<span class="unit">no echo</span>'; }
  else      { el.innerHTML=d.cm.toFixed(1)+'<span class="unit">cm</span>'; }
  el.classList.toggle('warn', d.blk);

  const pct=d.cm<0?0:Math.min(100,d.cm/100*100);
  $('dbar').style.width=pct+'%';
  $('dbar').style.background = d.blk?'var(--bad)':(d.cm<30?'var(--accent)':'var(--good)');

  $('dl').textContent=d.l; $('dr').textContent=d.r; $('sr').textContent=d.st;
  $('fs').style.display = d.fs?'inline-block':'none';

  trace.push(d.cm<0?null:d.cm);
  if(trace.length>150) trace.shift();
  draw();
}

function draw(){
  const c=$('cv'), g=c.getContext('2d'), r=devicePixelRatio||1;
  if(!c.clientHeight) return;
  c.width=c.clientWidth*r; c.height=c.clientHeight*r; g.scale(r,r);
  const W=c.clientWidth,H=c.clientHeight,MAX=120;
  g.clearRect(0,0,W,H);
  g.strokeStyle='#21262d'; g.lineWidth=1;
  for(let k=1;k<3;k++){const y=H*k/3;g.beginPath();g.moveTo(0,y);g.lineTo(W,y);g.stroke();}
  g.strokeStyle='#f85149'; g.setLineDash([3,3]);
  const gy=H-(15/MAX)*H; g.beginPath(); g.moveTo(0,gy); g.lineTo(W,gy); g.stroke();
  g.setLineDash([]);
  g.strokeStyle='#58a6ff'; g.lineWidth=2; g.beginPath();
  let pen=false;
  trace.forEach((v,k)=>{
    if(v==null){pen=false;return;}
    const x=k/149*W, y=H-Math.min(1,v/MAX)*H;
    pen?g.lineTo(x,y):g.moveTo(x,y); pen=true;
  });
  g.stroke();
}

/* ---------------- mode buttons ---------------- */
const setMode=m=>{
  mode=m;
  $('mF').classList.toggle('on',m===1);
  $('mS').classList.toggle('on',m===0);
  $('mR').classList.toggle('on',m===2);
  push();
};
$('mF').onclick=()=>setMode(1);
$('mS').onclick=()=>setMode(0);
$('mR').onclick=()=>setMode(2);

/* ---------------- speed dial (vertical drag) ---------------- */
(function(){
  const dw=$('dw'); let active=false, y0=0, s0=0;
  const render=()=>{
    $('sv').textContent=Math.round(speed);
    $('darc').style.strokeDashoffset=(231*(1-speed/100)).toFixed(1);
    $('knob').style.transform='rotate('+(-135+speed*2.7)+'deg)';
  };
  const down=e=>{active=true;y0=(e.touches?e.touches[0]:e).clientY;s0=speed;
                 dw.setPointerCapture&&e.pointerId!=null&&dw.setPointerCapture(e.pointerId);};
  const move=e=>{
    if(!active)return; e.preventDefault();
    const y=(e.touches?e.touches[0]:e).clientY;
    speed=Math.max(0,Math.min(100,s0+(y0-y)*0.6));   // drag up = faster
    render(); push();
  };
  const up=()=>{active=false;};
  dw.addEventListener('pointerdown',down); dw.addEventListener('pointermove',move,{passive:false});
  dw.addEventListener('pointerup',up);     dw.addEventListener('pointercancel',up);
  dw.addEventListener('touchstart',down,{passive:true});
  dw.addEventListener('touchmove',move,{passive:false});
  dw.addEventListener('touchend',up);
  render();
})();

/* ---------------- joystick ---------------- */
(function(){
  const stk=$('stk'), thb=$('thb');
  let active=false;
  const place=(dx,dy)=>{ thb.style.transform='translate('+dx+'px,'+dy+'px)'; };
  const grab=e=>{
    active=true; thb.classList.add('live');
    stk.setPointerCapture&&e.pointerId!=null&&stk.setPointerCapture(e.pointerId); move(e);
  };
  const move=e=>{
    if(!active)return; e.preventDefault&&e.preventDefault();
    const p=e.touches?e.touches[0]:e, r=stk.getBoundingClientRect();
    const lim=r.width*0.31;
    let dx=p.clientX-(r.left+r.width/2), dy=p.clientY-(r.top+r.height/2);
    const d=Math.hypot(dx,dy);
    if(d>lim){dx*=lim/d; dy*=lim/d;}
    place(dx,dy);
    steer=Math.round(dx/lim*100);
    push();
  };
  const rel=()=>{ active=false; thb.classList.remove('live'); place(0,0); steer=0; push(); };
  stk.addEventListener('pointerdown',grab); stk.addEventListener('pointermove',move,{passive:false});
  stk.addEventListener('pointerup',rel);    stk.addEventListener('pointercancel',rel);
  stk.addEventListener('touchstart',grab,{passive:true});
  stk.addEventListener('touchmove',move,{passive:false});
  stk.addEventListener('touchend',rel);
})();

/* ---------------- guard toggle ---------------- */
$('gd').onclick=()=>{ guard=!guard; $('gd').classList.toggle('on',guard); push(); };

addEventListener('resize',draw);
connect();
</script>
</body></html>
)HTMLPAGE";

// --------------------------- motor output -----------------------------
// Signed duty in -100..100. Sign picks direction, magnitude sets PWM.
void driveMotor(int en, int inA, int inB, int duty) {
  bool fwd = duty >= 0;
  int  mag = abs(duty);
  if (mag < 8) mag = 0;                  // deadband: L298N won't turn below this anyway
  digitalWrite(inA, mag == 0 ? LOW : (fwd ? HIGH : LOW));
  digitalWrite(inB, mag == 0 ? LOW : (fwd ? LOW  : HIGH));
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(en, map(mag, 0, 100, 0, 255));
#else
  ledcWrite(en == ENA ? 0 : 1, map(mag, 0, 100, 0, 255));
#endif
}

// Arcade mixing: mode+speed give magnitude and sign, steering splits it.
void applyDrive() {
  bool failsafe = (millis() - g_lastRx) > FAILSAFE_MS;
  g_blocked = g_guard && g_cm >= 0 && g_cm < GUARD_CM && g_mode == 1;

  int base = 0;
  if (!failsafe && !g_blocked) {
    if      (g_mode == 1) base =  g_speed;
    else if (g_mode == 2) base = -g_speed;
  }

  int mag = abs(base);
  int l = base + (g_steer * mag) / 100;
  int r = base - (g_steer * mag) / 100;

  int peak = max(abs(l), abs(r));        // scale down rather than clip, keeps the ratio
  if (peak > 100) { l = l * 100 / peak; r = r * 100 / peak; }

  g_dutyL = l; g_dutyR = r;
  driveMotor(ENA, IN1, IN2, l);
  driveMotor(ENB, IN3, IN4, r);
}

// ---------------------------- sonar ----------------------------------
void ping() {
  digitalWrite(TRIG, LOW);  delayMicroseconds(4);
  digitalWrite(TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  uint32_t us = pulseIn(ECHO, HIGH, ECHO_TIMEOUT);
  g_cm = (us == 0) ? -1.0f : us / 58.0f;   // 343 m/s, out and back
}

// --------------------------- websocket -------------------------------
void onWsEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t len) {
  if (type == WStype_CONNECTED) {
    Serial.printf("[ws] client %u up\n", num);
    g_lastRx = millis();
    return;
  }
  if (type == WStype_DISCONNECTED) {
    Serial.printf("[ws] client %u down -> stopping\n", num);
    g_mode = 0;                            // lose the phone, lose the throttle
    return;
  }
  if (type != WStype_TEXT || len < 2 || payload[0] != 'C') return;

  // "C<mode>,<speed>,<steer>,<guard>"
  int m, s, st, gd;
  if (sscanf((const char *)payload + 1, "%d,%d,%d,%d", &m, &s, &st, &gd) == 4) {
    g_mode  = constrain(m, 0, 2);
    g_speed = constrain(s, 0, 100);
    g_steer = constrain(st, -100, 100);
    g_guard = gd;
    g_lastRx = millis();
  }
}

// ------------------------------ setup --------------------------------
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== SPIKE 01b: drive + ranging ===");

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(TRIG, OUTPUT); pinMode(ECHO, INPUT);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(ENA, PWM_FREQ, PWM_BITS);
  ledcAttach(ENB, PWM_FREQ, PWM_BITS);
#else
  ledcSetup(0, PWM_FREQ, PWM_BITS); ledcAttachPin(ENA, 0);
  ledcSetup(1, PWM_FREQ, PWM_BITS); ledcAttachPin(ENB, 1);
#endif

  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.printf("AP  : %s / %s\n", AP_SSID, AP_PASS);
  Serial.printf("open: http://%s   (landscape)\n", WiFi.softAPIP().toString().c_str());

  http.on("/", []() { http.send_P(200, "text/html", PAGE); });
  http.begin();
  ws.begin();
  ws.onEvent(onWsEvent);
  Serial.println("ready — motors stopped until a client connects");
}

void loop() {
  http.handleClient();
  ws.loop();

  uint32_t now = millis();

  if (now - g_lastPing >= PING_PERIOD) { g_lastPing = now; ping(); }

  applyDrive();                            // every pass, so failsafe is prompt

  if (now - g_lastTx >= TX_PERIOD) {
    g_lastTx = now;
    if (ws.connectedClients() > 0) {
      bool fs = (now - g_lastRx) > FAILSAFE_MS;
      char buf[170];
      snprintf(buf, sizeof(buf),
               "{\"cm\":%.1f,\"l\":%d,\"r\":%d,\"st\":%d,\"blk\":%s,\"fs\":%s}",
               g_cm, g_dutyL, g_dutyR, g_steer,
               g_blocked ? "true" : "false", fs ? "true" : "false");
      ws.broadcastTXT(buf);
    }
  }
}
