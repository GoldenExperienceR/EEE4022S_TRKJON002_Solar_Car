// Solar tracker firmware — Arduino Nano (runs standalone, no PC needed).
// Pin-out (KiCad U2): SERVO_PAN D9, SERVO_TILT D10, LDR_TL/TR/BL/BR A0..A3, MOVING (from ESP32) D2.
// LDRs: 5V -> LDR -> Ax -> R -> GND (higher reading = more light), all four R equal.
// Power: Nano VIN from Buck B (~8 V), servos from Buck A (5 V), common GND.
//
// Turn-based tracking (one axis per turn):
//   DECIDE: pan needed (|dH| > TOL)? tilt needed (|dV| > TOL)? Pan first; after a turn the
//           other axis gets priority if it needs it.
//   TURN  : step that axis STEP_DEG (3) toward the light until it is balanced (< TOL_STOP), it hits
//           a limit, or |error| has not improved by IMPROVE counts for NO_GAIN_MS (5 s).
//   HOLD  : servo stays powered, deck locked for HOLD_MS (1 s), then DECIDE again.
//   WATCH : nothing needed -> both servos released, angles saved; re-enters DECIDE when needed.
// MOVING HIGH (car driving) -> PARK: tilt then pan to FLAT, servos released.
// Start-up: last angles from EEPROM, servos attached AT those angles (no jump), 3 s wait.
// Status LED (D13): on = turn/hold, slow blink = parked, off = watching.
// Serial 115200: one line per event (turn start/end + reason) and status every 0.5 s.
#include <Servo.h>
#include <EEPROM.h>

// ---------- tune these ----------
const int PAN_MIN = 0,  PAN_MAX = 180;
const int TILT_MIN = 0, TILT_MAX = 180;
const int FLAT_PAN = 90, FLAT_TILT = 90;   // angles where the panel lies flat on the car
const int PAN_SIGN = 1, TILT_SIGN = -1;    // confirmed 14/10
const int TOL = 30;                        // turn needed when |difference| > this (ADC counts)
const int TOL_STOP = 10;                   // turn ends when |difference| < this (balanced)
const int IMPROVE = 10;                    // "significant improvement" in |difference|
const unsigned long NO_GAIN_MS = 5000;     // end the turn if no improvement for this long
const unsigned long HOLD_MS = 1000;        // lock after each turn
const int STEP_DEG = 3;                    // degrees per tracking step (bigger = less sticking)
const unsigned long STEP_MS = 100, START_WAIT_MS = 3000;   // time between steps (~30 deg/s)
const int PARK_MS_PER_DEG = 15;
// ---------------------------------

struct Axis {
  Servo s; uint8_t pin, ee; int pos = 90, mn, mx, sign; bool on = false;
  Axis(uint8_t p, uint8_t e, int lo, int hi, int sg) : pin(p), ee(e), mn(lo), mx(hi), sign(sg) {}
  void engage()  { if (!on) { s.write(pos); s.attach(pin); on = true; } }
  void release() { if (on) { s.detach(); on = false; } EEPROM.update(ee, pos); EEPROM.update(10 + ee, 0x5A); }
  void load(int def) { pos = (EEPROM.read(10 + ee) == 0x5A) ? EEPROM.read(ee) : def; pos = constrain(pos, mn, mx); }
  void slewTo(int t) { engage(); while (pos != t) { pos += t > pos ? 1 : -1; s.write(pos); delay(PARK_MS_PER_DEG); } }
  bool step(int d) {                       // STEP_DEG toward the light; false if at a limit
    int n = constrain(pos + sign * (d > 0 ? STEP_DEG : -STEP_DEG), mn, mx);
    if (n == pos) return false;
    engage(); pos = n; s.write(pos); return true;
  }
};
Axis pan (9,  0, PAN_MIN,  PAN_MAX,  PAN_SIGN);
Axis tilt(10, 1, TILT_MIN, TILT_MAX, TILT_SIGN);

enum Mode { TURN, HOLD, WATCH, PARKED } mode = WATCH;
const char* NAME[] = {"TURN", "HOLD", "WATCH", "PARKED"};
Axis* act = &pan;                          // axis of the current / last turn
int v[4], dv, dh, best;
unsigned long tGain, tHold;

void readLDR() {
  for (int i = 0; i < 4; i++) { long s = 0; for (int k = 0; k < 4; k++) s += analogRead(A0 + i); v[i] = s / 4; }
  dv = (v[0] + v[1]) - (v[2] + v[3]);      // top - bottom  -> tilt
  dh = (v[0] + v[2]) - (v[1] + v[3]);      // left - right  -> pan
}
int err(Axis* a) { return a == &pan ? dh : dv; }
const char* nm(Axis* a) { return a == &pan ? "PAN" : "TILT"; }

bool movingSignal() {                      // debounced 50 ms
  static bool stable = false, last = false; static unsigned long since = 0;
  bool now = digitalRead(2);
  if (now != last) { last = now; since = millis(); }
  if (millis() - since >= 50) stable = last;
  return stable;
}

// choose the next axis: the other one first if it needs it, else the same one, else none
Axis* nextAxis(bool afterTurn) {
  bool needP = abs(dh) > TOL, needT = abs(dv) > TOL;
  Axis* other = (act == &pan) ? &tilt : &pan;
  if (!afterTurn) return needP ? &pan : needT ? &tilt : nullptr;
  if (abs(err(other)) > TOL) return other;
  return (needP || needT) ? act : nullptr;
}

void startTurn(Axis* a) {
  Axis* other = (a == &pan) ? &tilt : &pan;
  other->release();                        // one servo powered at a time
  act = a; best = abs(err(a)); tGain = millis(); mode = TURN;
  Serial.print(">> "); Serial.print(nm(a)); Serial.print(" turn, error "); Serial.println(err(a));
}

void endTurn(const char* why) {
  Serial.print("<< "); Serial.print(nm(act)); Serial.print(" done ("); Serial.print(why);
  Serial.print("), error "); Serial.print(err(act)); Serial.print(", angle "); Serial.println(act->pos);
  tHold = millis(); mode = HOLD;
}

void setup() {
  Serial.begin(115200);
  pinMode(2, INPUT); pinMode(13, OUTPUT);  // tie D2 to GND when the ESP32 is not connected
  pan.load(FLAT_PAN); tilt.load(FLAT_TILT);
  delay(START_WAIT_MS);
  Serial.println("tracker_nano (turn-based) ready");
}

void loop() {
  static unsigned long tStep = 0, tPrint = 0;

  if (movingSignal()) {
    if (mode != PARKED) {                  // tilt first, then pan (one servo at a time)
      pan.release(); tilt.slewTo(FLAT_TILT); delay(300); tilt.release();
      pan.slewTo(FLAT_PAN); delay(300); pan.release();
      mode = PARKED; Serial.println("PARKED");
    }
  } else if (millis() - tStep >= STEP_MS) {
    tStep = millis();
    readLDR();
    switch (mode) {
      case PARKED: mode = WATCH;           // car stopped: fall through and check the light
      case WATCH: { Axis* a = nextAxis(false); if (a) startTurn(a); break; }
      case TURN: {
        int e = abs(err(act));
        if (e < TOL_STOP) { endTurn("balanced"); break; }
        if (e <= best - IMPROVE) { best = e; tGain = millis(); }
        if (millis() - tGain >= NO_GAIN_MS) { endTurn("no improvement 5 s"); break; }
        if (!act->step(err(act))) endTurn("limit");
        break;
      }
      case HOLD:
        if (millis() - tHold >= HOLD_MS) {
          Axis* a = nextAxis(true);
          if (a) startTurn(a);
          else { pan.release(); tilt.release(); mode = WATCH; Serial.println("balanced -> WATCH (servos off)"); }
        }
        break;
    }
  }

  digitalWrite(13, mode == TURN || mode == HOLD ? HIGH : mode == PARKED ? (millis() / 500) % 2 : LOW);

  if (millis() - tPrint >= 500) {
    tPrint = millis();
    Serial.print(NAME[mode]); if (mode == TURN || mode == HOLD) { Serial.print(' '); Serial.print(nm(act)); }
    Serial.print("  LDR "); for (int i = 0; i < 4; i++) { Serial.print(v[i]); Serial.print(' '); }
    Serial.print(" dH "); Serial.print(dh); Serial.print(" dV "); Serial.print(dv);
    Serial.print("  pan "); Serial.print(pan.pos); Serial.print(" tilt "); Serial.println(tilt.pos);
  }
}
