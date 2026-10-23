# Dependencies

Running list of everything this project needs installed. Kept current as
libraries are added — if a sketch includes something, it is listed here.

Last updated: 2026-09-14

---

## Toolchain

| Item | Version / source | Notes |
|---|---|---|
| Arduino IDE | 2.x | |
| **esp32** board package | Espressif Systems, via Boards Manager | Board: **ESP32 Dev Module**. Note your core version — 2.x and 3.x differ in the LEDC PWM API, and sketches here guard for both. |
| **CP210x VCP driver** | Silicon Labs — Universal Windows Driver | Required. The SEEIT ESP32-DEV-38P uses a **CP2102**, not the CH340 found on many 30-pin boards, so a driver installed for an older board does not cover it. The current package is **INF-based with no .exe** — unzip and right-click `silabser.inf` → Install. |
| MATLAB / Simulink | university licence | `simulation/` |
| Python (Anaconda) | | `analysis/` |

### Bench setup gotchas — hard-won

- **Use a USB-A to USB-C cable**, into a USB-A port. USB-C devices must fit 5.1 kΩ
  pull-down resistors on CC1/CC2 to be detected by a USB-C host, and cheap dev
  boards frequently omit them. With a C-to-C cable the board can take power (red
  LED on) while never enumerating at all — no COM port, nothing in Device
  Manager, no Windows chime. A USB-A host supplies 5 V unconditionally and
  sidesteps the whole negotiation.
- **The board enumerated as COM11.** A `COM8` also appears in the port list and
  is **not** the board — it persists when the board is unplugged. Selecting it
  produces `A serial exception error occurred: Write timeout`, which looks like a
  board fault and isn't.
- **Diagnostic order** when no upload works: unplug and check whether the port
  disappears (if not, wrong port) → swap to an A-to-C data cable → check Device
  Manager for *any* new entry on plug-in → install the CP210x driver → only then
  suspect the board.
- **Upload Speed 115200** if the default 921600 times out.

---

## Arduino libraries — in use

### WebSockets — Markus Sattler

- **Header:** `WebSocketsServer.h`
- **Install:** Library Manager → search `WebSockets` → pick the one authored by **Markus Sattler**
- **Source:** `github.com/Links2004/arduinoWebSockets`
- **Used by:** spike 01a, spike 01b
- **Why:** persistent full-duplex link between the ESP32 and the dashboard.
  Plain HTTP cannot push telemetry at 10–20 Hz.

> **Watch out:** several similarly-named libraries exist with incompatible APIs —
> `ArduinoWebsockets` (Gil Maimon), `WebSockets2_Generic`, `WebSocketClient`.
> The author name is the way to tell them apart. Only Sattler's provides
> `WebSocketsServer.h`.

### Bundled with the esp32 board package — no install needed

| Header | Purpose | Used by |
|---|---|---|
| `WiFi.h` | softAP / station mode | 01a, 01b |
| `WebServer.h` | serves the dashboard page over HTTP :80 | 01a, 01b |

---

## Arduino libraries — expected, not yet needed

Listed so they can be planned for. **Do not install until the spike that needs
them.** None of these are used by any current sketch.

| Library | Author | For | Spike |
|---|---|---|---|
| Adafruit INA219 | Adafruit | battery voltage + current over I²C | 03 |
| Adafruit BusIO | Adafruit | dependency of the above, auto-installed | 03 |
| ESP32Servo | Kevin Harrington / John K. Bennett | servo control on ESP32 (the stock `Servo.h` is AVR-only) | 05 |
| ArduinoJson | Benoît Blanchon | structured command/telemetry messages in the real firmware — spikes use compact text instead | firmware |

The Arduino Nano tracker (spike 05) uses the standard AVR `Servo.h`, which ships
with the IDE. Nothing to install there.

---

## Python — `analysis/`

Nothing pinned yet. Expected: `pandas`, `numpy`, `matplotlib`, `pyserial`.
A `requirements.txt` goes here once the first analysis script exists.

---

## Conventions

- **A library earns its place.** Prefer what ships with the board package.
  Every dependency is a version-drift risk and something a future reader has to
  install to reproduce the build.
- **Pin nothing you haven't used.** Anticipated libraries stay in the section
  above until a sketch actually includes them.
- **Record the author**, not just the library name. Arduino's Library Manager has
  multiple libraries sharing a name, and picking the wrong one produces
  confusing compile errors rather than a clear "not found".
