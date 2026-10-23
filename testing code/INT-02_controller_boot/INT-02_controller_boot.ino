// INT-02 — Controller on Buck A + battery INA219. ESP32, powered from Buck A (5V pin), NO USB.
// Results go to a phone: join Wi-Fi "SolarCar-INT02" (password solarcar1), open http://192.168.4.1
// The page refreshes every second and shows:
//   boot count (kept in flash, so it increases on every reset) + last reset reason
//   INA219 (0x41: battery position, using the A0-bridged board since the 0x40 board failed 18/10) pack voltage, shunt voltage, current -> compare pack voltage with the DMM
// On-board LED (GPIO2): 3 fast blinks = booted, then a 1 Hz heartbeat = running.
// http://192.168.4.1/z resets the boot counter to 0 (do this before the 10-reset test).
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <Wire.h>
#include <Adafruit_INA219.h>

Adafruit_INA219 ina(0x41);            // was 0x40; A0-bridged board now in the battery position
WebServer server(80);
Preferences prefs;
uint32_t boots;
bool inaOk;
const char* reason;

const char* resetName(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:  return "power-on";
    case ESP_RST_EXT:      return "EN button";
    case ESP_RST_SW:       return "software";
    case ESP_RST_PANIC:    return "crash (panic)";
    case ESP_RST_BROWNOUT: return "BROWNOUT (supply dipped)";
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:      return "watchdog";
    default:               return "other";
  }
}

void page() {
  float bus = inaOk ? ina.getBusVoltage_V() : NAN;
  float shunt = inaOk ? ina.getShuntVoltage_mV() : NAN;
  float mA = inaOk ? ina.getCurrent_mA() : NAN;
  String s = "<html><head><meta http-equiv='refresh' content='1'><meta name='viewport' content='width=device-width'>"
             "<style>body{font-family:sans-serif;font-size:20px;margin:16px}b{font-size:28px}</style></head><body>";
  s += "Boot count: <b>" + String(boots) + "</b><br>";
  s += "Last reset: <b>" + String(reason) + "</b><br>";
  s += "Uptime: " + String(millis() / 1000) + " s<br><hr>";
  if (inaOk) {
    s += "Pack (INA219 bus): <b>" + String(bus + shunt / 1000, 3) + " V</b><br>";
    s += "Shunt: " + String(shunt, 2) + " mV<br>";
    s += "Current: " + String(mA, 1) + " mA<br>";
  } else s += "<b>INA219 0x41 not found</b> - check SDA 21 / SCL 22 / power<br>";
  s += "<hr><a href='/z'>Reset boot counter</a></body></html>";
  server.send(200, "text/html", s);
}

void setup() {
  Serial.begin(115200);
  pinMode(2, OUTPUT);
  for (int i = 0; i < 3; i++) { digitalWrite(2, HIGH); delay(80); digitalWrite(2, LOW); delay(80); }

  prefs.begin("int02", false);
  boots = prefs.getUInt("boots", 0) + 1;
  prefs.putUInt("boots", boots);
  reason = resetName(esp_reset_reason());

  Wire.begin(21, 22);
  inaOk = ina.begin();

  WiFi.softAP("SolarCar-INT02", "solarcar1");
  server.on("/", page);
  server.on("/z", []() { boots = 0; prefs.putUInt("boots", 0);
                         server.sendHeader("Location", "/"); server.send(303); });
  server.begin();
  Serial.printf("boot %lu, reset: %s, INA219 %s\n", (unsigned long)boots, reason, inaOk ? "OK" : "missing");
}

void loop() {
  server.handleClient();
  digitalWrite(2, (millis() / 500) % 2);            // 1 Hz heartbeat
}
