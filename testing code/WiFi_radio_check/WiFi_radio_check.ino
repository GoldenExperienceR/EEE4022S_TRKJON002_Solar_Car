// Wi-Fi radio check — ESP32 on USB ONLY, nothing else connected. Serial Monitor 115200.
// 1) scans for nearby networks (proves the radio can receive)
// 2) starts an open access point "ESP32-RADIO-TEST" on channel 1 (proves it can transmit)
#include <WiFi.h>

void setup() {
  Serial.begin(115200); delay(500);
  Serial.println("\n== Wi-Fi radio check ==");
  Serial.printf("Chip: %s rev %d, MAC %s\n", ESP.getChipModel(), ESP.getChipRevision(), WiFi.macAddress().c_str());

  WiFi.mode(WIFI_STA); delay(100);
  Serial.println("Scanning...");
  int n = WiFi.scanNetworks();
  Serial.printf("Found %d networks\n", n);
  for (int i = 0; i < n && i < 10; i++)
    Serial.printf("  %-32s ch %2d  %4d dBm\n", WiFi.SSID(i).c_str(), WiFi.channel(i), WiFi.RSSI(i));

  WiFi.mode(WIFI_AP); delay(100);
  bool ok = WiFi.softAP("ESP32-RADIO-TEST", nullptr, 1);     // open network, channel 1
  Serial.printf("softAP: %s, IP %s\n", ok ? "OK" : "FAILED", WiFi.softAPIP().toString().c_str());
}

void loop() {
  static uint32_t t = 0;
  if (millis() - t > 3000) { t = millis(); Serial.printf("running, %d client(s) connected\n", WiFi.softAPgetStationNum()); }
}
