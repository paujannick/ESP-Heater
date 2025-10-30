#include "wifi_support.h"

#include <Arduino.h>
#include <ESPmDNS.h>
#include <WiFi.h>
#include <WiFiManager.h>

namespace {

constexpr uint32_t kConfigPortalTimeoutSeconds = 180;

} // namespace

bool autoConfigureWiFi(const char *hostname, const char *apName, const char *apPassword) {
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(hostname);

  WiFiManager wifiManager;
  wifiManager.setHostname(hostname);
  wifiManager.setConfigPortalTimeout(kConfigPortalTimeoutSeconds);
  wifiManager.setConnectRetries(5);
  wifiManager.setMinimumSignalQuality(15);

  if (!wifiManager.autoConnect(apName, apPassword)) {
    Serial.println("[WiFi] Failed to connect or timeout reached");
    return false;
  }

  Serial.printf("[WiFi] Connected to %s\n", WiFi.SSID().c_str());
  Serial.printf("[WiFi] IP: %s\n", WiFi.localIP().toString().c_str());

  if (!MDNS.begin(hostname)) {
    Serial.println("[mDNS] Failed to start responder");
  } else {
    MDNS.addService("http", "tcp", 80);
  }

  return true;
}

void resetWiFiSettingsAndReboot() {
  WiFiManager wifiManager;
  wifiManager.resetSettings();
  delay(200);
  Serial.println("[WiFi] Settings cleared, rebooting...");
  ESP.restart();
}
