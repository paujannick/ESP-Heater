#include "wifi_support.h"

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiManager.h>

bool autoConfigureWiFi(const char *hostname, const char *apName, const char *apPassword) {
  WiFi.mode(WIFI_STA);
  WiFiManager wm;
  wm.setClass("invert");
  if (hostname && hostname[0] != '\0') {
    wm.setHostname(hostname);
  }
  return wm.autoConnect(apName, apPassword);
}

void resetWiFiSettingsAndReboot() {
  WiFiManager wm;
  wm.resetSettings();
  delay(100);
  ESP.restart();
}
