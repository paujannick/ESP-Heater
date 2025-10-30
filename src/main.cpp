#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ElegantOTA.h>

constexpr char WIFI_SSID[] = "YOUR_WIFI_SSID";
constexpr char WIFI_PASSWORD[] = "YOUR_WIFI_PASSWORD";
constexpr char OTA_HOSTNAME[] = "esp32-async-ota";

AsyncWebServer server(80);

void connectToWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(OTA_HOSTNAME);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.printf("Connecting to %s", WIFI_SSID);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print('.');
  }

  Serial.println();
  Serial.print("Connected! IP address: ");
  Serial.println(WiFi.localIP());
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("Booting...");

  connectToWiFi();

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "ESP32 ElegantOTA AsyncWebServer example");
  });

  ElegantOTA.begin(&server);
  ElegantOTA.setID(OTA_HOSTNAME);
  server.begin();

  Serial.println("HTTP server started");
  Serial.printf("OTA Update URL: http://%s/update\n", WiFi.localIP().toString().c_str());
}

void loop() {
  ElegantOTA.loop();
}
