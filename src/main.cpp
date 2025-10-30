#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <DNSServer.h>
#include <WiFiUdp.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncJson.h>
#include <AsyncElegantOTA.h>
#include <WiFiManager.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <RotaryEncoder.h>
#include <UniversalTelegramBot.h>
#include <WiFiClientSecure.h>
#include <pgmspace.h>
#include <memory>

#include "pins.h"
#include "config_manager.h"

static constexpr uint16_t SCREEN_WIDTH = 128;
static constexpr uint16_t SCREEN_HEIGHT = 64;

WiFiClientSecure secureClient;
std::unique_ptr<UniversalTelegramBot> telegramBot;
unsigned long lastTelegramPoll = 0;

OneWire oneWire(PIN_ONEWIRE);
DallasTemperature tempSensors(&oneWire);
DeviceAddress insideSensor, exhaustSensor;
bool insideSensorFound = false;
bool exhaustSensorFound = false;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
RotaryEncoder encoder(PIN_ENCODER_CLK, PIN_ENCODER_DT, RotaryEncoder::LatchMode::TWO03);

AsyncWebServer server(80);
ConfigManager configManager;

float insideTemp = NAN;
float exhaustTemp = NAN;
float heaterCurrent = NAN;
float targetTemp = 21.0f;

enum class HeaterPhase { Off, Preheat, Heating, Cooldown, Fault };
HeaterPhase currentPhase = HeaterPhase::Off;
String currentPhaseLabel = "OFF";

uint8_t currentLevel = 1;
unsigned long lastRelayPulse = 0;
unsigned long lastStepMillis = 0;
uint8_t stepsThisQuarter = 0;
unsigned long quarterWindowStart = 0;
unsigned long lastStabilizeCheck = 0;

bool heaterEnabled = false;

bool strokeFeedbackAvailable = false;
bool strokeLatched = false;
int strokeStableLevel = HIGH;
int strokeLastLevel = HIGH;
unsigned long strokeLastToggleCandidate = 0;
bool strokeSeenHigh = false;
bool strokeSeenLow = false;

static constexpr uint16_t STROKE_DEBOUNCE_MS = 40;

static constexpr unsigned long STARTUP_BOOST_DELAY_MS = 120000; // 2 Minuten
static constexpr uint8_t STARTUP_BOOST_PULSES = 10;
static constexpr unsigned long STARTUP_BOOST_INTERVAL_MS = 600;

unsigned long heaterOnSince = 0;
bool startupBoostArmed = false;
bool startupBoostActive = false;
uint8_t startupBoostRemaining = 0;
unsigned long lastStartupBoostPulse = 0;

bool heaterPowerState() {
  return strokeFeedbackAvailable ? strokeLatched : heaterEnabled;
}

void updateStrokeSense() {
  int rawLevel = digitalRead(PIN_STROKE_SENSE);

  if (rawLevel != strokeLastLevel) {
    strokeLastLevel = rawLevel;
    strokeLastToggleCandidate = millis();
  }

  if (millis() - strokeLastToggleCandidate >= STROKE_DEBOUNCE_MS && rawLevel != strokeStableLevel) {
    strokeStableLevel = rawLevel;
    strokeSeenHigh |= (strokeStableLevel == HIGH);
    strokeSeenLow |= (strokeStableLevel == LOW);
    strokeLatched = (strokeStableLevel == STROKE_SENSE_ACTIVE_LEVEL);
    if (!strokeFeedbackAvailable && strokeSeenHigh && strokeSeenLow) {
      strokeFeedbackAvailable = true;
      Serial.println("[SENSE] Stromstoß-Rückmeldung erkannt");
    }
    Serial.printf("[SENSE] Stromstoßkontakt: %s\n", strokeLatched ? "AN" : "AUS");
  }
}

struct RelayPulse {
  uint8_t pin;
  bool active;
  unsigned long started;
};

RelayPulse relayPulses[] = {
    {RELAY_STROKE_PIN, false, 0},
    {RELAY_ON_PIN, false, 0},
    {RELAY_OFF_PIN, false, 0},
    {RELAY_PLUS_PIN, false, 0},
    {RELAY_MINUS_PIN, false, 0},
};

void startRelayPulse(uint8_t pin) {
  for (auto &pulse : relayPulses) {
    if (pulse.pin == pin) {
      digitalWrite(pin, RELAY_ACTIVE_LEVEL);
      pulse.active = true;
      pulse.started = millis();
      lastRelayPulse = pulse.started;
      break;
    }
  }
}

void ensureStrokeState(bool desiredOn) {
  if (strokeFeedbackAvailable) {
    if (strokeLatched != desiredOn) {
      Serial.printf("[CTRL] Stromstoßrelais -> %s\n", desiredOn ? "AN" : "AUS");
      startRelayPulse(RELAY_STROKE_PIN);
    }
  } else if (desiredOn) {
    startRelayPulse(RELAY_STROKE_PIN);
  }
}

void updateRelayPulses() {
  const unsigned long now = millis();
  for (auto &pulse : relayPulses) {
    if (pulse.active && now - pulse.started >= RELAY_PULSE_MS) {
      digitalWrite(pulse.pin, !RELAY_ACTIVE_LEVEL);
      pulse.active = false;
    }
  }
}

bool isRelayActive(uint8_t pin) {
  for (const auto &pulse : relayPulses) {
    if (pulse.pin == pin) {
      return pulse.active;
    }
  }
  return false;
}

float readTemperature(const DeviceAddress &device) {
  if (!tempSensors.validAddress(device)) {
    return NAN;
  }
  return tempSensors.getTempC(device);
}

float readCurrent() {
  const int raw = analogRead(PIN_ACS712);
  const float voltage = (static_cast<float>(raw) / 4095.0f) * 3.3f;
  const auto &cal = configManager.config().calibration;
  return (voltage - cal.acsZero) / cal.acsSensitivity;
}

void updateTemperatures() {
  tempSensors.requestTemperatures();
  if (insideSensorFound) {
    insideTemp = readTemperature(insideSensor);
  }
  if (exhaustSensorFound) {
    exhaustTemp = readTemperature(exhaustSensor);
  }
}

void updateCurrent() {
  heaterCurrent = readCurrent();
}

HeaterPhase detectPhase() {
  if (heaterCurrent < 0.2f) {
    return HeaterPhase::Off;
  }
  bool exhaustValid = !isnan(exhaustTemp);
  if (heaterCurrent > 5.0f && (!exhaustValid || exhaustTemp < 40.0f)) {
    return HeaterPhase::Preheat;
  }
  if (heaterCurrent > 1.0f && (!exhaustValid || exhaustTemp >= 40.0f)) {
    return HeaterPhase::Heating;
  }
  if (heaterCurrent >= 0.3f && heaterCurrent < 1.0f && (!exhaustValid || exhaustTemp < 50.0f)) {
    return HeaterPhase::Cooldown;
  }
  return HeaterPhase::Fault;
}

void updatePhase() {
  HeaterPhase newPhase = detectPhase();
  if (newPhase != currentPhase) {
    currentPhase = newPhase;
    switch (currentPhase) {
    case HeaterPhase::Off:
      currentPhaseLabel = "OFF";
      break;
    case HeaterPhase::Preheat:
      currentPhaseLabel = "PREHEAT";
      break;
    case HeaterPhase::Heating:
      currentPhaseLabel = "HEAT";
      break;
    case HeaterPhase::Cooldown:
      currentPhaseLabel = "COOLDN";
      break;
    case HeaterPhase::Fault:
      currentPhaseLabel = "FAULT";
      break;
    }
  }
}

void applySafetyRules() {
  const auto &reg = configManager.config().regulation;
  if (!isnan(exhaustTemp)) {
    if (exhaustTemp > reg.exhaustOff) {
      Serial.println("[SAFE] Exhaust temp critical, turning heater off");
      startRelayPulse(RELAY_OFF_PIN);
      heaterEnabled = false;
      ensureStrokeState(false);
    } else if (exhaustTemp > reg.exhaustWarn) {
      Serial.println("[SAFE] Exhaust high, decreasing level");
      if (currentLevel > 1) {
        startRelayPulse(RELAY_MINUS_PIN);
        currentLevel--;
      }
    }
  }
}

void adjustLevel(int8_t delta) {
  int desired = constrain(static_cast<int>(currentLevel) + delta, 1, 10);
  if (desired > currentLevel) {
    Serial.println("[CTRL] Increase level");
    startRelayPulse(RELAY_PLUS_PIN);
  } else if (desired < currentLevel) {
    Serial.println("[CTRL] Decrease level");
    startRelayPulse(RELAY_MINUS_PIN);
  } else {
    return;
  }
  currentLevel = static_cast<uint8_t>(desired);
  lastStepMillis = millis();
  stepsThisQuarter++;
}

void regulate() {
  const auto &config = configManager.config();
  const auto &reg = config.regulation;
  const unsigned long now = millis();

  if (quarterWindowStart == 0 || now - quarterWindowStart > 15UL * 60UL * 1000UL) {
    quarterWindowStart = now;
    stepsThisQuarter = 0;
  }

  if (currentPhase != HeaterPhase::Heating) {
    return;
  }

  if (stepsThisQuarter >= reg.maxStepsPerQuarter) {
    return;
  }

  if (now - lastStepMillis < reg.minStepInterval * 1000UL) {
    return;
  }

  if (isnan(insideTemp)) {
    return;
  }

  if (insideTemp < config.targetTemp - reg.deltaDown) {
    adjustLevel(+1);
    return;
  }

  if (insideTemp > config.targetTemp + reg.deltaUpCritical) {
    adjustLevel(-1);
    return;
  }

  if (insideTemp >= config.targetTemp) {
    if (lastStabilizeCheck == 0) {
      lastStabilizeCheck = now;
    } else if (now - lastStabilizeCheck >= reg.stabilizeDuration * 1000UL) {
      adjustLevel(-1);
      lastStabilizeCheck = 0;
    }
  } else {
    lastStabilizeCheck = 0;
  }
}

void updateDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.printf("IN: %4.1fC", insideTemp);
  display.setCursor(64, 0);
  display.printf("EX: %4.1fC", exhaustTemp);

  display.setCursor(0, 16);
  display.printf("I: %4.1fA", heaterCurrent);
  display.setCursor(64, 16);
  display.printf("PH: %s", currentPhaseLabel.c_str());

  display.setCursor(0, 32);
  display.printf("Set: %4.1fC", targetTemp);
  display.setCursor(64, 32);
  display.printf("Lvl: %u", currentLevel);

  display.setCursor(0, 48);
  if (strokeFeedbackAvailable) {
    display.printf("Rel:%s", heaterPowerState() ? "AN" : "AUS");
  } else {
    display.print("Rel:--");
  }
  display.setCursor(64, 48);
  if (WiFi.isConnected()) {
    display.printf("RSSI:%3ddB", WiFi.RSSI());
  } else {
    display.print("WiFi:AP");
  }
  display.display();
}

void handleEncoder() {
  static int32_t lastPos = 0;
  static unsigned long pressedAt = 0;
  static bool initialised = false;
  encoder.tick();
  if (!initialised) {
    lastPos = encoder.getPosition();
    initialised = true;
  }
  int32_t newPos = encoder.getPosition();
  if (newPos != lastPos) {
    lastPos = newPos;
    targetTemp = newPos * 0.5f;
    targetTemp = constrain(targetTemp, 10.0f, 28.0f);
    encoder.setPosition(static_cast<int32_t>(targetTemp * 2.0f));
    lastPos = encoder.getPosition();
    configManager.config().targetTemp = targetTemp;
    configManager.save();
    Serial.printf("[ENC] New target temperature %.1f\n", targetTemp);
  }

  if (digitalRead(PIN_ENCODER_SW) == LOW) {
    if (pressedAt == 0) {
      pressedAt = millis();
    } else if (millis() - pressedAt > 1500) {
      Serial.println("[ENC] Long press detected, forcing WiFi portal");
      WiFiManager wm;
      wm.resetSettings();
      ESP.restart();
    }
  } else {
    pressedAt = 0;
  }
}

void pulseStroke() {
  startRelayPulse(RELAY_STROKE_PIN);
}

void setHeaterPower(bool on) {
  if (on == heaterEnabled && (!strokeFeedbackAvailable || heaterPowerState() == on)) {
    return;
  }
  heaterEnabled = on;
  if (on) {
    Serial.println("[CTRL] Heater ON");
    startRelayPulse(RELAY_ON_PIN);
    ensureStrokeState(true);
    if (strokeFeedbackAvailable) {
      heaterOnSince = 0;
    } else {
      heaterOnSince = millis();
    }
    startupBoostArmed = true;
    startupBoostActive = false;
    startupBoostRemaining = STARTUP_BOOST_PULSES;
    lastStartupBoostPulse = 0;
    lastStepMillis = millis();
    lastStabilizeCheck = 0;
    stepsThisQuarter = 0;
    quarterWindowStart = millis();
  } else {
    Serial.println("[CTRL] Heater OFF");
    startRelayPulse(RELAY_OFF_PIN);
    ensureStrokeState(false);
    startupBoostArmed = false;
    startupBoostActive = false;
    startupBoostRemaining = 0;
    lastStartupBoostPulse = 0;
    heaterOnSince = 0;
  }
}

void handleStartupBoost() {
  if (!startupBoostArmed || !heaterEnabled) {
    return;
  }

  const unsigned long now = millis();

  if (strokeFeedbackAvailable) {
    if (!heaterPowerState()) {
      heaterOnSince = 0;
      return;
    }
    if (heaterOnSince == 0) {
      heaterOnSince = now;
    }
  } else if (heaterOnSince == 0) {
    heaterOnSince = now;
  }

  if (!startupBoostActive) {
    if (now - heaterOnSince < STARTUP_BOOST_DELAY_MS) {
      return;
    }
    startupBoostActive = true;
    lastStartupBoostPulse = 0;
    Serial.println("[CTRL] Starte Initialisierungs-Boost (Stufe 10)");
  }

  if (startupBoostRemaining == 0) {
    startupBoostArmed = false;
    startupBoostActive = false;
    currentLevel = 10;
    lastStepMillis = now;
    stepsThisQuarter = 0;
    quarterWindowStart = now;
    lastStabilizeCheck = 0;
    Serial.println("[CTRL] Initialisierungs-Boost abgeschlossen (Stufe 10)");
    return;
  }

  if (lastStartupBoostPulse != 0 && now - lastStartupBoostPulse < STARTUP_BOOST_INTERVAL_MS) {
    return;
  }

  if (!isRelayActive(RELAY_PLUS_PIN)) {
    startRelayPulse(RELAY_PLUS_PIN);
    startupBoostRemaining--;
    lastStartupBoostPulse = now;
    Serial.printf("[CTRL] Boost-Puls PLUS (%u verbleibend)\n", startupBoostRemaining);
  }
}

void setupRelays() {
  for (auto &pulse : relayPulses) {
    pinMode(pulse.pin, OUTPUT);
    digitalWrite(pulse.pin, !RELAY_ACTIVE_LEVEL);
  }
}

void setupSensors() {
  pinMode(PIN_STROKE_SENSE, INPUT_PULLUP);
  strokeStableLevel = digitalRead(PIN_STROKE_SENSE);
  strokeLastLevel = strokeStableLevel;
  strokeLatched = (strokeStableLevel == STROKE_SENSE_ACTIVE_LEVEL);
  strokeSeenHigh = (strokeStableLevel == HIGH);
  strokeSeenLow = (strokeStableLevel == LOW);
  tempSensors.begin();
  const int deviceCount = tempSensors.getDeviceCount();
  Serial.printf("[TEMP] Found %d DS18B20 sensors\n", deviceCount);
  for (int i = 0; i < deviceCount; ++i) {
    DeviceAddress addr;
    if (tempSensors.getAddress(addr, i)) {
      if (!insideSensorFound) {
        memcpy(insideSensor, addr, sizeof(DeviceAddress));
        insideSensorFound = true;
      } else if (!exhaustSensorFound) {
        memcpy(exhaustSensor, addr, sizeof(DeviceAddress));
        exhaustSensorFound = true;
      }
    }
  }
  if (insideSensorFound) {
    tempSensors.setResolution(insideSensor, 12);
  }
  if (exhaustSensorFound) {
    tempSensors.setResolution(exhaustSensor, 12);
  }
}

void configureWiFi() {
  WiFi.mode(WIFI_STA);
  WiFiManager wm;
  wm.setClass("invert");
  wm.setHostname("Sunster-Heater");

  if (!wm.autoConnect("SunsterSetup", "sunster123")) {
    Serial.println("[WIFI] Failed to connect, restarting");
    delay(3000);
    ESP.restart();
  }

  Serial.printf("[WIFI] Connected to %s\n", WiFi.SSID().c_str());
}

void handleTelegramMessages() {
  auto &cfg = configManager.config().telegram;
  if (!cfg.enabled || !telegramBot) {
    return;
  }
  if (millis() - lastTelegramPoll < 2000) {
    return;
  }
  lastTelegramPoll = millis();

  int numMessages = telegramBot->getUpdates(telegramBot->last_message_received + 1);
  while (numMessages) {
    for (int i = 0; i < numMessages; ++i) {
      auto &msg = telegramBot->messages[i];
      if (msg.chat_id != cfg.chatId) {
        continue;
      }
      String text = msg.text;
      text.toLowerCase();
      if (text == "/status") {
        String payload = "Innen: " + String(insideTemp, 1) + "C\n";
        payload += "Abluft: " + String(exhaustTemp, 1) + "C\n";
        payload += "Strom: " + String(heaterCurrent, 1) + "A\n";
        payload += "Phase: " + currentPhaseLabel + "\n";
        telegramBot->sendMessage(msg.chat_id, payload, "Markdown");
      } else if (text == "/ein") {
        setHeaterPower(true);
        telegramBot->sendMessage(msg.chat_id, "Heizung eingeschaltet", "Markdown");
      } else if (text == "/aus") {
        setHeaterPower(false);
        telegramBot->sendMessage(msg.chat_id, "Heizung ausgeschaltet", "Markdown");
      } else if (text == "/plus") {
        adjustLevel(+1);
      } else if (text == "/minus") {
        adjustLevel(-1);
      } else if (text.startsWith("/set")) {
        float temp = text.substring(4).toFloat();
        if (temp >= 10.0f && temp <= 28.0f) {
          targetTemp = temp;
          configManager.config().targetTemp = targetTemp;
          configManager.save();
          telegramBot->sendMessage(msg.chat_id, "Solltemperatur aktualisiert", "Markdown");
        }
      }
    }
    numMessages = telegramBot->getUpdates(telegramBot->last_message_received + 1);
  }
}

String phaseToString(HeaterPhase phase) {
  switch (phase) {
  case HeaterPhase::Off:
    return "off";
  case HeaterPhase::Preheat:
    return "preheat";
  case HeaterPhase::Heating:
    return "heating";
  case HeaterPhase::Cooldown:
    return "cooldown";
  default:
    return "fault";
  }
}

void sendStatus(AsyncWebServerRequest *request) {
  StaticJsonDocument<512> doc;
  doc["inside_temp"] = insideTemp;
  doc["exhaust_temp"] = exhaustTemp;
  doc["current"] = heaterCurrent;
  doc["phase"] = phaseToString(currentPhase);
  doc["level"] = currentLevel;
  doc["target_temp"] = targetTemp;
  doc["heater_on"] = heaterPowerState();
  doc["heater_request"] = heaterEnabled;
  doc["stroke_feedback"] = strokeFeedbackAvailable;
  doc["wifi_rssi"] = WiFi.RSSI();

  String payload;
  serializeJson(doc, payload);
  request->send(200, "application/json", payload);
}

void handleControl(AsyncWebServerRequest *request, JsonVariant &json) {
  auto obj = json.as<JsonObject>();
  if (obj.containsKey("heater_on")) {
    setHeaterPower(obj["heater_on"].as<bool>());
  }
  if (obj.containsKey("command")) {
    String cmd = obj["command"].as<const char *>();
    if (cmd == "plus") {
      adjustLevel(+1);
    } else if (cmd == "minus") {
      adjustLevel(-1);
    } else if (cmd == "stroke") {
      pulseStroke();
    }
  }
  if (obj.containsKey("target_temp")) {
    targetTemp = obj["target_temp"].as<float>();
    targetTemp = constrain(targetTemp, 10.0f, 28.0f);
    configManager.config().targetTemp = targetTemp;
    configManager.save();
  }
  request->send(200, "application/json", "{}");
}

void handleConfigGet(AsyncWebServerRequest *request) {
  StaticJsonDocument<1024> doc;
  const auto &cfg = configManager.config();
  doc["target_temp"] = cfg.targetTemp;

  JsonObject reg = doc.createNestedObject("regulation");
  reg["delta_down"] = cfg.regulation.deltaDown;
  reg["delta_up"] = cfg.regulation.deltaUp;
  reg["delta_up_critical"] = cfg.regulation.deltaUpCritical;
  reg["min_step_interval"] = cfg.regulation.minStepInterval;
  reg["stabilize_duration"] = cfg.regulation.stabilizeDuration;
  reg["max_steps_per_quarter"] = cfg.regulation.maxStepsPerQuarter;
  reg["exhaust_warn"] = cfg.regulation.exhaustWarn;
  reg["exhaust_off"] = cfg.regulation.exhaustOff;

  JsonObject tel = doc.createNestedObject("telegram");
  tel["enabled"] = cfg.telegram.enabled;
  tel["token"] = cfg.telegram.token;
  tel["chat_id"] = cfg.telegram.chatId;

  JsonObject calib = doc.createNestedObject("calibration");
  calib["acs_zero"] = cfg.calibration.acsZero;
  calib["acs_sensitivity"] = cfg.calibration.acsSensitivity;

  String payload;
  serializeJson(doc, payload);
  request->send(200, "application/json", payload);
}

void handleConfigPost(AsyncWebServerRequest *request, JsonVariant &json) {
  auto obj = json.as<JsonObject>();
  auto &cfg = configManager.config();

  if (obj.containsKey("target_temp")) {
    cfg.targetTemp = constrain(obj["target_temp"].as<float>(), 10.0f, 28.0f);
    targetTemp = cfg.targetTemp;
  }
  if (obj.containsKey("regulation")) {
    auto reg = obj["regulation"].as<JsonObject>();
    cfg.regulation.deltaDown = reg["delta_down"].as<float>();
    cfg.regulation.deltaUp = reg["delta_up"].as<float>();
    cfg.regulation.deltaUpCritical = reg["delta_up_critical"].as<float>();
    cfg.regulation.minStepInterval = reg["min_step_interval"].as<uint32_t>();
    cfg.regulation.stabilizeDuration = reg["stabilize_duration"].as<uint32_t>();
    cfg.regulation.maxStepsPerQuarter = reg["max_steps_per_quarter"].as<uint8_t>();
    cfg.regulation.exhaustWarn = reg["exhaust_warn"].as<float>();
    cfg.regulation.exhaustOff = reg["exhaust_off"].as<float>();
  }
  if (obj.containsKey("telegram")) {
    auto tel = obj["telegram"].as<JsonObject>();
    cfg.telegram.enabled = tel["enabled"].as<bool>();
    cfg.telegram.token = tel["token"].as<const char *>();
    cfg.telegram.chatId = tel["chat_id"].as<const char *>();
  }
  if (obj.containsKey("calibration")) {
    auto cal = obj["calibration"].as<JsonObject>();
    cfg.calibration.acsZero = cal["acs_zero"].as<float>();
    cfg.calibration.acsSensitivity = cal["acs_sensitivity"].as<float>();
  }

  configManager.save();
  request->send(200, "application/json", "{}");
}

void handleWiFiReset(AsyncWebServerRequest *request) {
  File flag = LittleFS.open("/wifi_reset.flag", "w");
  if (flag) {
    flag.print("reset");
    flag.close();
  }
  request->send(200, "text/plain", "rebooting");
  delay(200);
  ESP.restart();
}

void setupWebServer() {
  server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest *request) { sendStatus(request); });

  AsyncCallbackJsonWebHandler *controlHandler = new AsyncCallbackJsonWebHandler(
      "/api/control", [](AsyncWebServerRequest *request, JsonVariant &json) { handleControl(request, json); });
  server.addHandler(controlHandler);

  AsyncCallbackJsonWebHandler *configPostHandler = new AsyncCallbackJsonWebHandler(
      "/api/config", [](AsyncWebServerRequest *request, JsonVariant &json) { handleConfigPost(request, json); });
  server.addHandler(configPostHandler);
  server.on("/api/config", HTTP_GET, [](AsyncWebServerRequest *request) { handleConfigGet(request); });
  server.on("/api/wifi/reset", HTTP_POST, [](AsyncWebServerRequest *request) { handleWiFiReset(request); });

  server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");

  AsyncElegantOTA.begin(&server);
  server.begin();
}

void initTelegram() {
  auto &cfg = configManager.config().telegram;
  if (!cfg.enabled || cfg.token.isEmpty() || cfg.chatId.isEmpty()) {
    Serial.println("[TG] Telegram disabled");
    return;
  }
  extern const char TELEGRAM_CERTIFICATE_ROOT[] PROGMEM;
  secureClient.setCACert(TELEGRAM_CERTIFICATE_ROOT);
  telegramBot = std::make_unique<UniversalTelegramBot>(cfg.token, secureClient);
}

void checkWifiResetFlag() {
  if (LittleFS.exists("/wifi_reset.flag")) {
    LittleFS.remove("/wifi_reset.flag");
    WiFiManager wm;
    wm.resetSettings();
    ESP.restart();
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("[BOOT] Sunster Heater Controller");

  pinMode(PIN_ENCODER_SW, INPUT_PULLUP);
  pinMode(PIN_ENCODER_CLK, INPUT_PULLUP);
  pinMode(PIN_ENCODER_DT, INPUT_PULLUP);

  setupRelays();

  if (!configManager.begin()) {
    Serial.println("[BOOT] Config failed, using defaults");
  }
  targetTemp = configManager.config().targetTemp;

  setupSensors();

  Wire.begin(PIN_OLED_SDA, PIN_OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDRESS)) {
    Serial.println("[OLED] Failed to start display");
  } else {
    display.clearDisplay();
    display.display();
  }

  configureWiFi();
  checkWifiResetFlag();

  setupWebServer();

  initTelegram();

  lastStepMillis = millis();
  quarterWindowStart = millis();
  encoder.setPosition(static_cast<int32_t>(targetTemp * 2.0f));
}

void loop() {
  handleEncoder();
  updateRelayPulses();
  updateStrokeSense();
  handleStartupBoost();

  static unsigned long lastSensorUpdate = 0;
  static unsigned long lastDisplayUpdate = 0;
  static unsigned long lastRegulate = 0;

  unsigned long now = millis();

  if (now - lastSensorUpdate >= 1000) {
    updateTemperatures();
    updateCurrent();
    updatePhase();
    applySafetyRules();
    lastSensorUpdate = now;
  }

  if (now - lastDisplayUpdate >= 500) {
    updateDisplay();
    lastDisplayUpdate = now;
  }

  if (now - lastRegulate >= 2000) {
    regulate();
    lastRegulate = now;
  }

  handleTelegramMessages();
}
