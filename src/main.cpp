#include <Arduino.h>
#include <ArduinoJson.h>
#include <AsyncTCP.h>
#include <DallasTemperature.h>
#include <ElegantOTA.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <OneWire.h>
#include <RotaryEncoder.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <cmath>
#include <cstring>
#include <driver/adc.h>

#include "config_manager.h"
#include "pins.h"
#include "wifi_support.h"

namespace {

constexpr char OTA_HOSTNAME[] = "sunster-heater";
constexpr char WIFI_AP_NAME[] = "SunsterSetup";
constexpr char WIFI_AP_PASSWORD[] = "sunster123";

constexpr float TARGET_MIN = 10.0f;
constexpr float TARGET_MAX = 28.0f;
constexpr float TARGET_STEP = 0.5f;

constexpr uint32_t SENSOR_INTERVAL_MS = 1000;
constexpr uint32_t DISPLAY_INTERVAL_MS = 500;
constexpr uint32_t REGULATION_INTERVAL_MS = 1000;
constexpr uint32_t TARGET_SAVE_DELAY_MS = 3000;
constexpr uint32_t WIFI_RESET_HOLD_MS = 5000;
constexpr uint32_t START_BOOST_DELAY_MS = 120000;
constexpr uint32_t START_BOOST_PULSE_INTERVAL_MS = 600;
constexpr uint8_t START_BOOST_PULSES = 10;
constexpr uint32_t BUTTON_DEBOUNCE_MS = 50;

constexpr float CURRENT_THRESHOLD_OFF = 0.2f;
constexpr float CURRENT_THRESHOLD_HEATING = 0.8f;
constexpr float CURRENT_THRESHOLD_PREHEAT = 5.0f;

constexpr float EXHAUST_THRESHOLD_HEATING = 40.0f;
constexpr float EXHAUST_THRESHOLD_COOLDOWN = 50.0f;

constexpr uint32_t FAILSTART_TIMEOUT_MS = 90000;
constexpr uint32_t STAGE_WINDOW_MS = 15UL * 60UL * 1000UL;

AsyncWebServer server(80);
OneWire oneWire(PIN_ONEWIRE);
DallasTemperature tempSensors(&oneWire);
RotaryEncoder encoder(PIN_ENCODER_DT, PIN_ENCODER_CLK, RotaryEncoder::LatchMode::TWO03);
Adafruit_SSD1306 display(128, 64, &Wire, -1);

ConfigManager configManager;

DeviceAddress insideSensor{};
DeviceAddress exhaustSensor{};
bool insideSensorPresent = false;
bool exhaustSensorPresent = false;
bool displayPresent = false;

enum class HeaterPhase { Off, Preheat, Heating, Cooldown, Fault };

struct TelemetryState {
  float insideTemp = NAN;
  float exhaustTemp = NAN;
  float current = NAN;
  bool heaterSupplyOn = false;
  bool heaterRequest = false;
  bool strokeFeedbackPresent = false;
  bool wifiConnected = false;
  int wifiRssi = 0;
  HeaterPhase phase = HeaterPhase::Off;
  uint8_t stage = 1;
  float targetTemp = TARGET_MIN;
  bool boostActive = false;
};

TelemetryState telemetry;

bool heaterRequestOn = false;
bool heaterSupplyState = false;
uint8_t heaterStage = 1;

unsigned long lastSensorPoll = 0;
unsigned long lastDisplayUpdate = 0;
unsigned long lastRegulationRun = 0;
unsigned long lastStageChangeMillis = 0;
unsigned long stageWindowStart = 0;
uint8_t stageChangesInWindow = 0;

unsigned long phaseStartMillis = 0;
HeaterPhase currentPhase = HeaterPhase::Off;

bool startBoostActive = false;
unsigned long startBoostStartMillis = 0;
unsigned long lastBoostPulseMillis = 0;
uint8_t boostPulsesSent = 0;

bool targetDirty = false;
unsigned long targetDirtyMillis = 0;

unsigned long buttonPressStart = 0;
bool buttonTriggeredReset = false;
bool buttonLastRawState = false;
bool buttonDebouncedState = false;
unsigned long buttonLastDebounceMillis = 0;

bool strokeFeedbackDetected = false;

long lastEncoderPosition = 0;
unsigned long criticalOvershootHoldUntil = 0;
bool sensorConfigDirty = false;

struct RelayPulseState {
  bool active = false;
  uint8_t pin = 0;
  unsigned long endMillis = 0;
};

RelayPulseState relayPulses[5];

float readCurrent();
void updateTelemetry();
void updateDisplay();
void processEncoder();
void processEncoderButton();
void evaluatePhase();
void applyRegulation();
void handleStartBoost();
void sendStatus(AsyncWebServerRequest *request);
void handleControlRequest(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total);
void setupWebServer();
void setHeaterRequest(bool on);
void issueStrokePulse();
void requestStageChange(int8_t delta, bool manualOverride = false);
const char *phaseName(HeaterPhase phase);
void processRelayPulses();
void syncEncoderToTarget();
bool addressesEqual(const uint8_t *lhs, const uint8_t *rhs);
bool applyStoredSensorAddress(const SensorAddressConfig &config, DeviceAddress &destination);
bool updateSensorAddressConfig(SensorAddressConfig &config, const DeviceAddress &address);

void IRAM_ATTR encoderISR() { encoder.tick(); }

bool isValidTemp(float value) { return !isnan(value) && value > -100.0f && value < 200.0f; }

bool addressesEqual(const uint8_t *lhs, const uint8_t *rhs) { return memcmp(lhs, rhs, 8) == 0; }

String formatSensorAddress(const uint8_t *address) {
  char buffer[17];
  for (size_t i = 0; i < 8; ++i) {
    snprintf(buffer + i * 2, 3, "%02X", address[i]);
  }
  return String(buffer);
}

bool applyStoredSensorAddress(const SensorAddressConfig &config, DeviceAddress &destination) {
  if (!config.valid) {
    return false;
  }

  DeviceAddress candidate;
  memcpy(candidate, config.address, sizeof(candidate));
  if (!tempSensors.validAddress(candidate) || !tempSensors.isConnected(candidate)) {
    return false;
  }

  memcpy(destination, candidate, sizeof(candidate));
  return true;
}

bool updateSensorAddressConfig(SensorAddressConfig &config, const DeviceAddress &address) {
  if (!config.valid || !addressesEqual(config.address, address)) {
    memcpy(config.address, address, sizeof(DeviceAddress));
    config.valid = true;
    return true;
  }
  return false;
}

void configurePins() {
  pinMode(RELAY_STROKE_PIN, OUTPUT);
  pinMode(RELAY_ON_PIN, OUTPUT);
  pinMode(RELAY_OFF_PIN, OUTPUT);
  pinMode(RELAY_PLUS_PIN, OUTPUT);
  pinMode(RELAY_MINUS_PIN, OUTPUT);

  digitalWrite(RELAY_STROKE_PIN, !RELAY_ACTIVE_LEVEL);
  digitalWrite(RELAY_ON_PIN, !RELAY_ACTIVE_LEVEL);
  digitalWrite(RELAY_OFF_PIN, !RELAY_ACTIVE_LEVEL);
  digitalWrite(RELAY_PLUS_PIN, !RELAY_ACTIVE_LEVEL);
  digitalWrite(RELAY_MINUS_PIN, !RELAY_ACTIVE_LEVEL);

  pinMode(PIN_STROKE_SENSE, INPUT_PULLUP);
  pinMode(PIN_ENCODER_SW, INPUT_PULLUP);
  pinMode(PIN_ENCODER_CLK, INPUT_PULLUP);
  pinMode(PIN_ENCODER_DT, INPUT_PULLUP);

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_ACS712, ADC_11db);
}

void pulseRelay(uint8_t pin, uint16_t duration = RELAY_PULSE_MS, bool blocking = false) {
  if (blocking) {
    digitalWrite(pin, RELAY_ACTIVE_LEVEL);
    delay(duration);
    digitalWrite(pin, !RELAY_ACTIVE_LEVEL);
    return;
  }

  processRelayPulses();

  for (auto &pulse : relayPulses) {
    if (pulse.active && pulse.pin == pin) {
      digitalWrite(pin, !RELAY_ACTIVE_LEVEL);
      pulse.active = false;
    }
  }

  for (auto &pulse : relayPulses) {
    if (!pulse.active) {
      digitalWrite(pin, RELAY_ACTIVE_LEVEL);
      pulse.active = true;
      pulse.pin = pin;
      pulse.endMillis = millis() + duration;
      return;
    }
  }

  Serial.println("[CTRL] Unable to schedule relay pulse - pool exhausted");
}

void processRelayPulses() {
  unsigned long now = millis();
  for (auto &pulse : relayPulses) {
    if (pulse.active && static_cast<long>(now - pulse.endMillis) >= 0) {
      digitalWrite(pulse.pin, !RELAY_ACTIVE_LEVEL);
      pulse.active = false;
    }
  }
}

void syncEncoderToTarget() {
  float target = constrain(configManager.config().targetTemp, TARGET_MIN, TARGET_MAX);
  long newPos = lroundf(target / TARGET_STEP);
  encoder.setPosition(newPos);
  lastEncoderPosition = newPos;
}

float readCurrent() {
  constexpr int samples = 10;
  uint32_t accumulator = 0;
  for (int i = 0; i < samples; ++i) {
    accumulator += analogRead(PIN_ACS712);
    delayMicroseconds(250);
  }

  float average = static_cast<float>(accumulator) / samples;
  constexpr float adcMax = 4095.0f;
  constexpr float referenceVoltage = 3.3f;
  float voltage = (average / adcMax) * referenceVoltage;
  const CalibrationConfig &cal = configManager.config().calibration;
  return (voltage - cal.acsZero) / cal.acsSensitivity;
}

void updateTelemetry() {
  unsigned long now = millis();
  if (now - lastSensorPoll < SENSOR_INTERVAL_MS) {
    return;
  }
  lastSensorPoll = now;

  tempSensors.requestTemperatures();
  float insideRaw = insideSensorPresent ? tempSensors.getTempC(insideSensor) : NAN;
  telemetry.insideTemp = isValidTemp(insideRaw) ? insideRaw : NAN;
  float exhaustRaw = exhaustSensorPresent ? tempSensors.getTempC(exhaustSensor) : NAN;
  telemetry.exhaustTemp = isValidTemp(exhaustRaw) ? exhaustRaw : NAN;
  telemetry.current = readCurrent();

  int strokeValue = digitalRead(PIN_STROKE_SENSE);
  if (strokeValue == STROKE_SENSE_ACTIVE_LEVEL) {
    strokeFeedbackDetected = true;
    heaterSupplyState = true;
  } else if (strokeFeedbackDetected) {
    heaterSupplyState = false;
  } else {
    heaterSupplyState = heaterRequestOn;
  }

  telemetry.heaterSupplyOn = heaterSupplyState;
  telemetry.heaterRequest = heaterRequestOn;
  telemetry.strokeFeedbackPresent = strokeFeedbackDetected;
  telemetry.wifiConnected = WiFi.isConnected();
  telemetry.wifiRssi = telemetry.wifiConnected ? WiFi.RSSI() : 0;
  telemetry.stage = heaterStage;
  telemetry.targetTemp = configManager.config().targetTemp;
  telemetry.phase = currentPhase;
  telemetry.boostActive = startBoostActive;
}

void updateDisplay() {
  if (!displayPresent) {
    return;
  }

  unsigned long now = millis();
  if (now - lastDisplayUpdate < DISPLAY_INTERVAL_MS) {
    return;
  }
  lastDisplayUpdate = now;

  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  if (isValidTemp(telemetry.insideTemp)) {
    display.printf("Innen: %4.1fC\n", telemetry.insideTemp);
  } else {
    display.println("Innen: ----");
  }

  if (isValidTemp(telemetry.exhaustTemp)) {
    display.printf("Abluft: %4.1fC\n", telemetry.exhaustTemp);
  } else {
    display.println("Abluft: ----");
  }

  if (!isnan(telemetry.current)) {
    display.printf("Strom: %4.1fA\n", telemetry.current);
  } else {
    display.println("Strom: ----");
  }

  display.printf("Phase: %s\n", phaseName(currentPhase));
  display.printf("Stufe: %u Soll:%4.1f\n", telemetry.stage, telemetry.targetTemp);

  if (telemetry.wifiConnected) {
    display.printf("WiFi: %d dBm\n", telemetry.wifiRssi);
  } else {
    display.println("WiFi: offline");
  }

  display.printf("Vers: %s\n", telemetry.heaterSupplyOn ? "AN" : "AUS");
  display.display();
}

void processEncoder() {
  long position = encoder.getPosition();
  if (position == lastEncoderPosition) {
    return;
  }

  long delta = position - lastEncoderPosition;
  lastEncoderPosition = position;

  float newTarget = configManager.config().targetTemp + static_cast<float>(delta) * TARGET_STEP;
  newTarget = constrain(newTarget, TARGET_MIN, TARGET_MAX);
  configManager.config().targetTemp = newTarget;
  telemetry.targetTemp = newTarget;
  targetDirty = true;
  targetDirtyMillis = millis();
}

void processEncoderButton() {
  unsigned long now = millis();
  bool rawPressed = digitalRead(PIN_ENCODER_SW) == LOW;

  if (rawPressed != buttonLastRawState) {
    buttonLastRawState = rawPressed;
    buttonLastDebounceMillis = now;
  }

  if (static_cast<long>(now - buttonLastDebounceMillis) >= static_cast<long>(BUTTON_DEBOUNCE_MS)) {
    if (rawPressed != buttonDebouncedState) {
      buttonDebouncedState = rawPressed;
      if (buttonDebouncedState) {
        buttonPressStart = now;
        buttonTriggeredReset = false;
      } else {
        buttonPressStart = 0;
        buttonTriggeredReset = false;
      }
    }
  }

  if (buttonDebouncedState && buttonPressStart != 0 && !buttonTriggeredReset &&
      static_cast<long>(now - buttonPressStart) >= static_cast<long>(WIFI_RESET_HOLD_MS)) {
    buttonTriggeredReset = true;
    resetWiFiSettingsAndReboot();
  }
}

void saveTargetIfDirty() {
  if (!targetDirty) {
    return;
  }
  if (millis() - targetDirtyMillis < TARGET_SAVE_DELAY_MS) {
    return;
  }
  if (configManager.save()) {
    Serial.printf("[CFG] Target temperature saved: %.1f\n", configManager.config().targetTemp);
  } else {
    Serial.println("[CFG] Failed to save target temperature");
  }
  targetDirty = false;
}

void setHeaterRequest(bool on) {
  if (heaterRequestOn == on) {
    return;
  }

  heaterRequestOn = on;
  telemetry.heaterRequest = heaterRequestOn;

  if (on) {
    criticalOvershootHoldUntil = 0;
    Serial.println("[CTRL] Heater ON requested");
    pulseRelay(RELAY_ON_PIN);
    startBoostActive = true;
    startBoostStartMillis = millis();
    lastBoostPulseMillis = 0;
    boostPulsesSent = 0;
    heaterStage = 1;
    stageChangesInWindow = 0;
    stageWindowStart = millis();
  } else {
    Serial.println("[CTRL] Heater OFF requested");
    pulseRelay(RELAY_OFF_PIN);
    startBoostActive = false;
    criticalOvershootHoldUntil = 0;
  }
}

void issueStrokePulse() {
  Serial.println("[CTRL] Stromstoß ausgelöst");
  pulseRelay(RELAY_STROKE_PIN);
}

void requestStageChange(int8_t delta, bool manualOverride) {
  if (delta == 0) {
    return;
  }

  uint8_t desired = static_cast<uint8_t>(constrain(heaterStage + delta, 1, 10));
  if (desired == heaterStage) {
    return;
  }

  unsigned long now = millis();
  const RegulationConfig &reg = configManager.config().regulation;

  if (!manualOverride) {
    if (now - lastStageChangeMillis < reg.minStepInterval * 1000UL) {
      return;
    }

    if (now - stageWindowStart > STAGE_WINDOW_MS) {
      stageWindowStart = now;
      stageChangesInWindow = 0;
    }

    if (stageChangesInWindow >= reg.maxStepsPerQuarter) {
      return;
    }
  }

  if (desired > heaterStage) {
    pulseRelay(RELAY_PLUS_PIN);
  } else {
    pulseRelay(RELAY_MINUS_PIN);
  }

  heaterStage = desired;
  lastStageChangeMillis = now;
  if (!manualOverride) {
    if (now - stageWindowStart > STAGE_WINDOW_MS) {
      stageWindowStart = now;
      stageChangesInWindow = 0;
    }
    ++stageChangesInWindow;
  }
  telemetry.stage = heaterStage;
}

const char *phaseName(HeaterPhase phase) {
  switch (phase) {
    case HeaterPhase::Off:
      return "Aus";
    case HeaterPhase::Preheat:
      return "Vorgl";
    case HeaterPhase::Heating:
      return "Heizen";
    case HeaterPhase::Cooldown:
      return "Nachlauf";
    case HeaterPhase::Fault:
      return "Fehler";
    default:
      return "?";
  }
}

void evaluatePhase() {
  HeaterPhase newPhase = currentPhase;
  bool currentValid = !isnan(telemetry.current);
  bool exhaustValid = isValidTemp(telemetry.exhaustTemp);

  if (!heaterRequestOn && exhaustValid && telemetry.exhaustTemp >= EXHAUST_THRESHOLD_COOLDOWN) {
    newPhase = HeaterPhase::Cooldown;
  } else if (currentValid && telemetry.current > CURRENT_THRESHOLD_HEATING && exhaustValid &&
             telemetry.exhaustTemp >= EXHAUST_THRESHOLD_HEATING) {
    newPhase = HeaterPhase::Heating;
  } else if (currentValid && telemetry.current > CURRENT_THRESHOLD_PREHEAT &&
             (!exhaustValid || telemetry.exhaustTemp < EXHAUST_THRESHOLD_HEATING)) {
    newPhase = HeaterPhase::Preheat;
  } else if (!heaterRequestOn && (!currentValid || telemetry.current < CURRENT_THRESHOLD_OFF) &&
             (!exhaustValid || telemetry.exhaustTemp < EXHAUST_THRESHOLD_COOLDOWN)) {
    newPhase = HeaterPhase::Off;
  } else if (currentValid && telemetry.current < CURRENT_THRESHOLD_OFF) {
    newPhase = HeaterPhase::Off;
  }

  if (newPhase != currentPhase) {
    currentPhase = newPhase;
    phaseStartMillis = millis();
    Serial.printf("[STATE] Phase -> %s\n", phaseName(currentPhase));
  }

  if (currentPhase == HeaterPhase::Preheat && millis() - phaseStartMillis > FAILSTART_TIMEOUT_MS) {
    Serial.println("[STATE] Fehlstart erkannt, schalte Heizung ab");
    setHeaterRequest(false);
    currentPhase = HeaterPhase::Fault;
    phaseStartMillis = millis();
  }

  telemetry.phase = currentPhase;
}

void applyRegulation() {
  unsigned long now = millis();
  if (now - lastRegulationRun < REGULATION_INTERVAL_MS) {
    return;
  }
  lastRegulationRun = now;

  const AppConfig &cfg = configManager.config();

  if (!heaterRequestOn) {
    return;
  }

  if (startBoostActive) {
    return;
  }

  if (isValidTemp(telemetry.exhaustTemp)) {
    if (telemetry.exhaustTemp >= cfg.regulation.exhaustOff) {
      Serial.println("[SAFE] Abgastemperatur kritisch, schalte ab");
      setHeaterRequest(false);
      return;
    }
    if (telemetry.exhaustTemp >= cfg.regulation.exhaustWarn) {
      requestStageChange(-1);
      return;
    }
  }

  if (!isValidTemp(telemetry.insideTemp)) {
    return;
  }

  float target = cfg.targetTemp;

  if (telemetry.insideTemp > target + cfg.regulation.deltaUpCritical) {
    requestStageChange(-1, true);
    criticalOvershootHoldUntil = now + cfg.regulation.stabilizeDuration * 1000UL;
  } else if (telemetry.insideTemp > target + cfg.regulation.deltaUp) {
    requestStageChange(-1);
  } else if (telemetry.insideTemp < target - cfg.regulation.deltaDown) {
    if (static_cast<long>(now - criticalOvershootHoldUntil) >= 0) {
      requestStageChange(+1);
    }
  }
}

void handleStartBoost() {
  if (!startBoostActive) {
    return;
  }

  unsigned long now = millis();
  if (now - startBoostStartMillis < START_BOOST_DELAY_MS) {
    telemetry.boostActive = true;
    return;
  }

  if (boostPulsesSent >= START_BOOST_PULSES) {
    startBoostActive = false;
    telemetry.boostActive = false;
    return;
  }

  if (now - lastBoostPulseMillis < START_BOOST_PULSE_INTERVAL_MS) {
    telemetry.boostActive = true;
    return;
  }

  pulseRelay(RELAY_PLUS_PIN);
  lastBoostPulseMillis = now;
  ++boostPulsesSent;
  heaterStage = min<uint8_t>(10, static_cast<uint8_t>(heaterStage + 1));
  telemetry.stage = heaterStage;
  telemetry.boostActive = boostPulsesSent < START_BOOST_PULSES;
}

void sendStatus(AsyncWebServerRequest *request) {
  AsyncResponseStream *response = request->beginResponseStream("application/json");
  StaticJsonDocument<512> doc;

  if (isValidTemp(telemetry.insideTemp)) {
    doc["inside_temp"] = telemetry.insideTemp;
  } else {
    doc["inside_temp"] = nullptr;
  }

  if (isValidTemp(telemetry.exhaustTemp)) {
    doc["exhaust_temp"] = telemetry.exhaustTemp;
  } else {
    doc["exhaust_temp"] = nullptr;
  }

  if (!isnan(telemetry.current)) {
    doc["current"] = telemetry.current;
  } else {
    doc["current"] = nullptr;
  }

  doc["phase"] = phaseName(telemetry.phase);
  doc["level"] = telemetry.stage;
  doc["target_temp"] = telemetry.targetTemp;
  doc["heater_on"] = telemetry.heaterSupplyOn;
  doc["heater_request"] = telemetry.heaterRequest;
  doc["stroke_feedback"] = telemetry.strokeFeedbackPresent;
  doc["boost_active"] = telemetry.boostActive;
  doc["uptime"] = millis() / 1000;
  doc["hostname"] = OTA_HOSTNAME;

  if (telemetry.wifiConnected) {
    doc["wifi_rssi"] = telemetry.wifiRssi;
    doc["ip"] = WiFi.localIP().toString();
  } else {
    doc["wifi_rssi"] = nullptr;
  }

  serializeJson(doc, *response);
  request->send(response);
}

void handleControlRequest(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
  if (index != 0 || len != total) {
    return;
  }

  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, data, len);
  if (error) {
    request->send(400, "application/json", "{\"error\":\"invalid_json\"}");
    return;
  }

  if (doc.containsKey("heater_on")) {
    setHeaterRequest(doc["heater_on"].as<bool>());
  }

  if (doc.containsKey("target_temp")) {
    float value = doc["target_temp"].as<float>();
    value = constrain(value, TARGET_MIN, TARGET_MAX);
    configManager.config().targetTemp = value;
    telemetry.targetTemp = value;
    targetDirty = true;
    targetDirtyMillis = millis();
    syncEncoderToTarget();
  }

  if (doc.containsKey("command")) {
    const char *cmd = doc["command"].as<const char *>();
    if (strcmp(cmd, "plus") == 0) {
      requestStageChange(+1, true);
    } else if (strcmp(cmd, "minus") == 0) {
      requestStageChange(-1, true);
    } else if (strcmp(cmd, "stroke") == 0) {
      issueStrokePulse();
    }
  }

  request->send(204);
}

void setupWebServer() {
  server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");

  server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest *request) { sendStatus(request); });

  server.on("/api/control", HTTP_POST, [](AsyncWebServerRequest *request) {}, nullptr,
            [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
              handleControlRequest(request, data, len, index, total);
            });

  server.on("/api/wifi/reset", HTTP_POST, [](AsyncWebServerRequest *request) {
    request->send(204);
    resetWiFiSettingsAndReboot();
  });

  server.onNotFound([](AsyncWebServerRequest *request) { request->send(404, "text/plain", "Not found"); });

  ElegantOTA.begin(&server);
  ElegantOTA.setID(OTA_HOSTNAME);
  server.begin();
}

void setupSensors() {
  tempSensors.begin();
  tempSensors.setResolution(11);

  const SensorConfig &sensorCfg = configManager.config().sensors;
  sensorConfigDirty = false;

  insideSensorPresent = applyStoredSensorAddress(sensorCfg.inside, insideSensor);
  if (insideSensorPresent) {
    tempSensors.setResolution(insideSensor, 11);
    Serial.printf("[TEMP] Innenfühler geladen: %s\n", formatSensorAddress(insideSensor).c_str());
  }

  exhaustSensorPresent = applyStoredSensorAddress(sensorCfg.exhaust, exhaustSensor);
  if (exhaustSensorPresent) {
    tempSensors.setResolution(exhaustSensor, 11);
    Serial.printf("[TEMP] Abluftfühler geladen: %s\n", formatSensorAddress(exhaustSensor).c_str());
  }

  DeviceAddress candidate{};
  int deviceCount = tempSensors.getDeviceCount();
  for (int i = 0; i < deviceCount && (!insideSensorPresent || !exhaustSensorPresent); ++i) {
    if (!tempSensors.getAddress(candidate, i)) {
      continue;
    }
    if (!tempSensors.validAddress(candidate) || !tempSensors.isConnected(candidate)) {
      continue;
    }

    if (!insideSensorPresent) {
      memcpy(insideSensor, candidate, sizeof(DeviceAddress));
      insideSensorPresent = true;
      tempSensors.setResolution(insideSensor, 11);
      if (updateSensorAddressConfig(configManager.config().sensors.inside, insideSensor)) {
        sensorConfigDirty = true;
      }
      Serial.printf("[TEMP] Innenfühler gelernt: %s\n", formatSensorAddress(insideSensor).c_str());
      continue;
    }

    if (!exhaustSensorPresent && !addressesEqual(candidate, insideSensor)) {
      memcpy(exhaustSensor, candidate, sizeof(DeviceAddress));
      exhaustSensorPresent = true;
      tempSensors.setResolution(exhaustSensor, 11);
      if (updateSensorAddressConfig(configManager.config().sensors.exhaust, exhaustSensor)) {
        sensorConfigDirty = true;
      }
      Serial.printf("[TEMP] Abluftfühler gelernt: %s\n", formatSensorAddress(exhaustSensor).c_str());
    }
  }

  if (!insideSensorPresent) {
    Serial.println("[TEMP] Innenfühler nicht gefunden");
  }

  if (!exhaustSensorPresent) {
    Serial.println("[TEMP] Abluftfühler nicht gefunden");
  }

  if (sensorConfigDirty) {
    if (configManager.save()) {
      Serial.println("[TEMP] Sensorspeicher aktualisiert");
    } else {
      Serial.println("[TEMP] Sensorspeicher konnte nicht gesichert werden");
    }
  }
}

void setupDisplay() {
  Wire.begin(PIN_OLED_SDA, PIN_OLED_SCL);
  displayPresent = display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDRESS);
  if (displayPresent) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Sunster Heizregler");
    display.println("Bootet...");
    display.display();
  } else {
    Serial.println("[OLED] Display nicht gefunden");
  }
}

void setupEncoderHardware() {
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_CLK), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_DT), encoderISR, CHANGE);

  syncEncoderToTarget();
}

} // namespace

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("[BOOT] Starte Sunster Heizungsregler");

  if (!LittleFS.begin(true)) {
    Serial.println("[FS] LittleFS konnte nicht eingehängt werden");
  }

  configurePins();

  bool initialButtonState = digitalRead(PIN_ENCODER_SW) == LOW;
  buttonLastRawState = initialButtonState;
  buttonDebouncedState = initialButtonState;
  buttonLastDebounceMillis = millis();

  if (!configManager.begin()) {
    Serial.println("[CFG] Konfiguration konnte nicht geladen werden");
  }

  setupSensors();
  setupDisplay();
  setupEncoderHardware();

  if (!autoConfigureWiFi(OTA_HOSTNAME, WIFI_AP_NAME, WIFI_AP_PASSWORD)) {
    Serial.println("[WiFi] Starte Konfigurationsportal");
  }
  WiFi.setAutoReconnect(true);

  setupWebServer();

  Serial.println("[BOOT] Setup abgeschlossen");
}

void loop() {
  processRelayPulses();
  ElegantOTA.loop();

  updateTelemetry();
  evaluatePhase();
  handleStartBoost();
  applyRegulation();
  processEncoder();
  processEncoderButton();
  saveTargetIfDirty();
  updateDisplay();
}
