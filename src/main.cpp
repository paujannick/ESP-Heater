#include <Arduino.h>
#include <ArduinoJson.h>
#include <AsyncTCP.h>
#include <DallasTemperature.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <OneWire.h>
#include <RotaryEncoder.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <array>
#include <cmath>
#include <cstring>
#include <driver/adc.h>

#include "config_manager.h"
#include "pins.h"
#include "wifi_support.h"

namespace {

constexpr char DEVICE_HOSTNAME[] = "sunster-heater";
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
constexpr uint32_t ENCODER_LONG_PRESS_MS = 1000;
constexpr uint32_t ENCODER_EDIT_TIMEOUT_MS = 30000;
constexpr uint32_t START_BOOST_DELAY_MS = 120000;
constexpr uint32_t START_BOOST_PULSE_INTERVAL_MS = 1200;
constexpr uint8_t START_BOOST_PULSES = 10;
constexpr uint32_t HEATER_ON_COMMAND_DELAY_MS = 5000;
constexpr uint32_t BUTTON_DEBOUNCE_MS = 50;
constexpr unsigned long RELAY_RETRIGGER_GUARD_MS = 400;

constexpr float CURRENT_THRESHOLD_OFF = 0.2f;
constexpr float CURRENT_THRESHOLD_HEATING = 0.8f;
constexpr float CURRENT_THRESHOLD_PREHEAT = 5.0f;

constexpr float EXHAUST_THRESHOLD_HEATING = 40.0f;
constexpr float EXHAUST_THRESHOLD_COOLDOWN = 50.0f;

constexpr uint32_t FAILSTART_TIMEOUT_MS = 180000;
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
  bool manualMode = false;
};

TelemetryState telemetry;

bool heaterRequestOn = false;
bool heaterSupplyState = false;
uint8_t heaterStage = 1;
bool manualControlMode = false;
bool mainRelayAssumedOn = false;
bool pendingMainRelayOff = false;

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

bool heaterOnPulsePending = false;
unsigned long heaterOnPulseDueMillis = 0;

bool targetDirty = false;
unsigned long targetDirtyMillis = 0;

unsigned long buttonPressStart = 0;
bool buttonTriggeredReset = false;
bool buttonLastRawState = false;
bool buttonDebouncedState = false;
unsigned long buttonLastDebounceMillis = 0;

bool strokeFeedbackAvailable = false;
bool strokeFeedbackSeenActive = false;
bool strokeFeedbackSeenInactive = false;

long lastEncoderPosition = 0;
bool encoderEditingActive = false;
float encoderDraftTarget = TARGET_MIN;
unsigned long encoderLastActivity = 0;
unsigned long criticalOvershootHoldUntil = 0;
bool sensorConfigDirty = false;
bool webServerRunning = false;

struct RelayPulseState {
  bool active = false;
  uint8_t pin = 0;
  unsigned long endMillis = 0;
};

RelayPulseState relayPulses[5];
std::array<unsigned long, 5> relayLastPulseMillis{};
std::array<String, 7> mirroredDisplayLines{};

float readCurrent();
void updateTelemetry();
void updateDisplay();
void processEncoder();
void processEncoderButton();
void evaluatePhase();
void applyRegulation();
void handleStartBoost();
void processPendingHeaterOnCommand();
void sendStatus(AsyncWebServerRequest *request);
void handleControlRequest(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total);
void sendConfig(AsyncWebServerRequest *request);
void handleConfigUpdate(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total);
void setupWebServer();
void setHeaterRequest(bool on);
bool issueStrokePulse();
void requestStageChange(int8_t delta, bool manualOverride = false);
const char *phaseName(HeaterPhase phase);
void processRelayPulses();
void syncEncoderToTarget();
void commitEncoderTarget();
void cancelEncoderEditing();
void updateEncoderEditingState();
bool addressesEqual(const uint8_t *lhs, const uint8_t *rhs);
bool applyStoredSensorAddress(const SensorAddressConfig &config, DeviceAddress &destination);
bool updateSensorAddressConfig(SensorAddressConfig &config, const DeviceAddress &address);
bool parseSensorAddressString(const char *text, DeviceAddress &out);
bool updateTargetTemperature(float value, bool scheduleSave);
bool isNumeric(JsonVariantConst value);
bool isBoolean(JsonVariantConst value);
std::array<String, 7> composeVirtualDisplayLines();
int relayIndexForPin(uint8_t pin);

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

bool isNumeric(JsonVariantConst value) {
  return value.is<int>() || value.is<long>() || value.is<unsigned int>() || value.is<unsigned long>() ||
         value.is<float>() || value.is<double>();
}

bool isBoolean(JsonVariantConst value) { return value.is<bool>(); }

bool parseSensorAddressString(const char *text, DeviceAddress &out) {
  if (text == nullptr) {
    return false;
  }

  size_t index = 0;
  int highNibble = -1;

  for (const char *p = text; *p != '\0'; ++p) {
    char c = *p;
    if (c == ':' || c == ' ' || c == '-') {
      continue;
    }

    int value = -1;
    if (c >= '0' && c <= '9') {
      value = c - '0';
    } else if (c >= 'a' && c <= 'f') {
      value = 10 + (c - 'a');
    } else if (c >= 'A' && c <= 'F') {
      value = 10 + (c - 'A');
    }

    if (value < 0) {
      return false;
    }

    if (highNibble < 0) {
      highNibble = value;
    } else {
      if (index >= 8) {
        return false;
      }
      out[index++] = static_cast<uint8_t>((highNibble << 4) | value);
      highNibble = -1;
    }
  }

  if (highNibble >= 0 || index != 8) {
    return false;
  }

  return true;
}

bool updateTargetTemperature(float value, bool scheduleSave) {
  value = constrain(value, TARGET_MIN, TARGET_MAX);
  float previous = configManager.config().targetTemp;
  bool changed = fabsf(value - previous) > 0.001f;

  configManager.config().targetTemp = value;
  telemetry.targetTemp = value;
  encoderEditingActive = false;
  encoderLastActivity = 0;
  syncEncoderToTarget();

  if (scheduleSave) {
    if (changed) {
      targetDirty = true;
      targetDirtyMillis = millis();
    }
  } else {
    targetDirty = false;
    targetDirtyMillis = 0;
  }

  return changed;
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

int relayIndexForPin(uint8_t pin) {
  switch (pin) {
    case RELAY_STROKE_PIN:
      return 0;
    case RELAY_ON_PIN:
      return 1;
    case RELAY_OFF_PIN:
      return 2;
    case RELAY_PLUS_PIN:
      return 3;
    case RELAY_MINUS_PIN:
      return 4;
    default:
      return -1;
  }
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

bool pulseRelay(uint8_t pin, uint16_t duration = RELAY_PULSE_MS, bool blocking = false) {
  unsigned long now = millis();
  int index = relayIndexForPin(pin);

  if (index >= 0) {
    unsigned long last = relayLastPulseMillis[static_cast<size_t>(index)];
    if (last != 0 && static_cast<long>(now - last) < static_cast<long>(RELAY_RETRIGGER_GUARD_MS)) {
      Serial.printf("[CTRL] Relay pulse on pin %u suppressed (%lums since last)\n", pin,
                    static_cast<unsigned long>(now - last));
      return false;
    }
  }

  if (blocking) {
    digitalWrite(pin, RELAY_ACTIVE_LEVEL);
    delay(duration);
    digitalWrite(pin, !RELAY_ACTIVE_LEVEL);
    if (index >= 0) {
      relayLastPulseMillis[static_cast<size_t>(index)] = now;
    }
    return true;
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
      pulse.endMillis = now + duration;
      if (index >= 0) {
        relayLastPulseMillis[static_cast<size_t>(index)] = now;
      }
      return true;
    }
  }

  Serial.println("[CTRL] Unable to schedule relay pulse - pool exhausted");
  return false;
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
  encoderDraftTarget = target;
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

  bool strokeActive = digitalRead(PIN_STROKE_SENSE) == STROKE_SENSE_ACTIVE_LEVEL;
  if (strokeActive) {
    strokeFeedbackSeenActive = true;
  } else {
    strokeFeedbackSeenInactive = true;
  }

  if (!strokeFeedbackAvailable && strokeFeedbackSeenActive && strokeFeedbackSeenInactive) {
    strokeFeedbackAvailable = true;
  }

  if (strokeFeedbackAvailable) {
    heaterSupplyState = strokeActive;
  } else {
    heaterSupplyState = heaterRequestOn;
  }

  telemetry.heaterSupplyOn = heaterSupplyState;
  telemetry.heaterRequest = heaterRequestOn;
  telemetry.manualMode = manualControlMode;
  telemetry.strokeFeedbackPresent = strokeFeedbackAvailable;
  telemetry.wifiConnected = WiFi.isConnected();
  telemetry.wifiRssi = telemetry.wifiConnected ? WiFi.RSSI() : 0;
  telemetry.stage = heaterStage;
  telemetry.targetTemp = encoderEditingActive ? encoderDraftTarget : configManager.config().targetTemp;
  telemetry.phase = currentPhase;
  telemetry.boostActive = startBoostActive;

  if (strokeFeedbackAvailable) {
    mainRelayAssumedOn = heaterSupplyState;
  }
}

std::array<String, 7> composeVirtualDisplayLines() {
  std::array<String, 7> lines{};

  lines[0] = F("Innen: ");
  if (isValidTemp(telemetry.insideTemp)) {
    lines[0] += String(telemetry.insideTemp, 1);
    lines[0] += F("C");
  } else {
    lines[0] += F("----");
  }

  lines[1] = F("Abluft: ");
  if (isValidTemp(telemetry.exhaustTemp)) {
    lines[1] += String(telemetry.exhaustTemp, 1);
    lines[1] += F("C");
  } else {
    lines[1] += F("----");
  }

  lines[2] = F("Strom: ");
  if (!isnan(telemetry.current)) {
    lines[2] += String(telemetry.current, 1);
    lines[2] += F("A");
  } else {
    lines[2] += F("----");
  }

  lines[3] = F("Phase: ");
  lines[3] += phaseName(currentPhase);

  lines[4] = F("Stufe: ");
  lines[4] += String(telemetry.stage);
  lines[4] += F(" Soll:");
  if (!isnan(telemetry.targetTemp)) {
    lines[4] += String(telemetry.targetTemp, 1);
  } else {
    lines[4] += F("--");
  }

  if (telemetry.wifiConnected) {
    lines[5] = F("WiFi: ");
    lines[5] += String(telemetry.wifiRssi);
    lines[5] += F(" dBm");
  } else {
    lines[5] = F("WiFi: offline");
  }

  lines[6] = F("Vers: ");
  lines[6] += telemetry.heaterSupplyOn ? F("AN") : F("AUS");

  return lines;
}

void updateDisplay() {
  unsigned long now = millis();
  if (now - lastDisplayUpdate < DISPLAY_INTERVAL_MS) {
    return;
  }
  lastDisplayUpdate = now;

  auto lines = composeVirtualDisplayLines();
  mirroredDisplayLines = lines;

  if (!displayPresent) {
    return;
  }

  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  for (const auto &line : lines) {
    display.println(line);
  }
  display.display();
}

void processEncoder() {
  long position = encoder.getPosition();
  if (position == lastEncoderPosition) {
    return;
  }

  long delta = position - lastEncoderPosition;
  if (!encoderEditingActive) {
    encoderEditingActive = true;
    encoderDraftTarget = configManager.config().targetTemp;
    Serial.println("[ENC] Zieltemperatur bearbeiten gestartet");
  }

  encoderDraftTarget = constrain(encoderDraftTarget + static_cast<float>(delta) * TARGET_STEP, TARGET_MIN, TARGET_MAX);

  long constrainedPosition = lroundf(encoderDraftTarget / TARGET_STEP);
  encoder.setPosition(constrainedPosition);
  lastEncoderPosition = constrainedPosition;
  telemetry.targetTemp = encoderDraftTarget;
  encoderLastActivity = millis();
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
        if (buttonPressStart != 0) {
          unsigned long pressDuration = now - buttonPressStart;
          if (!buttonTriggeredReset) {
            if (pressDuration >= ENCODER_LONG_PRESS_MS) {
              bool desiredState = !heaterRequestOn;
              Serial.printf("[ENC] Langer Druck erkannt, Heizung %s\n", desiredState ? "AN" : "AUS");
              cancelEncoderEditing();
              setHeaterRequest(desiredState);
            } else {
              Serial.println("[ENC] Kurzer Druck erkannt, Zieltemperatur übernehmen");
              commitEncoderTarget();
            }
          }
        }
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

void commitEncoderTarget() {
  float committed = encoderEditingActive ? encoderDraftTarget : configManager.config().targetTemp;
  committed = constrain(committed, TARGET_MIN, TARGET_MAX);
  float previous = configManager.config().targetTemp;
  bool changed = fabsf(committed - previous) > 0.001f;

  configManager.config().targetTemp = committed;
  telemetry.targetTemp = committed;

  if (changed) {
    targetDirty = true;
    targetDirtyMillis = millis();
  }

  encoderEditingActive = false;
  encoderLastActivity = 0;
  encoderDraftTarget = committed;
  syncEncoderToTarget();

  if (changed) {
    Serial.printf("[ENC] Zieltemperatur gesetzt: %.1f°C\n", committed);
  }
}

void cancelEncoderEditing() {
  if (!encoderEditingActive) {
    return;
  }
  encoderEditingActive = false;
  encoderLastActivity = 0;
  encoderDraftTarget = configManager.config().targetTemp;
  telemetry.targetTemp = encoderDraftTarget;
  syncEncoderToTarget();
  Serial.println("[ENC] Bearbeitung verworfen");
}

void updateEncoderEditingState() {
  if (!encoderEditingActive) {
    return;
  }

  unsigned long now = millis();
  if (static_cast<long>(now - encoderLastActivity) >= static_cast<long>(ENCODER_EDIT_TIMEOUT_MS)) {
    Serial.println("[ENC] Bearbeitung aufgrund Inaktivität beendet");
    cancelEncoderEditing();
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
    if (!mainRelayAssumedOn) {
      Serial.println("[CTRL] Hauptrelais EIN (automatisch)");
      if (issueStrokePulse()) {
        mainRelayAssumedOn = true;
      }
    }
    pendingMainRelayOff = false;
    heaterOnPulsePending = true;
    heaterOnPulseDueMillis = millis() + HEATER_ON_COMMAND_DELAY_MS;
    Serial.printf("[CTRL] Verzögerter EIN-Befehl (%lums)\n", static_cast<unsigned long>(HEATER_ON_COMMAND_DELAY_MS));
  } else {
    Serial.println("[CTRL] Heater OFF requested");
    heaterOnPulsePending = false;
    if (!pulseRelay(RELAY_OFF_PIN)) {
      Serial.println("[CTRL] AUS-Befehl konnte nicht gesendet werden (Relais gesperrt)");
    }
    startBoostActive = false;
    criticalOvershootHoldUntil = 0;
    if (mainRelayAssumedOn) {
      pendingMainRelayOff = true;
    }
  }
}

bool issueStrokePulse() {
  bool triggered = pulseRelay(RELAY_STROKE_PIN);
  if (triggered) {
    Serial.println("[CTRL] Stromstoß ausgelöst");
  } else {
    Serial.println("[CTRL] Stromstoß blockiert (Sicherheitssperre aktiv)");
  }
  return triggered;
}

void requestStageChange(int8_t delta, bool manualOverride) {
  if (delta == 0) {
    return;
  }

  if (!heaterRequestOn && !startBoostActive && !manualOverride) {
    return;
  }

  if (manualOverride && !heaterSupplyState && !heaterRequestOn && !startBoostActive) {
    Serial.println("[CTRL] Stufenänderung ignoriert - Versorgung aus");
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

  bool increased = desired > heaterStage;
  if (!pulseRelay(increased ? RELAY_PLUS_PIN : RELAY_MINUS_PIN)) {
    Serial.println("[CTRL] Stufenänderung blockiert - Relais gesperrt");
    return;
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

  if (pendingMainRelayOff && currentPhase == HeaterPhase::Off && !heaterRequestOn &&
      (isnan(telemetry.current) || telemetry.current < CURRENT_THRESHOLD_OFF) &&
      (!isValidTemp(telemetry.exhaustTemp) || telemetry.exhaustTemp < EXHAUST_THRESHOLD_COOLDOWN)) {
    Serial.println("[CTRL] Hauptrelais AUS nach sicherem Stopp");
    if (issueStrokePulse()) {
      pendingMainRelayOff = false;
      mainRelayAssumedOn = false;
    }
  }
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

  if (manualControlMode) {
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

void processPendingHeaterOnCommand() {
  if (!heaterOnPulsePending) {
    return;
  }

  unsigned long now = millis();
  if (static_cast<long>(now - heaterOnPulseDueMillis) < 0) {
    return;
  }

  Serial.println("[CTRL] Verzögerter EIN-Befehl ausgelöst");
  if (!pulseRelay(RELAY_ON_PIN)) {
    Serial.println("[CTRL] EIN-Befehl blockiert - versuche erneut");
    heaterOnPulseDueMillis = now + RELAY_RETRIGGER_GUARD_MS;
    return;
  }

  heaterOnPulsePending = false;
  startBoostActive = true;
  startBoostStartMillis = now;
  lastBoostPulseMillis = 0;
  boostPulsesSent = 0;
  heaterStage = 1;
  telemetry.stage = heaterStage;
  stageChangesInWindow = 0;
  stageWindowStart = now;
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

  if (!pulseRelay(RELAY_PLUS_PIN, RELAY_PULSE_MS, true)) {
    telemetry.boostActive = true;
    return;
  }
  now = millis();
  lastBoostPulseMillis = now;
  ++boostPulsesSent;
  heaterStage = min<uint8_t>(10, static_cast<uint8_t>(heaterStage + 1));
  telemetry.stage = heaterStage;
  telemetry.boostActive = boostPulsesSent < START_BOOST_PULSES;
}

void sendStatus(AsyncWebServerRequest *request) {
  AsyncResponseStream *response = request->beginResponseStream("application/json");
  StaticJsonDocument<512> doc;

  auto lines = composeVirtualDisplayLines();
  mirroredDisplayLines = lines;

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
  doc["mode"] = telemetry.manualMode ? "manual" : "auto";
  doc["uptime"] = millis() / 1000;
  doc["hostname"] = DEVICE_HOSTNAME;

  if (telemetry.wifiConnected) {
    doc["wifi_rssi"] = telemetry.wifiRssi;
    doc["ip"] = WiFi.localIP().toString();
  } else {
    doc["wifi_rssi"] = nullptr;
  }

  JsonArray displayLines = doc.createNestedArray("display");
  for (const auto &line : lines) {
    displayLines.add(line);
  }

  serializeJson(doc, *response);
  request->send(response);
}

void sendConfig(AsyncWebServerRequest *request) {
  AsyncResponseStream *response = request->beginResponseStream("application/json");
  StaticJsonDocument<1152> doc;
  const AppConfig &cfg = configManager.config();

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

  JsonObject telegram = doc.createNestedObject("telegram");
  telegram["enabled"] = cfg.telegram.enabled;
  telegram["token"] = cfg.telegram.token;
  telegram["chat_id"] = cfg.telegram.chatId;

  JsonObject calibration = doc.createNestedObject("calibration");
  calibration["acs_zero"] = cfg.calibration.acsZero;
  calibration["acs_sensitivity"] = cfg.calibration.acsSensitivity;

  JsonObject sensors = doc.createNestedObject("sensors");
  if (cfg.sensors.inside.valid) {
    sensors["inside"] = formatSensorAddress(cfg.sensors.inside.address);
  } else {
    sensors["inside"] = nullptr;
  }
  if (cfg.sensors.exhaust.valid) {
    sensors["exhaust"] = formatSensorAddress(cfg.sensors.exhaust.address);
  } else {
    sensors["exhaust"] = nullptr;
  }

  serializeJson(doc, *response);
  request->send(response);
}

void handleConfigUpdate(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
  if (index != 0 || len != total) {
    return;
  }

  StaticJsonDocument<1536> doc;
  DeserializationError error = deserializeJson(doc, data, len);
  if (error) {
    request->send(400, "application/json", "{\"error\":\"invalid_json\"}");
    return;
  }

  AppConfig &cfg = configManager.config();
  bool configChanged = false;
  bool sensorsChanged = false;

  if (doc.containsKey("target_temp")) {
    JsonVariantConst target = doc["target_temp"];
    if (!isNumeric(target)) {
      request->send(400, "application/json", "{\"error\":\"invalid_target_temp\"}");
      return;
    }

    bool changed = updateTargetTemperature(target.as<float>(), false);
    if (changed) {
      Serial.printf("[CFG] Zieltemperatur aktualisiert: %.1f°C\n", cfg.targetTemp);
      configChanged = true;
    }
  }

  JsonObjectConst reg = doc["regulation"].as<JsonObjectConst>();
  if (!reg.isNull()) {
    if (reg.containsKey("delta_down")) {
      JsonVariantConst value = reg["delta_down"];
      if (!isNumeric(value)) {
        request->send(400, "application/json", "{\"error\":\"invalid_regulation_delta_down\"}");
        return;
      }
      float newValue = value.as<float>();
      if (fabsf(newValue - cfg.regulation.deltaDown) > 0.0001f) {
        cfg.regulation.deltaDown = newValue;
        configChanged = true;
      }
    }

    if (reg.containsKey("delta_up")) {
      JsonVariantConst value = reg["delta_up"];
      if (!isNumeric(value)) {
        request->send(400, "application/json", "{\"error\":\"invalid_regulation_delta_up\"}");
        return;
      }
      float newValue = value.as<float>();
      if (fabsf(newValue - cfg.regulation.deltaUp) > 0.0001f) {
        cfg.regulation.deltaUp = newValue;
        configChanged = true;
      }
    }

    if (reg.containsKey("delta_up_critical")) {
      JsonVariantConst value = reg["delta_up_critical"];
      if (!isNumeric(value)) {
        request->send(400, "application/json", "{\"error\":\"invalid_regulation_delta_up_critical\"}");
        return;
      }
      float newValue = value.as<float>();
      if (fabsf(newValue - cfg.regulation.deltaUpCritical) > 0.0001f) {
        cfg.regulation.deltaUpCritical = newValue;
        configChanged = true;
      }
    }

    if (reg.containsKey("min_step_interval")) {
      JsonVariantConst value = reg["min_step_interval"];
      if (!isNumeric(value)) {
        request->send(400, "application/json", "{\"error\":\"invalid_regulation_min_step_interval\"}");
        return;
      }
      uint32_t newValue = value.as<uint32_t>();
      if (newValue != cfg.regulation.minStepInterval) {
        cfg.regulation.minStepInterval = newValue;
        configChanged = true;
      }
    }

    if (reg.containsKey("stabilize_duration")) {
      JsonVariantConst value = reg["stabilize_duration"];
      if (!isNumeric(value)) {
        request->send(400, "application/json", "{\"error\":\"invalid_regulation_stabilize_duration\"}");
        return;
      }
      uint32_t newValue = value.as<uint32_t>();
      if (newValue != cfg.regulation.stabilizeDuration) {
        cfg.regulation.stabilizeDuration = newValue;
        configChanged = true;
      }
    }

    if (reg.containsKey("max_steps_per_quarter")) {
      JsonVariantConst value = reg["max_steps_per_quarter"];
      if (!isNumeric(value)) {
        request->send(400, "application/json", "{\"error\":\"invalid_regulation_max_steps\"}");
        return;
      }
      uint8_t newValue = static_cast<uint8_t>(value.as<unsigned long>());
      if (newValue != cfg.regulation.maxStepsPerQuarter) {
        cfg.regulation.maxStepsPerQuarter = newValue;
        configChanged = true;
      }
    }

    if (reg.containsKey("exhaust_warn")) {
      JsonVariantConst value = reg["exhaust_warn"];
      if (!isNumeric(value)) {
        request->send(400, "application/json", "{\"error\":\"invalid_regulation_exhaust_warn\"}");
        return;
      }
      float newValue = value.as<float>();
      if (fabsf(newValue - cfg.regulation.exhaustWarn) > 0.0001f) {
        cfg.regulation.exhaustWarn = newValue;
        configChanged = true;
      }
    }

    if (reg.containsKey("exhaust_off")) {
      JsonVariantConst value = reg["exhaust_off"];
      if (!isNumeric(value)) {
        request->send(400, "application/json", "{\"error\":\"invalid_regulation_exhaust_off\"}");
        return;
      }
      float newValue = value.as<float>();
      if (fabsf(newValue - cfg.regulation.exhaustOff) > 0.0001f) {
        cfg.regulation.exhaustOff = newValue;
        configChanged = true;
      }
    }
  }

  JsonObjectConst telegram = doc["telegram"].as<JsonObjectConst>();
  if (!telegram.isNull()) {
    if (telegram.containsKey("enabled")) {
      JsonVariantConst value = telegram["enabled"];
      if (!isBoolean(value)) {
        request->send(400, "application/json", "{\"error\":\"invalid_telegram_enabled\"}");
        return;
      }
      bool newValue = value.as<bool>();
      if (newValue != cfg.telegram.enabled) {
        cfg.telegram.enabled = newValue;
        configChanged = true;
      }
    }

    if (telegram.containsKey("token")) {
      JsonVariantConst value = telegram["token"];
      if (!value.is<const char *>() && !value.isNull()) {
        request->send(400, "application/json", "{\"error\":\"invalid_telegram_token\"}");
        return;
      }
      const char *text = value.isNull() ? "" : value.as<const char *>();
      if (cfg.telegram.token != text) {
        cfg.telegram.token = text;
        configChanged = true;
      }
    }

    if (telegram.containsKey("chat_id")) {
      JsonVariantConst value = telegram["chat_id"];
      if (!value.is<const char *>() && !value.isNull()) {
        request->send(400, "application/json", "{\"error\":\"invalid_telegram_chat_id\"}");
        return;
      }
      const char *text = value.isNull() ? "" : value.as<const char *>();
      if (cfg.telegram.chatId != text) {
        cfg.telegram.chatId = text;
        configChanged = true;
      }
    }
  }

  JsonObjectConst calibration = doc["calibration"].as<JsonObjectConst>();
  if (!calibration.isNull()) {
    if (calibration.containsKey("acs_zero")) {
      JsonVariantConst value = calibration["acs_zero"];
      if (!isNumeric(value)) {
        request->send(400, "application/json", "{\"error\":\"invalid_calibration_acs_zero\"}");
        return;
      }
      float newValue = value.as<float>();
      if (fabsf(newValue - cfg.calibration.acsZero) > 0.0001f) {
        cfg.calibration.acsZero = newValue;
        configChanged = true;
      }
    }

    if (calibration.containsKey("acs_sensitivity")) {
      JsonVariantConst value = calibration["acs_sensitivity"];
      if (!isNumeric(value)) {
        request->send(400, "application/json", "{\"error\":\"invalid_calibration_acs_sensitivity\"}");
        return;
      }
      float newValue = value.as<float>();
      if (fabsf(newValue - cfg.calibration.acsSensitivity) > 0.0001f) {
        cfg.calibration.acsSensitivity = newValue;
        configChanged = true;
      }
    }
  }

  JsonObjectConst sensors = doc["sensors"].as<JsonObjectConst>();
  if (!sensors.isNull()) {
    if (sensors.containsKey("inside")) {
      JsonVariantConst value = sensors["inside"];
      if (value.isNull()) {
        if (cfg.sensors.inside.valid) {
          cfg.sensors.inside.valid = false;
          memset(cfg.sensors.inside.address, 0, sizeof(DeviceAddress));
          configChanged = true;
          sensorsChanged = true;
          Serial.println("[CFG] Innenfühler-Adresse gelöscht");
        }
      } else if (value.is<const char *>()) {
        DeviceAddress parsed{};
        if (!parseSensorAddressString(value.as<const char *>(), parsed)) {
          request->send(400, "application/json", "{\"error\":\"invalid_sensor_inside\"}");
          return;
        }
        if (!cfg.sensors.inside.valid || memcmp(cfg.sensors.inside.address, parsed, sizeof(DeviceAddress)) != 0) {
          memcpy(cfg.sensors.inside.address, parsed, sizeof(DeviceAddress));
          cfg.sensors.inside.valid = true;
          configChanged = true;
          sensorsChanged = true;
          Serial.printf("[CFG] Innenfühler-Adresse aktualisiert: %s\n", formatSensorAddress(parsed).c_str());
        }
      } else {
        request->send(400, "application/json", "{\"error\":\"invalid_sensor_inside\"}");
        return;
      }
    }

    if (sensors.containsKey("exhaust")) {
      JsonVariantConst value = sensors["exhaust"];
      if (value.isNull()) {
        if (cfg.sensors.exhaust.valid) {
          cfg.sensors.exhaust.valid = false;
          memset(cfg.sensors.exhaust.address, 0, sizeof(DeviceAddress));
          configChanged = true;
          sensorsChanged = true;
          Serial.println("[CFG] Abluftfühler-Adresse gelöscht");
        }
      } else if (value.is<const char *>()) {
        DeviceAddress parsed{};
        if (!parseSensorAddressString(value.as<const char *>(), parsed)) {
          request->send(400, "application/json", "{\"error\":\"invalid_sensor_exhaust\"}");
          return;
        }
        if (!cfg.sensors.exhaust.valid || memcmp(cfg.sensors.exhaust.address, parsed, sizeof(DeviceAddress)) != 0) {
          memcpy(cfg.sensors.exhaust.address, parsed, sizeof(DeviceAddress));
          cfg.sensors.exhaust.valid = true;
          configChanged = true;
          sensorsChanged = true;
          Serial.printf("[CFG] Abluftfühler-Adresse aktualisiert: %s\n", formatSensorAddress(parsed).c_str());
        }
      } else {
        request->send(400, "application/json", "{\"error\":\"invalid_sensor_exhaust\"}");
        return;
      }
    }
  }

  if (configChanged) {
    if (!configManager.save()) {
      request->send(500, "application/json", "{\"error\":\"save_failed\"}");
      return;
    }
    Serial.println("[CFG] Konfiguration gespeichert");
  }

  if (sensorsChanged) {
    if (cfg.sensors.inside.valid && applyStoredSensorAddress(cfg.sensors.inside, insideSensor)) {
      insideSensorPresent = true;
      tempSensors.setResolution(insideSensor, 11);
    } else {
      insideSensorPresent = false;
    }

    if (cfg.sensors.exhaust.valid && applyStoredSensorAddress(cfg.sensors.exhaust, exhaustSensor)) {
      exhaustSensorPresent = true;
      tempSensors.setResolution(exhaustSensor, 11);
    } else {
      exhaustSensorPresent = false;
    }
  }

  request->send(204);
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
    JsonVariantConst target = doc["target_temp"];
    if (!isNumeric(target)) {
      request->send(400, "application/json", "{\"error\":\"invalid_target_temp\"}");
      return;
    }

    bool changed = updateTargetTemperature(target.as<float>(), true);
    if (changed) {
      Serial.printf("[CTRL] Zieltemperatur extern gesetzt: %.1f°C\n", configManager.config().targetTemp);
    }
  }

  if (doc.containsKey("mode")) {
    const char *mode = doc["mode"].as<const char *>();
    if (mode != nullptr) {
      bool requestedManual = strcmp(mode, "manual") == 0;
      if (requestedManual != manualControlMode) {
        manualControlMode = requestedManual;
        telemetry.manualMode = manualControlMode;
        Serial.printf("[CTRL] Betriebsart gewechselt: %s\n", manualControlMode ? "Manuell" : "Automatik");
      }
    }
  }

  if (doc.containsKey("command")) {
    const char *cmd = doc["command"].as<const char *>();
    if (strcmp(cmd, "plus") == 0) {
      if (manualControlMode) {
        requestStageChange(+1, true);
      }
    } else if (strcmp(cmd, "minus") == 0) {
      if (manualControlMode) {
        requestStageChange(-1, true);
      }
    }
  }

  request->send(204);
}

void setupWebServer() {
  server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest *request) { sendStatus(request); });

  server.on("/api/control", HTTP_POST, [](AsyncWebServerRequest *request) {}, nullptr,
            [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
              handleControlRequest(request, data, len, index, total);
            });

  server.on("/api/config", HTTP_GET, [](AsyncWebServerRequest *request) { sendConfig(request); });

  server.on("/api/config", HTTP_POST, [](AsyncWebServerRequest *request) {}, nullptr,
            [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
              handleConfigUpdate(request, data, len, index, total);
            });

  server.on("/api/wifi/reset", HTTP_POST, [](AsyncWebServerRequest *request) {
    request->send(204);
    resetWiFiSettingsAndReboot();
  });

  server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");

  server.onNotFound([](AsyncWebServerRequest *request) { request->send(404, "text/plain", "Not found"); });

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
  mirroredDisplayLines = composeVirtualDisplayLines();
  setupEncoderHardware();

  bool wifiConnected = autoConfigureWiFi(DEVICE_HOSTNAME, WIFI_AP_NAME, WIFI_AP_PASSWORD);
  if (!wifiConnected) {
    Serial.println("[WiFi] Starte Konfigurationsportal");
  }
  WiFi.setAutoReconnect(true);

  if (wifiConnected) {
    setupWebServer();
    webServerRunning = true;
  }

  Serial.println("[BOOT] Setup abgeschlossen");
}

void loop() {
  if (!webServerRunning && WiFi.status() == WL_CONNECTED) {
    setupWebServer();
    webServerRunning = true;
  }

  processRelayPulses();

  processPendingHeaterOnCommand();

  updateTelemetry();
  evaluatePhase();
  handleStartBoost();
  applyRegulation();
  processEncoder();
  processEncoderButton();
  updateEncoderEditingState();
  saveTargetIfDirty();
  updateDisplay();
}
