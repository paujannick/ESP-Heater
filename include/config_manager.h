#pragma once

#include <Arduino.h>
#include <LittleFS.h>
#include <ArduinoJson.h>

struct RegulationConfig {
  float deltaDown = 1.5f;
  float deltaUp = 0.8f;
  float deltaUpCritical = 2.0f;
  uint32_t minStepInterval = 180; // seconds
  uint32_t stabilizeDuration = 480; // seconds
  uint8_t maxStepsPerQuarter = 2; // per 15 minutes
  float exhaustWarn = 90.0f;
  float exhaustOff = 120.0f;
};

struct TelegramConfig {
  bool enabled = false;
  String token;
  String chatId;
};

struct CalibrationConfig {
  float acsZero = 2.5f;
  float acsSensitivity = 0.1f; // V/A
};

struct AppConfig {
  float targetTemp = 21.0f;
  RegulationConfig regulation;
  TelegramConfig telegram;
  CalibrationConfig calibration;
};

class ConfigManager {
public:
  explicit ConfigManager(const char *configPath = "/config.json", const char *defaultPath = "/config_default.json")
      : _configPath(configPath), _defaultPath(defaultPath) {}

  bool begin() {
    if (!LittleFS.begin(true)) {
      Serial.println("[CFG] Failed to mount LittleFS");
      return false;
    }
    return load();
  }

  AppConfig &config() { return _config; }

  bool load() {
    if (!LittleFS.exists(_configPath)) {
      Serial.println("[CFG] Config not found, copying defaults");
      if (!copyDefault()) {
        return false;
      }
    }

    File file = LittleFS.open(_configPath, "r");
    if (!file) {
      Serial.println("[CFG] Failed to open config file");
      return false;
    }

    StaticJsonDocument<1024> doc;
    auto error = deserializeJson(doc, file);
    file.close();
    if (error) {
      Serial.printf("[CFG] JSON parse failed: %s\n", error.c_str());
      return false;
    }

    _config.targetTemp = doc["target_temp"].as<float>();

    auto reg = doc["regulation"].as<JsonObject>();
    if (!reg.isNull()) {
      _config.regulation.deltaDown = reg["delta_down"].as<float>();
      _config.regulation.deltaUp = reg["delta_up"].as<float>();
      _config.regulation.deltaUpCritical = reg["delta_up_critical"].as<float>();
      _config.regulation.minStepInterval = reg["min_step_interval"].as<uint32_t>();
      _config.regulation.stabilizeDuration = reg["stabilize_duration"].as<uint32_t>();
      _config.regulation.maxStepsPerQuarter = reg["max_steps_per_quarter"].as<uint8_t>();
      _config.regulation.exhaustWarn = reg["exhaust_warn"].as<float>();
      _config.regulation.exhaustOff = reg["exhaust_off"].as<float>();
    }

    auto telegram = doc["telegram"].as<JsonObject>();
    if (!telegram.isNull()) {
      _config.telegram.enabled = telegram["enabled"].as<bool>();
      _config.telegram.token = telegram["token"].as<const char *>();
      _config.telegram.chatId = telegram["chat_id"].as<const char *>();
    }

    auto calib = doc["calibration"].as<JsonObject>();
    if (!calib.isNull()) {
      _config.calibration.acsZero = calib["acs_zero"].as<float>();
      _config.calibration.acsSensitivity = calib["acs_sensitivity"].as<float>();
    }

    return true;
  }

  bool save() {
    StaticJsonDocument<1024> doc;
    doc["target_temp"] = _config.targetTemp;

    JsonObject reg = doc.createNestedObject("regulation");
    reg["delta_down"] = _config.regulation.deltaDown;
    reg["delta_up"] = _config.regulation.deltaUp;
    reg["delta_up_critical"] = _config.regulation.deltaUpCritical;
    reg["min_step_interval"] = _config.regulation.minStepInterval;
    reg["stabilize_duration"] = _config.regulation.stabilizeDuration;
    reg["max_steps_per_quarter"] = _config.regulation.maxStepsPerQuarter;
    reg["exhaust_warn"] = _config.regulation.exhaustWarn;
    reg["exhaust_off"] = _config.regulation.exhaustOff;

    JsonObject telegram = doc.createNestedObject("telegram");
    telegram["enabled"] = _config.telegram.enabled;
    telegram["token"] = _config.telegram.token;
    telegram["chat_id"] = _config.telegram.chatId;

    JsonObject calib = doc.createNestedObject("calibration");
    calib["acs_zero"] = _config.calibration.acsZero;
    calib["acs_sensitivity"] = _config.calibration.acsSensitivity;

    File file = LittleFS.open(_configPath, "w");
    if (!file) {
      Serial.println("[CFG] Failed to open config for writing");
      return false;
    }

    serializeJsonPretty(doc, file);
    file.close();
    return true;
  }

  bool reset() {
    if (LittleFS.exists(_configPath)) {
      LittleFS.remove(_configPath);
    }
    return copyDefault() && load();
  }

private:
  bool copyDefault() {
    if (!LittleFS.exists(_defaultPath)) {
      Serial.println("[CFG] Default config missing");
      return false;
    }

    File src = LittleFS.open(_defaultPath, "r");
    File dst = LittleFS.open(_configPath, "w");
    if (!src || !dst) {
      Serial.println("[CFG] Failed to copy default config");
      return false;
    }

    while (src.available()) {
      dst.write(src.read());
    }
    src.close();
    dst.close();
    return true;
  }

  const char *_configPath;
  const char *_defaultPath;
  AppConfig _config;
};
