#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>
#include <LittleFS.h>
#include <cstring>

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

struct SensorAddressConfig {
  bool valid = false;
  uint8_t address[8] = {};
};

struct SensorConfig {
  SensorAddressConfig inside;
  SensorAddressConfig exhaust;
};

struct AppConfig {
  float targetTemp = 21.0f;
  RegulationConfig regulation;
  TelegramConfig telegram;
  CalibrationConfig calibration;
  SensorConfig sensors;
};

class ConfigManager {
public:
  explicit ConfigManager(const char *configPath = "/config.json", const char *defaultPath = "/config_default.json")
      : _configPath(configPath), _defaultPath(defaultPath) {}

  bool begin() { return load(); }

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

    auto sensors = doc["sensors"].as<JsonObject>();
    if (!sensors.isNull()) {
      loadSensorAddress(sensors["inside"], _config.sensors.inside);
      loadSensorAddress(sensors["exhaust"], _config.sensors.exhaust);
    } else {
      _config.sensors.inside.valid = false;
      _config.sensors.exhaust.valid = false;
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

    JsonObject sensors = doc.createNestedObject("sensors");
    if (_config.sensors.inside.valid) {
      sensors["inside"] = sensorAddressToString(_config.sensors.inside.address);
    } else {
      sensors["inside"] = nullptr;
    }
    if (_config.sensors.exhaust.valid) {
      sensors["exhaust"] = sensorAddressToString(_config.sensors.exhaust.address);
    } else {
      sensors["exhaust"] = nullptr;
    }

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
  static int hexValue(char c) {
    if (c >= '0' && c <= '9') {
      return c - '0';
    }
    if (c >= 'a' && c <= 'f') {
      return 10 + (c - 'a');
    }
    if (c >= 'A' && c <= 'F') {
      return 10 + (c - 'A');
    }
    return -1;
  }

  static bool parseSensorAddress(const char *text, uint8_t *out) {
    if (text == nullptr) {
      return false;
    }

    uint8_t buffer[8] = {};
    size_t index = 0;
    int highNibble = -1;

    for (const char *p = text; *p != '\0'; ++p) {
      if (*p == ':' || *p == ' ' || *p == '-') {
        continue;
      }
      int value = hexValue(*p);
      if (value < 0) {
        return false;
      }
      if (highNibble < 0) {
        highNibble = value;
      } else {
        if (index >= 8) {
          return false;
        }
        buffer[index++] = static_cast<uint8_t>((highNibble << 4) | value);
        highNibble = -1;
      }
    }

    if (highNibble >= 0 || index != 8) {
      return false;
    }

    memcpy(out, buffer, sizeof(buffer));
    return true;
  }

  static void loadSensorAddress(JsonVariantConst value, SensorAddressConfig &out) {
    const char *text = value.is<const char *>() ? value.as<const char *>() : nullptr;
    if (parseSensorAddress(text, out.address)) {
      out.valid = true;
    } else {
      out.valid = false;
    }
  }

  static String sensorAddressToString(const uint8_t *address) {
    char buffer[17];
    for (size_t i = 0; i < 8; ++i) {
      snprintf(buffer + i * 2, 3, "%02X", address[i]);
    }
    return String(buffer);
  }

  bool copyDefault() {
    if (LittleFS.exists(_defaultPath)) {
      return copyFile(_defaultPath, _configPath);
    }

    Serial.println("[CFG] Default config missing, writing embedded defaults");

    static constexpr const char FALLBACK_CONFIG[] = R"({
  "target_temp": 21.0,
  "regulation": {
    "delta_down": 1.5,
    "delta_up": 0.8,
    "delta_up_critical": 2.0,
    "min_step_interval": 180,
    "stabilize_duration": 480,
    "max_steps_per_quarter": 2,
    "exhaust_warn": 90,
    "exhaust_off": 120
  },
  "telegram": {
    "enabled": false,
    "token": "",
    "chat_id": ""
  },
  "calibration": {
    "acs_zero": 2.5,
    "acs_sensitivity": 0.1
  },
  "sensors": {
    "inside": null,
    "exhaust": null
  }
})";

    if (!writeTextFile(_configPath, FALLBACK_CONFIG)) {
      return false;
    }

    if (!LittleFS.exists(_defaultPath)) {
      if (!writeTextFile(_defaultPath, FALLBACK_CONFIG)) {
        Serial.println("[CFG] Failed to persist embedded defaults as config_default.json");
      }
    }

    return true;
  }

  static bool copyFile(const char *srcPath, const char *dstPath) {
    File src = LittleFS.open(srcPath, "r");
    if (!src) {
      Serial.printf("[CFG] Failed to open %s for reading\n", srcPath);
      return false;
    }

    File dst = LittleFS.open(dstPath, "w");
    if (!dst) {
      Serial.printf("[CFG] Failed to open %s for writing\n", dstPath);
      src.close();
      return false;
    }

    uint8_t buffer[128];
    while (size_t read = src.read(buffer, sizeof(buffer))) {
      if (dst.write(buffer, read) != read) {
        Serial.println("[CFG] Failed to copy default config");
        src.close();
        dst.close();
        return false;
      }
    }

    src.close();
    dst.close();
    return true;
  }

  static bool writeTextFile(const char *path, const char *contents) {
    File file = LittleFS.open(path, "w");
    if (!file) {
      Serial.printf("[CFG] Failed to open %s for writing\n", path);
      return false;
    }

    size_t length = std::strlen(contents);
    size_t written = file.write(reinterpret_cast<const uint8_t *>(contents), length);
    file.close();

    if (written != length) {
      Serial.printf("[CFG] Failed to write full contents to %s\n", path);
      return false;
    }

    return true;
  }

  const char *_configPath;
  const char *_defaultPath;
  AppConfig _config;
};
