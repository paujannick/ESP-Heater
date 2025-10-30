#pragma once

#include <Arduino.h>

// Relay outputs
static constexpr uint8_t RELAY_STROKE_PIN = 23;
static constexpr uint8_t RELAY_ON_PIN = 18;
static constexpr uint8_t RELAY_OFF_PIN = 19;
static constexpr uint8_t RELAY_PLUS_PIN = 21;
static constexpr uint8_t RELAY_MINUS_PIN = 22;

// Sensors
static constexpr uint8_t PIN_ACS712 = 36; // ADC1 channel 0
static constexpr uint8_t PIN_ONEWIRE = 14;
static constexpr uint8_t PIN_ENCODER_CLK = 32;
static constexpr uint8_t PIN_ENCODER_DT = 33;
static constexpr uint8_t PIN_ENCODER_SW = 25;
static constexpr uint8_t PIN_STROKE_SENSE = 34; // optional optocoupler input
static constexpr uint8_t STROKE_SENSE_ACTIVE_LEVEL = LOW; // optocoupler pulls to GND when active

// Display
static constexpr uint8_t PIN_OLED_SDA = 26;
static constexpr uint8_t PIN_OLED_SCL = 27;

static constexpr uint8_t OLED_I2C_ADDRESS = 0x3C;

// General
static constexpr uint8_t RELAY_ACTIVE_LEVEL = LOW; // low-active board
static constexpr uint16_t RELAY_PULSE_MS = 300;
