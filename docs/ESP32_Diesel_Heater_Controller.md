# ESP32 Diesel Heater Controller Documentation

## 1. Project Overview
The ESP32 Diesel Heater Controller provides an intelligent and network-connected replacement for the original handheld remote of a Sunster 8 kW (and similar) diesel air heater. An ESP32 DevKit drives relay outputs to emulate button presses, measures supply current and air temperatures, and exposes an interactive web-based interface, encoder control, and Telegram bot for remote operation. Autonomous regulation with safety supervision allows the heater to run unattended while protecting the installation from over-temperature and start faults.

## 2. Functional Summary
- **Platform**: ESP32 DevKit (WROOM-32)
- **Power**: 12 V automotive supply, fused and clamped, stepped down to 5 V
- **I/O**: five relay outputs, two DS18B20 temperature inputs, ACS712 current sensor, SSD1306 OLED, rotary encoder with push button, optional optocoupler feedback
- **Connectivity**: Wi-Fi with captive portal, asynchronous web UI, OTA firmware updates, configurable Telegram bot
- **Control Logic**: multi-state heater sequencing with smooth staging, current- and temperature-based safety checks, configurable regulation parameters

## 3. System Architecture
```mermaid
graph TD
    A[12 V Vehicle Supply] -->|5 A Fuse| B[TVS SMBJ12A]
    B --> C[LM2596 Step-Down 5 V]
    C --> D[ESP32 DevKit]
    C --> E[5x Relay Board]
    C --> F[ACS712 Current Sensor]
    D -->|GPIO18/19/21/22/23| E
    D -->|GPIO36| F
    D -->|GPIO14| G[DS18B20 Sensors]
    D -->|GPIO26/27 (I²C)| H[SSD1306 OLED]
    D -->|GPIO32/33/25| I[Rotary Encoder]
    D -->|GPIO34| J[Optocoupler Feedback]
    D -->|Wi-Fi| K[Web UI / OTA / Telegram]
```

### 3.1 Power Distribution and Protection
1. **Supply Path**: 12 V input → 5 A fuse → SMBJ12A TVS diode → LM2596 buck converter → regulated 5 V rail.
2. **Loads**: ESP32 module, relay board, and ACS712 share the 5 V rail; DS18B20 sensors and SSD1306 draw from the ESP32 3V3 regulator when required.
3. **Grounding**: Star-ground topology referencing the ESP32 ground to avoid relay switching noise. Place decoupling capacitors (100 nF) near the ACS712 output and analog reference node to improve ADC stability.

### 3.2 Signal Interfaces
| Function | GPIO | Electrical Notes |
| --- | --- | --- |
| Relay – ON | GPIO18 | Active-low relay coil input
| Relay – OFF | GPIO19 | Active-low relay coil input
| Relay – PLUS | GPIO21 | Stage increment impulse (~300 ms)
| Relay – MINUS | GPIO22 | Stage decrement impulse (~300 ms)
| Relay – Boost / Power Pulse | GPIO23 | Short toggle pulse
| ACS712 OUT | GPIO36 (ADC1_CH0) | 12-bit ADC, 0–3.3 V; midscale at ~2.5 V |
| DS18B20 Bus | GPIO14 | One-wire, 4.7 kΩ pull-up to 3V3 |
| OLED SDA/SCL | GPIO26 / GPIO27 | I²C, 0x3C address |
| Encoder CLK/DT/SW | GPIO32 / GPIO33 / GPIO25 | Quadrature inputs + push button |
| Optocoupler Feedback | GPIO34 | Optional input-only channel |

## 4. Bill of Materials (Core Components)
| Qty | Component | Specification | Notes |
| --- | --- | --- | --- |
| 1 | ESP32 DevKit (WROOM-32) | Wi-Fi / BLE MCU | Primary controller |
| 1 | LM2596 Step-Down Converter | 12 V → 5 V, adjustable | Power supply |
| 1 | Fuse + holder | 5 A blade or mini | Inline supply protection |
| 1 | TVS Diode | SMBJ12A | Surge suppression |
| 1 | 5-Channel Relay Board | 5 V coils, opto-isolated, low-active | Simulates remote buttons |
| 1 | ACS712 Current Sensor | 20 A model | Heater current monitoring |
| 2 | DS18B20 Temperature Sensors | Waterproof probes | Cabin and exhaust temp |
| 1 | SSD1306 OLED | 128×64, I²C | Status display |
| 1 | Rotary Encoder with Push Button | Quadrature output | Setpoint adjustment |
| 1 | PC817 Optocoupler | Optional | Stromstoß feedback |
| — | Passive Components | 4.7 kΩ pull-up, 100 nF decouplers | Sensor support |
| 1 | Enclosure / PCB | Automotive-rated | Mounting and wiring |

## 5. Firmware Structure
```
platformio.ini          → ESP32 build configuration
src/main.cpp            → Application entry point, state machine, networking
include/pins.h          → GPIO assignments
include/credentials.h   → Default Telegram credentials (fallback)
data/index.html         → Responsive web UI (status + controls)
data/style.css          → UI styling
data/app.js             → Web app logic (REST endpoints, dynamic updates)
data/config_default.json→ Default configuration seed
```

### 5.1 Configuration Files
- **/config.json** (LittleFS at runtime): persists Wi-Fi credentials, regulation thresholds, ACS712 calibration offsets, Telegram configuration, and other tunable parameters.
- **config_default.json**: copied to `/config.json` on first boot to provide safe defaults.

### 5.2 Networking and UI
- Asynchronous web server exposes live telemetry (temperatures, current, phase, stage) and controls (On, Off, Plus, Minus, setpoint slider).
- Captive portal via WiFiManager (`SunsterSetup` / `sunster123`) launches automatically if no known networks are available or on manual trigger from the web UI.
- OTA firmware updates served at `/update`.
- Telegram bot (optional) supports commands `/ein`, `/aus`, `/plus`, `/minus`, `/status`, `/set <temp>` once enabled and configured.

## 6. Control Logic
The regulation engine (“Smooth10”) interprets heater current and exhaust temperature to identify operating phases and drive the relay interface accordingly.

### 6.1 Operating States
| State | Detection Criteria | Behavior |
| --- | --- | --- |
| **AUS** | Current < 0.2 A | Standby; no relays active |
| **Vorglühen** | Current > 5 A, exhaust < 40 °C | Wait for ignition |
| **Heizen** | Current > 1 A, exhaust ≥ 40 °C | Active setpoint regulation |
| **Nachlauf** | Current ≈ 0.5 A, exhaust < 50 °C | Allow cooldown |
| **Fehlstart** | No transition within 90 s | Issue OFF command |

### 6.2 Regulation Strategy
- Increase stage if cabin temperature is below setpoint minus `Δ_down`.
- Decrease stage when cabin temperature is stable above setpoint for `stability_timer` seconds.
- Apply expedited down-regulation when temperature exceeds setpoint + `Δ_up2`.
- Enforce a minimum dwell (180 s default) between stage changes and limit to two steps per 15-minute window.
- Safety overrides reduce stage when exhaust temperature exceeds 90 °C and shut the heater down above 120 °C.

## 7. User Interfaces
### 7.1 OLED Display
Refreshes every 0.5 s showing:
- Cabin temperature, exhaust temperature, current draw
- Operating phase and stage (1–10)
- Setpoint temperature
- Wi-Fi signal strength (RSSI)
- Versorgungsstatus des Stromstoßrelais (mit Rückmeldung, falls angeschlossen)

### 7.2 Rotary Encoder
- Rotational input adjusts setpoint ±0.5 °C.
- Push button reserved for future functions (e.g., menu or manual reset).

### 7.3 Web Interface
- Responsive layout with live telemetry panels and control buttons.
- Parameter settings page for fine-tuning regulation and Telegram connectivity.
- Anzeige des tatsächlichen Heizversorgungszustands (inklusive Feedback-Hinweis, falls kein Stromstoßkontakt vorhanden ist).
- Button to trigger Wi-Fi reconfiguration (drops a flag file and reboots into captive portal mode).

### 7.4 Telegram Bot
- Optional remote control layer for start/stop and telemetry requests.
- Enabled through web UI with token and chat ID configuration stored in `/config.json`.

## 8. Safety Considerations
- Exhaust temperature monitoring for stage reduction and emergency shutdown.
- Start failure detection ensures the heater cannot hang in preheat indefinitely.
- Cooling run-on after shutdown to purge fuel and heat.
- Relay outputs are time-limited and debounced to mimic human button presses safely.
- Optionally integrate over-current protection via ACS712 thresholds.
- Optional Stromstoß feedback contact prevents double-triggering and keeps the supply state in sync with user commands.

## 9. Expansion Opportunities
- MQTT integration for smart-home platforms.
- Touchscreen interface (Nextion/TFT) as an alternative to the OLED.
- Real-time web graphs for temperature and current trends.
- Bluetooth-based setup assistant.
- SD card logging for long-term diagnostics.

## 10. Deployment Checklist
1. Assemble hardware with fused 12 V input, TVS diode, and LM2596 regulator.
2. Wire sensors, display, and encoder to the specified GPIOs; maintain star-grounding.
3. Flash firmware via PlatformIO; ensure `config_default.json` is present in LittleFS data.
4. Connect to `SunsterSetup` portal to configure Wi-Fi (or use existing credentials).
5. Verify web UI, OLED output, and encoder operation.
6. Observe heater phases and current draw during a test run to validate regulation thresholds.
7. Configure Telegram bot if remote control is required.
8. Document calibration values and store the enclosure in a vibration-resistant mount.

## 11. Revision History
| Version | Date | Notes |
| --- | --- | --- |
| 1.0 | 2024-03-XX | Initial hardware and firmware documentation extracted from project prompt |

---
*Prepared for integration into repository README or project handover packages.*
