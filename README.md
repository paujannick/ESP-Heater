# ESP Heater Controller

This repository contains the PlatformIO-based firmware, web assets, configuration templates, and documentation for an ESP32-based controller that automates and remotely operates Sunster-class 8 kW diesel air heaters.

## Documentation
- [Comprehensive project documentation](docs/ESP32_Diesel_Heater_Controller.md)
- [Pinbelegung (Kurzreferenz)](docs/Pinbelegung.md)

## Repository Structure
- `platformio.ini` – PlatformIO environment definition with required libraries
- `src/`, `include/` – firmware sources (state machine, peripherals, configuration headers)
- `data/` – LittleFS assets served by the web interface (HTML, CSS, JavaScript, default config)
- `docs/` – design documentation and handover material

## Getting Started
1. Install [PlatformIO](https://platformio.org/install) and clone this repository.
2. Adjust `data/config_default.json` if you want different default regulation, calibration, or Telegram values.
3. Build and upload the filesystem and firmware: `pio run --target uploadfs` followed by `pio run --target upload`.
4. After the ESP32 boots it will expose the captive portal `SunsterSetup` / `sunster123` for Wi-Fi provisioning.
5. Open the assigned IP address in a browser to control the heater and monitor sensor data, or use the integrated Telegram bot once configured.

> ℹ️ **Stromstoßrelais** – Falls der optionale Rückmeldekontakt des Stromstoßrelais über den Optokoppler an GPIO34 angeschlossen wird,
> erkennt die Firmware den tatsächlichen Versorgungszustand der Heizung automatisch. Die Steuerung löst nur dann weitere Impulse aus,
> wenn die Rückmeldung nicht zur gewünschten Anforderung passt; ohne Rückmeldekontakt verhält sie sich wie gewohnt.

> 🔄 **Automatischer Start-Boost** – Nach dem Einschalten wartet die Steuerung etwa zwei Minuten und sendet anschließend zehn PLUS-Impulse,
> sodass die Heizung sicher auf Stufe 10 startet. Danach übernimmt die Regelung wieder normal und passt die Stufe bedarfsgerecht an.

## License
Specify your preferred license terms here.
