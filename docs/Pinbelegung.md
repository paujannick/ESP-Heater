# Pinbelegung ESP32 Diesel-Heizung

Diese Übersicht fasst alle aktuell verwendeten GPIOs der Firmware zusammen. Sie dient als schnelle Referenz beim Aufbau, Debuggen oder Erweitern der Steuerung.

| Funktion | GPIO | Richtung | Hinweise |
| --- | --- | --- | --- |
| Relais Stromstoß (Energieversorgung) | 23 | Ausgang | Low-aktiver Impuls (~300 ms) zum Einschalten des Stromstoßrelais |
| Relais EIN | 18 | Ausgang | Simuliert die EIN-Taste der Fernbedienung |
| Relais AUS | 19 | Ausgang | Simuliert die AUS-Taste der Fernbedienung |
| Relais PLUS | 21 | Ausgang | Erhöht die Heizstufe |
| Relais MINUS | 22 | Ausgang | Verringert die Heizstufe |
| Stromsensor ACS712 | 36 | Eingang (ADC1_CH0) | Analoge Messung, 0–3,3 V; Kalibrierung über Konfiguration |
| 1-Wire Bus DS18B20 | 14 | I/O | Gemeinsamer Bus mit 4,7 kΩ Pull-up nach 3,3 V |
| OLED SDA | 26 | Ausgang (I²C) | Gemeinsame I²C-Leitung für Display |
| OLED SCL | 27 | Ausgang (I²C) | Gemeinsame I²C-Leitung für Display |
| Drehencoder CLK | 32 | Eingang | Benötigt Pull-up, wird intern aktiviert |
| Drehencoder DT | 33 | Eingang | Benötigt Pull-up, wird intern aktiviert |
| Drehencoder Taster | 25 | Eingang | Aktiv LOW; langer Druck öffnet das WLAN-Setup |
| Stromstoß-Rückmeldung (Optokoppler) | 34 | Eingang | Optional; aktiv LOW; ermöglicht sichere Erkennung der Versorgung |

Weitere feste Signale:

- I²C-Adresse des OLED-Displays: `0x3C`
- Relais-Aktivlevel: LOW (die Ausgänge ziehen die optogekoppelten Relais nach GND)
- Pulsdauer der Relais: 300 ms pro Ansteuerung

Die Tabelle entspricht ebenfalls der Definition in `include/pins.h` und wird automatisch von der Firmware verwendet. Bei Änderungen an der Hardware sollten beide Stellen angepasst werden.
