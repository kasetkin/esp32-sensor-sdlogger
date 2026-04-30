# ESP32 Sensor SD Logger — Claude Code Instructions

## Build system

This is an ESP-IDF project targeting the ESP32-C6. Use **ESP-IDF v6.0** (not v5.x).

Before any `idf.py` call, source the ESP-IDF v6.0 environment:

```bash
source /home/kasetkin/.espressif/release-v6.0/esp-idf/export.sh
```

## Compiling the test app

```bash
source /home/kasetkin/.espressif/release-v6.0/esp-idf/export.sh
cd test_app
idf.py build
```

## Compiling the main app

```bash
source /home/kasetkin/.espressif/release-v6.0/esp-idf/export.sh
idf.py build
```

## Project layout

| Path | Purpose |
|------|---------|
| `main/` | Main firmware sources (tasks, sensors, GPS, SD, BLE) |
| `test_app/main/` | Unity-based hardware test suite |
| `components/i2cdev/` | I2C master driver (esp-idf-lib) |
| `components/sht3x/` | SHT3x temperature/humidity sensor driver |
| `modules/TinyGPSPlus/` | GPS NMEA parser |
