# ESP32 Sensor SD Logger — Claude Code Instructions

## Build system

This is an ESP-IDF project targeting the ESP32-C6. Use **ESP-IDF v6.0.1** (not v5.x).

Inside the devcontainer the ESP-IDF environment is configured automatically — no
manual `source export.sh` is needed.

**Outside devcontainer** (local install):
```bash
source ~/.espressif/v6.0.1/esp-idf/export.sh
```

## Device

The ESP32-C6 board is connected at `/dev/ttyACM0`.

> If `/dev/ttyACM0` does not exist on the host (no board connected or a different port), comment out the `--device=/dev/ttyACM0` line in `.devcontainer/devcontainer.json` before building or starting the container — Docker will refuse to start if the device is listed but absent.

## Compiling the test app

```bash
cd test_app
idf.py build
```

## Flashing and running tests

`idf.py flash monitor` does NOT work — idf_monitor requires an interactive TTY.
Use `flash_and_test.py` instead, which flashes via esptool then reads Unity output
over serial:

```bash
# build
cd test_app && idf.py build

# flash + capture Unity output
cd ..
python3 flash_and_test.py --end-marker "Failures"
```

Other useful flags: `--no-flash` (skip flashing, read serial only), `--timeout`,
`--idle-timeout`, `--port`, `--baud`. Run `python3 flash_and_test.py --help` for full
usage.

## Compiling the main app

```bash
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
