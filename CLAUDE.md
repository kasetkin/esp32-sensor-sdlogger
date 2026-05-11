# ESP32 Sensor SD Logger — Claude Code Instructions

## Build system

This is an ESP-IDF project targeting the ESP32-C6. Use **ESP-IDF v6.0** (not v5.x).

Before any `idf.py` call, source the ESP-IDF v6.0 environment:

```bash
source ~/.espressif/release-v6.0/esp-idf/export.sh
```

## Device

The ESP32-C6 board is connected at `/dev/ttyACM0`.

## Compiling the test app

```bash
source ~/.espressif/release-v6.0/esp-idf/export.sh
cd test_app
idf.py build
```

## Flashing and running tests

`idf.py flash monitor` does NOT work — idf_monitor requires an interactive TTY.
Use the two-step approach instead:

**Step 1 — flash** (from `test_app/build/` directory):
```bash
source ~/.espressif/release-v6.0/esp-idf/export.sh
cd test_app/build
python -m esptool --chip esp32c6 -b 460800 -p /dev/ttyACM0 \
    --before default-reset --after hard-reset write-flash \
    --flash-mode dio --flash-size 2MB --flash-freq 80m "@flash_args"
```

**Step 2 — read serial output** (reset device, then collect Unity output):
```python
import serial, time

with serial.Serial('/dev/ttyACM0', 115200, timeout=1) as ser:
    ser.setDTR(False); ser.setRTS(True); time.sleep(0.2)
    ser.setRTS(False); ser.setDTR(True); time.sleep(0.5)
    deadline = time.time() + 60
    last_data = time.time()
    while time.time() < deadline:
        line = ser.readline()
        if line:
            print(line.decode('utf-8', errors='replace').rstrip(), flush=True)
            last_data = time.time()
            if 'Tests' in line.decode() and 'Failures' in line.decode():
                time.sleep(2)
                break
        elif time.time() - last_data > 10:
            break
```

Or as a one-liner shell command (build + flash):
```bash
source ~/.espressif/release-v6.0/esp-idf/export.sh && \
  cd test_app && idf.py build && \
  cd build && python -m esptool --chip esp32c6 -b 460800 -p /dev/ttyACM0 \
    --before default-reset --after hard-reset write-flash \
    --flash-mode dio --flash-size 2MB --flash-freq 80m "@flash_args"
```

## Compiling the main app

```bash
source ~/.espressif/release-v6.0/esp-idf/export.sh
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
