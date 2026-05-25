# ESP32 GNSS Sensor Logger

A DIY data logger for the ESP32 that records high-precision GNSS position, temperature, humidity, and battery level to an SD card while streaming live telemetry over Bluetooth Low Energy (BLE).

## What it does

- Reads GNSS data from a **Unicore UM980** receiver (multi-band GPS/GLONASS/Galileo/BeiDou) over UART
- Reads **temperature and humidity** from an SHT3x sensor over I2C
- Monitors **battery voltage** via ADC
- Writes timestamped log entries to an **SD card** (FAT32, SPI)
- Streams all data in real time over **BLE** to a connected phone or tablet
- Advertises as `QSTARZ_EMULATOR` for compatibility with the [Bluetooth GNSS](https://github.com/ykasidit/bluetooth_gnss) Android app (tested against v2.1.6) using the Qstarz binary packet format
- Supports both standard NMEA and **Unicore PPP/RTK** positioning output

## BLE services exposed

| Service | Characteristics |
|---|---|
| Battery Service (0x180F) | Battery level % |
| Environmental Sensing (0x181A) | Temperature, Humidity |
| Nordic UART / SPP | Raw NMEA stream, Qstarz binary packets, full log stream, command input |

### Sending commands over BLE

Write a UTF-8 string to the **TX characteristic** (`6E400003-B5A3-F393-E0A9-E50E24DCCA9E`) of the Nordic UART service. The device strips trailing `\r\n` and dispatches the command.

#### Built-in commands

| Command | Effect |
|---|---|
| `led=on` | Turns the user LED on |
| `led=off` | Turns the user LED off |
| `wifi=enable` | *(reserved — not yet implemented)* |
| `wifi=disable` | *(reserved — not yet implemented)* |

#### Forwarding commands to the UM980

Any unrecognised command is forwarded verbatim to the UM980 GNSS module over UART with `\r\n` appended. This allows sending standard Unicore configuration commands directly from a BLE terminal app, for example:

```
FRESET
CONFIG SIGNALGROUP 2
PPPNAV 0.1
```

> **Tip:** [nRF Connect](https://www.nordicsemi.com/Products/Development-tools/nRF-Connect-for-mobile) and [Serial Bluetooth Terminal](https://play.google.com/store/apps/details?id=de.kai_morich.serial_bluetooth_terminal) both support writing to GATT characteristics and work well for this.

## Hardware

| Component | Interface |
|---|---|
| ESP32-C6 | — |
| Unicore UM980 GNSS module | UART1 (GPIO16/17), 115200 baud |
| SHT3x temperature/humidity sensor | I2C (GPIO22/23, addr 0x44) |
| SD card | SPI (GPIO18–21, FAT32) |
| 2:1 voltage divider (2×5.1 kΩ) | ADC on GPIO2 (3.3–4.2 V Li-ion battery) |

## Development environment

The project ships with a **Dev Container** configuration (`espressif/idf:v6.0.1`). With VS Code and the [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extension installed, open the repo and choose **Reopen in Container** — the ESP-IDF toolchain is configured automatically.

> **No board connected?** If `/dev/ttyACM0` does not exist on your host (board unplugged or on a different port), comment out the `--device=/dev/ttyACM0` line in `.devcontainer/devcontainer.json` before building or starting the container, otherwise Docker will refuse to start.

Without the devcontainer (local ESP-IDF install):
```sh
source ~/.espressif/v6.0.1/esp-idf/export.sh
```

## Build

```sh
# main firmware
idf.py build

# test app
cd test_app && idf.py build
```

## Flashing and testing

`idf.py flash monitor` does not work without an interactive TTY. Use `flash_and_test.py` instead — it flashes via esptool and reads serial output:

```sh
# flash and capture Unity test output
python3 flash_and_test.py --end-marker "Failures"
```

Useful flags: `--no-flash` (read serial only), `--port`, `--baud`, `--timeout`, `--idle-timeout`. Run `python3 flash_and_test.py --help` for full usage.

## Dependencies

- **ESP-IDF v6.0.1** — do not use v5.x; the project requires v6.0.1 APIs
- **TinyGPSPlus** (git submodule) — a [forked version](https://github.com/kasetkin/TinyGPSPlus) that extends the original library with support for Unicore proprietary NMEA sentences used for PPP/RTK positioning

## Architecture

Five FreeRTOS tasks run concurrently on ESP-IDF v6.0.1, written in C++26:

- **GPSTask** — reads and parses NMEA / Unicore PPP messages from the UM980
- **SensorsTask** — samples battery ADC and SHT3x every second
- **LoggerTask** — aggregates data and writes log entries to the SD card
- **BleSppServerTask** — serves the GATT server and pushes updates to BLE clients
- **ErrorTask** — signals startup errors via the user LED (SOS blink pattern)
