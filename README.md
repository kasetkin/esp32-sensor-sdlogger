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
| ESP32 | — |
| Unicore UM980 GNSS module | UART1 (GPIO16/17), 115200 baud |
| SHT3x temperature/humidity sensor | I2C (addr 0x44) |
| SD card | SPI (FAT32) |
| 2:1 voltage divider (2×200 kΩ) | ADC on GPIO2 (3.3–4.2 V Li-ion battery) |

## Build

Requires **ESP-IDF**. Clone with submodules, then:

```sh
idf.py build
idf.py -p PORT flash monitor
```

## Dependencies

Unicore PPPNAV message parsing is handled by a [forked version of TinyGPSPlus](https://github.com/kasetkin/TinyGPSPlus) (included as a git submodule) that extends the original library with support for Unicore proprietary NMEA sentences used for PPP/RTK positioning.

## Architecture

Four FreeRTOS tasks run concurrently:

- **GPSTask** — reads and parses NMEA / Unicore PPP messages from the UM980
- **SensorsTask** — samples battery ADC and SHT3x every second
- **LoggerTask** — aggregates data and writes log entries to the SD card
- **BleSppServerTask** — serves the GATT server and pushes updates to BLE clients
