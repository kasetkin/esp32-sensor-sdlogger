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
| Nordic UART / SPP | Raw NMEA stream, Qstarz binary packets, full log stream |

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

## Architecture

Four FreeRTOS tasks run concurrently:

- **GPSTask** — reads and parses NMEA / Unicore PPP messages from the UM980
- **SensorsTask** — samples battery ADC and SHT3x every second
- **LoggerTask** — aggregates data and writes log entries to the SD card
- **BleSppServerTask** — serves the GATT server and pushes updates to BLE clients
