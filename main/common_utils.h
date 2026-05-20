#pragma once

#include <esp_err.h>
#include <string>

/// intended only for ESP32-C6 from SeeedStudio

void enableRf(const bool enableRf);
void enableExtAntenna(const bool enableExtAnt);
void enableUserLED(const bool enableLED);

/// configure wakeup timer
[[nodiscard("device won't wake on timer if unchecked")]]
esp_err_t registerWakeupTimer(const uint32_t wakeupMicrosec);

/// sleep for wakeupMicrosec from above + ?10ms? using esp_light_sleep
void correctLightSleep();

[[nodiscard("NVS unavailable if init failure ignored")]]
esp_err_t initNvsFlash();

/// for loggertask code migration, because it was written for Arduino
unsigned long millisFromStart();
/// emulate code from RTC.h
uint64_t getValidTime();

std::string intToStringWithZeros(const int value, const size_t numberOfDigits);

#include <charconv>

// Append any integer or float/double to `out` via to_chars — zero heap allocation.
// Buffer sizing (shortest round-trip, base 10):
//   uint64_t:    20 chars  (19 digits + sign)
//   double:      24 chars  (sign + 17 sig.digits + '.' + 'e' + sign + 3 exp.digits)
//   long double: 26 chars  (x86 80-bit, 18 sig.digits, exponent up to ±4932, 4 exp.digits)
// On ESP32-C6 long double == double (24 chars), but 28 = 26 + 2 keeps the template
// correct on any platform without wasting stack space.
template<typename T>
inline void appendNum(std::string& out, T value) noexcept {
    char buf[28];
    auto [ptr, ec] = std::to_chars(buf, buf + sizeof(buf), value);
    out.append(buf, ptr);
}

// Append zero-padded integer directly into `out`, replacing intToStringWithZeros at call sites.
void appendZeroPadded(std::string& out, int value, int width) noexcept;

// esp_err_t initI2C();
/// ESP tasks
// void sht3xTask(void *pvParameters);
// void light_sleep_ble_sensor(void *args);
