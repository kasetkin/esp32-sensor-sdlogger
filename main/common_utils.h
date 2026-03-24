#pragma once

#include "esp_err.h"

/// intended only for ESP32-C6 from SeeedStudio

void enableRf(const bool enableRf);
void enableExtAntenna(const bool enableExtAnt);
void enableUserLED(const bool enableLED);

/// configure wakeup timer
esp_err_t registerWakeupTimer(const uint32_t wakeupMicrosec);

/// sleep for wakeupMicrosec from above + ?10ms? using esp_light_sleep
void correctLightSleep();

esp_err_t initNvsFlash();

/// for loggertask code migration, because it was written for Arduino
unsigned long millisFromStart();
/// emulate code from RTC.h
uint64_t getValidTime();

// esp_err_t initI2C();
/// ESP tasks
// void sht3xTask(void *pvParameters);
// void light_sleep_ble_sensor(void *args);
