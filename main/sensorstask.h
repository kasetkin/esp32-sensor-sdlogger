#pragma once

#include <memory>
#include "esp_err.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/gpio.h"
#include "loggertask.h"

class SensorsTask
{
public:
    esp_err_t setupLogger(std::shared_ptr<LoggerTask> logger);
    esp_err_t init();
    void executeTask();

private:
    static constexpr gpio_num_t VOLTAGE_PIN = GPIO_NUM_2;
    static const unsigned long SENSORS_PERIOD_MS = 1 * 1000;
    static constexpr double MAX_VOLTAGE = 4200;
    static constexpr double MIN_VOLTAGE = 3300;

    int readBatteryVoltageMilliV();
    int convertVoltageToPercent(int batteryVoltageMilliV);

    adc_oneshot_unit_handle_t adc1_handle = nullptr;
    adc_cali_handle_t adc1_cali_chan0_handle = nullptr;
    std::shared_ptr<LoggerTask> m_logger;
};