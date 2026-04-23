#pragma once

#include <memory>
#include <string>
#include "esp_err.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/gpio.h"
#include "loggertask.h"
#include <i2cdev.h>
#include <sht3x.h>

class SensorsTask
{
public:
    esp_err_t init();
    void executeTask();

    using SensorsReadyEvent = std::function<void(int batteryVoltage, int batteryPercent, float envTemperature, float envHumidity, const std::string &message)>;
    void configureReadyEvent(SensorsReadyEvent readyEvent);

private:
    /// BATTERY voltage sensor via ADC pin 
    static constexpr gpio_num_t VOLTAGE_PIN = GPIO_NUM_2;
    static constexpr double RESISTOR_GND_2_SENSOR = 4974; //5028;       //162500; //200000; // ~ 200 kOhm 
    static constexpr double RESISTOR_SENSOR_2_VBAT = 4967; //5020;      //162700; //200000;// ~ 200 kOhm 
    static constexpr double voltageDividerCoefficient = (RESISTOR_GND_2_SENSOR + RESISTOR_SENSOR_2_VBAT) / RESISTOR_GND_2_SENSOR;
    static constexpr double MAX_VOLTAGE = 4200; // V * 10^-3
    static constexpr double MIN_VOLTAGE = 3300; // V * 10^-3
    static constexpr unsigned long SENSORS_PERIOD_MS = 1 * 1000;
    static constexpr size_t ADC_READS_COUNT = 10;

    /// ENVIRONMENT sensor, SHT31 via I2C bus
    static constexpr uint8_t SHT3X_ADDR = SHT3X_I2C_ADDR_GND; // 0x44
    static constexpr gpio_num_t I2C_MASTER_SDA = GPIO_NUM_22;
    static constexpr gpio_num_t I2C_MASTER_SCL = GPIO_NUM_23;
    static constexpr i2c_port_t SHT3X_I2C_PORT = I2C_NUM_0;
    
    adc_oneshot_unit_handle_t adc1_handle = nullptr;
    adc_cali_handle_t adc1_cali_chan0_handle = nullptr;
    SensorsReadyEvent m_readyEvent;
    sht3x_t m_sht3dev;

    /// always 3 digits after '.'
    static std::string toTelemetryRoundedString(const float value);

    esp_err_t initAdc();
    esp_err_t initI2C(); 
    int readBatteryVoltageMilliV();
    int convertVoltageToPercent(int batteryVoltageMilliV);
};