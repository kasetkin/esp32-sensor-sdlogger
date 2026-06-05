#pragma once

#include <cmath>
#include <numeric>
#include <optional>
#include <expected>
#include <memory>
#include <string>
#include <functional>
#include <esp_err.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include <driver/gpio.h>
#include "i2cdev.h"
#include "sht3x.h"

struct SensorsValues
{
public:
    std::optional<int> batteryVoltageMilliV;
    std::optional<int> batteryPercent;
    std::optional<float> envTemperature;
    std::optional<float> envHumidity;
    std::optional<float> barometricPressure;

    std::string toTelemetryString() const;
    std::string toLogString() const;
    /// up to 3 decimal digits; trailing zeros and dot stripped
    static std::string toTelemetryRoundedString(const float value);
};

class SensorsTask
{
public:
    [[nodiscard("sensors unavailable if init failure ignored")]]
    esp_err_t init(bool readAdc);
    ~SensorsTask();
    void executeTask();
    [[nodiscard("output params undefined on failure")]]
    esp_err_t readEnvironment(float &temperature, float &humidity);

    using SensorsReadyEvent = std::function<void(const SensorsValues &values)>;
    void configureReadyEvent(SensorsReadyEvent readyEvent);

    static int convertVoltageToPercent(int batteryVoltageMilliV);

    SensorsTask() = default;
    SensorsTask(const SensorsTask &) = delete("SensorsTask owns I2C device handles — copying aliases hardware resources");
    SensorsTask &operator=(const SensorsTask &) = delete("SensorsTask owns I2C device handles — copying aliases hardware resources");

    static constexpr double MAX_VOLTAGE = 4090.0; // mV — fully charged Li-ion (measured)
    static constexpr double MIN_VOLTAGE = 3200.0; // mV — empty (0 %)
    static constexpr int LOW_DISCHARGE_VOLTAGE = 3150; // mV — deep-discharge sleep threshold

private:
    /// BATTERY voltage sensor via ADC pin
    static constexpr gpio_num_t VOLTAGE_PIN = GPIO_NUM_2;
    static constexpr double RESISTOR_GND_2_SENSOR = 4974;   // ~5.1 kΩ
    static constexpr double RESISTOR_SENSOR_2_VBAT = 4967;  // ~5.1 kΩ
    static constexpr double voltageDividerCoefficient = (RESISTOR_GND_2_SENSOR + RESISTOR_SENSOR_2_VBAT) / RESISTOR_GND_2_SENSOR;
    static constexpr uint32_t SENSORS_PERIOD_MS = 1 * 1000;
    static constexpr uint32_t LOW_POWER_SLEEP_TIMER_DURATION_US = 5 * 1000 * 1000; 
    static constexpr size_t ADC_READS_COUNT = 10;

    /// ENVIRONMENT sensor, SHT31 via I2C bus
    static constexpr uint8_t SHT3X_ADDR = SHT3X_I2C_ADDR_GND; // 0x44
    static constexpr gpio_num_t I2C_MASTER_SDA = GPIO_NUM_22;
    static constexpr gpio_num_t I2C_MASTER_SCL = GPIO_NUM_23;
    static constexpr i2c_port_t SHT3X_I2C_PORT = I2C_NUM_0;
    
    bool m_readAdc = true;
    adc_oneshot_unit_handle_t adc1_handle = nullptr;
    adc_cali_handle_t adc1_cali_chan0_handle = nullptr;
    bool m_i2cInitialized = false;
    SensorsReadyEvent m_readyEvent;
    sht3x_t m_sht3dev;

    [[nodiscard("false means ADC is uncalibrated")]]
    static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
    static void adc_calibration_deinit(adc_cali_handle_t handle);

    [[nodiscard("ADC unavailable if init failure ignored")]]
    esp_err_t initAdc();
    [[nodiscard("I2C unavailable if init failure ignored")]]
    esp_err_t initI2C();
    void deinitAdc();
    void deinitI2C();
    std::expected<int, esp_err_t> readBatteryVoltageMilliV();
};