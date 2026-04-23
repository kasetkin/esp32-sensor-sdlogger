#include "sensorstask.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_log.h>

sht3x_t SensorsTask::m_sht3dev;

void SensorsTask::configureReadyEvent(SensorsReadyEvent readyEvent)
{
    m_readyEvent = readyEvent;
}

static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    static const char * TAG = "ADC-calibration";

    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

esp_err_t SensorsTask::initAdc()
{
    //-------------ADC1 Init---------------//
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
        .clk_src = ADC_DIGI_CLK_SRC_XTAL,
        .ulp_mode = ADC_ULP_MODE_DISABLE
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_2, &config));

    //-------------ADC1 Calibration Init---------------//

    bool do_calibration1_chan0 = adc_calibration_init(ADC_UNIT_1, ADC_CHANNEL_2, ADC_ATTEN_DB_0, &adc1_cali_chan0_handle);
    if (do_calibration1_chan0)
        return ESP_OK;
    else
        return ESP_FAIL;
}

esp_err_t SensorsTask::initI2C() 
{
    static const char * TAG = "sensors-init-i2c";
    const esp_err_t initErr = i2cdev_init();
    if (initErr != ESP_OK) {
        ESP_LOGE(TAG, "can not init I2C: %d err", initErr);
        return ESP_FAIL;
    }

    memset(&m_sht3dev, 0, sizeof(sht3x_t));
    const esp_err_t descriptorInitErr = sht3x_init_desc(&m_sht3dev, SHT3X_ADDR, SHT3X_I2C_PORT, I2C_MASTER_SDA, I2C_MASTER_SCL);
    if (descriptorInitErr != ESP_OK) {
        ESP_LOGE(TAG, "can not init I2C descriptor structure: %d err", descriptorInitErr);
        return ESP_FAIL;
    }

    const esp_err_t sensorInitErr = sht3x_init(&m_sht3dev);
    if (sensorInitErr != ESP_OK) {
        ESP_LOGE(TAG, "can not init SHT3X sensor via I2C: %d err", sensorInitErr);
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t SensorsTask::init()
{
    static const char * TAG = "sensors-init";
    ESP_LOGI(TAG, "init all sensors: start");
    const esp_err_t adcErr = initAdc();
    if (adcErr != ESP_OK)
        return adcErr;

    const esp_err_t i2cErr = initI2C();
    if (i2cErr != ESP_OK)
        return i2cErr;
    
    return ESP_OK;
}

int SensorsTask::readBatteryVoltageMilliV()
{
// #ifdef HAS_PMU
//     if (pmu_found && PMU) {
//         const int batteryPercent = PMU->getBatteryPercent(); /// 0 .. 100
//         const uint16_t batteryVoltage = PMU->getBattVoltage(); /// millivolt
//         message =
//             std::string("BATVOLT;") + std::to_string(batteryVoltage)
//             + std::string(";BATPERC;") + std::to_string(batteryPercent)
//             + std::string(";");
//     }
// #endif

    int adc_raw = 0;
    int voltage = 0;
    int32_t voltage_mean = 0;
    for (size_t i = 0; i < ADC_READS_COUNT; ++i) {
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_2, &adc_raw));
        // ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1, ADC_CHANNEL_2, adc_raw);
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan0_handle, adc_raw, &voltage));
        // ESP_LOGI(TAG, "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_1, ADC_CHANNEL_2, voltage);
        voltage_mean += voltage;
    }

    const int scaledVoltage = (voltage_mean * 4) / ADC_READS_COUNT; /// 4 because of ADC_12dB and 2 because of 200kOhm 2:1 divider 
    const double realVoltage = voltageDividerCoefficient * scaledVoltage;
    return static_cast<int>(realVoltage);
}

int SensorsTask::convertVoltageToPercent(int batteryVoltageMilliV)
{
    constexpr double VOLTAGE_DELTA = MAX_VOLTAGE - MIN_VOLTAGE;
    const double value = (batteryVoltageMilliV - MIN_VOLTAGE) / VOLTAGE_DELTA * 100.0f;
    return std::max<double>(0.0f, std::min<double>(100.0, value));
}

std::string SensorsTask::toTelemetryRoundedString(const float value)
{
    std::string fullString = std::to_string(value);
    const size_t dotPos = fullString.find('.');
    if (dotPos == std::string::npos)
        return fullString;

    const size_t newLenght = std::min(dotPos + static_cast<size_t>(4), fullString.size());
    fullString.resize(newLenght);
    return fullString;
}



void SensorsTask::executeTask()
{
    static const char * TAG = "sensors-task";
    while (true) {
        std::string message;

        const int batteryVoltageMilliV = readBatteryVoltageMilliV();
        const int batteryPercent = convertVoltageToPercent(batteryVoltageMilliV);

        message += std::string("BATVOLT;") + std::to_string(batteryVoltageMilliV) + std::string(";");
        message += std::string("BATPERC;") + std::to_string(batteryPercent) + std::string(";");

        
        float envTemperature = -275.0;
        float envHumidity = -1.0;
        const esp_err_t readError = sht3x_measure(&m_sht3dev, &envTemperature, &envHumidity);
        if (readError == ESP_OK) {
            ESP_LOGI(TAG, "SHT3x Sensor: %.2f °C, %.2f %%", envTemperature, envHumidity);

            message += std::string("TEMP;") + toTelemetryRoundedString(envTemperature) + std::string(";");
            message += std::string("HUMID;") + toTelemetryRoundedString(envHumidity) + std::string(";");

            // if (!std::isnan(barometric_pressure))
            //     result += std::string("PRESS;") + toTelemetryRoundedString(barometric_pressure) + std::string(";");
        } else {
            ESP_LOGE(TAG, "sensor read error: %d", readError);
            envTemperature = -275.0;
            envHumidity = -1.0;
        }        

        ESP_LOGI(TAG, "new sensor values: %s", message.c_str());

        m_readyEvent(batteryVoltageMilliV, batteryPercent, envTemperature, envHumidity, message);
        vTaskDelay(pdMS_TO_TICKS(SENSORS_PERIOD_MS));
    }
}