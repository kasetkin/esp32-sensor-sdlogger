#include "sensorstask.h"

#include <cmath>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_sleep.h>

#include "common_utils.h"

std::string SensorsValues::toTelemetryRoundedString(const float value)
{
    std::string fullString = std::to_string(value);
    const size_t dotPos = fullString.find('.');
    if (dotPos == std::string::npos)
        return fullString;

    const size_t newLenght = std::min(dotPos + static_cast<size_t>(4), fullString.size());
    fullString.resize(newLenght);
    return fullString;
}

std::string SensorsValues::toString() const
{
    std::string message;
    if (batteryVoltageMilliV > 0)
        message += std::string("BATVOLT;") + std::to_string(batteryVoltageMilliV) + std::string(";");

    if (batteryPercent > 0)
        message += std::string("BATPERC;") + std::to_string(batteryPercent) + std::string(";");

    if (!std::isnan(envTemperature))
        message += std::string("TEMP;") + toTelemetryRoundedString(envTemperature) + std::string(";");

    if (!std::isnan(envHumidity))
        message += std::string("HUMID;") + toTelemetryRoundedString(envHumidity) + std::string(";");

    if (!std::isnan(barometricPressure))
        message += std::string("PRESS;") + toTelemetryRoundedString(barometricPressure) + std::string(";");

    return message;
}

void SensorsTask::configureReadyEvent(SensorsReadyEvent readyEvent)
{
    m_readyEvent = readyEvent;
}

bool SensorsTask::adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
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
    /// should be the same in init config and calibretion!!!
    const adc_atten_t ADC_ATTENUATION = ADC_ATTEN_DB_6;

    //-------------ADC1 Init---------------//
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
        .clk_src = ADC_DIGI_CLK_SRC_XTAL,
        .ulp_mode = ADC_ULP_MODE_DISABLE
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTENUATION,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_2, &config));

    //-------------ADC1 Calibration Init---------------//

    bool do_calibration1_chan0 = adc_calibration_init(ADC_UNIT_1, ADC_CHANNEL_2, ADC_ATTENUATION, &adc1_cali_chan0_handle);
    if (do_calibration1_chan0)
        return ESP_OK;

    adc_oneshot_del_unit(adc1_handle);
    adc1_handle = nullptr;
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
        i2cdev_done();
        return ESP_FAIL;
    }

    const esp_err_t sensorInitErr = sht3x_init(&m_sht3dev);
    if (sensorInitErr != ESP_OK) {
        ESP_LOGE(TAG, "can not init SHT3X sensor via I2C: %d err", sensorInitErr);
        sht3x_free_desc(&m_sht3dev);
        i2cdev_done();
        return ESP_FAIL;
    }

    m_i2cInitialized = true;
    return ESP_OK;
}

esp_err_t SensorsTask::init()
{
    static const char * TAG = "sensors-init";
    ESP_LOGI(TAG, "init all sensors: start");

    deinitI2C();
    deinitAdc();

    const esp_err_t adcErr = initAdc();
    if (adcErr != ESP_OK)
        return adcErr;

    const esp_err_t i2cErr = initI2C();
    if (i2cErr != ESP_OK) {
        deinitAdc();
        return i2cErr;
    }

    const esp_err_t timerErr = registerWakeupTimer(LOW_POWER_SLEEP_TIMER_DURATION_US);
    if (timerErr != ESP_OK)
        return timerErr;

    return ESP_OK;
}

SensorsTask::~SensorsTask()
{
    deinitI2C();
    deinitAdc();
}

void SensorsTask::adc_calibration_deinit(adc_cali_handle_t handle)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_delete_scheme_curve_fitting(handle);
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_delete_scheme_line_fitting(handle);
#endif
}

void SensorsTask::deinitAdc()
{
    if (adc1_cali_chan0_handle) {
        adc_calibration_deinit(adc1_cali_chan0_handle);
        adc1_cali_chan0_handle = nullptr;
    }
    if (adc1_handle) {
        adc_oneshot_del_unit(adc1_handle);
        adc1_handle = nullptr;
    }
}

void SensorsTask::deinitI2C()
{
    if (!m_i2cInitialized)
        return;
    sht3x_free_desc(&m_sht3dev);
    i2cdev_done();
    m_i2cInitialized = false;
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
    static const char * TAG = "ADC-measure";

    int adc_raw = 0;
    int voltage = 0;
    int32_t voltage_mean = 0;
    for (size_t i = 0; i < ADC_READS_COUNT; ++i) {
        const esp_err_t adcReadError = adc_oneshot_read(adc1_handle, ADC_CHANNEL_2, &adc_raw);
        if (adcReadError != ESP_OK) {
            ESP_LOGE(TAG, "ADC reading error %d", adcReadError);
            return -1;
        }

        const esp_err_t calibrationErr = adc_cali_raw_to_voltage(adc1_cali_chan0_handle, adc_raw, &voltage);
        if (calibrationErr != ESP_OK) {
            ESP_LOGE(TAG, "ADC calibration error %d, raw value is %d", calibrationErr, adc_raw);
            return -1;
        }        
        // ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1, ADC_CHANNEL_2, adc_raw);
        // ESP_LOGI(TAG, "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_1, ADC_CHANNEL_2, voltage);
        voltage_mean += voltage;
    }

    const int scaledVoltage = voltage_mean / ADC_READS_COUNT;
    ESP_LOGI(TAG, "ADC pin voltage: %d ", scaledVoltage);

    const double realVoltage = voltageDividerCoefficient * scaledVoltage;
    return static_cast<int>(realVoltage);
}

esp_err_t SensorsTask::readEnvironment(float &temperature, float &humidity)
{
    return sht3x_measure(&m_sht3dev, &temperature, &humidity);
}

int SensorsTask::convertVoltageToPercent(int batteryVoltageMilliV)
{
    constexpr double VOLTAGE_DELTA = MAX_VOLTAGE - MIN_VOLTAGE;
    const double value = (batteryVoltageMilliV - MIN_VOLTAGE) / VOLTAGE_DELTA * 100.0f;
    return std::max<double>(0.0f, std::min<double>(100.0, value));
}

void SensorsTask::executeTask()
{
    static const char * TAG = "sensors-task";
    while (true) {
        SensorsValues v;
        v.batteryVoltageMilliV = readBatteryVoltageMilliV();
        if (v.batteryVoltageMilliV > 0) {
            v.batteryPercent = convertVoltageToPercent(v.batteryVoltageMilliV);
            if (v.batteryVoltageMilliV < LOW_DISCHARGE_VOLTAGE) {
                ESP_LOGE(TAG, "battery voltage too low (%d), sleep", v.batteryVoltageMilliV);
                correctLightSleep();

                /// \todo find way to disable peripherals, maybe add N-type MOSFET (like AO3400A) between 3V3 pin and devices 

                continue;
            }
        } else {
            ESP_LOGE(TAG, "battery ADC read error");
        }

        const esp_err_t readError = sht3x_measure(&m_sht3dev, &v.envTemperature, &v.envHumidity);
        if (readError == ESP_OK) {
            ESP_LOGI(TAG, "SHT3x Sensor: %.2f °C, %.2f %%", v.envTemperature, v.envHumidity);
        } else {
            ESP_LOGE(TAG, "sensor read error: %d", readError);
            v.envTemperature = std::numeric_limits<float>::quiet_NaN();
            v.envHumidity = std::numeric_limits<float>::quiet_NaN();
        }        

        m_readyEvent(v);
        vTaskDelay(pdMS_TO_TICKS(SENSORS_PERIOD_MS));
    }
}