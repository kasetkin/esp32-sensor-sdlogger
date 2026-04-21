#include "sensorstask.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

esp_err_t SensorsTask::setupLogger(std::shared_ptr<LoggerTask> logger)
{
    if (!logger)
        return ESP_FAIL;

    m_logger = logger;
    return ESP_OK;
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

esp_err_t SensorsTask::init()
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

int SensorsTask::readBatteryVoltageMilliV()
{
    // static const char * TAG = "ADC-read";

    // int32_t adc_raw_mean = 0;
    
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

    const int realVoltage = (voltage_mean * 4 * 2) / ADC_READS_COUNT; /// 4 because of ADC_12dB and 2 because of 200kOhm 2:1 divider 
    return realVoltage;
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
        const int batteryVoltageMilliV = readBatteryVoltageMilliV();
        const int batteryPercent = convertVoltageToPercent(batteryVoltageMilliV);
        std::string message =
            std::string("BATVOLT;") + std::to_string(batteryVoltageMilliV)
            + std::string(";BATPERC;") + std::to_string(batteryPercent)
            + std::string(";");

        ESP_LOGI(TAG, "new sensor values: %s", message.c_str());
        if (m_logger)
            m_logger->setSensorsLog(message);

        vTaskDelay(pdMS_TO_TICKS(SENSORS_PERIOD_MS));
    }
}