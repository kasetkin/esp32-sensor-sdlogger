#include <memory>
#include <string>
#include "esp_err.h"
#include "esp_log.h"

#include <sys/stat.h>

#include "main.h"
#include "common_utils.h"
#include "sdcard.h"
#include "gpstask.h"
#include "sensorstask.h"
#include "errortask.h"
#include "bleservertask.h"
#include "loggertask.h"

static const char *TAG = "main-body";
const uint32_t DEFAULT_TASK_STACK_SIZE = 16384;
#define EXAMPLE_MAX_CHAR_SIZE    64

static std::shared_ptr<GpsTask> gpsTask;
static std::shared_ptr<SensorsTask> sensorTask;
static std::shared_ptr<LoggerTask> loggerTask;
static std::shared_ptr<ErrorTask> errorTask;
static std::shared_ptr<BleSppServerTask> bleTask;

void startErrorTask(ErrorTask::ErrorCode code)
{
    errorTask = std::make_shared<ErrorTask>(code);
    xTaskCreate([](void *)
    { 
        errorTask->execute();
        vTaskDelete(nullptr);
    }, "error_task", DEFAULT_TASK_STACK_SIZE, nullptr, 6, nullptr);
}

extern "C" void app_main(void)
{
    esp_err_t ret;

    ret = initNvsFlash();
    if (ret != ESP_OK)
        return;

    enableRf(true);
    enableExtAntenna(false);
    if (const esp_err_t timerErr = registerWakeupTimer(100 * 1000); timerErr != ESP_OK)
        ESP_LOGE(TAG, "registerWakeupTimer failed: %d", timerErr);

    ESP_LOGI(TAG, "create GPS task object");
    gpsTask = std::make_shared<GpsTask>();

    ESP_LOGI(TAG, "create logger task object");
    loggerTask = std::make_shared<LoggerTask>();

    ESP_LOGI(TAG, "create sensors task");
    sensorTask = std::make_shared<SensorsTask>();

    ESP_LOGI(TAG, "create BLE task");
    bleTask = std::make_shared<BleSppServerTask>();
    ESP_LOGI(TAG, "start BLE task");
    bleTask->startServer();

    ESP_LOGI(TAG, "configure GPS module UART");
    ret = gpsTask->configureUart();
    if (ret != ESP_OK) {
        startErrorTask(ErrorTask::ErrorCode::ecGpsUartFail);
        return;
    }

    ESP_LOGI(TAG, "configure TinyGPS");
    ret = gpsTask->configureTinyGps();
    if (ret != ESP_OK) {
        startErrorTask(ErrorTask::ErrorCode::ecTinyGpsFail);
        return;
    }

    ESP_LOGI(TAG, "configure GPS module settings");
    ret = gpsTask->configureUM980();
    if (ret != ESP_OK) {
        startErrorTask(ErrorTask::ErrorCode::ecUM980Fail);
        return;
    }

    ESP_LOGI(TAG, "configure sensors");
    ret = sensorTask->init();
    if (ret != ESP_OK) {
        startErrorTask(ErrorTask::ErrorCode::ecSensorsFail);
        return;
    }

    ESP_LOGI(TAG, "init SD card");
    std::shared_ptr<SdCard> my_sdcard = std::make_shared<SdCard>();
    ret = my_sdcard->mountFilesystem();
    if (ret != ESP_OK) {
        startErrorTask(ErrorTask::ErrorCode::ecSdCardFilesystemFail);
        return;
    }

    my_sdcard->printInfoToStdout();
    const bool cardOk = my_sdcard->cardIsMounted();
    if (!cardOk) {
        startErrorTask(ErrorTask::ErrorCode::ecSdCardFilesystemFail);
        return;
    }

    ESP_LOGI(TAG, "pass SdCard to Logger");
    loggerTask->configureSdCard(my_sdcard);

    ESP_LOGI(TAG, "configure Logger::ReadyEvent (pass log string to BLE task)");
    loggerTask->configureLogReadyEvent([](std::string_view log)
    {
        if (!bleTask)
            return;

        bleTask->appendLog(log);
        bleTask->appendLog("\r\n");
    });
    
    ESP_LOGI(TAG, "configure GpsTask events - begin");
    gpsTask->configureNmeaEvent([](std::string_view nmea)
    {
        if (bleTask)
            bleTask->appendNmea(nmea);

        if (loggerTask)
            loggerTask->addNmeaLog(nmea);
    });
    gpsTask->configureGnssEvent([](std::string_view gnss)
    {
        if (loggerTask)
            loggerTask->setGpsLog(gnss);
    });
    gpsTask->configurePppEvent([](std::string_view ppp)
    {
        if (loggerTask)
            loggerTask->setGpsLog(ppp);
    });
    gpsTask->configureQStarZEvent([](const GpsTask::QStarZPackets &packets)
    {
        if (bleTask)
            bleTask->transmitQstarzPackets(packets);
    });
    ESP_LOGI(TAG, "configure GpsTask events - end");

    ESP_LOGI(TAG, "configure Sensors::ReadyEvent (pass battery, temp, humidity to BLE task)");
    sensorTask->configureReadyEvent([](const SensorsValues &values)
    {
        ESP_LOGI(TAG, "Logger: ReadyEnevt");
        const std::string message = values.toTelemetryString();
        if (bleTask) {
            ESP_LOGI(TAG, "Logger: ReadyEnevt: %s", values.toLogString().c_str());
            bleTask->setSensorsValues(values);
        }

        if (loggerTask)
            loggerTask->setSensorsLog(message);
    });

    ESP_LOGI(TAG, "configure BLE command handler");
    bleTask->configureCommandReceivedEvent([](std::string_view cmd)
    {
        std::string trimmed(cmd);
        while (!trimmed.empty() && (trimmed.back() == '\r' || trimmed.back() == '\n'))
            trimmed.pop_back();

        ESP_LOGI(TAG, "BLE command received: [%s]", trimmed.c_str());

        if (trimmed == "wifi=enable") {
            ESP_LOGI(TAG, "wifi=enable: not yet implemented");
        } else if (trimmed == "wifi=disable") {
            ESP_LOGI(TAG, "wifi=disable: not yet implemented");
        } else if (trimmed == "led=on") {
            enableUserLED(true);
        } else if (trimmed == "led=off") {
            enableUserLED(false);
        } else if (!trimmed.empty() && gpsTask) {
            gpsTask->sendData(trimmed);
            gpsTask->sendData("\r\n");
        }
    });


    ESP_LOGI(TAG, "start Logger task");
    xTaskCreate([](void *)
    { 
        loggerTask->executeTask();
        vTaskDelete(nullptr);
    }, "logger_task", DEFAULT_TASK_STACK_SIZE, nullptr, 6, nullptr);

    ESP_LOGI(TAG, "listen from GPS module, start task");
    xTaskCreate([](void *)
    { 
        gpsTask->executeTask();
        vTaskDelete(nullptr);
    }, "gps_task", DEFAULT_TASK_STACK_SIZE, nullptr, 6, nullptr);

    ESP_LOGI(TAG, "read sensors, start task");
    xTaskCreate([](void *)
    { 
        sensorTask->executeTask();
        vTaskDelete(nullptr);
    }, "sensors_task", DEFAULT_TASK_STACK_SIZE, nullptr, 6, nullptr);

    startErrorTask(ErrorTask::ErrorCode::ecOK);
}
