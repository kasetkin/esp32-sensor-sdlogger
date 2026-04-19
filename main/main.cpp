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

static const char *TAG = "example";
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
    }, "error_task", 4096, nullptr, 6, nullptr);
}

extern "C" void app_main(void)
{
    esp_err_t ret;

    ret = initNvsFlash();
    if (ret != ESP_OK)
        return;

    enableRf(true);
    enableExtAntenna(true);
    registerWakeupTimer(100 * 1000);

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
    const auto card = my_sdcard->card();
    if (card == nullptr) {
        startErrorTask(ErrorTask::ErrorCode::ecSdCardFilesystemFail);
        return;
    }

    ESP_LOGI(TAG, "pass SdCard to Logger");
    loggerTask->configureSdCard(my_sdcard);
    
    ESP_LOGI(TAG, "pass Logger to GpsTask");
    gpsTask->setupLogger(loggerTask);
    ESP_LOGI(TAG, "pass BleServer to GpsTask");
    gpsTask->setupBleTask(bleTask);

    ESP_LOGI(TAG, "pass Logger to SensorsTask");
    sensorTask->setupLogger(loggerTask);


    ESP_LOGI(TAG, "start Logger task");
    xTaskCreate([](void *)
    { 
        loggerTask->executeTask();
    }, "logger_task", 4096, nullptr, 6, nullptr);

    ESP_LOGI(TAG, "listen from GPS module, start task");
    xTaskCreate([](void *)
    { 
        gpsTask->executeTask();
    }, "gps_task", 4096, nullptr, 6, nullptr);

    ESP_LOGI(TAG, "read sensors, start task");
    xTaskCreate([](void *)
    { 
        sensorTask->executeTask();
    }, "sensors_task", 4096, nullptr, 6, nullptr);

    startErrorTask(ErrorTask::ErrorCode::ecOK);
}
