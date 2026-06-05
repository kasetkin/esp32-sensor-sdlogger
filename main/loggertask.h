#pragma once

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>
#include <functional>
#include <shared_mutex>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "sdcard.h"

class LoggerTask
{
  public:
    // static void listSDFiles(const char * dirname, uint8_t levels);
    // static void writeFile(const char * path, const char * message);
    // static void createSDDir(const char * path);
    // static void appendSDFile(const char * path, const char * message);
    // static void readSDFile(const char * path, std::vector<uint8_t> &fileData);
    static std::string generateFilename();


    using LogReadyEvent = std::function<void(std::string_view log)>;
    void configureLogReadyEvent(LogReadyEvent readyEvent);

    void configureSdCard(const std::shared_ptr<SdCard> &card);
    void executeTask();
    void doLogging(uint64_t rtcSec);
    void addNmeaLog(std::string_view nmeaMessage);
    void setGnssLog(std::string_view gpsMessage);
    void setSensorsLog(std::string_view sensorsMessage);

    LoggerTask() = default;
    LoggerTask(const LoggerTask &) = delete("LoggerTask holds an SdCard reference and shared state — copying is not meaningful");
    LoggerTask &operator=(const LoggerTask &) = delete("LoggerTask holds an SdCard reference and shared state — copying is not meaningful");

  private:
    // We log exactly once per wall-clock second. Within a second we wait for
    // that second's GNSS epoch (which wakes us via setGnssLog); if it has not
    // arrived by LOG_HEARTBEAT_DEADLINE_MS we log without GNSS data so the line
    // still carries device info + sensors. The state is polled this often.
    static constexpr uint32_t LOG_POLL_INTERVAL_MS = 100;
    static constexpr uint64_t LOG_HEARTBEAT_DEADLINE_MS = 900;

    TaskHandle_t m_loggerTaskHandle = nullptr;
    uint64_t m_lastLoggedSecond = 0;
    mutable std::shared_mutex m_mutex;
    std::shared_ptr<SdCard> m_sdCard;
    LogReadyEvent m_readyEvent;
    std::string currentDate;
    std::string m_nmeaLog;
    std::string m_gnssLog;
    std::string m_sensorsLog;

    void logCurrentState(uint64_t rtcSec);
    void logNmeaStream();
    void resetState();
    std::string generateDeviceInfoLog(uint64_t rtcSec) const;
};
