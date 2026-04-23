#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <functional>
#include <shared_mutex>
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
    static std::string toStringWithZeros(const int value, const size_t numberOfDigits);

    using LogReadyEvent = std::function<void(const std::string &log)>;
    void configureLogReadyEvent(LogReadyEvent readyEvent);

    void configureSdCard(const std::shared_ptr<SdCard> &card);
    void executeTask();
    void doLogging();
    void addNmeaLog(const std::string &nmeaMessage);
    void setGpsLog(const std::string &gpsMessage);
    void setPppLog(const std::string &pppMessage);
    void setSensorsLog(const std::string &sensorsMessage);

  private:
    static const unsigned long LOG_PERIOD_MS = 1 * 1000;

    unsigned long lastLogTime = 0;
    mutable std::shared_mutex m_mutex;
    std::shared_ptr<SdCard> m_sdCard;
    LogReadyEvent m_readyEvent;
    std::string currentDate;
    std::string m_nmeaLog;
    std::string m_gpsLog;
    std::string m_pppLog;
    std::string m_sensorsLog;

    void logCurrentState();
    void logNmeaStream();
    void resetState();
    std::string generateDeviceInfoLog() const;
    std::string generateDevicePowerLog() const;
};
