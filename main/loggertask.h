#pragma once

#include <cstdint>
#include <string>
#include <vector>
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

    void configureSdCard(const std::shared_ptr<SdCard> &card);
    void executeTask();
    void setGpsLog(const std::string &gpsMessage);
    void setPppLog(const std::string &pppMessage);

  private:
    static const unsigned long LOG_PERIOD_MS = 1 * 1000;

    unsigned long lastLogTime = 0;
    std::shared_ptr<SdCard> m_sdCard;
    std::string currentDate;
    std::string m_gpsLog;
    std::string m_pppLog;

    void logCurrentState();
    void resetState();
    std::string generateTelemetryLog(double temperature, double relative_humidity, double barometric_pressure) const;
    std::string generateDeviceInfoLog() const;
    std::string generateDevicePowerLog() const;

        /// always 3 digits after '.'
    static std::string toTelemetryRoundedString(const float value);
};
