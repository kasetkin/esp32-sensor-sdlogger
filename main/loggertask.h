#pragma once

#include <cstdint>
#include <string>
#include <vector>

class LoggerTask
{
  public:
    static void listSDFiles(const char * dirname, uint8_t levels);
    static void writeFile(const char * path, const char * message);
    static void createSDDir(const char * path);
    static void appendSDFile(const char * path, const char * message);
    static void readSDFile(const char * path, std::vector<uint8_t> &fileData);
    static std::string generateFilename();
    int32_t runOnce();

  private:
    static const unsigned long LOG_PERIOD_MS = 1 * 1000;
    static const uint32_t MAX_GPS_TO_RTC_MAX_TIME_DELTA_SEC = 20;

    unsigned long lastLogTime = 0;
    std::string currentDate;

    void logCurrentState();
    std::string generateGpsLog() const;
    std::string generatePppLog() const;
    std::string generateTelemetryLog(double temperature, double relative_humidity, double barometric_pressure) const;
    std::string generateDeviceInfoLog() const;
    std::string generateDevicePowerLog() const;

    static std::string dopToMeters(const uint32_t dop);
    static std::string toStringWithZeros(const int value, const size_t numberOfDigits);
    /// always 3 digits after '.'
    static std::string toTelemetryRoundedString(const float value);
};
