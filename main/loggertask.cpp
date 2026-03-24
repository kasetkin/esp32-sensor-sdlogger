#include "loggertask.h"

#include <array>
#include <cmath>
#include <chrono>
#include "esp_log.h"
// #include "Telemetry/EnvironmentTelemetry.h"
// #include "FSCommon.h"
// #include "SPILock.h"
// #include "RTC.h"
// #include "main.h"
// #include "configuration.h"
#include "TinyGPS++.h"
#include "common_utils.h"
#include "unicore.h"

// #include <SD.h>
// #include <SPI.h>

static const char * logsPath = "/logs";
static const char * LOGTASKTAG = "LogTask";
static const std::string ownerId = "54321"; // &ownerId = devicestate.owner.id;
static const std::string ownerShortName = "sdlogger"; // &ownerShortName = devicestate.owner.short_name;
static const std::string ownerFullName = "sdlogger_UM980"; // &ownerFullName = devicestate.owner.long_name;



int32_t LoggerTask::runOnce()
{
    const unsigned long thisMoment = millisFromStart();
    ESP_LOGD(LOGTASKTAG, "SdLoggerModule | runOnce, time is %d", thisMoment);

    if (thisMoment < lastLogTime + LOG_PERIOD_MS) {
        const unsigned long timeToSleep = LOG_PERIOD_MS + lastLogTime - thisMoment + 1;
        ESP_LOGD(LOGTASKTAG, "too early, sleep for %d millisec", timeToSleep);
        return static_cast<int32_t>(timeToSleep);
    }

    logCurrentState();

    return static_cast<int32_t>(LOG_PERIOD_MS);
}

std::string LoggerTask::dopToMeters(const uint32_t dop)
{
    const auto dv = std::div(dop, 100);
    return std::to_string(dv.quot) + '.' + std::to_string(dv.rem);
}

std::string LoggerTask::toStringWithZeros(const int value, const size_t numberOfDigits)
{
    const std::string basicString = std::to_string(value);
    if (numberOfDigits > basicString.size())
        return std::string(numberOfDigits - basicString.size(), '0') + basicString;
    else
        return basicString;
}

std::string LoggerTask::toTelemetryRoundedString(const float value)
{
    std::string fullString = std::to_string(value);
    const size_t dotPos = fullString.find('.');
    if (dotPos == std::string::npos)
        return fullString;

    const size_t newLenght = std::min(dotPos + static_cast<size_t>(4), fullString.size());
    fullString.resize(newLenght);
    return fullString;
}

void LoggerTask::logCurrentState()
{
    ESP_LOGD(LOGTASKTAG, "SdLoggerModule | message generation - start");
    createSDDir(logsPath);

    const std::string filename = generateFilename() + ".csv";
    const std::string deviceLog = generateDeviceInfoLog();
    const std::string devicePower = generateDevicePowerLog();
    const std::string gpsLog = generateGpsLog();
    const std::string pppLog = generatePppLog();
    const std::string envTelemetry = generateTelemetryLog(36.6f, 42.0f, 960.0f);

    //! \todo distance between GNSS and PPP positions

    const std::string fullLogMessage = deviceLog + devicePower + gpsLog + pppLog + envTelemetry + std::string("\n");
    ESP_LOGD(LOGTASKTAG, "SdLoggerModule | message generation - end");
    ESP_LOGD(LOGTASKTAG, "SdLoggerModule | full message: \\");
    ESP_LOGD(LOGTASKTAG, "%s \\",  deviceLog.c_str());
    ESP_LOGD(LOGTASKTAG, "%s \\",  gpsLog.c_str());
    ESP_LOGD(LOGTASKTAG, "%s \\", pppLog.c_str());
    ESP_LOGD(LOGTASKTAG, "%s \\",  envTelemetry.c_str());
    ESP_LOGD(LOGTASKTAG, "END-OF-LINE");

    const std::string fullpath = std::string(logsPath) + "/" + filename;
    appendSDFile(fullpath.c_str(), fullLogMessage.c_str());
}

std::string LoggerTask::generateTelemetryLog(double temperature, double relative_humidity, double barometric_pressure) const
{
    ESP_LOGD(LOGTASKTAG, "SdLoggerModule | generate telemetry - start");

    // auto &envTelemetry = m.variant.environment_metrics;

    std::string result;
    if (!std::isnan(temperature))
        result += std::string("TEMP;") + toTelemetryRoundedString(temperature) + std::string(";");

    if (!std::isnan(relative_humidity))
        result += std::string("HUMID;") + toTelemetryRoundedString(relative_humidity) + std::string(";");

    if (!std::isnan(barometric_pressure))
        result += std::string("PRESS;") + toTelemetryRoundedString(barometric_pressure) + std::string(";");

    ESP_LOGD(LOGTASKTAG, "SdLoggerModule | generate telemetry - end | result: %s", result.c_str());
    return result;
}

std::string LoggerTask::generateDevicePowerLog() const
{
    ESP_LOGD(LOGTASKTAG, "SdLoggerModule | generate device power log - start");
    std::string message;
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

    //! \todo implement voltage measurement with 1:2 200kOhm resistors

    ESP_LOGD(LOGTASKTAG, "SdLoggerModule | generate device power log - end | result: %s", message.c_str());
    return message;
}

std::string LoggerTask::generateDeviceInfoLog() const
{
    ESP_LOGD(LOGTASKTAG, "SdLoggerModule | generate device info - start");

    // const bool requestLocalTime = false;
    // uint32_t rtc_sec = getValidTime(RTCQuality::RTCQualityDevice, requestLocalTime);
    const uint64_t rtc_sec = getValidTime();
    const std::string message =
            std::string("ID;") + std::string(ownerId)
        + std::string(";NAME;") + std::string(ownerShortName)
        + std::string(";FULLNAME;") + std::string(ownerFullName)
        + std::string(";RTCSEC;") + std::to_string(rtc_sec)
        + std::string(";");

    ESP_LOGD(LOGTASKTAG, "SdLoggerModule | generate device info - end | result: %s", message.c_str());
    return message;
}

std::string LoggerTask::generateFilename()
{
    const uint64_t rtc_sec = getValidTime();
    if (rtc_sec == 0)
        return "NO-DATE-FILE";

    struct tm  gmTime{};
    const time_t stampT = static_cast<time_t>(rtc_sec);
    gmTime = *gmtime(&stampT);

    constexpr int GMTIME_YEAR_FIX = 1900;
    constexpr int GMTIME_MONTH_FIX = 1;
    gmTime.tm_year += GMTIME_YEAR_FIX;
    gmTime.tm_mon += GMTIME_MONTH_FIX;

    const std::string yearStr = std::to_string(gmTime.tm_year);
    const std::string monthStr = toStringWithZeros(gmTime.tm_mon, 2);
    const std::string dayStr = toStringWithZeros(gmTime.tm_mday, 2);
    std::string filename = yearStr + "-" + monthStr + "-" + dayStr + "-" + ownerFullName;

    ESP_LOGD(LOGTASKTAG, "timestamp from RTC: %d, date string: %s", rtc_sec, filename.c_str());

    return filename;
}

std::string LoggerTask::generateGpsLog() const
{
    ESP_LOGD(LOGTASKTAG, "SdLoggerModule | generate GPS info - start");
    const auto &p = localPosition;

    if (!locationHas3DFix(p)) {
        ESP_LOGD(LOGTASKTAG, "SdLoggerModule | generate GPS info - end | no fix");
        return "";
    }

    const bool requestLocalTime = false;
    uint32_t rtc_sec = getValidTime();
    const bool correctTime = (rtc_sec >= p.timestamp - MAX_GPS_TO_RTC_MAX_TIME_DELTA_SEC)
        && (rtc_sec <= p.timestamp + MAX_GPS_TO_RTC_MAX_TIME_DELTA_SEC);

    if (!correctTime) {
        ESP_LOGD(LOGTASKTAG, "SdLoggerModule | generate GPS info - end | too old coordinates!!! ");
        ESP_LOGD(LOGTASKTAG, "SdLoggerModule | rtc time %d, GPS time %d", rtc_sec, p.timestamp);
        return "";
    }

    struct tm  gmTime{};
    const time_t stampT = static_cast<time_t>(p.timestamp);
    gmTime = *gmtime(&stampT);

    constexpr int GMTIME_YEAR_FIX = 1900;
    constexpr int GMTIME_MONTH_FIX = 1;
    gmTime.tm_year += GMTIME_YEAR_FIX;
    gmTime.tm_mon += GMTIME_MONTH_FIX;

    const std::string yearStr = std::to_string(gmTime.tm_year);
    const std::string monthStr = toStringWithZeros(gmTime.tm_mon, 2);
    const std::string dayStr = toStringWithZeros(gmTime.tm_mday, 2);
    const std::string dateString = yearStr + "-" + monthStr + "-" + dayStr;

    const std::string hoursStr = toStringWithZeros(gmTime.tm_hour, 2);
    const std::string minutesStr = toStringWithZeros(gmTime.tm_min, 2);
    const std::string secondsStr = toStringWithZeros(gmTime.tm_sec, 2);
    const std::string millisWithZeros = p.timestamp_millis_adjust > 0
        ? '.' + toStringWithZeros(p.timestamp_millis_adjust, 3)
        : std::string();

    const std::string timeString = hoursStr + ":" + minutesStr + ":" + secondsStr + millisWithZeros;
    const std::string dateTimeStringFull = dateString + 'T' + timeString + 'Z';

    // ESP_LOGD(LOGTASKTAG, "date from GPS: %d-%d-%dT%d:%d:%d.%dZ",
    //     gmTime.tm_year, gmTime.tm_mon, gmTime.tm_mday,
    //     gmTime.tm_hour, gmTime.tm_min, gmTime.tm_sec,
    //     p.timestamp_millis_adjust
    // );
    // ESP_LOGD(LOGTASKTAG, "date formatted by code: %s", dateTimeStringFull.c_str());

    const double lat = static_cast<double>(p.latitude_i) * 1e-7;
    const double lon = static_cast<double>(p.longitude_i) * 1e-7;
    const std::string message =
        std::string("DT;") + dateTimeStringFull
        + std::string(";GNSSSEC;") + std::to_string(p.timestamp)
        + std::string(";LAT;") + std::to_string(lat)
        + std::string(";LON;") + std::to_string(lon)
        /// over ellipsoid (WGS-84)
        + std::string(";ALT;") + std::to_string(p.altitude)
        /// altitude over geoid, but which one???
        + std::string(";ALTHAE;") + std::to_string(p.altitude_hae)
        /// geoid undulation (separation), use 'Gravity' tool from 'geographiclib' library
        /// $ /usr/local/bin/Gravity -n egm96 --input-string "27.988 86.925" -H
        /// and result sould be ~ "-28.7422" in meters
        + std::string(";UNDUL;") + std::to_string(p.altitude_geoidal_separation)
        + std::string(";SATS;") + std::to_string(p.sats_in_view)
        + std::string(";PDOP;") + dopToMeters(p.PDOP)
        + std::string(";HDOP;") + dopToMeters(p.HDOP)
        + std::string(";VDOP;") + dopToMeters(p.VDOP)
        + std::string(";");

    ESP_LOGD(LOGTASKTAG, "SdLoggerModule | generate GPS info - end");
    return message;
}

std::string LoggerTask::generatePppLog() const
{
    ESP_LOGD(LOGTASKTAG, "SdLoggerModule | generate PPP info - start");
    const auto &p = localPPP;

    const bool requestLocalTime = false;
    uint32_t rtc_sec = getValidTime();
    const bool correctTime = (rtc_sec >= p.utxSeconds - MAX_GPS_TO_RTC_MAX_TIME_DELTA_SEC)
        && (rtc_sec <= p.utxSeconds + MAX_GPS_TO_RTC_MAX_TIME_DELTA_SEC);

    if (!correctTime) {
        ESP_LOGD(LOGTASKTAG, "SdLoggerModule | generate PPP info - end | too old coordinates!!!");
        ESP_LOGD(LOGTASKTAG, "SdLoggerModule | solution age %d, rtc time %d, GPS time %d", p.solutionAge, rtc_sec, p.utxSeconds);
        return "";
    }

    struct tm  gmTime{};
    const time_t stampT = static_cast<time_t>(p.utxSeconds);
    gmTime = *gmtime(&stampT);

    constexpr int GMTIME_YEAR_FIX = 1900;
    constexpr int GMTIME_MONTH_FIX = 1;
    gmTime.tm_year += GMTIME_YEAR_FIX;
    gmTime.tm_mon += GMTIME_MONTH_FIX;

    const std::string yearStr = std::to_string(gmTime.tm_year);
    const std::string monthStr = toStringWithZeros(gmTime.tm_mon, 2);
    const std::string dayStr = toStringWithZeros(gmTime.tm_mday, 2);
    const std::string dateString = yearStr + "-" + monthStr + "-" + dayStr;

    const std::string hoursStr = toStringWithZeros(gmTime.tm_hour, 2);
    const std::string minutesStr = toStringWithZeros(gmTime.tm_min, 2);
    const std::string secondsStr = toStringWithZeros(gmTime.tm_sec, 2);
    const std::string millisWithZeros = p.millisecs > 0
        ? '.' + toStringWithZeros(p.millisecs, 3)
        : std::string();

    const std::string timeString = hoursStr + ":" + minutesStr + ":" + secondsStr + millisWithZeros;
    const std::string dateTimeStringFull = dateString + 'T' + timeString + 'Z';

    ESP_LOGD(LOGTASKTAG, "date from GPS: %d-%d-%dT%d:%d:%d.%dZ",
        gmTime.tm_year, gmTime.tm_mon, gmTime.tm_mday,
        gmTime.tm_hour, gmTime.tm_min, gmTime.tm_sec,
        p.millisecs
    );
    ESP_LOGD(LOGTASKTAG, "date formatted by code: %s", dateTimeStringFull.c_str());

    const double latPpp = static_cast<double>(p.lat) * 1e-7;
    const double lonPpp = static_cast<double>(p.lon) * 1e-7;

    const double latGnss = static_cast<double>(localPosition.latitude_i) * 1e-7;
    const double lonGnss = static_cast<double>(localPosition.longitude_i) * 1e-7;

    const double gnssToPppDistance = GeoCoord::latLongToMeter(latPpp, lonPpp, latGnss, lonGnss);

    const std::string message =
        std::string("PPP_SOLUTION_STATUS;") + solutionStatusStr(p.solutionStatus)
        + std::string(";PPP_POSITION;") + positionTypeStr(p.positionType)
        + std::string(";PPP_SERVICE;") + serviceIdStr(p.serviceId)
        + std::string(";PPP_DATUM;") + datumIdStr(p.datumId)
        + std::string(";PPP_DT;") + dateTimeStringFull
        + std::string(";PPP_TIME;") + std::to_string(p.utxSeconds)
        + std::string(";PPP_AGE;") + std::to_string(p.solutionAge)
        + std::string(";PPP_LAT;") + std::to_string(latPpp)
        + std::string(";PPP_LON;") + std::to_string(lonPpp)
        + std::string(";PPP_GNSS_OFFSET;") + std::to_string(gnssToPppDistance)
        /// over ellipsoid (WGS-84)
        + std::string(";PPP_ALT;") + std::to_string(p.alt)
        /// altitude over geoid, but which one???
        // + std::string(";ALTHAE;") + std::to_string(p.altitude_hae)
        /// geoid undulation (separation), use 'Gravity' tool from 'geographiclib' library
        /// $ /usr/local/bin/Gravity -n egm96 --input-string "27.988 86.925" -H
        /// and result sould be ~ "-28.7422" in meters
        // + std::string(";UNDUL;") + std::to_string(p.altitude_geoidal_separation)
        + std::string(";PPP_SATS;") + std::to_string(p.satellites)
        + std::string(";PPP_STATION_ID;") + std::to_string(p.stationId)
        + std::string(";PPP_LATSTDDEV;") + std::to_string(p.latStdDev)
        + std::string(";PPP_LONSTDDEV;") + std::to_string(p.lonStdDev)
        + std::string(";PPP_ALTSTDDEV;") + std::to_string(p.altStdDev)
        + std::string(";");

    ESP_LOGD(LOGTASKTAG, "SdLoggerModule | generate PPP info - end");
    return message;
}

void LoggerTask::listSDFiles(const char * dirname, uint8_t levels)
{
    concurrency::LockGuard g(spiLock);
    ESP_LOGD(LOGTASKTAG, "Listing directory: %s\n", dirname);

    const bool cardIsReady = testAndInitSDCard();
    if (!cardIsReady)
        return;

    File root = SD.open(dirname);
    if(!root){
        ESP_LOGD(LOGTASKTAG, "Failed to open directory");
        return;
    }
    if(!root.isDirectory()){
        ESP_LOGD(LOGTASKTAG, "Not a directory");
        return;
    }

    File file = root.openNextFile();
    while(file){
        if (file.isDirectory()) {
            ESP_LOGD(LOGTASKTAG, "  DIR : %s", file.name());
            if(levels)
                listDir(file.name(), levels - 1);
    } else {
        ESP_LOGD(LOGTASKTAG, "  FILE: %s  SIZE: %lu", file.name(), file.size());
    }
        file = root.openNextFile();
    }
}

void LoggerTask::writeFile(const char * path, const char * message)
{
    concurrency::LockGuard g(spiLock);
    ESP_LOGD(LOGTASKTAG, "Writing file: %s\n", path);

    const bool cardIsReady = testAndInitSDCard();
    if (!cardIsReady)
        return;

    File file = SD.open(path, FILE_WRITE);
    if (!file) {
        ESP_LOGD(LOGTASKTAG, "Failed to open file for writing");
        return;
    }

    if (file.print(message))
        ESP_LOGD(LOGTASKTAG, "File written");
    else
       ESP_LOGD(LOGTASKTAG, "Write failed");

    file.close();
}

void LoggerTask::createSDDir(const char * path)
{
    // concurrency::LockGuard g(spiLock);

    const bool cardIsReady = testAndInitSDCard();
    if (!cardIsReady)
        return;

    if (SD.exists(path)) {
        ESP_LOGD(LOGTASKTAG, "Path: <%s> already exists, do nothing\n", path);
        return;
    }

    ESP_LOGD(LOGTASKTAG, "Creating Dir: %s\n", path);
    if (SD.mkdir(path))
        ESP_LOGD(LOGTASKTAG, "Dir created");
    else
        ESP_LOGD(LOGTASKTAG, "mkdir failed");
}

void LoggerTask::appendSDFile(const char * path, const char * message)
{
    concurrency::LockGuard g(spiLock);
    ESP_LOGD(LOGTASKTAG, "Appending to file: %s\n", path);

    const bool cardIsReady = testAndInitSDCard();
    if (!cardIsReady)
        return;

    File file = SD.open(path, FILE_APPEND);
    if (!file){
        ESP_LOGD(LOGTASKTAG, "Failed to open file for appending");
        return;
    }

    if (file.print(message))
        ESP_LOGD(LOGTASKTAG, "Message appended");
    else
        ESP_LOGD(LOGTASKTAG, "Append failed");

    file.close();
}

void LoggerTask::readSDFile(const char * path, std::vector<uint8_t> &fileData)
{
    concurrency::LockGuard g(spiLock);
    ESP_LOGD(LOGTASKTAG, "Reading file: %s\n", path);

    fileData.clear();

    const bool cardIsReady = testAndInitSDCard();
    if (!cardIsReady)
        return;

    File file = SD.open(path);
    if(!file) {
        ESP_LOGD(LOGTASKTAG, "Failed to open file for reading");
        return;
    }

    fileData.reserve(file.size());

    ESP_LOGD(LOGTASKTAG, "Read from file: ");

    constexpr size_t MAX_BUFFER_SIZE = 256;
    std::array<uint8_t, MAX_BUFFER_SIZE> readBuffer;
    const uint16_t blockSize = readBuffer.size();
    while(file.available()) {
        const int realBlockSize = file.read(readBuffer.data(), blockSize);
        std::copy(readBuffer.cbegin(), readBuffer.cbegin() + realBlockSize, std::back_inserter(fileData));
        ESP_LOGD(LOGTASKTAG, "read new %d bytes from the file", realBlockSize);
    }

    fileData.shrink_to_fit();
    file.close();
}
