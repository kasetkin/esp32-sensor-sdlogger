#include "loggertask.h"

#include <format>
#include <array>
#include <cmath>
#include <chrono>
#include <mutex>
#include <freertos/FreeRTOS.h>
#include <esp_log.h>
#include "TinyGPS++.h"
#include "common_utils.h"
#include "unicore.h"

static constexpr std::string_view ownerId = "54321"; // &ownerId = devicestate.owner.id;
static constexpr std::string_view ownerShortName = "sdlogger"; // &ownerShortName = devicestate.owner.short_name;
static constexpr std::string_view ownerFullName = "sdlogger_UM980"; // &ownerFullName = devicestate.owner.long_name;

void LoggerTask::setGpsLog(std::string_view gpsMessage)
{
    std::unique_lock oneModuleLock(m_mutex);
    m_gpsLog = gpsMessage;
}

void LoggerTask::setPppLog(std::string_view pppMessage)
{
    std::unique_lock oneModuleLock(m_mutex);
    m_pppLog = pppMessage;
}

void LoggerTask::setSensorsLog(std::string_view sensorsMessage)
{
    std::unique_lock oneModuleLock(m_mutex);
    m_sensorsLog = sensorsMessage;
}

void LoggerTask::addNmeaLog(std::string_view nmeaMessage)
{
    std::unique_lock oneModuleLock(m_mutex);
    /// add, not override!
    m_nmeaLog += nmeaMessage;
}

void LoggerTask::configureSdCard(const std::shared_ptr<SdCard> &card)
{
    m_sdCard = card;
}

void LoggerTask::configureLogReadyEvent(LogReadyEvent readyEvent)
{
    m_readyEvent = std::move(readyEvent);
}

void LoggerTask::executeTask()
{
    static const char * LOGTASKTAG = "LogTask";

    lastLogTime = millisFromStart();
    while (true) {
        /// write-lock inside
        doLogging();

        lastLogTime += LOG_PERIOD_MS;
        const unsigned long now = millisFromStart();
        const unsigned long timeToSleep = lastLogTime - now; // unsigned: valid while on schedule
        if (now <= lastLogTime && timeToSleep < LOG_PERIOD_MS * 3600 * 24) {
            ESP_LOGI(LOGTASKTAG, "SdLoggerModule | next log in %lu ms", timeToSleep);
            vTaskDelay(pdMS_TO_TICKS(timeToSleep));
        } else {
            // logging overran a whole period (or millis wrapped): drop the
            // backlog and realign instead of bursting to catch up
            ESP_LOGW(LOGTASKTAG, "SdLoggerModule | logging overran the period, resync");
            lastLogTime = now;
        }
    }
}

void LoggerTask::doLogging()
{
    std::unique_lock fullLock(m_mutex);
    logCurrentState();
    logNmeaStream();
    resetState();
}

/// lock messages before!
void LoggerTask::resetState()
{
    m_gpsLog.clear();
    m_pppLog.clear();
    m_sensorsLog.clear();
    m_nmeaLog.clear();
}

/// lock messages before!
void LoggerTask::logCurrentState()
{
    static const char * LOGSTATETAG = "LogState";

    ESP_LOGI(LOGSTATETAG, "SdLoggerModule | message generation - start");
    // createSDDir(logsPath);

    const std::string filename = generateFilename() + ".csv";
    const std::string deviceLog = generateDeviceInfoLog();

    const std::string fullLogMessage = deviceLog + m_sensorsLog + m_gpsLog + m_pppLog + std::string("\n");
    ESP_LOGI(LOGSTATETAG, "SdLoggerModule | message generation - end");
    ESP_LOGI(LOGSTATETAG, "SdLoggerModule | full message: \\");
    ESP_LOGI(LOGSTATETAG, "%s \\", deviceLog.c_str());
    ESP_LOGI(LOGSTATETAG, "%s \\", m_gpsLog.c_str());
    ESP_LOGI(LOGSTATETAG, "%s \\", m_pppLog.c_str());
    ESP_LOGI(LOGSTATETAG, "%s \\", m_sensorsLog.c_str());
    ESP_LOGI(LOGSTATETAG, "filename: %s", filename.c_str());
    ESP_LOGI(LOGSTATETAG, "END-OF-LINE");

    const std::string fullpath = "/" + filename; //std::string(logsPath) + 
    if (!m_sdCard)
        return;

    if (m_readyEvent)
        m_readyEvent(fullLogMessage);
        
    if (const esp_err_t err = m_sdCard->appendFile(fullpath, fullLogMessage); err != ESP_OK)
        ESP_LOGE(LOGSTATETAG, "appendFile failed: %d", err);
}

void LoggerTask::logNmeaStream()
{
    static const char * LOGNMEATAG = "LogNmea";
    const std::string filename = generateFilename() + "_nmea.csv";
    const std::string fullpath = "/" + filename;
    if (!m_sdCard)
        return;

    if (m_nmeaLog.empty())
        return;

    m_nmeaLog += std::string("\r\n");
    if (const esp_err_t err = m_sdCard->appendFile(fullpath, m_nmeaLog); err != ESP_OK)
        ESP_LOGE(LOGNMEATAG, "appendFile failed: %d", err);
}

std::string LoggerTask::generateDeviceInfoLog() const
{
    // ESP_LOGI(LOGTASKTAG, "SdLoggerModule | generate device info - start");

    // const bool requestLocalTime = false;
    // uint32_t rtc_sec = getValidTime(RTCQuality::RTCQualityDevice, requestLocalTime);
    const uint64_t rtc_sec = getValidTime();
    std::string message;
    message.reserve(80);
    message += std::string_view("ID;");
    message += ownerId;
    message += std::string_view(";NAME;");
    message += ownerShortName;
    message += std::string_view(";FULLNAME;");
    message += ownerFullName;
    message += std::string_view(";RTCSEC;");
    appendNum(message, rtc_sec);
    message += std::string_view(";");

    // ESP_LOGI(LOGTASKTAG, "SdLoggerModule | generate device info - end | result: %s", message.c_str());
    return message;
}

std::string LoggerTask::generateFilename()
{
    static const char * LOGFILENAMETAG = "LOG Filename";
    const uint64_t rtc_sec = getValidTime();
    
    if (rtc_sec == 0)
        return "NO-DATE-FILE";

    struct tm  gmTime{};
    const time_t stampT = static_cast<time_t>(rtc_sec);
    gmtime_r(&stampT, &gmTime);

    constexpr int GMTIME_YEAR_FIX = 1900;
    constexpr int GMTIME_MONTH_FIX = 1;
    gmTime.tm_year += GMTIME_YEAR_FIX;
    gmTime.tm_mon += GMTIME_MONTH_FIX;

    std::string filename = std::format("{:04d}-{:02d}-{:02d}-{}",
                                       gmTime.tm_year, gmTime.tm_mon, gmTime.tm_mday,
                                       ownerFullName);

    ESP_LOGI(LOGFILENAMETAG, "timestamp from RTC: %lld, date string: %s", rtc_sec, filename.c_str());

    return filename;
}


// void LoggerTask::listSDFiles(const char * dirname, uint8_t levels)
// {
//     concurrency::LockGuard g(spiLock);
//     ESP_LOGI(LOGTASKTAG, "Listing directory: %s\n", dirname);
// 
//     const bool cardIsReady = testAndInitSDCard();
//     if (!cardIsReady)
//         return;
// 
//     File root = SD.open(dirname);
//     if(!root){
//         ESP_LOGI(LOGTASKTAG, "Failed to open directory");
//         return;
//     }
//     if(!root.isDirectory()){
//         ESP_LOGI(LOGTASKTAG, "Not a directory");
//         return;
//     }
// 
//     File file = root.openNextFile();
//     while(file){
//         if (file.isDirectory()) {
//             ESP_LOGI(LOGTASKTAG, "  DIR : %s", file.name());
//             if(levels)
//                 listDir(file.name(), levels - 1);
//     } else {
//         ESP_LOGI(LOGTASKTAG, "  FILE: %s  SIZE: %lu", file.name(), file.size());
//     }
//         file = root.openNextFile();
//     }
// }

// void LoggerTask::writeFile(const char * path, const char * message)
// {
//     concurrency::LockGuard g(spiLock);
//     ESP_LOGI(LOGTASKTAG, "Writing file: %s\n", path);
//
//     const bool cardIsReady = testAndInitSDCard();
//     if (!cardIsReady)
//         return;
//
//     File file = SD.open(path, FILE_WRITE);
//     if (!file) {
//         ESP_LOGI(LOGTASKTAG, "Failed to open file for writing");
//         return;
//     }
//
//     if (file.print(message))
//         ESP_LOGI(LOGTASKTAG, "File written");
//     else
//        ESP_LOGI(LOGTASKTAG, "Write failed");
//
//     file.close();
// }

// void LoggerTask::createSDDir(const char * path)
// {
//     // concurrency::LockGuard g(spiLock);
//
//     const bool cardIsReady = testAndInitSDCard();
//     if (!cardIsReady)
//         return;
//
//     if (SD.exists(path)) {
//         ESP_LOGI(LOGTASKTAG, "Path: <%s> already exists, do nothing\n", path);
//         return;
//     }
//
//     ESP_LOGI(LOGTASKTAG, "Creating Dir: %s\n", path);
//     if (SD.mkdir(path))
//         ESP_LOGI(LOGTASKTAG, "Dir created");
//     else
//         ESP_LOGI(LOGTASKTAG, "mkdir failed");
// }

// void LoggerTask::appendSDFile(const char * path, const char * message)
// {
//     // concurrency::LockGuard g(spiLock);
//     ESP_LOGI(LOGTASKTAG, "Appending to file: %s\n", path);

//     const bool cardIsReady = testAndInitSDCard();
//     if (!cardIsReady)
//         return;

//     File file = SD.open(path, FILE_APPEND);
//     if (!file){
//         ESP_LOGI(LOGTASKTAG, "Failed to open file for appending");
//         return;
//     }

//     if (file.print(message))
//         ESP_LOGI(LOGTASKTAG, "Message appended");
//     else
//         ESP_LOGI(LOGTASKTAG, "Append failed");

//     file.close();
// }

// void LoggerTask::readSDFile(const char * path, std::vector<uint8_t> &fileData)
// {
//     concurrency::LockGuard g(spiLock);
//     ESP_LOGI(LOGTASKTAG, "Reading file: %s\n", path);
// 
//     fileData.clear();
// 
//     const bool cardIsReady = testAndInitSDCard();
//     if (!cardIsReady)
//         return;
// 
//     File file = SD.open(path);
//     if(!file) {
//         ESP_LOGI(LOGTASKTAG, "Failed to open file for reading");
//         return;
//     }
// 
//     fileData.reserve(file.size());
// 
//     ESP_LOGI(LOGTASKTAG, "Read from file: ");
// 
//     constexpr size_t MAX_BUFFER_SIZE = 256;
//     std::array<uint8_t, MAX_BUFFER_SIZE> readBuffer;
//     const uint16_t blockSize = readBuffer.size();
//     while(file.available()) {
//         const int realBlockSize = file.read(readBuffer.data(), blockSize);
//         std::copy(readBuffer.cbegin(), readBuffer.cbegin() + realBlockSize, std::back_inserter(fileData));
//         ESP_LOGI(LOGTASKTAG, "read new %d bytes from the file", realBlockSize);
//     }
// 
//     fileData.shrink_to_fit();
//     file.close();
// }
