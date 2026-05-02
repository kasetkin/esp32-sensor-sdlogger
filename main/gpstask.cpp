#include "gpstask.h"

#include <cmath>
#include <ctime>
#include <chrono>
#include <array>
#include <iostream>
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "common_utils.h"
#include "unicore.h"
// #include "fake_nmea.h"

static const char * LOGTASKTAG = "gps logger";

esp_err_t GpsTask::configureUart()
{
    const uart_config_t uart_config = {
        .baud_rate = UM980_UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
        .flags = {}
    };
    // We won't use a buffer for sending data.
    const esp_err_t driverRet = uart_driver_install(GPS_UART_PORT, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    if (driverRet != ESP_OK)
        return driverRet;

    const esp_err_t uartConfigRet = uart_param_config(GPS_UART_PORT, &uart_config);
    if (uartConfigRet != ESP_OK) 
        return uartConfigRet;

    const esp_err_t gpioConfigRet = uart_set_pin(GPS_UART_PORT, UART_TX_GPIO_PIN, UART_RX_GPIO_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (gpioConfigRet != ESP_OK)
        return gpioConfigRet;
    
    return ESP_OK;
}

void GpsTask::gpsUartDelay()
{
    vTaskDelay(pdMS_TO_TICKS(GPS_TASK_TX2RX_DELAY_MICROSEC));
}

esp_err_t GpsTask::configureUM980()
{
    std::string reply;
    ESP_LOGI(LOGTASKTAG, "UM980 configuration: start");
    /// check for receiver
    sendStringAndWait("VERSION\r\n", reply);
    if (reply.find("UM980") == std::string::npos) {
        ESP_LOGE(LOGTASKTAG, "UM980 not detected, VERSION reply: %s", reply.c_str());
        return ESP_FAIL;
    }

    /// request config, only for debug
    sendStringAndWait("CONFIG\r\n", reply);

    // /// setup baudrate (needed if it is different and we want to switch it)
    // sendStringAndWait("CONFIG COM1 115200\r\n");
    // sendStringAndWait("CONFIG COM2 115200\r\n");
    // sendStringAndWait("CONFIG COM3 115200\r\n");
    // sendStringAndWait("SAVECONFIG\r\n");

    sendStringAndWait("CONFIG SIGNALGROUP 2\r\n", reply);
    sendStringAndWait("MODE ROVER SURVEY DEFAULT\r\n", reply);
    sendStringAndWait("CONFIG RTK TIMEOUT 0\r\n", reply);
    /// 'AUTO' or 'E6-HAS' or 'B2b-PPP' or 'SSR-RX' or 'L6MDCPPP' ?
    sendStringAndWait("CONFIG PPP ENABLE AUTO\r\n", reply);
    /// we don't need default 15cm precision, 70cm in horizontal and 100cm in vertical should be enough
    sendStringAndWait("CONFIG PPP CONVERGE 75 200\r\n", reply);
    sendStringAndWait("CONFIG PPP DATUM WGS84\r\n", reply);
    sendStringAndWait("CONFIG DGPS TIMEOUT 0\r\n", reply);
    sendStringAndWait("CONFIG MMP ENABLE\r\n", reply);
    sendStringAndWait("CONFIG PVTALG MULTI\r\n", reply);
    sendStringAndWait("CONFIG IONMODE GPSK8\r\n", reply);
    sendStringAndWait("CONFIG ANTIJAM FORCE\r\n", reply);
    sendStringAndWait("CONFIG PSRVELDRPOS DISABLE\r\n", reply);
    sendStringAndWait("CONFIG UNDULATION AUTO\r\n", reply);
    sendStringAndWait("CONFIG NMEA0183 V411\r\n", reply);
    sendStringAndWait("CONFIG SBAS ENABLE AUTO\r\n", reply); /// why not, if there is any in your region
    sendStringAndWait("CONFIG SBAS TIMEOUT 1800\r\n", reply);
    sendStringAndWait("CONFIG STANDALONE ENABLE\r\n", reply); /// not sure if it's a good idea

    sendStringAndWait("UNMASK ALL\r\n", reply); /// enable all GNSS systems (GPS, Galileo, Beidou, Glonass, QZSS, IRNSS)
    sendStringAndWait("UNMASK GPS\r\n", reply); /// USA
    sendStringAndWait("UNMASK BDS\r\n", reply); /// Beidou, China
    sendStringAndWait("UNMASK GLO\r\n", reply); /// GLONASS, Russia
    sendStringAndWait("UNMASK GAL\r\n", reply); /// Galileo, Europe
    sendStringAndWait("UNMASK QZSS\r\n", reply); /// Quasi-Zenith Satellite System, Japanese
    sendStringAndWait("UNMASK IRNSS\r\n", reply); /// NavIC, Indian
    sendStringAndWait("MASK 0.0\r\n", reply); /// mask elevation angle

    /// disable currently enabled messages
    sendStringAndWait("UNLOG\r\n", reply);

    /// enable necessary NMEA messages
    sendStringAndWait("GPGGA 1\r\n", reply);
    sendStringAndWait("GPGSA 1\r\n", reply);
    sendStringAndWait("GPRMC 1\r\n", reply);
    sendStringAndWait("GPGSV 1\r\n", reply);
    /// Enable Unicore specific PPP messages
    sendStringAndWait("PPPNAVA 1\r\n", reply);

    /// print ALL observations from antenna!!!
    /// TOO MANY symbols even at speed 115200,
    /// \todo test with higher speed
    //sendStringAndWait("OBSVMA 1\r\n");

    sendStringAndWait("SAVECONFIG\r\n", reply);
    ESP_LOGI(LOGTASKTAG, "UM980 configuration: success");
    return ESP_OK;
}

esp_err_t GpsTask::configureTinyGps()
{
    gsafixtype.begin(m_gps, NMEA_MSG_GXGSA, 2);
    gsapdop.begin(m_gps, NMEA_MSG_GXGSA, 15);
    gsahdop.begin(m_gps, NMEA_MSG_GXGSA, 16);
    gsavdop.begin(m_gps, NMEA_MSG_GXGSA, 17);

    pppnavWeek.begin(m_gps, UNICORE_MSG_PPPNAV, 4);
    pppnavSecsOFWeek.begin(m_gps, UNICORE_MSG_PPPNAV, 5);
    pppnavLeapSecs.begin(m_gps, UNICORE_MSG_PPPNAV, 8);

    pppnavSolStatus.begin(m_gps, UNICORE_MSG_PPPNAV, 9);
    pppnavPosType.begin(m_gps, UNICORE_MSG_PPPNAV, 10);
    pppnavLat.begin(m_gps, UNICORE_MSG_PPPNAV, 11);
    pppnavLon.begin(m_gps, UNICORE_MSG_PPPNAV, 12);
    pppnavAlt.begin(m_gps, UNICORE_MSG_PPPNAV, 13);
    pppnavDatumId.begin(m_gps, UNICORE_MSG_PPPNAV, 15);
    pppnavLatStdDev.begin(m_gps, UNICORE_MSG_PPPNAV, 16);
    pppnavLonStdDev.begin(m_gps, UNICORE_MSG_PPPNAV, 17);
    pppnavAltStdDev.begin(m_gps, UNICORE_MSG_PPPNAV, 18);
    pppnavStationId.begin(m_gps, UNICORE_MSG_PPPNAV, 19);
    pppnavSolAge.begin(m_gps, UNICORE_MSG_PPPNAV, 21);
    pppnavSatellites.begin(m_gps, UNICORE_MSG_PPPNAV, 23);

    return ESP_OK;
}

esp_err_t GpsTask::setupLogger(std::shared_ptr<LoggerTask> logger)
{
    m_logger.reset();
    if (!logger)
        return ESP_FAIL;
    
    m_logger = logger;
    return ESP_OK;
}

esp_err_t GpsTask::setupBleTask(std::shared_ptr<BleSppServerTask> ble)
{
    m_ble.reset();
    if (!ble)
        return ESP_FAIL;
    
    m_ble = ble;
    return ESP_OK;
}

int GpsTask::sendData(const char* data)
{
    static const char *TX_TASK_TAG = "TX_TASK1";
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(GPS_UART_PORT, data, len);
    if (txBytes != len)
        ESP_LOGE(TX_TASK_TAG, "wrong string size");
        
    return txBytes;
}

int GpsTask::sendStringAndWait(const std::string &str, std::string &reply)
{
    static const char *TX_TASK_TAG = "TX_TASK2";
    const int len = str.size();
    const int txBytes = uart_write_bytes(GPS_UART_PORT, str.c_str(), len);
    if (txBytes != len)
        ESP_LOGE(TX_TASK_TAG, "wrong string size");

    gpsUartDelay();

    readFromUart(reply);
    if (reply.size() > 0)
        ESP_LOGI(TX_TASK_TAG, "reply: %s", reply.c_str());

    return txBytes;
}

// void GpsTask::tx_task(void *arg)
// {
//     static const char *TX_TASK_TAG = "TX_TASK";
//     esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
//     while (1) {
//         sendData(TX_TASK_TAG, "Hello world");
//         vTaskDelay(2000 / portTICK_PERIOD_MS);
//     }
// }

void GpsTask::readFromUart(std::string &newData)
{
    newData.clear();

    const uint32_t readTimeoutInTicks = pdMS_TO_TICKS(1);
    std::array<char, RX_BUF_SIZE + 1> data;
    const int rxBytes = uart_read_bytes(GPS_UART_PORT, data.data(), RX_BUF_SIZE, readTimeoutInTicks);
    if (rxBytes > 0) {
        for (size_t i = 0; i < static_cast<size_t>(rxBytes); ++i) 
            newData.push_back(data[i]);
    }
}

void GpsTask::executeTask()
{
    static const char *GPS_TASK_TAG = "GPS_TASK";
    std::string dataAsString;
    while (true) {
        readFromUart(dataAsString);
        if (dataAsString.size() > 0) {
            ESP_LOGI(GPS_TASK_TAG, "Read %u bytes: '%s'", dataAsString.size(), dataAsString.c_str());
            // ESP_LOG_BUFFER_HEXDUMP(GPS_TASK_TAG, data.data(), rxBytes, ESP_LOG_INFO);

            for (auto c : dataAsString)
                m_gps.encode(c);

            if (m_logger)
                m_logger->addNmeaLog(dataAsString);

            if (m_ble)
                 m_ble->appendNmea(dataAsString); /// send NMEA stream only, QSTARZ emlation is done inside `processNewLocation()`

            dataAsString.clear();
            
            if (m_gps.location.isValid())
                ESP_LOGI(GPS_TASK_TAG, "Valid location from GPS");

            const bool newLocation = processNewLocation();
            if (newLocation)
                ESP_LOGI(GPS_TASK_TAG, "new location reported");
        }

        vTaskDelay(pdMS_TO_TICKS(GPS_TASK_DELAY_MS));
    }
}

bool GpsTask::hasLock(const GpsInfo &info)
{
    if (info.quality == TinyGPSLocation::Quality::GPS
        || info.quality == TinyGPSLocation::Quality::DGPS
        || info.quality == TinyGPSLocation::Quality::PPS
        || info.quality == TinyGPSLocation::Quality::RTK
        || info.quality == TinyGPSLocation::Quality::FloatRTK) {
        // Use GPGSA fix type 2D/3D (better) if available
        // 0 -- no data,
        // 1 -- Fix not available
        // 2 -- 2D fix, good or not enough ???
        // 3 -- 3D fix
        if (info.fixType == 3 
            || info.fixType == 2
            || info.fixType == 0)
            return true;
    }

    return false;
}

bool GpsTask::has3DLock(const GpsInfo &info)
{
    if (info.quality == TinyGPSLocation::Quality::GPS
        || info.quality == TinyGPSLocation::Quality::DGPS
        || info.quality == TinyGPSLocation::Quality::PPS
        || info.quality == TinyGPSLocation::Quality::RTK
        || info.quality == TinyGPSLocation::Quality::FloatRTK) {
        // Use GPGSA fix type 2D/3D (better) if available
        // 0 -- no data,
        // 1 -- Fix not available
        // 2 -- 2D fix, good or not enough ???
        // 3 -- 3D fix
        if (info.fixType == 3)
            return true;
    }

    return false;
}

bool GpsTask::processNewLocation()
{
    static const char * NEW_LOCATION_TAG = "read-gps-location";
    ESP_LOGD(NEW_LOCATION_TAG, "start reading GPS location");

    if (!m_gps.location.isUpdated() && !m_gps.altitude.isUpdated()) {
        ESP_LOGD(NEW_LOCATION_TAG, "GPS location or altitude is not updated, no new GPS logs");
        return false;
    }

    ESP_LOGI(NEW_LOCATION_TAG, "location and altitude are updated");
    if (!m_gps.location.isValid()) {
        ESP_LOGD(NEW_LOCATION_TAG, "GPS location is invalid, no new GPS logs");
        return false;
    }

    ESP_LOGI(NEW_LOCATION_TAG, "location is valid");

    const uint32_t GPS_SOLUTION_MAX_AGE_MS = 10 * 1000;
    if ((m_gps.location.age() > GPS_SOLUTION_MAX_AGE_MS)
          || (gsafixtype.age() > GPS_SOLUTION_MAX_AGE_MS)
          || (m_gps.time.age() > GPS_SOLUTION_MAX_AGE_MS) 
          || (m_gps.date.age() > GPS_SOLUTION_MAX_AGE_MS)) {
        ESP_LOGD(NEW_LOCATION_TAG, "some GPS data is TOO OLD: location, GSA fix, time/date");
        return false;
    }

    ESP_LOGI(NEW_LOCATION_TAG, "location, GSA-fix, time, date are fresh and valid");

    GpsInfo gpsInfo{};
    gpsInfo.fixType = atoi(gsafixtype.value()); // will set to zero if no data
    gpsInfo.quality = m_gps.location.FixQuality();
    gpsInfo.lat = m_gps.location.lat();
    gpsInfo.lon = m_gps.location.lng();
    gpsInfo.altitude = m_gps.altitude.meters();
    gpsInfo.geoidAlt = m_gps.geoidHeight.meters();

    if (!hasLock(gpsInfo))
        return false;
    
    ESP_LOGI(NEW_LOCATION_TAG, "GPS has 2D lock");

    // We know the solution is fresh and valid, so just read the data
    {
        // positional timestamp
        struct tm t;
        t.tm_sec = m_gps.time.second();
        t.tm_min = m_gps.time.minute();
        t.tm_hour = m_gps.time.hour();
        t.tm_mday = m_gps.date.day();
        t.tm_mon = m_gps.date.month() - 1;
        t.tm_year = m_gps.date.year() - 1900;
        t.tm_isdst = false;
        const auto timestamp = std::mktime(&t);
        gpsInfo.worldTime = std::chrono::system_clock::from_time_t(timestamp);

        const int32_t microsec = static_cast<int32_t>(m_gps.time.centisecond()) * 10000;
        struct timeval gpsTimeNow = { .tv_sec = timestamp, .tv_usec = microsec };
        struct timezone myZone = { .tz_minuteswest = -300, .tz_dsttime = DST_NONE };
        settimeofday(&gpsTimeNow, &myZone);

        ESP_LOGI(NEW_LOCATION_TAG, "timestamp generated");

        gpsInfo.gsaPDOP = TinyGPSPlus::parseDecimal(gsapdop.value());
        gpsInfo.gsaHDOP = TinyGPSPlus::parseDecimal(gsahdop.value());
        gpsInfo.gsaVDOP = TinyGPSPlus::parseDecimal(gsavdop.value());

        ESP_LOGI(NEW_LOCATION_TAG, "GSA h/v/p DOP ok");
    }

    PppInfo pppInfo{};
    {
        pppInfo.lat = parseDegreesLatLon(pppnavLat.value());
        pppInfo.lon = parseDegreesLatLon(pppnavLon.value());
        pppInfo.alt = static_cast<int32_t>(atol(pppnavAlt.value()));
        pppInfo.latStdDev = static_cast<float>(atof(pppnavLatStdDev.value()));
        pppInfo.lonStdDev = static_cast<float>(atof(pppnavLonStdDev.value()));
        pppInfo.altStdDev = static_cast<float>(atof(pppnavAltStdDev.value()));
        pppInfo.satellites = static_cast<int32_t>(atol(pppnavSatellites.value()));
        pppInfo.solutionAge = static_cast<int32_t>(atol(pppnavSolAge.value()));
        pppInfo.solutionStatus = parseSolutionStatus(pppnavSolStatus.value(), pppInfo.outputDelayMs);
        pppInfo.positionType = parsePositionType(pppnavPosType.value());
        pppInfo.datumId = parseDatumId(pppnavDatumId.value());
        pppInfo.stationId = parseStationId(pppnavStationId.value());
        pppInfo.serviceId = parsePppService(pppInfo.stationId);

        uint32_t week = static_cast<int32_t>(atol(pppnavWeek.value()));
        uint32_t millisOfWeek = static_cast<int32_t>(atoll(pppnavSecsOFWeek.value()));
        uint32_t leapSecs = static_cast<uint32_t>(atol(pppnavLeapSecs.value()));
        pppInfo.utxSeconds = computeUtxTime(week, millisOfWeek, leapSecs, pppInfo.millisecs);

        ESP_LOGI(NEW_LOCATION_TAG, "PPP info parsed");
    }

    {
        const std::string gpsLog = printGpsTimeInfo(gpsInfo) + printGpsGeoInfo(gpsInfo);
        if (m_logger)
            m_logger->setGpsLog(gpsLog);
    }

    {
        const std::array<std::string, 4> emulatedQstarz = emulateQstarzBinary(gpsInfo);
        if (m_ble)
            m_ble->transmitQstarzPackets(emulatedQstarz);
    }

    {
        const double latPpp = static_cast<double>(pppInfo.lat) * 1e-7;
        const double lonPpp = static_cast<double>(pppInfo.lon) * 1e-7;
        const double latGnss = gpsInfo.lat; //static_cast<double>(localPosition.latitude_i) * 1e-7;
        const double lonGnss = gpsInfo.lon; //static_cast<double>(localPosition.longitude_i) * 1e-7;
        const double gnssToPppDistance = geoDistance(latPpp, lonPpp, latGnss, lonGnss);
        
        const std::string pppLog = printPppTimeInfo(pppInfo) + printPppGeoInfo(pppInfo, gnssToPppDistance);
        if (m_logger)
            m_logger->setPppLog(pppLog);
    }
    
    return true;
}

void GpsTask::logNmeaMessageToSd(const std::string &msg)
{
//     static const char * logsPath = "/logs";
//     const std::string filename = sdLoggerModule->generateFilename() + "_nmea.csv";
//     const std::string fullLogMessage = msg + std::string("\r\n");
//
//     const std::string fullpath = std::string(logsPath) + "/" + filename;
//     sdLoggerModule->appendSDFile(fullpath.c_str(), fullLogMessage.c_str());
}

std::string GpsTask::dopToMeters(const uint32_t dop)
{
    const auto dv = std::div(dop, 100);
    return std::to_string(dv.quot) + '.' + std::to_string(dv.rem);
}

double GpsTask::geoDistance(const double &lat1, const double &lon1, const double &lat2, const double &lon2)
{
    constexpr double R          = 6371000.0;
    constexpr double DEG_TO_RAD = M_PI / 180.0;

    const double dLat    = (lat2 - lat1) * DEG_TO_RAD;
    const double dLon    = (lon2 - lon1) * DEG_TO_RAD;
    const double lat1Rad = lat1 * DEG_TO_RAD;
    const double lat2Rad = lat2 * DEG_TO_RAD;

    const double sinDLat = std::sin(dLat / 2.0);
    const double sinDLon = std::sin(dLon / 2.0);
    const double a = sinDLat * sinDLat
                   + std::cos(lat1Rad) * std::cos(lat2Rad) * sinDLon * sinDLon;
    const double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));

    return R * c;
}


std::string GpsTask::printGpsGeoInfo(const GpsInfo &p)
{
    std::string message = std::string("LAT;") + std::to_string(p.lat);
    message += std::string(";LON;") + std::to_string(p.lon);
        /// over ellipsoid (WGS-84)
    message += std::string(";ALT;") + std::to_string(p.altitude);
        /// altitude over geoid, but which one???
    message += std::string(";ALTHAE;") + std::to_string(p.altitude);
        /// geoid undulation (separation), use 'Gravity' tool from 'geographiclib' library
        /// $ /usr/local/bin/Gravity -n egm96 --input-string "27.988 86.925" -H
        /// and result sould be ~ "-28.7422" in meters
    message += std::string(";UNDUL;") + std::to_string(p.geoidAlt);
    message += std::string(";SATS;") + std::string("0"); //std::to_string(p.sats_in_view)
    message += std::string(";PDOP;") + dopToMeters(p.gsaPDOP);
    message += std::string(";HDOP;") + dopToMeters(p.gsaHDOP);
    message += std::string(";VDOP;") + dopToMeters(p.gsaVDOP);
    message += std::string(";");
    return message;
}

std::string GpsTask::printGpsTimeInfo(const GpsInfo &p)
{
    // ESP_LOGI(LOGTASKTAG, "SdLoggerModule generate GPS info - start");
    
    if (!has3DLock(p)) {
        ESP_LOGI(LOGTASKTAG, "SdLoggerModule | generate GPS info - end | no fix");
        return "";
    }

    // const bool requestLocalTime = false;
    // const uint64_t rtc_sec = getValidTime();
    // const auto rtc_time = millis(); //std::chrono::system_clock::now();
    // const int64_t deltaInSeconds = static_cast<int64_t>(rtc_time) - static_cast<int64_t>(p.worldTime);
    // const bool correctTime = std::abs(deltaInSeconds) <= MAX_GPS_TO_RTC_MAX_TIME_DELTA_SEC;

    // if (!correctTime) {
    //     ESP_LOGI(LOGTASKTAG, "SdLoggerModule | generate GPS info - end | too old coordinates!!! ");
    //     ESP_LOGI(LOGTASKTAG, "SdLoggerModule | delta %lld", deltaInSeconds);
    //     std::cout << "rtc_time " << rtc_time << std::endl;
    //     std::cout << "gnss time " << p.worldTime << std::endl;
    //     return "";
    // }
 

    ESP_LOGI(LOGTASKTAG, "try to get GNSS seconds timestamp");

    const std::time_t stampT = std::chrono::system_clock::to_time_t(p.worldTime);
    const auto gpsSecs = std::chrono::duration_cast<std::chrono::seconds>(p.worldTime.time_since_epoch()).count();
    struct tm  gmTime{};
    gmTime = *gmtime(&stampT);
    
    constexpr int GMTIME_YEAR_FIX = 1900;
    constexpr int GMTIME_MONTH_FIX = 1;
    gmTime.tm_year += GMTIME_YEAR_FIX;
    gmTime.tm_mon += GMTIME_MONTH_FIX;

    const std::string yearStr = std::to_string(gmTime.tm_year);
    const std::string monthStr = LoggerTask::toStringWithZeros(gmTime.tm_mon, 2);
    const std::string dayStr = LoggerTask::toStringWithZeros(gmTime.tm_mday, 2);
    const std::string dateString = yearStr + "-" + monthStr + "-" + dayStr;

    const std::string hoursStr = LoggerTask::toStringWithZeros(gmTime.tm_hour, 2);
    const std::string minutesStr = LoggerTask::toStringWithZeros(gmTime.tm_min, 2);
    const std::string secondsStr = LoggerTask::toStringWithZeros(gmTime.tm_sec, 2);
    const std::string millisWithZeros = std::string();
    // p.timestamp_millis_adjust > 0 ? '.' + LoggerTask::toStringWithZeros(p.timestamp_millis_adjust, 3) : std::string();

    const std::string timeString = hoursStr + ":" + minutesStr + ":" + secondsStr + millisWithZeros;
    const std::string dateTimeStringFull = dateString + 'T' + timeString + 'Z';

    // ESP_LOGI(LOGTASKTAG, "date from GPS: %d-%d-%dT%d:%d:%d.%dZ",
    //     gmTime.tm_year, gmTime.tm_mon, gmTime.tm_mday,
    //     gmTime.tm_hour, gmTime.tm_min, gmTime.tm_sec,
    //     p.timestamp_millis_adjust
    // );
    // ESP_LOGI(LOGTASKTAG, "date formatted by code: %s", dateTimeStringFull.c_str());

    // const double lat = static_cast<double>(p.location.lat()) * 1e-7;
    // const double lon = static_cast<double>(p.location.lng()) * 1e-7;

    std::string message = std::string("DT;") + dateTimeStringFull;
    message += std::string(";GNSSSEC;") + std::to_string(gpsSecs) + ";";

    return message;
}

std::string GpsTask::printPppTimeInfo(const PppInfo &p)
{
    // ESP_LOGI(LOGTASKTAG, "SdLoggerModule | generate PPP Time info - start");
    // uint32_t rtc_sec = getValidTime();
    // const bool correctTime = (rtc_sec >= p.utxSeconds - MAX_GPS_TO_RTC_MAX_TIME_DELTA_SEC)
    //     && (rtc_sec <= p.utxSeconds + MAX_GPS_TO_RTC_MAX_TIME_DELTA_SEC);
    // if (!correctTime) {
    //     ESP_LOGI(LOGTASKTAG, "SdLoggerModule | generate PPP info - end | too old coordinates!!!");
    //     ESP_LOGI(LOGTASKTAG, "SdLoggerModule | solution age %d, rtc time %d, GPS time %d", p.solutionAge, rtc_sec, p.utxSeconds);
    //     return "";
    // }

    struct tm  gmTime{};
    const time_t stampT = static_cast<time_t>(p.utxSeconds);
    gmTime = *gmtime(&stampT);

    constexpr int GMTIME_YEAR_FIX = 1900;
    constexpr int GMTIME_MONTH_FIX = 1;
    gmTime.tm_year += GMTIME_YEAR_FIX;
    gmTime.tm_mon += GMTIME_MONTH_FIX;

    const std::string yearStr = std::to_string(gmTime.tm_year);
    const std::string monthStr = LoggerTask::toStringWithZeros(gmTime.tm_mon, 2);
    const std::string dayStr = LoggerTask::toStringWithZeros(gmTime.tm_mday, 2);
    const std::string dateString = yearStr + "-" + monthStr + "-" + dayStr;

    const std::string hoursStr = LoggerTask::toStringWithZeros(gmTime.tm_hour, 2);
    const std::string minutesStr = LoggerTask::toStringWithZeros(gmTime.tm_min, 2);
    const std::string secondsStr = LoggerTask::toStringWithZeros(gmTime.tm_sec, 2);
    const std::string millisWithZeros = p.millisecs > 0
        ? '.' + LoggerTask::toStringWithZeros(p.millisecs, 3)
        : std::string();

    const std::string timeString = hoursStr + ":" + minutesStr + ":" + secondsStr + millisWithZeros;
    const std::string dateTimeStringFull = dateString + 'T' + timeString + 'Z';

    ESP_LOGI(LOGTASKTAG, "date from PPP: %d-%d-%dT%d:%d:%d.%dZ",
        gmTime.tm_year, gmTime.tm_mon, gmTime.tm_mday,
        gmTime.tm_hour, gmTime.tm_min, gmTime.tm_sec,
        p.millisecs
    );
    ESP_LOGI(LOGTASKTAG, "date formatted by code: %s", dateTimeStringFull.c_str());

    const std::string message =
        std::string("PPP_SOLUTION_STATUS;") + solutionStatusStr(p.solutionStatus)
        + std::string(";PPP_POSITION;") + positionTypeStr(p.positionType)
        + std::string(";PPP_SERVICE;") + serviceIdStr(p.serviceId)
        + std::string(";PPP_DATUM;") + datumIdStr(p.datumId)
        + std::string(";PPP_DT;") + dateTimeStringFull
        + std::string(";PPP_TIME;") + std::to_string(p.utxSeconds)
        + std::string(";PPP_AGE;") + std::to_string(p.solutionAge);

    ESP_LOGI(LOGTASKTAG, "SdLoggerModule | generate PPP Time info - end");
    return message;
}

std::string GpsTask::printPppGeoInfo(const PppInfo &p, const double &gnssToPppDistance)
{
    ESP_LOGI(LOGTASKTAG, "SdLoggerModule | generate PPP GEO info - start");


    const double latPpp = static_cast<double>(p.lat) * 1e-7;
    const double lonPpp = static_cast<double>(p.lon) * 1e-7;

    const std::string message =
        std::string(";PPP_LAT;") + std::to_string(latPpp)
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

    ESP_LOGI(LOGTASKTAG, "SdLoggerModule | generate PPP GEO info - end");
    return message;
}

template<typename T>
void appendRaw(std::string &buf, const T &data)
{
    buf.append(reinterpret_cast<const char *>(&data), sizeof(T));
}

// Binary record format: Qstarz BL-1000GT, 64 bytes, little-endian
std::array<std::string, 4> GpsTask::emulateQstarzBinary(const GpsInfo &p)
{
    const auto epoch = p.worldTime.time_since_epoch();
    const auto secs = std::chrono::duration_cast<std::chrono::seconds>(epoch);
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(epoch) - secs;

    // mode: 1=fix not available, 2=2D, 3=3D (maps directly from fixType; 0->1)
    const uint8_t mode = (p.fixType >= 2 && p.fixType <= 3) ? p.fixType : 1;
    const uint8_t rcr = 'T';
    const uint16_t time_ms = static_cast<uint16_t>(ms.count());

    // Convert decimal degrees to DDDMM.MMMM
    auto toQstarzDeg = [](double decimal) -> double {
        const double absVal = std::abs(decimal);
        const double deg = std::trunc(absVal);
        const double min = (absVal - deg) * 60.0;
        const double result = deg * 100.0 + min;
        return decimal < 0.0 ? -result : result;
    };
    const double dLat = toQstarzDeg(p.lat);
    const double dLon = toQstarzDeg(p.lon);

    const uint32_t time_s = static_cast<uint32_t>(secs.count());
    const float speed_kmph = 0.0f;
    const float height_m = static_cast<float>(p.altitude);
    const float heading = 0.0f;
    const int16_t Gx = 0;
    const int16_t Gy = 0;
    const int16_t Gz = 0;
    const uint16_t maxSNR = 0;
    const float hdop = static_cast<float>(p.gsaHDOP) / 100.0f;
    const float vdop = static_cast<float>(p.gsaVDOP) / 100.0f;
    const uint8_t numSatView = 0;
    const uint8_t numSatUse = 0;
    const uint8_t fixQual = static_cast<uint8_t>(p.quality);
    const uint8_t batPerc = 0;
    const uint16_t dummy = 0;
    const uint8_t series_number = 0; /// Bluetooth GNSS expects '0'

    std::string buf;
    buf.reserve(64);
    /// first
    appendRaw(buf, mode);
    appendRaw(buf, rcr);
    appendRaw(buf, time_ms);
    appendRaw(buf, dLat);
    appendRaw(buf, dLon);
    /// second
    appendRaw(buf, time_s);
    appendRaw(buf, speed_kmph);
    appendRaw(buf, height_m);
    appendRaw(buf, heading);
    appendRaw(buf, Gx);
    appendRaw(buf, Gy);
    /// third
    appendRaw(buf, Gz); //  2
    appendRaw(buf, maxSNR); //  4
    appendRaw(buf, hdop); //  8
    appendRaw(buf, vdop); // 12
    
    appendRaw(buf, numSatView); // 13
    appendRaw(buf, numSatUse); // 14
    appendRaw(buf, fixQual); // 15
    appendRaw(buf, batPerc); // 16

    appendRaw(buf, dummy); 
    appendRaw(buf, series_number);

    const uint8_t unknown_field1 = 0;
    const uint32_t unknown_field2 = 0; 
    appendRaw(buf, unknown_field1);
    appendRaw(buf, unknown_field2);

    assert(buf.size() == 64);

    /// emulate QSTARZ connections specific: send data by packets of 20 bytes
    std::array<std::string, 4> packets;

    packets[0].append(buf.cbegin(), buf.cbegin() + 20);
    packets[1].append(buf.cbegin() + 20, buf.cbegin() + 40);
    packets[2].append(buf.cbegin() + 40, buf.cbegin() + 60);
    packets[3].append(buf.cbegin() + 60, buf.cbegin() + 64);

    ESP_LOGD(LOGTASKTAG, "first packet.length: %u", packets[0].size());
    ESP_LOGD(LOGTASKTAG, "first packet[0]: %u, should be 1, 2, 3", packets[0][0]);
    ESP_LOGD(LOGTASKTAG, "third packet.length: %u", packets[2].size());
    ESP_LOGD(LOGTASKTAG, "third packet[16]: %u", packets[2][16]);
    ESP_LOGD(LOGTASKTAG, "third packet[17]: %u", packets[2][17]);
    ESP_LOGD(LOGTASKTAG, "third packet[18]: %u", packets[2][18]);
    ESP_LOGD(LOGTASKTAG, "third packet[19]: %u", packets[2][19]);
    


    return packets;
}
