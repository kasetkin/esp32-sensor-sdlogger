#include "gpstask.h"

#include <charconv>
#include <format>
#include <cmath>
#include <numbers>
#include <ctime>
#include <chrono>
#include <algorithm>
#include <array>
#include <span>
#include <iostream>
#include <esp_log.h>
#include <driver/uart.h>
#include <driver/gpio.h>
#include "common_utils.h"
#include "unicore.h"
// #include "fake_nmea.h"

static constexpr const char LOGTASKTAG[] = "gps logger";

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
    if (const esp_err_t driverRet = uart_driver_install(GPS_UART_PORT, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
        driverRet != ESP_OK)
        return driverRet;

    if (const esp_err_t uartConfigRet = uart_param_config(GPS_UART_PORT, &uart_config); uartConfigRet != ESP_OK)
        return uartConfigRet;

    if (const esp_err_t gpioConfigRet = uart_set_pin(GPS_UART_PORT, UART_TX_GPIO_PIN, UART_RX_GPIO_PIN,
                                                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        gpioConfigRet != ESP_OK)
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

    sendStringAndWait("UNLOG\r\n", reply);


    /// check for receiver
    bool hasCorrectAnswer = false;
    while (!hasCorrectAnswer) {
        gpsUartDelay();

        sendStringAndWait("VERSION\r\n", reply);
        if (reply.find("UM980") != std::string::npos) {
            ESP_LOGI(LOGTASKTAG, "UM980 detected, VERSION reply: %s", reply.c_str());
            hasCorrectAnswer = true;
            break;
        }

        //! \todo try different UART speed (9600 ... 460800)
        ESP_LOGI(LOGTASKTAG, "UM980 not detected, VERSION reply: %s", reply.c_str());
        gpsUartDelay();
    }

    /// request config, only for debug
    sendStringAndWait("CONFIG\r\n", reply);

    // /// setup baudrate (needed if it is different and we want to switch it)
    // sendStringAndWait("CONFIG COM1 115200\r\n", reply);
    // sendStringAndWait("CONFIG COM2 115200\r\n", reply);
    // sendStringAndWait("CONFIG COM3 115200\r\n", reply);
    // sendStringAndWait("SAVECONFIG\r\n", reply);

    sendStringAndWait("MODE ROVER SURVEY DEFAULT\r\n", reply);
    sendStringAndWait("CONFIG RTK TIMEOUT 0\r\n", reply);
    /// 'AUTO' or 'E6-HAS' or 'B2b-PPP' or 'SSR-RX' or 'L6MDCPPP' ?
    sendStringAndWait("CONFIG PPP ENABLE E6-HAS\r\n", reply);
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
    sendStringAndWait("MASK 10.0\r\n", reply); /// mask elevation angle

    /// disable currently enabled messages
    sendStringAndWait("UNLOG\r\n", reply);

    /// enable necessary NMEA messages
    sendStringAndWait("GNGGA 1\r\n", reply);
    sendStringAndWait("GNGSA 1\r\n", reply);
    sendStringAndWait("GNRMC 1\r\n", reply);
    sendStringAndWait("GNGSV 1\r\n", reply);
    /// Enable Unicore specific PPP messages
    sendStringAndWait("PPPNAVA 1\r\n", reply);

    /// print ALL observations from antenna!!!
    /// TOO MANY symbols even at speed 115200,
    /// \todo test with higher speed
    //sendStringAndWait("OBSVMA 1\r\n");

    sendStringAndWait("SAVECONFIG\r\n", reply);

    /// nice to have, but it will reboot UM980 module =( and it's not
    sendStringAndWait("CONFIG SIGNALGROUP 2\r\n", reply);
    if (reply.find("system is rebooting") != std::string::npos) {
        ESP_LOGI(LOGTASKTAG, "UM980 is rebooting after SIGNALGROUP change, wait for %u ms", GPS_TASK_REBOOT_DELAY_MICROSEC);
        vTaskDelay(pdMS_TO_TICKS(GPS_TASK_REBOOT_DELAY_MICROSEC));
    }

    ESP_LOGI(LOGTASKTAG, "UM980 configuration: success");
    return ESP_OK;
}

esp_err_t GpsTask::configureTinyGps()
{
    ggaEpoch.begin(m_gps, NMEA_MSG_GXGGA.data(), 6);

    gsafixtype.begin(m_gps, NMEA_MSG_GXGSA.data(), 2);
    gsapdop.begin(m_gps, NMEA_MSG_GXGSA.data(), 15);
    gsahdop.begin(m_gps, NMEA_MSG_GXGSA.data(), 16);
    gsavdop.begin(m_gps, NMEA_MSG_GXGSA.data(), 17);

    for (int i = 0; i < GSA_SAT_FIELDS; ++i)
        gsaSat[i].begin(m_gps, NMEA_MSG_GXGSA.data(), 3 + i);

    pppnavWeek.begin(m_gps, UNICORE_MSG_PPPNAV.data(), 4);
    pppnavSecsOFWeek.begin(m_gps, UNICORE_MSG_PPPNAV.data(), 5);
    pppnavLeapSecs.begin(m_gps, UNICORE_MSG_PPPNAV.data(), 8);

    pppnavSolStatus.begin(m_gps, UNICORE_MSG_PPPNAV.data(), 9);
    pppnavPosType.begin(m_gps, UNICORE_MSG_PPPNAV.data(), 10);
    pppnavLat.begin(m_gps, UNICORE_MSG_PPPNAV.data(), 11);
    pppnavLon.begin(m_gps, UNICORE_MSG_PPPNAV.data(), 12);
    pppnavAlt.begin(m_gps, UNICORE_MSG_PPPNAV.data(), 13);
    pppnavDatumId.begin(m_gps, UNICORE_MSG_PPPNAV.data(), 15);
    pppnavLatStdDev.begin(m_gps, UNICORE_MSG_PPPNAV.data(), 16);
    pppnavLonStdDev.begin(m_gps, UNICORE_MSG_PPPNAV.data(), 17);
    pppnavAltStdDev.begin(m_gps, UNICORE_MSG_PPPNAV.data(), 18);
    pppnavStationId.begin(m_gps, UNICORE_MSG_PPPNAV.data(), 19);
    pppnavSolAge.begin(m_gps, UNICORE_MSG_PPPNAV.data(), 21);
    pppnavSatellites.begin(m_gps, UNICORE_MSG_PPPNAV.data(), 23);

    return ESP_OK;
}

int GpsTask::sendData(std::string_view data)
{
    static constexpr const char TX_TASK_TAG[] = "TX_TASK1";
    const int len = data.size();
    const int txBytes = uart_write_bytes(GPS_UART_PORT, data.data(), len);
    if (txBytes != len)
        ESP_LOGE(TX_TASK_TAG, "wrong string size");
        
    return txBytes;
}

int GpsTask::sendStringAndWait(std::string_view data, std::string &reply)
{
    static constexpr const char TX_TASK_TAG[] = "TX_TASK2";
    const int len = data.size();
    const int txBytes = uart_write_bytes(GPS_UART_PORT, data.data(), len);
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
    if (rxBytes > 0)
        newData.append(&data[0], static_cast<size_t>(rxBytes));
}

void GpsTask::configureNmeaEvent(NmeaStringReadyEvent event)
{
    m_nmeaEvent = std::move(event);
}

void GpsTask::configureGnssEvent(GnssLogReadyEvent event)
{
    m_gnssEvent = std::move(event);
}

void GpsTask::configurePppEvent(PppLogReadyEvent event)
{
    m_pppEvent = std::move(event);
}

void GpsTask::configureQStarZEvent(QStarZPacketsReadyEvent event)
{
    m_qstarzEvent = std::move(event);
}

void GpsTask::terminate()
{
    m_terminateASAP = true;
}

void GpsTask::executeTask()
{
    static constexpr const char GPS_TASK_TAG[] = "GPS_TASK";
    std::string dataAsString;
    while (!m_terminateASAP) {
        readFromUart(dataAsString);
        if (dataAsString.size() > 0) {
            ESP_LOGI(GPS_TASK_TAG, "Read %zu bytes: '%s'", dataAsString.size(), dataAsString.c_str());
            // ESP_LOG_BUFFER_HEXDUMP(GPS_TASK_TAG, data.data(), rxBytes, ESP_LOG_INFO);

            for (const auto c : dataAsString) {
                m_gps.encode(c);

                bool gsaUpdated = true;
                for (const auto &gsaX: gsaSat)
                    gsaUpdated = gsaUpdated && gsaX.isUpdated();

                if (gsaUpdated) [[unlikely]] {
                    ESP_LOGV(GPS_TASK_TAG, "new GNGSA info parsed!");
                    for (int i = 0; i < GSA_SAT_FIELDS; ++i) {
                        uint8_t prn = 0;
                        const char* sv = gsaSat[i].value();
                        std::from_chars(sv, sv + strlen(sv), prn);
                        if (prn != 0) {
                            SatelliteInfo sat{};
                            sat.prn = prn;
                            m_pendingSatellites.insert(sat);
                            ESP_LOGV(GPS_TASK_TAG, "satellite %u", prn);
                        }
                    }
                }

                const bool allNew = m_gps.location.isUpdated() 
                    && m_gps.altitude.isUpdated()
                    && pppnavSolStatus.isUpdated()
                    && ggaEpoch.isUpdated();
                if (allNew) [[unlikely]] {
                    const bool newLocation = processNewLocation();
                    if (newLocation)
                        ESP_LOGI(GPS_TASK_TAG, "new location reported");
                }
            }

            if (m_nmeaEvent)
                m_nmeaEvent(dataAsString);

            dataAsString.clear();
            
            if (m_gps.location.isValid())
                ESP_LOGI(GPS_TASK_TAG, "Valid location from GPS");
        }

        vTaskDelay(pdMS_TO_TICKS(GPS_TASK_DELAY_MS));
    }
}

bool GpsTask::hasLock(const GpsInfo &info)
{
    using Q = TinyGPSLocation::Quality;
    constexpr std::array validQuality{Q::GPS, Q::DGPS, Q::PPS, Q::RTK, Q::FloatRTK};
    // fixType: 0=no data, 2=2D fix, 3=3D fix (1=no fix excluded)
    constexpr std::array<uint8_t, 3> validFixType{0, 2, 3};
    return std::ranges::contains(validQuality, info.quality)
        && std::ranges::contains(validFixType, info.fixType);
}

bool GpsTask::has3DLock(const GpsInfo &info)
{
    using Q = TinyGPSLocation::Quality;
    constexpr std::array validQuality{Q::GPS, Q::DGPS, Q::PPS, Q::RTK, Q::FloatRTK};
    return std::ranges::contains(validQuality, info.quality) && info.fixType == 3;
}

bool GpsTask::processNewLocation()
{
    static constexpr const char NEW_LOCATION_TAG[] = "read-gps-location";
    ESP_LOGD(NEW_LOCATION_TAG, "start reading GPS location");

    ///first check if everything is updated
    const bool allNew = m_gps.location.isUpdated() 
        && m_gps.altitude.isUpdated()
        && pppnavSolStatus.isUpdated()
        && ggaEpoch.isUpdated();

    if (!allNew) {
        ESP_LOGD(NEW_LOCATION_TAG, "not all NMEA/Unicore messages accumulated, wait for them");
        return false;
    }

    ESP_LOGI(NEW_LOCATION_TAG, "location and altitude are updated");

    GpsInfo gpsInfo{};
    /// move now to clear m_pendingSatellites buffer even if location is bad and we 'return false;'
    ESP_LOGI(NEW_LOCATION_TAG, "number of accumulated satellites %zu", m_pendingSatellites.size());
    gpsInfo.satellites = std::move(m_pendingSatellites);

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

    const char* fv = gsafixtype.value();
    std::from_chars(fv, fv + strlen(fv), gpsInfo.fixType); // leaves fixType == 0 if no data
    gpsInfo.quality = m_gps.location.FixQuality();
    gpsInfo.lat = m_gps.location.lat();
    gpsInfo.lon = m_gps.location.lng();
    gpsInfo.altitude = m_gps.altitude.meters();
    gpsInfo.geoidAlt = m_gps.geoidHeight.meters();

    if (!hasLock(gpsInfo))
        return false;
    
    if (has3DLock(gpsInfo))
        ESP_LOGI(NEW_LOCATION_TAG, "GPS has 3D lock");
    else
        ESP_LOGI(NEW_LOCATION_TAG, "GPS has 2D lock");

    // We know the solution is fresh and valid, so just read the data
    {
        // positional timestamp
        struct tm t = {
            .tm_sec  = static_cast<int>(m_gps.time.second()),
            .tm_min  = static_cast<int>(m_gps.time.minute()),
            .tm_hour = static_cast<int>(m_gps.time.hour()),
            .tm_mday = static_cast<int>(m_gps.date.day()),
            .tm_mon  = static_cast<int>(m_gps.date.month()) - 1,
            .tm_year = static_cast<int>(m_gps.date.year()) - 1900,
            .tm_wday = 0,
            .tm_yday = 0,
            .tm_isdst = 0,
        };
        const auto timestamp = std::mktime(&t);
        gpsInfo.worldTime = std::chrono::system_clock::from_time_t(timestamp);

        const int32_t microsec = static_cast<int32_t>(m_gps.time.centisecond()) * 10000;
        struct timeval gpsTimeNow = { .tv_sec = timestamp, .tv_usec = microsec };
        //! \todo read TimeZone from SDCard or internal memory
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
        auto fromField = [](const char* s, auto& val) static {
            if (s && *s) std::from_chars(s, s + strlen(s), val);
        };
        fromField(pppnavAlt.value(),        pppInfo.alt);
        fromField(pppnavLatStdDev.value(),  pppInfo.latStdDev);
        fromField(pppnavLonStdDev.value(),  pppInfo.lonStdDev);
        fromField(pppnavAltStdDev.value(),  pppInfo.altStdDev);
        fromField(pppnavSatellites.value(), pppInfo.satellites);
        fromField(pppnavSolAge.value(),     pppInfo.solutionAge);
        pppInfo.solutionStatus = parseSolutionStatus(pppnavSolStatus.value(), pppInfo.outputDelayMs);
        pppInfo.positionType = parsePositionType(pppnavPosType.value());
        pppInfo.datumId = parseDatumId(pppnavDatumId.value());
        pppInfo.stationId = parseStationId(pppnavStationId.value());
        pppInfo.serviceId = parsePppService(pppInfo.stationId);

        uint32_t week{}, millisOfWeek{}, leapSecs{};
        fromField(pppnavWeek.value(),       week);
        fromField(pppnavSecsOFWeek.value(), millisOfWeek);
        fromField(pppnavLeapSecs.value(),   leapSecs);
        pppInfo.utxSeconds = computeUtxTime(week, millisOfWeek, leapSecs, pppInfo.millisecs);

        ESP_LOGI(NEW_LOCATION_TAG, "PPP info parsed");
    }

    {
        const std::string gpsLog = printGpsTimeInfo(gpsInfo) + printGpsGeoInfo(gpsInfo);
        if (m_gnssEvent)
            m_gnssEvent(gpsLog);
    }

    {
        const auto emulatedQstarz = emulateQstarzBinary(gpsInfo);
        if (m_qstarzEvent)
            m_qstarzEvent(emulatedQstarz);
    }

    {
        const double latPpp = static_cast<double>(pppInfo.lat) * 1e-7;
        const double lonPpp = static_cast<double>(pppInfo.lon) * 1e-7;
        const double latGnss = gpsInfo.lat; //static_cast<double>(localPosition.latitude_i) * 1e-7;
        const double lonGnss = gpsInfo.lon; //static_cast<double>(localPosition.longitude_i) * 1e-7;
        const double gnssToPppDistance = geoDistance(latPpp, lonPpp, latGnss, lonGnss);
        
        const std::string pppLog = printPppTimeInfo(pppInfo) + printPppGeoInfo(pppInfo, gnssToPppDistance);
        if (m_pppEvent)
            m_pppEvent(pppLog);
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

void GpsTask::dopToMeters(std::string& out, const uint32_t dop) noexcept
{
    const auto [quot, rem] = std::div(dop, 100);
    std::format_to(std::back_inserter(out), "{}.{:02}", quot, rem);
}

double GpsTask::geoDistance(const double &lat1, const double &lon1, const double &lat2, const double &lon2)
{
    constexpr double R          = 6371000.0;
    constexpr double DEG_TO_RAD = std::numbers::pi / 180.0;

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
    std::string message;
    message.reserve(160);
    message += "LAT;";
    appendNum(message, p.lat);
    message += ";LON;";
    appendNum(message, p.lon);
    /// over ellipsoid (WGS-84)
    message += ";ALT;";
    appendNum(message, p.altitude);
    /// altitude over geoid, but which one???
    message += ";ALTHAE;";
    appendNum(message, p.altitude);
    /// geoid undulation (separation), use 'Gravity' tool from 'geographiclib' library
    /// $ /usr/local/bin/Gravity -n egm96 --input-string "27.988 86.925" -H
    /// and result sould be ~ "-28.7422" in meters
    message += ";UNDUL;";
    appendNum(message, p.geoidAlt);
    message += ";SATS;";
    appendNum(message, p.satellites.size());
    message += ";PDOP;";
    dopToMeters(message, p.gsaPDOP);
    message += ";HDOP;";
    dopToMeters(message, p.gsaHDOP);
    message += ";VDOP;";
    dopToMeters(message, p.gsaVDOP);
    message += ";";
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
    gmtime_r(&stampT, &gmTime);
    
    constexpr int GMTIME_YEAR_FIX = 1900;
    constexpr int GMTIME_MONTH_FIX = 1;
    gmTime.tm_year += GMTIME_YEAR_FIX;
    gmTime.tm_mon += GMTIME_MONTH_FIX;

    // const double lat = static_cast<double>(p.location.lat()) * 1e-7;
    // const double lon = static_cast<double>(p.location.lng()) * 1e-7;

    std::string message;
    message.reserve(80);
    message += "DT;";
    std::format_to(std::back_inserter(message), "{:04d}-{:02d}-{:02d}T{:02d}:{:02d}:{:02d}Z",
                   gmTime.tm_year, gmTime.tm_mon, gmTime.tm_mday,
                   gmTime.tm_hour, gmTime.tm_min, gmTime.tm_sec);
    message += ";GNSSSEC;";
    appendNum(message, gpsSecs);
    message += ';';

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

    std::string dateTimeStringFull;
    dateTimeStringFull.reserve(28);
    if (p.millisecs > 0)
        std::format_to(std::back_inserter(dateTimeStringFull),
                       "{:04d}-{:02d}-{:02d}T{:02d}:{:02d}:{:02d}.{:03d}Z",
                       gmTime.tm_year, gmTime.tm_mon, gmTime.tm_mday,
                       gmTime.tm_hour, gmTime.tm_min, gmTime.tm_sec, p.millisecs);
    else
        std::format_to(std::back_inserter(dateTimeStringFull),
                       "{:04d}-{:02d}-{:02d}T{:02d}:{:02d}:{:02d}Z",
                       gmTime.tm_year, gmTime.tm_mon, gmTime.tm_mday,
                       gmTime.tm_hour, gmTime.tm_min, gmTime.tm_sec);

    ESP_LOGI(LOGTASKTAG, "date from PPP: %d-%d-%dT%d:%d:%d.%dZ",
        gmTime.tm_year, gmTime.tm_mon, gmTime.tm_mday,
        gmTime.tm_hour, gmTime.tm_min, gmTime.tm_sec,
        p.millisecs
    );
    ESP_LOGI(LOGTASKTAG, "date formatted by code: %s", dateTimeStringFull.c_str());

    std::string message;
    message.reserve(200);
    message += "PPP_SOLUTION_STATUS;";
    message += solutionStatusStr(p.solutionStatus);
    message += ";PPP_POSITION;";
    message += positionTypeStr(p.positionType);
    message += ";PPP_SERVICE;";
    message += serviceIdStr(p.serviceId);
    message += ";PPP_DATUM;";
    message += datumIdStr(p.datumId);
    message += ";PPP_DT;";
    message += dateTimeStringFull;
    message += ";PPP_TIME;";
    appendNum(message, p.utxSeconds);
    message += ";PPP_AGE;";
    appendNum(message, p.solutionAge);

    ESP_LOGI(LOGTASKTAG, "SdLoggerModule | generate PPP Time info - end");
    return message;
}

std::string GpsTask::printPppGeoInfo(const PppInfo &p, const double &gnssToPppDistance)
{
    ESP_LOGI(LOGTASKTAG, "SdLoggerModule | generate PPP GEO info - start");


    const double latPpp = static_cast<double>(p.lat) * 1e-7;
    const double lonPpp = static_cast<double>(p.lon) * 1e-7;

    std::string message;
    message.reserve(200);
    message += ";PPP_LAT;";
    appendNum(message, latPpp);
    message += ";PPP_LON;";
    appendNum(message, lonPpp);
    message += ";PPP_GNSS_OFFSET;";
    appendNum(message, gnssToPppDistance);
    /// over ellipsoid (WGS-84)
    message += ";PPP_ALT;";
    appendNum(message, p.alt);
    /// altitude over geoid, but which one???
    // message += ";ALTHAE;";
    // appendNum(message, p.altitude_hae);
    /// geoid undulation (separation), use 'Gravity' tool from 'geographiclib' library
    /// $ /usr/local/bin/Gravity -n egm96 --input-string "27.988 86.925" -H
    /// and result sould be ~ "-28.7422" in meters
    // message += ";UNDUL;";
    // appendNum(message, p.altitude_geoidal_separation);
    message += ";PPP_SATS;";
    appendNum(message, p.satellites);
    message += ";PPP_STATION_ID;";
    appendNum(message, p.stationId);
    message += ";PPP_LATSTDDEV;";
    appendNum(message, p.latStdDev);
    message += ";PPP_LONSTDDEV;";
    appendNum(message, p.lonStdDev);
    message += ";PPP_ALTSTDDEV;";
    appendNum(message, p.altStdDev);
    message += ";";

    ESP_LOGI(LOGTASKTAG, "SdLoggerModule | generate PPP GEO info - end");
    return message;
}

template<typename T>
    requires std::is_trivially_copyable_v<T>
void appendRaw(std::vector<std::byte> &buf, const T &data)
{
    const auto bytes = std::as_bytes(std::span{&data, 1});
    buf.insert(buf.end(), bytes.begin(), bytes.end());
}

// Binary record format: Qstarz BL-1000GT, 64 bytes, little-endian
GpsTask::QStarZPackets GpsTask::emulateQstarzBinary(const GpsInfo &p)
{
    const auto epoch = p.worldTime.time_since_epoch();
    const auto secs = std::chrono::duration_cast<std::chrono::seconds>(epoch);
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(epoch) - secs;

    // mode: 1=fix not available, 2=2D, 3=3D (maps directly from fixType; 0->1)
    const uint8_t mode = (p.fixType >= 2 && p.fixType <= 3) ? p.fixType : 1;
    const uint8_t rcr = 'T';
    const uint16_t time_ms = static_cast<uint16_t>(ms.count());

    // Convert decimal degrees to DDDMM.MMMM
    auto toQstarzDeg = [](double decimal) static -> double {
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
    const uint8_t numSatUse = static_cast<uint8_t>(
        std::min(p.satellites.size(), static_cast<size_t>(255)));
    const uint8_t numSatView = numSatUse;   // GSA = used sats; view requires GSV (future)
    const uint8_t fixQual = static_cast<uint8_t>(p.quality);
    const uint8_t batPerc = 0;
    const uint16_t dummy = 0;
    const uint8_t series_number = 0; /// Bluetooth GNSS expects '0'

    std::vector<std::byte> buf;
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
    QStarZPackets packets;

    const std::span<const std::byte> view{buf};
    packets[0].assign_range(view.subspan( 0, 20));
    packets[1].assign_range(view.subspan(20, 20));
    packets[2].assign_range(view.subspan(40, 20));
    packets[3].assign_range(view.subspan(60,  4));

    ESP_LOGD(LOGTASKTAG, "first packet.length: %zu", packets[0].size());
    ESP_LOGD(LOGTASKTAG, "first packet[0]: %d, should be 1, 2, 3", std::to_integer<int>(packets[0][0]));
    ESP_LOGD(LOGTASKTAG, "third packet.length: %zu", packets[2].size());
    ESP_LOGD(LOGTASKTAG, "third packet[16]: %d", std::to_integer<int>(packets[2][16]));
    ESP_LOGD(LOGTASKTAG, "third packet[17]: %d", std::to_integer<int>(packets[2][17]));
    ESP_LOGD(LOGTASKTAG, "third packet[18]: %d", std::to_integer<int>(packets[2][18]));
    ESP_LOGD(LOGTASKTAG, "third packet[19]: %d", std::to_integer<int>(packets[2][19]));
    


    return packets;
}
