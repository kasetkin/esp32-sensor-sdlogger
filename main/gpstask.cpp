#include "gpstask.h"

#include <charconv>
#include <format>
#include <cmath>
#include <numbers>
#include <ctime>
#include <chrono>
#include <algorithm>
#include <utility>
#include <array>
#include <ranges>
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
    ESP_LOGI(LOGTASKTAG, "UM980 configuration: start");

    sendStringAndWait("UNLOG\r\n");

    /// check for receiver
    bool hasCorrectAnswer = false;
    while (!hasCorrectAnswer) {
        gpsUartDelay();

        const auto versionResult = sendStringAndWait("VERSION\r\n");
        if (versionResult && versionResult->contains("UM980")) {
            ESP_LOGI(LOGTASKTAG, "UM980 detected, VERSION reply: %s", versionResult->c_str());
            hasCorrectAnswer = true;
            break;
        }

        //! \todo try different UART speed (9600 ... 460800)
        ESP_LOGI(LOGTASKTAG, "UM980 not detected, VERSION reply: %s",
                 versionResult ? versionResult->c_str() : "(UART error)");
        gpsUartDelay();
    }

    /// request config, only for debug
    sendStringAndWait("CONFIG\r\n");

    // /// setup baudrate (needed if it is different and we want to switch it)
    // sendStringAndWait("CONFIG COM1 115200\r\n");
    // sendStringAndWait("CONFIG COM2 115200\r\n");
    // sendStringAndWait("CONFIG COM3 115200\r\n");
    // sendStringAndWait("SAVECONFIG\r\n");

    sendStringAndWait("MODE ROVER SURVEY DEFAULT\r\n");
    sendStringAndWait("CONFIG RTK TIMEOUT 0\r\n");
    /// 'AUTO' or 'E6-HAS' or 'B2b-PPP' or 'SSR-RX' or 'L6MDCPPP' ?
    sendStringAndWait("CONFIG PPP ENABLE E6-HAS\r\n");
    /// we don't need default 15cm precision, 70cm in horizontal and 100cm in vertical should be enough
    sendStringAndWait("CONFIG PPP CONVERGE 75 200\r\n");
    sendStringAndWait("CONFIG PPP DATUM WGS84\r\n");
    sendStringAndWait("CONFIG DGPS TIMEOUT 0\r\n");
    sendStringAndWait("CONFIG MMP ENABLE\r\n");
    sendStringAndWait("CONFIG PVTALG MULTI\r\n");
    sendStringAndWait("CONFIG IONMODE GPSK8\r\n");
    sendStringAndWait("CONFIG ANTIJAM FORCE\r\n");
    sendStringAndWait("CONFIG PSRVELDRPOS DISABLE\r\n");
    sendStringAndWait("CONFIG UNDULATION AUTO\r\n");
    sendStringAndWait("CONFIG NMEA0183 V410\r\n"); /// IMPORTANT!!! if use "V411" => GSV message, term 17 becomes incorrect 
    sendStringAndWait("CONFIG SBAS ENABLE AUTO\r\n"); /// why not, if there is any in your region
    sendStringAndWait("CONFIG SBAS TIMEOUT 1800\r\n");
    sendStringAndWait("CONFIG STANDALONE ENABLE\r\n"); /// not sure if it's a good idea

    sendStringAndWait("UNMASK ALL\r\n"); /// enable all GNSS systems (GPS, Galileo, Beidou, Glonass, QZSS, IRNSS)
    sendStringAndWait("UNMASK GPS\r\n"); /// USA
    sendStringAndWait("UNMASK BDS\r\n"); /// Beidou, China
    sendStringAndWait("UNMASK GLO\r\n"); /// GLONASS, Russia
    sendStringAndWait("UNMASK GAL\r\n"); /// Galileo, Europe
    sendStringAndWait("UNMASK QZSS\r\n"); /// Quasi-Zenith Satellite System, Japanese
    sendStringAndWait("UNMASK IRNSS\r\n"); /// NavIC, Indian
    sendStringAndWait("MASK 10.0\r\n"); /// mask elevation angle

    /// disable currently enabled messages
    sendStringAndWait("UNLOG\r\n");

    /// enable necessary NMEA messages
    sendStringAndWait("GNGGA 1\r\n");
    sendStringAndWait("GNGSA 1\r\n");
    sendStringAndWait("GNRMC 1\r\n");
    sendStringAndWait("GNGSV 1\r\n");
    /// Enable Unicore specific PPP messages
    sendStringAndWait("PPPNAVA 1\r\n");

    /// print ALL observations from antenna!!!
    /// TOO MANY symbols even at speed 115200,
    /// \todo test with higher speed
    //sendStringAndWait("OBSVMA 1\r\n");

    sendStringAndWait("SAVECONFIG\r\n");

    /// nice to have, but it will reboot UM980 module =( and it's not
    const auto signalResult = sendStringAndWait("CONFIG SIGNALGROUP 2\r\n");
    if (signalResult && signalResult->contains("system is rebooting")) {
        ESP_LOGI(LOGTASKTAG, "UM980 is rebooting after SIGNALGROUP change, wait for %u ms", GPS_TASK_REBOOT_DELAY_MICROSEC);
        vTaskDelay(pdMS_TO_TICKS(GPS_TASK_REBOOT_DELAY_MICROSEC));
    }

    ESP_LOGI(LOGTASKTAG, "UM980 configuration: success");
    return ESP_OK;
}

esp_err_t GpsTask::configureTinyGps()
{
    ggaEpoch.begin(m_gps, NMEA_MSG_GXGGA.data(), 6);

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

std::expected<std::string, esp_err_t> GpsTask::sendStringAndWait(std::string_view data)
{
    static constexpr const char TX_TASK_TAG[] = "TX_TASK2";
    const int txBytes = uart_write_bytes(GPS_UART_PORT, data.data(), data.size());
    if (txBytes != static_cast<int>(data.size())) {
        ESP_LOGE(TX_TASK_TAG, "wrong string size");
        return std::unexpected(ESP_FAIL);
    }

    gpsUartDelay();

    std::string reply;
    readFromUart(reply);
    if (!reply.empty())
        ESP_LOGI(TX_TASK_TAG, "reply: %s", reply.c_str());

    return reply;
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

                const bool allNew = m_gps.location.isUpdated()
                    && m_gps.altitude.isUpdated()
                    && m_gps.satellites.isUpdated()
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
            
            if (m_gps.location.age() != ULONG_MAX)
                ESP_LOGI(GPS_TASK_TAG, "Valid location from GPS");
        }

        vTaskDelay(pdMS_TO_TICKS(GPS_TASK_DELAY_MS));
    }
}

using Q = TinyGPSLocation::GnssQuality;
static constexpr std::array VALID_GPS_QUALITY{Q::GPS, Q::DGPS, Q::PPS, Q::RTK, Q::FloatRTK};
static constexpr std::array<uint8_t, 3> VALID_FIX_TYPES{0, 2, 3};

bool GpsTask::hasLock(const GpsInfo &info)
{
    return std::ranges::contains(VALID_GPS_QUALITY, info.quality) ;
        //&& std::ranges::contains(VALID_FIX_TYPES, info.positioningMode);
}

bool GpsTask::has3DLock(const GpsInfo &info)
{
    return std::ranges::contains(VALID_GPS_QUALITY, info.quality) ; //&& info.positioningMode == TinyGPSLocation::PositioningMode::;
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

    const uint32_t GPS_SOLUTION_MAX_AGE_MS = 10 * 1000;
    if ((m_gps.location.age() > GPS_SOLUTION_MAX_AGE_MS)
          || (m_gps.satellites.age() > GPS_SOLUTION_MAX_AGE_MS)
          || (m_gps.time.age() > GPS_SOLUTION_MAX_AGE_MS)
          || (m_gps.date.age() > GPS_SOLUTION_MAX_AGE_MS)) {
        ESP_LOGD(NEW_LOCATION_TAG, "some GPS data is TOO OLD: location, GSA fix, time/date");
        return false;
    }

    const auto locData = m_gps.location.consume();
    const auto satsData = m_gps.satellites.consume();
    const auto altData = m_gps.altitude.consume();
    const auto geoData = m_gps.geoidHeight.consume();
    const auto timeData = m_gps.time.consume();
    const auto dateData = m_gps.date.consume();
    const auto fixHDOP = m_gps.hdop.consume();
    const auto fixVDOP = m_gps.vdop.consume();
    const auto fixPDOP = m_gps.pdop.consume();
    const auto usedCount = m_gps.satellitesUsedCount.consume();

    if (!locData || !satsData || !altData || !geoData || !timeData || !dateData) {
        ESP_LOGD(NEW_LOCATION_TAG, "GPS location is invalid, no new GPS logs");
        return false;
    }

    ESP_LOGI(NEW_LOCATION_TAG, "location, GSA-fix, time, date are fresh and valid");

    // NMEA 0183 4.10: GGA term 7 should equal the GSA-union count under normal
    // operation. Receivers vary (some cap at 12, some report the multi-GNSS
    // sum) so the library does not enforce — log a warning so anomalies
    // surface during integration.
    if (usedCount && *usedCount != satsData->inSolutionCount())
        ESP_LOGW(NEW_LOCATION_TAG,
            "GGA reports %lu satellites in use but GSA union has %zu",
            static_cast<unsigned long>(*usedCount),
            satsData->inSolutionCount());

    GpsInfo gpsInfo{};
    gpsInfo.fix2D3DType  = locData->fix2D3DType;
    gpsInfo.fix2D3DMode  = locData->fix2D3DMode;
    gpsInfo.positioningMode  = locData->fixMode;
    gpsInfo.quality  = locData->fixQuality;
    gpsInfo.lat      = locData->latDeg();
    gpsInfo.lon      = locData->lngDeg();
    gpsInfo.altitude = altData->meters();
    gpsInfo.geoidAlt = geoData->meters();
    gpsInfo.fixHDOP = fixHDOP->dop();
    gpsInfo.fixVDOP = fixVDOP->dop();
    gpsInfo.fixPDOP = fixPDOP->dop();

    const auto allSats = satsData->all();
    for (std::size_t i = 0; i < allSats.size(); ++i)
    {
        if (!satsData->inSolution(i))
            continue;

        const auto& sat = allSats[i];
        gpsInfo.satellites.insert({
            .prn       = sat.prn,
            .systemId  = sat.systemId,
            .elevation = sat.elevationDeg,   // -1 when not reported (from GSV)
            .azimuth   = sat.azimuthDeg,
            .cn0       = sat.cn0DbHz,
        });
    }
    gpsInfo.satellitesInView = satsData->inViewCount();
    ESP_LOGI(NEW_LOCATION_TAG, "satellites: %zu in solution, %zu in view",
             gpsInfo.satellites.size(), gpsInfo.satellitesInView);

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
            .tm_sec  = static_cast<int>(timeData->second()),
            .tm_min  = static_cast<int>(timeData->minute()),
            .tm_hour = static_cast<int>(timeData->hour()),
            .tm_mday = static_cast<int>(dateData->day()),
            .tm_mon  = static_cast<int>(dateData->month()) - 1,
            .tm_year = static_cast<int>(dateData->year()) - 1900,
            .tm_wday = 0,
            .tm_yday = 0,
            .tm_isdst = 0,
        };
        const auto timestamp = std::mktime(&t);
        gpsInfo.worldTime = std::chrono::system_clock::from_time_t(timestamp);

        const int32_t microsec = static_cast<int32_t>(timeData->centisecond()) * 10000;
        struct timeval gpsTimeNow = { .tv_sec = timestamp, .tv_usec = microsec };
        //! \todo read TimeZone from SDCard or internal memory
        struct timezone myZone = { .tz_minuteswest = -300, .tz_dsttime = DST_NONE };
        settimeofday(&gpsTimeNow, &myZone);

        ESP_LOGI(NEW_LOCATION_TAG, "timestamp generated");
    }

    PppInfo pppInfo{};
    {
        pppInfo.lat = parseDegreesLatLon(pppnavLat.value());
        pppInfo.lon = parseDegreesLatLon(pppnavLon.value());
        auto fromField = [](std::string_view s, auto& val) static {
            if (!s.empty()) std::from_chars(s.data(), s.data() + s.size(), val);
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
        const double latGnss = gpsInfo.lat.value_or(std::numeric_limits<double>::quiet_NaN());
        const double lonGnss = gpsInfo.lon.value_or(std::numeric_limits<double>::quiet_NaN());
        const double gnssToPppDistance = geoDistance(latPpp, lonPpp, latGnss, lonGnss);
        
        const std::string pppLog = printPppTimeInfo(pppInfo) + printPppGeoInfo(pppInfo, gnssToPppDistance);
        if (m_pppEvent)
            m_pppEvent(pppLog);
    }
    
    return true;
}

void GpsTask::logNmeaMessageToSd(std::string_view msg)
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
    dopToMeters(message, p.fixPDOP.value_or(-1.0));
    message += ";HDOP;";
    dopToMeters(message, p.fixHDOP.value_or(-1.0));
    message += ";VDOP;";
    dopToMeters(message, p.fixVDOP.value_or(-1.0));
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
    if (!p.lat || !p.lat || !p.lon)
        return {};

    const auto epoch = p.worldTime.time_since_epoch();
    const auto secs = std::chrono::duration_cast<std::chrono::seconds>(epoch);
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(epoch) - secs;

    const auto fix2d3dModeVal = p.fix2D3DType.value_or(TinyGPSLocation::Fix2D3DType::None);
    const uint8_t mode = (fix2d3dModeVal == TinyGPSLocation::Fix2D3DType::Fix2D || fix2d3dModeVal == TinyGPSLocation::Fix2D3DType::Fix3D) ? static_cast<uint8_t>(fix2d3dModeVal) : 1;
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
    const double dLat = toQstarzDeg(p.lat.value());
    const double dLon = toQstarzDeg(p.lon.value());

    const uint32_t time_s = static_cast<uint32_t>(secs.count());
    const float speed_kmph = 0.0f;
    const float height_m = static_cast<float>(p.altitude.value());
    const float heading = 0.0f;
    const int16_t Gx = 0;
    const int16_t Gy = 0;
    const int16_t Gz = 0;
    const uint16_t maxSNR = 0;
    const float dop = static_cast<float>(p.fixHDOP.value_or(-1.0)) / 100.0f;
    const float vdop = static_cast<float>(p.fixVDOP.value_or(-1.0)) / 100.0f;
    const uint8_t numSatUse = static_cast<uint8_t>(
        std::min(p.satellites.size(), static_cast<size_t>(255)));
    const uint8_t numSatView = static_cast<uint8_t>(
        std::min(p.satellitesInView, static_cast<size_t>(255)));   // in-view count from GNGSV
    const uint8_t fixQual = static_cast<uint8_t>(std::to_underlying(p.quality.value_or(TinyGPSLocation::GnssQuality::Invalid)));
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
    appendRaw(buf, dop); //  8
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
