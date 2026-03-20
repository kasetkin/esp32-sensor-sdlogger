#include "gpstask.h"

#include <ctime>
#include <chrono>
#include <array>
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "common_utils.h"
#include "unicore.h"


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
    /// check for receiver
    sendStringAndWait("VERSION\r\n");
    //! \todo check if answer contains UM980

    /// request config, only for debug
    sendStringAndWait("CONFIG\r\n");

    // /// setup baudrate (needed if it is different and we want to switch it)
    // sendStringAndWait("CONFIG COM1 115200\r\n");
    // sendStringAndWait("CONFIG COM2 115200\r\n");
    // sendStringAndWait("CONFIG COM3 115200\r\n");
    // sendStringAndWait("SAVECONFIG\r\n");

    sendStringAndWait("CONFIG SIGNALGROUP 2\r\n");
    sendStringAndWait("MODE ROVER SURVEY DEFAULT\r\n");
    sendStringAndWait("CONFIG RTK TIMEOUT 0\r\n");
    /// 'AUTO' or 'E6-HAS' or 'B2b-PPP' or 'SSR-RX' or 'L6MDCPPP' ?
    sendStringAndWait("CONFIG PPP ENABLE AUTO\r\n");
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
    sendStringAndWait("CONFIG NMEA0183 V411\r\n");
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
    sendStringAndWait("MASK 0.0\r\n"); /// mask elevation angle

    /// configure NMEA messages
    sendStringAndWait("GPGGA 1\r\n");
    sendStringAndWait("GPGSA 1\r\n");
    sendStringAndWait("GPRMC 1\r\n");
    /// Enable Unicore specific PPP messages
    sendStringAndWait("PPPNAVA 1\r\n");

    /// print ALL observations from antenna!!!
    /// TOO MANY symbols even at speed 115200,
    /// \todo test with higher speed
    //sendStringAndWait("OBSVMA 1\r\n");

    sendStringAndWait("SAVECONFIG\r\n");
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


int GpsTask::sendData(const char* data)
{
    static const char *TX_TASK_TAG = "TX_TASK1";
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(GPS_UART_PORT, data, len);
    if (txBytes != len)
        ESP_LOGE(TX_TASK_TAG, "wrong string size");
        
    return txBytes;
}

int GpsTask::sendStringAndWait(const std::string &str)
{
    static const char *TX_TASK_TAG = "TX_TASK2";
    const int len = str.size();
    const int txBytes = uart_write_bytes(GPS_UART_PORT, str.c_str(), len);
    if (txBytes != len)
        ESP_LOGE(TX_TASK_TAG, "wrong string size");

    gpsUartDelay();
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

void GpsTask::executeTask()
{
    static const char *RX_TASK_TAG = "RX_TASK";
    const uint32_t readTimeoutInTicks = pdMS_TO_TICKS(1);
    std::array<char, RX_BUF_SIZE + 1> data;
    while (true) {
        const int rxBytes = uart_read_bytes(GPS_UART_PORT, data.data(), RX_BUF_SIZE, readTimeoutInTicks);
        if (rxBytes > 0) {
            data[rxBytes] = 0;
            ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
            // ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data.data(), rxBytes, ESP_LOG_INFO);

            for (size_t i = 0; i < rxBytes; ++i)
                m_gps.encode(data[i]);

            if (m_gps.location.isValid())
                ESP_LOGI(RX_TASK_TAG, "Valid location from GPS");
        }

        const bool newLocation = hasNewLocation();
        vTaskDelay(pdMS_TO_TICKS(GPS_TASK_DELAY_MS));
    }
}

bool GpsTask::hasLock()
{
    // Using GPGGA fix quality indicator
    const TinyGPSLocation::Quality fixQuality = m_gps.location.FixQuality();
    if (fixQuality == TinyGPSLocation::Quality::GPS
        || fixQuality == TinyGPSLocation::Quality::DGPS
        || fixQuality == TinyGPSLocation::Quality::PPS
        || fixQuality == TinyGPSLocation::Quality::RTK
        || fixQuality == TinyGPSLocation::Quality::FloatRTK) {
        // Use GPGSA fix type 2D/3D (better) if available
        // 0 -- no data,
        // 1 -- Fix not available
        // 2 -- 2D fix, good or not enough ???
        // 3 -- 3D fix
        if (fixType == 3 || fixType == 2 || fixType == 0)
            return true;
    }

    return false;
}

bool GpsTask::hasNewLocation()
{
    fixType = atoi(gsafixtype.value()); // will set to zero if no data
    if (!hasLock())
        return false;
    
    if (!m_gps.location.isUpdated() && !m_gps.altitude.isUpdated())
        return false;

    /// maybe log something like 'no coords at time XXX'
    if (!m_gps.location.isValid())
        return false;

    const uint32_t GPS_SOLUTION_MAX_AGE_MS = 10 * 1000;
    if ((m_gps.location.age() > GPS_SOLUTION_MAX_AGE_MS)
          || (gsafixtype.age() > GPS_SOLUTION_MAX_AGE_MS)
          || (m_gps.time.age() > GPS_SOLUTION_MAX_AGE_MS) 
          || (m_gps.date.age() > GPS_SOLUTION_MAX_AGE_MS)) {
        // LOG_WARN("SOME data is TOO OLD: LOC %u, TIME %u, DATE %u", reader.location.age(), reader.time.age(), reader.date.age());
        return false;
    }

    // We know the solution is fresh and valid, so just read the data
    auto &loc = m_gps.location;

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
    const auto point = std::chrono::high_resolution_clock::from_time_t(timestamp);

    const int32_t PDOP = TinyGPSPlus::parseDecimal(gsapdop.value());
    const int32_t HDOP = TinyGPSPlus::parseDecimal(gsahdop.value());
    const int32_t VDOP = TinyGPSPlus::parseDecimal(gsavdop.value());

    localPPP = PppInfo{};
    localPPP.lat = parseDegreesLatLon(pppnavLat.value());
    localPPP.lon = parseDegreesLatLon(pppnavLon.value());
    localPPP.alt = static_cast<int32_t>(atol(pppnavAlt.value()));
    localPPP.latStdDev = static_cast<float>(atof(pppnavLatStdDev.value()));
    localPPP.lonStdDev = static_cast<float>(atof(pppnavLonStdDev.value()));
    localPPP.altStdDev = static_cast<float>(atof(pppnavAltStdDev.value()));
    localPPP.satellites = static_cast<int32_t>(atol(pppnavSatellites.value()));
    localPPP.solutionAge = static_cast<int32_t>(atol(pppnavSolAge.value()));
    localPPP.solutionStatus = parseSolutionStatus(pppnavSolStatus.value());
    localPPP.positionType = parsePositionType(pppnavPosType.value());
    localPPP.datumId = parseDatumId(pppnavDatumId.value());
    localPPP.stationId = parseStationId(pppnavStationId.value());
    localPPP.serviceId = parsePppService(localPPP.stationId);

    uint32_t week = static_cast<int32_t>(atol(pppnavWeek.value()));
    uint32_t millisOfWeek = static_cast<int32_t>(atoll(pppnavSecsOFWeek.value()));
    uint32_t leapSecs = static_cast<uint32_t>(atol(pppnavLeapSecs.value()));
    localPPP.utxSeconds = computeUtxTime(week, millisOfWeek, leapSecs, localPPP.millisecs);

    return true;
}

void GpsTask::logNmeaMessageToSd(const std::string &msg)
{
//     static const char * logsPath = "/logs";
// #ifdef GPS_DEBUG
//     LOG_DEBUG("GPS->SdLoggerModule | message generation - start");
// #endif
//     sdLoggerModule->createSDDir(logsPath);
//
//     const std::string filename = sdLoggerModule->generateFilename() + "_nmea.csv";
//     const std::string fullLogMessage = msg + std::string("\r\n");
//
//     const std::string fullpath = std::string(logsPath) + "/" + filename;
//     sdLoggerModule->appendSDFile(fullpath.c_str(), fullLogMessage.c_str());
}