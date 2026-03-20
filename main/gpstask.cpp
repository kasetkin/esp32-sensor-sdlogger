#include "gpstask.h"

#include <array>
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "common_utils.h"



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

        vTaskDelay(pdMS_TO_TICKS(GPS_TASK_DELAY_MS));
    }
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